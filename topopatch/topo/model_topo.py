"""TopoRulePatch model: topology-aware verified path repair (torch).

Architecture (<=5M params; actual ~2.6M):
  - shared conv encoder over 6ch BEV (occ/orig/rule/goal + coords), same
    shapes as frozen M2 v2 (fresh weights, own code — M2 untouched);
  - rule-feature MLP over padded per-rule features (3,6);
  - topology head: 3-class logits (SIDE_NEG / SIDE_POS / AMBIGUOUS);
  - M=4 spline heads tagged to modes: heads {0,1} -> NEG, {2,3} -> POS.
    Ambiguous samples train all heads (uniform mode target) and WTA over
    all heads; otherwise WTA is restricted to the ground-truth mode's
    heads. Mode-conditional WTA IS the diversity mechanism: heads of
    different modes never compete for the same sample.

Losses (all supervised/geometric; no simulator futures, no RL/LLM):
  dense-trajectory WTA (mode-conditional) + ctrl anchor term + global
  confidence CE + topology-mode CE + multi-rule rect clearance (max over
  rules, winner-only) + occupancy-soft penalty (bilinear sample of the
  planner-available occ grid at predicted points) + curvature penalty.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

from topopatch import N_TOPO_CLASSES

BEV_PX = 128
N_CTRL = 6
N_HYP = 4
N_LOCAL_PTS = 32
# head index -> topology mode (NEG=0, POS=1); AMBIGUOUS trains all heads.
HEAD_MODES = (0, 0, 1, 1)


def conv_block(cin, cout):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, stride=2, padding=1),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
    )


class TopoRulePatchNet(nn.Module):
    def __init__(self, m_hyp=N_HYP, n_ctrl=N_CTRL, in_ch=6,
                 n_topo=N_TOPO_CLASSES):
        super().__init__()
        self.m_hyp, self.n_ctrl = int(m_hyp), int(n_ctrl)
        assert self.m_hyp == len(HEAD_MODES)
        self.n_topo = int(n_topo)
        self.enc = nn.Sequential(
            conv_block(in_ch, 32),    # 64
            conv_block(32, 64),       # 32
            conv_block(64, 128),      # 16
            conv_block(128, 128),     # 8
        )
        self.spatial_fc = nn.Sequential(
            nn.Linear(8 * 8 * 128, 256), nn.ReLU(inplace=True),
            nn.Dropout(p=0.2),
        )
        self.path_mlp = nn.Sequential(
            nn.Linear(2 * N_LOCAL_PTS, 128), nn.ReLU(inplace=True),
            nn.Linear(128, 128), nn.ReLU(inplace=True),
        )
        self.rule_mlp = nn.Sequential(
            nn.Linear(3 * 6, 64), nn.ReLU(inplace=True),
            nn.Linear(64, 64), nn.ReLU(inplace=True),
        )
        self.fuse = nn.Sequential(
            nn.Linear(256 + 128 + 64, 256), nn.ReLU(inplace=True),
            nn.Dropout(p=0.2),
        )
        self.topo_head = nn.Linear(256, self.n_topo)
        # M4 adaptive scope heads (lightweight, on fused features):
        # repairability logit, span regression (m), complexity logits.
        # Topology head is preserved as an ABLATION only (M4 decision:
        # topology is not a core contribution; do not tune it further).
        self.repair_head = nn.Linear(256, 1)
        self.span_head = nn.Linear(256, 1)
        self.complexity_head = nn.Linear(256, 3)
        self.heads = nn.ModuleList([
            nn.Linear(256, n_ctrl * 2 + 1) for _ in range(m_hyp)
        ])

    def forward(self, bev, path_pts, rule_feats):
        f = self.spatial_fc(self.enc(bev).flatten(1))
        g = self.path_mlp(path_pts.flatten(1))
        r = self.rule_mlp(rule_feats.flatten(1))
        h = self.fuse(torch.cat([f, g, r], dim=1))
        topo_logits = self.topo_head(h)
        scope = {"repair_logit": self.repair_head(h).squeeze(1),
                 "span_pred": self.span_head(h).squeeze(1),
                 "complexity_logits": self.complexity_head(h)}
        ctrls, confs = [], []
        for head in self.heads:
            o = head(h)
            ctrls.append(torch.tanh(o[:, :self.n_ctrl * 2]).view(-1, self.n_ctrl, 2))
            confs.append(o[:, self.n_ctrl * 2])
        return (torch.stack(ctrls, dim=1), torch.stack(confs, dim=1),
                topo_logits, scope)  # (B,M,6,2),(B,M),(B,3),scope


def count_params(model) -> int:
    return sum(p.numel() for p in model.parameters())


def torch_catmull_rom(ctrl, samples_per_seg=12):
    """Differentiable batched Catmull-Rom. ctrl: (...,K,2) -> (...,1+(K-1)*S,2)."""
    ext = torch.cat([ctrl[..., :1, :], ctrl, ctrl[..., -1:, :]], dim=-2)
    segs = []
    for i in range(1, ext.shape[-2] - 2):
        p0, p1, p2, p3 = ext[..., i - 1, :], ext[..., i, :], \
            ext[..., i + 1, :], ext[..., i + 2, :]
        t = torch.linspace(0, 1, samples_per_seg + 1, device=ctrl.device,
                           dtype=ctrl.dtype)[1:].view(*([1] * (p1.dim() - 1)),
                                                       -1, 1)
        pt = 0.5 * ((2 * p1).unsqueeze(-2) + (-p0 + p2).unsqueeze(-2) * t
                    + (2 * p0 - 5 * p1 + 4 * p2 - p3).unsqueeze(-2) * t * t
                    + (-p0 + 3 * p1 - 3 * p2 + p3).unsqueeze(-2) * t * t * t)
        segs.append(pt)
    head = ctrl[..., :1, :]
    return torch.cat([head] + segs, dim=-2)


def _poly_margin_penalty(pred_dense, polys, polys_n, margin=0.0625):
    """Max over rules of mean ReLU(margin - dist). polys: (B,K,E,2) float
    with per-batch counts polys_n (B,1); padded rows are zero-area at
    origin — excluded via counts. Differentiable."""
    B, M, P, _ = pred_dense.shape
    K = polys.shape[1]
    pens = []
    for k in range(K):
        poly = polys[:, k]  # (B,E,2)
        corners = torch.cat([poly, poly[:, :1]], dim=1)
        a = corners[:, :-1, :].unsqueeze(1).unsqueeze(3)  # (B,1,E-1?,1,2)
        b = corners[:, 1:, :].unsqueeze(1).unsqueeze(3)
        q = pred_dense.unsqueeze(2)
        ab = b - a
        t = ((q - a) * ab).sum(-1) / (ab.pow(2).sum(-1) + 1e-9)
        t = t.clamp(0, 1).unsqueeze(-1)
        proj = a + t * ab
        d = (q - proj).pow(2).sum(-1).add(1e-6).sqrt()
        dmin = d.min(dim=2).values  # (B,M,P)
        pens.append(torch.relu(margin - dmin).mean(dim=2))  # (B,M)
    pen_rules = torch.stack(pens, dim=2)  # (B,M,K)
    valid = (torch.arange(K, device=polys.device)[None, :]
             < polys_n.to(torch.long)).unsqueeze(1).expand(B, M, K)
    pen_rules = pen_rules.masked_fill(~valid, -1.0)
    return pen_rules.max(dim=2).values.clamp(min=0.0)  # (B,M)


def occ_soft_penalty(pred_dense, occ):
    """Mean bilinear-sampled occupancy at predicted points.

    occ: (B,1,H,W) float {0 free, 1 blocked} in BEV frame; pred_dense in
    norm coords [-1,1]. Differentiable; planner-available info only.
    """
    B, M = pred_dense.shape[:2]
    occ_bm = occ.unsqueeze(1).expand(B, M, 1, occ.shape[2],
                                     occ.shape[3]).reshape(B * M, 1,
                                                           occ.shape[2],
                                                           occ.shape[3])
    grid = pred_dense.view(B * M, -1, 1, 2)
    # grid_sample expects (x,y) in [-1,1] with align_corners=False mapping
    # that matches our pixel-center convention closely enough for a loss.
    samp = F.grid_sample(occ_bm, grid,
                         mode="bilinear", align_corners=False)
    return samp.view(B, M, -1).mean(dim=2)


def curvature_penalty(pred_dense, cos_thresh=0.5):
    """Mean ReLU(cos_thresh - cos(angle)) over dense triplets.

    cos_thresh=0.5 penalizes turns sharper than 60 deg (verifier limit).
    """
    v1 = pred_dense[..., 1:-1, :] - pred_dense[..., :-2, :]
    v2 = pred_dense[..., 2:, :] - pred_dense[..., 1:-1, :]
    n1 = v1.pow(2).sum(-1).add(1e-12).sqrt()
    n2 = v2.pow(2).sum(-1).add(1e-12).sqrt()
    cosang = (v1 * v2).sum(-1) / (n1 * n2)
    return torch.relu(cos_thresh - cosang).mean(dim=-1)  # (B,M)


def topo_wta_loss(pred_ctrls, pred_confs, topo_logits, tgt_ctrl, tgt_dense,
                  tgt_topo, tgt_amb, polys, polys_n, occ,
                  end_weight=5.0, dense_weight=1.0, ctrl_weight=0.5,
                  margin_lambda=2.0, margin=0.0625, occ_lambda=0.5,
                  curv_lambda=0.2, topo_weight=1.0, mode_condition=True,
                  scope=None, tgt_repair=None, tgt_span=None,
                  tgt_cx=None, repair_w=1.0, span_w=0.05, cx_w=0.5,
                  repair_pos_weight=1.5):
    """Mode-conditional WTA + topology CE + geometric penalties.

    tgt_topo: (B,) long {0 NEG, 1 POS, 2 AMB}; tgt_amb: (B,) bool.
    topo_weight=0 + mode_condition=False disables ALL topology
    supervision (ablation: plain multimodal WTA). Returns (loss, parts).

    M4 scope multitask (optional): scope=(repair_logit, span_pred,
    complexity_logits); tgt_repair (B,) float {0,1}, tgt_span (B,)
    meters, tgt_cx (B,) long {0,1,2}. Adds BCE (pos-weighted) + MSE +
    CE. GT is rigid-invariant (lengths), so augmentation needs no
    label recompute for scope.
    """
    B, M = pred_ctrls.shape[:2]
    dev = pred_ctrls.device
    sq = (pred_ctrls - tgt_ctrl[:, None]) ** 2
    w = torch.ones(sq.shape[2], device=dev)
    w[0] = end_weight
    w[-1] = end_weight
    mse_ctrl = (sq * w[None, None, :, None]).mean(dim=(2, 3))
    pred_dense = torch_catmull_rom(pred_ctrls)
    mse_dense = ((pred_dense - tgt_dense[:, None]) ** 2).mean(dim=(2, 3))
    mse = ctrl_weight * mse_ctrl + dense_weight * mse_dense
    head_modes = torch.tensor(HEAD_MODES, device=dev)  # (M,)
    # allowed heads per sample: ground-truth mode's heads, or all if AMB
    # (or always all when mode conditioning is ablated).
    mode_of = tgt_topo.clamp(0, 1)  # AMB->POS row unused (masked below)
    allowed = (head_modes[None, :] == mode_of[:, None])
    allowed = allowed | tgt_amb[:, None]
    if not mode_condition:
        allowed = torch.ones_like(allowed, dtype=torch.bool)
    mse_masked = mse.masked_fill(~allowed, float("inf"))
    win = mse_masked.argmin(dim=1)
    loss_reg = mse[torch.arange(B), win].mean()
    loss_conf = torch.nn.functional.cross_entropy(pred_confs, win)
    # topology target: hard label, or uniform over NEG/POS if ambiguous.
    with torch.no_grad():
        tgt_soft = torch.zeros(B, 3, device=dev)
        hard = ~tgt_amb
        tgt_soft[hard, tgt_topo[hard]] = 1.0
        tgt_soft[tgt_amb, 0] = 0.5
        tgt_soft[tgt_amb, 1] = 0.5
    loss_topo = -(tgt_soft * torch.log_softmax(topo_logits, dim=1)).sum(1).mean()
    loss = loss_reg + loss_conf + topo_weight * loss_topo
    # Geometric penalties apply to ALL heads (mean), not just the WTA
    # winner: every mode is a verifier candidate at inference, so every
    # mode must avoid rules and walls regardless of side. reg stays
    # winner-only (mode specialization).
    pen = _poly_margin_penalty(pred_dense, polys, polys_n, margin=margin)
    pen_w = pen.mean()
    loss = loss + margin_lambda * pen_w
    occ_pen = occ_soft_penalty(pred_dense, occ)
    occ_w = occ_pen.mean()
    loss = loss + occ_lambda * occ_w
    curv = curvature_penalty(pred_dense)
    curv_w = curv.mean()
    loss = loss + curv_lambda * curv_w
    loss_repair = loss_span = loss_cx = None
    rep_acc = span_mae = cx_acc = float("nan")
    if scope is not None and tgt_repair is not None:
        rep_logit = scope["repair_logit"]
        pw = torch.tensor([repair_pos_weight], device=dev)
        loss_repair = torch.nn.functional.binary_cross_entropy_with_logits(
            rep_logit, tgt_repair.float(), pos_weight=pw)
        loss = loss + repair_w * loss_repair
        with torch.no_grad():
            rep_acc = float((((rep_logit >= 0).float()
                              == tgt_repair.float())).float().mean())
    if scope is not None and tgt_span is not None:
        loss_span = torch.nn.functional.mse_loss(
            scope["span_pred"], tgt_span.float())
        loss = loss + span_w * loss_span
        with torch.no_grad():
            span_mae = float((scope["span_pred"].detach()
                              - tgt_span.float()).abs().mean())
    if scope is not None and tgt_cx is not None:
        loss_cx = torch.nn.functional.cross_entropy(
            scope["complexity_logits"], tgt_cx.long())
        loss = loss + cx_w * loss_cx
        with torch.no_grad():
            cx_acc = float((scope["complexity_logits"].detach().argmax(
                dim=1) == tgt_cx.long()).float().mean())
    with torch.no_grad():
        mean_min = mse.min(dim=1).values.mean()
        pred_mode = topo_logits.argmax(dim=1)
        mode_acc_all = (pred_mode == tgt_topo).float().mean()
        hard = ~tgt_amb
        mode_acc_hard = (pred_mode[hard] == tgt_topo[hard]).float().mean() \
            if bool(hard.any()) else float("nan")
    with torch.no_grad():
        parts = {"reg": float(loss_reg.detach()),
                 "conf": float(loss_conf.detach()),
                 "topo": float((topo_weight * loss_topo).detach()),
                 "min_mse": float(mean_min),
                 "margin": float(pen_w.detach()),
                 "occ": float(occ_w.detach()),
                 "curv": float(curv_w.detach()),
                 "mode_acc": float(mode_acc_all),
                 "mode_acc_hard": float(mode_acc_hard),
                 "repair": float(loss_repair.detach())
                 if loss_repair is not None else float("nan"),
                 "span": float(loss_span.detach())
                 if loss_span is not None else float("nan"),
                 "cx": float(loss_cx.detach())
                 if loss_cx is not None else float("nan"),
                 "rep_acc": rep_acc, "span_mae": span_mae,
                 "cx_acc": cx_acc}
    return loss, parts
