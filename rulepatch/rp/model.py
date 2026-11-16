"""RulePatchNet v2: spatially-preserving encoder + dense trajectory loss.

v1 diagnosis (held-out): global average pooling destroyed localization
(winner RMSE ~1m); v2 flattens the 8x8x128 feature map into the head and
adds coordinate channels, keeping <=3M params.
Input: 6x128x128 BEV (occ/orig-path/rule/goal/coord-x/coord-y) + 32x2 path.
Output: M hypotheses x (6 spline control points + confidence).
"""
import numpy as np
import torch
import torch.nn as nn

from . import BEV_PX, N_CTRL, N_HYP, N_LOCAL_PTS
from .bev import add_coords  # noqa: re-exported (numpy-only, ROS-safe)


def conv_block(cin, cout):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, stride=2, padding=1),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
    )


class RulePatchNet(nn.Module):
    def __init__(self, m_hyp=N_HYP, n_ctrl=N_CTRL, in_ch=6):
        super().__init__()
        self.m_hyp, self.n_ctrl = int(m_hyp), int(n_ctrl)
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
        self.fuse = nn.Sequential(
            nn.Linear(256 + 128, 256), nn.ReLU(inplace=True),
            nn.Dropout(p=0.2),
        )
        self.heads = nn.ModuleList([
            nn.Linear(256, n_ctrl * 2 + 1) for _ in range(m_hyp)
        ])

    def forward(self, bev, path_pts):
        f = self.spatial_fc(self.enc(bev).flatten(1))
        g = self.path_mlp(path_pts.flatten(1))
        h = self.fuse(torch.cat([f, g], dim=1))
        ctrls, confs = [], []
        for head in self.heads:
            o = head(h)
            ctrls.append(torch.tanh(o[:, :self.n_ctrl * 2]).view(-1, self.n_ctrl, 2))
            confs.append(o[:, self.n_ctrl * 2])
        return torch.stack(ctrls, dim=1), torch.stack(confs, dim=1)  # (B,M,6,2),(B,M)


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


def rect_margin_penalty(pred_dense, rect, margin=0.0625):
    """Mean ReLU(margin - dist) of dense points to rect edges.

    pred_dense: (B,M,P,2) norm coords; rect: (B,4,2) norm coords (closed
    implicitly); margin in norm units (0.0625 ~= 0.4m). Differentiable.
    """
    B, M, P, _ = pred_dense.shape
    corners = torch.cat([rect, rect[:, :1]], dim=1)  # (B,5,2)
    a = corners[:, :-1, :].unsqueeze(1).unsqueeze(3)  # (B,1,4,1,2)
    b = corners[:, 1:, :].unsqueeze(1).unsqueeze(3)
    q = pred_dense.unsqueeze(2)  # (B,M,1,P,2)
    ab = b - a
    t = ((q - a) * ab).sum(-1) / (ab.pow(2).sum(-1) + 1e-9)
    t = t.clamp(0, 1).unsqueeze(-1)
    proj = a + t * ab
    d = (q - proj).pow(2).sum(-1).add(1e-6).sqrt()  # eps: stable grads
    dmin = d.min(dim=2).values  # (B,M,P)
    return torch.relu(margin - dmin).mean(dim=2)  # (B,M)


def wta_loss(pred_ctrls, pred_confs, tgt_ctrl, tgt_dense=None,
             end_weight=5.0, dense_weight=1.0, ctrl_weight=0.5,
             rect=None, margin_lambda=2.0, margin=0.0625):
    """WTA over dense shape (+ctrl anchor) + confidence CE to winner.

    Optional winner-only rect clearance penalty (keeps multimodality:
    losers are not pulled toward one conservative mode).
    """
    B, M = pred_ctrls.shape[:2]
    sq = (pred_ctrls - tgt_ctrl[:, None]) ** 2
    w = torch.ones(sq.shape[2], device=sq.device)
    w[0] = end_weight
    w[-1] = end_weight
    mse_ctrl = (sq * w[None, None, :, None]).mean(dim=(2, 3))
    pred_dense = torch_catmull_rom(pred_ctrls)  # (B,M,61,2)
    if tgt_dense is not None:
        mse_dense = ((pred_dense - tgt_dense[:, None]) ** 2).mean(dim=(2, 3))
        mse = ctrl_weight * mse_ctrl + dense_weight * mse_dense
    else:
        mse = mse_ctrl
    win = mse.argmin(dim=1)
    loss_reg = mse[torch.arange(B), win].mean()
    loss_conf = nn.functional.cross_entropy(pred_confs, win)
    loss = loss_reg + loss_conf
    pen = None
    if rect is not None:
        # winner-only application via indexing (gradient flows to the
        # winner slice only, preserving head diversity)
        pen_all = rect_margin_penalty(pred_dense, rect, margin=margin)
        pen = pen_all[torch.arange(B, device=pred_dense.device), win].mean()
        loss = loss + margin_lambda * pen
    with torch.no_grad():
        mean_min = mse.min(dim=1).values.mean()
    parts = {"reg": float(loss_reg), "conf": float(loss_conf),
             "min_mse": float(mean_min),
             "margin": float(pen) if pen is not None else 0.0}
    return loss, parts
