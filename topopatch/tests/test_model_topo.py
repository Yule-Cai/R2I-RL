"""Unit tests for TopoRulePatch model + loss (needs torch, no ROS)."""
import numpy as np
import torch

from topopatch.topo.model_topo import (
    HEAD_MODES, TopoRulePatchNet, count_params, curvature_penalty,
    occ_soft_penalty, torch_catmull_rom, topo_wta_loss)


def _batch(b=2):
    torch.manual_seed(0)
    m = TopoRulePatchNet()
    m.eval()
    bev = torch.rand(b, 6, 128, 128)
    loc = torch.rand(b, 32, 2) * 2 - 1
    rf = torch.rand(b, 3, 6)
    return m, bev, loc, rf


def test_param_budget():
    m = TopoRulePatchNet()
    n = count_params(m)
    assert n <= 5_000_000, n
    assert n > 2_000_000  # nontrivial shared trunk (sanity)


def test_forward_shapes():
    m, bev, loc, rf = _batch()
    with torch.no_grad():
        ctrls, confs, topo, scope = m(bev, loc, rf)
    assert ctrls.shape == (2, 4, 6, 2)
    assert confs.shape == (2, 4)
    assert topo.shape == (2, 3)
    assert bool(((ctrls <= 1.0) & (ctrls >= -1.0)).all())


def test_head_mode_tags():
    assert tuple(HEAD_MODES) == (0, 0, 1, 1)


def test_loss_finite_and_backward():
    torch.manual_seed(1)
    m = TopoRulePatchNet()
    m.train()
    B = 3
    bev = torch.rand(B, 6, 128, 128)
    loc = torch.rand(B, 32, 2) * 2 - 1
    rf = torch.rand(B, 3, 6)
    tgt_c = torch.rand(B, 6, 2) * 2 - 1
    tgt_d = torch.rand(B, 61, 2) * 2 - 1
    tgt_t = torch.tensor([0, 1, 2])
    tgt_a = torch.tensor([False, False, True])
    rect = torch.zeros(B, 3, 4, 2)
    rect[:, 0] = torch.tensor([[-0.2, -0.2], [-0.2, 0.2],
                               [0.2, 0.2], [0.2, -0.2]])
    pn = torch.ones(B, 1, dtype=torch.long)
    occ = torch.zeros(B, 1, 128, 128)
    pc, pf, pt, ps = m(bev, loc, rf)
    loss, parts = topo_wta_loss(pc, pf, pt, tgt_c, tgt_d, tgt_t,
                                tgt_a, rect, pn, occ)
    assert torch.isfinite(loss)
    for k in ("reg", "conf", "topo", "min_mse", "margin", "occ", "curv",
              "mode_acc", "mode_acc_hard"):
        assert k in parts and np.isfinite(parts[k]), k
    loss.backward()
    grads = [p.grad for p in m.parameters() if p.requires_grad]
    assert any(g is not None and bool((g != 0).any()) for g in grads)


def test_mode_conditional_winner():
    # NEG sample whose closest head is POS-only must still pick a NEG head.
    torch.manual_seed(0)
    B = 1
    tgt_c = torch.zeros(B, 6, 2)
    tgt_d = torch.zeros(B, 61, 2)
    pc = torch.zeros(B, 4, 6, 2)
    pc[0, 2:] = 0.1  # POS heads near (unconstrained WTA would pick these)
    pc[0, :2] = 0.5  # NEG heads farther
    pf = torch.zeros(B, 4)
    pt = torch.zeros(B, 3)
    tgt_t = torch.tensor([0])
    tgt_a = torch.tensor([False])
    rect = torch.zeros(B, 3, 4, 2)
    pn = torch.zeros(B, 1, dtype=torch.long)  # no rules -> margin 0
    occ = torch.zeros(B, 1, 128, 128)
    _, parts = topo_wta_loss(pc, pf, pt, tgt_c, tgt_d, tgt_t,
                             tgt_a, rect, pn, occ, margin_lambda=0.0,
                             occ_lambda=0.0, curv_lambda=0.0,
                             dense_weight=0.0, ctrl_weight=1.0,
                             end_weight=1.0)
    # winner must be head 0 or 1 (NEG) with reg = plain ctrl MSE = 0.25
    # (unconstrained WTA would give 0.01 from the POS heads).
    assert abs(parts["reg"] - 0.25) < 1e-6


def test_curvature_and_occ_penalties():
    straight = torch.zeros(1, 1, 61, 2)
    straight[0, 0, :, 0] = torch.linspace(-1, 1, 61)
    assert float(curvature_penalty(straight)) < 1e-6
    occ = torch.zeros(1, 1, 128, 128)
    assert float(occ_soft_penalty(straight, occ)) == 0.0
    occ_full = torch.ones(1, 1, 128, 128)
    assert float(occ_soft_penalty(straight, occ_full)) > 0.9


def test_catmull_rom_shape():
    c = torch.rand(2, 4, 6, 2)
    d = torch_catmull_rom(c)
    assert d.shape == (2, 4, 61, 2)


def test_no_topo_ablation_unconstrains_winner():
    # Same setup as mode-conditional test, but mode_condition=False ->
    # winner is the global closest head (POS head 2/3, reg 0.01) and the
    # topo term is exactly zero.
    torch.manual_seed(0)
    B = 1
    tgt_c = torch.zeros(B, 6, 2)
    tgt_d = torch.zeros(B, 61, 2)
    pc = torch.zeros(B, 4, 6, 2)
    pc[0, 2:] = 0.1
    pc[0, :2] = 0.5
    pf = torch.zeros(B, 4)
    pt = torch.zeros(B, 3)
    tgt_t = torch.tensor([0])
    tgt_a = torch.tensor([False])
    rect = torch.zeros(B, 3, 4, 2)
    pn = torch.zeros(B, 1, dtype=torch.long)
    occ = torch.zeros(B, 1, 128, 128)
    _, parts = topo_wta_loss(pc, pf, pt, tgt_c, tgt_d, tgt_t,
                             tgt_a, rect, pn, occ, margin_lambda=0.0,
                             occ_lambda=0.0, curv_lambda=0.0,
                             dense_weight=0.0, ctrl_weight=1.0,
                             end_weight=1.0, topo_weight=0.0,
                             mode_condition=False)
    assert abs(parts["reg"] - 0.01) < 1e-6
    assert abs(parts["topo"]) < 1e-9
