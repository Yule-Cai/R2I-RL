import numpy as np
import torch

from rp.model import (RulePatchNet, count_params, wta_loss, add_coords,
                      torch_catmull_rom, rect_margin_penalty)


def test_param_budget():
    m = RulePatchNet()
    n = count_params(m)
    assert n <= 3_000_000, n
    m1 = RulePatchNet(m_hyp=1)
    assert count_params(m1) <= 3_000_000


def test_coords():
    b = np.random.rand(2, 4, 128, 128).astype(np.float32)
    c = add_coords(b)
    assert c.shape == (2, 6, 128, 128)


def test_forward_and_loss():
    torch.manual_seed(0)
    m = RulePatchNet()
    bev = torch.rand(2, 6, 128, 128)
    pts = torch.rand(2, 32, 2) * 2 - 1
    ctrls, confs = m(bev, pts)
    assert ctrls.shape == (2, 3, 6, 2) and confs.shape == (2, 3)
    assert bool(((ctrls <= 1.0) & (ctrls >= -1.0)).all())
    tgt = torch.rand(2, 6, 2) * 2 - 1
    dense = torch.rand(2, 61, 2) * 2 - 1
    d = torch_catmull_rom(ctrls)
    assert d.shape == (2, 3, 61, 2)
    loss, parts = wta_loss(ctrls, confs, tgt, tgt_dense=dense)
    assert torch.isfinite(loss)
    loss.backward()
    grads = [p.grad for p in m.parameters() if p.requires_grad]
    assert any(g is not None and bool((g != 0).any()) for g in grads)


def test_rect_margin_penalty():
    torch.manual_seed(0)
    pred = torch.rand(2, 3, 61, 2, requires_grad=True)
    rect = torch.tensor([[[-0.1, -0.1], [-0.1, 0.1], [0.1, 0.1], [0.1, -0.1]]],
                        dtype=torch.float32).expand(2, 4, 2)
    pen = rect_margin_penalty(pred, rect)
    assert pen.shape == (2, 3)
    assert bool((pen >= 0).all())
    far = torch.full((1, 1, 61, 2), 0.9, requires_grad=True)
    pen_far = rect_margin_penalty(far, rect[:1])
    assert float(pen_far) == 0.0
    pen.sum().backward()
    assert pred.grad is not None and bool((pred.grad != 0).any())
