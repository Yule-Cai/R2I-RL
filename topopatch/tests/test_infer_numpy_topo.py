"""Parity test: NumpyTopoNet vs torch TopoRulePatchNet (no ROS)."""
import numpy as np
import torch

from topopatch.topo.infer_numpy_topo import NumpyTopoNet, fuse_topo_state
from topopatch.topo.model_topo import TopoRulePatchNet


def test_numpy_parity(tmp_path):
    torch.manual_seed(1)
    m = TopoRulePatchNet()
    m.eval()
    bev = torch.rand(1, 6, 128, 128)
    pts = (torch.rand(1, 32, 2) * 2 - 1)
    rf = torch.rand(1, 3, 6)
    with torch.no_grad():
        tc, tf, tt, ts = m(bev, pts, rf)
    fused = fuse_topo_state(m.state_dict())
    fused["m_hyp"] = np.array([4])
    fused["n_ctrl"] = np.array([6])
    fused["n_topo"] = np.array([3])
    fp = str(tmp_path / "fused.npz")
    np.savez(fp, **fused)
    nm = NumpyTopoNet(fp)
    nc, nf, ntp, nsc = nm.forward(bev[0].numpy(), pts[0].numpy(), rf[0].numpy())
    assert nc.shape == (4, 6, 2) and nf.shape == (4,) and ntp.shape == (3,)
    assert np.allclose(nc, tc[0].numpy(), atol=1e-4), \
        np.abs(nc - tc[0].numpy()).max()
    assert np.allclose(nf, tf[0].numpy(), atol=1e-4)
    import torch.nn.functional as F
    assert np.allclose(ntp, F.softmax(tt, dim=1)[0].numpy(), atol=1e-6)
    assert abs(ntp.sum() - 1.0) < 1e-9
    import torch as _t
    assert abs(nsc["rep_prob"] - float(_t.sigmoid(ts["repair_logit"])[0])) < 1e-6
    assert abs(nsc["span_pred"] - float(ts["span_pred"][0])) < 1e-4
    assert nsc["cx_pred"] == int(ts["complexity_logits"][0].argmax())
