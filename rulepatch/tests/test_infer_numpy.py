import numpy as np
import torch

from rp.model import RulePatchNet, add_coords
from rp.infer_numpy import NumpyPatchNet, fuse_state_dict


def test_numpy_parity(tmp_path):
    torch.manual_seed(1)
    m = RulePatchNet(m_hyp=3)
    m.eval()
    bev = add_coords(torch.rand(1, 4, 128, 128).numpy())
    bev_t = torch.from_numpy(bev)
    pts = (torch.rand(1, 32, 2) * 2 - 1)
    with torch.no_grad():
        tc, tf = m(bev_t, pts)
    fused = fuse_state_dict(m.state_dict())
    fused["m_hyp"] = np.array([3])
    fused["n_ctrl"] = np.array([6])
    fp = str(tmp_path / "fused.npz")
    np.savez(fp, **fused)
    nm = NumpyPatchNet(fp)
    nc, nf = nm.forward(bev[0], pts[0].numpy())
    assert nc.shape == (3, 6, 2) and nf.shape == (3,)
    assert np.allclose(nc, tc[0].numpy(), atol=1e-4), \
        np.abs(nc - tc[0].numpy()).max()
    assert np.allclose(nf, tf[0].numpy(), atol=1e-4)
