import numpy as np
import torch

from rp.train import augment


def test_augment_geometry_consistent():
    rng = np.random.RandomState(0)
    # single path pixel at norm (0.5, 0.0) -> BEV pixel (u,v)
    bev = np.zeros((4, 128, 128), np.float32)
    u = int(round((0.5 * 0.5 + 0.5) * 127))
    v = int(round((0.5 - 0.0 * 0.5) * 127))
    bev[1, v, u] = 1.0
    loc = np.array([[0.5, 0.0]], np.float32)
    z = np.zeros((1, 2), np.float32)
    b, l, _, _, _ = augment(bev, loc, z, z, z, rng)
    # find the hot pixel in augmented channel 1
    vv, uu = np.where(b[1] > 0.5)
    assert len(uu) == 1
    # norm coords of that pixel must equal transformed local point
    xu = (uu[0] / 127) * 2 - 1
    yv = 1 - (vv[0] / 127) * 2
    assert abs(xu - l[0, 0]) < 0.02 and abs(yv - l[0, 1]) < 0.02, \
        (xu, yv, l[0])
    assert b.flags["C_CONTIGUOUS"]
    torch.from_numpy(b)  # must not raise on strides
