import numpy as np

from rp.bev import (render_bev, render_bev_fast, world_to_norm, norm_to_world, window_for_patch,
                    local_path_points, target_control_points)
from rp.verifier import verify_patch, select_hypothesis
from rp import BEV_PX


def occ_free(x, y):
    return False


def occ_wall(x, y):
    return x > 0.5  # vertical wall east of x=0.5


def test_bev_channels():
    A, B = (-2.0, 0.0), (2.0, 0.0)
    c = window_for_patch(A, B, (-20, -20, 20, 20))
    assert abs(c[0]) < 1e-9 and abs(c[1]) < 1e-9
    bev = render_bev(occ_free, c, [A, (0, 0), B],
                     [(0.5, -0.5), (0.5, 0.5), (1.5, 0.5), (1.5, -0.5)], (4.0, 0.0))
    assert bev.shape == (4, BEV_PX, BEV_PX)
    assert bev[0].sum() == 0.0
    assert bev[1].sum() > 0 and bev[2].sum() > 0 and bev[3].sum() > 0
    # roundtrip
    w = norm_to_world(world_to_norm([A, B], c), c)
    assert np.allclose(w, [A, B])


def test_bev_fast_parity():
    # full grid row0=ymin, res 0.1, origin (-20,-20): wall at x>0.5
    res, ox, oy = 0.1, -20.0, -20.0
    full = np.zeros((400, 400), np.uint8)
    full[:, int((0.5 - ox) / res):] = 1
    c = (0.0, 0.0)
    seg = [(-2, 0), (0, 0), (2, 0)]
    rule = [(0.5, -0.5), (0.5, 0.5), (1.5, 0.5), (1.5, -0.5)]
    slow = render_bev(lambda x, y: x > 0.5, c, seg, rule, (4.0, 0.0))
    fast = render_bev_fast(full, res, ox, oy, c, seg, rule, (4.0, 0.0))
    assert fast.shape == slow.shape
    assert np.allclose(fast, slow, atol=1e-6), np.abs(fast - slow).max()


def test_decode_pins_anchors():
    from rp.verifier import decode_ctrl
    c = (0.0, 0.0)
    ctrl = world_to_norm([(-2, 0), (-1, 0), (0, 0), (1, 0), (1.9, 0), (2, 0)], c)
    d = decode_ctrl(ctrl, c, A=(-2.1, 0.1), B=(2.1, -0.1))
    assert abs(d[0, 0] + 2.1) < 1e-9 and abs(d[0, 1] - 0.1) < 1e-9
    assert abs(d[-1, 0] - 2.1) < 1e-9 and abs(d[-1, 1] + 0.1) < 1e-9
    d2 = decode_ctrl(ctrl, c)
    assert abs(d2[0, 0] + 2.0) < 1e-9  # raw without anchors


def test_verifier_accept_and_reject():
    A, B = (-2.0, 0.0), (2.0, 0.0)
    c = (0.0, 0.0)
    orig = [(-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0)]
    rule = [(0.4, -1), (0.4, 1), (0.8, 1), (0.8, -1)]  # thin keepout on the line
    bounds = (-20, -20, 20, 20)
    good = world_to_norm([(-2, 0), (-1, 1.2), (0, 1.4), (1, 1.2), (1.9, 0.4), (2, 0)], c)
    ok, reason = verify_patch(good, c, A, B, orig, rule, occ_free, bounds)
    assert ok, reason
    bad = world_to_norm([(-2, 0), (-1, 0), (0, 0), (1, 0), (1.9, 0), (2, 0)], c)
    ok2, r2 = verify_patch(bad, c, A, B, orig, rule, occ_free, bounds)
    assert not ok2 and "KEEP_OUT" in r2
    # collision
    ok3, r3 = verify_patch(good, c, A, B, orig,
                           [(9, 9), (9, 10), (10, 10), (10, 9)], occ_wall, bounds)
    assert not ok3 and r3 == "collision"


def test_select_fallback():
    A, B = (-2.0, 0.0), (2.0, 0.0)
    c = (0.0, 0.0)
    orig = [(-2, 0), (0, 0), (2, 0)]
    rule = [(0.4, -1), (0.4, 1), (0.8, 1), (0.8, -1)]
    bounds = (-20, -20, 20, 20)
    bad = world_to_norm([(-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0), (2, 0)], c)
    idx, dense, reason = select_hypothesis(
        [bad, bad, bad], [0.5, 0.3, 0.2], c, A, B, orig, rule, occ_free, bounds)
    assert idx is None and "all-fail" in reason
