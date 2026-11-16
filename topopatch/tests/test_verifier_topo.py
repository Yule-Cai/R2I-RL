"""Unit tests for multi-rule verifier (no ROS, no torch)."""
import numpy as np

from topopatch.topo.verifier_topo import (select_topo, verify_dense_multi,
                                          verify_patch_multi)


def occ_free(x, y):
    return False


BOUNDS = (-20, -20, 20, 20)
A, B = (-2.0, 0.0), (2.0, 0.0)
ORIG = [(-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0)]
RULE = [(0.4, -1), (0.4, 1), (0.8, 1), (0.8, -1)]


def _norm(pts, center=(0.0, 0.0)):
    from rulepatch.rp.bev import world_to_norm
    return world_to_norm(pts, center)


def _smooth_ctrl(amp=0.3):
    # 6 control points sampled from a gentle sine arc (model-like output)
    from rulepatch.rp.geom import resample_polyline
    xs = np.linspace(-2, 2, 61)
    ys = amp * np.sin((xs + 2) / 4 * np.pi)
    dense = np.stack([xs, ys], axis=1)
    return resample_polyline(dense, 6)


def test_multi_rule_keepout_second_rule():
    # smooth shallow arc avoids `far` but crosses RULE -> must fail on RULE.
    # (Straight == orig would trip no-progress instead; keep novelty.)
    from rulepatch.rp.verifier import decode_ctrl
    c = (0.0, 0.0)
    shallow = _norm(_smooth_ctrl(0.3), c)
    far = [(9, 9), (9, 10), (10, 10), (10, 9)]
    ok, _ = verify_dense_multi(decode_ctrl(shallow, c, A=A, B=B), A, B,
                               ORIG, [far], occ_free, BOUNDS, c)
    assert ok
    ok2, r2 = verify_dense_multi(decode_ctrl(shallow, c, A=A, B=B), A, B,
                                 ORIG, [far, RULE], occ_free, BOUNDS, c)
    assert not ok2 and r2 == "KEEP_OUT violation"


def test_single_rule_matches_m2():
    # with one rule, multi verifier agrees with frozen M2 verifier
    from rulepatch.rp import verifier as V
    c = (0.0, 0.0)
    good = _norm([(-2, 0), (-1, 1.2), (0, 1.4), (1, 1.2), (1.9, 0.4),
                  (2, 0)], c)
    ok1, _ = V.verify_patch(good, c, A, B, ORIG, RULE, occ_free, BOUNDS)
    ok2, _ = verify_patch_multi(good, c, A, B, ORIG, [RULE], occ_free,
                                BOUNDS)
    assert ok1 and ok2
    bad = _norm([(-2, 0), (-1, 0), (0, 0), (1, 0), (1.9, 0), (2, 0)], c)
    ok3, _ = V.verify_patch(bad, c, A, B, ORIG, RULE, occ_free, BOUNDS)
    ok4, _ = verify_patch_multi(bad, c, A, B, ORIG, [RULE], occ_free,
                                BOUNDS)
    assert not ok3 and not ok4


def test_select_threshold_abstains():
    c = (0.0, 0.0)
    good = _norm([(-2, 0), (-1, 1.2), (0, 1.4), (1, 1.2), (1.9, 0.4),
                  (2, 0)], c)
    ctrls = np.stack([good, good])
    idx, _, rs = select_topo(ctrls, np.array([0.9, 0.1]), c, A, B, ORIG,
                             [RULE], occ_free, BOUNDS, tau=0.0)
    assert idx == 0
    idx2, _, rs2 = select_topo(ctrls, np.array([0.9, 0.1]), c, A, B,
                               ORIG, [RULE], occ_free, BOUNDS, tau=2.0)
    assert idx2 is None and rs2.startswith("abstain")
