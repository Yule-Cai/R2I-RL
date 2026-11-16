"""Unit tests for training-time augmentation (needs torch for import)."""
import numpy as np
import pytest

torch = pytest.importorskip("torch")

from topopatch.topo.extract_topo import extract_topo_sample
from topopatch.topo.train_topo import _augment


def occ_free(x, y):
    return False


BOUNDS = (-20, -20, 20, 20)


class SeqRng:
    def __init__(self, vals):
        self.v = list(vals)

    def randint(self, a, b):
        return self.v.pop(0)


def _sample():
    rect = [(2.5, -0.5), (2.5, 0.5), (3.5, 0.5), (3.5, -0.5)]
    xs = np.linspace(0, 8, 81)
    orig = np.stack([xs, np.zeros_like(xs)], axis=1)
    bump = np.where((xs > 2) & (xs < 6),
                    2.0 * np.sin((xs - 2) / 4 * np.pi), 0.0)
    repl = np.stack([xs, bump], axis=1)
    rec = {"uid": "a1", "map_id": "m",
           "goal": np.asarray([8.0, 0.0]), "orig": orig, "repl": repl,
           "polys": [rect], "geoms": ["rect"], "n_rules": 1}
    s, reason = extract_topo_sample(rec, occ_free, BOUNDS, None)
    assert s is not None, reason
    assert int(s["topo_cls"][0]) == 1  # SIDE_POS
    return s


def _args(s):
    from rulepatch.rp.bev import world_to_norm
    from rulepatch.rp.geom import resample_polyline
    center = tuple(s["center"])
    return (dict(bev4=s["bev"].astype(np.float32) / 255.0,
                 local=s["local"].astype(np.float32),
                 tgt=s["tgt"].astype(np.float32),
                 dense=world_to_norm(
                     resample_polyline(s["repl_seg"], 61),
                     center).astype(np.float32),
                 polys_n=s["polys"].astype(np.float32),
                 rule_feats=s["rule_feats"].astype(np.float32),
                 center=center, A=tuple(s["A"]), B=tuple(s["B"]),
                 repl_seg=np.asarray(s["repl_seg"], dtype=np.float64)))


def test_augment_identity_preserves_label():
    s = _sample()
    kw = _args(s)
    out = _augment(**kw, rng=SeqRng([0, 0, 0]))
    (b, local2, tgt2, dense2, polys2, rf, cls, amb) = out
    assert np.array_equal(b, kw["bev4"])
    assert np.array_equal(local2, kw["local"])
    assert cls == int(s["topo_cls"][0]) == 1
    assert amb is False
    assert np.allclose(rf, kw["rule_feats"])


def test_augment_yflip_swaps_side_and_centroid():
    s = _sample()
    kw = _args(s)
    out = _augment(**kw, rng=SeqRng([0, 1, 0]))
    (b, local2, tgt2, dense2, polys2, rf, cls, amb) = out
    assert cls == 0  # SIDE_NEG after y flip
    assert amb is False
    # centroid col negated, w/h untouched, area/cross invariant
    assert abs(rf[0, 2] + kw["rule_feats"][0, 2]) < 1e-6
    assert abs(rf[0, 1] - kw["rule_feats"][0, 1]) < 1e-6
    assert abs(rf[0, 3] - kw["rule_feats"][0, 3]) < 1e-6
    assert abs(rf[0, 0] - kw["rule_feats"][0, 0]) < 1e-9
    # BEV rule channel still covers the (flipped) rule: union count same
    assert int((b[2] > 0.5).sum()) == int((kw["bev4"][2] > 0.5).sum())
