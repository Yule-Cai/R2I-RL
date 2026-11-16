"""Unit tests for topo extraction (no ROS; synthetic records)."""
import numpy as np

from topopatch.topo.extract_topo import (_rule_feats, extract_topo_sample,
                                         load_topo_raw)


def occ_free(x, y):
    return False


BOUNDS = (-20, -20, 20, 20)


def _arc_pair(bulge=2.0, x1=0.0, x2=8.0):
    xs = np.linspace(x1, x2, 81)
    orig = np.stack([xs, np.zeros_like(xs)], axis=1)
    bump = np.where((xs > 2) & (xs < 6),
                    bulge * np.sin((xs - 2) / 4 * np.pi), 0.0)
    repl = np.stack([xs, bump], axis=1)
    return orig, repl


def _rec(polys, geoms, uid="t000001", goal=(8.0, 0.0)):
    orig, repl = _arc_pair()
    return {"uid": uid, "map_id": "office-A-s101",
            "goal": np.asarray(goal, dtype=np.float64),
            "orig": orig, "repl": repl, "polys": polys,
            "geoms": geoms, "n_rules": len(polys)}


def test_single_rect_ok_and_labeled():
    rect = [(2.5, -0.5), (2.5, 0.5), (3.5, 0.5), (3.5, -0.5)]
    s, reason = extract_topo_sample(_rec([rect], ["rect"]), occ_free,
                                    BOUNDS, None)
    assert s is not None, reason
    assert int(s["topo_cls"][0]) == 1  # SIDE_POS (arc over +y)
    assert s["map_id"] == "office-A-s101"
    assert s["polys"].shape == (3, 12, 2) and int(s["polys_n"][0]) == 1
    assert s["rule_feats"].shape == (3, 6)
    assert s["bev"].shape == (4, 128, 128)
    assert s["rect"].shape == (4, 2)  # union rect
    assert isinstance(s["topo_corr"], int)


def test_two_rule_union_channel():
    # orig crosses rectA; repl threads over rectA, under rectB.
    rectA = [(2.0, -0.5), (2.0, 0.4), (4.0, 0.4), (4.0, -0.5)]
    rectB = [(2.0, 1.0), (2.0, 2.0), (6.0, 2.0), (6.0, 1.0)]
    xs = np.linspace(0, 8, 161)
    orig = np.stack([xs, np.zeros_like(xs)], axis=1)
    ys = np.zeros_like(xs)
    for k, x in enumerate(xs):
        if 2.0 <= x <= 4.6:
            ys[k] = 0.7
        elif 1.6 < x < 2.0:
            ys[k] = 0.7 * (x - 1.6) / 0.4
        elif 4.6 < x < 5.0:
            ys[k] = 0.7 * (5.0 - x) / 0.4
    repl = np.stack([xs, ys], axis=1)
    rec = {"uid": "t2", "map_id": "m",
           "goal": np.asarray([8.0, 0.0]), "orig": orig, "repl": repl,
           "polys": [rectA, rectB], "geoms": ["rect", "rect"], "n_rules": 2}
    s, reason = extract_topo_sample(rec, occ_free, BOUNDS, None)
    assert s is not None, reason
    assert s["topo_corr"] == 1
    assert int(s["polys_n"][0]) == 2
    # union rule channel covers both rects
    assert int((s["bev"][2] > 0).sum()) > 400


def test_teacher_no_divergence_drops():
    rect = [(2.5, -0.7), (2.5, 0.7), (3.5, 0.7), (3.5, -0.7)]
    orig, _ = _arc_pair()
    rec = _rec([rect], ["rect"])
    rec["repl"] = np.asarray(orig)  # repl == orig -> no divergence
    s, reason = extract_topo_sample(rec, occ_free, BOUNDS, None)
    assert s is None and "anchor" in reason


def test_rule_feats_shape_and_padding():
    # layout: [area_n, cx_n, cy_n, w_n, h_n, cross]; centroid/size in
    # window-normalized coords (augmentation frame).
    a = [(0, 0), (0, 1), (1, 1), (1, 0)]
    f = _rule_feats([a], (0.5, 0.5), (1.5, 0.5), (1.0, 0.5))
    assert f.shape == (3, 6)
    assert (f[1:] == 0).all()
    assert abs(f[0, 0] - 1.0 / 25.0) < 1e-9  # unit-square area
    assert 0.0 < f[0, 3] <= 1.0 and 0.0 < f[0, 4] <= 1.0  # w/h fractions
    assert -1.0 <= f[0, 1] <= 1.0 and -1.0 <= f[0, 2] <= 1.0


def test_split_items_share_column_base(tmp_path):
    # Regression: per-item entries must be views of one materialized
    # column, not fresh zip decodes (100x blowup -> OOM, 2026-09-25).
    from topopatch.topo.extract_topo import load_topo_split_items
    n = 8
    kw = {"n": np.array([n]),
          "bev": np.zeros((n, 4, 128, 128), np.uint8),
          "uid": np.array([f"u{i}" for i in range(n)]),
          "topo_corr": np.zeros(n, np.int64)}
    np.savez_compressed(str(tmp_path / "train.npz"), **kw)
    items = load_topo_split_items(str(tmp_path), "train")
    assert len(items) == n
    assert items[0]["bev"].base is items[1]["bev"].base
