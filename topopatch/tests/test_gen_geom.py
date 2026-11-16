"""Unit tests for topo dataset geometry samplers (no ROS)."""
import math
import random

import numpy as np

from topopatch.tools.gen_topo_dataset import (
    GEOM_TYPES, clearance_mask, count_inside, pip, sample_convex,
    sample_elongated, sample_n_rules, sample_narrow, sample_notch,
    sample_rect, try_place_rules)


def test_clearance_mask_centered():
    img = np.full((40, 40), 254, dtype=np.uint8)
    img[20, 20] = 0  # single wall pixel
    m = clearance_mask(img, 0.05, clear_m=0.30)  # r=6 -> 13x13 box
    # center (20,20) blocked (box contains the wall); far corner clear
    assert not m[20, 20]
    assert m[5, 5]
    # brute-force cross-check on random map (interior points only)
    rng = np.random.RandomState(0)
    noisy = np.where(rng.random((40, 40)) < 0.9, 254, 0).astype(np.uint8)
    m2 = clearance_mask(noisy, 0.05, clear_m=0.30)
    free = (noisy >= 254)
    for (y, x) in [(10, 10), (20, 20), (30, 30), (12, 28)]:
        expect = bool(free[y - 6:y + 7, x - 6:x + 7].all())
        assert bool(m2[y, x]) == expect, (y, x)


def _area(pts):
    a = 0.0
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return abs(a) * 0.5


def test_all_samplers_valid():
    rng = random.Random(0)
    fns = {
        "rect": lambda: sample_rect(rng, 0, 0, 1.0, 2.0),
        "convex": lambda: sample_convex(rng, 0, 0, 1.0, 2.0),
        "notch": lambda: sample_notch(rng, 0, 0, 1.5, 2.5),
        "narrow": lambda: sample_narrow(rng, 0, 0, 0.3, 1.0, 2.0),
        "elongated": lambda: sample_elongated(rng, 0, 0, 0.3, 1.0, 2.0,
                                              0.3),
    }
    for name, fn in fns.items():
        for _ in range(20):
            poly, gt = fn()
            assert gt == name
            assert len(poly) >= 3, (name, poly)
            assert all(math.isfinite(x) and math.isfinite(y)
                       for x, y in poly)
            assert _area(poly) > 0.05, (name, _area(poly))


def test_narrow_is_thin_and_long():
    rng = random.Random(5)
    poly, _ = sample_narrow(rng, 0, 0, 0.0, 1.0, 2.0)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    # perpendicular to heading 0 -> long in y, thin in x
    assert (max(ys) - min(ys)) > 1.5
    assert (max(xs) - min(xs)) < 0.6


def test_n_rules_distribution():
    rng = random.Random(1)
    ns = [sample_n_rules(rng) for _ in range(2000)]
    assert set(ns) == {1, 2, 3}
    assert 0.4 < sum(1 for v in ns if v == 1) / 2000 < 0.6


def test_try_place_rules_synthetic():
    rng = random.Random(3)
    xs = np.linspace(0, 10, 101)
    orig = list(zip(xs, np.zeros_like(xs)))
    s, g = (-1.0, 0.0), (11.0, 0.0)
    for _ in range(10):
        rules = try_place_rules(rng, orig, s, g, 1.0, 2.0, 0.3)
        if rules is None:
            continue
        assert 1 <= len(rules) <= 3
        assert sum(count_inside(orig, r["poly"]) for r in rules) >= 5
        assert all(r["geom"] in GEOM_TYPES for r in rules)
        return
    raise AssertionError("placement never succeeded on trivial straight path")


def test_pip_degenerate_safe():
    assert isinstance(pip((0, 0), [(0, 0), (1, 1)]), bool)
