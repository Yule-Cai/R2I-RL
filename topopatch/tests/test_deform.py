"""Unit tests for the classical deformation baseline (no ROS, no torch)."""
import math

import numpy as np

from topopatch.topo.deform import (_bilinear, deform_patch, make_sdf_fn)


def _flat_sdf(cx=0.0, cy=0.0, res=0.1, n=64, wall_at=None):
    # SDF-like grid: distance to a vertical wall line x=wall_at (or big).
    yy, xx = np.mgrid[0:n, 0:n]
    wx = (xx + 0.5) * res + cx
    if wall_at is None:
        return np.full((n, n), 5.0), cx, cy, res
    return np.abs(wx - wall_at).astype(np.float64), cx, cy, res


def test_endpoints_pinned_and_deterministic():
    orig = [(float(x), 0.0) for x in np.linspace(0, 6, 31)]
    rect = [(2.0, -0.5), (2.0, 0.5), (4.0, 0.5), (4.0, -0.5)]
    g, ox, oy, res = _flat_sdf()
    fn = make_sdf_fn(g, ox, oy, res)
    p1 = deform_patch(orig, [rect], fn, (0, 0), (6, 0))
    p2 = deform_patch(orig, [rect], fn, (0, 0), (6, 0))
    assert np.array_equal(p1, p2)
    assert np.allclose(p1[0], (0, 0)) and np.allclose(p1[-1], (6, 0))
    assert p1.shape == (61, 2)


def test_detours_around_rect():
    orig = [(float(x), 0.0) for x in np.linspace(0, 6, 31)]
    rect = [(2.0, -0.5), (2.0, 0.5), (4.0, 0.5), (4.0, -0.5)]
    g, ox, oy, res = _flat_sdf()
    fn = make_sdf_fn(g, ox, oy, res)
    p = deform_patch(orig, [rect], fn, (0, 0), (6, 0))
    # must leave the rect: signed check (unsigned edge distance alone
    # cannot distinguish inside from outside — regression test for the
    # incenter-attractor bug).
    from rulepatch.rp.geom import point_in_polygon, count_inside
    assert count_inside(p, rect) == 0
    from topopatch.topo.labels import rule_clearance
    assert rule_clearance(p, rect) > 0.05


def test_bilinear_exact_on_grid_nodes():
    g = np.arange(16, dtype=float).reshape(4, 4)
    v = _bilinear(g, np.array([1.0, 2.0]), np.array([1.0, 3.0]))
    assert abs(v[0] - 5.0) < 1e-9 and abs(v[1] - 14.0) < 1e-9


def test_sdf_fn_outside_is_free():
    g, ox, oy, res = _flat_sdf()
    fn = make_sdf_fn(g, ox, oy, res)
    v, gr = fn(np.array([100.0]), np.array([100.0]))
    assert v[0] == 0.0 and (gr[0] == 0).all()
