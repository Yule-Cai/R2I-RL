# Unit tests for the RuleSync-3D gate and planners (no ROS, no Gazebo).
import math

import numpy as np
import pytest

from rulesync3d.rulesync3d.gate_3d import (
    gate_3d_path,
    path_intersects_volumes,
    segment_intersects_aabb,
    segment_intersects_convex,
    segment_intersects_sphere,
)

BOUNDS = (0, 0, 0, 10, 10, 5)
FREE = lambda x, y, z: False  # noqa: E731
AABB = {"geometry_type": 0, "bmin": np.array([4.9, 4.9, 1.9]),
        "bmax": np.array([5.1, 5.1, 2.1])}


def test_aabb_thin_slab_between_samples_blocked():
    p = [(0, 5, 2), (4, 5, 2), (6, 5, 2), (10, 5, 2)]
    assert not any(
        (lambda q: AABB["bmin"][0] <= q[0] <= AABB["bmax"][0]
         and AABB["bmin"][1] <= q[1] <= AABB["bmax"][1]
         and AABB["bmin"][2] <= q[2] <= AABB["bmax"][2])(q) for q in p)
    ok, rs = gate_3d_path(p, [AABB], FREE, BOUNDS)
    assert not ok and "KEEP_OUT_VOLUME" in rs


def test_aabb_clean_above_passes():
    p = [(0, 5, 4), (4, 5, 4), (6, 5, 4), (10, 5, 4)]
    ok, rs = gate_3d_path(p, [AABB], FREE, BOUNDS)
    assert ok and rs == "gate3d:ok"


def test_aabb_primitives():
    assert segment_intersects_aabb((0, 0, 0), (2, 2, 2),
                                   (1, 1, 1), (1.5, 1.5, 1.5))
    assert not segment_intersects_aabb((0, 0, 0), (0.5, 0.5, 0.5),
                                       (1, 1, 1), (1.5, 1.5, 1.5))
    assert segment_intersects_aabb((0, 1, 1), (3, 1, 1),
                                   (1, 0, 0), (2, 2, 2))  # collinear-ish
    assert not segment_intersects_aabb((0, 3, 3), (3, 3, 3),
                                       (1, 0, 0), (2, 2, 2))


def test_sphere_primitives():
    assert segment_intersects_sphere((0, 0, 0), (4, 0, 0),
                                     (2, 0, 0), 0.5)
    assert not segment_intersects_sphere((0, 2, 0), (4, 2, 0),
                                         (2, 0, 0), 0.5)
    assert segment_intersects_sphere((2, 0, 0), (5, 0, 0),
                                     (2, 0, 0), 0.5)  # from inside


def test_convex_primitives():
    # Box [-1, 2]^3, with inside defined by n.p + off >= 0.
    hs = [((1, 0, 0), 1.0), ((-1, 0, 0), 2.0),
          ((0, 1, 0), 1.0), ((0, -1, 0), 2.0),
          ((0, 0, 1), 1.0), ((0, 0, -1), 2.0)]
    assert segment_intersects_convex((0, 1.5, 1.5), (3, 1.5, 1.5), hs)
    assert not segment_intersects_convex((0, 3, 3), (3, 3, 3), hs)
    assert segment_intersects_convex((1.5, 1.5, 1.5), (5, 5, 5), hs)


UNIT_CUBE_HALFSPACES = [
    ((1, 0, 0), 0), ((-1, 0, 0), 1),
    ((0, 1, 0), 0), ((0, -1, 0), 1),
    ((0, 0, 1), 0), ((0, 0, -1), 1),
]


@pytest.mark.parametrize("p,q,expected", [
    ((-1, 0.9, 0.5), (0.9, 2, 0.5), False),  # misses corner
    ((-1, 0.5, 0.5), (2, 0.5, 0.5), True),   # through body
    ((0.5, 0.5, 0.5), (2, 2, 2), True),      # endpoint inside
    ((0.2, 0.2, 0.2), (0.8, 0.8, 0.8), True),
    ((-1, 0, 0.5), (1, 2, 0.5), True),       # edge tangency
    ((-1, 0, 0), (1, 2, 2), True),           # vertex tangency
    ((-1, 1, 0.5), (2, 1, 0.5), True),       # along face
    ((-1, 2, 0.5), (2, 2, 0.5), False),      # parallel outside
    ((-1, 0.5, 0.5), (0, 0.5, 0.5), True),   # endpoint contact
    ((0.5, 0.5, 0.5), (0.5, 0.5, 0.5), True),
    ((0, 1, 1), (0, 1, 1), True),            # boundary point
    ((-0.1, 1.1, 0.5), (-0.1, 1.1, 0.5), False),
])
def test_convex_shared_interval_and_contacts(p, q, expected):
    for start, end in ((p, q), (q, p)):
        assert segment_intersects_convex(start, end, UNIT_CUBE_HALFSPACES) == expected
        assert segment_intersects_aabb(start, end, (0, 0, 0), (1, 1, 1)) == expected
        # Rotation/translation exercise non-axis-aligned plane normals.
        rotation = np.array([[0.6, -0.8, 0], [0.8, 0.6, 0], [0, 0, 1]])
        shift = np.array([3.0, -2.0, 0.25])
        hs = [(rotation @ n, off - (rotation @ n) @ shift)
              for n, off in UNIT_CUBE_HALFSPACES]
        assert segment_intersects_convex(rotation @ start + shift,
                                         rotation @ end + shift, hs) == expected


def test_convex_oblique_face_and_plane_order():
    # Tetrahedron x,y,z >= 0 and x+y+z <= 1, not a box.
    hs = [((1, 0, 0), 0), ((0, 1, 0), 0), ((0, 0, 1), 0),
          ((-1, -1, -1), 1)]
    for planes in (hs, list(reversed(hs))):
        assert not segment_intersects_convex((-1, 0.8, 0.1), (0.8, 1, 0.1), planes)
        assert segment_intersects_convex((-1, 0.2, 0.2), (1, 0.2, 0.2), planes)
        assert segment_intersects_convex((-1, 0.8, 0.1), (0.8, 1, 0.1),
                                         [(n, off + 2) for n, off in planes])


def test_convex_tolerance_and_coefficient_scaling():
    eps = 1e-9
    assert segment_intersects_convex((-1, 1 + eps / 2, 0.5),
                                     (2, 1 + eps / 2, 0.5), UNIT_CUBE_HALFSPACES)
    assert not segment_intersects_convex((-1, 1 + 2 * eps, 0.5),
                                         (2, 1 + 2 * eps, 0.5), UNIT_CUBE_HALFSPACES)
    for scale in (0.01, 100.0):
        planes = [(np.array(n) * scale, off * scale)
                  for n, off in UNIT_CUBE_HALFSPACES]
        assert not segment_intersects_convex((-1, 0.9, 0.5), (0.9, 2, 0.5),
                                             planes, eps=eps * scale)


def test_convex_gate_accepts_corner_miss_and_blocks_crossing():
    # Translate the regression case into the gate's configured bounds.
    shift = np.array([2.0, 2.0, 1.0])
    volume = {"geometry_type": 2,
              "halfspaces": [(n, off - np.array(n) @ shift)
                             for n, off in UNIT_CUBE_HALFSPACES]}
    clean = np.array([(-1, 0.9, 0.5), (0.9, 2, 0.5)]) + shift
    crossing = np.array([(-1, 0.5, 0.5), (2, 0.5, 0.5)]) + shift
    assert gate_3d_path(clean, [volume], FREE, BOUNDS) == (True, "gate3d:ok")
    ok, reason = gate_3d_path(crossing, [volume], FREE, BOUNDS)
    assert not ok and "KEEP_OUT_VOLUME" in reason


def test_gate_rejects_bad_volume_and_bounds():
    p = [(0, 0, 1), (5, 5, 1), (9, 9, 1)]
    ok, rs = gate_3d_path(p, [{"geometry_type": 99}], FREE, BOUNDS)
    assert not ok and "bad-volume" in rs
    ok, rs = gate_3d_path([(0, 0, 1), (50, 50, 1)], [], FREE, BOUNDS)
    assert not ok and "bounds" in rs


def test_voxel_astar_routes_and_respects_lethal():
    from rulesync3d.rulesync3d.planners_3d import (voxel_astar,
                                                   voxelize_bounds)
    B = (0, 0, 0, 10, 10, 5)
    grid, org = voxelize_bounds([(4.0, 0.0, 0.0, 6.0, 7.0, 5.0)],
                                0.2, B, margin=0.15)
    p = voxel_astar(grid, org, 0.2, (1, 5, 2.5), (9, 5, 2.5))
    assert p is not None and len(p) >= 2
    assert math.dist(p[0], (1, 5, 2.5)) < 0.3
    # sealed world -> None
    grid2, _ = voxelize_bounds([(0.0, 0.0, 0.0, 10.0, 10.0, 5.0)],
                               0.2, B, margin=0.0)
    assert voxel_astar(grid2, org, 0.2, (1, 5, 2.5), (9, 5, 2.5)) is None


def test_shortcut_smooth_collapses_free_line():
    from rulesync3d.rulesync3d.planners_3d import shortcut_smooth
    p = [(float(i), 5.0, 2.5) for i in range(9)]
    s = shortcut_smooth(p, FREE, vols=())
    assert s[0] == p[0] and s[-1] == p[-1]
    assert len(s) == 2
    # kinked raw path around a box: the straight shortcut pierces it,
    # so smoothing must preserve a kink (and stay violation-free)
    A = {"geometry_type": 0, "bmin": np.array([3.9, 4.9, 1.9]),
         "bmax": np.array([4.1, 5.1, 2.1])}
    p3 = [(0.0, 5.0, 2.0), (3.0, 5.0, 2.0), (4.0, 6.0, 2.0),
          (5.0, 5.0, 2.0), (8.0, 5.0, 2.0)]
    assert not path_intersects_volumes(p3, [A])
    assert path_intersects_volumes([p3[0], p3[-1]], [A])
    s2 = shortcut_smooth(p3, FREE, vols=(A,))
    assert len(s2) > 2
    assert not path_intersects_volumes(s2, [A])


def test_smooth_reserve_keeps_clearance():
    # Smoothing against reserve-inflated volumes keeps clearance from
    # the nominal gate boundary (tracking tube): the razor-graze that
    # fits nominal must detour (or hold the kink) under reserve.
    from rulesync3d.rulesync3d.gate_3d import inflate_gate_vols
    from rulesync3d.rulesync3d.planners_3d import shortcut_smooth
    A = {"geometry_type": 0, "bmin": np.array([3.9, 4.9, 1.9]),
         "bmax": np.array([4.1, 5.1, 2.1])}
    R = inflate_gate_vols([A], 0.5)[0]
    assert float(R["bmin"][0]) < 3.9 and float(R["bmax"][0]) > 4.1
    p3 = [(0.0, 5.0, 2.0), (3.0, 5.0, 2.0), (4.0, 6.0, 2.0),
          (5.0, 5.0, 2.0), (8.0, 5.0, 2.0)]
    s_nom = shortcut_smooth(p3, FREE, vols=(A,))
    s_res = shortcut_smooth(p3, FREE, vols=(R,))
    assert not path_intersects_volumes(s_nom, [A])
    assert not path_intersects_volumes(s_res, [A])
    assert not path_intersects_volumes(s_res, [R])


def test_rrt_connect_finds_gap():
    from rulesync3d.rulesync3d.planners_3d import (rrt_connect,
                                                   voxelize_bounds)
    B = (0, 0, 0, 10, 10, 5)
    grid, org = voxelize_bounds([(4.0, 0.0, 0.0, 6.0, 7.0, 5.0)],
                                0.2, B, margin=0.15)
    occ = lambda x, y, z: grid[int((z - org[2]) / 0.2)] \
        [int((y - org[1]) / 0.2)][int((x - org[0]) / 0.2)]  # noqa: E731
    p = rrt_connect((1, 5, 2.5), (9, 5, 2.5), occ, B, seed=0)
    assert p is not None and len(p) >= 2
    assert math.dist(p[0], (1, 5, 2.5)) < 0.6
    assert math.dist(p[-1], (9, 5, 2.5)) < 0.6
