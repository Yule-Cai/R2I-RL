"""Unit tests for the M4 universal output gate (no ROS, no torch)."""
import ast
import inspect

import numpy as np

import topopatch.topo.gate as gate_module
from topopatch.topo.gate import (MAX_FALLBACK_REPLANS, gate_full_path,
                                 gate_patch)


def occ_free(x, y):
    return False


def occ_wall(x, y):
    return x > 5.0


BOUNDS = (-20, -20, 20, 20)
RECT = [(0.0, -1.0), (0.0, 1.0), (2.0, 1.0), (2.0, -1.0)]


def test_clean_path_passes():
    path = [(-4.0, 0.0), (-1.0, 0.0), (4.0, 0.0), (6.0, 0.0)]
    # crosses RECT! use a path around it instead
    path = [(-4.0, 2.0), (0.0, 2.0), (4.0, 2.0), (6.0, 2.0)]
    ok, reason = gate_full_path(path, [RECT], occ_free, BOUNDS)
    assert ok and reason == "gate:ok"


def test_rule_crossing_blocked():
    path = [(-4.0, 0.0), (1.0, 0.0), (4.0, 0.0), (6.0, 0.0)]
    ok, reason = gate_full_path(path, [RECT], occ_free, BOUNDS)
    assert not ok and "KEEP_OUT" in reason


def test_collision_blocked():
    far = [(9.0, 9.0), (9.0, 10.0), (10.0, 10.0), (10.0, 9.0)]
    path = [(0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0)]
    ok, reason = gate_full_path(path, [far], occ_wall, BOUNDS)
    assert not ok and reason == "gate:collision"


def test_out_of_bounds_blocked():
    path = [(0.0, 0.0), (4.0, 0.0), (100.0, 0.0), (101.0, 0.0)]
    ok, reason = gate_full_path(path, [RECT], occ_free, BOUNDS)
    assert not ok and "bounds" in reason


def test_short_path_blocked():
    ok, reason = gate_full_path([(0.0, 0.0)], [RECT], occ_free, BOUNDS)
    assert not ok


def test_multi_rule_any_violation_blocks():
    far = [(9.0, 9.0), (9.0, 10.0), (10.0, 10.0), (10.0, 9.0)]
    path = [(-4.0, 0.0), (1.0, 0.0), (4.0, 0.0), (6.0, 0.0)]
    ok, _ = gate_full_path(path, [far, RECT], occ_free, BOUNDS)
    assert not ok
    ok2, _ = gate_full_path(
        [(-4.0, 2.0), (0.0, 2.0), (4.0, 2.0), (6.0, 2.0)],
        [far, RECT], occ_free, BOUNDS)
    assert ok2


def test_patch_gate_mirrors_verifier():
    from rulepatch.rp.verifier import decode_ctrl
    from rulepatch.rp.bev import world_to_norm
    c = (0.0, 0.0)
    orig = [(-2.0, 0.0), (2.0, 0.0)]
    ctrl = world_to_norm(
        np.array([[-2.0, 0.0], [-1.0, 0.0], [0.0, 0.0],
                  [1.0, 0.0], [1.9, 0.0], [2.0, 0.0]]), c)
    dense = decode_ctrl(ctrl, c, A=(-2.0, 0.0), B=(2.0, 0.0))
    ok, reason = gate_patch(dense, (-2.0, 0.0), (2.0, 0.0), orig,
                            [RECT], occ_free, BOUNDS, c)
    assert not ok and reason.startswith("gate:")


def test_bounded_retry_policy_constants():
    assert MAX_FALLBACK_REPLANS == 2


def test_full_path_gate_has_no_training_package_import():
    """The deployed deterministic gate must remain standalone."""
    tree = ast.parse(inspect.getsource(gate_module))
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)
    assert not any(name == "rulepatch" or name.startswith("rulepatch.")
                   for name in imported)


# --- P0 gate audit: continuous segment-level KEEP_OUT checking ---
# A segment can cross a forbidden polygon even when both sampled
# endpoints lie outside it. Point-in-polygon sampling alone misses
# these; the gate must use exact segment-polygon intersection.

THIN_BAR = [(0.0, -1.0), (0.0, 1.0), (0.1, 1.0), (0.1, -1.0)]


def test_thin_bar_crossing_between_samples_blocked():
    # No path sample lies inside THIN_BAR, but the middle segment
    # passes straight through it. Old point-sampling gate passed this.
    path = [(-4.0, 0.0), (-1.0, 0.0), (1.1, 0.0), (4.0, 0.0)]
    from rulepatch.rp.geom import point_in_polygon
    import numpy as _np
    assert not any(
        point_in_polygon(xy, _np.asarray(THIN_BAR)) for xy in path)
    ok, reason = gate_full_path(path, [THIN_BAR], occ_free, BOUNDS)
    assert not ok and "KEEP_OUT" in reason


def test_diagonal_corner_clip_blocked():
    sq = [(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0)]
    path = [(0.0, 0.0), (0.9, 0.9), (2.1, 2.1), (4.0, 4.0)]
    ok, reason = gate_full_path(path, [sq], occ_free, BOUNDS)
    assert not ok and "KEEP_OUT" in reason


def test_segment_grazing_outside_passes():
    # Parallel pass 0.5 m clear above the bar: must NOT be blocked
    # (no false positives from the segment check).
    path = [(-4.0, 1.5), (-1.0, 1.5), (1.1, 1.5), (4.0, 1.5)]
    ok, reason = gate_full_path(path, [THIN_BAR], occ_free, BOUNDS)
    assert ok and reason == "gate:ok"


def test_collinear_overlap_blocked():
    bar = [(1.0, -0.5), (1.0, 0.5), (3.0, 0.5), (3.0, -0.5)]
    path = [(-2.0, 0.0), (0.0, 0.0), (4.0, 0.0), (6.0, 0.0)]
    ok, reason = gate_full_path(path, [bar], occ_free, BOUNDS)
    assert not ok and "KEEP_OUT" in reason


def test_path_intersects_polygons_primitives():
    from topopatch.topo.gate import (path_intersects_polygons,
                                     segment_intersects_polygon,
                                     segments_intersect)
    assert segments_intersect((0, 0), (2, 2), (0, 2), (2, 0))
    assert not segments_intersect((0, 0), (1, 1), (2, 2), (3, 3))
    assert segments_intersect((0, 0), (4, 0), (1, 0), (3, 0))  # collinear
    sq = [(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0)]
    assert segment_intersects_polygon((0, 0), (3, 3), sq)
    assert not segment_intersects_polygon((0, 0), (0.5, 0.5), sq)
    assert segment_intersects_polygon((1.5, 1.5), (5, 5), sq)  # from inside
    assert path_intersects_polygons(
        [(0, 0), (0.5, 0.5), (3, 3)], [sq])
    assert not path_intersects_polygons(
        [(0, 0), (0.5, 0.5), (0.9, 0.9)], [sq])
