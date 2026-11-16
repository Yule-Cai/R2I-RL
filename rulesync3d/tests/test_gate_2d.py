"""Lean tests for the packaged deterministic 2-D admission gate."""

import ast
import inspect

from rulesync3d.rulesync3d import gate2d


BOUNDS = (-20.0, -20.0, 20.0, 20.0)
SQUARE = [(0.0, -1.0), (0.0, 1.0), (2.0, 1.0), (2.0, -1.0)]


def _free(_x, _y):
    return False


def test_continuous_crossing_is_rejected():
    path = [(-4.0, 0.0), (-1.0, 0.0), (3.0, 0.0), (6.0, 0.0)]
    accepted, reason = gate2d.gate_full_path(
        path, [SQUARE], _free, BOUNDS)
    assert not accepted
    assert reason == "gate:KEEP_OUT-violation"


def test_boundary_contact_is_rejected():
    path = [(-4.0, 1.0), (-1.0, 1.0), (3.0, 1.0), (6.0, 1.0)]
    accepted, reason = gate2d.gate_full_path(
        path, [SQUARE], _free, BOUNDS)
    assert not accepted
    assert reason == "gate:KEEP_OUT-violation"


def test_clear_path_is_accepted():
    path = [(-4.0, 2.0), (-1.0, 2.0), (3.0, 2.0), (6.0, 2.0)]
    assert gate2d.gate_full_path(path, [SQUARE], _free, BOUNDS) == (
        True, "gate:ok")


def test_inflated_boundary_blocks_near_miss_continuously():
    # The path remains 0.15 m above the raw polygon.  It is valid for a
    # point robot, but must be rejected for rho=0.20 m.
    path = [(-4.0, 1.15), (-1.0, 1.15), (3.0, 1.15), (6.0, 1.15)]
    assert gate2d.gate_full_path(path, [SQUARE], _free, BOUNDS) == (
        True, "gate:ok")
    assert gate2d.gate_full_path(
        path, [SQUARE], _free, BOUNDS, inflation_radius=0.20) == (
            False, "gate:KEEP_OUT-violation")


def test_inflated_corner_uses_euclidean_radius():
    # Closest approach to corner (2, 1) is sqrt(0.1^2 + 0.1^2).
    path = [(2.1, 1.1), (3.0, 2.0), (4.0, 3.0), (5.0, 4.0)]
    assert not gate2d.gate_full_path(
        path, [SQUARE], _free, BOUNDS, inflation_radius=0.15)[0]
    assert gate2d.gate_full_path(
        path, [SQUARE], _free, BOUNDS, inflation_radius=0.13)[0]


def test_invalid_inflation_fails_closed():
    path = [(-4.0, 2.0), (-1.0, 2.0), (3.0, 2.0), (6.0, 2.0)]
    assert gate2d.gate_full_path(
        path, [SQUARE], _free, BOUNDS, inflation_radius=-0.1) == (
            False, "gate:bad-inflation")


def test_invalid_geometry_and_collision_fail_closed():
    path = [(-4.0, 2.0), (-1.0, 2.0), (3.0, 2.0), (6.0, 2.0)]
    assert gate2d.gate_full_path(path, [[(0.0, 0.0)]], _free, BOUNDS) == (
        False, "gate:bad-rule")
    assert gate2d.gate_full_path(
        path, [], lambda x, y: x > 5.0, BOUNDS) == (
            False, "gate:collision")


def test_packaged_gate_has_no_research_tree_imports():
    tree = ast.parse(inspect.getsource(gate2d))
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)
    assert not any(
        name == "rulepatch" or name.startswith("rulepatch.")
        or name == "topopatch" or name.startswith("topopatch.")
        for name in imported)
