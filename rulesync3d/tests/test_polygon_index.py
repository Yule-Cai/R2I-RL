"""Conservative broad-phase equivalence and snapshot-cache lifecycle tests."""

import numpy as np
import pytest
import ast
from pathlib import Path
import time
from types import SimpleNamespace as N

from rulesync3d.rulesync3d.gate2d import (
    PreparedPolygonIndex, SnapshotPolygonIndexCache, gate_full_path,
    path_intersects_polygons,
)

SQUARE = [(0., 0.), (2., 0.), (2., 2.), (0., 2.)]
CONCAVE = [(0., 0.), (2., 0.), (2., 1.), (1., 1.), (1., 2.), (0., 2.)]


@pytest.mark.parametrize('rho', [0., .05, .2, .5])
@pytest.mark.parametrize('path', [
    [(-1., 1.), (3., 1.)],       # both outside; continuous crossing
    [(-1., 2.), (3., 2.)],       # boundary contact
    [(-1., 2.15), (3., 2.15)],   # only inflated geometry may reject
    [(2.1, 2.1), (3., 3.)],     # Euclidean corner, not a box verdict
    [(1.5, 1.5), (1.6, 1.6)],   # outside concavity but inside its AABB
    [(1., 1.), (1., 1.)],       # zero-length segment inside
    [(-1., -1.), (-1., -1.)],   # zero-length segment outside
    [(-1., 2. + 5e-10), (3., 2. + 5e-10)],
])
def test_equivalence_on_boundary_and_concave_cases(rho, path):
    for polygon in [SQUARE, CONCAVE, list(reversed(SQUARE)), SQUARE + SQUARE[:1]]:
        prepared = PreparedPolygonIndex([polygon], rho)
        assert prepared.intersects_path(path) == path_intersects_polygons(
            path, [polygon], inflation_radius=rho)


def test_random_multirule_polyline_equivalence():
    rng = np.random.default_rng(20261010)
    for _ in range(100):
        polygons = [np.asarray(CONCAVE) + rng.uniform(-10, 10, size=2)
                    for _ in range(12)]
        rho = float(rng.choice([0., .05, .2, .5]))
        index = PreparedPolygonIndex(polygons, rho)
        for _ in range(10):
            path = rng.uniform(-12, 12, size=(8, 2))
            assert index.intersects_path(path) == path_intersects_polygons(
                path, polygons, inflation_radius=rho)


def test_bbox_is_only_broad_phase():
    index = PreparedPolygonIndex([CONCAVE])
    stats = {}
    assert not index.intersects_path([(1.5, 1.5), (1.8, 1.8)], stats)
    assert stats['exact_checks'] == 1


@pytest.mark.parametrize('dy', [-5e-13, -9e-13, -1.1e-12, -2e-12, 2e-12, 1e-11])
def test_near_horizontal_legacy_raycast_cannot_be_pruned(dy):
    polygon = [(0., 0.), (2., dy), (2., 2.), (0., 2.)]
    index = PreparedPolygonIndex([polygon])
    for x in [-100., -10., -3., 3., 10., 100.]:
        path = [(x, dy / 2), (x, dy / 2)]
        assert index.intersects_path(path) == path_intersects_polygons(path, [polygon])


def test_singular_legacy_raycast_is_not_silently_authorized():
    polygon = [(0., 0.), (2., -1e-12), (2., 2.), (0., 2.)]
    index = PreparedPolygonIndex([polygon])
    with pytest.raises(ZeroDivisionError):
        index.intersects_path([(-3., -5e-13), (-3., -5e-13)])


def test_empty_index_and_separated_rules_skip_narrow_phase():
    stats = {}
    path = [(-2., -2.), (-1., -1.)]
    assert not PreparedPolygonIndex([]).intersects_path(path, stats)
    assert stats['exact_checks'] == 0
    assert not PreparedPolygonIndex([SQUARE], .2).intersects_path(path, stats)
    assert stats['exact_checks'] == 0


def test_index_owns_geometry_and_margin():
    original = np.asarray(SQUARE)
    index = PreparedPolygonIndex([original], .2)
    original[:] += 100
    assert index.intersects_path([(-1., 1.), (3., 1.)])
    assert not index.polygons[0].flags.writeable
    path = [(-2., 3.), (-1., 3.), (3., 3.), (4., 3.)]
    assert gate_full_path(path, index, lambda x, y: False,
                          (-10, -10, 10, 10), .1) == (
                              False, 'gate:index-margin-mismatch')


@pytest.mark.parametrize('changed', ['epoch', 'hash', 'frame', 'rho'])
def test_verified_snapshot_key_change_rebuilds(changed):
    cache = SnapshotPolygonIndexCache()
    params = dict(epoch=1, snapshot_hash='a' * 64, frame='map',
                  polygons=[SQUARE], inflation_radius=.2)
    index, hit = cache.get(**params)
    assert not hit
    assert cache.get(**params) == (index, True)
    replacements = dict(epoch=('epoch', 2), hash=('snapshot_hash', 'b' * 64),
                        frame=('frame', 'odom'), rho=('inflation_radius', .3))
    field, value = replacements[changed]
    params[field] = value
    rebuilt, hit = cache.get(**params)
    assert not hit and rebuilt is not index


def test_remove_relax_and_return_to_old_identity_do_not_reuse_wrong_geometry():
    cache = SnapshotPolygonIndexCache()
    path = [(-1., 1.), (3., 1.)]
    original, _ = cache.get(1, 'a' * 64, 'map', [SQUARE], .2)
    assert original.intersects_path(path)
    removed, hit = cache.get(2, 'b' * 64, 'map', [], .2)
    assert not hit and not removed.intersects_path(path)
    relaxed, hit = cache.get(3, 'c' * 64, 'map',
                            [np.asarray(SQUARE) + 10], .2)
    assert not hit and not relaxed.intersects_path(path)
    restored, hit = cache.get(1, 'a' * 64, 'map', [SQUARE], .2)
    assert not hit and restored.intersects_path(path)


@pytest.mark.parametrize('rho', [-1, float('nan'), float('inf')])
def test_invalid_margin_is_not_cached(rho):
    cache = SnapshotPolygonIndexCache()
    with pytest.raises(ValueError):
        cache.get(1, 'a' * 64, 'map', [SQUARE], rho)


def test_nonfinite_input_fails_closed():
    bad = np.asarray(SQUARE)
    bad[0, 0] = np.nan
    with pytest.raises(ValueError):
        PreparedPolygonIndex([bad])
    path = [(-2., 3.), (-1., 3.), (3., 3.), (4., np.nan)]
    for rules in [[SQUARE], PreparedPolygonIndex([SQUARE])]:
        assert gate_full_path(path, rules, lambda x, y: False,
                              (-10, -10, 10, 10)) == (False, 'gate:bad-path')


def _callback_fixture(indexed=True):
    """Run actual callback code with duck-typed messages, not a ROS executor.

    This tests guard ordering/atomic outputs only. It is not transport,
    scheduling, or closed-loop ROS validation.
    """
    from rulesync3d.rulesync3d.authorization import path_digest, token_id
    from rulesync3d.rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash
    source = Path(__file__).resolve().parents[2] / 'topopatch_ros/topopatch_ros/gate_node.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'GateNode')
    method = next(n for n in cls.body if getattr(n, 'name', '') == '_on_listed')
    scope = dict(time=time, snapshot_hash=snapshot_hash,
                 rule_snapshot_items=rule_snapshot_items, gate_full_path=gate_full_path,
                 path_digest=path_digest, token_id=token_id, ExecutionAuthorization=N)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    rule = N(rule_id='square', epoch=1, rule_type=0, severity=1, frame_id='map',
             polygon=N(points=[N(x=x, y=y, z=0) for x, y in SQUARE]),
             speed_limit=0., valid_from=None, valid_until=None)
    response = N(active=[rule], epoch=1, degraded=False,
                 active_hash=snapshot_hash(rule_snapshot_items([rule])))
    message = N(header=N(frame_id='map'), poses=[
        N(header=N(frame_id='map'), pose=N(position=N(x=x, y=3., z=0)))
        for x in [-3., -1., 3., 4.]])
    auths, verdicts = [], []
    params = dict(rho=.2, use_polygon_index=indexed, publish_legacy_path=False)
    node = N(_listing=True, _pending_t0=0, _pending=message, _map=None,
             _polygon_index=SnapshotPolygonIndexCache(),
             get_parameter=lambda k: N(value=params[k]),
             auth=N(publish=auths.append),
             _verdict=lambda go, reason, *a, **kw: verdicts.append((go, reason, kw)),
             get_logger=lambda: N(warning=lambda message: None))
    return scope['_on_listed'], node, response, auths, verdicts


@pytest.mark.parametrize('failure', ['degraded', 'hash', 'rule-frame', 'path-frame', 'missing'])
def test_cache_hit_never_bypasses_current_response_checks(failure):
    callback, node, response, auths, verdicts = _callback_fixture()
    callback(node, N(result=lambda: response))
    assert len(auths) == 1 and verdicts[-1][0]
    original = node._polygon_index._index
    if failure == 'degraded':
        response.degraded = True
    elif failure == 'hash':
        response.active_hash = '0' * 64
    elif failure == 'rule-frame':
        from rulesync3d.rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash
        response.active[0].frame_id = 'odom'
        response.active_hash = snapshot_hash(rule_snapshot_items(response.active))
    elif failure == 'path-frame':
        node._pending.poses[1].header.frame_id = 'odom'
    else:
        response = None
    callback(node, N(result=lambda: response))
    assert not verdicts[-1][0] and len(auths) == 1
    assert node._polygon_index._index is original


def test_actual_callback_rechecks_each_new_candidate_on_cache_hit():
    callback, node, response, auths, verdicts = _callback_fixture()
    callback(node, N(result=lambda: response))
    original = node._polygon_index._index
    for pose in node._pending.poses:
        pose.pose.position.y = 1.
    callback(node, N(result=lambda: response))
    assert node._polygon_index._index is original
    assert verdicts[-1][:2] == (False, 'gate:KEEP_OUT-violation')
    assert len(auths) == 1


def test_callback_direct_and_indexed_bind_identical_atomic_authorizations():
    outputs = []
    for indexed in [False, True]:
        callback, node, response, auths, verdicts = _callback_fixture(indexed)
        callback(node, N(result=lambda: response))
        assert verdicts[-1][0]
        a = auths[0]
        outputs.append((a.epoch, a.snapshot_hash, a.path_digest, a.token_id, a.gate_reason))
    assert outputs[0] == outputs[1]


def test_callback_singular_geometry_denies_instead_of_publishing():
    from rulesync3d.rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash
    callback, node, response, auths, verdicts = _callback_fixture()
    polygon = [(0., 0.), (2., -1e-12), (2., 2.), (0., 2.)]
    response.active[0].polygon.points = [N(x=x, y=y, z=0.) for x, y in polygon]
    response.active_hash = snapshot_hash(rule_snapshot_items(response.active))
    for pose in node._pending.poses:
        pose.pose.position.x = -3.
        pose.pose.position.y = -5e-13
    callback(node, N(result=lambda: response))
    assert not auths and verdicts[-1][:2] == (False, 'gate:rules-unknown')
