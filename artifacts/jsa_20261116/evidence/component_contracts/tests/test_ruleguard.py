# Unit tests for RuleGuard (no ROS, no Gazebo, deterministic).
import math

import numpy as np

from rulesync3d.rulesync3d.ruleguard import (
    CLEAR,
    HOLD,
    WARNING,
    RuleGuard,
    min_clearance_2d,
    min_clearance_3d,
    sd_aabb,
    sd_poly_2d,
    sd_sphere,
    snapshot_hash,
    volume_snapshot_items,
)

AABB = {"geometry_type": 0, "rule_id": "v1", "epoch": 7,
        "bmin": np.array([4.0, 4.0, 1.0]),
        "bmax": np.array([6.0, 6.0, 3.0])}
SPH = {"geometry_type": 1, "rule_id": "v2", "epoch": 7,
       "center": np.array([0.0, 0.0, 2.0]), "radius": 1.0}
SQR = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]

RHO = 0.4  # body margin used across guard tests


def _guard(vols=(AABB,)):
    g = RuleGuard(rho_warn=0.5, rho_release=0.2)
    h = snapshot_hash(volume_snapshot_items(vols))
    g.update_snapshot(7, h)
    return g, h


def test_sd_aabb_exact():
    assert sd_aabb((0, 0, 0), (4, 4, 1), (6, 6, 3)) > 5.0
    assert abs(sd_aabb((7, 5, 2), (4, 4, 1), (6, 6, 3)) - 1.0) < 1e-9
    assert sd_aabb((5, 5, 2), (4, 4, 1), (6, 6, 3)) < 0  # inside


def test_sd_sphere_exact():
    assert abs(sd_sphere((3, 0, 2), (0, 0, 2), 1.0) - 2.0) < 1e-9
    assert sd_sphere((0, 0, 2), (0, 0, 2), 1.0) < 0


def test_sd_poly_2d():
    assert sd_poly_2d((0, 0), SQR) > 5.0
    assert abs(sd_poly_2d((7, 5), SQR) - 1.0) < 1e-9
    assert sd_poly_2d((5, 5), SQR) < 0


def test_guard_clear_warning_hold_3d():
    g, h = _guard()
    st, _ = g.check_3d((0, 0, 2), [AABB], RHO, 7, h)
    assert st == CLEAR
    # sd = 0.6 -> clearance 0.2 in (0, 0.5]: WARNING
    st, info = g.check_3d((3.4, 5, 2), [AABB], RHO, 7, h)
    assert st == WARNING
    assert abs(info["clearance"] - (0.6 - RHO)) < 1e-9
    # sd = 0.3 -> clearance -0.1: HOLD
    st, info = g.check_3d((3.7, 5, 2), [AABB], RHO, 7, h)
    assert st == HOLD


def test_guard_hold_release_hysteresis():
    g, h = _guard()
    st, _ = g.check_3d((5, 5, 2), [AABB], RHO, 7, h)  # deep inside
    assert st == HOLD
    # clearance 0.1: above 0 but below release 0.2 -> stays HOLD
    st, _ = g.check_3d((3.5, 5, 2), [AABB], RHO, 7, h)
    assert st == HOLD
    # clearance 0.3 (> release): first time still HOLD (needs 2 in a row)
    st, _ = g.check_3d((3.3, 5, 2), [AABB], RHO, 7, h)
    assert st == HOLD
    # second consecutive -> WARNING band (0.3 <= 0.5)
    st, _ = g.check_3d((3.3, 5, 2), [AABB], RHO, 7, h)
    assert st == WARNING


def test_guard_epoch_disagreement_hold():
    g, h = _guard()
    st, info = g.check_3d((0, 0, 2), [AABB], RHO, 8, h)  # wrong epoch
    assert st == HOLD and info["degraded"]
    st, info = g.check_3d((0, 0, 2), [AABB], RHO, 7, "deadbeef")
    assert st == HOLD and info["degraded"]
    # consistent again -> CLEAR (fail-closed only during disagreement)
    st, info = g.check_3d((0, 0, 2), [AABB], RHO, 7, h)
    assert st == CLEAR and not info["degraded"]


def test_guard_no_snapshot_hold():
    g = RuleGuard()
    st, info = g.check_3d((0, 0, 2), [AABB], RHO, 7, "x")
    assert st == HOLD and info["degraded"]


def test_bound_execution_requires_explicit_binding():
    g, _ = _guard()
    st, info = g.check_bound_3d((0, 0, 2), [AABB], RHO)
    assert st == HOLD
    assert info["degraded"] and info["reason"] == "execution-not-bound"


def test_bound_execution_agrees_with_monitor_identity():
    g, h = _guard()
    assert g.bind_execution(7, h)
    assert g.execution_identity() == (7, h)
    st, info = g.check_bound_3d((0, 0, 2), [AABB], RHO)
    assert st == CLEAR and not info["degraded"]


def test_mid_execution_authority_change_holds_next_check():
    """An e=7 path cannot continue after authority advances to e=8."""
    g, h7 = _guard()
    assert g.bind_execution(7, h7)
    st, _ = g.check_bound_3d((0, 0, 2), [AABB], RHO)
    assert st == CLEAR

    v8 = dict(AABB)
    v8["epoch"] = 8
    v8["bmin"] = np.array([3.5, 3.5, 1.0])
    h8 = snapshot_hash(volume_snapshot_items([v8]))
    g.update_snapshot(8, h8)

    # Geometry is still far away; HOLD is caused only by the stale execution
    # identity, before another control command may be authorized.
    st, info = g.check_bound_3d((0, 0, 2), [v8], RHO)
    assert st == HOLD and info["degraded"]
    assert info["reason"] == "epoch-disagreement"
    assert info["monitor_epoch"] == 8
    assert info["authority_epoch"] == 7


def test_execution_must_be_reauthorized_after_epoch_change():
    g, h7 = _guard()
    g.bind_execution(7, h7)
    v8 = dict(AABB)
    v8["epoch"] = 8
    h8 = snapshot_hash(volume_snapshot_items([v8]))
    g.update_snapshot(8, h8)
    st, _ = g.check_bound_3d((0, 0, 2), [v8], RHO)
    assert st == HOLD

    assert g.bind_execution(8, h8)
    st, info = g.check_bound_3d((0, 0, 2), [v8], RHO)
    assert st == CLEAR and not info["degraded"]
    g.clear_execution()
    assert g.execution_identity() is None


def test_guard_invalid_geometry_hold():
    g, h = _guard()
    st, info = g.check_3d((0, 0, 2), [{"geometry_type": 99}], RHO, 7, h)
    assert st == HOLD and info["degraded"]


def test_guard_2d_polygon():
    g = RuleGuard(rho_warn=0.5, rho_release=0.2)
    h = snapshot_hash([("p1", 3, 3, ("poly",))])
    g.update_snapshot(3, h)
    st, _ = g.check_2d((0, 0), [SQR], RHO, 3, h)
    assert st == CLEAR
    st, _ = g.check_2d((3.4, 5), [SQR], RHO, 3, h)
    assert st == WARNING
    st, _ = g.check_2d((5, 5), [SQR], RHO, 3, h)
    assert st == HOLD


def test_guard_sphere_multi_volume_min():
    g, h = _guard(vols=(AABB, SPH))
    # near sphere surface: sd small -> HOLD after margin
    st, _ = g.check_3d((0.9, 0, 2), [AABB, SPH], RHO, 7, h)
    assert st == HOLD  # |0.9|-1-0.4 = -0.5
    # hysteresis: first clear above release still HOLD, second releases
    st, _ = g.check_3d((0, 0, 5), [AABB, SPH], RHO, 7, h)
    assert st == HOLD
    st, _ = g.check_3d((0, 0, 5), [AABB, SPH], RHO, 7, h)
    assert st == CLEAR


def test_snapshot_hash_stable():
    h1 = snapshot_hash(volume_snapshot_items([AABB, SPH]))
    h2 = snapshot_hash(volume_snapshot_items([SPH, AABB]))  # order-free
    assert h1 == h2 and len(h1) == 64
    h3 = snapshot_hash(volume_snapshot_items([AABB]))
    assert h3 != h1


def test_snapshot_hash_distinguishes_sub_micro_geometry_change():
    a = dict(AABB, bmax=np.array([1.0, 2.0, 3.0]))
    b = dict(AABB, bmax=np.array([1.0000001192092896, 2.0, 3.0]))
    assert snapshot_hash(volume_snapshot_items([a])) != snapshot_hash(
        volume_snapshot_items([b]))


def test_invalidate_snapshot_preserves_execution_and_forces_hold():
    guard = RuleGuard()
    guard.update_snapshot(4, "h4")
    guard.bind_execution(4, "h4")
    guard.invalidate_snapshot()
    assert guard.execution_identity() == (4, "h4")
    state, info = guard.check_bound(1.0)
    assert state == HOLD
    assert info["reason"] == "monitor-no-snapshot"


def _h1(v):
    from rulesync3d.rulesync3d.ruleguard import canon_volume
    return snapshot_hash([canon_volume(v)])


def test_hash_covers_every_volume_field():
    import copy
    base = {"geometry_type": 0, "rule_id": "v", "epoch": 9,
            "rule_type": 0, "severity": 1, "frame_id": "map",
            "center": [1.0, 2.0, 3.0], "dims": [2.0, 2.0, 2.0],
            "vertices": [], "valid_from": [5, 0], "valid_until": [6, 0]}
    h0 = _h1(base)
    muts = []
    for fld, val in [("rule_id", "w"), ("epoch", 10), ("rule_type", 1),
                     ("severity", 2), ("frame_id", "odom"),
                     ("geometry_type", 1),
                     ("center", [1.0, 2.0, 3.1]),
                     ("dims", [2.0, 2.0, 2.1]),
                     ("valid_from", [5, 1]), ("valid_until", [7, 0])]:
        m = dict(base)
        m[fld] = val
        muts.append((fld, _h1(m)))
    for fld, h in muts:
        assert h != h0, f"hash blind to volume field {fld}"


def test_hash_covers_convex_vertices():
    v0 = {"geometry_type": 2, "rule_id": "c", "epoch": 1,
          "rule_type": 0, "severity": 1, "frame_id": "map",
          "center": [0, 0, 0], "dims": [0, 0, 0],
          "vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
          "valid_from": [0, 0], "valid_until": [9, 0]}
    import copy
    v1 = copy.deepcopy(v0)
    v1["vertices"][3] = [0, 0, 1.5]  # move ONE vertex
    assert _h1(v0) != _h1(v1)


def test_hash_cross_schema_convergence():
    # msg-style dict vs INFLATED gate dict of the same volume hash equal
    # (all parties must converge on one identity per snapshot state).
    msg = {"geometry_type": 0, "rule_id": "v", "epoch": 9,
           "rule_type": 0, "severity": 1, "frame_id": "map",
           "center": [10.0, 10.0, 1.5], "dims": [3.8, 3.8, 2.8],
           "vertices": [], "valid_from": [5, 0], "valid_until": [6, 0]}
    gate = {"geometry_type": 0, "rule_id": "v", "epoch": 9,
            "rule_type": 0, "severity": 1, "frame_id": "map",
            "bmin": np.array([8.1, 8.1, 0.1]),
            "bmax": np.array([11.9, 11.9, 2.9]),
            "valid_from": [5, 0], "valid_until": [6, 0]}
    assert _h1(msg) == _h1(gate)
    sph_m = {"geometry_type": 1, "rule_id": "s", "epoch": 9,
             "rule_type": 0, "severity": 1, "frame_id": "map",
             "center": [1.0, 2.0, 3.0], "dims": [1.6, 0, 0],
             "vertices": [], "valid_from": [5, 0], "valid_until": [6, 0]}
    sph_g = {"geometry_type": 1, "rule_id": "s", "epoch": 9,
             "rule_type": 0, "severity": 1, "frame_id": "map",
             "center": np.array([1.0, 2.0, 3.0]), "radius": 1.6,
             "valid_from": [5, 0], "valid_until": [6, 0]}
    assert _h1(sph_m) == _h1(sph_g)


def test_hash_covers_every_2d_field():
    from rulesync3d.rulesync3d.ruleguard import rule_snapshot_items

    def _h(r):
        return snapshot_hash(rule_snapshot_items([r]))

    base = {"rule_id": "p", "epoch": 4, "rule_type": 0, "severity": 1,
            "frame_id": "map",
            "polygon": [[0, 0], [2, 0], [2, 2], [0, 2]],
            "speed_limit": 1.5, "valid_from": [1, 0],
            "valid_until": [9, 0]}
    h0 = _h(base)
    import copy
    cases = [("rule_id", "q"), ("epoch", 5), ("rule_type", 1),
             ("severity", 2), ("frame_id", "odom"),
             ("speed_limit", 2.5), ("valid_from", [1, 1]),
             ("valid_until", [8, 0])]
    for fld, val in cases:
        m = copy.deepcopy(base)
        m[fld] = val
        assert _h(m) != h0, f"hash blind to 2D field {fld}"
    m = copy.deepcopy(base)
    m["polygon"][2] = [2.5, 2.0]  # move ONE vertex
    assert _h(m) != h0, "hash blind to polygon vertex"


def test_convex_fallback_conservative():
    # unit cube as vertices: bbox fallback must never report MORE
    # clearance than the true box for axis-aligned probes.
    from rulesync3d.rulesync3d.ruleguard import min_clearance_3d as mc
    C = {"geometry_type": 2, "rule_id": "c", "epoch": 1,
         "vertices": [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1),
                      (1, 1, 0), (1, 0, 1), (0, 1, 1), (1, 1, 1)]}
    B = {"geometry_type": 0, "rule_id": "b", "epoch": 1,
         "bmin": np.array([0.0, 0.0, 0.0]),
         "bmax": np.array([1.0, 1.0, 1.0])}
    for p in [(2, 0.5, 0.5), (-1, 0.5, 0.5), (0.5, 0.5, 0.5)]:
        assert mc(p, [C], 0.0) <= mc(p, [B], 0.0) + 1e-9
