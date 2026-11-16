"""2D/3D durable-authorization parity tests (§9 cheap gate, no ROS needed).

Pure tests load snapshot_identity.py by file path (same precedent as
test_validator.py) and run on any machine. ROS-gated tests (marked) need
the built workspace + checkout on PYTHONPATH and run on onexplayer:

  A. stale commit certificate rejected
  B. target hash mismatch rejected
  C. uncommitted REMOVE cannot become authoritative
  D. uncommitted SHRINK/RELAX cannot become authoritative
  E. durable committed update survives restart (selection rule; live
     restart proven by ProtoCrash2D C3 + ProtoRestart2D-3 rows)
  F. bridge-state timeout leaves DEGRADED true
  G. same epoch + same IDs + changed geometry is detected as mismatch
"""
import copy
import importlib.util
import sys
import types
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve()
_MGR_DIR = _HERE.parent.parent / "rulesync_manager"
_SI_PATH = _MGR_DIR / "snapshot_identity.py"
_BRIDGE_PATH = (_HERE.parent.parent.parent / "rulesync_nav2_bridge"
               / "rulesync_nav2_bridge" / "bridge_node.py")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_si = _load("rs_snapshot_identity", _SI_PATH)
try:
    # Repository-root PYTHONPATH layout.
    from rulesync3d.rulesync3d import ruleguard as _rg
except ImportError:
    # Colcon source/install layout, where the package itself is on PYTHONPATH.
    from rulesync3d import ruleguard as _rg


class FakeT:
    def __init__(self, sec, ns=0):
        self.sec, self.nanosec = sec, ns


class FakePt:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


def _rule(rule_id="p0", epoch=10, rule_type=0, severity=1, frame="map",
          pts=None, speed=0.0, vf=(100, 0), vu=(4600, 0)):
    r = types.SimpleNamespace()
    r.rule_id, r.epoch, r.rule_type = rule_id, epoch, rule_type
    r.severity, r.frame_id = severity, frame
    pts = pts if pts is not None else [(0, 0), (4, 0), (4, 4), (0, 4)]
    r.polygon = types.SimpleNamespace(
        points=[FakePt(x, y) for x, y in pts])
    r.speed_limit = speed
    r.valid_from, r.valid_until = FakeT(*vf), FakeT(*vu)
    return r


def _H(rule):
    return _si._rule_snapshot_hash([rule])


# ---------- §5 / G: complete 2D snapshot identity ----------

def test_hash_covers_every_2d_field():
    base = _rule()
    h0 = _H(base)
    assert len(h0) == 64
    cases = [
        _rule(rule_id="p1"),
        _rule(epoch=11),
        _rule(rule_type=1, speed=1.0),
        _rule(severity=0),
        _rule(frame="odom"),
        _rule(speed=1.5, rule_type=1),
        _rule(vf=(101, 0)),
        _rule(vu=(4500, 0)),
    ]
    for i, m in enumerate(cases):
        assert _H(m) != h0, f"hash blind to 2D mutation #{i}"


def test_hash_one_vertex_change():
    h0 = _H(_rule())
    moved = _rule(pts=[(0, 0), (4.5, 0), (4, 4), (0, 4)])
    assert _H(moved) != h0


def test_hash_distinguishes_one_float32_ulp_geometry_change():
    # This value is the next float32 above 1.0. Exact canonicalization must
    # not collapse it into the identity of the 1.0 geometry.
    a = _rule(pts=[(0, 0), (1.0, 0), (1, 1), (0, 1)])
    b = _rule(pts=[(0, 0), (1.0000001192092896, 0), (1, 1), (0, 1)])
    assert _H(a) != _H(b)


def test_ruleguard_hash_parity_with_manager():
    vectors = [
        [_rule()],
        [_rule(rule_id="r|quoted", pts=[
            (0, 0), (1.0000001192092896, 0), (1, 1)])],
        [_rule(rule_type=1, speed=0.25, vf=(7, 8), vu=(9, 10))],
    ]
    for rules in vectors:
        assert _si._rule_snapshot_hash(rules) == _rg.snapshot_hash(
            _rg.rule_snapshot_items(rules))


def test_hash_z_only_change():
    a = types.SimpleNamespace(
        rule_id="p0", epoch=10, rule_type=0, severity=1, frame_id="map",
        polygon=types.SimpleNamespace(
            points=[FakePt(0, 0, 0), FakePt(4, 0, 0), FakePt(4, 4, 0)]),
        speed_limit=0.0, valid_from=FakeT(100), valid_until=FakeT(4600))
    b = copy.deepcopy(a)
    b.polygon.points[2] = FakePt(4, 4, 0.5)
    assert _si._rule_snapshot_hash([a]) != _si._rule_snapshot_hash([b])


def test_same_epoch_ids_geometry_mismatch_detected():
    # G: identical epoch + IDs but different geometry MUST be a mismatch.
    before = {"p0": _rule(epoch=10)}
    after = {"p0": _rule(epoch=10, pts=[(0, 0), (2, 0), (2, 2), (0, 2)])}
    hb = _si._rule_snapshot_hash(list(before.values()))
    ha = _si._rule_snapshot_hash(list(after.values()))
    assert hb != ha
    assert _si.recovery_action(10, hb, ["p0"], True, 10, ha, ["p0"]) \
        == "hold-degraded"


# ---------- op classes ----------

def test_rule_op_class_add_remove():
    assert _si._rule_op_class({}, {"p0": _rule()}) == "ADD"
    assert _si._rule_op_class({"p0": _rule()}, {}) == "REMOVE"


def test_rule_op_class_tighten_relax_keepout():
    big = {"p0": _rule(pts=[(0, 0), (4, 0), (4, 4), (0, 4)])}
    small = {"p0": _rule(pts=[(0, 0), (2, 0), (2, 2), (0, 2)])}
    assert _si._rule_op_class(small, big) == "TIGHTEN"
    assert _si._rule_op_class(big, small) == "RELAX"


def test_rule_op_class_speed_limit_direction():
    loose = {"s0": _rule(rule_id="s0", rule_type=1, speed=2.0)}
    tight = {"s0": _rule(rule_id="s0", rule_type=1, speed=0.5)}
    assert _si._rule_op_class(loose, tight) == "TIGHTEN"
    assert _si._rule_op_class(tight, loose) == "RELAX"


def test_rule_op_class_replace_and_mixed():
    a = {"p0": _rule()}
    b = {"p1": _rule(rule_id="p1")}
    assert _si._rule_op_class(a, b) == "REPLACE"
    # severity-only change: same measure, different authority -> REPLACE
    c = {"p0": _rule(severity=0)}
    assert _si._rule_op_class(a, c) == "REPLACE"
    # mixed directions -> REPLACE
    m0 = {"p0": _rule(pts=[(0, 0), (2, 0), (2, 2), (0, 2)]),
          "p1": _rule(rule_id="p1",
                      pts=[(0, 0), (2, 0), (2, 2), (0, 2)])}
    m1 = {"p0": _rule(pts=[(0, 0), (4, 0), (4, 4), (0, 4)]),
          "p1": _rule(rule_id="p1",
                      pts=[(0, 0), (1, 0), (1, 1), (0, 1)])}
    assert _si._rule_op_class(m0, m1) == "REPLACE"


# ---------- A/B: certificate identity ----------

def test_cert_exact_match_accepts():
    assert _si.cert_matches("tx1", 10, "ab12", "tx1", 10, "ab12")


def test_cert_stale_txn_rejected():
    # A: a COMMIT carrying an older prepared txn must fail closed.
    assert not _si.cert_matches("tx1", 10, "ab12", "tx0", 10, "ab12")


def test_cert_hash_mismatch_rejected():
    # B: same txn/epoch but different target content must fail closed.
    assert not _si.cert_matches("tx1", 10, "ab12", "tx1", 10, "zz99")


def test_cert_epoch_mismatch_rejected():
    assert not _si.cert_matches("tx1", 10, "ab12", "tx1", 11, "ab12")


def test_cert_empty_vs_set_rejected():
    assert not _si.cert_matches("tx1", 10, "ab12", "", 10, "")


# ---------- durable selection (E rule) + C/D recovery rule ----------

def _wal_commit(epoch, kind="rule-decision", txn="tx", ids=("p0",)):
    key = ("target_volumes" if kind == "vol-decision" else "target_rules")
    return {"kind": kind, "txn_id": txn, "epoch": epoch,
            "target_hash": "h%d" % epoch, "decision": "COMMIT",
            key: [{"rule_id": i} for i in ids]}


def test_durable_select_no_wal_keeps_snapshot():
    ep, dicts = _si.select_durable_target(10, [{"rule_id": "p0"}], [],
                                          "rule-decision")
    assert (ep, dicts) == (10, [{"rule_id": "p0"}])


def test_durable_select_newer_commit_wins():
    # E: a durable COMMIT newer than the snapshot survives restart.
    ep, dicts = _si.select_durable_target(
        10, [{"rule_id": "p0"}],
        [_wal_commit(12, ids=[])], "rule-decision")
    assert ep == 12 and dicts == []


def test_durable_select_abort_and_old_ignored():
    abort = dict(_wal_commit(12), decision="ABORT")
    ep, _ = _si.select_durable_target(10, [{"rule_id": "p0"}], [abort],
                                      "rule-decision")
    assert ep == 10
    ep2, _ = _si.select_durable_target(10, [{"rule_id": "p0"}],
                                       [_wal_commit(9)], "rule-decision")
    assert ep2 == 10


def test_recovery_converge_only_on_exact_match():
    assert _si.recovery_action(10, "h", ["p0"], True, 10, "h", ["p0"]) \
        == "converge"
    assert _si.recovery_action(10, "h", ["p0"], False, 10, "h", ["p0"]) \
        == "hold-degraded"


def test_uncommitted_remove_cannot_become_authoritative():
    # C: durable snapshot still holds p0; bridge already dropped it
    # (uncommitted REMOVE visible). Recovery must NOT converge/adopt —
    # it must hold (and the manager replays the durable target).
    assert _si.recovery_action(10, "hsnap", ["p0"], True, 10, "hempty",
                               []) == "hold-degraded"


def test_uncommitted_relax_cannot_become_authoritative():
    # D: durable snapshot holds the BIGGER keepout; bridge shows the
    # shrunk (relaxed) geometry with no durable COMMIT. Must hold.
    assert _si.recovery_action(10, "hbig", ["p0"], True, 10, "hsmall",
                               ["p0"]) == "hold-degraded"


# ---------- ROS-gated: bridge parity (A/B predicates + §8 hash) ----------

def _load_bridge():
    pytest.importorskip("rclpy")
    return _load("rs_bridge_node", _BRIDGE_PATH)


def test_bridge_hash_parity_with_manager():
    bn = _load_bridge()
    vecs = [
        [_rule()],
        [_rule(), _rule(rule_id="p1", epoch=12, rule_type=1, speed=1.0,
                        pts=[(1, 1), (3, 1), (3, 3)])],
        [_rule(severity=0, frame="odom", vf=(7, 5), vu=(99, 1))],
    ]
    for v in vecs:
        assert bn.bridge_rule_hash(v) == _si._rule_snapshot_hash(v)


def test_bridge_cert_predicate_matches_manager():
    bn = _load_bridge()
    cases = [("tx1", 10, "ab12", "tx1", 10, "ab12", True),
             ("tx1", 10, "ab12", "tx0", 10, "ab12", False),
             ("tx1", 10, "ab12", "tx1", 10, "zz99", False),
             ("tx1", 10, "ab12", "tx1", 11, "ab12", False)]
    for st, se, sh, rt, re, rh, want in cases:
        assert bn.bridge_cert_ok(st, se, sh, rt, re, rh) == want
        assert _si.cert_matches(st, se, sh, rt, re, rh) == want


# ---------- ROS-gated: F (timeout leaves DEGRADED) + interface fields ----------

def _load_manager():
    pytest.importorskip("rclpy")
    pytest.importorskip("rulesync_interfaces")
    import sys as _sys
    pkg = types.ModuleType("parity_pkg")
    pkg.__path__ = [str(_MGR_DIR)]
    _sys.modules.setdefault("parity_pkg", pkg)
    for sub in ("snapshot_identity", "vote_coordinator"):
        _load(f"parity_pkg.{sub}", _MGR_DIR / f"{sub}.py")
    return _load("parity_pkg.manager_node", _MGR_DIR / "manager_node.py")


def test_watchdog_timeout_leaves_degraded():
    # F: a lost bridge-state response must NOT clear DEGRADED.
    mn = _load_manager()

    class Log:
        def info(self, *a):
            pass

        def warning(self, *a):
            pass

    stub = types.SimpleNamespace(
        _vol_reconciling=False, _reconciling=True, _reconcile_t0=0.0,
        _degraded=False, _bstate_fails=0, committed_epoch=10,
        get_logger=lambda: Log(),
        log_txn=lambda **kw: None,
        # vol-domain watchdog short-circuits on _vol_reconciling=False;
        # stub it so this test isolates the 2D fail-closed branch.
        _vol_reconcile_watchdog=lambda: None,
        _recreate_bstate_client=lambda self=None: None)
    import time as _time
    stub._reconcile_t0 = _time.monotonic() - 30.0
    mn.RuleManager._reconcile_watchdog(stub)
    assert stub._degraded is True
    assert stub._reconciling is False


def test_vote_service_carries_certificate_fields():
    try:
        from rulesync_interfaces.srv import BridgeVote as _BV
    except Exception:
        pytest.skip("needs built interfaces")
    req = _BV.Request()
    req.txn_id = "tx1"
    req.target_hash = "ab12"
    assert (req.txn_id, req.target_hash) == ("tx1", "ab12")


def test_state_services_carry_snapshot_hash():
    try:
        from rulesync_interfaces.srv import BridgeState as _BS
        from rulesync_interfaces.srv import ListRules as _LR
        from rulesync_interfaces.srv import ListVolumes as _LV
    except Exception:
        pytest.skip("needs built interfaces")
    assert _BS.Response().active_hash == ""
    assert _LR.Response().active_hash == ""
    assert _LR.Response().degraded is False
    assert _LV.Response().degraded is False
