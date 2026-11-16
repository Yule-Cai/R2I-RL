"""Actual manager callback bodies with controlled transport; no ROS runtime.

These are correctness tests, not simulated navigation trials. Storage uses a
real temporary WAL; only participant transport and ROS messages are replaced.
"""
import ast
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as N, MethodType

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "rulesync_manager"


def load(name):
    spec = importlib.util.spec_from_file_location(name, SOURCE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


va, si = load("volume_authority"), load("snapshot_identity")
H1, H2 = "1" * 64, "2" * 64


def test_snapshot_is_immutable_and_readers_do_not_share_payload():
    cache = va.VolumeAuthority()
    rows = [dict(rule_id="a", center=[1, 2, 3])]
    view = cache.install_durable(1, H1, rows)
    rows[0]["center"][0] = 9
    view.volumes()[0]["center"][1] = 9
    assert cache.read().volumes()[0]["center"] == [1, 2, 3]
    assert view.ready is False
    with pytest.raises(AttributeError):
        view.ready = True


def test_only_exact_current_ack_releases_and_duplicate_does_not_revoke():
    cache = va.VolumeAuthority()
    cache.install_durable(1, H1, [])
    assert cache.mark_converged(1, H1, True)
    assert cache.install_durable(1, H1, []).ready
    cache.install_durable(2, H2, [])
    assert not cache.read().ready
    assert not cache.mark_converged(1, H1, True)
    assert not cache.mark_converged(2, H1, True)
    assert cache.mark_converged(2, H2, True)
    cache.invalidate()
    assert not cache.read().available and not cache.read().ready
    assert not cache.mark_converged(2, H2, True)


@pytest.mark.parametrize("epoch,digest,rows", [(0, H2, []), (1, H2, []),
    (1, H1, [dict(other=1)]), (-1, H1, []), (2, "invalid", []),
    (2, H2, [dict(x=float("nan"))])])
def test_invalid_install_never_changes_view(epoch, digest, rows):
    cache = va.VolumeAuthority()
    old = cache.install_durable(1, H1, [])
    with pytest.raises(ValueError):
        cache.install_durable(epoch, digest, rows)
    assert cache.read() == old


def test_concurrent_readers_never_observe_torn_identity_geometry():
    cache = va.VolumeAuthority()
    cache.install_durable(0, H1, [dict(epoch=0)])

    def write():
        for epoch in range(1, 501):
            cache.install_durable(epoch, H1, [dict(epoch=epoch)])
            cache.mark_converged(epoch, H1, True)

    def read():
        for _ in range(2000):
            view = cache.read()
            assert view.volumes()[0]["epoch"] == view.epoch

    with ThreadPoolExecutor(max_workers=4) as pool:
        for future in [pool.submit(write), *[pool.submit(read) for _ in range(3)]]:
            future.result()


def volume(epoch, dims=(2., 2., 2.)):
    return N(rule_id="a", epoch=epoch, rule_type=0, severity=1, frame_id="map",
             geometry_type=0, center=N(x=3., y=3., z=3.),
             dimensions=N(x=dims[0], y=dims[1], z=dims[2]), vertices=[],
             valid_from=N(sec=0, nanosec=0), valid_until=N(sec=100, nanosec=0))


def dict_volume(row):
    v = volume(row["epoch"], row["dimensions"])
    v.center = N(**dict(zip(("x", "y", "z"), row["center"])))
    v.rule_id = row["rule_id"]
    return v


def manager(tmp_path, monkeypatch, transport="ack", crash="", op="RELAX"):
    tree = ast.parse((SOURCE / "manager_node.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RuleManager")
    names = {"_wal_write", "_on_durable_decision", "_restore_volume_authority",
             "_volume_authority_not_ready", "_volume_authority_ready",
             "_durable_vol_target", "on_commit_volume", "on_remove_volume",
             "_vol_begin_txn", "_vol_to_dict"}
    scope = dict(time=time, json=json, READY=0, **{name: getattr(si, name) for name in
                 ("_vol_snapshot_hash", "_vol_op_class")})
    exec(compile(ast.Module(body=[n for n in cls.body if getattr(n, "name", "") in names],
                           type_ignores=[]), "actual_manager_callbacks", "exec"), scope)
    monkeypatch.setitem(sys.modules, "rulesync_interfaces.msg", N(RuleVolume=N))
    node = N(_volume_authority=va.VolumeAuthority(), _vol_degraded=False,
             active_vol={"a": volume(1)}, committed_epoch=1, _vol_txn_n=0,
             staged_vol={"a": volume(2, (1., 1., 1.))}, staged_vol_epoch=2,
             notifications=[], calls=[])
    for name in names:
        setattr(node, name, MethodType(scope[name], node))
    node.get_parameter = lambda name: N(value=str(tmp_path / "wal.jsonl") if name == "wal_path" else 15.)
    node._dict_to_vol = dict_volume
    node.get_logger = lambda: N(warning=lambda message: None)
    node._publish_volume_authority = lambda: node.notifications.append(node._volume_authority.read())
    node.log_txn = lambda **kw: None
    node._vol_gates_open = lambda: not node._vol_degraded
    node._save_state = lambda: None
    node._wal_read = lambda: [json.loads(s) for s in (tmp_path / "wal.jsonl").read_text().splitlines()] if (tmp_path / "wal.jsonl").exists() else []
    node._restore_volume_authority()
    node._volume_authority_ready(1, si._vol_snapshot_hash(list(node.active_vol.values())))
    node.notifications.clear()

    def round_call(vols, epoch, operation, timeout, tx, digest):
        node.calls.append(operation)
        if operation == "commit":
            # Durable authority is visible, but neither READY nor applied.
            view = node._volume_authority.read()
            assert view.available and view.epoch == 2 and not view.ready
            assert node._vol_degraded and node.committed_epoch == 1
            assert not node._vol_gates_open()
            assert view.snapshot_hash == digest
            assert si._vol_snapshot_hash(vols) == digest
            return {} if transport == "lost-ack" else {"bridge": 0}
        return {"bridge": 1 if transport == "reject" else 0}

    node._volume_round = round_call

    def crash_at(point, *args):
        if point == crash:
            raise RuntimeError("injected crash")

    node._vol_crash_hook = crash_at
    callback = node.on_remove_volume if op == "REMOVE" else node.on_commit_volume
    request = N(epoch=2, rule_id="a")
    return node, lambda: callback(request, N())


@pytest.mark.parametrize("op", ["RELAX", "REMOVE"])
@pytest.mark.parametrize("transport", ["ack", "lost-ack", "reject"])
def test_transaction_publish_order_and_ack_readiness(tmp_path, monkeypatch, op, transport):
    node, execute = manager(tmp_path, monkeypatch, transport=transport, op=op)
    old = node._volume_authority.read()
    result = execute()
    view = node._volume_authority.read()
    if transport == "reject":
        assert view == old and not node.notifications
        assert "commit" not in node.calls
        return
    assert view.epoch == 2 and view.ready == (transport == "ack")
    assert node.notifications[0].epoch == 2 and not node.notifications[0].ready
    assert view.volumes() == ([] if op == "REMOVE" else [node._vol_to_dict(node.staged_vol.get("a", volume(2, (1., 1., 1.))))])
    assert node._vol_gates_open() == (transport == "ack")


@pytest.mark.parametrize("op", ["RELAX", "REMOVE"])
@pytest.mark.parametrize("crash", ["pre-wal", "post-wal", "post-dispatch"])
def test_retained_wal_recovery_does_not_depend_on_notice_delivery(tmp_path, monkeypatch, op, crash):
    node, execute = manager(tmp_path, monkeypatch, crash=crash, op=op)
    with pytest.raises(RuntimeError, match="injected crash"):
        execute()
    node.notifications.clear()  # Assume every notification was lost.
    node._volume_authority = va.VolumeAuthority()
    node._restore_volume_authority()
    recovered = node._volume_authority.read()
    assert recovered.available and recovered.epoch == (1 if crash == "pre-wal" else 2)
    assert not recovered.ready  # Restart does not infer an ACK from authority.
    assert not node._volume_authority.mark_converged(99, H2, True)


def test_fsync_failure_cannot_publish(tmp_path, monkeypatch):
    node, execute = manager(tmp_path, monkeypatch)
    old = node._volume_authority.read()
    monkeypatch.setattr("os.fsync", lambda fd: (_ for _ in ()).throw(OSError("fsync failed")))
    with pytest.raises(OSError):
        execute()
    assert node._volume_authority.read() == old and not node.notifications
    assert "commit" not in node.calls


def test_corrupt_retained_target_is_unknown_not_silently_empty(tmp_path, monkeypatch):
    node, execute = manager(tmp_path, monkeypatch)
    execute()
    rows = node._wal_read()
    rows[0]["target_hash"] = H2
    node._wal_read = lambda: rows
    node._volume_authority = va.VolumeAuthority()
    node._restore_volume_authority()
    assert not node._volume_authority.read().available
