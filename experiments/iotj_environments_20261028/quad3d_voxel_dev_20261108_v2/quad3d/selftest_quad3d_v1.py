"""Offline self-test for the B1 quadrotor 3D DEV package (no ROS, no Gazebo).

Checks the pieces the smoke test depends on, using the same pure-Python
predicates the runner imports from the installed ``rulesync3d`` runtime:

  * durable PREPARE/VOTE/COMMIT + WAL replay + fail-closed vote refusal;
  * content-addressed execution identity (epoch/snapshot/path digest);
  * RuleGuard HOLD on an authority epoch change during execution and
    CLEAR after re-admission bound to the new identity;
  * polygon extrusion, conservative occupancy dilation, exact 3D gate
    acceptance/rejection;
  * a partial-height KEEP_OUT volume is over-flown by the 3D planner while
    the straight through-path is rejected (the 3D-specific behaviour);
  * optional structural validation of a produced 3D case JSON.

Run from the experiment directory with the source-tree ``rulesync3d`` package
on PYTHONPATH (the launcher pins the installed runtime to the same sha256):

  PYTHONPATH=.:../rulesync3d python3 quad3d/selftest_quad3d_v1.py \
      --output <evidence.json>
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from quad3d import authority3d, scene3d  # noqa: E402
from rulesync3d import gate_3d, ruleguard  # noqa: E402

FLIGHT = {
    "z_floor_m": 0.03, "z_max_m": 1.70, "z_plan_min_m": 0.45,
    "z_plan_max_m": 1.65, "z_spawn_m": 0.12, "cruise_z_m": 0.90,
    "body_radius_m": 0.25, "occ_clearance_m": 0.30, "voxel_res_m": 0.10,
}
CASES = []


def check(name):
    def decorator(function):
        CASES.append((name, function))
        return function
    return decorator


def flat_map(size_cells=160, resolution=0.05):
    return {"width": size_cells, "height": size_cells,
            "resolution": resolution, "origin": [0.0, 0.0],
            "data": [0] * (size_cells * size_cells)}


def box(bmin, bmax, name="V"):
    return {"id": name, "geometry_type": 0, "bmin": list(bmin),
            "bmax": list(bmax)}


@check("authority_durable_commit_and_replay")
def test_authority(tmp):
    root = Path(tmp) / "authority"
    auth = authority3d.DurableVolumeAuthority(root)
    genesis = auth.recover()
    assert genesis["genesis_written"] and auth.epoch == 0
    r1 = box((1.0, 1.0, 0.0), (2.0, 2.0, 1.65), "R1")
    prepared = auth.propose([r1])
    vote = auth.vote(prepared["txn_id"],
                     lambda volumes, evidence: (True, "synthetic-ok"))
    assert vote["passed"]
    first = auth.commit(prepared["txn_id"])
    assert first["epoch"] == 1 and len(first["snapshot_hash"]) == 64
    r2 = box((1.5, 1.5, 0.0), (2.5, 2.5, 1.05), "R2")
    prepared = auth.propose([r1, r2])
    auth.vote(prepared["txn_id"],
              lambda volumes, evidence: (True, "synthetic-ok"))
    second = auth.commit(prepared["txn_id"])
    assert second["epoch"] == 2
    replay_root = Path(tmp) / "replay"
    replay_root.mkdir(exist_ok=True)
    (replay_root / "wal.jsonl").write_text((root / "wal.jsonl").read_text())
    replayed = authority3d.DurableVolumeAuthority(replay_root)
    result = replayed.recover()
    assert result["replayed_epoch"] == 2
    assert result["replayed_snapshot_hash"] == second["snapshot_hash"]
    assert len(result["commits"]) == 2
    return {"epoch": result["replayed_epoch"],
            "snapshot_hash": result["replayed_snapshot_hash"],
            "commits": len(result["commits"]),
            "wal_records": result["records"]}


@check("authority_fail_closed_vote")
def test_fail_closed(tmp):
    auth = authority3d.DurableVolumeAuthority(Path(tmp) / "failclosed")
    auth.recover()
    prepared = auth.propose([box((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))])
    auth.vote(prepared["txn_id"],
              lambda volumes, evidence: (False, "synthetic-no-path"))
    refused = False
    try:
        auth.commit(prepared["txn_id"])
    except authority3d.CommitRefused:
        refused = True
    assert refused, "commit was not refused after a failed vote"
    assert auth.epoch == 0, "epoch advanced after a refused commit"
    operations = [json.loads(line)["op"]
                  for line in (Path(tmp) / "failclosed/wal.jsonl")
                  .read_text().splitlines()]
    assert operations[-1] == "ABORT", operations
    return {"refused": refused, "epoch": auth.epoch,
            "operations": operations}


@check("execution_identity_tokens")
def test_identity(tmp):
    auth = authority3d.DurableVolumeAuthority(Path(tmp) / "identity")
    auth.recover()
    prepared = auth.propose([box((1.0, 1.0, 0.0), (2.0, 2.0, 1.65))])
    auth.vote(prepared["txn_id"],
              lambda volumes, evidence: (True, "ok"))
    record = auth.commit(prepared["txn_id"])
    digest = hashlib.sha256(b"synthetic-path").hexdigest()
    token = auth.issue_token(digest)
    ok, reason, _ = auth.verify_token(
        digest, token, record["epoch"], record["snapshot_hash"])
    assert ok and reason == "ok"
    stale, stale_reason, _ = auth.verify_token(
        digest, token, record["epoch"] + 1, record["snapshot_hash"])
    assert not stale and stale_reason == "authorization-token-mismatch"
    other, other_reason, _ = auth.verify_token(
        hashlib.sha256(b"other-path").hexdigest(), token, record["epoch"],
        record["snapshot_hash"])
    assert not other and other_reason == "authorization-token-mismatch"
    return {"token": token, "stale_reason": stale_reason,
            "other_reason": other_reason}


@check("guard_hold_on_epoch_disagreement")
def test_guard(tmp):
    volumes = [box((1.0, 1.0, 0.0), (2.0, 2.0, 1.65))]
    gate_volumes = scene3d.inflate_volumes(volumes, 0.25)
    guard = ruleguard.RuleGuard(rho_warn=0.30, rho_release=0.10)
    hash1 = ruleguard.snapshot_hash(
        ruleguard.volume_snapshot_items(volumes))
    guard.update_snapshot(1, hash1)
    guard.bind_execution(1, hash1)
    state, info = guard.check_bound_3d([-1.0, -1.0, 0.9], gate_volumes, 0.0)
    assert state in ("CLEAR", "WARNING"), (state, info)
    volumes2 = volumes + [box((3.0, 3.0, 0.0), (4.0, 4.0, 1.05))]
    hash2 = ruleguard.snapshot_hash(
        ruleguard.volume_snapshot_items(volumes2))
    guard.update_snapshot(2, hash2)
    state, info = guard.check_bound_3d(
        [-1.0, -1.0, 0.9], scene3d.inflate_volumes(volumes2, 0.25), 0.0)
    assert state == "HOLD" and info["reason"] == "epoch-disagreement", (
        state, info)
    guard.clear_execution()
    assert guard.bind_execution(2, hash2)
    state, info = guard.check_bound_3d(
        [-1.0, -1.0, 0.9], scene3d.inflate_volumes(volumes2, 0.25), 0.0)
    assert state in ("CLEAR", "WARNING"), (state, info)
    return {"hold_reason": "epoch-disagreement", "post_readmission": state}


@check("extrusion_and_exact_gate")
def test_gate(tmp):
    map_record = flat_map()
    occ = scene3d.MapOccupancy(map_record, FLIGHT["occ_clearance_m"],
                               FLIGHT["z_floor_m"], FLIGHT["z_max_m"])
    polygon = [[2.0, 2.0], [2.6, 2.0], [2.6, 2.6], [2.0, 2.6]]
    volume = scene3d.extrude_polygon(polygon, 0.0, 1.05)
    assert volume["bmin"] == [2.0, 2.0, 0.0]
    assert volume["bmax"] == [2.6, 2.6, 1.05]
    through = [[1.0, 2.3, 0.9], [3.5, 2.3, 0.9]]
    bounds = (0.0, 0.0, FLIGHT["z_floor_m"], 8.0, 8.0, FLIGHT["z_max_m"])
    ok, through_reason = scene3d.gate_path(
        through, occ, [volume], 0.25, bounds)
    assert not ok and through_reason == "gate3d:KEEP_OUT_VOLUME-violation", (
        ok, through_reason)
    # above the inflated band: 1.05 + 0.25 = 1.30 -> 1.40 is admissible
    above = [[1.0, 2.3, 1.40], [3.5, 2.3, 1.40]]
    ok, above_reason = scene3d.gate_path(
        above, occ, [volume], 0.25, bounds)
    assert ok and above_reason == "gate3d:ok", (ok, above_reason)
    return {"through": through_reason, "above": above_reason}


@check("occupancy_dilation_conservative")
def test_occupancy(tmp):
    size = 40
    data = [0] * (size * size)
    data[20 * size + 20] = 100  # one occupied 5 cm cell at (1.0, 1.0)
    record = {"width": size, "height": size, "resolution": 0.05,
              "origin": [0.0, 0.0], "data": data}
    occ = scene3d.MapOccupancy(record, 0.30, 0.03, 1.70)
    assert occ.occ_fn(1.02, 1.02, 0.9), "occupied cell not blocked"
    assert not occ.occ_fn(1.0, 1.45, 0.9), "free 0.45 m away is blocked"
    assert occ.occ_fn(1.0, 1.25, 0.9), "dilation radius not applied"
    assert occ.occ_fn(-0.1, 1.0, 0.9), "outside the map must be blocked"
    assert occ.occ_fn(1.0, 1.0, 2.0), "above the declared band must be blocked"
    return {"dilation_checked": True}


@check("overflight_partial_height_volume")
def test_overflight(tmp):
    map_record = flat_map()
    occ = scene3d.MapOccupancy(map_record, FLIGHT["occ_clearance_m"],
                               FLIGHT["z_floor_m"], FLIGHT["z_max_m"])
    bounds = scene3d.planning_bounds(occ, FLIGHT["z_plan_min_m"],
                                     FLIGHT["z_plan_max_m"])
    bounds_gate = (occ.xmin, occ.ymin, FLIGHT["z_floor_m"],
                   occ.xmax, occ.ymax, FLIGHT["z_max_m"])
    volume = box((3.0, 2.5, 0.0), (4.0, 3.5, 1.05), "R2")
    start = [1.0, 3.0, 0.9]
    goal = [6.0, 3.0, 0.9]
    path, planner = scene3d.plan_path(
        occ, bounds, FLIGHT["voxel_res_m"], start, goal, [volume], 0.25)
    assert path is not None, "no 3D path found around/over the volume"
    assert planner == "gate_consistent_astar_26conn"
    smoothed = scene3d.smooth_path(path, occ, [volume], 0.25)
    ok, reason = scene3d.gate_path(smoothed, occ, [volume], 0.25,
                                   bounds_gate)
    assert ok, reason
    max_z = max(point[2] for point in smoothed)
    assert max_z >= 1.30, f"path did not over-fly the low band (max z {max_z})"
    straight = [start, goal]
    ok, straight_reason = scene3d.gate_path(
        straight, occ, [volume], 0.25, bounds_gate)
    assert not ok and straight_reason == "gate3d:KEEP_OUT_VOLUME-violation"
    repeat, _ = scene3d.plan_path(
        occ, bounds, FLIGHT["voxel_res_m"], start, goal, [volume], 0.25)
    assert repeat == path, "3D planner is not deterministic"
    return {"planner": planner, "raw_points": len(path),
            "smoothed_points": len(smoothed),
            "max_z_m": max_z, "straight_reason": straight_reason}


@check("case_json_structure")
def test_case_structure(tmp):
    case_path = Path(tmp) / "case.json"
    if not case_path.exists():
        return {"skipped": "no case supplied"}
    case = json.loads(case_path.read_text())
    required = {"schema_version", "kind", "id", "scene", "map_sha256",
                "source_case", "route", "spawn_xyz", "goal_xyz", "flight",
                "rules", "trigger", "timing", "declared_fidelity",
                "qualification"}
    missing = required - set(case)
    assert not missing, missing
    assert case["schema_version"] == 1
    assert case["kind"] == "rulesync_quad3d_volume_task"
    assert case["flight"]["speed_xy_mps"] in (0.1, 0.2, 0.25)
    for key in ("initial", "replacement"):
        rule = case["rules"][key]
        assert rule["volumes"]
        for volume in rule["volumes"]:
            assert volume["geometry_type"] == 0
            assert len(volume["bmin"]) == 3 and len(volume["bmax"]) == 3
            assert all(volume["bmax"][i] > volume["bmin"][i] for i in range(3))
    assert case["qualification"]["replacement_intersects_initial_path"]
    assert case["qualification"]["initial_rule_clear_of_initial_path"]
    return {"case_id": case["id"],
            "replacement_band_top_m": case["qualification"]
            ["replacement_band_top_m"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--case", type=Path)
    options = parser.parse_args()
    results = []
    passed = True
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        if options.case:
            (tmp / "case.json").write_text(Path(options.case).read_text())
        for name, function in CASES:
            started = time.monotonic()
            try:
                detail = function(tmp) or {}
                results.append({"test": name, "passed": True,
                                "wall_s": time.monotonic() - started,
                                "detail": detail})
            except Exception as exc:  # noqa: BLE001 - evidence, not control
                passed = False
                results.append({"test": name, "passed": False,
                                "wall_s": time.monotonic() - started,
                                "error": repr(exc)})
            print(("PASS " if results[-1]["passed"] else "FAIL ") + name,
                  flush=True)
    payload = {
        "schema": "rulesync.quad3d.selftest.v1",
        "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        "python": sys.version.split()[0],
        "rulesync3d_modules": {
            "gate_3d_sha256": _sha(gate_3d.__file__),
            "planner_sha256": _sha(scene3d.planners_3d.__file__),
            "ruleguard_sha256": _sha(ruleguard.__file__),
        },
        "quad3d_modules": {
            "authority3d_sha256": _sha(authority3d.__file__),
            "scene3d_sha256": _sha(scene3d.__file__),
        },
        "tests": results,
        "passed": passed,
    }
    text = json.dumps(payload, indent=2) + "\n"
    if options.output:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(text)
    print(text)
    raise SystemExit(0 if passed else 1)


def _sha(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


if __name__ == "__main__":
    main()
