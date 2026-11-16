"""Offline dry-run of the B1 quadrotor 3D mission logic (no Gazebo, no ROS).

Purpose: validate the *sequencing and geometry* of the DEV runner before/without
a simulator slot:

* initial 3D plan (gate-consistent A*) -> admission -> pure-pursuit advancement
  over the densified waypoint chain until the displacement trigger fires;
* durable mid-run replacement commit -> RuleGuard HOLD on the epoch
  disagreement -> stale-token rejection -> negative control gate rejection of
  the pre-update path;
* recovery 3D replan under the committed snapshot -> re-admission -> pursuit to
  arrival;
* post-hoc clearance audit of the simulated track against the declared KEEP_OUT
  volumes and the dilated occupancy.

This is a deterministic point-model check of the runner's logic, NOT a physics
or flight-dynamics simulation and NOT paper evidence for the smoke test.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from quad3d import authority3d, scene3d, voxel_contract  # noqa: E402
from rulesync3d import ruleguard  # noqa: E402

DT = 0.05
MAX_STEPS = 200000


def load(scene_dir, case_path):
    scene_dir = Path(scene_dir)
    case = json.loads(Path(case_path).read_text())
    map_record = scene3d.load_map(scene_dir / "nav_v1/map.json")
    flight = case["flight"]
    occ, _ = voxel_contract.load_occupancy(
        scene_dir, map_record, flight,
        os.environ.get('RULESYNC_QUAD_OCCUPANCY_MODE'),
        os.environ.get('RULESYNC_QUAD_VOXEL_MANIFEST'))
    bounds = scene3d.planning_bounds(occ, flight["z_plan_min_m"],
                                     flight["z_plan_max_m"])
    bounds_gate = (occ.xmin, occ.ymin, flight["z_floor_m"],
                   occ.xmax, occ.ymax, flight["z_max_m"])
    return case, flight, occ, bounds, bounds_gate


def velocity_towards(current, target, flight, integral):
    gain = float(flight["position_gain"])
    vmax_xy = float(flight["speed_xy_mps"])
    vmax_z = float(flight["speed_z_mps"])
    ex, ey, ez = (target[0] - current[0], target[1] - current[1],
                  target[2] - current[2])
    vz_unsat = gain * ez + 0.8 * integral
    if abs(vz_unsat) < vmax_z:
        integral = max(-0.20, min(0.20, integral + ez * DT))
    vx, vy = gain * ex, gain * ey
    horizontal = math.hypot(vx, vy)
    if horizontal > vmax_xy:
        vx, vy = vx / horizontal * vmax_xy, vy / horizontal * vmax_xy
    vz = max(-vmax_z, min(vmax_z, gain * ez + 0.8 * integral))
    return [vx, vy, vz], integral


def simulate(path, start, flight, arrival=None, trigger=None, spawn=None,
             max_s=400.0):
    """Kinematic point following the runner's pure-pursuit/velocity law.

    Velocity is integrated exactly (perfect velocity tracking): this checks
    waypoint advancement, trigger placement, and arrival sequencing only.
    """
    current = list(start)
    integral = 0.0
    index = min(range(len(path)),
                key=lambda i: math.dist(current, path[i]))
    lookahead = float(flight["lookahead_m"])
    steps = 0
    track = [list(current)]
    trigger_at = None
    while steps < MAX_STEPS and steps * DT < max_s:
        steps += 1
        while (index + 1 < len(path)
               and math.dist(current, path[index + 1]) < lookahead):
            index += 1
        target = path[index]
        velocity, integral = velocity_towards(current, target, flight,
                                              integral)
        current = [current[i] + velocity[i] * DT for i in range(3)]
        track.append(list(current))
        if (trigger is not None and trigger_at is None
                and math.dist(current[:2], spawn[:2])
                >= float(flight.get("trigger", trigger))):
            trigger_at = {"sim_s": steps * DT, "pose": list(current),
                          "index": index}
        if arrival is not None and (
                math.dist(current[:2], arrival[:2])
                <= float(flight["arrival_xy_tolerance_m"])):
            return {"reached": True, "sim_s": steps * DT,
                    "pose": list(current), "track": track,
                    "trigger": trigger_at, "max_index": index,
                    "steps": steps}
    return {"reached": False, "sim_s": steps * DT, "pose": list(current),
            "track": track, "trigger": trigger_at, "max_index": index,
            "steps": steps}


def feasible(occ, bounds, bounds_gate, start, goal, volumes, inflation,
             reserve, res):
    path, planner = scene3d.plan_path(occ, bounds, res, start, goal, volumes,
                                      inflation + reserve)
    if path is None:
        return False, f"no-path-({planner})"
    ok, reason = scene3d.gate_path(path, occ, volumes, inflation, bounds_gate)
    return bool(ok), f"({planner}, {reason})"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, required=True)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    options = parser.parse_args()
    case, flight, occ, bounds, bounds_gate = load(
        options.scene_dir, options.case)
    reserve = 0.05
    inflation = float(flight["body_radius_m"])
    res = float(flight["voxel_res_m"])
    spawn = [float(v) for v in case["spawn_xyz"]]
    goal = [float(v) for v in case["goal_xyz"]]
    trigger = float(case["trigger"]["distance_m"])
    initial_volumes = case["rules"]["initial"]["volumes"]
    replacement_volumes = case["rules"]["replacement"]["volumes"]
    report = {"schema": "rulesync.quad3d.dry_run.v1", "case_id": case["id"],
              "declared_scope": "deterministic point-model logic check; "
                                "not physics and not smoke-test evidence"}
    with tempfile.TemporaryDirectory() as tmp:
        authority = authority3d.DurableVolumeAuthority(Path(tmp) / "auth")
        authority.recover()
        guard = ruleguard.RuleGuard(rho_warn=0.30, rho_release=0.10)
        # 1. initial commit with independent vote
        txn = authority.propose(initial_volumes)
        vote = authority.vote(
            txn["txn_id"],
            lambda volumes, evidence: feasible(
                occ, bounds, bounds_gate,
                [spawn[0], spawn[1], flight["cruise_z_m"]],
                [goal[0], goal[1], flight["cruise_z_m"]],
                volumes, inflation, reserve, res))
        commit1 = authority.commit(txn["txn_id"])
        guard.update_snapshot(commit1["epoch"], commit1["snapshot_hash"])
        report["initial_commit"] = {"vote": vote, "epoch": commit1["epoch"],
                                    "snapshot_hash":
                                        commit1["snapshot_hash"]}
        # 2. initial path + admission
        path, planner, ok, reason = scene3d.initial_flight_path(
            occ, bounds, bounds_gate, spawn, goal, flight["cruise_z_m"],
            flight["z_spawn_m"], initial_volumes, inflation, res=res,
            reserve=reserve)
        report["initial_path"] = {"planner": planner, "gate": reason,
                                  "accepted": ok, "points": len(path),
                                  "max_z_m": max(p[2] for p in path)}
        assert ok, reason
        digest = scene3d.path_digest(path, 1)
        token = authority.issue_token(digest)
        guard.bind_execution(commit1["epoch"], commit1["snapshot_hash"])
        flight["trigger"] = trigger
        first_leg = simulate(path, spawn[:2] + [flight["cruise_z_m"]], flight,
                             trigger=trigger, spawn=spawn)
        report["initial_flight"] = {
            "trigger": first_leg["trigger"],
            "advanced_waypoints": first_leg["max_index"],
            "path_points": len(path),
        }
        assert first_leg["trigger"] is not None, "update trigger never fired"
        assert first_leg["max_index"] > 1, "pure pursuit never advanced"
        # 3. mid-run replacement commit -> monitor HOLD on epoch disagreement
        txn = authority.propose(initial_volumes + replacement_volumes)
        pose_at_update = first_leg["trigger"]["pose"]
        vote2 = authority.vote(
            txn["txn_id"],
            lambda volumes, evidence: feasible(
                occ, bounds, bounds_gate, pose_at_update,
                [goal[0], goal[1], flight["cruise_z_m"]],
                volumes, inflation, reserve, res))
        commit2 = authority.commit(txn["txn_id"])
        guard.update_snapshot(commit2["epoch"], commit2["snapshot_hash"])
        state, info = guard.check_bound_3d(
            pose_at_update, authority.gate_volumes(inflation), 0.0)
        stale_ok, stale_reason, _ = authority.verify_token(
            digest, token, commit2["epoch"], commit2["snapshot_hash"])
        gate_ok, gate_reason = scene3d.gate_path(
            path, occ, authority.volumes, inflation, bounds_gate)
        report["mid_run_update"] = {
            "vote": vote2, "epoch": commit2["epoch"],
            "monitor_state": state, "monitor_info": info,
            "stale_token_accepted": bool(stale_ok),
            "stale_reason": stale_reason,
            "preupdate_path_gate": {"accepted": bool(gate_ok),
                                    "reason": gate_reason},
        }
        assert state == "HOLD" and info["reason"] == "epoch-disagreement"
        assert not stale_ok
        assert not gate_ok
        # 4. recovery replan + readmission + arrival
        recovery, planner2, raw = None, None, None
        current = pose_at_update
        start = [current[0], current[1],
                 max(current[2], flight["z_plan_min_m"])]
        goal_hover = [goal[0], goal[1], flight["cruise_z_m"]]
        raw, planner2 = scene3d.plan_path(
            occ, bounds, res, start, goal_hover, authority.volumes,
            inflation + reserve)
        assert raw is not None, "no recovery path"
        smoothed = scene3d.smooth_path(
            raw, occ, authority.volumes, inflation + reserve,
            bounds=bounds_gate)
        recovery = scene3d.densify(scene3d.dedupe(
            [start] + smoothed[1:]
            + [[goal[0], goal[1], flight["z_spawn_m"]]]))
        ok2, reason2 = scene3d.gate_path(
            recovery, occ, authority.volumes, inflation, bounds_gate)
        report["recovery_path"] = {
            "planner": planner2, "gate": reason2, "accepted": ok2,
            "points": len(recovery),
            "max_z_m": max(p[2] for p in recovery)}
        assert ok2, reason2
        guard.clear_execution()
        assert guard.bind_execution(commit2["epoch"],
                                    commit2["snapshot_hash"])
        state2, info2 = guard.check_bound_3d(
            current, authority.gate_volumes(inflation), 0.0)
        report["readmission_monitor_state"] = state2
        assert state2 != "HOLD", info2
        second_leg = simulate(
            recovery, current, flight,
            arrival=[goal[0], goal[1], flight["cruise_z_m"]])
        report["recovery_flight"] = {
            "reached": second_leg["reached"],
            "sim_s": second_leg["sim_s"],
            "final_pose": second_leg["pose"],
            "goal_xy_error_m": math.dist(second_leg["pose"][:2], goal[:2]),
        }
        assert second_leg["reached"], "recovery flight did not arrive"
        # 5. audit each leg against the snapshot committed at that time
        committed = initial_volumes + replacement_volumes
        leg1 = scene3d.resample(first_leg["track"])
        leg2 = scene3d.resample(second_leg["track"])
        report["audit"] = {
            "epoch1_snapshot_clearance": scene3d.clearance_profile(
                leg1, initial_volumes, inflation),
            "epoch1_inside_future_replacement_zone_samples":
                scene3d.clearance_profile(
                    leg1, replacement_volumes, inflation)[
                        "raw_violation_samples"],
            "epoch2_snapshot_clearance": scene3d.clearance_profile(
                leg2, committed, inflation),
            "physical_penetration_samples":
                scene3d.physical_penetration_profile(leg1 + leg2, occ),
            "planning_margin_breach_samples":
                scene3d.occupancy_profile(leg1 + leg2, occ),
            "track_samples": len(leg1) + len(leg2),
        }
        joined = leg1 + leg2
        penetration = [point for point in joined
                       if occ.physical_fn(*point)]
        report["audit"]["physical_penetration_first_points"] = \
            penetration[:8]
        report["audit"]["planning_margin_first_points"] = [
            point for point in joined if occ.occ_fn(*point)][:8]
        options.out.parent.mkdir(parents=True, exist_ok=True)
        options.out.write_text(json.dumps(report, indent=2) + "\n")
        assert report["audit"]["epoch1_snapshot_clearance"][
            "raw_violation_samples"] == 0
        assert report["audit"]["epoch2_snapshot_clearance"][
            "raw_violation_samples"] == 0
        assert report["audit"]["physical_penetration_samples"] == 0
        options.out.parent.mkdir(parents=True, exist_ok=True)
        options.out.write_text(json.dumps(report, indent=2) + "\n")
        report["passed"] = True
        report["authority_wal_records"] = authority._wal_count
    text = json.dumps(report, indent=2) + "\n"
    options.out.parent.mkdir(parents=True, exist_ok=True)
    options.out.write_text(text)
    print(json.dumps({key: report[key] for key in
                      ("case_id", "passed", "initial_commit",
                       "mid_run_update", "recovery_path",
                       "recovery_flight", "audit")}, indent=2)[:4000])
    raise SystemExit(0 if report.get("passed") else 1)


if __name__ == "__main__":
    main()
