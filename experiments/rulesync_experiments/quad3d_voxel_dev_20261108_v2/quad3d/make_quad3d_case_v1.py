"""Build a 3D quadrotor volume-task case from a prepared 2D recovery case.

The 2D case (``recovery_inputs_v1/<scene>_<route>_replace.json``) supplies the
qualified route, spawn/goal and the two rule polygons.  This generator lifts
each rule polygon to an axis-aligned KEEP_OUT box over a *declared height
band*, then fail-closed-checks the resulting 3D case with the same predicates
the runner uses:

* the extruded 3D initial path passes the 3D gate under the initial snapshot;
* the replacement box (inflated by the body radius) really does invalidate
  that initial path in 3D (otherwise the mid-run HOLD demonstration would be
  vacuous);
* the initial box does not touch the initial path;
* spawn and goal are outside both inflated snapshots;
* the update trigger fires before the vehicle reaches the replacement box.

Usage (runs on the handheld where the prepared scene lives):

  python3 quad3d/make_quad3d_case_v1.py \
      --scene-dir iotj_scene_build/scenes/hssd_106366173_174226431 \
      --route-id route_04 --out quad3d_cases/<case>.json

Exit code 0 only if every qualification check passes; the produced case JSON
records all of them under ``qualification``.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from quad3d import scene3d  # noqa: E402

FLIGHT = {
    "z_floor_m": 0.03,
    "z_max_m": 1.70,
    "z_plan_min_m": 0.45,
    "z_plan_max_m": 1.65,
    "z_spawn_m": 0.12,
    "cruise_z_m": 0.90,
    "body_radius_m": 0.25,
    "occ_clearance_m": 0.30,
    "voxel_res_m": 0.10,
    "speed_xy_mps": 0.20,
    "speed_z_mps": 0.20,
    "command_rate_hz": 20.0,
    "position_gain": 1.5,
    "lookahead_m": 0.25,
    "arrival_xy_tolerance_m": 0.15,
    "arrival_z_tolerance_m": 0.10,
    "hover_hold_s": 1.0,
    "hold_dwell_s": 1.0,
    "guard_rho_warn_m": 0.30,
    "guard_rho_release_m": 0.10,
}
INITIAL_BAND_TOP = 1.65
REPLACEMENT_BAND_TOP = 1.05


def sha256_file(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def path_length(points):
    return float(sum(math.dist(points[i], points[i + 1])
                     for i in range(len(points) - 1)))


def box_center_xy(volume):
    return ((volume["bmin"][0] + volume["bmax"][0]) / 2.0,
            (volume["bmin"][1] + volume["bmax"][1]) / 2.0)


def point_in_inflated(point, volume, inflation):
    return all(volume["bmin"][i] - inflation < point[i]
               < volume["bmax"][i] + inflation for i in range(3))


def polygon_hits_path_3d(path, volume, inflation):
    """Reuse the exact 3D segment-vs-AABB predicate on the initial path."""
    from rulesync3d import gate_3d
    return gate_3d.path_intersects_volumes(
        [[float(point[0]), float(point[1]), float(point[2])]
         for point in path],
        [{"geometry_type": 0,
          "bmin": [float(value) - inflation for value in volume["bmin"]],
          "bmax": [float(value) + inflation for value in volume["bmax"]]}])


def build_case(scene_dir, route_case_path, route_id, replacement_band_top,
               initial_band_top, cruise_z, speed, trigger_distance,
               wall_budget_s):
    scene_dir = Path(scene_dir)
    scene = scene_dir.name
    map_path = scene_dir / "nav_v1/map.json"
    map_record = scene3d.load_map(map_path)
    map_sha = sha256_file(map_path)
    routes_path = scene_dir / "routes_v1.json"
    routes = json.loads(routes_path.read_text())
    if routes["map_sha256"] != map_sha:
        raise SystemExit("routes_v1 map_sha256 does not match nav_v1/map.json")
    case = json.loads(Path(route_case_path).read_text())
    if case["map_sha256"] != map_sha:
        raise SystemExit("2D case map_sha256 does not match nav_v1/map.json")
    if case["scene"] != scene:
        raise SystemExit("2D case scene does not match --scene-dir")
    if route_id is None:
        route_id = case["id"].rsplit("_", 2)[-2]
    route_entry = next((r for r in routes["routes"] if r["id"] == route_id),
                       None)
    if route_entry is None:
        raise SystemExit(f"route {route_id} is not in routes_v1.json")
    flight = dict(FLIGHT)
    flight.update(cruise_z_m=float(cruise_z))
    flight.update(speed_xy_mps=float(speed), speed_z_mps=float(speed))
    inflation = flight["body_radius_m"]
    spawn_xy = [float(v) for v in case["spawn"]]
    goal_xy = [float(v) for v in case["goal"]]
    route_xy = [[float(x), float(y)] for x, y in case["initial_reference_path"]]
    if math.dist(route_xy[0], spawn_xy) > 1e-6:
        raise SystemExit("2D case reference path does not start at spawn")
    if math.dist(route_xy[-1], goal_xy) > 1e-6:
        raise SystemExit("2D case reference path does not end at goal")
    length = path_length(route_xy)
    straight = math.dist(spawn_xy, goal_xy)
    # ``reference_length_m`` in the 2D case is the conservative 4-connected
    # route length; the stored ``initial_reference_path`` is the refined
    # polyline actually flown/qualified.  Keep a geometric sanity bound
    # instead of forcing the two to be equal.
    if not (straight - 1e-6 <= length <= 3.0 * straight + 1.0):
        raise SystemExit(
            f"reference path length {length:.2f} m is inconsistent with "
            f"the {straight:.2f} m spawn-goal distance")
    occ = scene3d.MapOccupancy(map_record, flight["occ_clearance_m"],
                               flight["z_floor_m"], flight["z_max_m"])
    bounds_gate = (occ.xmin, occ.ymin, flight["z_floor_m"],
                   occ.xmax, occ.ymax, flight["z_max_m"])
    bounds = scene3d.planning_bounds(occ, flight["z_plan_min_m"],
                                     flight["z_plan_max_m"])
    initial_volume = scene3d.extrude_polygon(
        case["initial_rule"], flight["z_floor_m"], initial_band_top)
    initial_volume["id"] = "R1_initial"
    replacement_volume = scene3d.extrude_polygon(
        case["replacement_rule"], flight["z_floor_m"],
        float(replacement_band_top))
    replacement_volume["id"] = "R2_replacement"
    path, planner, ok, reason = scene3d.initial_flight_path(
        occ, bounds, bounds_gate, spawn_xy, goal_xy,
        flight["cruise_z_m"], flight["z_spawn_m"], [initial_volume],
        inflation, res=flight["voxel_res_m"])
    # recovery feasibility under the committed initial + replacement snapshot
    recovery_ok, recovery_planner, recovery_path = False, None, None
    if path is not None:
        raw, recovery_planner = scene3d.plan_path(
            occ, bounds, flight["voxel_res_m"],
            [spawn_xy[0], spawn_xy[1], flight["cruise_z_m"]],
            [goal_xy[0], goal_xy[1], flight["cruise_z_m"]],
            [initial_volume, replacement_volume], inflation)
        if raw is not None:
            smoothed = scene3d.smooth_path(
                raw, occ, [initial_volume, replacement_volume], inflation,
                bounds=bounds_gate)
            candidate = scene3d.dedupe(
                [[spawn_xy[0], spawn_xy[1], flight["z_spawn_m"]]] + smoothed
                + [[goal_xy[0], goal_xy[1], flight["z_spawn_m"]]])
            recovery_ok, _ = scene3d.gate_path(
                candidate, occ, [initial_volume, replacement_volume],
                inflation, bounds_gate)
            recovery_path = candidate
    qualification = {
        "initial_path_gate": reason,
        "initial_path_gate_ok": bool(ok),
        "initial_path_planner": planner,
        "initial_path_points": len(path) if path else None,
        "initial_path_length_m": path_length(path) if path else None,
        "initial_path_max_z_m": (max(point[2] for point in path)
                                 if path else None),
        "recovery_path_ok": bool(recovery_ok),
        "recovery_path_planner": recovery_planner,
        "recovery_path_points": (len(recovery_path)
                                 if recovery_path else None),
        "recovery_path_max_z_m": (max(point[2] for point in recovery_path)
                                  if recovery_path else None),
        "replacement_intersects_initial_path": bool(
            polygon_hits_path_3d(path, replacement_volume, inflation)),
        "initial_rule_clear_of_initial_path": bool(
            not polygon_hits_path_3d(path, initial_volume, inflation)),
        "spawn_clear_of_rules": bool(
            not point_in_inflated(
                [spawn_xy[0], spawn_xy[1], flight["cruise_z_m"]],
                initial_volume, inflation)
            and not point_in_inflated(
                [spawn_xy[0], spawn_xy[1], flight["cruise_z_m"]],
                replacement_volume, inflation)),
        "goal_clear_of_rules": bool(
            not point_in_inflated(
                [goal_xy[0], goal_xy[1], flight["cruise_z_m"]],
                initial_volume, inflation)
            and not point_in_inflated(
                [goal_xy[0], goal_xy[1], flight["cruise_z_m"]],
                replacement_volume, inflation)),
        "trigger_before_replacement_m": float(
            math.dist(spawn_xy, box_center_xy(replacement_volume))
            - float(trigger_distance)),
        "replacement_band_top_m": float(replacement_band_top),
        "reference_length_m_2d_case": float(case["reference_length_m"]),
        "four_connected_route_length_m": float(
            route_entry["four_connected_length_m"]),
        "occupancy_clearance_m": flight["occ_clearance_m"],
        "body_radius_m": inflation,
    }
    problems = []
    if not qualification["initial_path_gate_ok"]:
        problems.append(f"initial 3D path rejected: {reason}")
    if not qualification["recovery_path_ok"]:
        problems.append("no admitted recovery path exists under the "
                        "committed initial + replacement snapshot")
    if not qualification["replacement_intersects_initial_path"]:
        problems.append("replacement volume does not invalidate the initial "
                        "3D path (HOLD demonstration would be vacuous)")
    if not qualification["initial_rule_clear_of_initial_path"]:
        problems.append("initial rule touches the initial 3D path")
    if not qualification["spawn_clear_of_rules"]:
        problems.append("spawn is inside an inflated rule volume")
    if not qualification["goal_clear_of_rules"]:
        problems.append("goal is inside an inflated rule volume")
    if qualification["trigger_before_replacement_m"] < 1.0:
        problems.append("replacement volume is too close to spawn for a "
                        "mid-run update")
    if problems:
        raise SystemExit("case rejected: " + "; ".join(problems))
    case_id = (f"{scene}_{route_id}_quad3d_"
               f"r2top{int(round(float(replacement_band_top) * 100)):03d}"
               f"_v1")
    sim_budget = (length / flight["speed_xy_mps"] * 2.5
                  + path_length(path) / flight["speed_xy_mps"]
                  + 120.0)
    record = {
        "schema_version": 1,
        "kind": "rulesync_quad3d_volume_task",
        "id": case_id,
        "scene": scene,
        "map_sha256": map_sha,
        "source_case": {
            "file": Path(route_case_path).name,
            "sha256": sha256_file(route_case_path),
            "schema_version": case["schema_version"],
        },
        "route": {
            "source": "recovery_inputs_v1 initial_reference_path",
            "route_id": route_id,
            "length_m": length,
            "xy_path": route_xy,
        },
        "spawn_xyz": [spawn_xy[0], spawn_xy[1], flight["z_spawn_m"]],
        "goal_xyz": [goal_xy[0], goal_xy[1], flight["z_spawn_m"]],
        "flight": flight,
        "rules": {
            "initial": {
                "id": "R1_initial",
                "declared_band": {"label": "full_declared_flight_band",
                                  "zmin": flight["z_floor_m"],
                                  "zmax": initial_band_top},
                "volumes": [initial_volume],
                "source_polygon_xy": case["initial_rule"],
            },
            "replacement": {
                "id": "R2_replacement",
                "declared_band": {"label": "low_band_partial_height",
                                  "zmin": flight["z_floor_m"],
                                  "zmax": float(replacement_band_top)},
                "volumes": [replacement_volume],
                "source_polygon_xy": case["replacement_rule"],
            },
        },
        "trigger": {"type": "displacement_from_spawn",
                    "distance_m": float(trigger_distance)},
        "timing": {
            "wall_budget_s": float(wall_budget_s),
            "expected_sim_budget_s": float(sim_budget),
        },
        "declared_fidelity": {
            "body": "velocity_level_quadrotor_no_px4_no_aero",
            "occupancy": "conservative_2d_projection_lifted_to_declared_band",
            "scope": "DEV_single_scene_single_route_not_formal_denominator",
        },
        "qualification": qualification,
    }
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, required=True)
    parser.add_argument("--route-case", type=Path, required=True)
    parser.add_argument("--route-id", default=None)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--replacement-band-top", type=float,
                        default=REPLACEMENT_BAND_TOP)
    parser.add_argument("--initial-band-top", type=float,
                        default=INITIAL_BAND_TOP)
    parser.add_argument("--cruise-z", type=float, default=0.90)
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--trigger-distance", type=float, default=0.50)
    parser.add_argument("--wall-budget", type=float, default=600.0)
    options = parser.parse_args()
    record = build_case(
        options.scene_dir, options.route_case, options.route_id,
        options.replacement_band_top, options.initial_band_top,
        options.cruise_z, options.speed, options.trigger_distance,
        options.wall_budget)
    options.out.parent.mkdir(parents=True, exist_ok=True)
    options.out.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"case": str(options.out),
                      "id": record["id"],
                      "route_id": record["route"]["route_id"],
                      "route_length_m": record["route"]["length_m"],
                      "case_sha256": sha256_file(options.out),
                      "qualification": record["qualification"]}, indent=2))


if __name__ == "__main__":
    main()
