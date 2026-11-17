"""Opt-in, evaluation-only Gazebo model-pose capture for the existing runner.

PosePublisher reads the model's ECM Pose (world parent), not wheel odometry.
Only the top-level model pose is published; the ROS bridge uses a private topic,
NOT /tf. No world-pose value is fed to the controller or production monitor.
"""
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET

TOPIC = "/rulesync_test/world_pose"
MODEL = "turtlebot3_waffle"
WORLD = "default"
MAX_RECEIPT_AGE_S = 0.25
MAX_SIM_GAP_S = 0.10
MAX_MATCH_GAP_S = 0.020


def write_manifest(args, log_dir):
    scripts = Path(__file__).resolve().parent
    sources = {}
    for path in (Path(__file__).resolve(), scripts / "ruleguard_controller_loop_matrix.py",
                 Path("/opt/ros/lyrical/share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro"),
                 Path("/opt/ros/lyrical/share/nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro")):
        sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    (Path(log_dir) / "world_capture_manifest.json").write_text(json.dumps({
        "schema": 1, "pre_run_wall_ns": time.time_ns(), "arguments": vars(args),
        "source_sha256": sources, "requested_world_publish_hz": 50,
        "topic": TOPIC, "expected_parent": WORLD, "expected_child": MODEL,
        "thresholds": {"max_receipt_age_s": MAX_RECEIPT_AGE_S,
                       "max_sim_gap_s": MAX_SIM_GAP_S,
                       "max_match_gap_s": MAX_MATCH_GAP_S,
                       "stationary_xy_error_m": .02, "stationary_yaw_error_rad": .03},
        "interpretation": "PASS means capture and requested terminal were observed, not safety certification",
    }, indent=2, sort_keys=True) + "\n")


def instrument_robot(source, log_dir, env):
    """Generate a run-local SDF; leave the installed model untouched."""
    target = Path(log_dir) / "robot_world_pose.sdf"
    subprocess.run(["xacro", "-o", str(target), str(source)],
                   check=True, env=env)
    tree = ET.parse(target)
    model = tree.getroot().find("model")
    if model is None or model.get("name") != MODEL:
        raise RuntimeError("unexpected robot model in generated SDF")
    # Model origin and base_footprint have the same XY reference in this SDF.
    footprint = model.find("link[@name='base_footprint']")
    if footprint is None or footprint.find("pose") is not None:
        raise RuntimeError("base_footprint reference requires explicit inspection")
    plugin = ET.SubElement(model, "plugin", {
        "filename": "gz-sim-pose-publisher-system",
        "name": "gz::sim::systems::PosePublisher",
    })
    for key, value in {
        "publish_model_pose": "true", "publish_link_pose": "false",
        "publish_nested_model_pose": "false", "publish_visual_pose": "false",
        "publish_collision_pose": "false", "publish_sensor_pose": "false",
        "static_publisher": "false", "use_pose_vector_msg": "true",
        "update_frequency": "50", "topic": TOPIC,
    }.items():
        ET.SubElement(plugin, key).text = value
    tree.write(target, encoding="unicode", xml_declaration=True)
    return target


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_transform(parent, child, sim_ns, xyz, quaternion):
    # Gazebo may represent the root world with an empty scoped name. Do not
    # silently accept that: the pilot must resolve the observed exact name.
    if parent != WORLD or child != MODEL:
        raise ValueError(f"unexpected world/model frames: {parent!r}/{child!r}")
    if sim_ns <= 0 or not all(math.isfinite(v) for v in (*xyz, *quaternion)):
        raise ValueError("invalid model pose or simulation timestamp")
    if abs(sum(v * v for v in quaternion) - 1.0) > 1e-3:
        raise ValueError("model pose quaternion is not normalized")


def matched_errors(world_rows, odom_rows):
    """Nearest sim-stamp comparison, with an explicit admissible match gap."""
    times = [row["sim_ns"] for row in world_rows]
    errors, gaps = [], []
    for row in odom_rows:
        pos = bisect.bisect_left(times, row["sim_ns"])
        choices = [i for i in (pos - 1, pos) if 0 <= i < len(times)]
        if not choices:
            continue
        index = min(choices, key=lambda i: abs(times[i] - row["sim_ns"]))
        gap = abs(times[index] - row["sim_ns"]) / 1e9
        if gap <= MAX_MATCH_GAP_S:
            reference = world_rows[index]
            errors.append(math.hypot(row["x"] - reference["x"],
                                     row["y"] - reference["y"]))
            gaps.append(gap)
    return errors, gaps


def validate_capture_health(rows, errors, now_ns):
    if errors:
        raise RuntimeError(f"world capture failed: {errors[:3]}")
    if not rows:
        raise RuntimeError("world capture is empty")
    age = (now_ns - rows[-1]["receipt_ns"]) / 1e9
    if age < 0 or age > MAX_RECEIPT_AGE_S:
        raise RuntimeError(f"world capture stale/invalid: {age:.3f}s")


def validate_active_sequence(rows):
    if len(rows) < 20:
        raise RuntimeError("world capture sample requirement not met")
    gaps = [(b["sim_ns"] - a["sim_ns"]) / 1e9 for a, b in zip(rows, rows[1:])]
    if min(gaps) <= 0 or max(gaps) > MAX_SIM_GAP_S:
        raise RuntimeError("world capture timestamp/gap contract failed")
    return gaps


def make_world_probe(base_class):
    # Delayed imports let pure capture-validation tests run without ROS.
    from tf2_msgs.msg import TFMessage
    from rulesync3d.ruleguard import min_clearance_2d
    import ruleguard_controller_loop_matrix as base

    class WorldPoseProbe(base_class):
        def __init__(self, *args):
            super().__init__(*args)
            self.world_samples = []
            self.odom_samples = []
            self.guard_events = []
            self.command_events = []
            self.capture_errors = []
            self.alignment_verified = False
            self.create_subscription(TFMessage, TOPIC, self._on_world_pose, 100)

        def _on_world_pose(self, message):
            received = time.monotonic_ns()
            try:
                if len(message.transforms) != 1:
                    raise ValueError("world topic must contain exactly one model pose")
                transform = message.transforms[0]
                position, q = transform.transform.translation, transform.transform.rotation
                sim_ns = stamp_ns(transform.header.stamp)
                validate_transform(transform.header.frame_id, transform.child_frame_id,
                                   sim_ns, (position.x, position.y, position.z),
                                   (q.x, q.y, q.z, q.w))
                if self.world_samples and sim_ns <= self.world_samples[-1]["sim_ns"]:
                    raise ValueError("duplicate or regressing world simulation timestamp")
                self.world_samples.append({
                    "receipt_ns": received, "sim_ns": sim_ns,
                    "frame": transform.header.frame_id,
                    "child": transform.child_frame_id,
                    "x": position.x, "y": position.y, "z": position.z,
                    "quaternion": [q.x, q.y, q.z, q.w],
                    "active": self.started_at is not None,
                    "clearance_m": min_clearance_2d(
                        (position.x, position.y), [base.KEEP_OUT], base.RHO),
                })
            except ValueError as exc:
                self.capture_errors.append(str(exc))

        def _on_odom(self, message):
            received = time.monotonic_ns()
            super()._on_odom(message)
            self.odom_samples.append({
                "receipt_ns": received, "sim_ns": stamp_ns(message.header.stamp),
                "frame": message.header.frame_id, "child": message.child_frame_id,
                "x": self.map_xy[0], "y": self.map_xy[1],
                "active": self.started_at is not None,
                "monitor_input_xy": [self.map_xy[0], self.map_xy[1]
                    + float(getattr(self, "monitor_y_offset", 0.0))],
                "monitor_stamp_mode": "node_clock_at_publication",
            })

        def _on_guard_state(self, message):
            super()._on_guard_state(message)
            world = self.world_samples[-1] if self.world_samples else None
            received = time.monotonic_ns()
            self.guard_events.append({
                "receipt_ns": received, "state": int(message.state),
                "reason": message.reason, "active": self.started_at is not None,
                "authority_epoch": int(message.authority_epoch),
                "authority_hash": message.authority_hash,
                "execution_epoch": int(message.execution_epoch),
                "execution_hash": message.execution_hash,
                "monitor_clearance_m": (float(message.clearance)
                    if math.isfinite(message.clearance) else None),
                "latest_world_sim_ns": world["sim_ns"] if world else None,
                "latest_world_receipt_age_s": (
                    (received - world["receipt_ns"]) / 1e9 if world else None),
                "latest_world_clearance_m": world["clearance_m"] if world else None,
            })

        def _on_nominal_cmd(self, message):
            super()._on_nominal_cmd(message)
            self.command_events.append({
                "receipt_ns": time.monotonic_ns(),
                "linear_x": self.actual_linear,
                "angular_z": float(message.twist.angular.z),
                "active": self.started_at is not None,
            })

        def verify_stationary_alignment(self):
            self.assert_truth_healthy()
            for row in self.world_samples[-20:]:
                if math.hypot(row["x"] - base.SPAWN[0],
                              row["y"] - base.SPAWN[1]) > 0.02:
                    raise RuntimeError("world/map XY spawn alignment failed")
                q = row["quaternion"]
                yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                 1 - 2 * (q[1] ** 2 + q[2] ** 2))
                if abs(yaw) > 0.03:
                    raise RuntimeError("unexpected stationary world yaw")
            if math.hypot(self.map_xy[0] - base.SPAWN[0],
                          self.map_xy[1] - base.SPAWN[1]) > 0.02:
                raise RuntimeError("odom/map XY spawn alignment failed")
            self.alignment_verified = True

        def assert_truth_healthy(self):
            validate_capture_health(self.world_samples, self.capture_errors,
                                    time.monotonic_ns())

        def world_arrived(self):
            return bool(self.world_samples) and math.hypot(
                self.world_samples[-1]["x"] - base.GOAL[0],
                self.world_samples[-1]["y"] - base.GOAL[1]) <= base.GOAL_TOLERANCE_M

        def truth_result(self):
            self.assert_truth_healthy()
            active = [row for row in self.world_samples if row["active"]]
            if not self.alignment_verified:
                raise RuntimeError("world capture alignment requirement not met")
            gaps = validate_active_sequence(active)
            active_odom = [row for row in self.odom_samples if row["active"]]
            errors, match_gaps = matched_errors(active, active_odom)
            if len(errors) < 20:
                raise RuntimeError("insufficient sim-stamp-aligned odometry samples")
            minimum = min(row["clearance_m"] for row in active)
            hold_rows = [row for row in self.guard_events if row["active"]
                         and row["reason"] in (
                             "boundary-violated", "boundary-violation", "inside-forbidden")]
            hold = hold_rows[0] if hold_rows else None
            post = [row for row in active if hold
                    and row["receipt_ns"] >= hold["receipt_ns"]]
            post_distance = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                                for a, b in zip(post, post[1:]))
            return {
                "source": "Gazebo ECM top-level model Pose via PosePublisher",
                "evaluation_only": True, "frame": WORLD, "child": MODEL,
                "map_equals_world_xy": True, "alignment_verified": True,
                "sample_count": len(active), "max_sim_gap_s": max(gaps),
                "median_sim_gap_s": statistics.median(gaps),
                "min_inflated_clearance_m": minimum,
                "sampled_original_region_entry": minimum <= -base.RHO,
                "sampled_arrival": any(math.hypot(row["x"] - base.GOAL[0],
                        row["y"] - base.GOAL[1]) <= base.GOAL_TOLERANCE_M
                        for row in active),
                "first_boundary_hold_latest_sample": hold,
                "post_hold_sampled_travel_m": post_distance if hold else None,
                "odom_world_matched_count": len(errors),
                "odom_world_error_median_m": statistics.median(errors),
                "odom_world_error_max_m": max(errors),
                "odom_world_match_gap_max_s": max(match_gaps),
                "final_xy": [active[-1]["x"], active[-1]["y"]],
                "scope": "sampled simulator model-point geometry, not swept footprint or physical safety",
                "run_start_receipt_ns": int(self.started_at * 1e9),
            }

        def save_observations(self, log_dir):
            for name, rows in (("world_pose", self.world_samples),
                               ("odom_and_monitor_input", self.odom_samples),
                               ("guard_states", self.guard_events),
                               ("commands", self.command_events)):
                with open(Path(log_dir) / f"{name}.jsonl", "w", encoding="utf-8") as stream:
                    for row in rows:
                        stream.write(json.dumps(row, allow_nan=False) + "\n")
            (Path(log_dir) / "capture_validation.json").write_text(json.dumps({
                "alignment_verified": self.alignment_verified,
                "errors": self.capture_errors,
                "world_samples": len(self.world_samples),
                "odom_samples": len(self.odom_samples),
            }, indent=2) + "\n")

    return WorldPoseProbe
