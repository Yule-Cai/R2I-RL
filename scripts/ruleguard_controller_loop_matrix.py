#!/usr/bin/env python3
"""Isolated Gazebo controller-loop stress run for one enforcement arm.

The nominal candidate is geometrically compliant.  After the controller begins
tracking, this driver injects the same deterministic angular disturbance after
the controller output.  ``gate`` exposes the legacy admitted path directly;
``gate_guard`` routes it through the independently observed RuleGuard.

Run from a sourced ROS workspace.  The script starts and stops only its own
process groups and writes one JSON result plus per-process logs.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rulesync3d.ruleguard import min_clearance_2d
from rulesync_interfaces.msg import RuleGuardState, RuleSpec
from rulesync_interfaces.srv import CommitRule, ListRules, ProposeRule
from std_msgs.msg import String


SPAWN = (-2.0, -0.5)
GOAL = (2.0, -0.5)
KEEP_OUT = [(-0.8, 0.25), (0.8, 0.25), (0.8, 0.65), (-0.8, 0.65)]
RHO = 0.20
DEFAULT_BIAS_START_X = -1.40
DEFAULT_BIAS_ANGULAR_Z = 0.25
GOAL_TOLERANCE_M = 0.25


class Processes:
    def __init__(self, log_dir, env):
        self.log_dir = Path(log_dir)
        self.env = dict(env)
        self.items = []

    def start(self, name, args, delay=0.0):
        stream = open(self.log_dir / f"{name}.log", "w", encoding="utf-8")
        proc = subprocess.Popen(
            args,
            stdout=stream,
            stderr=subprocess.STDOUT,
            env=self.env,
            start_new_session=True,
            text=True,
        )
        self.items.append((name, proc, stream))
        if delay:
            time.sleep(delay)
        return proc

    def assert_alive(self):
        dead = [(name, proc.returncode) for name, proc, _ in self.items
                if proc.poll() is not None]
        if dead:
            raise RuntimeError(f"process exited unexpectedly: {dead}")

    def close(self):
        for _, proc, _ in reversed(self.items):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 8.0
        for _, proc, _ in self.items:
            try:
                proc.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for _, _, stream in self.items:
            stream.close()


class MatrixProbe(Node):
    def __init__(self, arm, bias_start_x, bias_end_x, bias_angular_z):
        super().__init__(f"ruleguard_controller_loop_{arm}")
        self.arm = arm
        self.bias_start_x = bias_start_x
        self.bias_end_x = bias_end_x
        self.bias_angular_z = bias_angular_z
        self.pose_pub = self.create_publisher(
            PoseStamped, "/ruleguard/pose", 20)
        self.candidate_pub = self.create_publisher(
            PathMsg, "/rulepatch/repaired_path", 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 20)
        self.propose = self.create_client(
            ProposeRule, "/rulesync/propose_rule")
        self.commit = self.create_client(CommitRule, "/rulesync/commit_rule")
        self.list_rules = self.create_client(
            ListRules, "/rulesync/list_rules")

        self.create_subscription(Odometry, "/odom", self._on_odom, 30)
        self.create_subscription(
            TwistStamped, "/cmd_vel_nominal", self._on_nominal_cmd, 30)
        self.create_subscription(
            String, "/rulepatch/gate_status", self._on_gate_status, 20)
        self.create_subscription(
            RuleGuardState, "/rulesync/ruleguard_state",
            self._on_guard_state, 30)
        controller_topic = ("/rulepatch/gated_path" if arm == "gate"
                            else "/rulepatch/executable_path")
        self.create_subscription(
            PathMsg, controller_topic, self._on_controller_path, 20)

        self.map_xy = None
        self.track = []
        self.clearances = []
        self.gate_status = None
        self.guard_state = None
        self.controller_path_seen = False
        self.stop_path_seen = False
        self.started_at = None
        self.bias_at = None
        self.bias_end_at = None
        self.warning_at = None
        self.hold_at = None
        self.hold_xy = None
        self.hold_clearance = None
        self.zero_cmd_at = None
        self.arrived_at = None
        self.finished = False
        self.nominal_linear = 0.0
        self.actual_linear = 0.0
        self.max_actual_linear_after_hold = 0.0

    def _on_odom(self, message):
        x = SPAWN[0] + float(message.pose.pose.position.x)
        y = SPAWN[1] + float(message.pose.pose.position.y)
        self.map_xy = (x, y)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        # Test-only observation bias: controller TF/odometry and evaluation
        # streams retain their original values. Legacy default remains zero.
        pose.pose.position.y = y + float(getattr(self, "monitor_y_offset", 0.0))
        pose.pose.position.z = float(message.pose.pose.position.z)
        pose.pose.orientation = message.pose.pose.orientation
        self.pose_pub.publish(pose)
        if self.started_at is not None:
            now = time.monotonic()
            self.track.append((now, x, y))
            self.clearances.append(min_clearance_2d(
                (x, y), [KEEP_OUT], RHO))
            if (self.arrived_at is None
                    and math.hypot(x - GOAL[0], y - GOAL[1])
                    <= GOAL_TOLERANCE_M):
                self.arrived_at = now

    def _on_nominal_cmd(self, message):
        self.nominal_linear = float(message.twist.linear.x)
        command = TwistStamped()
        command.header = message.header
        command.twist = message.twist
        now = time.monotonic()
        if self.finished:
            command.twist.linear.x = 0.0
            command.twist.angular.z = 0.0
        elif (self.started_at is not None and self.map_xy is not None
              and self.map_xy[0] >= self.bias_start_x
              and (self.bias_end_x is None
                   or self.map_xy[0] <= self.bias_end_x)
              and command.twist.linear.x > 0.02):
            if self.bias_at is None:
                self.bias_at = now
            command.twist.angular.z = self.bias_angular_z
        elif self.bias_at is not None and self.bias_end_at is None:
            self.bias_end_at = now
        self.actual_linear = float(command.twist.linear.x)
        if self.hold_at is not None:
            self.max_actual_linear_after_hold = max(
                self.max_actual_linear_after_hold,
                abs(self.actual_linear))
            if abs(self.actual_linear) <= 0.01 and self.zero_cmd_at is None:
                self.zero_cmd_at = now
        self.cmd_pub.publish(command)

    def _on_gate_status(self, message):
        try:
            self.gate_status = json.loads(message.data)
        except Exception:
            pass

    def _on_guard_state(self, message):
        self.guard_state = message
        now = time.monotonic()
        if message.state == RuleGuardState.WARNING and self.warning_at is None:
            self.warning_at = now
        if (self.started_at is not None
                and message.state == RuleGuardState.HOLD
                and message.reason in ("boundary-violated",
                                       "boundary-violation",
                                       "inside-forbidden")
                and self.hold_at is None):
            self.hold_at = now
            self.hold_xy = self.map_xy
            if self.clearances:
                self.hold_clearance = self.clearances[-1]

    def _on_controller_path(self, message):
        if message.poses:
            self.controller_path_seen = True
        elif self.started_at is not None:
            self.stop_path_seen = True

    def publish_candidate(self):
        path = PathMsg()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x in (-2.0, -1.2, -0.4, 0.4, 1.2, 2.0):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = GOAL[1]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.candidate_pub.publish(path)

    def publish_stop(self):
        command = TwistStamped()
        for _ in range(5):
            self.cmd_pub.publish(command)


def spin_until(node, predicate, timeout, candidate=False):
    deadline = time.monotonic() + timeout
    next_candidate = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if candidate and now >= next_candidate:
            node.publish_candidate()
            next_candidate = now + 0.5
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def call(node, client, request, timeout=35.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError(f"service call timed out: {client.srv_name}")
    return future.result()


def make_rule():
    rule = RuleSpec()
    rule.rule_id = "controller_loop_keepout"
    rule.epoch = 1
    rule.rule_type = RuleSpec.KEEP_OUT
    rule.severity = RuleSpec.ENFORCED
    rule.frame_id = "map"
    rule.polygon.points = [Point32(x=x, y=y) for x, y in KEEP_OUT]
    rule.valid_from = Time(sec=0)
    rule.valid_until = Time(sec=3600)
    return rule


def start_stack(args, log_dir, env, commands=None):
    commands = commands or {}
    sim_dir = Path("/opt/ros/lyrical/share/nav2_minimal_tb3_sim")
    env = dict(env)
    # The models directory resolves model:// URIs; its package-share parent
    # also resolves package://nav2_minimal_tb3_sim/... visual mesh URIs.
    # Set both for the server, not only inside the later spawn launch.
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(value for value in (
        str(sim_dir / "models"), str(sim_dir.parent),
        env.get("GZ_SIM_RESOURCE_PATH", "")) if value)
    processes = Processes(log_dir, env)
    robot_sdf = sim_dir / "urdf/gz_waffle.sdf.xacro"
    if bool(getattr(args, "world_truth", False)):
        from gazebo_world_pose import instrument_robot
        robot_sdf = instrument_robot(robot_sdf, log_dir, env)
    world = Path(log_dir) / "tb3_sandbox.sdf"
    subprocess.run(
        ["xacro", "-o", str(world), "headless:=True",
         str(sim_dir / "worlds/tb3_sandbox.sdf.xacro")],
        check=True,
        env=env,
    )
    # ``-s`` starts only the Gazebo server, so no X display wrapper is needed.
    # Keeping gz as the process-group leader also makes cleanup exact.
    processes.start(
        "gazebo", ["gz", "sim", "-r", "-s", str(world)], delay=8.0)
    processes.start(
        "spawn",
        [
            "ros2", "launch", "nav2_minimal_tb3_sim", "spawn_tb3.launch.py",
            "robot_name:=turtlebot3_waffle",
            f"robot_sdf:={robot_sdf}",
            f"x_pose:={SPAWN[0]}", f"y_pose:={SPAWN[1]}", "z_pose:=0.01",
        ],
        delay=0.5,
    )
    if bool(getattr(args, "world_truth", False)):
        processes.start("world_pose_bridge", [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/rulesync_test/world_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ], delay=0.2)
    robot_description = (sim_dir / "urdf/turtlebot3_waffle.urdf").read_text()
    processes.start(
        "robot_state_publisher",
        [
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args", "-p", "use_sim_time:=true",
            "-p", f"robot_description:={robot_description}",
        ],
        delay=0.5,
    )
    processes.start(
        "map_odom_tf",
        [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--frame-id", "map", "--child-frame-id", "odom",
            "--x", str(SPAWN[0]), "--y", str(SPAWN[1]), "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
        ],
        delay=0.5,
    )

    bridge_env = dict(env)
    bridge_env["RULESYNC_TEST_POSE2D"] = f"{SPAWN[0]},{SPAWN[1]}"
    bridge_env["RULESYNC_TEST_GOAL2D"] = f"{GOAL[0]},{GOAL[1]}"
    original_env = processes.env
    processes.env = bridge_env
    processes.start(
        "bridge",
        [
            "ros2", "run", "rulesync_nav2_bridge", "bridge", "--ros-args",
            "-p", "auto_replan:=false", "-p", "auto_rollback:=false",
        ],
        delay=0.5,
    )
    processes.env = original_env
    processes.start(
        "manager",
        commands.get("manager", ["ros2", "run", "rulesync_manager", "manager"]) + ["--ros-args",
            "-p", f"state_path:={Path(log_dir) / 'state.json'}",
            "-p", f"wal_path:={Path(log_dir) / 'wal.jsonl'}",
            "-p", f"txn_log_path:={Path(log_dir) / 'txn.log'}",
            "-p", "prepare_timeout_s:=12.0",
        ],
        delay=0.5,
    )
    gate_args = [
        "ros2", "run", "topopatch_ros", "gate_node", "--ros-args",
        "-p", f"rho:={args.rho}",
    ]
    if bool(getattr(args, "use_polygon_index", False)):
        gate_args += ["-p", "use_polygon_index:=true"]
    if args.arm == "gate":
        gate_args += ["-p", "publish_legacy_path:=true"]
    processes.start("gate", gate_args, delay=0.3)
    if args.arm == "gate_guard":
        processes.start(
            "guard", commands.get("guard", ["ros2", "run", "rulesync_guard", "ruleguard"]) + [
                "--ros-args", "-p", f"rho:={args.rho}",
            ],
            delay=0.3)
    input_topic = ("/rulepatch/gated_path" if args.arm == "gate"
                   else "/rulepatch/executable_path")
    processes.start(
        "controller",
        [
            "ros2", "run", "rulepatch_ros", "pp_driver", "--ros-args",
            "-r", f"/rulepatch/repaired_path:={input_topic}",
            "-r", "/cmd_vel:=/cmd_vel_nominal",
            "-p", f"speed:={args.speed}",
            "-p", f"lookahead:={args.lookahead}",
            "-p", f"goal_tol:={args.goal_tolerance}",
        ],
        delay=0.3,
    )
    return processes


def run(args):
    global SPAWN, GOAL, KEEP_OUT, RHO, GOAL_TOLERANCE_M
    SPAWN = (float(args.spawn_x), float(args.path_y))
    GOAL = (float(args.goal_x), float(args.path_y))
    KEEP_OUT = [
        (float(args.rule_x_min), float(args.rule_y_min)),
        (float(args.rule_x_max), float(args.rule_y_min)),
        (float(args.rule_x_max), float(args.rule_y_max)),
        (float(args.rule_x_min), float(args.rule_y_max)),
    ]
    RHO = float(args.rho)
    GOAL_TOLERANCE_M = float(args.goal_tolerance)
    log_dir = Path(args.log_root) / f"{args.arm}_{int(time.time())}"
    log_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env["GZ_PARTITION"] = f"rulesync_guard_stage3_{args.domain_id}"
    env.pop("ROS_DISCOVERY_SERVER", None)
    # rclpy.init() reads the current process environment, whereas child
    # processes receive ``env`` explicitly.  Both sides must join the same
    # isolated DDS graph.
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    sim_models = "/opt/ros/lyrical/share/nav2_minimal_tb3_sim/models"
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        value for value in (sim_models, env.get("GZ_SIM_RESOURCE_PATH", ""))
        if value)
    result = {
        "arm": args.arm,
        "domain_id": args.domain_id,
        "gz_partition": env["GZ_PARTITION"],
        "log_dir": str(log_dir),
        "spawn": list(SPAWN),
        "goal": list(GOAL),
        "keep_out": [list(point) for point in KEEP_OUT],
        "rho": RHO,
        "speed": float(args.speed),
        "lookahead": float(args.lookahead),
        "goal_tolerance": GOAL_TOLERANCE_M,
        "use_polygon_index": bool(getattr(args, "use_polygon_index", False)),
        "world_truth_enabled": bool(getattr(args, "world_truth", False)),
        "monitor_y_offset_m": float(getattr(args, "monitor_y_offset", 0.0)),
        "legacy_track_source": "spawn XY plus wheel-integrated Gazebo DiffDrive odometry",
        "result": "FAIL",
    }
    processes = None
    node = None
    try:
        if bool(getattr(args, "world_truth", False)):
            from gazebo_world_pose import write_manifest
            write_manifest(args, log_dir)
        processes = start_stack(args, log_dir, env)
        rclpy.init()
        probe_class = MatrixProbe
        if bool(getattr(args, "world_truth", False)):
            from gazebo_world_pose import make_world_probe
            probe_class = make_world_probe(MatrixProbe)
        node = probe_class(
            args.arm, args.bias_start_x, args.bias_end_x,
            args.bias_angular_z)
        node.monitor_y_offset = float(getattr(args, "monitor_y_offset", 0.0))
        if node.monitor_y_offset and not bool(getattr(args, "world_truth", False)):
            raise RuntimeError("monitor bias requires independent --world-truth capture")
        if not spin_until(node, lambda: node.map_xy is not None, 45.0):
            raise RuntimeError("Gazebo odometry did not arrive")
        if bool(getattr(args, "world_truth", False)):
            if not spin_until(node, lambda: len(node.world_samples) >= 20, 15.0):
                raise RuntimeError("independent Gazebo model pose did not arrive")
            node.verify_stationary_alignment()
        processes.assert_alive()
        for client in (node.list_rules, node.propose, node.commit):
            if not client.wait_for_service(timeout_sec=40.0):
                raise RuntimeError(f"service unavailable: {client.srv_name}")

        def manager_ready():
            try:
                response = call(
                    node, node.list_rules, ListRules.Request(), timeout=3.0)
                return response is not None and not response.degraded
            except Exception:
                return False

        if not spin_until(node, manager_ready, 45.0):
            raise RuntimeError("manager did not converge")
        proposed = call(
            node, node.propose, ProposeRule.Request(rule=make_rule()))
        if proposed is None or not proposed.accepted:
            raise RuntimeError(
                f"proposal rejected: {getattr(proposed, 'reason', None)}")
        committed = call(
            node, node.commit, CommitRule.Request(epoch=1), timeout=45.0)
        if committed is None or not committed.committed:
            raise RuntimeError(
                f"commit rejected: {getattr(committed, 'reason', None)}")

        if not spin_until(
                node, lambda: node.controller_path_seen,
                20.0, candidate=True):
            raise RuntimeError("controller-facing path was not released")
        node.started_at = time.monotonic()

        def terminal_observed():
            if bool(getattr(args, "world_truth", False)):
                node.assert_truth_healthy()
                arrived = node.world_arrived()
            else:
                arrived = node.arrived_at is not None
            if arrived:
                return True
            if args.arm == "gate_guard":
                return (node.hold_at is not None
                        and node.stop_path_seen
                        and node.zero_cmd_at is not None)
            if bool(getattr(args, "world_truth", False)):
                return any(row["active"] and row["clearance_m"] <= -RHO
                           for row in node.world_samples)
            return bool(node.clearances) and min(node.clearances) <= -RHO

        completed = spin_until(node, terminal_observed, args.run_timeout)
        node.finished = True
        node.publish_stop()
        spin_until(node, lambda: False, 1.0)

        min_clearance = min(node.clearances) if node.clearances else math.nan
        max_deviation = max(
            (abs(y - GOAL[1]) for _, _, y in node.track), default=math.nan)
        distance = 0.0
        distance_after_hold = 0.0
        for (_, x0, y0), (_, x1, y1) in zip(node.track, node.track[1:]):
            segment = math.hypot(x1 - x0, y1 - y0)
            distance += segment
        if node.hold_at is not None:
            post_hold = [sample for sample in node.track
                         if sample[0] >= node.hold_at]
            for (_, x0, y0), (_, x1, y1) in zip(
                    post_hold, post_hold[1:]):
                distance_after_hold += math.hypot(x1 - x0, y1 - y0)
        result.update(
            label=args.label,
            bias_start_x=args.bias_start_x,
            bias_end_x=args.bias_end_x,
            bias_angular_z=args.bias_angular_z,
            completed=bool(completed),
            gate_accepted=bool(node.gate_status
                               and node.gate_status.get("execute")),
            controller_path_seen=node.controller_path_seen,
            bias_applied=node.bias_at is not None,
            bias_completed=node.bias_end_at is not None,
            track_samples=len(node.track),
            travel_m=round(distance, 4),
            min_clearance_m=(round(min_clearance, 4)
                             if math.isfinite(min_clearance) else None),
            max_lateral_deviation_m=(round(max_deviation, 4)
                                     if math.isfinite(max_deviation) else None),
            warning_observed=node.warning_at is not None,
            hold_observed=node.hold_at is not None,
            arrived=node.arrived_at is not None,
            entered_original_forbidden=(
                math.isfinite(min_clearance) and min_clearance <= -RHO),
            stop_path_observed=node.stop_path_seen,
            stop_latency_s=(round(node.zero_cmd_at - node.hold_at, 4)
                            if node.zero_cmd_at and node.hold_at else None),
            hold_clearance_m=(round(node.hold_clearance, 4)
                              if node.hold_clearance is not None else None),
            hold_xy=([round(node.hold_xy[0], 4),
                      round(node.hold_xy[1], 4)]
                     if node.hold_xy else None),
            travel_after_hold_m=round(distance_after_hold, 4),
            max_command_after_hold_mps=round(
                node.max_actual_linear_after_hold, 4),
            final_xy=([round(node.map_xy[0], 4), round(node.map_xy[1], 4)]
                      if node.map_xy else None),
        )
        disturbance_observed = (
            abs(args.bias_angular_z) <= 1e-12 or result["bias_applied"])
        if args.arm == "gate":
            passed = (completed and result["gate_accepted"]
                      and disturbance_observed)
        else:
            valid_terminal = (
                result["arrived"]
                or (result["hold_observed"]
                    and result["stop_path_observed"]
                    and result["stop_latency_s"] is not None))
            passed = (completed and result["gate_accepted"]
                      and disturbance_observed and valid_terminal)
        result["result"] = "PASS" if passed else "FAIL"
        if bool(getattr(args, "world_truth", False)):
            result["world_truth"] = node.truth_result()
            if args.arm == "gate_guard":
                valid_terminal = (result["world_truth"]["sampled_arrival"]
                    or (result["hold_observed"] and result["stop_path_observed"]
                        and result["stop_latency_s"] is not None))
                passed = (completed and result["gate_accepted"]
                          and disturbance_observed and valid_terminal)
                result["result"] = "PASS" if passed else "FAIL"
    except Exception as exc:
        result["result"] = "FAIL"
        result["error"] = str(exc)
    finally:
        if node is not None:
            node.finished = True
            node.publish_stop()
            if bool(getattr(args, "world_truth", False)):
                try:
                    node.save_observations(log_dir)
                except Exception as exc:
                    result["result"] = "FAIL"
                    result["capture_save_error"] = str(exc)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if processes is not None:
            processes.close()
        output = log_dir / "result.json"
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["result"] == "PASS" else 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=("gate", "gate_guard"), required=True)
    parser.add_argument("--domain-id", type=int, required=True)
    parser.add_argument(
        "--log-root",
        default=os.environ.get(
            "RULESYNC_LOG_ROOT", "/tmp/rulesync_guard_stage3"))
    parser.add_argument("--run-timeout", type=float, default=25.0)
    parser.add_argument("--label", default="strong")
    parser.add_argument(
        "--bias-start-x", type=float, default=DEFAULT_BIAS_START_X)
    parser.add_argument("--bias-end-x", type=float)
    parser.add_argument(
        "--bias-angular-z", type=float, default=DEFAULT_BIAS_ANGULAR_Z)
    parser.add_argument("--spawn-x", type=float, default=-2.0)
    parser.add_argument("--goal-x", type=float, default=2.0)
    parser.add_argument("--path-y", type=float, default=-0.5)
    parser.add_argument("--rule-x-min", type=float, default=-0.8)
    parser.add_argument("--rule-x-max", type=float, default=0.8)
    parser.add_argument("--rule-y-min", type=float, default=0.25)
    parser.add_argument("--rule-y-max", type=float, default=0.65)
    parser.add_argument("--rho", type=float, default=0.20)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--lookahead", type=float, default=0.50)
    parser.add_argument("--goal-tolerance", type=float, default=0.25)
    parser.add_argument("--use-polygon-index", action="store_true",
                        help="Opt-in snapshot-bound 2-D geometry index; default remains direct")
    parser.add_argument("--world-truth", action="store_true",
                        help="Record independent Gazebo model pose; require valid capture")
    parser.add_argument("--monitor-y-offset", type=float, default=0.0,
                        help="Test-only RuleGuard pose offset (m); requires --world-truth")
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(run(parse_args()))
