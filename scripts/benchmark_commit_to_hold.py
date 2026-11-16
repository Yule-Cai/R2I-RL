#!/usr/bin/env python3
"""Commit-to-HOLD instrumentation on the existing ROS/Gazebo controller stack.

Production algorithms are inherited, not reimplemented. Wrapper subclasses
timestamp WAL return (after fsync), snapshot callback entry and HOLD decision.
A test-only asynchronous ListRules relay injects response delay/loss without
blocking the monitor executor or changing transaction traffic. All timestamps
use the same host's monotonic clock. Loss has no new-snapshot observation;
it must stop through freshness/request timeout instead.

Run after sourcing the ROS workspace. Outputs include the pre-run manifest,
append-only per-process event logs, individual trial records and a summary.
"""
import argparse
import hashlib
import json
import math
import os
import platform
from pathlib import Path
import sys
import time

CONDITIONS = {
    "normal_200ms": (0.20, "normal"),
    "normal_50ms": (0.05, "normal"),
    "response_delay_300ms": (0.20, "delay"),
    "response_loss": (0.20, "loss"),
}


def emit(event, **fields):
    row = {"event": event, "t_ns": time.monotonic_ns(), **fields}
    with open(os.environ["RULESYNC_TIMELINE_LOG"], "a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, allow_nan=False) + "\n")


def events(path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:  # A concurrent final append can be partial.
            continue
    return rows


def run_role(role, ros_args):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=ros_args)
    if role == "manager":
        from rulesync_manager.manager_node import RuleManager

        class TimedManager(RuleManager):
            def _wal_write(self, record):
                result = super()._wal_write(record)
                if record.get("decision") == "COMMIT":
                    emit("durable_commit", epoch=int(record["epoch"]))
                return result

        node = TimedManager()
    elif role == "guard":
        from rulesync_guard.guard_node import RuleGuardNode, HOLD

        class TimedGuard(RuleGuardNode):
            def _on_snapshot(self, future, sequence):
                entered = time.monotonic_ns()
                current = sequence == self._snapshot_pending_seq
                super()._on_snapshot(future, sequence)
                if current and self._guard.epoch is not None:
                    emit("snapshot_observed", callback_entry_ns=entered,
                         epoch=int(self._guard.epoch))

            def _set_state(self, state, reason, degraded, clearance):
                if state == HOLD and (self._state != HOLD or self._reason != reason):
                    execution = self._guard.execution_identity()
                    emit("hold_decision", reason=reason,
                         authority_epoch=int(self._guard.epoch or 0),
                         execution_epoch=int(execution[0]) if execution else None)
                return super()._set_state(state, reason, degraded, clearance)

        node = TimedGuard()
    else:
        from rclpy.node import Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.task import Future
        from rulesync_interfaces.srv import ListRules
        from std_srvs.srv import SetBool

        class SnapshotRelay(Node):
            def __init__(self):
                super().__init__("snapshot_fault_relay")
                self.enabled = False
                self.group = ReentrantCallbackGroup()
                self.waiters = []
                self.client = self.create_client(ListRules, "/rulesync/list_rules",
                                                 callback_group=self.group)
                self.create_service(ListRules, "/benchmark/list_rules", self.query,
                                    callback_group=self.group)
                self.create_service(SetBool, "/benchmark/fault", self.control,
                                    callback_group=self.group)
                self.create_timer(0.005, self.release, callback_group=self.group)

            def control(self, request, response):
                self.enabled = request.data
                # Release withheld responses only after the measured trial.
                if not self.enabled:
                    for _, pending in self.waiters:
                        if not pending.done():
                            pending.set_result(True)
                    self.waiters.clear()
                response.success = True
                return response

            def release(self):
                now = time.monotonic()
                for deadline, pending in list(self.waiters):
                    if deadline <= now and not pending.done():
                        pending.set_result(True)
                        self.waiters.remove((deadline, pending))

            async def query(self, request, response):
                result = await self.client.call_async(request)
                mode = os.environ["RULESYNC_TIMELINE_MODE"]
                if self.enabled and mode != "normal":
                    pending = Future()
                    deadline = time.monotonic() + 0.30 if mode == "delay" else math.inf
                    self.waiters.append((deadline, pending))
                    await pending
                return result

        node = SnapshotRelay()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def run_condition(args, name, index):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMsg
    from rulesync_interfaces.msg import RuleGuardState
    from rulesync_interfaces.srv import ListRules, ProposeRule, CommitRule
    from std_srvs.srv import SetBool
    import ruleguard_controller_loop_matrix as base

    poll, mode = CONDITIONS[name]
    folder = args.output / name
    folder.mkdir()
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain_start + index),
               GZ_PARTITION=f"rulesync_timeline_{args.domain_start + index}",
               RULESYNC_TIMELINE_MODE=mode,
               RULESYNC_TIMELINE_LOG=str(folder / "events.jsonl"))
    env.pop("ROS_DISCOVERY_SERVER", None)
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(filter(None, [
        "/opt/ros/lyrical/share/nav2_minimal_tb3_sim/models",
        env.get("GZ_SIM_RESOURCE_PATH", "")]))
    os.environ.update(env)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)

    class Probe(base.MatrixProbe):
        def __init__(self):
            super().__init__("gate_guard", math.inf, None, 0.0)
            self.fault = self.create_client(SetBool, "/benchmark/fault")
            self.target_epoch = None
            self.hold_ns = None
            self.zero_ns = None
            self.stop_ns = None
            self.stop_since = None
            self.moving_since = None
            self.speed_seen = 0.0
            self.trial_start_ns = None

        def _on_guard_state(self, message):
            self.guard_state = message
            if (self.trial_start_ns is not None and self.hold_ns is None
                    and message.state == RuleGuardState.HOLD):
                self.hold_ns = time.monotonic_ns()
                emit("hold_received", reason=message.reason, target_epoch=self.target_epoch)

        def _on_nominal_cmd(self, message):
            super()._on_nominal_cmd(message)
            if (self.hold_ns is not None and self.zero_ns is None
                    and abs(message.twist.linear.x) <= 0.01
                    and abs(message.twist.angular.z) <= 0.01):
                self.zero_ns = time.monotonic_ns()
                emit("zero_command_received", target_epoch=self.target_epoch)

        def _on_odom(self, message):
            super()._on_odom(message)
            now = time.monotonic_ns()
            speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
            self.speed_seen = speed
            if speed > 0.05:
                if self.moving_since is None:
                    self.moving_since = now
            else:
                self.moving_since = None
            if self.zero_ns is not None and speed <= 0.01 and abs(message.twist.twist.angular.z) <= 0.01:
                if self.stop_since is None:
                    self.stop_since = now
                if now - self.stop_since >= 200_000_000 and self.stop_ns is None:
                    self.stop_ns = self.stop_since
                    emit("odometry_stopped", stopped_since_ns=self.stop_ns,
                         target_epoch=self.target_epoch)
            else:
                self.stop_since = None

        def publish_candidate(self):
            if self.map_xy is None:
                return
            path = PathMsg()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            x0, y0 = self.map_xy
            for k in range(9):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x0 + (2.0 - x0) * k / 8
                pose.pose.position.y = y0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.candidate_pub.publish(path)

    stack_args = argparse.Namespace(arm="gate_guard", rho=0.20, speed=0.20,
                                   lookahead=0.40, goal_tolerance=0.15)
    command = [sys.executable, str(Path(__file__).resolve())]
    manager = command + ["--role", "manager"]
    guard = command + ["--role", "guard", "--ros-args", "-p",
                       "manager_service:=/benchmark/list_rules", "-p",
                       f"snapshot_poll_s:={poll}"]
    processes = None
    probe = None
    rows = []
    try:
        processes = base.start_stack(stack_args, folder, env,
                                     commands={"manager": manager, "guard": guard})
        processes.start("snapshot_relay", command + ["--role", "relay"])
        rclpy.init()
        probe = Probe()
        if not base.spin_until(probe, lambda: probe.map_xy is not None, 45):
            raise RuntimeError("no Gazebo odometry")
        for client in (probe.list_rules, probe.propose, probe.commit, probe.fault):
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError(f"missing service {client.srv_name}")

        def commit(epoch):
            rule = base.make_rule()
            rule.epoch = epoch
            # Geometrically identical, far-away rule isolates identity revocation.
            for point in rule.polygon.points:
                point.y += 5.0
            result = base.call(probe, probe.propose, ProposeRule.Request(rule=rule))
            if not result.accepted:
                raise RuntimeError(f"proposal {epoch}: {result.reason}")
            result = base.call(probe, probe.commit, CommitRule.Request(epoch=epoch))
            if not result.committed:
                raise RuntimeError(f"commit {epoch}: {result.reason}")

        # Let manager recovery complete before the initial transaction.
        ready = False
        for _ in range(225):
            if not base.call(probe, probe.list_rules, ListRules.Request()).degraded:
                ready = True
                break
            base.spin_until(probe, lambda: False, 0.2)
        if not ready:
            raise RuntimeError("manager remained degraded")
        commit(1)
        for repetition in range(args.repetitions):
            epoch = repetition + 2
            probe.trial_start_ns = None
            probe.hold_ns = probe.zero_ns = probe.stop_ns = probe.stop_since = None
            probe.moving_since = None
            probe.controller_path_seen = False
            base.call(probe, probe.fault, SetBool.Request(data=False))
            base.spin_until(probe, lambda: False, 1.25)
            if not base.spin_until(probe, lambda: probe.controller_path_seen,
                                   8, candidate=True):
                raise RuntimeError("fresh authorization did not reach controller")
            if not base.spin_until(probe, lambda: probe.moving_since is not None
                                   and time.monotonic_ns() - probe.moving_since > 300_000_000, 8):
                raise RuntimeError("robot not moving before rule update")
            # Fixed phase offsets, declared in the manifest; never selected by result.
            base.spin_until(probe, lambda: False, (repetition % 4) * 0.037)
            probe.target_epoch = epoch
            probe.trial_start_ns = time.monotonic_ns()
            base.call(probe, probe.fault, SetBool.Request(data=True))
            commit(epoch)
            if not base.spin_until(probe, lambda: probe.stop_ns is not None, 8):
                raise RuntimeError("no sustained odometry stop after update")
            processes.assert_alive()
            raw = [row for row in events(folder / "events.jsonl")
                   if row["t_ns"] >= probe.trial_start_ns]
            durable = next(row["t_ns"] for row in raw
                           if row["event"] == "durable_commit" and row["epoch"] == epoch)
            hold = next(row for row in raw if row["event"] == "hold_decision"
                        and row["t_ns"] >= durable)
            observed = next((row["callback_entry_ns"] for row in raw
                             if row["event"] == "snapshot_observed" and row["epoch"] == epoch
                             and row["callback_entry_ns"] <= hold["t_ns"]), None)
            if not durable <= hold["t_ns"] <= probe.zero_ns <= probe.stop_ns:
                raise RuntimeError("invalid measured event ordering")
            if mode == "loss":
                if hold["reason"] not in ("monitor-snapshot-stale", "manager-response-timeout"):
                    raise RuntimeError(f"loss did not stop through timeout: {hold}")
            elif hold["reason"] != "epoch-disagreement" or observed is None:
                raise RuntimeError(f"new identity not observed before HOLD: {hold}")
            row = dict(condition=name, repetition=repetition + 1, epoch=epoch,
                       commit_ns=durable, observed_ns=observed, hold_ns=hold["t_ns"],
                       zero_cmd_ns=probe.zero_ns, stopped_odom_ns=probe.stop_ns,
                       hold_reason=hold["reason"], final_xy=probe.map_xy,
                       commit_to_observed_ms=(observed - durable) / 1e6 if observed else None,
                       commit_to_hold_ms=(hold["t_ns"] - durable) / 1e6,
                       observed_to_hold_ms=(hold["t_ns"] - observed) / 1e6 if observed else None,
                       hold_to_zero_ms=(probe.zero_ns - hold["t_ns"]) / 1e6,
                       commit_to_stop_ms=(probe.stop_ns - durable) / 1e6)
            rows.append(row)
            with (folder / "trials.jsonl").open("a") as stream:
                stream.write(json.dumps(row) + "\n")
            print(json.dumps(row), flush=True)
        return rows
    finally:
        if probe is not None:
            probe.finished = True
            probe.publish_stop()
            base.spin_until(probe, lambda: False, 0.3)
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if processes is not None:
            processes.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("manager", "guard", "relay"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--domain-start", type=int, default=181)
    parser.add_argument("--conditions", nargs="+", choices=CONDITIONS, default=list(CONDITIONS))
    args, ros_args = parser.parse_known_args()
    if args.role:
        run_role(args.role, ros_args)
        return
    if not args.output or args.repetitions < 1 or ros_args:
        parser.error("provide a new --output directory and positive repetitions")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(conditions={name: CONDITIONS[name] for name in args.conditions},
                    repetitions=args.repetitions, speed_mps=0.20, lookahead_m=0.40,
                    phase_offsets_s=[0, 0.037, 0.074, 0.111],
                    snapshot_stale_s=1.0, request_timeout_s=1.0, hold_tick_s=0.20,
                    stopping_speed_mps=0.01, stopping_angular_radps=0.01,
                    stopping_confirmation_s=0.20, clock="same-host monotonic_ns",
                    commit_anchor="return from production WAL write + flush + fsync",
                    geometry="same far-away KEEP_OUT; only epoch changes",
                    domain_start=args.domain_start,
                    hostname=platform.node(), python=sys.version,
                    ros_distro=os.environ.get("ROS_DISTRO"))
    root = Path(__file__).resolve().parent.parent
    source_paths = [
        "scripts/benchmark_commit_to_hold.py", "scripts/ruleguard_controller_loop_matrix.py",
        "rulesync/src/rulesync_manager/rulesync_manager/manager_node.py",
        "rulesync/src/rulesync_manager/rulesync_manager/validator.py",
        "rulesync/src/rulesync_guard/rulesync_guard/guard_node.py",
        "rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py",
        "topopatch_ros/topopatch_ros/gate_node.py", "rulepatch_ros/rulepatch_ros/pp_driver.py",
    ]
    manifest["source_sha256"] = {
        path: hashlib.sha256((root / path).read_bytes()).hexdigest() for path in source_paths}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    rows = []
    try:
        for i, name in enumerate(args.conditions):
            rows.extend(run_condition(args, name, i))
    except Exception as exc:
        (args.output / "failure.json").write_text(json.dumps({"error": repr(exc)}, indent=2))
        raise
    import statistics
    summary = {}
    for name in args.conditions:
        selected = [row for row in rows if row["condition"] == name]
        summary[name] = {"n": len(selected)}
        for metric in ("commit_to_observed_ms", "observed_to_hold_ms", "commit_to_hold_ms",
                       "hold_to_zero_ms", "commit_to_stop_ms"):
            values = [row[metric] for row in selected if row[metric] is not None]
            summary[name][metric] = (dict(median=statistics.median(values),
                                         minimum=min(values), maximum=max(values)) if values else None)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
