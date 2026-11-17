#!/usr/bin/env python3
"""Benchmark-only RuleSync update-load instrumentation; production is inherited.

Run one fresh Gazebo stack per load/repetition. Updates are serialized on a
fixed request grid: a busy slot is recorded, never queued or silently retried.
The candidate producer retries at 50 ms only while a binding is unavailable.
Independent model-world pose is capture QA, not input to the production stack.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
import platform
from pathlib import Path
import random
import shutil
import sys
import time

MODULES = {
    "rulesync_manager.manager_node": "rulesync/src/rulesync_manager/rulesync_manager/manager_node.py",
    "rulesync_manager.validator": "rulesync/src/rulesync_manager/rulesync_manager/validator.py",
    "rulesync_guard.guard_node": "rulesync/src/rulesync_guard/rulesync_guard/guard_node.py",
    "rulesync_nav2_bridge.bridge_node": "rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py",
    "topopatch_ros.gate_node": "topopatch_ros/topopatch_ros/gate_node.py",
    "rulepatch_ros.pp_driver": "rulepatch_ros/rulepatch_ros/pp_driver.py",
    "rulesync3d.ruleguard": "rulesync3d/rulesync3d/ruleguard.py",
    "rulesync3d.gate2d": "rulesync3d/rulesync3d/gate2d.py",
}


def emit(event, **fields):
    row = dict(event=event, t_ns=time.monotonic_ns(), **fields)
    folder = Path(os.environ["RULESYNC_CHURN_FOLDER"])
    role = os.environ.get("RULESYNC_CHURN_ROLE", "runner")
    with (folder / f"{role}_events.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, allow_nan=False) + "\n")
    return row["t_ns"]


def run_role(role, ros_args):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    os.environ["RULESYNC_CHURN_ROLE"] = role
    rclpy.init(args=ros_args)
    if role == "manager":
        from rulesync_manager.manager_node import RuleManager

        class TimedManager(RuleManager):
            def _on_durable_decision(self, record):
                # Actual post-fsync hook, before any revocation publication.
                if record.get("decision") == "COMMIT":
                    emit("durable_commit", epoch=int(record["epoch"]),
                         snapshot_hash=record["target_hash"], txn_id=record["txn_id"])
                return super()._on_durable_decision(record)

        node = TimedManager()
    else:
        from rulesync_guard.guard_node import RuleGuardNode

        class PathPublisher:
            def __init__(self, publisher, owner):
                self.publisher, self.owner = publisher, owner

            def publish(self, message):
                self.publisher.publish(message)
                execution = self.owner._guard.execution_identity()
                emit("path_release" if message.poses else "path_stop",
                     execution=list(execution) if execution else None,
                     stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                     poses=len(message.poses), state=self.owner._state,
                     token_id=self.owner._token_id)

        class TimedGuard(RuleGuardNode):
            def __init__(self):
                self._logged_state = None
                super().__init__()
                self._path_pub = PathPublisher(self._path_pub, self)

            def _publish_state(self):
                super()._publish_state()
                execution = self._guard.execution_identity()
                identity = [int(self._guard.epoch or 0), str(self._guard.snap_hash or "")]
                value = (self._state, self._reason, self._degraded,
                         tuple(identity), execution)
                if value != self._logged_state:
                    emit("guard_state", state=self._state, reason=self._reason,
                         degraded=self._degraded, authority=identity,
                         execution=list(execution) if execution else None)
                    self._logged_state = value

            def _on_snapshot(self, future, sequence):
                entered = time.monotonic_ns()
                current = sequence == self._snapshot_pending_seq
                super()._on_snapshot(future, sequence)
                # _degraded also becomes True on epoch-disagreement HOLD;
                # that does NOT mean the successful snapshot was unobserved.
                # A successful callback refreshes _snapshot_at; invalidation
                # sets it to zero. Do not censor the first observation by HOLD.
                if (current and self._guard.epoch is not None
                        and self._snapshot_at >= entered / 1e9):
                    emit("snapshot_observed", callback_entry_ns=entered,
                         authority=[int(self._guard.epoch), str(self._guard.snap_hash)])

        node = TimedGuard()
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


def run_trial(args, trial, folder):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMsg
    from rulesync_interfaces.msg import RuleGuardState
    from rulesync_interfaces.srv import ListRules, ProposeRule, CommitRule
    import ruleguard_controller_loop_matrix as base
    from gazebo_world_pose import make_world_probe, write_manifest, validate_active_sequence

    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1",
               GZ_PARTITION=f"rulesync_churn_{args.domain_id}",
               RULESYNC_CHURN_FOLDER=str(folder), RULESYNC_CHURN_ROLE="runner")
    env.pop("ROS_DISCOVERY_SERVER", None)
    os.environ.update(env)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)

    class Probe(make_world_probe(base.MatrixProbe)):
        def __init__(self):
            super().__init__("gate_guard", math.inf, None, 0.0)
            self.desired_epoch = 1
            self.retry_enabled = False
            self.create_timer(.05, self.retry_candidate)

        def retry_candidate(self):
            state = self.guard_state
            if self.retry_enabled and (state is None or state.state == RuleGuardState.HOLD
                    or state.degraded or state.execution_epoch != self.desired_epoch
                    or state.execution_hash != state.authority_hash):
                self.publish_candidate()

        def publish_candidate(self):
            if self.map_xy is None:
                return
            path = PathMsg()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            x, y = self.map_xy
            for k in range(9):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x + (base.GOAL[0] - x) * k / 8
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.candidate_pub.publish(path)
            emit("candidate_request", desired_epoch=self.desired_epoch,
                 stamp_ns=path.header.stamp.sec * 10**9 + path.header.stamp.nanosec)

        def _on_controller_path(self, message):
            super()._on_controller_path(message)
            emit("controller_path_received", poses=len(message.poses),
                 stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)

    stack_args = argparse.Namespace(arm="gate_guard", rho=.20, speed=.15,
                                   lookahead=.40, goal_tolerance=.15,
                                   world_truth=True, monitor_y_offset=0.0)
    # write_manifest serializes arguments, so provide only JSON-native fields.
    write_manifest(stack_args, folder)
    command = [sys.executable, str(Path(__file__).resolve())]
    processes, probe = None, None
    result = dict(trial=trial, result="FAIL")
    try:
        processes = base.start_stack(stack_args, folder, env, commands={
            "manager": command + ["--role", "manager"],
            "guard": command + ["--role", "guard"],
        })
        rclpy.init()
        probe = Probe()
        if not base.spin_until(probe, lambda: probe.map_xy is not None
                               and len(probe.world_samples) >= 20, 45):
            raise RuntimeError("missing odometry/world model pose")
        for client in (probe.list_rules, probe.propose, probe.commit):
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError(f"missing service: {client.srv_name}")
        ready = False
        for _ in range(225):
            if not base.call(probe, probe.list_rules, ListRules.Request()).degraded:
                ready = True
                break
            base.spin_until(probe, lambda: False, .2)
        if not ready:
            raise RuntimeError("manager remained degraded")
        # Service discovery waits do not spin this probe's subscriptions.
        # Reacquire the stationary stream before applying capture freshness;
        # never relax the freshness/identity checks or authorize while waiting.
        emit("startup_services_ready")
        base.spin_until(probe, lambda: False, .5)
        probe.verify_stationary_alignment()
        emit("startup_capture_ready")

        def rule(epoch):
            value = base.make_rule()
            value.epoch = epoch
            # Same geometry at all epochs, deliberately away from the path.
            for point in value.polygon.points:
                point.y += 5.0
            return value

        proposal = base.call(probe, probe.propose, ProposeRule.Request(rule=rule(1)))
        if not proposal.accepted:
            raise RuntimeError(f"initial proposal rejected: {proposal.reason}")
        committed = base.call(probe, probe.commit, CommitRule.Request(epoch=1))
        if not committed.committed:
            raise RuntimeError(f"initial commit rejected: {committed.reason}")
        probe.retry_enabled = True
        if not base.spin_until(probe, lambda: probe.controller_path_seen
                and probe.guard_state is not None and probe.guard_state.execution_epoch == 1
                and probe.guard_state.state != RuleGuardState.HOLD, 8):
            raise RuntimeError("initial execution binding unavailable")
        # The same fixed warm-up for all loads, not conditioned on movement.
        base.spin_until(probe, lambda: False, .4)
        probe.started_at = time.monotonic()
        start_ns = emit("measurement_start")
        deadline = start_ns + round(args.duration * 1e9)
        period = trial["period_s"]
        slots = (int(round(args.duration / period)) - 1) if period else 0
        next_slot, epoch, pending = 1, 1, None

        def advance_transaction():
            nonlocal pending
            if pending is None or not pending["future"].done():
                return
            response = pending["future"].result()
            if pending["phase"] == "propose":
                emit("proposal_result", slot=pending["slot"], epoch=pending["epoch"],
                     accepted=response.accepted, reason=response.reason)
                if response.accepted:
                    pending["phase"] = "commit"
                    pending["future"] = probe.commit.call_async(
                        CommitRule.Request(epoch=pending["epoch"]))
                    return
            else:
                emit("commit_result", slot=pending["slot"], epoch=pending["epoch"],
                     committed=response.committed, reason=response.reason)
                if response.committed:
                    probe.desired_epoch = pending["epoch"]
            pending = None

        while time.monotonic_ns() < deadline:
            rclpy.spin_once(probe, timeout_sec=.01)
            advance_transaction()
            now = time.monotonic_ns()
            while next_slot <= slots and now >= start_ns + round(next_slot * period * 1e9):
                if pending is not None:
                    emit("update_slot_busy", slot=next_slot,
                         scheduled_ns=start_ns + round(next_slot * period * 1e9))
                else:
                    epoch += 1
                    emit("update_request", slot=next_slot, epoch=epoch,
                         scheduled_ns=start_ns + round(next_slot * period * 1e9))
                    pending = dict(slot=next_slot, epoch=epoch, phase="propose",
                                   future=probe.propose.call_async(
                                       ProposeRule.Request(rule=rule(epoch))))
                next_slot += 1
            probe.assert_truth_healthy()
        emit("measurement_end", window_end_ns=deadline)
        # Preserve the outcome of a last in-flight attempt; never silently drop it.
        drain_deadline = time.monotonic() + 35
        while pending is not None and time.monotonic() < drain_deadline:
            rclpy.spin_once(probe, timeout_sec=.01)
            advance_transaction()
        if pending is not None:
            emit("transaction_timeout", slot=pending["slot"], epoch=pending["epoch"])
            raise RuntimeError("outstanding transaction failed to finish")
        recovered = base.spin_until(probe, lambda: probe.guard_state is not None
            and probe.guard_state.execution_epoch == probe.desired_epoch
            and probe.guard_state.authority_epoch == probe.desired_epoch
            and probe.guard_state.execution_hash == probe.guard_state.authority_hash
            and not probe.guard_state.degraded
            and probe.guard_state.state != RuleGuardState.HOLD, 3)
        emit("recovery_end", recovered=recovered, desired_epoch=probe.desired_epoch)
        processes.assert_alive()
        probe.assert_truth_healthy()
        active = [row for row in probe.world_samples if row["active"]]
        gaps = validate_active_sequence(active)
        result.update(result="PASS", initial_binding=True, recovery_after_window=recovered,
                      world_samples=len(active), max_sim_gap_s=max(gaps),
                      final_world_xy=[active[-1]["x"], active[-1]["y"]])
        if any(math.hypot(row["x"] - base.GOAL[0], row["y"] - base.GOAL[1])
               <= .15 for row in active):
            raise RuntimeError("goal reached during exposure; availability confounded")
    except Exception as exc:
        result.update(result="FAIL", error=repr(exc))
    finally:
        if probe is not None:
            probe.retry_enabled = False
            probe.finished = True
            probe.publish_stop()
            base.spin_until(probe, lambda: False, .3)
            try:
                probe.save_observations(folder)
            except Exception as exc:
                result.update(result="FAIL", capture_save_error=repr(exc))
            finally:
                probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if processes is not None:
            processes.close()
        (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("manager", "guard"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--periods", nargs="+", type=float, default=[0, 2, 1, .5])
    parser.add_argument("--duration", type=float, default=12)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--domain-id", type=int, default=198)
    parser.add_argument("--cohort", default="Churn-12")
    args, ros_args = parser.parse_known_args()
    if args.role:
        run_role(args.role, ros_args)
        return
    if (not args.output or ros_args or args.duration <= 0 or args.duration > 12
            or args.repetitions < 1 or any(p < 0 or (p and
                (args.duration / p < 2 or abs(args.duration / p - round(args.duration / p)) > 1e-8))
                for p in args.periods)):
        parser.error("new output required; duration in (0,12], each nonzero period divides duration")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    trials = [dict(period_s=p, repetition=r) for p in args.periods
              for r in range(1, args.repetitions + 1)]
    random.Random(20261010).shuffle(trials)
    root = Path(__file__).resolve().parent.parent
    sources = ["scripts/benchmark_rule_churn.py", "scripts/audit_rule_churn.py",
               "scripts/test_rule_churn_audit.py", "scripts/gazebo_world_pose.py",
               "scripts/ruleguard_controller_loop_matrix.py", *MODULES.values()]
    hashes = {}
    for relative in sources:
        source, target = root / relative, args.output / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        hashes[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    installed = {}
    for module, relative in MODULES.items():
        path = Path(importlib.util.find_spec(module).origin)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != hashes[relative]:
            raise RuntimeError(f"installed/source SHA mismatch: {module}")
        installed[module] = dict(path=str(path), sha256=digest)
    manifest = dict(cohort=args.cohort, pre_run_wall_ns=time.time_ns(), trials=trials,
        runtime=dict(hostname=platform.node(), python=sys.version,
                     platform=platform.platform(), ros_distro=os.environ.get("ROS_DISTRO"),
                     monotonic_resolution_s=time.get_clock_info("monotonic").resolution),
        source_sha256=hashes, installed_modules=installed, duration_s=args.duration,
        domain_id=args.domain_id, seed=20261010,
        fixed=dict(speed_mps=.15, rho_m=.20, lookahead_m=.40, goal_tolerance_m=.15,
                   snapshot_poll_s=.20, snapshot_stale_s=1.0, request_timeout_s=1.0,
                   pose_stale_s=.50, candidate_retry_s=.05, index_enabled=False),
        policy="all attempts; no retries; serial fixed-grid transactions; busy slots retained",
        scope="same far-away KEEP_OUT geometry, epoch/full hash changes; local fail-stop no injected link fault",
        availability="continuous event integration: durable identity = guard authority = execution = last nonempty release, non-HOLD and not degraded",
        stale_release="last nonempty released identity != durable authority until empty path publication; not proof of physical motion",
        clock="same-host monotonic_ns; publication instrumentation after production call return")
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    records = []
    for index, trial in enumerate(trials, 1):
        folder = args.output / f"trial_{index:02d}"
        folder.mkdir()
        record = dict(index=index, **run_trial(args, trial, folder))
        records.append(record)
        with (args.output / "trials.jsonl").open("a") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")
        print(json.dumps(record), flush=True)
    unchanged = all(hashlib.sha256((root / relative).read_bytes()).hexdigest() == digest
                    for relative, digest in hashes.items())
    (args.output / "completion.json").write_text(json.dumps(dict(
        attempted=len(records), failures=sum(r["result"] != "PASS" for r in records),
        source_unchanged=unchanged, completed_wall_ns=time.time_ns()), indent=2) + "\n")
    return int(not unchanged or any(r["result"] != "PASS" for r in records))


if __name__ == "__main__":
    sys.exit(main())
