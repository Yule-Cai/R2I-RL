#!/usr/bin/env python3
"""Pilot: an in-flight KEEP_OUT update, with matched execution-monitor arms.

Reuse the existing production transaction/gate/controller and world-pose probe.
Only the geometry-only ablation deliberately disables execution identity checks;
snapshot validation, freshness, polling, pose checks and stop latching remain.
No replan or new candidate is submitted after the update. All attempts are saved.
This runner currently covers 2-D only; it is not the proposed 54-row cohort.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import random
import shutil
import sys
import time

ARMS = ("gate_only", "geometry_only", "full")
CONDITIONS = ("normal", "delay", "loss")
# Explicit conservative planar envelope, relative to the model/base_footprint.
# Encloses the installed waffle chassis, wheels, casters and camera projection.
# rho=.28 also encloses its .2702 m maximum corner radius. This is a declared
# envelope, not a reconstruction of Gazebo's continuous collision trajectories.
BODY = ((-.21, -.17), (.11, -.17), (.11, .17), (-.21, .17))
NEW_REGION = ((-.30, -.80), (.70, -.80), (.70, -.20), (-.30, -.20))


def emit(event, **fields):
    from benchmark_rule_churn import emit as write
    return write(event, **fields)


def transformed_body(row):
    qx, qy, qz, qw = row["quaternion"]
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(max(-1., min(1., 2*(qw*qy - qz*qx))))
    if abs(roll) > .05 or abs(pitch) > .05:
        raise ValueError("non-planar pose exceeds declared envelope evaluation scope")
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz))
    c, s = math.cos(yaw), math.sin(yaw)
    return [(row["x"] + c*x - s*y, row["y"] + s*x + c*y) for x, y in BODY]


def convex_intersects(a, b):
    """Closed-set SAT, including touching edges. Both inputs must be convex."""
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (p[1] - q[1], q[0] - p[0])
            left = [x*axis[0] + y*axis[1] for x, y in a]
            right = [x*axis[0] + y*axis[1] for x, y in b]
            if max(left) < min(right) or max(right) < min(left):
                return False
    return True


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx*dx + dy*dy
    t = max(0., min(1., ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/n)) if n else 0.
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def body_distance(body, region=NEW_REGION):
    if convex_intersects(body, region):
        return 0.0  # Do not mislabel this as a signed penetration depth.
    return min(point_segment_distance(p, a, b)
               for vertices, edges in ((body, region), (region, body))
               for p in vertices for a, b in zip(edges, edges[1:] + edges[:1]))


def measured_stop(rows, commit_ns, speed_limit=.01, sustain_s=.20):
    """World-position finite-difference speed, simulation-time sustain window.

    Do not substitute command zero for a stop. No pre-COMMIT motion interval is
    eligible. Return the first low-speed sample receipt and its confirmation.
    """
    onset = None
    for a, b in zip(rows, rows[1:]):
        dt = (b["sim_ns"] - a["sim_ns"])/1e9
        if dt <= 0 or dt > .1:
            raise ValueError("invalid/gapped world sample interval")
        speed = math.hypot(b["x"]-a["x"], b["y"]-a["y"])/dt
        if a["receipt_ns"] < commit_ns or speed > speed_limit:
            onset = None
        else:
            if onset is None:
                onset = b
            if (b["sim_ns"] - onset["sim_ns"])/1e9 >= sustain_s:
                return dict(onset_receipt_ns=onset["receipt_ns"],
                            confirmed_receipt_ns=b["receipt_ns"],
                            onset_sim_ns=onset["sim_ns"], sustain_sim_s=sustain_s)
    return None


def read_events(folder):
    rows = []
    for path in sorted(Path(folder).glob("*_events.jsonl")):
        rows.extend(json.loads(line) for line in path.read_text().splitlines() if line)
    return sorted(rows, key=lambda row: row["t_ns"])


def evaluate(folder, worlds, guards, commands):
    from gazebo_world_pose import validate_active_sequence
    rows = read_events(folder)
    commits = [r for r in rows if r["event"] == "durable_commit" and r["epoch"] == 2]
    if len(commits) != 1:
        raise ValueError("exactly one durable epoch-2 commit required")
    commit = commits[0]
    t0 = commit["t_ns"]
    active = [r for r in worlds if r["active"]]
    gaps = validate_active_sequence(active)
    post = [r for r in active if r["receipt_ns"] >= t0]
    pre = [r for r in active if r["receipt_ns"] < t0]
    if len(post) < 20:
        raise ValueError("insufficient post-COMMIT world samples")
    observed = next((r for r in rows if r["event"] == "snapshot_observed"
                     and r["authority"] == [2, commit["snapshot_hash"]]
                     and r["t_ns"] >= t0), None)
    # Distinguish a fresh post-COMMIT HOLD from a repeated already-latched HOLD.
    # Message constants are CLEAR=0, WARNING=1, HOLD=2.
    prior = [r for r in rows if r["event"] == "guard_decision" and r["t_ns"] < t0]
    held_before = bool(prior and prior[-1]["state"] == "HOLD")
    hold = next((r for r in guards if r["receipt_ns"] >= t0 and r["state"] == 2), None)
    zero = next((r for r in commands if r["receipt_ns"] >= (hold["receipt_ns"] if hold else t0)
                 and abs(r["linear_x"]) <= .01 and abs(r["angular_z"]) <= .01), None)
    stop = measured_stop(active, t0)
    entered = [r for r in post if convex_intersects(transformed_body(r), list(NEW_REGION))]
    first_entry = entered[0] if entered else None
    def ms(t):
        return (t-t0)/1e6 if t is not None else None
    hold_world = next((r for r in post if hold and r["receipt_ns"] >= hold["receipt_ns"]), None)
    stop_world = next((r for r in post if stop and r["sim_ns"] == stop["onset_sim_ns"]), None)
    travel = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"])
                 for a, b in zip(post, post[1:])
                 if stop and b["sim_ns"] <= stop["onset_sim_ns"])
    return dict(durable_identity=[2, commit["snapshot_hash"]],
        commit_ns=t0, commit_to_observed_ms=ms(observed["t_ns"] if observed else None),
        hold_active_before_commit=held_before,
        commit_to_hold_receipt_ms=ms(hold["receipt_ns"] if hold and not held_before else None),
        hold_reason=hold["reason"] if hold else None,
        commit_to_zero_command_receipt_ms=ms(zero["receipt_ns"] if zero else None),
        commit_to_world_low_speed_onset_ms=ms(stop["onset_receipt_ns"] if stop else None),
        commit_to_world_low_speed_confirmation_ms=ms(stop["confirmed_receipt_ns"] if stop else None),
        world_low_speed=stop, post_commit_sample_count=len(post), max_sim_gap_s=max(gaps),
        sampled_body_entry=first_entry is not None,
        sampled_body_entry_count=len(entered),
        first_body_entry_sim_ns=first_entry["sim_ns"] if first_entry else None,
        minimum_sampled_body_distance_m=min(body_distance(transformed_body(r)) for r in post),
        hold_sampled_body_distance_m=body_distance(transformed_body(hold_world)) if hold_world else None,
        stop_sampled_body_distance_m=body_distance(transformed_body(stop_world)) if stop_world else None,
        commit_to_stop_sampled_travel_m=travel if stop else None,
        first_post_commit_xy=[post[0]["x"], post[0]["y"]],
        last_pre_commit_xy=[pre[-1]["x"], pre[-1]["y"]] if pre else None,
        commit_receipt_bracket_ms=(post[0]["receipt_ns"] - pre[-1]["receipt_ns"])/1e6 if pre else None,
        post_commit_final_xy=[post[-1]["x"], post[-1]["y"]],
        scope="sampled declared planar body envelope; no continuous swept-body or physical safety proof",
        clock="same-host monotonic receipts for latencies; sim time for world velocity/sustain; receipt boundary is not exact COMMIT sim time")


def geometry_core_class(core_class):
    """Expose the single identity ablation to ROS-free contract tests."""
    class GeometryCore(core_class):
        def check_bound(self, clearance, velocity=None):
            if self.execution_identity() is None:
                return super().check_bound(clearance, velocity)
            # Deliberate ablation: keep the original execution pin for logging,
            # but ignore disagreement when evaluating geometry. Same hysteresis.
            return self.check(clearance, self.epoch, self.snap_hash, velocity)
    return GeometryCore


def timed_guard_class(node_class, core_class):
    """Instrument the production node; expose callback ablation for unit tests."""
    class TimedGuard(node_class):
        def __init__(self):
            self.geometry_only = os.environ["RULESYNC_DYNAMIC_ARM"] == "geometry_only"
            self._in_snapshot = False
            self._identity_stop_suppressed = False
            super().__init__()
            if self.geometry_only:
                self._guard = geometry_core_class(core_class)(self._guard.rho_warn, self._guard.rho_release)

        def _on_snapshot(self, future, sequence):
            entered = time.monotonic_ns()
            current = sequence == self._snapshot_pending_seq
            old_latch = self._latched_hold
            self._in_snapshot = True
            self._identity_stop_suppressed = False
            try:
                super()._on_snapshot(future, sequence)
            finally:
                self._in_snapshot = False
            if self._identity_stop_suppressed:
                self._latched_hold = old_latch
            if current and self._guard.epoch is not None and self._snapshot_at >= entered/1e9:
                emit("snapshot_observed", callback_entry_ns=entered,
                     authority=[int(self._guard.epoch), str(self._guard.snap_hash)])

        def _set_state(self, state, reason, degraded, clearance):
            if self.geometry_only and self._in_snapshot and reason == "epoch-disagreement":
                self._identity_stop_suppressed = True
                emit("ablated_identity_stop", authority=[self._guard.epoch, self._guard.snap_hash],
                     execution=list(self._guard.execution_identity()))
                return
            if self._state != state or self._reason != reason:
                execution = self._guard.execution_identity()
                emit("guard_decision", state=state, reason=reason,
                     authority=[self._guard.epoch, self._guard.snap_hash],
                     execution=list(execution) if execution else None)
                if state == "HOLD":
                    emit("hold_decision", reason=reason,
                         authority=[self._guard.epoch, self._guard.snap_hash],
                         execution=list(execution) if execution else None)
            return super()._set_state(state, reason, degraded, clearance)

        def _publish_stop(self):
            if self._in_snapshot and self._identity_stop_suppressed:
                return
            return super()._publish_stop()
    return TimedGuard


def run_role(role, ros_args):
    if role == "manager":
        from benchmark_rule_churn import run_role as inherited
        return inherited("manager", ros_args)
    if role == "relay":
        from benchmark_commit_to_hold import run_role as inherited
        return inherited("relay", ros_args)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rulesync_guard.guard_node import RuleGuardNode
    from rulesync3d.ruleguard import RuleGuard
    rclpy.init(args=ros_args)

    os.environ["RULESYNC_CHURN_ROLE"] = "guard"
    node = timed_guard_class(RuleGuardNode, RuleGuard)()
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
    from rulesync_interfaces.msg import RuleGuardState
    from rulesync_interfaces.srv import ListRules, ProposeRule, CommitRule
    from geometry_msgs.msg import Point32
    from std_srvs.srv import SetBool
    import ruleguard_controller_loop_matrix as base
    from gazebo_world_pose import make_world_probe, write_manifest
    arm, mode = trial["arm"], trial["condition"]
    stack_arm = "gate" if arm == "gate_only" else "gate_guard"
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1",
        GZ_PARTITION=f"rulesync_dynamic_{args.domain_id}",
        RULESYNC_DYNAMIC_ARM=arm, RULESYNC_CHURN_FOLDER=str(folder),
        RULESYNC_CHURN_ROLE="runner", RULESYNC_TIMELINE_MODE=mode,
        RULESYNC_TIMELINE_LOG=str(folder / "relay_events.jsonl"))
    env.pop("ROS_DISCOVERY_SERVER", None)
    os.environ.update(env)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)

    class Probe(make_world_probe(base.MatrixProbe)):
        def __init__(self):
            super().__init__(stack_arm, math.inf, None, 0.)
            self.fault = self.create_client(SetBool, "/benchmark/fault")

        def _save_live(self, name, rows, before):
            # Preserve partial evidence even if SIGKILL/container removal prevents
            # the normal finally block. Only evaluation streams are instrumented.
            with (folder/f"{name}_live.jsonl").open("a") as stream:
                for row in rows[before:]:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")

        def _on_world_pose(self, message):
            before = len(self.world_samples)
            super()._on_world_pose(message)
            self._save_live("world_pose", self.world_samples, before)

        def _on_odom(self, message):
            before = len(self.odom_samples)
            super()._on_odom(message)
            self._save_live("odom_and_monitor_input", self.odom_samples, before)

        def _on_guard_state(self, message):
            before = len(self.guard_events)
            super()._on_guard_state(message)
            self._save_live("guard_states", self.guard_events, before)

        def _on_nominal_cmd(self, message):
            before = len(self.command_events)
            super()._on_nominal_cmd(message)
            self._save_live("commands", self.command_events, before)

    stack_args = argparse.Namespace(arm=stack_arm, rho=.28, speed=.20,
        lookahead=.40, goal_tolerance=.15, world_truth=True, monitor_y_offset=0.)
    write_manifest(stack_args, folder)
    command = [sys.executable, str(Path(__file__).resolve())]
    processes, probe = None, None
    result = dict(**trial, result="FAIL", phase="startup")
    try:
        processes = base.start_stack(stack_args, folder, env, commands={
            "manager": command + ["--role", "manager"],
            "guard": command + ["--role", "guard", "--ros-args", "-p",
                                  "manager_service:=/benchmark/list_rules"],
        })
        if arm != "gate_only":
            processes.start("snapshot_relay", command + ["--role", "relay"])
        rclpy.init()
        probe = Probe()
        if not base.spin_until(probe, lambda: probe.map_xy is not None and len(probe.world_samples) >= 20, 45):
            raise RuntimeError("missing odometry or independent world pose")
        clients = [probe.list_rules, probe.propose, probe.commit]
        if arm != "gate_only":
            clients.append(probe.fault)
        for client in clients:
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError(f"missing service {client.srv_name}")
        ready = False
        for _ in range(225):
            if not base.call(probe, probe.list_rules, ListRules.Request()).degraded:
                ready = True
                break
            base.spin_until(probe, lambda: False, .2)
        if not ready:
            raise RuntimeError("manager recovery incomplete")
        base.spin_until(probe, lambda: False, .5)
        probe.verify_stationary_alignment()

        def commit_rule(epoch):
            rule = base.make_rule()
            rule.epoch = epoch
            if epoch == 1:
                for p in rule.polygon.points:
                    p.y += 5.
            else:
                rule.polygon.points = [Point32(x=x, y=y) for x, y in NEW_REGION]
            proposed = base.call(probe, probe.propose, ProposeRule.Request(rule=rule))
            emit("proposal_reply", epoch=epoch, accepted=proposed.accepted, reason=proposed.reason)
            if not proposed.accepted:
                raise RuntimeError(f"proposal rejected: {proposed.reason}")
            committed = base.call(probe, probe.commit, CommitRule.Request(epoch=epoch))
            emit("commit_reply", epoch=epoch, committed=committed.committed, reason=committed.reason)
            if not committed.committed:
                raise RuntimeError(f"commit rejected: {committed.reason}")

        commit_rule(1)
        result["phase"] = "initial_binding"
        probe.started_at = time.monotonic()
        if not base.spin_until(probe, lambda: probe.controller_path_seen and
                (arm == "gate_only" or (probe.guard_state is not None
                 and probe.guard_state.execution_epoch == 1
                 and probe.guard_state.state != RuleGuardState.HOLD)), 8, candidate=True):
            raise RuntimeError("initial authorized execution unavailable")
        result["phase"] = "moving_trigger"
        if not base.spin_until(probe, lambda: probe.map_xy[0] >= -1.40, 15):
            raise RuntimeError("moving trigger not reached")
        if arm != "gate_only":
            fault = base.call(probe, probe.fault, SetBool.Request(data=True))
            if not fault.success:
                raise RuntimeError("fault relay activation failed")
        emit("update_requested", xy=list(probe.map_xy), condition=mode)
        result["phase"] = "dynamic_commit"
        commit_rule(2)
        result["phase"] = "post_commit_capture"
        # Equal simulated exposure even when the limited CPU makes Gazebo
        # slower than real time. Keep latency receipts on the host clock.
        start_sim_ns = probe.world_samples[-1]["sim_ns"]
        end_sim_ns = start_sim_ns + round(args.duration*1e9)
        wall_deadline = time.monotonic() + 4*args.duration + 5
        emit("measurement_start", start_sim_ns=start_sim_ns, end_sim_ns=end_sim_ns)
        while probe.world_samples[-1]["sim_ns"] < end_sim_ns:
            rclpy.spin_once(probe, timeout_sec=.01)
            probe.assert_truth_healthy()
            if time.monotonic() >= wall_deadline:
                raise RuntimeError("incomplete fixed simulation-time exposure")
        emit("measurement_end", observed_end_sim_ns=probe.world_samples[-1]["sim_ns"])
        processes.assert_alive()
        probe.assert_truth_healthy()
        metrics = evaluate(folder, probe.world_samples, probe.guard_events, probe.command_events)
        # Completeness is not success/safety: a baseline body entry is expected
        # to be reportable, not converted to FAIL or removed from the cohort.
        result.update(result="PASS", phase="complete", metrics=metrics)
    except Exception as exc:
        result.update(result="FAIL", error=repr(exc))
    finally:
        if probe is not None:
            try:
                probe.save_observations(folder)  # Before cleanup stop commands.
            except Exception as exc:
                result.update(result="FAIL", capture_save_error=repr(exc))
            probe.finished = True
            probe.publish_stop()
            base.spin_until(probe, lambda: False, .3)
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if processes is not None:
            processes.close()
        (folder / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def audit_run(folder):
    """Recompute every completed attempt; never turn absent/failed rows into PASS."""
    folder = Path(folder)
    manifest = json.loads((folder/"manifest.json").read_text())
    audit = dict(planned=len(manifest["trials"]), attempted=0, completed=0,
                 not_attempted=[], errors=[], trial_metrics=[])
    for index, trial in enumerate(manifest["trials"], 1):
        path = folder/f"trial_{index:02d}"
        if not path.exists():
            audit["not_attempted"].append(index)
            continue
        audit["attempted"] += 1
        try:
            result = json.loads((path/"result.json").read_text())
            if any(result.get(k) != v for k, v in trial.items()):
                raise ValueError("trial assignment mismatch")
            if result["result"] != "PASS":
                raise ValueError(f"recorded failure: {result.get('error')}")
            def load(name):
                return [json.loads(line) for line in (path/f"{name}.jsonl").read_text().splitlines()]
            metrics = evaluate(path, load("world_pose"), load("guard_states"), load("commands"))
            if metrics != result["metrics"]:
                raise ValueError("saved/recomputed metrics disagree")
            audit["completed"] += 1
            audit["trial_metrics"].append(dict(index=index, **trial, **metrics))
        except Exception as exc:
            audit["errors"].append(dict(index=index, error=repr(exc)))
    for relative, digest in manifest["source_sha256"].items():
        if hashlib.sha256((folder/"source"/relative).read_bytes()).hexdigest() != digest:
            audit["errors"].append(dict(source=relative, error="frozen source checksum mismatch"))
    audit["audit_passed"] = not audit["errors"] and audit["completed"] == audit["planned"]
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("manager", "guard", "relay"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--audit", type=Path, help="ROS-free recomputation of an archived run")
    parser.add_argument("--arms", nargs="+", choices=ARMS, default=list(ARMS))
    parser.add_argument("--conditions", nargs="+", choices=CONDITIONS, default=["normal"])
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--duration", type=float, default=10.,
                        help="post-update exposure seconds in simulation time")
    parser.add_argument("--domain-id", type=int, default=197)
    args, ros_args = parser.parse_known_args()
    if args.audit:
        if ros_args or args.role or args.output:
            parser.error("--audit cannot be combined with role/output or ROS arguments")
        audit = audit_run(args.audit)
        (args.audit/"recomputed_audit.json").write_text(json.dumps(audit, indent=2) + "\n")
        print(json.dumps(audit, indent=2))
        return int(not audit["audit_passed"])
    if args.role:
        return run_role(args.role, ros_args)
    if not args.output or ros_args or args.repetitions < 1 or not 8 <= args.duration <= 12:
        parser.error("fresh output required; duration 8..12 seconds; repetitions >=1")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    from benchmark_rule_churn import MODULES
    root = Path(__file__).resolve().parent.parent
    sources = ["scripts/benchmark_dynamic_rule_update.py", "scripts/test_dynamic_rule_update.py",
        "scripts/benchmark_rule_churn.py", "scripts/benchmark_commit_to_hold.py",
        "scripts/ruleguard_controller_loop_matrix.py", "scripts/gazebo_world_pose.py", *MODULES.values()]
    hashes = {}
    for relative in sources:
        source, target = root/relative, args.output/"source"/relative
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
    trials = [dict(arm=a, condition=c, repetition=r) for a in args.arms
              for c in args.conditions for r in range(1, args.repetitions+1)]
    random.Random(20261010).shuffle(trials)
    manifest = dict(cohort="DynamicUpdate-2D-pilot", trials=trials, source_sha256=hashes,
        installed_modules=installed, pre_run_wall_ns=time.time_ns(), hostname=platform.node(),
        arguments={**vars(args), "output": str(args.output)}, body_envelope_xy=BODY,
        new_region_xy=NEW_REGION, fixed=dict(speed_mps=.20, rho_m=.28, trigger_x=-1.40,
            snapshot_poll_s=.20, snapshot_stale_s=1., pose_stale_s=.50),
        ablation="geometry_only disables in-flight epoch/hash disagreement only; initial admission/token validation and freshness retained",
        policy="pilot only; all attempts retained; no retries; no post-update candidate/replan; PASS means capture complete, not no entry",
        exposure="duration in simulation time after commit reply; separate 4*duration+5 s wall timeout",
        scope="2-D sampled planar envelope; 3-D and Orange Pi not executed")
    (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    records = []
    for i, trial in enumerate(trials, 1):
        folder = args.output/f"trial_{i:02d}"
        folder.mkdir()
        try:
            record = dict(index=i, **run_trial(args, trial, folder))
        except Exception as exc:
            # Manifest/capture setup also belongs to the attempt denominator.
            # For example a recreated container may lack the installed SDF.
            record = dict(index=i, **trial, result="FAIL", phase="preflight",
                          error=repr(exc))
            (folder/"result.json").write_text(json.dumps(record, indent=2) + "\n")
        records.append(record)
        with (args.output/"trials.jsonl").open("a") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")
        print(json.dumps(record), flush=True)
    unchanged = all(hashlib.sha256((root/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    (args.output/"completion.json").write_text(json.dumps(dict(attempted=len(records),
        failures=sum(r["result"] != "PASS" for r in records), source_unchanged=unchanged,
        completed_wall_ns=time.time_ns()), indent=2) + "\n")
    return int(not unchanged or any(r["result"] != "PASS" for r in records))


if __name__ == "__main__":
    sys.exit(main())
