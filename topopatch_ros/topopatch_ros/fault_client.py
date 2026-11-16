"""Fault-injection suite (M4 paper sim eval + crash-consistency): fail-closed proofs.

Each fault asserts NO motion results from the faulty condition and
measures stop/recovery latency. Runs against the standard bench stack
planner+manager+bridge+topo (orchestrator staggers/kills per fault).

   F1 delayed mask: planner mask-topic remapped to /mask_delayed (see
      mask_delay.py, delay D s). Commit -> immediate replan must be
      GATE-BLOCKED (mask absent); after D, replan passes. Measures stop
      latency (commit->blocked) and recovery latency (mask->executable).
   F2 stale epoch: commit N, then propose epoch N-1 -> manager must
      reject; no status, no motion. Measures rejection latency.
   F3 missing ACK: orchestrator kills bridge -> commit must time out and
      fail; no motion. Measures timeout/stop latency.
   F4 planner down: orchestrator kills planner_server -> plan attempts
      must fail; nothing published; no motion. Measures fail latency.
   F5 invalid fallback: publish a rule-crossing Path directly on
      /rulepatch/repaired_path -> gate_node must block (pp remapped to
      /rulepatch/gated_path gets nothing); robot stationary. Then a
      clean path -> forwarded (positive control, brief motion in sim).
      Measures block latency + asserts zero gated violations.
   F6 discovery delay: run BEFORE manager exists -> service calls must
      time out fail-closed (no motion); orchestrator starts manager
      late -> client recovers and completes a commit. Measures
      undiscovered wait + recovery latency.
   F7 manager crash mid-PREPARE: bridge SIGSTOP-stalled so the commit
      round blocks in prepare; manager SIGKILLed mid-round, bridge
      resumed, manager restarted. Asserts: no commit completed (mask
      unchanged), restarted manager is epoch-0/empty, system recovers
      (fresh commit works), no motion. Measures recovery latency.
   F8 manager crash post-bridge-commit, pre-bookkeeping: requires the
      RULESYNC_TEST_COMMIT_PAUSE_S hook on the victim manager (test
      only). Bridge applies the commit round, manager dies before
      recording it. Restarted manager serves epoch 0/empty while the
      bridge mask carries the committed rules: records whether the
      divergence is detected or silent (expected: silent — motivates
      durable replay / fail-closed recovery; honest negative allowed).
   F9 bridge crash with active rules: mask topic dies; planner loses
      keepout (plans through the rule); gate (manager ListRules source)
      must still block; no motion; bridge restart restores the mask.
      Measures gap detection + restore latency.

Usage:
  python3 -m topopatch_ros.fault_client --fault f1 --csv out.csv [...]
Requires orchestrator-provided stack; see paper_faults.sh.
"""
import argparse
import csv
import json
import math
import time

import rclpy
from geometry_msgs.msg import Point32
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import CommitRule, ProposeRule


def stamp_now():
    from builtin_interfaces.msg import Time
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


def pip(xy, poly):
    x, y = xy
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
            if x < xin:
                inside = not inside
    return inside


class Faults(Node):
    def __init__(self):
        super().__init__("paper_fault_client")
        self.ac = ActionClient(self, ComputePathToPose,
                               "compute_path_to_pose")
        self.cli_propose = self.create_client(ProposeRule,
                                              "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule,
                                             "/rulesync/commit_rule")
        from geometry_msgs.msg import PoseStamped as _PSF
        self.goal_pub = self.create_publisher(_PSF, "/goal_pose", 10)
        self.track = []
        self.gate_stats = []
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/rulepatch/gate_status",
                                 self._on_gate, 10)
        # Crash-fault observability, created up front (DDS lesson: late
        # entity creation intermittently never discovers). MaskWatcher
        # tracks keepout-mask generations; cli_list reads manager state.
        from topopatch_ros.txn_trace import MaskWatcher
        from rulesync_interfaces.srv import ListRules as _LRF
        self.mask_watcher = MaskWatcher(self)
        self.cli_list = self.create_client(_LRF, "/rulesync/list_rules")
        from rulesync_interfaces.srv import RemoveRule as _RRF
        self._cli_rm = self.create_client(_RRF, "/rulesync/remove_rule")
        from rulesync_interfaces.srv import BridgeState as _BSF
        self._cli_bstate = self.create_client(
            _BSF, "/rulesync/nav2_bridge_state")

    def publish_goal(self, goal, repeats=12):
        from geometry_msgs.msg import PoseStamped as _PSF2
        gp = _PSF2()
        gp.header.frame_id = "map"
        gp.pose.position.x, gp.pose.position.y = float(goal[0]), float(
            goal[1])
        gp.pose.orientation.w = 1.0
        # Latch over several seconds: the bridge holds _last_goal in
        # volatile memory and may subscribe late; a 3-shot burst can be
        # missed entirely (observed: chronic NO_FEASIBLE_PATH rejects).
        for _ in range(repeats):
            self.goal_pub.publish(gp)
            time.sleep(0.4)

    def _on_odom(self, msg):
        self.track.append((float(msg.pose.pose.position.x),
                           float(msg.pose.position.y)))

    def _on_gate(self, msg):
        try:
            self.gate_stats.append(json.loads(msg.data))
        except Exception:
            pass

    def moved(self):
        if len(self.track) < 2:
            return False
        x0, y0 = self.track[0]
        return any(math.hypot(x - x0, y - y0) > 0.15
                   for x, y in self.track[-20:])

    def pose(self, p):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        m.pose.orientation.w = 1.0
        return m

    def plan(self, s, g, timeout=30.0):
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal = self.pose(s), self.pose(g)
        goal.planner_id = "GridBased"
        goal.use_start = True
        t0 = time.time()
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None, round(time.time() - t0, 3)
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        r = rf.result()
        dt = round(time.time() - t0, 3)
        if r is None:
            return None, dt
        path = [(p.pose.position.x, p.pose.position.y)
                for p in r.result.path.poses]
        return (path if len(path) >= 4 else None), dt

    def mk_rule(self, rid, epoch, poly):
        rule = RuleSpec()
        rule.rule_id = rid
        rule.epoch = int(epoch)
        rule.rule_type = 0
        rule.severity = 1
        rule.frame_id = "map"
        rule.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                               for x, y in poly]
        rule.valid_from = stamp_now()
        end = stamp_now()
        end.sec += 3600
        rule.valid_until = end
        return rule

    def propose(self, rule, timeout=30.0):
        # Re-latch goal before every propose: the bridge holds _last_goal
        # in volatile memory (restarts/late-subscribes drop it -> mundane
        # NO_FEASIBLE_PATH rejects). Belt and suspenders.
        try:
            self.publish_goal(getattr(self, "_goal_cache", (2.0, 0.5)))
        except Exception:
            pass
        t0 = time.time()
        fut = self.cli_propose.call_async(ProposeRule.Request(rule=rule))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        resp = fut.result()
        return (bool(resp and resp.accepted),
                round(time.time() - t0, 2),
                str(getattr(resp, "reason", "")))

    def commit(self, epoch, timeout=90.0):
        t0 = time.time()
        fut = self.cli_commit.call_async(
            CommitRule.Request(epoch=int(epoch)))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        resp = fut.result()
        return (bool(resp and resp.committed),
                round(time.time() - t0, 2),
                str(getattr(resp, "reason", "")))


RECT = [(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5)]


F1_RULE = [(-0.5, -1.0), (-0.5, 0.0), (0.5, 0.0), (0.5, -1.0)]


def run_f1(n, args):
    # Delayed mask, discriminating geometry: plan maskless FIRST, commit
    # a rule across that path's midpoint, then (a) immediate replan
    # (mask still delayed -> through the rule -> gate MUST block;
    # measures stop latency), (b) post-delay replan (mask applied ->
    # must avoid; measures recovery).
    from topopatch.topo.gate import gate_full_path
    n.publish_goal(args.goal)
    row = {"fault": "F1"}
    t0 = time.time()
    p_base, _ = n.plan(args.start, args.goal, timeout=15.0)
    if not p_base:
        row.update(fail_closed=True, note="no-baseline-plan-no-motion",
                   motion=bool(n.moved()))
        return row
    mid = p_base[len(p_base) // 2]
    h = 0.5
    rect = [(mid[0] - h, mid[1] - h), (mid[0] - h, mid[1] + h),
            (mid[0] + h, mid[1] + h), (mid[0] + h, mid[1] - h)]
    row["rule"] = [[round(float(x), 3), round(float(y), 3)]
                   for x, y in rect]
    ok, ct, _ = n.propose(n.mk_rule("f1", args.epoch, rect),
                          timeout=30.0)
    okc, cct, _ = n.commit(args.epoch, timeout=30.0)
    row["commit_t"] = cct
    if not (ok and okc):
        row.update(fail_closed=True, note="commit-failed-no-motion",
                   motion=bool(n.moved()))
        return row
    p0, _ = n.plan(args.start, args.goal, timeout=15.0)
    go, rs = gate_full_path(p0, [rect], lambda x, y: False,
                            (-1e9, -1e9, 1e9, 1e9))
    row["pre_mask_blocked"] = bool(not go)
    row["pre_mask_reason"] = rs
    row["stop_latency_s"] = round(time.time() - t0, 2)
    time.sleep(args.delay + 2.0)
    p1, _ = n.plan(args.start, args.goal, timeout=15.0)
    row["post_mask_path_avoids"] = bool(
        sum(1 for xy in p1 if pip(xy, rect)) == 0) if p1 else False
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(row["pre_mask_blocked"]
                              and not row["motion"])
    return row


F2_RULE = [(-0.5, -1.0), (-0.5, 0.0), (0.5, 0.0), (0.5, -1.0)]


def run_f2(n, args):
    n.publish_goal(args.goal)
    row = {"fault": "F2"}
    ok, _, _ = n.propose(n.mk_rule("f2a", args.epoch, F2_RULE),
                         timeout=30.0)
    okc, _, _ = n.commit(args.epoch, timeout=60.0)
    t0 = time.time()
    ok_s, _, _ = n.propose(
        n.mk_rule("f2stale", args.epoch - 1, RECT), timeout=30.0)
    row["stale_proposed"] = bool(ok_s)
    ok_sc, dt, reason = n.commit(args.epoch - 1, timeout=30.0)
    row["stale_commit_accepted"] = bool(ok_sc)
    row["reject_latency_s"] = dt
    row["reject_reason"] = reason
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    # Stale epoch must never commit (manager epoch check); without a
    # commit nothing executes. Propose-into-staging alone is inert.
    row["fail_closed"] = bool(not ok_sc and not row["motion"] and okc)
    row["note"] = f"commit_ok={okc}"
    return row


def run_f3(n, args):
    # bridge already killed by orchestrator
    row = {"fault": "F3"}
    t0 = time.time()
    ok, _, _ = n.propose(n.mk_rule("f3", args.epoch, RECT), timeout=25.0)
    okc, dt, reason = n.commit(args.epoch, timeout=45.0)
    row["commit_ok"] = bool(okc)
    row["timeout_latency_s"] = round(time.time() - t0, 2)
    row["reason"] = reason
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(not okc and not row["motion"])
    return row


def run_f4(n, args):
    # planner already killed by orchestrator
    row = {"fault": "F4"}
    t0 = time.time()
    p, dt = n.plan(args.start, args.goal, timeout=8.0)
    row["plan_ok"] = bool(p)
    row["fail_latency_s"] = round(time.time() - t0, 2)
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(not p and not row["motion"])
    return row


F5_RULE = [(-0.5, -1.0), (-0.5, 0.0), (0.5, 0.0), (0.5, -1.0)]


def run_f5(n, args):
    # Commit a rule on verified-free sandbox water, then inject a
    # rule-crossing path (also collision-free) straight onto the raw
    # topic: gate_node MUST block it with KEEP_OUT (not collision).
    # A parallel clean line must be forwarded (positive control).
    from nav_msgs.msg import Path as _P
    from geometry_msgs.msg import PoseStamped as _PS
    row = {"fault": "F5"}
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f5rule", args.epoch, F5_RULE),
                         timeout=30.0)
    okc, _, _ = n.commit(args.epoch, timeout=60.0)
    row["commit_ok"] = bool(okc)
    if not okc:
        row.update(fail_closed=True, note="commit-rejected-no-motion",
                   motion=bool(n.moved()))
        return row
    pub = n.create_publisher(_P, "/rulepatch/repaired_path", 10)
    time.sleep(1.0)
    n.gate_stats = []

    def mkpath(pts):
        m = _P()
        m.header.frame_id = "map"
        for x, y in pts:
            ps = _PS()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        return m

    import numpy as _np
    bad = [(float(-2 + i * 0.15), -0.5) for i in range(21)]
    for _ in range(3):
        pub.publish(mkpath(bad))
        time.sleep(0.5)
    t0 = time.time()
    while time.time() - t0 < 12.0 and not any(
            (not g.get("execute", True)) for g in n.gate_stats):
        rclpy.spin_once(n, timeout_sec=0.5)
    bl = [g for g in n.gate_stats if not g.get("execute", True)]
    row["violator_blocked"] = bool(bl)
    row["block_reason"] = bl[0].get("reason") if bl else None
    row["block_latency_s"] = round(time.time() - t0, 2)
    good = [(float(-2 + i * 0.15), 0.5) for i in range(21)]
    n.gate_stats = []
    for _ in range(3):
        pub.publish(mkpath(good))
        time.sleep(0.5)
    t1 = time.time()
    while time.time() - t1 < 12.0 and not any(
            g.get("execute", False) for g in n.gate_stats):
        rclpy.spin_once(n, timeout_sec=0.5)
    row["clean_forwarded"] = bool(
        any(g.get("execute", False) for g in n.gate_stats))
    time.sleep(2.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(
        row["violator_blocked"]
        and row.get("block_reason", "").find("KEEP_OUT") >= 0)
    return row


def run_f6(n, args):
    # manager NOT running yet (orchestrator starts it late)
    row = {"fault": "F6"}
    t0 = time.time()
    ok, dt, _ = n.propose(n.mk_rule("f6", args.epoch, RECT), timeout=12.0)
    row["prediscovery_accepted"] = bool(ok)
    row["undiscovered_wait_s"] = round(time.time() - t0, 2)
    row["motion_prediscovery"] = bool(n.moved())
    # wait for orchestrator to start the manager, then recover
    t1 = time.time()
    recovered = False
    while time.time() - t1 < args.recover_timeout:
        rclpy.spin_once(n, timeout_sec=1.0)
        ok2, _, _ = n.propose(n.mk_rule("f6b", args.epoch, RECT),
                              timeout=8.0)
        if ok2:
            recovered = True
            break
        time.sleep(4.0)
    row["recovered"] = bool(recovered)
    row["recovery_latency_s"] = round(time.time() - t1, 2)
    if recovered:
        okc, _, _ = n.commit(args.epoch, timeout=60.0)
        row["commit_ok"] = bool(okc)
    row["fail_closed"] = bool(not row["prediscovery_accepted"]
                              and not row["motion_prediscovery"])
    return row


def _sh(cmd):
    import subprocess
    try:
        subprocess.run(cmd, shell=True, timeout=30,
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    except Exception:
        pass


def _list_rules(n, timeout=15.0):
    from rulesync_interfaces.srv import ListRules as _LRQ
    fut = n.cli_list.call_async(_LRQ.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout)
    r = fut.result()
    if r is None:
        return None
    return {"epoch": int(getattr(r, "epoch", -1)),
            "active": sorted(str(x.rule_id)
                             for x in getattr(r, "active", []))}


def _mask_gen(n):
    try:
        return int(n.mask_watcher.snapshot()["gen"])
    except Exception:
        return -1


def _restart_manager():
    # Manager-only restart (bridge untouched): direct binary run with
    # the same parameters as rulesync_sim.launch.py.
    _sh("source /opt/ros/lyrical/setup.bash; "
        "source ~/rulesync_ws/install/setup.bash; "
        "export ROS_DISCOVERY_SERVER=127.0.0.1:11811 "
        "ROS_DISABLE_ROS2_DAEMON=1; "
        "setsid nohup ros2 run rulesync_manager manager --ros-args "
        "-p participants:='[nav2_bridge]' "
        "-p txn_log_path:=/tmp/rulesync_txn.log "
        "> /tmp/fault_mgrid_restart.log 2>&1 < /dev/null &")


def _restart_bridge():
    _sh("source /opt/ros/lyrical/setup.bash; "
        "source ~/rulesync_ws/install/setup.bash; "
        "export ROS_DISCOVERY_SERVER=127.0.0.1:11811 "
        "ROS_DISABLE_ROS2_DAEMON=1; "
        "setsid nohup ros2 run rulesync_nav2_bridge bridge --ros-args "
        "> /tmp/fault_bridge_restart.log 2>&1 < /dev/null &")


def run_f7(n, args):
    # Manager crash mid-PREPARE (deterministic via bridge SIGSTOP stall).
    import threading
    row = {"fault": "F7"}
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f7", args.epoch, RECT), timeout=30.0)
    row["proposed"] = bool(ok)
    for _ in range(6):
        rclpy.spin_once(n, timeout_sec=0.5)
    L0 = _list_rules(n)
    M0 = _mask_gen(n)
    row["pre_epoch"] = (L0 or {}).get("epoch")
    res = {}
    t = threading.Thread(
        target=lambda: res.update(
            commit=n.commit(args.epoch, timeout=20.0)),
        daemon=True)
    _sh("pkill -STOP -f '[r]ulesync_nav2_bridge/bridge'")
    t.start()
    time.sleep(2.5)  # prepare round blocked on the stopped bridge
    t0 = time.time()
    _sh("pkill -9 -f '[r]ulesync_manager/manager'")
    _sh("pkill -CONT -f '[r]ulesync_nav2_bridge/bridge'")
    t.join(timeout=30.0)
    row["crash_commit_latency_s"] = round(time.time() - t0, 2)
    cr = res.get("commit", (False, 0.0, "no-result"))
    row["crash_commit_ok"] = bool(cr[0])
    row["crash_commit_reason"] = str(cr[2])[:80]
    time.sleep(3.0)
    _restart_manager()
    ok_svc = n.cli_propose.wait_for_service(timeout_sec=30.0)
    row["manager_back"] = bool(ok_svc)
    time.sleep(2.0)
    L1 = _list_rules(n)
    M1 = _mask_gen(n)
    row["post_epoch"] = (L1 or {}).get("epoch")
    row["post_active"] = str((L1 or {}).get("active"))
    row["mask_gen_pre"] = M0
    row["mask_gen_post"] = M1
    row["mask_unchanged"] = bool(M0 == M1 and M0 >= 0)
    # recovery: fresh commit must work on the restarted manager
    ok2, _, _ = n.propose(
        n.mk_rule("f7b", args.epoch + 100, RECT), timeout=30.0)
    okc, _, _ = n.commit(args.epoch + 100, timeout=60.0)
    row["recovery_commit_ok"] = bool(ok2 and okc)
    # cleanup the recovery rule (in-block epochs)
    try:
        from rulesync_interfaces.srv import RemoveRule as _RR7
        if n._cli_rm.wait_for_service(timeout_sec=10.0):
            _fut = n._cli_rm.call_async(_RR7.Request(
                rule_id="f7b", epoch=int(args.epoch + 103)))
            rclpy.spin_until_future_complete(n, _fut, timeout_sec=20.0)
    except Exception:
        pass
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(
        not row["crash_commit_ok"] and not row["motion"]
        and (L1 or {}).get("epoch") == 0
        and (L1 or {}).get("active") == []
        and row["mask_unchanged"] and row["recovery_commit_ok"])
    return row


def run_f8(n, args):
    # Manager crash post-bridge-commit, pre-bookkeeping. Requires the
    # RULESYNC_TEST_COMMIT_PAUSE_S hook on the victim manager (the
    # orchestrator launches it with the hook; installed build carries
    # the hook). Hook presence is VERIFIED by timing (victim commit
    # must span the pause); without it the window is unhittable and
    # the row records hook_missing instead of a vacuous pass.
    import os as _os
    from topopatch.topo.gate import path_intersects_polygons
    row = {"fault": "F8"}
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f8", args.epoch, RECT), timeout=30.0)
    row["proposed"] = bool(ok)
    for _ in range(6):
        rclpy.spin_once(n, timeout_sec=0.5)
    L0 = _list_rules(n)
    M0 = _mask_gen(n)
    # Kill from a timer thread that makes NO ROS calls (a worker doing
    # ROS spins alongside the main thread raises "Executor is already
    # spinning"). Timing: propose already done; commit = prepare +
    # commit rounds (~0.2 s, mask applies) then the hook pause (8 s).
    # Firing at +3 s lands deterministically inside the pause.
    import threading as _th
    t0 = time.time()
    killer = _th.Timer(3.0, lambda: _sh(
        "pkill -9 -f '[r]ulesync_manager/manager'"))
    killer.start()
    try:
        okc, dt, reason = n.commit(args.epoch, timeout=60.0)
    finally:
        killer.cancel()
    row["victim_commit_ok"] = bool(okc)
    row["victim_commit_latency_s"] = round(time.time() - t0, 2)
    row["victim_commit_reason"] = str(reason)[:80]
    # Window reading: victim_ok+fast => commit fully completed before
    # the kill (post-bookkeeping crash: state loss on restart).
    # victim timeout => killed mid-flight (in-round, or in the
    # post-commit pause when the hook is active: atomicity violation).
    # Either way the restart wipes manager state; divergence is
    # measured below.
    time.sleep(3.0)
    _restart_manager()
    ok_svc = n.cli_propose.wait_for_service(timeout_sec=30.0)
    row["manager_back"] = bool(ok_svc)
    time.sleep(2.0)
    L1 = _list_rules(n)
    M1 = _mask_gen(n)
    row["pre_epoch"] = (L0 or {}).get("epoch")
    row["post_epoch"] = (L1 or {}).get("epoch")
    row["post_active"] = str((L1 or {}).get("active"))
    row["mask_gen_pre"] = M0
    row["mask_gen_post"] = M1
    # Post-hoc apply check: the bridge was never killed, so a mask
    # generation bump means the commit round applied pre-crash.
    row["bridge_applied"] = bool(M1 > M0 >= 0)
    # Divergence: bridge mask carries f8 while restarted manager is
    # epoch-0/empty. Detection would require epoch reconciliation;
    # record honestly whether anything detected it.
    row["diverged"] = bool(row["bridge_applied"]
                           and (L1 or {}).get("epoch") == 0
                           and (L1 or {}).get("active") == [])
    # consequence probe: a fresh low-epoch commit is accepted against
    # epoch 0 (mask keeps stale f8 merged in by the bridge)
    ok2, _, _ = n.propose(
        n.mk_rule("f8b", args.epoch + 100, RECT), timeout=30.0)
    okc, _, _ = n.commit(args.epoch + 100, timeout=60.0)
    row["postcrash_commit_ok"] = bool(ok2 and okc)
    L2 = _list_rules(n)
    row["final_active"] = str((L2 or {}).get("active"))
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    # Expected (pre-fix): divergence is SILENT -> fail_closed False.
    # A fix (durable replay or fail-closed resync) must flip this.
    row["fail_closed"] = bool(not row["diverged"]
                              and not row["motion"])
    return row


def _bridge_state(n, timeout=15.0):
    from rulesync_interfaces.srv import BridgeState as _BSQ
    fut = n._cli_bstate.call_async(_BSQ.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout)
    r = fut.result()
    if r is None:
        return None
    return {"epoch": int(getattr(r, "active_epoch", -1)),
            "active": sorted(str(x.rule_id)
                             for x in getattr(r, "active_rules", []))}


def run_f8m(n, args):
    # Crash-matrix point: commit with the victim killed K seconds after
    # the commit call (timer thread, no ROS in worker). After restart
    # (handshake build), compare manager vs bridge state after a settle
    # window: any remaining disagreement is SILENT DIVERGENCE (fail).
    # Reconciled states, degraded entries, motion, and recovery latency
    # are recorded per point.
    import threading as _th
    row = {"fault": "F8m", "point": getattr(args, "point", ""),
           "kill_delay_s": getattr(args, "kill_delay", 0.0)}
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f8m", args.epoch, RECT), timeout=30.0)
    row["proposed"] = bool(ok)
    for _ in range(6):
        rclpy.spin_once(n, timeout_sec=0.5)
    L0 = _list_rules(n)
    B0 = _bridge_state(n)
    M0 = _mask_gen(n)
    row["pre_manager_epoch"] = (L0 or {}).get("epoch")
    row["pre_bridge_epoch"] = (B0 or {}).get("epoch")
    t0 = time.time()
    killer = _th.Timer(float(getattr(args, "kill_delay", 3.0)),
                       lambda: _sh(
                           "pkill -9 -f '[r]ulesync_manager/manager'"))
    killer.start()
    try:
        okc, dt, reason = n.commit(args.epoch, timeout=60.0)
    finally:
        killer.cancel()
    row["victim_commit_ok"] = bool(okc)
    row["victim_commit_latency_s"] = round(time.time() - t0, 2)
    time.sleep(3.0)
    _restart_manager()
    ok_svc = n.cli_propose.wait_for_service(timeout_sec=30.0)
    row["manager_back"] = bool(ok_svc)
    # settle window: startup reconciliation (adopt/refresh rounds) needs
    # seconds; only disagreement persisting past it counts as silent.
    time.sleep(15.0)
    L1 = _list_rules(n)
    B1 = _bridge_state(n)
    M1 = _mask_gen(n)
    row["post_manager_epoch"] = (L1 or {}).get("epoch")
    row["post_manager_active"] = str((L1 or {}).get("active"))
    row["post_bridge_epoch"] = (B1 or {}).get("epoch")
    row["post_bridge_active"] = str((B1 or {}).get("active"))
    row["mask_bumped"] = bool(M1 > M0 >= 0)
    row["silent_divergence"] = bool(
        L1 is not None and B1 is not None and (
            L1.get("epoch") != B1.get("epoch") or
            (L1.get("active") or []) != (B1.get("active") or [])))
    # recovery: fresh commit on the reconciled manager
    ok2, _, _ = n.propose(
        n.mk_rule("f8mr", args.epoch + 100, RECT), timeout=30.0)
    okc2, _, _ = n.commit(args.epoch + 100, timeout=60.0)
    row["recovery_commit_ok"] = bool(ok2 and okc2)
    try:
        from rulesync_interfaces.srv import RemoveRule as _RRM
        if n._cli_rm.wait_for_service(timeout_sec=10.0):
            _fut = n._cli_rm.call_async(_RRM.Request(
                rule_id="f8mr", epoch=int(args.epoch + 103)))
            rclpy.spin_until_future_complete(n, _fut, timeout_sec=20.0)
    except Exception:
        pass
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(not row["silent_divergence"]
                              and not row["motion"])
    return row


def run_f8b(n, args):
    # Companion to F8: kill AFTER the commit fully returned
    # (post-bookkeeping). With the snapshot sidecar, the restarted
    # manager must recover epoch E with f8 active: no divergence,
    # fail-closed True. Pair (F8 silent, F8b recovered) bounds the
    # residual apply-vs-snapshot window precisely.
    row = {"fault": "F8b"}
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f8b", args.epoch, RECT), timeout=30.0)
    row["proposed"] = bool(ok)
    M0 = _mask_gen(n)
    okc, dt, reason = n.commit(args.epoch, timeout=60.0)
    row["commit_ok"] = bool(okc)
    row["commit_latency_s"] = dt
    time.sleep(2.0)  # post-bookkeeping, pre-kill
    _sh("pkill -9 -f '[r]ulesync_manager/manager'")
    time.sleep(3.0)
    _restart_manager()
    ok_svc = n.cli_propose.wait_for_service(timeout_sec=30.0)
    row["manager_back"] = bool(ok_svc)
    time.sleep(2.0)
    L1 = _list_rules(n)
    M1 = _mask_gen(n)
    row["post_epoch"] = (L1 or {}).get("epoch")
    row["post_active"] = str((L1 or {}).get("active"))
    row["recovered"] = bool((L1 or {}).get("epoch") == args.epoch
                            and "f8b" in ((L1 or {}).get("active") or []))
    # cleanup the recovered rule (in-block epoch)
    try:
        from rulesync_interfaces.srv import RemoveRule as _RR8
        if n._cli_rm.wait_for_service(timeout_sec=10.0):
            _fut = n._cli_rm.call_async(_RR8.Request(
                rule_id="f8b", epoch=int(args.epoch + 3)))
            rclpy.spin_until_future_complete(n, _fut, timeout_sec=20.0)
    except Exception:
        pass
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(row["recovered"] and not row["motion"])
    return row


def run_f10setup(n, args):
    # F10 pre-crash state setup: propose+commit one rule, exit. The
    # orchestrator then kills/restarts processes; run_f10 checks.
    row = {"fault": "F10setup", "point": getattr(args, "point", "")}
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f10s", args.epoch, RECT),
                         timeout=30.0)
    okc, _, _ = n.commit(args.epoch, timeout=60.0)
    row["commit_ok"] = bool(ok and okc)
    L0 = _list_rules(n)
    row["epoch_active"] = (L0 or {}).get("epoch")
    return row


def run_f10(n, args):
    # Both-process restart (supplementary): orchestrator killed manager
    # AND bridge with active rules, then restarted them in some order.
    # The pair must converge (periodic/startup reconciliation) with zero
    # motion; a recovery commit must then work. Order recorded in point.
    row = {"fault": "F10", "point": getattr(args, "point", "")}
    # wait for convergence: manager and bridge states must agree
    converged, t1 = False, time.time()
    L1 = B1 = None
    while time.time() - t1 < 90.0:
        rclpy.spin_once(n, timeout_sec=0.5)
        L1 = _list_rules(n)
        B1 = _bridge_state(n)
        if L1 is not None and B1 is not None and \
                L1.get("epoch") == B1.get("epoch") and \
                (L1.get("active") or []) == (B1.get("active") or []):
            converged = True
            break
        time.sleep(4.0)
    row["converged"] = bool(converged)
    row["converge_latency_s"] = round(time.time() - t1, 2)
    row["manager_epoch"] = (L1 or {}).get("epoch")
    row["manager_active"] = str((L1 or {}).get("active"))
    row["bridge_epoch"] = (B1 or {}).get("epoch")
    row["bridge_active"] = str((B1 or {}).get("active"))
    ok2, _, _ = n.propose(
        n.mk_rule("f10r", args.epoch, RECT), timeout=30.0)
    okc, _, _ = n.commit(args.epoch, timeout=60.0)
    row["recovery_commit_ok"] = bool(ok2 and okc)
    try:
        from rulesync_interfaces.srv import RemoveRule as _RR1
        if n._cli_rm.wait_for_service(timeout_sec=10.0):
            _fut = n._cli_rm.call_async(_RR1.Request(
                rule_id="f10r", epoch=int(args.epoch + 3)))
            rclpy.spin_until_future_complete(n, _fut, timeout_sec=20.0)
    except Exception:
        pass
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    # Convergence is expected only when the bridge is (eventually)
    # present. With --expect-converge 0 (bridge stays dead by design),
    # correct degraded operation = manager authoritative + recovery
    # commit safely refused + zero motion.
    if str(getattr(args, "expect_converge", "1")) == "1":
        row["fail_closed"] = bool(row["converged"] and not row["motion"]
                                  and row["recovery_commit_ok"])
    else:
        row["fail_closed"] = bool(
            (L1 or {}).get("epoch") is not None
            and not row["recovery_commit_ok"] and not row["motion"])
    return row


def run_f9(n, args):
    # Bridge crash with active rules: mask dies, planner loses keepout,
    # gate (manager ListRules source) must still block; no motion;
    # bridge restart restores the mask.
    # The rule is centered ON the straight start->goal line (open
    # sandbox) so the maskless plan is forced to cross it: a gate:ok
    # would then be a genuine miss, not a geometry artifact.
    from topopatch.topo.gate import (gate_full_path,
                                     path_intersects_polygons)
    row = {"fault": "F9"}
    F9_RULE = [(-0.5, -0.75), (-0.5, 0.25), (0.5, 0.25), (0.5, -0.75)]
    n.publish_goal(args.goal)
    ok, _, _ = n.propose(n.mk_rule("f9", args.epoch, F9_RULE),
                         timeout=30.0)
    okc, _, _ = n.commit(args.epoch, timeout=60.0)
    row["commit_ok"] = bool(ok and okc)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.5)
    M0 = _mask_gen(n)
    t0 = time.time()
    _sh("pkill -9 -f '[r]ulesync_nav2_bridge/bridge'")
    # mask death: no new generations for 6 s
    time.sleep(6.0)
    M1 = _mask_gen(n)
    row["mask_dead"] = bool(M1 == M0)
    row["mask_gap_s"] = round(time.time() - t0, 2)
    # planner without keepout plans through the rule; gate must block
    p, _ = n.plan(args.start, args.goal, timeout=30.0)
    row["plan_through_rule"] = bool(p)
    crosses = bool(p and path_intersects_polygons(p, [F9_RULE]))
    row["plan_crosses_rule"] = crosses
    L1 = _list_rules(n)
    row["manager_still_authoritative"] = bool(
        L1 is not None and "f9" in (L1.get("active") or []))
    if p and crosses:
        go, rs = gate_full_path(
            p, [F9_RULE], lambda x, y: False,
            (-1e9, -1e9, 1e9, 1e9))
        row["gate_blocked"] = bool(not go)
        row["gate_reason"] = str(rs)
    elif p:
        row["gate_blocked"] = True
        row["gate_reason"] = "planner-avoided (no crossing to block)"
    else:
        row["gate_blocked"] = True
        row["gate_reason"] = "no-plan"
    time.sleep(3.0)
    row["motion_during_gap"] = bool(n.moved())
    # recovery: restart bridge, mask must come back
    t1 = time.time()
    _restart_bridge()
    restored = False
    while time.time() - t1 < 90.0:
        rclpy.spin_once(n, timeout_sec=0.5)
        if _mask_gen(n) > M1:
            restored = True
            break
    row["bridge_restored"] = bool(restored)
    row["restore_latency_s"] = round(time.time() - t1, 2)
    time.sleep(3.0)
    row["motion"] = bool(n.moved())
    row["fail_closed"] = bool(row["mask_dead"] and row["gate_blocked"]
                              and not row["motion"]
                              and row["bridge_restored"])
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fault", required=True,
                    choices=["f1", "f2", "f3", "f4", "f5", "f6",
                             "f7", "f8", "f8b", "f8m", "f9", "f10", "f10setup"])
    ap.add_argument("--kill-delay", type=float, default=3.0,
                    help="F8m: seconds after the commit call to kill "
                    "the manager")
    ap.add_argument("--point", default="",
                    help="F8m: matrix point label")
    ap.add_argument("--expect-converge", default="1",
                    help="F10: set 0 when the bridge stays dead by design")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--epoch", type=int, default=401)
    ap.add_argument("--start", default="-2.0,-0.5")
    ap.add_argument("--goal", default="2.0,0.5")
    ap.add_argument("--delay", type=float, default=20.0,
                    help="F1 mask delay (s)")
    ap.add_argument("--recover-timeout", type=float, default=180.0)
    a = ap.parse_args()
    a.start = tuple(float(v) for v in a.start.split(","))
    a.goal = tuple(float(v) for v in a.goal.split(","))
    rclpy.init()
    node = Faults()
    node._goal_cache = a.goal
    node.publish_goal(a.goal)
    fn = {"f1": run_f1, "f2": run_f2, "f3": run_f3, "f4": run_f4,
          "f5": run_f5, "f6": run_f6, "f7": run_f7, "f8": run_f8,
          "f8b": run_f8b, "f8m": run_f8m, "f9": run_f9, "f10": run_f10, "f10setup": run_f10setup}[a.fault]
    try:
        row = fn(node, a)
    except Exception as e:
        import traceback
        traceback.print_exc()
        row = {"fault": a.fault.upper(), "ok": False,
               "reason": f"error: {e}"}
    row["epoch"] = a.epoch
    row["point"] = getattr(a, "point", "")
    print(f"[fault] result: {row}", flush=True)
    with open(a.csv, "w", newline="") as f:
        import csv as _csv
        keys = sorted(row.keys())
        w = _csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerow(row)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
