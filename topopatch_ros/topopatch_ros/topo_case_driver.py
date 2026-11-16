"""TopoRulePatch Gazebo closed-loop case driver (minimal stack).

No BT/NavigateToPose here: plans come from standalone planner_server
(compute_path_to_pose, use_start from the known spawn). Flow per case:
plan -> 1.5x1.5 KEEP_OUT rect across plan midpoint -> propose+commit via
RuleSync manager -> wait TopoRulePatch status -> pp_driver (already
running) executes the repaired/fallback path -> arrival + rule
compliance from the odom track.

Usage:
  python3 -m topopatch_ros.topo_case_driver --csv ~/rulepatch/gz_cases.csv
"""
import argparse
import csv
import math
import time

import rclpy
from geometry_msgs.msg import Point32
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import CommitRule, ProposeRule

CASES = [((2.23, -0.88), 601), ((1.5, 1.5), 611), ((0.5, -2.0), 621)]
SPAWN = (-2.0, -0.5)


def _densify(path, n=32):
    """Linear densification for sparse planner outputs (ThetaStar
    straights). A 2-point straight plan is valid, just sparse."""
    import numpy as _np
    p = _np.asarray(path, dtype=_np.float64)
    if len(p) >= n:
        return [tuple(q) for q in p.tolist()]
    seg = _np.hypot(p[1:, 0] - p[:-1, 0], p[1:, 1] - p[:-1, 1])
    cum = _np.concatenate([[0.0], _np.cumsum(seg)])
    total = cum[-1]
    if total < 1e-9:
        return [tuple(p[0])] * n
    us = _np.linspace(0, total, n)
    xs = _np.interp(us, cum, p[:, 0])
    ys = _np.interp(us, cum, p[:, 1])
    return list(zip(xs.tolist(), ys.tolist()))
RECT_HALF = 0.5  # 1.0x1.0 rules: bridge-feasible in sandbox; the model
# prescores all-fail here (OOD arena) -> abstain -> fallback evidence.
# Static map->odom TF of the minimal stack (demo_topo.sh): odom origin in
# map frame. Odom track points convert as p_map = p_odom + (TX, TY).
ODOM_TX, ODOM_TY = -2.0, -0.5


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


def stamp_now():
    from builtin_interfaces.msg import Time
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


class Driver(Node):
    def __init__(self):
        super().__init__("topo_case_driver")
        self.ac = ActionClient(self, ComputePathToPose,
                               "compute_path_to_pose")
        self.cli_propose = self.create_client(ProposeRule,
                                              "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule,
                                             "/rulesync/commit_rule")
        self.track = []
        self.statuses = []
        self.mask_nonzero = 0
        self._map = None
        self.scan_min = float("inf")
        self.scan_contacts = 0
        self.exec_mode = "pp"
        self.controller_id = ""
        self._last_repaired = None
        from sensor_msgs.msg import LaserScan as _LS
        self.create_subscription(_LS, "/scan", self._on_scan, 10)
        from nav_msgs.msg import Path as _RP
        self.create_subscription(_RP, "/rulepatch/repaired_path",
                                 self._on_repaired, 10)
        from geometry_msgs.msg import PoseStamped as _PS
        self.goal_pub = self.create_publisher(_PS, "/goal_pose", 10)
        from nav_msgs.msg import Path as _Path
        self.path_pub = self.create_publisher(
            _Path, "/rulepatch/repaired_path", 10)
        from topopatch_ros.txn_trace import MaskWatcher
        from rulesync_interfaces.srv import ListRules as _LR2
        self.mask_watcher = MaskWatcher(self)
        self.cli_list = self.create_client(_LR2, "/rulesync/list_rules")
        self.path_pub_gated = self.create_publisher(
            _Path, "/rulepatch/gated_path", 10)
        from nav_msgs.msg import OccupancyGrid as _OG
        self.create_subscription(_OG, "/rulesync/keepout_mask",
                                 self._on_mask, 10)
        from rclpy.qos import QoSProfile as _QP
        from rclpy.qos import DurabilityPolicy as _DP
        self.create_subscription(
            _OG, "/map", self._on_map,
            _QP(depth=10, durability=_DP.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/rulepatch/status",
                                 self._on_status, 10)
        # DDS lesson (narrow-B campaign): create ALL clients up front.
        # A remove_rule client created minutes after startup
        # intermittently never discovers its service, leaking rules.
        from rulesync_interfaces.srv import RemoveRule as _RRinit
        self._cli_rm = self.create_client(
            _RRinit, "/rulesync/remove_rule")
        # Physical contact instrumentation (Blocker C): Gazebo contact
        # sensor bridged to /base_contact (ros_gz_interfaces/Contacts).
        # Pure measurement: never influences decisions. Missing topic
        # (older stacks) simply records contact_available=False.
        self.contact_n = 0
        self.contact_first_t = None
        self.contact_available = False
        try:
            from ros_gz_interfaces.msg import Contacts as _CT
            self.create_subscription(_CT, "/base_contact",
                                     self._on_contact, 10)
            self.contact_available = True
        except Exception:
            pass

    def _on_contact(self, msg):
        try:
            n = len(getattr(msg, "contacts", []) or [])
        except Exception:
            return
        if n > 0:
            self.contact_n += n
            if self.contact_first_t is None:
                import time as _t
                self.contact_first_t = _t.time()

    def _on_repaired(self, msg):
        self._last_repaired = [(p.pose.position.x, p.pose.position.y)
                               for p in msg.poses]

    def _on_scan(self, msg):
        try:
            import math as _m
            vals = [v for v in msg.ranges
                    if v is not None and _m.isfinite(v)]
            if vals:
                mn = min(vals)
                if mn < self.scan_min:
                    self.scan_min = float(mn)
                if mn < 0.30:
                    self.scan_contacts += 1
        except Exception:
            pass

    def _on_map(self, msg):
        self._map = msg

    def map_bounds(self):
        m = self._map
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        return (ox, oy, ox + m.info.width * m.info.resolution,
                oy + m.info.height * m.info.resolution)

    def occ_fn(self):
        m = self._map
        w, data = m.info.width, list(m.data)
        res, ox, oy = m.info.resolution, m.info.origin.position.x, \
            m.info.origin.position.y

        def fn(x, y):
            ix = int((x - ox) / res)
            iy = int((y - oy) / res)
            if 0 <= ix < w and 0 <= iy < len(data) // w:
                return data[iy * w + ix] > 50
            return True
        return fn

    def _on_mask(self, msg):
        try:
            self.mask_nonzero = sum(1 for v in msg.data if v > 0)
        except Exception:
            pass

    def _on_odom(self, msg):
        self.track.append((float(msg.pose.pose.position.x),
                           float(msg.pose.pose.position.y)))

    def _on_status(self, msg):
        import json
        try:
            self.statuses.append(json.loads(msg.data))
        except Exception:
            pass

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
        goal.planner_id = getattr(self, "planner_id", "GridBased")
        goal.use_start = True
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        r = rf.result()
        if r is None:
            return None
        path = [(p.pose.position.x, p.pose.position.y)
                for p in r.result.path.poses]
        if len(path) < 2:
            return None
        return _densify(path)

    def propose_commit(self, rule, timeout=90.0):
        t0 = time.time()
        try:
            self._mask_gen_at_commit = \
                self.mask_watcher.snapshot()["gen"]
        except Exception:
            pass
        fut = self.cli_propose.call_async(ProposeRule.Request(rule=rule))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        resp = fut.result()
        if resp is None or not resp.accepted:
            return False, round(time.time() - t0, 2)
        fut2 = self.cli_commit.call_async(
            CommitRule.Request(epoch=int(rule.epoch)))
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=timeout)
        resp2 = fut2.result()
        okc = bool(resp2 and resp2.committed)
        return (okc, round(time.time() - t0, 2))

    def wait_status(self, epoch, timeout=40.0):
        t0 = time.time()
        want = f"epoch={epoch}"
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            for s in self.statuses:
                if want in str(s.get("note", "")):
                    return s
        return None

    def to_map(self, xy):
        sx, sy = getattr(self, "spawn_xy", SPAWN)
        return (xy[0] + sx, xy[1] + sy)

    def wait_arrival(self, gx, gy, timeout=150.0, tol=0.5, guard=None,
                     guard_polys=(), r_body=0.0):
        """pp-mode arrival wait, optionally RuleGuard-monitored (2D).
        guard: RuleGuard instance or None. On monitor HOLD: stop_robot()
        and return False with self._last_guard recording the intervention
        (fail-closed execution monitoring; frozen 2D behavior unchanged
        when guard is None)."""
        from rulesync3d.rulesync3d.ruleguard import HOLD as _GH
        from rulesync3d.rulesync3d.ruleguard import WARNING as _GW
        t0 = time.time()
        self._last_guard = {"guard_warn_n": 0, "guard_hold_n": 0,
                            "guard_min_clearance": None,
                            "guard_hold": False}
        _gmin = float("inf")
        _last_chk = 0.0
        arrived = False
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.track:
                mx, my = self.to_map(self.track[-1])
                if guard is not None and \
                        time.time() - _last_chk >= 0.5:
                    _last_chk = time.time()
                    gst, ginfo = guard.check_bound_2d(
                        (mx, my), guard_polys, float(r_body))
                    clr = float(ginfo.get("clearance", float("inf")))
                    if clr < _gmin:
                        _gmin = clr
                    if gst == _GW:
                        self._last_guard["guard_warn_n"] += 1
                    elif gst == _GH:
                        self._last_guard["guard_hold_n"] += 1
                        self._last_guard["guard_hold"] = True
                        self.stop_robot()
                        break
                if math.hypot(mx - gx, my - gy) < tol:
                    arrived = True
                    break
        if _gmin != float("inf"):
            self._last_guard["guard_min_clearance"] = round(_gmin, 3)
        return arrived

    def stop_robot(self):
        """Publish an empty path straight to the gated topic: pp clears
        its tracking and stops (no stale paths across cells)."""
        from nav_msgs.msg import Path as _PSP
        m = _PSP()
        m.header.frame_id = "map"
        self.path_pub_gated.publish(m)

    def publish_path(self, path):
        from nav_msgs.msg import Path as _Path
        from geometry_msgs.msg import PoseStamped as _PS2
        msg = _Path()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = _PS2()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def current_map_pose(self):
        if self.track:
            return self.to_map(self.track[-1])
        return getattr(self, "spawn_xy", SPAWN)

    def execute_path(self, path, goal, exec_mode="pp",
                     controller_id="", timeout=150.0, tol=0.5):
        """Execute a VERIFIED path. pp: publish for pp_driver.
        followpath: send to controller_server FollowPath action.
        Returns dict(exec_t, arrived, min_goal_dist, ...)."""
        gx, gy = goal
        out = {}
        t0 = time.time()
        if exec_mode == "followpath":
            from nav2_msgs.action import FollowPath
            from geometry_msgs.msg import PoseStamped as _PS3
            from nav_msgs.msg import Path as _P3
            ac = ActionClient(self, FollowPath, "follow_path")
            if not ac.wait_for_server(timeout_sec=10.0):
                out.update(executed=False,
                           reason="followpath-server-unavailable")
                return out
            msg = _P3()
            msg.header.frame_id = "map"
            for x, y in path:
                ps = _PS3()
                ps.header.frame_id = "map"
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
            gl = FollowPath.Goal()
            gl.path = msg
            gl.controller_id = {"dwb": "FollowPathDWB",
                                "mppi": "FollowPathMPPI"}.get(
                                    controller_id, controller_id)
            gl.goal_checker_id = "general_goal_checker"
            f = ac.send_goal_async(gl)
            rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
            gh = f.result()
            if gh is None or not gh.accepted:
                out.update(executed=False, reason="followpath-rejected")
                return out
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf,
                                             timeout_sec=timeout)
            r = rf.result()
            import math as _m2
            track_map = [self.to_map(xy) for xy in self.track]
            mind = (min(_m2.hypot(mx - gx, my - gy)
                        for mx, my in track_map)
                    if track_map else None)
            if r is None:
                out.update(executed=True, arrived=False,
                           reason="followpath-timeout")
            else:
                out.update(executed=True,
                           arrived=bool(r.status == 4),
                           result_code=int(r.status),
                           reason=getattr(r.result, "error_msg", ""))
            out["exec_t"] = round(time.time() - t0, 1)
            out["min_goal_dist"] = (round(mind, 3)
                                    if mind is not None else None)
            return out
        self.publish_path(path)
        arrived = bool(self.wait_arrival(gx, gy, timeout=timeout, tol=tol))
        out.update(executed=True, arrived=arrived,
                   exec_t=round(time.time() - t0, 1))
        return out

    def do_execute(self, path, goal, out):
        """Execute a VERIFIED path under the configured mode; fills
        arrived/exec_t/min_goal_dist/scan stats into out."""
        if self.exec_mode == "followpath":
            r = self.execute_path(path, goal, exec_mode="followpath",
                                  controller_id=self.controller_id)
            out.update({k: r.get(k) for k in
                        ("executed", "arrived", "exec_t",
                         "min_goal_dist", "reason", "result_code")
                        if k in r})
            out["exec_mode"] = "followpath"
            out["controller"] = self.controller_id
        else:
            self.publish_path(path)
            t0 = time.time()
            _gg, _gp, _ge, _gh = getattr(self, "_g2d",
                                         (None, (), 0, ""))
            out["arrived"] = bool(self.wait_arrival(
                *goal, guard=_gg, guard_polys=_gp))
            out["exec_t"] = round(time.time() - t0, 1)
            out["exec_mode"] = "pp"
            out.update({f"g2d_{k}": v for k, v in
                        getattr(self, "_last_guard", {}).items()})
        out["scan_min_m"] = round(self.scan_min, 3) if \
            self.scan_min != float("inf") else None
        out["scan_contacts"] = int(self.scan_contacts)
        return out

    def run_baseline(self, goal, epoch):
        """No-rule baseline: plan + execute via pp, no commit."""
        gx, gy = goal
        self.stop_robot()
        self.statuses = []
        self.track = []
        out = {"goal": goal, "epoch": epoch, "mode": "baseline",
               "map": getattr(self, "map_tag", ""),
               "planner": getattr(self, "planner_id", ""),
               "map_known": bool(self._map is not None)}
        from geometry_msgs.msg import PoseStamped
        gp = PoseStamped()
        gp.header.frame_id = "map"
        gp.pose.position.x, gp.pose.position.y = float(gx), float(gy)
        gp.pose.orientation.w = 1.0
        self.goal_pub.publish(gp)
        start = self.current_map_pose()
        out["start"] = [round(float(start[0]), 3),
                        round(float(start[1]), 3)]
        t0 = time.time()
        plan = self.plan(start, goal)
        out["plan_t"] = round(time.time() - t0, 2)
        if not plan:
            out.update(ok=False, reason="no-plan")
            return out
        self.do_execute(plan, goal, out)
        track_map = [self.to_map(xy) for xy in self.track]
        mind = (min(math.hypot(mx - gx, my - gy) for mx, my in track_map)
                if track_map else None)
        out["track_n"] = len(self.track)
        out["min_goal_dist"] = (round(mind, 3)
                                if mind is not None else None)
        out["ok"] = True
        return out

    def run_case(self, goal, epoch, fixed_rule=None,
               fixed_rules=None):
        gx, gy = goal
        self.stop_robot()
        # capture start BEFORE resetting the track
        start = self.current_map_pose()
        self.statuses = []
        self.track = []
        self.scan_min = float("inf")
        self.scan_contacts = 0
        self.contact_n = 0
        self.contact_first_t = None
        out = {"goal": goal, "epoch": epoch,
               "map": getattr(self, "map_tag", ""),
               "planner": getattr(self, "planner_id", ""),
               "map_known": bool(self._map is not None)}
        t0 = time.time()
        # feeds bridge feasibility + Nav2 input (M2 pattern)
        from geometry_msgs.msg import PoseStamped
        gp = PoseStamped()
        gp.header.frame_id = "map"
        gp.pose.position.x, gp.pose.position.y = float(gx), float(gy)
        gp.pose.orientation.w = 1.0
        for _ in range(6):
            self.goal_pub.publish(gp)
            time.sleep(0.3)
        out["start"] = [round(float(start[0]), 3),
                        round(float(start[1]), 3)]
        plan = self.plan(start, goal)
        out["plan_t"] = round(time.time() - t0, 2)
        if not plan:
            out.update(ok=False, reason="no-plan")
            return out
        if fixed_rules is not None:
            rects = [[(float(p[0]), float(p[1])) for p in poly]
                     for poly in fixed_rules]
            out["rule_src"] = "manifest"
        elif fixed_rule is not None:
            rects = [[(float(p[0]), float(p[1])) for p in fixed_rule]]
            out["rule_src"] = "prescored"
        else:
            mid = plan[len(plan) // 2]
            rect = [(mid[0] - RECT_HALF, mid[1] - RECT_HALF),
                    (mid[0] - RECT_HALF, mid[1] + RECT_HALF),
                    (mid[0] + RECT_HALF, mid[1] + RECT_HALF),
                    (mid[0] + RECT_HALF, mid[1] - RECT_HALF)]
            out["rule_src"] = "auto"
            rects = [rect]
        out["rules"] = [[[round(float(x), 3), round(float(y), 3)]
                         for x, y in r] for r in rects]
        out["rule"] = out["rules"][0]
        out["n_rules_placed"] = len(rects)
        # incremental commit (one rule per epoch in this case's block
        # of 10): bridge validates feasibility per rule.
        placed, tids, ct = [], [], 0.0
        for k, poly in enumerate(rects):
            rule = RuleSpec()
            rule.rule_id = f"topo-case-{epoch}-{k}"
            rule.epoch = int(epoch) + k
            rule.rule_type = 0
            rule.severity = 1
            rule.frame_id = "map"
            rule.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                                   for x, y in poly]
            rule.valid_from = stamp_now()
            end = stamp_now()
            end.sec += 3600
            rule.valid_until = end
            ok1, ct1 = self.propose_commit(rule)
            ct += ct1
            if ok1:
                placed.append(poly)
                tids.append(f"topo-case-{epoch}-{k}")
            else:
                out.setdefault("skipped_rules", []).append(k)
        out["commit_t"] = round(ct, 2)
        out["placed_n"] = len(placed)
        if not placed:
            out.update(ok=False, reason="commit-failed")
            return out
        rects = placed
        rect = rects[0]
        epoch = epoch + len(placed) - 1
        # 2D RuleGuard context (retrofit subset): monitor actual pp-track
        # against the committed polygons + epoch. Built only when enabled;
        # frozen 2D behavior unchanged otherwise.
        self._g2d = (None, (), 0, "")
        if bool(getattr(self, "_guard2d", False)):
            from rulesync3d.rulesync3d.ruleguard import (
                RuleGuard, snapshot_hash)
            _g = RuleGuard()
            _items = [(f"topo-case-{epoch}-{k}", int(epoch), 3,
                       ("poly", tuple((round(float(x), 4),
                                             float(y)) for x, y in poly)))
                      for k, poly in enumerate(rects)]
            _h = snapshot_hash(_items)
            _g.update_snapshot(int(epoch), _h)
            _g.bind_execution(int(epoch), _h)
            self._g2d = (_g, [list(map(list, p)) for p in rects],
                         int(epoch), _h)
        try:
            st = self.wait_status(epoch)
            if not st:
                out.update(ok=False, reason="no-status")
                return out
            out.update(accepted=bool(st.get("accepted")),
                       fallback=bool(st.get("fallback")),
                       reason=st.get("reason"),
                       topo_pred=st.get("topo_pred"),
                       conf=st.get("conf"),
                       latency_ms=st.get("latency_ms"),
                       infer_ms=st.get("infer_ms"))
            if out["accepted"] and self.exec_mode == "followpath":
                # Accepted repaired path comes from topo_inference; forward
                # it to the controller under test (pp would bypass it).
                rp = self._last_repaired
                if rp:
                    r = self.execute_path(
                        rp, goal, exec_mode="followpath",
                        controller_id=self.controller_id)
                    out["fp_exec"] = {k: r.get(k) for k in
                                      ("executed", "arrived", "exec_t",
                                       "reason", "result_code")}
                out["exec_mode"] = "followpath"
                out["controller"] = self.controller_id
            elif out["accepted"]:
                out["exec_mode"] = "pp"
            if out["fallback"] and not out["accepted"]:
                # M4 Task A: universal output invariant. The fallback replan
                # is GATED before execution (bounded retries for mask
                # propagation, then safe stop — never publish a violator).
                # No auto RemoveRule: abort is reported, human decides.
                from topopatch.topo.gate import (FALLBACK_RETRY_GAP_S,
                                                 MAX_FALLBACK_REPLANS,
                                                 gate_full_path)
                from topopatch_ros.txn_trace import wait_mask_ready
                import time as _time
                _mr, _mw, _md = wait_mask_ready(
                    self, self.mask_watcher, self.cli_list, epoch,
                    timeout_s=3.0,
                    base_gen=getattr(self, "_mask_gen_at_commit", None))
                out["mask_ready"] = bool(_mr)
                out["mask_wait_ms"] = round(_mw, 1)
                fb, attempts, gate_rs = None, 0, "gate:not-attempted"
                gate_ms = 0.0
                for _ in range(MAX_FALLBACK_REPLANS):
                    attempts += 1
                    t1 = _time.time()
                    cand = self.plan(self.current_map_pose(), goal,
                                     timeout=30.0)
                    out.setdefault("fallback_t_first",
                                   round(_time.time() - t1, 2))
                    if not cand:
                        gate_rs = "gate:plan-failed"
                    elif self._map is None:
                        _tg = _time.monotonic()
                        go, gate_rs = gate_full_path(
                            cand, rects, lambda x, y: False,
                            (-1e9, -1e9, 1e9, 1e9))
                        gate_ms += (_time.monotonic() - _tg) * 1000.0
                        if go:
                            fb = cand
                            break
                    else:
                        _tg2 = _time.monotonic()
                        go, gate_rs = gate_full_path(
                            cand, rects, self.occ_fn(), self.map_bounds())
                        gate_ms += (_time.monotonic() - _tg2) * 1000.0
                        if go:
                            fb = cand
                            break
                    if attempts < MAX_FALLBACK_REPLANS:
                        _time.sleep(FALLBACK_RETRY_GAP_S)
                out["fallback_attempts"] = attempts
                out["fallback_gate"] = gate_rs
                out["gate_ms"] = round(gate_ms, 1)
                out["fallback_ok"] = bool(fb)
                out["abort_recommended"] = not bool(fb)
                if fb:
                    bad_fb = sum(
                        1 for xy in fb
                        if any(pip(xy, r) for r in rects))
                    out["fallback_inside_rule"] = bad_fb
                    out["fallback_compliant"] = (bad_fb == 0)
                    if self.exec_mode == "followpath":
                        r = self.execute_path(
                            fb, goal, exec_mode="followpath",
                            controller_id=self.controller_id)
                        out["fp_exec"] = {k: r.get(k) for k in
                                          ("executed", "arrived", "exec_t",
                                           "reason", "result_code")}
                        out["exec_mode"] = "followpath"
                        out["controller"] = self.controller_id
                    else:
                        self.publish_path(fb)
                        out["exec_mode"] = "pp"
            # No dead 150 s waits: only wait for arrival when a path was
            # actually commanded (accepted repair or working fallback).
            # Otherwise nothing moves by design (abstain/all-fail-safe-stop
            # or unaffected nominal) and waiting the full timeout would
            # burn ~150 s per such case with zero information gain.
            if out.get("accepted") or out.get("fallback_ok"):
                t0 = time.time()
                _gg, _gp, _ge, _gh = getattr(self, "_g2d",
                                             (None, (), 0, ""))
                out["arrived"] = bool(self.wait_arrival(
                    gx, gy, guard=_gg, guard_polys=_gp))
                out["exec_t"] = round(time.time() - t0, 1)
                out.update({f"g2d_{k}": v for k, v in
                            getattr(self, "_last_guard", {}).items()})
            else:
                out["arrived"] = False
                out["exec_t"] = 0.0
                out["exec_skipped"] = "no-executable-path"
            bad = sum(1 for xy in self.track
                      if any(pip(self.to_map(xy), r) for r in rects))
            track_map = [self.to_map(xy) for xy in self.track]
            mind = (min(math.hypot(mx - gx, my - gy) for mx, my in track_map)
                    if track_map else None)
            out["track_n"] = len(self.track)
            out["track_inside_rule"] = bad
            out["rule_compliant"] = (bad == 0)
            out["min_goal_dist"] = (round(mind, 3)
                                    if mind is not None else None)
            # Proximity + physical contact measurement (Blocker C).
            # scan_min/contact are lidar-proximity proxies; contact_n
            # comes from the deterministic Gazebo contact sensor when
            # the stack provides /base_contact.
            out["scan_min_m"] = (round(self.scan_min, 3)
                                 if self.scan_min != float("inf") else None)
            out["scan_contacts"] = int(self.scan_contacts)
            out["contact_available"] = bool(self.contact_available)
            out["contact_n"] = int(self.contact_n)
            out["contact_first_t"] = (round(self.contact_first_t - t0, 2)
                                      if self.contact_first_t is not None
                                      else None)
            out["ok"] = True
            return out
        finally:
            # Guaranteed cleanup on ALL paths (manager merges commits;
            # without removal rules accumulate and topo refuses with
            # too-many-rules). Uses the up-front remove client (a client
            # created minutes after startup intermittently never
            # discovers its service). Verified via ListRules; failure is
            # recorded loudly on the row.
            try:
                from rulesync_interfaces.srv import RemoveRule as _RR2
                _rm_ok, _rm_detail = True, []
                if self._cli_rm.wait_for_service(timeout_sec=10.0):
                    for _j, _tid in enumerate(tids):
                        _fut = self._cli_rm.call_async(_RR2.Request(
                            rule_id=_tid,
                            epoch=int(epoch) + 3 + _j))
                        rclpy.spin_until_future_complete(self, _fut,
                                                         timeout_sec=15.0)
                        _rr = _fut.result()
                        _one = bool(_rr and getattr(_rr, "ok", False))
                        _rm_ok = bool(_rm_ok and _one)
                        _rm_detail.append(
                            f"{_tid}:{_one}")
                    try:
                        if self.cli_list.wait_for_service(
                                timeout_sec=10.0):
                            from rulesync_interfaces.srv import (
                                ListRules as _LRV)
                            _lf = self.cli_list.call_async(
                                _LRV.Request())
                            rclpy.spin_until_future_complete(
                                self, _lf, timeout_sec=10.0)
                            _lr = _lf.result()
                            if _lr is not None:
                                _active = {
                                    getattr(r, "rule_id", "")
                                    for r in getattr(_lr, "active", [])}
                                _gone = _active.isdisjoint(set(tids))
                                _rm_ok = bool(_rm_ok and _gone)
                                _rm_detail.append(f"verified_gone:{_gone}")
                    except Exception as _ve:
                        _rm_detail.append(f"verify_error:{_ve}"[:80])
                else:
                    _rm_ok = False
                    _rm_detail.append("remove_rule service unavailable")
                out["cleanup_removed"] = bool(_rm_ok)
                out["cleanup_detail"] = str(_rm_detail)
                if not _rm_ok:
                    print(f"[driver] WARNING: cleanup failed epoch={epoch} "
                          f"detail={_rm_detail}", flush=True)
            except Exception as _e:
                out["cleanup_removed"] = False
                out["cleanup_error"] = str(_e)[:100]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/home/yule/rulepatch/gz_cases.csv")
    ap.add_argument("--cases", default=None,
                    help="cases.json {epoch: {goal, rule}} from "
                    "prescore_gazebo; overrides auto placement")
    ap.add_argument("--single-goal", default=None,
                    help="x,y: run ONE case at this goal (real robot)")
    ap.add_argument("--single-epoch", type=int, default=201)
    ap.add_argument("--rect-half", type=float, default=None,
                    help="override Rect half-size (m) for the single case")
    ap.add_argument("--no-rule", action="store_true",
                    help="baseline: no rule commit, plan+execute only")
    ap.add_argument("--exec", default="pp", choices=["pp", "followpath"],
                    help="execution backend (followpath uses the Nav2 "
                    "controller under test)")
    ap.add_argument("--controller", default="",
                    help="controller_id for FollowPath goal")
    ap.add_argument("--planner", default="GridBased",
                    help="planner_id for compute_path_to_pose")
    ap.add_argument("--cases-file", default=None,
                    help="json list [{goal:[x,y], epoch:int, rule:[[x,y]]}] "
                    "overriding scripted CASES (paper matrix)")
    ap.add_argument("--guard2d", action="store_true",
                    help="RuleGuard 2D retrofit: monitor pp-track against "
                    "committed polygons (subset runs only; frozen 2D "
                    "behavior unchanged when off)")
    a = ap.parse_args()
    rclpy.init()
    node = Driver()
    node.exec_mode = a.exec
    node.controller_id = a.controller
    node.planner_id = a.planner
    node._guard2d = bool(a.guard2d)
    import os as _os2
    node.map_tag = _os2.environ.get("TOPO_MAP", "")
    import os as _os
    if _os.environ.get("TOPO_SPAWN"):
        node.spawn_xy = tuple(float(v) for v in _os.environ["TOPO_SPAWN"].split(","))
    if not node.ac.wait_for_server(timeout_sec=30.0):
        print("[topo-gz] FATAL: no compute_path_to_pose", flush=True)
        return
    for name, cli in (("propose", node.cli_propose),
                      ("commit", node.cli_commit)):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f"[topo-gz] FATAL: /rulesync/{name} unavailable",
                  flush=True)
            return
    rows = []
    fixed = {}
    if a.cases:
        import json as _json
        fixed = {str(c["epoch"]): c for c in _json.load(open(a.cases))}
    if a.single_goal:
        gx, gy = (float(v) for v in a.single_goal.split(","))
        global RECT_HALF
        if a.rect_half is not None:
            RECT_HALF = float(a.rect_half)
        print(f"[topo] single epoch={a.single_epoch} goal=({gx},{gy}) "
              f"no_rule={a.no_rule}", flush=True)
        try:
            if a.no_rule:
                row = node.run_baseline((gx, gy), a.single_epoch)
            else:
                row = node.run_case((gx, gy), a.single_epoch)
        except Exception as e:
            import traceback
            traceback.print_exc()
            row = {"goal": (gx, gy), "epoch": a.single_epoch,
                   "ok": False, "reason": f"error: {e}",
                   "traceback": traceback.format_exc()[-300:]}
        print(f"[topo] result: {row}", flush=True)
        rows.append(row)
    else:
        _cases = list(CASES)
        if a.cases_file:
            import json as _jf
            _cases = [((c["goal"][0], c["goal"][1]), c["epoch"],
                       c.get("uid", ""))
                      for c in _jf.load(open(a.cases_file))]
        else:
            _cases = [(gx, gy, ep, "") for (gx, gy), ep in _cases]
        for (gx, gy), epoch, _uid in _cases:
            if str(epoch) in fixed:
                gx, gy = fixed[str(epoch)]["goal"]
            print(f"[topo-gz] case epoch={epoch} goal=({gx},{gy})",
                  flush=True)
            try:
                _fc = fixed.get(str(epoch), {})
                _frs = _fc.get("rules", None)
                if _frs is None and _fc.get("rule") is not None:
                    _frs = [_fc["rule"]]
                row = node.run_case((gx, gy), epoch,
                                    fixed_rules=_frs)
                if _uid:
                    row["uid"] = _uid
            except Exception as e:
                import traceback
                traceback.print_exc()
                row = {"goal": (gx, gy), "epoch": epoch, "ok": False,
                       "reason": f"error: {e}",
                       "traceback": traceback.format_exc()[-300:]}
                if _uid:
                    row["uid"] = _uid
            print(f"[topo-gz] result: {row}", flush=True)
            rows.append(row)
            # Crash-safe streaming: rewrite the CSV after every case so
            # a timeout/kill mid-campaign loses at most one row (the
            # physics campaign runs hours; buffered-only writes lose
            # everything on interruption).
            _dump_rows(a.csv, rows)
            # Fail loud on repeated cleanup failures (rules would
            # accumulate and later rows go no-status).
            if row.get("cleanup_removed") is False:
                _nbad = sum(1 for r in rows[-3:]
                            if r.get("cleanup_removed") is False)
                if _nbad >= 3:
                    print("[topo-gz] FATAL: 3 consecutive cleanup "
                          "failures — aborting campaign", flush=True)
                    break
    print(f"[topo-gz] wrote {a.csv}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


def _dump_rows(csv_path, rows):
    import csv as _csv
    with open(csv_path, "w", newline="") as f:
        keys = sorted({k for r in rows for k in r.keys()})
        w = _csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})


if __name__ == "__main__":
    main()
