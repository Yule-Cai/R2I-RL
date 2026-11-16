"""RulePatchNet Gazebo demo driver (onexplayer, domain 0 full stack).

Modes:
  accept   : commit crossing KEEP_OUT -> inference patch accepted ->
             cancel Nav2 goal -> pp_driver tracks repaired path -> goal.
  fallback : commit oversized KEEP_OUT (span > window) -> Nav2 full-replan
             fallback -> pp_driver tracks -> goal.
  rollback : commit corridor-blocking KEEP_OUT -> plans fail ->
             auto_repair rolls back (remove_rule) -> status logged.

Usage:
  source /opt/ros/lyrical/setup.bash; source ~/rulesync_ws/install/setup.bash
  export PYTHONPATH=/home/yule/R2I-RL-checkout/rulepatch:$PYTHONPATH
  python3 demo_rulepatch_client.py --mode accept --goal-x 4.0 --goal-y 1.0 ...
"""
import argparse
import csv
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import FollowPath

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import (ProposeRule, CommitRule, RemoveRule,
                                     ListRules)


def make_pose(x, y, frame="map"):
    m = PoseStamped()
    m.header.frame_id = frame
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.orientation.w = 1.0
    return m


def stamp_now():
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = int(time.time())
    return t


class Demo(Node):
    def __init__(self, csv_path, drive="pp"):
        super().__init__("rulepatch_demo_client")
        self.rows = []
        self.csv_path = csv_path
        self.drive = drive
        self.nav = None
        self.follow = None
        if drive == "followpath":
            self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.follow = ActionClient(self, FollowPath, "follow_path")
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cli_propose = self.create_client(ProposeRule, "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule, "/rulesync/commit_rule")
        self.cli_remove = self.create_client(RemoveRule, "/rulesync/remove_rule")
        self.cli_list = self.create_client(ListRules, "/rulesync/list_rules")
        for n, c in (("propose", self.cli_propose), ("commit", self.cli_commit),
                     ("remove", self.cli_remove), ("list", self.cli_list)):
            for _ in range(10):
                if c.wait_for_service(timeout_sec=15.0):
                    break
            else:
                raise RuntimeError(f"/rulesync/{n} unavailable")
        if drive == "followpath":
            # NavigateToPose discovery flakes for short-lived clients; retry.
            for _ in range(10):
                if self.nav.wait_for_server(timeout_sec=20.0):
                    break
            else:
                raise RuntimeError("NavigateToPose unavailable")
        self.status = None
        self.auto_status = None
        self.repaired = None
        self.create_subscription(String, "/rulepatch/status",
                                 lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(String, "/rulepatch/auto_status",
                                 lambda m: setattr(self, "auto_status", m.data),
                                 10)
        self.create_subscription(Path, "/rulepatch/repaired_path",
                                 self._on_repaired, 10)
        self.repaired_pub = self.create_publisher(
            Path, "/rulepatch/repaired_path", 10)
        self.odom_xy = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        # Persistent sim-time seed publisher: transient initialpose bursts
        # are missed under slow discovery; republish until localized.
        from rclpy.parameter import Parameter as _P
        self._seed_node = rclpy.create_node(
            "rp_seed_persistent",
            parameter_overrides=[_P("use_sim_time", value=True)])
        self._seed_pub = self._seed_node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self._amcl = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda m: setattr(self, "_amcl", True), 10)

    def set_initial_pose(self, x=-2.0, y=-0.5, repeats=4):
        # Publish with SIM-TIME stamps (stamp 0 -> AMCL extrapolation failure
        # and no map->odom TF). Node persists; callers republish until
        # localized (see wait_localized).
        for _ in range(repeats):
            m = PoseWithCovarianceStamped()
            m.header.stamp = self._seed_node.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
            m.pose.pose.orientation.w = 1.0
            m.pose.covariance[0] = m.pose.covariance[7] = 0.25
            m.pose.covariance[35] = 0.0685
            self._seed_pub.publish(m)
            time.sleep(0.5)
        self.log(event="initialpose_sent", x=x, y=y)

    def wait_localized(self, timeout=120.0):
        t0 = time.time()
        last_repub = 0.0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            rclpy.spin_once(self._seed_node, timeout_sec=0.1)
            if time.time() - last_repub > 5.0:
                self.set_initial_pose()
                last_repub = time.time()
            if self._amcl:
                self.log(event="localized")
                return True
        return False

    def wait_localized(self, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self._amcl:
                self.log(event="localized")
                return True
        return False

    def _on_repaired(self, msg):
        self.repaired = [(p.pose.position.x, p.pose.position.y)
                         for p in msg.poses]

    def _on_odom(self, msg):
        self.odom_xy = (float(msg.pose.pose.position.x),
                        float(msg.pose.pose.position.y))

    def log(self, **kw):
        kw.setdefault("t_wall", time.time())
        self.rows.append(kw)

    def jiggle(self, seconds=4.0, ang=0.4):
        """Open-loop in-place rotation (TwistStamped for the gz bridge).

        Forces AMCL laser updates (motion-gated) so map->odom TF appears.
        No TF needed to execute. Safe: pure rotation, no translation.
        """
        from geometry_msgs.msg import TwistStamped as _TS
        pub = self.create_publisher(_TS, "/cmd_vel", 10)
        t0 = time.time()
        while time.time() - t0 < seconds:
            m = _TS()
            m.twist.angular.z = float(ang)
            pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.1)
        m = _TS()
        pub.publish(m)
        self.destroy_publisher(pub)
        self.log(event="jiggle_done")
        time.sleep(1.0)

    def drive_initial(self, gx, gy, timeout=120.0):
        """PP mode: plan via ComputePathToPose, publish for pp_driver."""
        from nav2_msgs.action import ComputePathToPose as _CPP
        from rclpy.action import ActionClient as _AC
        ac = _AC(self, _CPP, "compute_path_to_pose")
        if not ac.wait_for_server(timeout_sec=30.0):
            self.log(event="drive_initial", ok=False, reason="no-server")
            return False
        mxy = self.robot_map_xy()
        if mxy is None:
            self.log(event="drive_initial", ok=False, reason="no-tf")
            return False
        goal = _CPP.Goal()
        goal.start = make_pose(*mxy)
        goal.goal = make_pose(gx, gy)
        goal.planner_id = "GridBased"
        goal.use_start = True
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.log(event="drive_initial", ok=False, reason="rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
        res = rf.result()
        try:
            path = [(p.pose.position.x, p.pose.position.y)
                    for p in res.result.path.poses]
        except Exception:
            self.log(event="drive_initial", ok=False, reason="exc")
            return False
        if len(path) < 5:
            self.log(event="drive_initial", ok=False, reason=f"short:{len(path)}")
            return False
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.repaired_pub.publish(msg)
        self.goal_pub.publish(make_pose(gx, gy))  # inference needs /goal_pose
        self._last_goal_xy = (float(gx), float(gy))
        self.log(event="initial_plan", n_poses=len(path))
        return True

    def send_goal(self, x, y):
        gp = PoseStamped()
        gp.header.frame_id = "map"
        gp.pose.position.x, gp.pose.position.y = float(x), float(y)
        gp.pose.orientation.w = 1.0
        self.goal_pub.publish(gp)
        self._last_goal_xy = (float(x), float(y))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.w = 1.0
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        h = fut.result()
        self.log(event="goal_sent", accepted=bool(h.accepted))
        return h

    def wait_plannable(self, gx, gy, timeout=180.0):
        """Gate the scenario on real planner readiness (costmap warmup).

        Start pose comes from TF map->base_link (odom coords are NOT map
        coords). Requires a valid TF (BT planning needs it too).
        """
        from nav2_msgs.action import ComputePathToPose as _CPP
        from rclpy.action import ActionClient as _AC
        import tf2_ros as _tf2
        ac = _AC(self, _CPP, "compute_path_to_pose")
        if not ac.wait_for_server(timeout_sec=30.0):
            return False
        if not hasattr(self, "_tf_buffer"):
            self._tf_buffer = _tf2.Buffer()
            _tf2.TransformListener(self._tf_buffer, self)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                from rclpy.time import Time as _T
                t = self._tf_buffer.lookup_transform(
                    "map", "base_link", _T())
                sx = float(t.transform.translation.x)
                sy = float(t.transform.translation.y)
            except Exception:
                time.sleep(2.0)
                continue
            goal = _CPP.Goal()
            goal.start = make_pose(sx, sy)
            goal.goal = make_pose(gx, gy)
            goal.planner_id = "GridBased"
            goal.use_start = True
            fut = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
            gh = fut.result()
            if gh is None or not gh.accepted:
                time.sleep(2.0)
                continue
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
            r = rf.result()
            try:
                n = len(r.result.path.poses)
            except Exception:
                n = 0
            if n > 5:
                self.log(event="plannable", n_poses=n)
                return True
            time.sleep(3.0)
        return False

    def wait_odom_move(self, timeout=150.0):
        t0 = time.time()
        x0 = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.odom_xy is None:
                continue
            if x0 is None:
                x0 = self.odom_xy
            if abs(self.odom_xy[0] - x0[0]) + abs(self.odom_xy[1] - x0[1]) > 0.15:
                self.log(event="moving", xy=self.odom_xy)
                return True
        return False

    def _repudiate_goal(self):
        # Re-publish latched-style goal for late-joining subscribers
        # (/goal_pose is transient; discovery can miss the first burst).
        if getattr(self, "_last_goal_xy", None) is not None:
            gp = PoseStamped()
            gp.header.frame_id = "map"
            gp.pose.position.x, gp.pose.position.y = self._last_goal_xy
            gp.pose.orientation.w = 1.0
            self.goal_pub.publish(gp)

    def wait_status(self, timeout=90.0):
        t0 = time.time()
        last_repub = 0.0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - last_repub > 2.0:
                self._repudiate_goal()
                last_repub = time.time()
            if self.status is not None:
                s = json.loads(self.status)
                self.status = None
                self.log(event="rulepatch_status", **{k: s.get(k) for k in
                         ("accepted", "fallback", "reason", "hyp", "conf",
                          "latency_ms", "fallback_ok")})
                return s
        return None

    def robot_map_xy(self, timeout=15.0):
        """Robot pose in MAP frame via TF (None if unavailable)."""
        try:
            import tf2_ros as _tf2
            if not hasattr(self, "_tf_buffer"):
                self._tf_buffer = _tf2.Buffer()
                _tf2.TransformListener(self._tf_buffer, self)
            from rclpy.time import Time as _T
            t0 = time.time()
            while time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.2)
                try:
                    t = self._tf_buffer.lookup_transform(
                        "map", "base_link", _T())
                    return (float(t.transform.translation.x),
                            float(t.transform.translation.y))
                except Exception:
                    time.sleep(0.5)
            return None
        except Exception:
            return None

    def follow_repaired(self, timeout=150.0):
        """Trim repaired path at nearest odom point, track via Nav2 FollowPath."""
        t0 = time.time()
        while self.repaired is None and time.time() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.repaired is None:
            self.log(event="follow_nopath")
            return False
        mxy = self.robot_map_xy()
        if mxy is None:
            self.log(event="follow_notf")
            return False
        import numpy as np
        p = np.array(self.repaired)
        j = int(np.argmin(np.hypot(p[:, 0] - mxy[0], p[:, 1] - mxy[1])))
        seg = [tuple(q) for q in p[j:]]
        if not self.follow.wait_for_server(timeout_sec=20.0):
            self.log(event="follow_unavailable")
            return False
        goal = FollowPath.Goal()
        goal.path.header.frame_id = "map"
        for x, y in seg:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            goal.path.poses.append(ps)
        goal.controller_id = ""
        fut = self.follow.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.log(event="follow_rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        res = rf.result()
        ok = res is not None and int(res.status) == 4  # SUCCEEDED
        self.log(event="follow_done", ok=ok,
                 status=int(res.status) if res else -1)
        return ok

    def wait_near(self, x, y, timeout=150.0, tol=0.6):
        t0 = time.time()
        while time.time() - t0 < timeout:
            mxy = self.robot_map_xy()
            if mxy and abs(mxy[0] - x) < tol and abs(mxy[1] - y) < tol:
                self.log(event="goal_reached", xy=mxy)
                return True
            time.sleep(0.5)
        self.log(event="goal_timeout", xy=self.robot_map_xy())
        return False

    def current_plan(self, timeout=30.0):
        # PP mode: the live path is what we published for pp_driver.
        if getattr(self, "repaired", None):
            return list(self.repaired)
        from nav_msgs.msg import Path as _Path
        box = {}
        sub = self.create_subscription(
            _Path, "/plan", lambda m: box.__setitem__(
                "path", [(p.pose.position.x, p.pose.position.y)
                         for p in m.poses]), 10)
        t0 = time.time()
        while not box and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        return box.get("path")

    @staticmethod
    def _pip(xy, pts) -> bool:
        x, y = float(xy[0]), float(xy[1])
        inside, n = False, len(pts)
        for i in range(n):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % n]
            if ((y0 > y) != (y1 > y)) and (
                    x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0):
                inside = not inside
        return inside

    def pick_zone(self, live, w, h, frac, laterals=(0.0, 0.15, -0.15, 0.3,
                                                   -0.3),
                  fracs=None):
        """First candidate zone crossed by >=5 live-path poses."""
        for fr in (fracs or (frac,)):
            for lat in laterals:
                zone = self.zone_from_plan(live, w=w, h=h, frac=fr,
                                           lateral=lat)
                n = sum(1 for p in live if self._pip(p, zone))
                self.log(event="zone_candidate", lateral=lat, inside=n)
                if n >= 5:
                    return zone
        return self.zone_from_plan(live, w=w, h=h, frac=frac, lateral=0.0)

    def zone_from_plan(self, path, w=0.8, h=1.0, frac=0.5, lateral=0.0):
        """KEEP_OUT rect near the live path (guaranteed affected).

        lateral: offset (m) along the path normal; 0 = centered (hardest),
        ~half-width = grazing clip (easier, still invalidates the path).
        """
        import math
        i = min(len(path) - 1, max(1, int(len(path) * frac)))
        cx, cy = path[i]
        j0, j1 = max(0, i - 5), min(len(path) - 1, i + 5)
        dx = path[j1][0] - path[j0][0]
        dy = path[j1][1] - path[j0][1]
        L = math.hypot(dx, dy) + 1e-9
        cx += -dy / L * lateral
        cy += dx / L * lateral
        return [(cx - w / 2, cy - h / 2), (cx - w / 2, cy + h / 2),
                (cx + w / 2, cy + h / 2), (cx + w / 2, cy - h / 2)]

    def rule(self, rid, epoch, pts):
        r = RuleSpec()
        r.rule_id, r.epoch, r.rule_type, r.severity = rid, int(epoch), 0, 1
        r.frame_id = "map"
        r.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                            for x, y in pts]
        r.speed_limit = 0.0
        r.valid_from = stamp_now()
        end = stamp_now()
        end.sec += 3600
        r.valid_until = end
        return r

    def base_epoch(self):
        fut = self.cli_list.call_async(ListRules.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return int(fut.result().epoch)

    def propose_commit(self, rule):
        rq = ProposeRule.Request(rule=rule)
        fut = self.cli_propose.call_async(rq)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        if not fut.result().accepted:
            self.log(event="propose_rejected", reason=str(fut.result().reason))
            return False
        fut2 = self.cli_commit.call_async(CommitRule.Request(epoch=int(rule.epoch)))
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=90.0)
        ok = bool(fut2.result().committed)
        self.log(event="committed", rule_id=rule.rule_id, ok=ok)
        return ok

    def remove(self, rid, epoch):
        fut = self.cli_remove.call_async(RemoveRule.Request(rule_id=rid,
                                                            epoch=int(epoch)))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=90.0)
        ok = bool(fut.result().ok)
        self.log(event="removed", rule_id=rid, ok=ok)
        return ok

    def save(self):
        keys = sorted({k for row in self.rows for k in row})
        with open(self.csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)
        print(f"[demo] csv: {self.csv_path} ({len(self.rows)} rows)")


def rect(x1, y1, x2, y2):
    return [(x1, y1), (x1, y2), (x2, y2), (x2, y1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["accept", "fallback", "rollback"],
                    default="accept")
    ap.add_argument("--goal-x", type=float, default=2.23)
    ap.add_argument("--goal-y", type=float, default=-0.88)
    ap.add_argument("--keepout", type=float, nargs=4,
                    default=[2.0, 0.5, 3.0, 1.5],
                    help="M1-proven navigable zone (south detour open)")
    ap.add_argument("--zone-frac", type=float, default=0.5,
                    help="fraction along live path for auto zone center")
    ap.add_argument("--zone-wh", type=float, nargs=2, default=[0.8, 1.0],
                    help="auto zone width/height (m)")
    ap.add_argument("--zone-lateral", type=float, default=0.0,
                    help="auto zone lateral offset (m); grazing clip if ~half width")
    ap.add_argument("--zone-static", action="store_true",
                    help="use --keepout rect as-is (pre-scored offline)")
    ap.add_argument("--csv", default="/tmp/rulepatch_demo.csv")
    ap.add_argument("--status-json", default="/tmp/rulepatch_demo_status.json")
    ap.add_argument("--seed", action="store_true",
                    help="reset AMCL to spawn pose (fresh sim only)")
    ap.add_argument("--warmup", type=float, default=60.0,
                    help="passive spin seconds after init (discovery settle)")
    ap.add_argument("--drive", choices=["pp", "followpath"], default="pp",
                    help="pp: track via pp_driver (minimal stack); followpath: "
                    "Nav2 FollowPath (full stack)")
    args = ap.parse_args()
    import faulthandler as _fh
    _fh.dump_traceback_later(600, exit=False)
    rclpy.init()
    demo = Demo(args.csv, drive=args.drive)
    statuses = []
    if args.warmup > 0:
        print(f"[demo] warmup {args.warmup}s for discovery...",
              flush=True)
        t0 = time.time()
        while time.time() - t0 < args.warmup:
            rclpy.spin_once(demo, timeout_sec=0.5)
    try:
        e0 = demo.base_epoch()
        if args.seed:
            demo.set_initial_pose()
            assert demo.wait_localized(), "AMCL never localized"
        # Start motion. PP mode: plan + publish for pp_driver (no BT).
        # FollowPath mode: NavigateToPose immediately (within AMCL TF
        # lifetime ~10s while stationary).
        h = None
        if args.drive == "pp":
            demo.jiggle()
            assert demo.drive_initial(args.goal_x, args.goal_y), \
                "initial plan failed"
        else:
            for _ in range(4):
                h = demo.send_goal(args.goal_x, args.goal_y)
                if h is not None and bool(getattr(h, "accepted", False)):
                    break
                time.sleep(3.0)
            assert h is not None and bool(getattr(h, "accepted", False)), \
                "goal never accepted"
        if args.mode != "rollback":
            assert demo.wait_odom_move(), "robot never started moving"
        if args.mode == "accept":
            live = demo.current_plan(timeout=30.0)
            if live and len(live) > 20 and not args.zone_static:
                lats = (args.zone_lateral, 0.0, 0.15, -0.15, 0.3, -0.3)
                zone = demo.pick_zone(
                    live, w=args.zone_wh[0], h=args.zone_wh[1],
                    frac=args.zone_frac, laterals=lats)
                demo.log(event="zone_auto", zone=zone)
            else:
                kx1, ky1, kx2, ky2 = args.keepout
                zone = rect(kx1, ky1, kx2, ky2)
                demo.log(event="zone_static", zone=zone)
            assert demo.propose_commit(demo.rule("rp_keepout", e0 + 1, zone))
            if args.drive == "followpath":
                fut = h.cancel_goal_async()
                rclpy.spin_until_future_complete(demo, fut, timeout_sec=10.0)
            s = demo.wait_status()
            assert s is not None, "no rulepatch status"
            statuses.append(s)
            if args.drive == "followpath":
                followed = demo.follow_repaired() if s.get("accepted") else False
            else:
                followed = True  # pp_driver auto-tracks repaired_path
            ok = demo.wait_near(args.goal_x, args.goal_y, timeout=120.0)
            print(f"[demo] accept: accepted={s.get('accepted')} "
                  f"followed={followed} reached={ok}")
            demo.remove("rp_keepout", e0 + 2)
        elif args.mode == "fallback":
            # Zone the learned patch cannot solve (large) but global Nav2
            # replan can: span guard or all-fail -> fallback replan.
            live = demo.current_plan(timeout=30.0)
            if live and len(live) > 20:
                zone = demo.zone_from_plan(live, w=3.0, h=2.0, frac=0.5,
                                           lateral=0.0)
                demo.log(event="zone_auto", zone=zone)
            else:
                zone = rect(-2.0, -3.0, 8.0, 3.0)
            assert demo.propose_commit(demo.rule("rp_big", e0 + 1, zone))
            if args.drive == "followpath":
                fut = h.cancel_goal_async()
                rclpy.spin_until_future_complete(demo, fut, timeout_sec=10.0)
            s = demo.wait_status()
            assert s is not None, "no rulepatch status"
            statuses.append(s)
            print(f"[demo] fallback={s.get('fallback')} ok={s.get('fallback_ok')}")
            if args.drive == "followpath":
                followed = demo.follow_repaired() if s.get("fallback_ok") else False
            else:
                followed = True
            ok = demo.wait_near(args.goal_x, args.goal_y, timeout=120.0)
            print(f"[demo] fallback followed={followed} reached={ok}")
            demo.remove("rp_big", e0 + 2)
        else:
            # corridor-blocking zone -> bridge auto-rollback (remove as new
            # epoch) after persistent planning failure. Poll list_rules.
            kx1, ky1, kx2, ky2 = args.keepout
            assert demo.propose_commit(demo.rule("rp_wall", e0 + 1,
                                                 rect(kx1, ky1, kx2, ky2)))
            from rulesync_interfaces.srv import ListRules as _LR
            t0 = time.time()
            rolled_back = False
            while time.time() - t0 < 240.0:
                rclpy.spin_once(demo, timeout_sec=1.0)
                fut = demo.cli_list.call_async(_LR.Request())
                rclpy.spin_until_future_complete(demo, fut, timeout_sec=10.0)
                res = fut.result()
                if res is not None:
                    active = [r.rule_id for r in res.active]
                    if "rp_wall" not in active:
                        rolled_back = True
                        demo.log(event="rollback_observed",
                                 epoch=int(res.epoch))
                        break
            statuses.append({"rollback": rolled_back})
            print(f"[demo] rollback seen: {rolled_back}")
            if h is not None:
                fut = h.cancel_goal_async()
                rclpy.spin_until_future_complete(demo, fut, timeout_sec=10.0)
    finally:
        json.dump(statuses, open(args.status_json, "w"), indent=1)
        demo.save()
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
