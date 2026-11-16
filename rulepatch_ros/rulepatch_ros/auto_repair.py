"""auto_repair: post-commit plan validation + rollback (M1 blocker fixes).

Watches /rulesync/staged_set commit events:
  1. auto replan after commit (programmatic ComputePathToPose, no manual resend)
  2. post-commit plan validation (local geometric check vs active KEEP_OUTs)
  3. rollback on persistent planning failure (remove_rule newest, JSON log)

Params: max_failures (default 3), rollback_log (/tmp/rulepatch_rollback.log).
Pubs: /rulepatch/auto_plan (Path), /rulepatch/auto_status (String JSON).
"""
import json
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from rulesync_interfaces.msg import StagedSet
from rulesync_interfaces.srv import ListRules, RemoveRule

from rulepatch_ros.nav2_plan_client import Nav2Planner


def pip(xy, pts) -> bool:
    x, y = float(xy[0]), float(xy[1])
    inside, n = False, len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if ((y0 > y) != (y1 > y)) and (
                x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0):
            inside = not inside
    return inside


def validate_path(path, keepouts, occ_fn=None) -> tuple:
    """Local post-commit check. Returns (ok, reason)."""
    if not path or len(path) < 2:
        return False, "empty path"
    for poly in keepouts:
        for xy in path:
            if pip(xy, poly):
                return False, "KEEP_OUT violation"
    if occ_fn is not None:
        for xy in path[::5]:
            if occ_fn(*xy):
                return False, "collision"
    return True, "ok"


class AutoRepair(Node):
    def __init__(self):
        super().__init__("rulepatch_auto_repair")
        self.declare_parameter("max_failures", 3)
        self.declare_parameter("plan_timeout", 30.0)
        self.declare_parameter("rollback_log", "/tmp/rulepatch_rollback.log")
        self.declare_parameter("planner_action", "compute_path_to_pose")
        qos_vol = QoSProfile(depth=10,
                             durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.RELIABLE)
        qos_latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self._goal = None
        self._map = None
        self._fails = 0
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map,
                                 qos_latched)
        self.create_subscription(StagedSet, "/rulesync/staged_set",
                                 self._on_staged, qos_vol)
        self.plan_pub = self.create_publisher(Path, "/rulepatch/auto_plan", 10)
        self.status_pub = self.create_publisher(
            String, "/rulepatch/auto_status", 10)
        self.cli_list = self.create_client(ListRules, "/rulesync/list_rules")
        self.cli_remove = self.create_client(RemoveRule, "/rulesync/remove_rule")
        self.planner = Nav2Planner(
            self, str(self.get_parameter("planner_action").value))
        # Same transient-burst caveat as inference: poll committed epochs.
        self._last_seen_epoch = 0
        self._listing = False
        self.create_timer(3.0, self._poll_tick)
        self.get_logger().info("rulepatch_auto_repair ready")

    def _poll_tick(self):
        if self._listing or self._goal is None:
            return
        if not self.cli_list.wait_for_service(timeout_sec=1.0):
            return
        self._listing = True
        fut = self.cli_list.call_async(ListRules.Request())
        fut.add_done_callback(self._on_listed)

    def _on_listed(self, fut):
        self._listing = False
        try:
            res = fut.result()
            if res is None:
                return
            if int(res.epoch) > self._last_seen_epoch:
                self._last_seen_epoch = int(res.epoch)
                self.after_commit(note=f"epoch={res.epoch}")
        except Exception as e:
            self.get_logger().warning(f"list poll failed: {e}")

    def _on_goal(self, msg):
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_map(self, msg):
        self._map = msg

    def occ_fn(self):
        if self._map is None:
            return None
        m, w = self._map, self._map.info.width
        data = list(m.data)
        res, ox, oy = m.info.resolution, m.info.origin.position.x, \
            m.info.origin.position.y

        def fn(x, y):
            ix, iy = int((x - ox) / res), int((y - oy) / res)
            if 0 <= ix < w and 0 <= iy < len(data) // w:
                return data[iy * w + ix] > 50
            return True
        return fn

    def _on_staged(self, msg):
        if str(msg.op) != "commit" or self._goal is None:
            return
        self.after_commit(note=f"epoch={msg.epoch}")

    def active_keepouts(self):
        if not self.cli_list.wait_for_service(timeout_sec=10.0):
            return []
        fut = self.cli_list.call_async(ListRules.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            return []
        rules = list(res.active)
        epoch = int(res.epoch)
        polys = [[(p.x, p.y) for p in r.polygon.points] for r in rules
                 if int(r.rule_type) == 0]
        return polys, rules, epoch

    def after_commit(self, note=""):
        status = {"note": note}
        try:
            keepouts, rules, epoch = self.active_keepouts()
            gx, gy = self._goal
            sx, sy = self.robot_xy()
            if not self.planner.wait(timeout=15.0):
                raise RuntimeError("planner unavailable")
            path, dt = self.planner.plan(
                (sx, sy), (gx, gy),
                timeout=float(self.get_parameter("plan_timeout").value))
            if path is None:
                return self.on_fail(status, "plan failed (empty)")
            ok, reason = validate_path(path, keepouts, self.occ_fn())
            if not ok:
                return self.on_fail(status, f"validation: {reason}")
            self._fails = 0
            self.publish_path(path)
            status.update(ok=True, reason="ok", n_poses=len(path),
                          plan_s=round(dt, 3))
        except Exception as e:
            return self.on_fail(status, f"error: {e}")
        self.publish_status(status)
        return status

    def on_fail(self, status, reason):
        self._fails += 1
        status.update(ok=False, reason=reason, fails=self._fails)
        if self._fails >= int(self.get_parameter("max_failures").value):
            status["rollback"] = self.do_rollback()
            self._fails = 0
        self.publish_status(status)
        return status

    def do_rollback(self):
        """Remove newest active rule as a new epoch; log JSON line."""
        try:
            keepouts, rules, epoch = self.active_keepouts()
            if not rules:
                return {"ok": False, "reason": "nothing active"}
            newest = max(rules, key=lambda r: int(r.epoch))
            rid = str(newest.rule_id)
            if not self.cli_remove.wait_for_service(timeout_sec=10.0):
                return {"ok": False, "reason": "remove service unavailable"}
            req = RemoveRule.Request(rule_id=rid, epoch=epoch + 1)
            fut = self.cli_remove.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
            ok = bool(fut.result().ok) if fut.result() else False
            rec = {"t_wall": time.time(), "op": "auto-rollback",
                   "rule_id": rid, "epoch": epoch + 1, "ok": ok}
            with open(str(self.get_parameter("rollback_log").value),
                      "a") as f:
                f.write(json.dumps(rec) + "\n")
            self.get_logger().info(f"auto-rollback {rid} ok={ok}")
            return {"ok": ok, "rule_id": rid}
        except Exception as e:
            return {"ok": False, "reason": str(e)}

    def robot_xy(self):
        from nav_msgs.msg import Odometry as _Od
        box = {}
        sub = self.create_subscription(
            _Od, "/odom", lambda m: box.__setitem__(
                "xy", (float(m.pose.pose.position.x),
                       float(m.pose.pose.position.y))), 10)
        t0 = time.time()
        while "xy" not in box and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        return box.get("xy", (0.0, 0.0))

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.plan_pub.publish(msg)

    def publish_status(self, status):
        m = String()
        m.data = json.dumps(status)
        self.status_pub.publish(m)
        self.get_logger().info(f"auto_repair: {m.data[:250]}")


def main():
    rclpy.init()
    node = AutoRepair()
    ex = rclpy.executors.MultiThreadedExecutor()
    try:
        ex.add_node(node)
        ex.spin()
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
