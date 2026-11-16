"""rulepatch_inference: KEEP_OUT commit -> learned local patch -> verifier.

Subs: /map, /plan (latest Nav2 plan), /goal_pose, /rulesync/staged_set.
Pubs: /rulepatch/repaired_path (Path), /rulepatch/status (String JSON).

Pipeline per commit carrying KEEP_OUT rules:
  old path crossing -> entry/exit anchors A/B (+margin) -> 128x128 BEV ->
  NumpyPatchNet (M=3) -> verifier (conf order) -> repaired path or fallback.
Fallback (span overflow or all-hypothesis fail) = standard Nav2 full replan
via ComputePathToPose (programmatic planning, M1 blocker fix).
"""
import json
import time

# rp/ (rulepatch core) is provided via PYTHONPATH, e.g.
#   export PYTHONPATH=/home/yule/R2I-RL-checkout/rulepatch:$PYTHONPATH
# set by the demo/bringup scripts before launching this node.

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from rulesync_interfaces.msg import StagedSet
from rulesync_interfaces.srv import ListRules

from rulepatch_ros.nav2_plan_client import Nav2Planner

BEV_PX, BEV_SIZE_M = 128, 12.8
MARGIN_M = 1.0


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


def crossing_anchors(path, poly, margin_m=3.0):
    """Entry/exit indices of path through polygon, widened by margin_m
    arc-length on each side (matches training divergence-span scale ~9m).
    """
    import math
    inside = [pip(p, poly) for p in path]
    if not any(inside):
        return None
    i0 = inside.index(True)
    i1 = len(inside) - 1 - inside[::-1].index(True)
    # walk back from entry
    acc, iA = 0.0, i0
    while iA > 0 and acc < margin_m:
        acc += math.hypot(path[iA][0] - path[iA - 1][0],
                           path[iA][1] - path[iA - 1][1])
        iA -= 1
    # walk forward from exit
    acc, iB = 0.0, i1
    while iB < len(path) - 1 and acc < margin_m:
        acc += math.hypot(path[iB + 1][0] - path[iB][0],
                           path[iB + 1][1] - path[iB][1])
        iB += 1
    return max(0, iA), min(len(path) - 1, iB)


class Inference(Node):
    def __init__(self):
        super().__init__("rulepatch_inference")
        self.declare_parameter("weights_npz", "")
        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("status_topic", "/rulepatch/status")
        self.declare_parameter("repaired_topic", "/rulepatch/repaired_path")
        qos_latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self._map = None
        self._plan = None
        self._goal = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map,
                                 qos_latched)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        # PP-drive mode publishes the live path here instead of /plan.
        self.create_subscription(Path, "/rulepatch/repaired_path",
                                 self._on_plan, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        qos_vol = QoSProfile(depth=10,
                             durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(StagedSet, "/rulesync/staged_set",
                                 self._on_staged, qos_vol)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("repaired_topic").value), 10)
        self.planner = Nav2Planner(
            self, str(self.get_parameter("planner_action").value))
        self.declare_parameter("anchor_margin_m", 3.0)
        self.net = None
        wp = str(self.get_parameter("weights_npz").value)
        if wp:
            self.load_weights(wp)
        # Reliable trigger: the manager's StagedSet topic bursts are
        # transient and easily missed; poll list_rules for new epochs.
        self.cli_list = self.create_client(ListRules, "/rulesync/list_rules")
        self._last_repaired_epoch = 0
        self._listing = False
        self._polls = 0
        self.create_timer(2.0, self._poll_tick)
        self.get_logger().info("rulepatch_inference ready")

    def _poll_tick(self):
        self._polls += 1
        if self._polls % 5 == 1:
            self.get_logger().info(
                f"poll#{self._polls} plan={self._plan is not None} "
                f"goal={self._goal is not None} map={self._map is not None} "
                f"last_ep={self._last_repaired_epoch} listing={self._listing}")
        if self._listing:
            return
        if not self.cli_list.wait_for_service(timeout_sec=1.0):
            if self._polls % 15 == 1:
                self.get_logger().warning("list_rules service undiscovered")
            return
        self._listing = True
        fut = self.cli_list.call_async(ListRules.Request())
        fut.add_done_callback(self._on_listed)

    def _on_listed(self, fut):
        self._listing = False
        try:
            res = fut.result()
            if res is None:
                self.get_logger().warning("list returned None")
                return
            epoch = int(res.epoch)
            n_active = len(res.active)
            if epoch != self._last_repaired_epoch or self._polls % 30 == 1:
                self.get_logger().info(
                    f"listed epoch={epoch} active={n_active}")
            if epoch <= self._last_repaired_epoch:
                return
            keepouts = []
            for r in res.active:
                if int(r.rule_type) == 0:
                    keepouts.append([(p.x, p.y) for p in r.polygon.points])
            if not keepouts or not self._plan or not self._goal \
                    or self._map is None:
                return
            self._last_repaired_epoch = epoch
            self.get_logger().info(
                f"new committed epoch {epoch}: repairing")
            self.repair(self._plan, keepouts[0], self._goal,
                        note=f"epoch={epoch}")
        except Exception as e:
            self.get_logger().warning(f"list poll failed: {e}")

    def load_weights(self, path):
        from rp.infer_numpy import NumpyPatchNet
        self.net = NumpyPatchNet(path)
        self.get_logger().info(f"weights loaded: {path}")

    def _on_map(self, msg):
        self._map = msg

    def _on_plan(self, msg):
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if path == getattr(self, "_last_published", None):
            return  # ignore our own repaired-path echo
        self._plan = path
        self.get_logger().info(f"plan rx n={len(self._plan)}")
        self.get_logger().info(f"plan rx n={len(self._plan)}")

    def _on_goal(self, msg):
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_staged(self, msg):
        self.get_logger().info(
            f"staged rx op={msg.op} epoch={msg.epoch} n_rules={len(msg.rules)}")
        if str(msg.op) != "commit":
            return
        keepouts = []
        for r in msg.rules:
            if int(r.rule_type) == 0:
                keepouts.append([(p.x, p.y) for p in r.polygon.points])
        if not keepouts:
            return
        if not self._plan or not self._goal or self._map is None:
            self.publish_status({"reason": "no-plan-yet",
                                 "has_plan": bool(self._plan),
                                 "has_goal": self._goal is not None,
                                 "has_map": self._map is not None})
            return
        self.repair(self._plan, keepouts[0], self._goal,
                    note=f"epoch={msg.epoch}")

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

    def repair(self, path, rule_poly, goal, note=""):
        from rp.bev import (render_bev, window_for_patch, local_path_points)
        from rp.verifier import select_hypothesis, decode_ctrl
        t0 = time.time()
        status = {"note": note, "fallback": False, "accepted": False}
        try:
            span = crossing_anchors(
                path, rule_poly,
                margin_m=float(
                    self.get_parameter("anchor_margin_m").value))
            if span is None:
                status.update(reason="unaffected-path",
                              latency_ms=(time.time() - t0) * 1000)
                self.publish_status(status)
                return status
            iA, iB = span
            A, B = tuple(path[iA]), tuple(path[iB])
            if max(abs(A[0] - B[0]), abs(A[1] - B[1])) > BEV_SIZE_M - 2.0:
                return self.fallback(path, rule_poly, goal, status, t0,
                                     reason="span-overflow")
            cx = (A[0] + B[0]) / 2.0
            cy = (A[1] + B[1]) / 2.0
            center = (cx, cy)
            seg = path[iA:iB + 1]
            bev = render_bev(self.occ_fn(), center, seg, rule_poly, goal)
            from rp.bev import world_to_norm  # noqa
            import numpy as np
            local = local_path_points(seg, center)
            ms = (time.time() - t0) * 1000
            from rp.bev import add_coords  # numpy-only (no torch onboard)
            c, f = self.net.forward(
                add_coords(bev.astype(np.float64)),
                local.astype(np.float64))
            fwd_ms = (time.time() - t0) * 1000 - ms
            bounds = self.map_bounds()
            idx, dense, reason = select_hypothesis(
                c, f, center, A, B, seg, rule_poly, self.occ_fn(), bounds)
            per_hyp = []
            for k in range(len(f)):
                from rp.verifier import verify_patch
                okk, rr = verify_patch(c[k], center, A, B, seg, rule_poly,
                                       self.occ_fn(), bounds)
                per_hyp.append({"hyp": k, "conf": float(f[k]), "ok": bool(okk),
                                "reason": rr})
            status["hypotheses"] = per_hyp
            status["ctrls"] = np.asarray(c, dtype=np.float64).round(4).tolist()
            status["confs"] = [float(v) for v in f]
            status["center"] = [float(center[0]), float(center[1])]
            status["A"] = [float(A[0]), float(A[1])]
            status["B"] = [float(B[0]), float(B[1])]
            if idx is None:
                return self.fallback(path, rule_poly, goal, status, t0,
                                     reason=reason)
            full = path[:iA + 1] + [tuple(p) for p in dense.tolist()] \
                + path[iB:]
            self.publish_path(full)
            status.update(accepted=True, hyp=int(idx), conf=float(f[idx]),
                          reason="ok", iA=iA, iB=iB,
                          latency_ms=(time.time() - t0) * 1000,
                          infer_ms=fwd_ms)
        except Exception as e:
            status.update(reason=f"error: {e}",
                          latency_ms=(time.time() - t0) * 1000)
        self.publish_status(status)
        return status

    def fallback(self, path, rule_poly, goal, status, t0, reason):
        start = tuple(path[0])
        if self.planner.wait(timeout=10.0):
            new_path, _ = self.planner.plan(start, goal, timeout=30.0)
        else:
            new_path = None
        if new_path:
            self.publish_path(new_path)
        status.update(fallback=True, reason=f"fallback:{reason}",
                      fallback_ok=bool(new_path),
                      latency_ms=(time.time() - t0) * 1000)
        self.publish_status(status)
        return status

    def map_bounds(self):
        m = self._map
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        return (ox, oy, ox + m.info.width * m.info.resolution,
                oy + m.info.height * m.info.resolution)

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._last_published = [(float(x), float(y)) for x, y in path]
        self.path_pub.publish(msg)

    def publish_status(self, status):
        m = String()
        m.data = json.dumps(status)
        self.status_pub.publish(m)
        self.get_logger().info(f"rulepatch: {m.data[:300]}")


def main():
    rclpy.init()
    node = Inference()
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
