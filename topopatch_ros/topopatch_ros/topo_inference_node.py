"""TopoRulePatch ROS2 inference node (multi-rule, verified).

Mirrors rulepatch_ros.inference_node structure (topics/params), with the
M3 repair path: union BEV over up to 3 KEEP_OUTs, per-rule geometric
features, NumpyTopoNet fused forward, mode-conditional multi-hypothesis
selection under the frozen operating threshold, deterministic
multi-rule verification, repaired-path publish or full-replan fallback.

M2 files are NOT modified; shared helpers (crossing_anchors,
Nav2Planner) are imported read-only.
"""
import json
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from rulesync_interfaces.msg import StagedSet
from rulesync_interfaces.srv import ListRules

N_RULES_MAX = 3


def _union_span(path, polys):
    """Affected span over the union of rule polygons.

    Per-poly crossing_anchors (M2 logic, imported frozen); overall span
    = min iA .. max iB. Returns (iA, iB) or None when no poly affects
    the path.
    """
    from rulepatch_ros.inference_node import crossing_anchors
    spans = []
    for poly in polys:
        sp = crossing_anchors(path, poly, margin_m=3.0)
        if sp is not None:
            spans.append(sp)
    if not spans:
        return None
    return min(s[0] for s in spans), max(s[1] for s in spans)


class TopoInference(Node):
    def __init__(self):
        super().__init__("topo_inference")
        self.declare_parameter("weights_npz", "")
        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("status_topic", "/rulepatch/status")
        self.declare_parameter("repaired_topic", "/rulepatch/repaired_path")
        self.declare_parameter("anchor_margin_m", 3.0)
        self.declare_parameter("tau", 1.0264495134353637)  # frozen VAL op
        self.declare_parameter("use_scope", False)  # M4 adaptive model
        self._plan = None
        self._goal = None
        self._map = None
        self._last_published = None
        # QoS mirrors M2's proven minimal-stack wiring (volatile),
        # except /map: map_server publishes it once (latched) at startup,
        # so a volatile late-joiner misses it -> transient-local here.
        from rclpy.qos import QoSProfile, DurabilityPolicy
        q_map = QoSProfile(depth=10,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map,
                                 q_map)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(Path, "/rulepatch/repaired_path",
                                 self._on_echo, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal,
                                 10)
        self.create_subscription(StagedSet, "/rulesync/staged_set",
                                 self._on_staged, 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("repaired_topic").value), 10)
        # NOTE: no Nav2Planner client here on purpose (see fallback()).
        wp = str(self.get_parameter("weights_npz").value)
        self.load_weights(wp)
        # Reliable trigger (M2 pattern): the manager's StagedSet bursts
        # are transient and easily missed; poll list_rules for new
        # committed epochs instead.
        self.cli_list = self.create_client(ListRules,
                                           "/rulesync/list_rules")
        self._last_repaired_epoch = 0
        self._listing = False
        self._listing_since = 0.0
        self._polls = 0
        self._list_timeouts = 0
        # Poll interval 0.5 s (latency optimization): commit-to-status
        # latency dominates end-to-end. Decisions unchanged (same tau,
        # same verifier); only the trigger is more responsive.
        self.create_timer(0.5, self._poll_tick)
        self.get_logger().info("topo_inference ready")

    def _poll_tick(self):
        self._polls += 1
        if self._polls % 5 == 1:
            self.get_logger().info(
                f"poll#{self._polls} plan={self._plan is not None} "
                f"goal={self._goal is not None} map={self._map is not None} "
                f"last_ep={self._last_repaired_epoch} "
                f"listing={self._listing}")
        if self._listing:
            # Liveness guard (physics campaign): a ListRules response is
            # occasionally lost entirely (DDS flakiness under Gazebo
            # load), leaving _listing stuck True forever — topo then
            # never issues another poll and every later case goes
            # no-status. Reset an over-age in-flight call and retry.
            # Decisions are unchanged (same tau/verifier/policy); only
            # the trigger is resilient. A stale response arriving late
            # is still processed by _on_listed (epoch-guarded).
            import time as _t
            if _t.monotonic() - self._listing_since > 5.0:
                self._listing = False
                self._list_timeouts += 1
                self.get_logger().warning(
                    "list_rules call over-age (>5 s): resetting poll "
                    f"(timeouts={self._list_timeouts})")
            else:
                return
        if not self.cli_list.wait_for_service(timeout_sec=1.0):
            if self._polls % 15 == 1:
                self.get_logger().warning("list_rules undiscovered")
            return
        self._listing = True
        import time as _t2
        self._listing_since = _t2.monotonic()
        fut = self.cli_list.call_async(ListRules.Request())
        fut.add_done_callback(self._on_listed)

    def _on_listed(self, fut):
        self._listing = False
        try:
            res = fut.result()
            if res is None:
                return
            epoch = int(res.epoch)
            if epoch <= self._last_repaired_epoch:
                return
            keepouts = []
            for r in res.active:
                if int(r.rule_type) == 0:
                    keepouts.append([(p.x, p.y) for p in r.polygon.points])
            if not keepouts or not self._plan or not self._goal \
                    or self._map is None:
                return
            if len(keepouts) > N_RULES_MAX:
                self.publish_status({"reason": "too-many-rules",
                                     "n_rules": len(keepouts)})
                return
            self._last_repaired_epoch = epoch
            self.get_logger().info(
                f"new committed epoch {epoch}: repairing "
                f"{len(keepouts)} rule(s)")
            self.repair(self._plan, keepouts, self._goal,
                        note=f"epoch={epoch}")
        except Exception as e:
            self.get_logger().warning(f"list poll failed: {e}")

    def load_weights(self, path):
        from topopatch.topo.infer_numpy_topo import NumpyTopoNet
        self.net = NumpyTopoNet(path)
        self.get_logger().info(f"topo weights loaded: {path}")

    def _on_map(self, msg):
        self._map = msg

    def _on_plan(self, msg):
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if path == getattr(self, "_last_published", None):
            return
        self._plan = path

    def _on_echo(self, msg):
        pass

    def _on_goal(self, msg):
        self._goal = (float(msg.pose.position.x),
                      float(msg.pose.position.y))

    def _on_staged(self, msg):
        self.get_logger().info(
            f"staged rx op={msg.op} epoch={msg.epoch} "
            f"n_rules={len(msg.rules)}")
        if str(msg.op) != "commit":
            return
        keepouts = []
        for r in msg.rules:
            if int(r.rule_type) == 0:
                keepouts.append([(p.x, p.y) for p in r.polygon.points])
        if not keepouts:
            return
        if not self._plan or not self._goal or self._map is None:
            self.publish_status({"reason": "no-plan-yet"})
            return
        if len(keepouts) > N_RULES_MAX:
            self.publish_status({"reason": "too-many-rules",
                                 "n_rules": len(keepouts)})
            return
        self.repair(self._plan, keepouts, self._goal,
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

    def repair(self, path, polys, goal, note=""):
        from rulepatch.rp import BEV_SIZE_M
        from rulepatch.rp.bev import (add_coords, local_path_points,
                                      render_bev, _rasterize_polygon)
        from topopatch.topo.extract_topo import _rule_feats
        from topopatch.topo.verifier_topo import (select_topo,
                                                 verify_patch_multi)
        t0 = time.time()
        status = {"note": note, "fallback": False, "accepted": False,
                  "n_rules": len(polys)}
        try:
            span = _union_span(path, polys)
            if span is None:
                status.update(reason="unaffected-path",
                              latency_ms=(time.time() - t0) * 1000)
                self.publish_status(status)
                return status
            iA, iB = span
            A, B = tuple(path[iA]), tuple(path[iB])
            if max(abs(A[0] - B[0]), abs(A[1] - B[1])) > BEV_SIZE_M - 2.0:
                return self.fallback(path, polys, goal, status, t0,
                                     reason="span-overflow")
            center = ((A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0)
            seg = path[iA:iB + 1]
            occ = self.occ_fn()
            bev = render_bev(occ, center, seg, polys[0], goal)
            for extra in polys[1:]:
                _rasterize_polygon(bev[2], extra, center)
            np.clip(bev, 0, 1, out=bev)
            local = local_path_points(seg, center)
            rf = _rule_feats(polys, A, B, center)
            ms = (time.time() - t0) * 1000
            ctrls, confs, topo, scope = self.net.forward(
                add_coords(bev.astype(np.float64)),
                local.astype(np.float64),
                rf.astype(np.float64))
            fwd_ms = (time.time() - t0) * 1000 - ms
            bounds = self.map_bounds()
            tau = float(self.get_parameter("tau").value)
            use_scope = bool(self.get_parameter("use_scope").value)
            idx, dense, reason = select_topo(
                ctrls, confs, center, A, B, seg, polys, occ, bounds,
                tau=tau,
                scope=(scope if (use_scope and scope is not None)
                       else None))
            per_hyp = []
            for k in range(len(confs)):
                okk, rr = verify_patch_multi(
                    ctrls[k], center, A, B, seg, polys, occ, bounds)
                per_hyp.append({"hyp": k, "conf": float(confs[k]),
                                "ok": bool(okk), "reason": rr})
            status.update({
                "hypotheses": per_hyp,
                "confs": [float(v) for v in confs],
                "topo_pred": int(np.argmax(topo)),
                "topo_probs": [float(v) for v in topo],
                "rep_prob": (float(scope["rep_prob"])
                             if scope is not None else None),
                "span_pred": (float(scope["span_pred"])
                              if scope is not None else None),
                "cx_pred": (int(scope["cx_pred"])
                            if scope is not None else None),
                "use_scope": bool(use_scope),
                "center": [float(center[0]), float(center[1])],
                "A": [float(A[0]), float(A[1])],
                "B": [float(B[0]), float(B[1])],
                "tau": tau})
            if idx is None:
                return self.fallback(path, polys, goal, status, t0,
                                     reason=reason)
            full = path[:iA + 1] + [tuple(p) for p in dense.tolist()] \
                + path[iB:]
            self.publish_path(full)
            status.update(accepted=True, hyp=int(idx),
                          conf=float(confs[idx]), reason="ok", iA=iA,
                          iB=iB,
                          latency_ms=(time.time() - t0) * 1000,
                          infer_ms=fwd_ms)
        except Exception as e:
            import traceback
            status.update(reason=f"error: {e}",
                          traceback=traceback.format_exc()[-500:],
                          latency_ms=(time.time() - t0) * 1000)
        self.publish_status(status)
        return status

    def fallback(self, path, polys, goal, status, t0, reason):
        # Pure node: NO planner call here. Nested
        # spin_until_future_complete inside the list-callback executor
        # thread invalidates the spin context (crashed the node after the
        # first fallback, 2026-09-26). The case driver (separate process)
        # performs the rule-aware replan on fallback=True and publishes
        # it to /rulepatch/repaired_path itself.
        status.update(fallback=True, reason=f"fallback:{reason}",
                      fallback_ok=False,
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
        self.get_logger().info(f"topopatch: {m.data[:300]}")


def main():
    rclpy.init()
    node = TopoInference()
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
