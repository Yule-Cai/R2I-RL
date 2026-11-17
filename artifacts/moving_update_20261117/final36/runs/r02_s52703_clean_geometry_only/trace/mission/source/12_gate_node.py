"""Gate node: architectural enforcement of the universal output invariant.

This node verifies raw candidate paths against the current committed hard-rule
snapshot plus occupancy and bounds.  It binds accepted candidates in one
atomic ExecutionAuthorization containing the complete Path, epoch, snapshot
hash, and path digest.

New deployment: RuleGuard consumes the authorization and is the sole publisher
to the controller-facing executable path.  ``gated_path`` remains an optional
legacy output for the frozen evaluation harness; it does not provide the
independent execution-monitor architecture by itself.

Usage:
  python3 -m topopatch_ros.gate_node   (same env/PYTHONPATH as topo nodes)
"""
import json
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from rulesync_interfaces.msg import ExecutionAuthorization
from rulesync_interfaces.srv import ListRules

try:  # Installed ROS package.
    from rulesync3d.authorization import path_digest, token_id
    from rulesync3d.gate2d import gate_full_path, SnapshotPolygonIndexCache
    from rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash
except ImportError:  # Repository-root offline/source execution.
    from rulesync3d.rulesync3d.authorization import path_digest, token_id
    from rulesync3d.rulesync3d.gate2d import (
        gate_full_path, SnapshotPolygonIndexCache,
    )
    from rulesync3d.rulesync3d.ruleguard import (
        rule_snapshot_items, snapshot_hash,
    )


class GateNode(Node):
    def __init__(self):
        super().__init__("rulepatch_gate")
        self.declare_parameter("in_topic", "/rulepatch/repaired_path")
        self.declare_parameter("out_topic", "/rulepatch/gated_path")
        self.declare_parameter("status_topic", "/rulepatch/gate_status")
        self.declare_parameter(
            "authorization_topic", "/rulesync/execution_authorization")
        # Must match RuleGuard's represented-body/safety inflation.  This is
        # applied continuously to rule geometry, not only at sampled poses.
        self.declare_parameter("rho", 0.20)
        # Opt in after validating the target ROS deployment; historical runs
        # retain the direct backend and their frozen timing measurements.
        self.declare_parameter("use_polygon_index", False)
        self._polygon_index = SnapshotPolygonIndexCache()
        # Compatibility output for the frozen evaluation harness.  A new
        # deployment must put the controller on RuleGuard's executable_topic,
        # not this legacy path topic.
        self.declare_parameter("publish_legacy_path", False)
        self._map = None
        self.blocked = 0
        self.forwarded = 0
        q_map = QoSProfile(depth=10,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map,
                                 q_map)
        self.create_subscription(Path,
                                 str(self.get_parameter("in_topic").value),
                                 self._on_candidate, 10)
        self.pub = self.create_publisher(
            Path, str(self.get_parameter("out_topic").value), 10)
        self.stat = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.auth = self.create_publisher(
            ExecutionAuthorization,
            str(self.get_parameter("authorization_topic").value), 10)
        self.cli_list = self.create_client(ListRules,
                                           "/rulesync/list_rules")
        self._listing = False
        self._pending = None
        self._pending_t0 = 0.0

    def _on_map(self, msg):
        self._map = msg

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

    def map_bounds(self):
        m = self._map
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        return (ox, oy, ox + m.info.width * m.info.resolution,
                oy + m.info.height * m.info.resolution)

    def _on_candidate(self, msg):
        # Fully async (M2 pattern): NEVER block-spin inside the
        # subscription callback (nested spin_until_future_complete on an
        # executor-driven node hangs the ListRules call every time).
        if self._listing:
            self._verdict(False, "gate:busy", time.time(), len(msg.poses))
            return
        if not self.cli_list.service_is_ready():
            self._verdict(False, "gate:rules-unknown", time.time(), 0)
            return
        self._listing = True
        self._pending = msg
        self._pending_t0 = time.time()
        fut = self.cli_list.call_async(ListRules.Request())
        fut.add_done_callback(self._on_listed)

    def _on_listed(self, fut):
        self._listing = False
        t0 = self._pending_t0 or time.time()
        try:
            res = fut.result()
            if res is None:
                self._verdict(False, "gate:rules-unknown", t0, 0)
                return
            if bool(getattr(res, "degraded", True)):
                self._verdict(False, "gate:manager-degraded", t0, 0)
                return
            computed_hash = snapshot_hash(rule_snapshot_items(res.active))
            if computed_hash != str(res.active_hash):
                self._verdict(False, "gate:snapshot-hash-mismatch", t0, 0)
                return
            keepouts = [[(p.x, p.y) for p in r.polygon.points]
                        for r in res.active
                        if int(r.rule_type) == 0
                        and int(getattr(r, "severity", 1)) == 1]
            msg = self._pending
            path_frame = str(msg.header.frame_id)
            if not path_frame:
                self._verdict(False, "gate:path-frame-empty", t0, 0)
                return
            for rule in res.active:
                if int(rule.rule_type) == 0 and \
                        int(getattr(rule, "severity", 1)) == 1 and \
                        str(rule.frame_id) != path_frame:
                    self._verdict(False, "gate:rule-frame-mismatch", t0, 0)
                    return
            for stamped in msg.poses:
                pose_frame = str(stamped.header.frame_id)
                if pose_frame and pose_frame != path_frame:
                    self._verdict(False, "gate:path-frame-mismatch", t0, 0)
                    return
            path = [(p.pose.position.x, p.pose.position.y)
                    for p in msg.poses]
            rho = float(self.get_parameter("rho").value)
            geometry = keepouts
            if bool(self.get_parameter("use_polygon_index").value):
                # The response's full hash and every frame have already been
                # checked above. A cache hit replaces geometry preparation,
                # never the committed-state query or authorization check.
                geometry, _ = self._polygon_index.get(
                    res.epoch, computed_hash, path_frame, keepouts, rho)
            if self._map is None:
                go, reason = gate_full_path(
                    path, geometry, lambda x, y: False,
                    (-1e9, -1e9, 1e9, 1e9),
                    inflation_radius=rho)
            else:
                go, reason = gate_full_path(
                    path, geometry, self.occ_fn(), self.map_bounds(),
                    inflation_radius=rho)
            tok = ""
            if go:
                digest = path_digest(msg)
                tok = token_id(res.epoch, computed_hash, digest)
                auth = ExecutionAuthorization()
                auth.header = msg.header
                auth.token_id = tok
                auth.epoch = int(res.epoch)
                auth.snapshot_hash = computed_hash
                auth.path_digest = digest
                auth.gate_reason = str(reason)
                auth.path = msg
                # Publish the atomic path+identity authorization before the
                # legacy path.  The standalone RuleGuard consumes only this
                # message and is the sole publisher to executable_topic.
                self.auth.publish(auth)
                if bool(self.get_parameter("publish_legacy_path").value):
                    self.pub.publish(msg)
            self._verdict(go, reason, t0, len(path),
                          epoch=int(res.epoch),
                          snap_hash=computed_hash, token=tok)
        except Exception as e:
            self.get_logger().warning(f"gate list failed: {e}")
            self._verdict(False, "gate:rules-unknown", t0, 0)

    def _verdict(self, go, reason, t0, n, epoch=None, snap_hash="",
                 token=""):
        if go:
            self.forwarded += 1
        else:
            self.blocked += 1
        st = {"execute": bool(go), "reason": reason,
              "latency_ms": round((time.time() - t0) * 1000, 2),
              "n_pts": n, "forwarded": self.forwarded,
              "blocked": self.blocked,
              "epoch": epoch, "snapshot_hash": snap_hash,
              "token_id": token}
        m = String()
        m.data = json.dumps(st)
        self.stat.publish(m)
        self.get_logger().info(f"gate: {m.data[:200]}")


def main():
    rclpy.init()
    node = GateNode()
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
