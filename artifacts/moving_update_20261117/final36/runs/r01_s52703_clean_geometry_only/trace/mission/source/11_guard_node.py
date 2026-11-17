"""Independent ROS 2 execution monitor for snapshot-bound 2-D motion.

The geometric gate publishes one atomic ExecutionAuthorization containing the
candidate Path and the manager identity that authorized it.  This process
independently polls the manager, recomputes the canonical hash, binds the
execution once, and becomes the sole publisher to ``executable_topic``.

No authorization, stale localization, manager DEGRADED, a hash mismatch, an
epoch change, or a boundary violation all fail closed to HOLD.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from rulesync_interfaces.msg import ExecutionAuthorization, RuleGuardState
from rulesync_interfaces.srv import ListRules

try:  # Installed ament package.
    from rulesync3d.authorization import (
        connector_is_safe, verify_authorization,
    )
    from rulesync3d.ruleguard import (
        CLEAR, HOLD, WARNING, RuleGuard, rule_snapshot_items, snapshot_hash,
    )
except ImportError:  # Repository-root source execution.
    from rulesync3d.rulesync3d.authorization import (
        connector_is_safe, verify_authorization,
    )
    from rulesync3d.rulesync3d.ruleguard import (
        CLEAR, HOLD, WARNING, RuleGuard, rule_snapshot_items, snapshot_hash,
    )


_STATE_CODE = {
    CLEAR: RuleGuardState.CLEAR,
    WARNING: RuleGuardState.WARNING,
    HOLD: RuleGuardState.HOLD,
}


class RuleGuardNode(Node):
    def __init__(self):
        super().__init__("rulesync_guard")
        self.declare_parameter(
            "authorization_topic", "/rulesync/execution_authorization")
        self.declare_parameter(
            "executable_topic", "/rulepatch/executable_path")
        # This topic must contain actual localized pose in the rule frame.
        # An odom-frame pose must be transformed upstream before publication.
        self.declare_parameter("pose_topic", "/ruleguard/pose")
        self.declare_parameter("state_topic", "/rulesync/ruleguard_state")
        self.declare_parameter("manager_service", "/rulesync/list_rules")
        self.declare_parameter("rule_frame", "map")
        self.declare_parameter("rho", 0.20)
        self.declare_parameter("rho_warn", 0.50)
        self.declare_parameter("rho_release", 0.20)
        self.declare_parameter("snapshot_poll_s", 0.20)
        self.declare_parameter("snapshot_stale_s", 1.00)
        self.declare_parameter("snapshot_request_timeout_s", 1.00)
        self.declare_parameter("pose_stale_s", 0.50)
        self.declare_parameter("path_start_tolerance", 0.25)
        self.declare_parameter("hold_publish_s", 0.20)

        self._frame = str(self.get_parameter("rule_frame").value)
        self._rho = float(self.get_parameter("rho").value)
        self._guard = RuleGuard(
            rho_warn=float(self.get_parameter("rho_warn").value),
            rho_release=float(self.get_parameter("rho_release").value),
        )
        self._polygons = []
        self._snapshot_at = 0.0
        self._pose_at = 0.0
        self._pose_xy = None
        self._pose_frame = ""
        self._snapshot_pending = False
        self._snapshot_request_at = 0.0
        self._snapshot_request_seq = 0
        self._snapshot_pending_seq = None
        self._token_id = ""
        self._latched_hold = False
        self._state = HOLD
        self._reason = "startup-no-authority"
        self._degraded = True
        self._clearance = math.nan

        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter("executable_topic").value), 10)
        self._state_pub = self.create_publisher(
            RuleGuardState, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(
            ExecutionAuthorization,
            str(self.get_parameter("authorization_topic").value),
            self._on_authorization, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("pose_topic").value),
            self._on_pose, 20)
        self._list_cli = self.create_client(
            ListRules, str(self.get_parameter("manager_service").value))

        poll_s = float(self.get_parameter("snapshot_poll_s").value)
        hold_s = float(self.get_parameter("hold_publish_s").value)
        self.create_timer(max(0.05, poll_s), self._poll_snapshot)
        self.create_timer(max(0.05, hold_s), self._safety_tick)
        self.get_logger().info(
            "RuleGuard fail-closed: controller must subscribe to "
            f"{self.get_parameter('executable_topic').value}")

    def _poll_snapshot(self):
        now = time.monotonic()
        if self._snapshot_pending:
            timeout = float(
                self.get_parameter("snapshot_request_timeout_s").value)
            if now - self._snapshot_request_at <= max(0.10, timeout):
                return
            # A lost async response must not wedge the monitor forever.  Keep
            # the motion path fail-closed, retire this request generation, and
            # issue a fresh request on the next timer tick.  A late callback
            # is ignored by its sequence number below.
            self._snapshot_pending = False
            self._snapshot_pending_seq = None
            self._invalidate("manager-response-timeout")
        if not self._list_cli.service_is_ready():
            return
        self._snapshot_request_seq += 1
        request_seq = self._snapshot_request_seq
        self._snapshot_pending = True
        self._snapshot_pending_seq = request_seq
        self._snapshot_request_at = now
        future = self._list_cli.call_async(ListRules.Request())
        future.add_done_callback(
            lambda done, seq=request_seq: self._on_snapshot(done, seq))

    def _on_snapshot(self, future, request_seq):
        if request_seq != self._snapshot_pending_seq:
            return
        self._snapshot_pending = False
        self._snapshot_pending_seq = None
        try:
            response = future.result()
            if response is None:
                return
            if bool(getattr(response, "degraded", True)):
                self._invalidate("manager-degraded")
                return
            computed = snapshot_hash(rule_snapshot_items(response.active))
            if computed != str(response.active_hash):
                self._invalidate("snapshot-hash-mismatch")
                return

            polygons = []
            for rule in response.active:
                if (int(rule.rule_type) != 0
                        or int(getattr(rule, "severity", 1)) != 1):
                    continue
                if str(rule.frame_id) != self._frame:
                    self._invalidate("rule-frame-mismatch")
                    return
                polygon = [(float(p.x), float(p.y))
                           for p in rule.polygon.points]
                if len(polygon) < 3:
                    self._invalidate("invalid-rule-geometry")
                    return
                polygons.append(polygon)

            old_identity = (self._guard.epoch, self._guard.snap_hash)
            self._polygons = polygons
            self._guard.update_snapshot(response.epoch, computed)
            self._snapshot_at = time.monotonic()
            self._degraded = False
            new_identity = (self._guard.epoch, self._guard.snap_hash)
            execution = self._guard.execution_identity()
            if execution is not None and execution != new_identity:
                self._latched_hold = True
                self._set_state(HOLD, "epoch-disagreement", True,
                                math.nan)
                self._publish_stop()
            elif old_identity != new_identity and execution is None:
                self._set_state(HOLD, "execution-not-bound", False,
                                math.nan)
        except Exception as exc:
            self.get_logger().warning(f"snapshot poll failed: {exc}")

    def _invalidate(self, reason):
        changed = (self._guard.epoch is not None or self._reason != reason)
        self._guard.invalidate_snapshot()
        self._latched_hold = True
        self._polygons = []
        self._snapshot_at = 0.0
        self._set_state(HOLD, reason, True, math.nan)
        self._publish_stop()
        if changed:
            self.get_logger().warning(f"RuleGuard HOLD: {reason}")

    def _on_authorization(self, message):
        ok, reason, _ = verify_authorization(
            message.path, message.path_digest, message.token_id,
            message.epoch, message.snapshot_hash)
        if not ok:
            self._reject_authorization(reason)
            return
        if str(message.path.header.frame_id) != self._frame:
            self._reject_authorization("authorization-frame-mismatch")
            return
        if not message.path.poses:
            self._reject_authorization("authorization-empty-path")
            return
        for stamped in message.path.poses:
            frame = str(stamped.header.frame_id)
            if frame and frame != self._frame:
                self._reject_authorization("authorization-frame-mismatch")
                return
        if not self._snapshot_fresh():
            self._reject_authorization("monitor-snapshot-stale")
            return
        authority = (self._guard.epoch, self._guard.snap_hash)
        proposed = (int(message.epoch), str(message.snapshot_hash))
        if proposed != authority:
            self._reject_authorization("authorization-identity-mismatch")
            return
        if not self._pose_fresh():
            self._reject_authorization("actual-pose-stale")
            return
        if self._pose_frame != self._frame:
            self._reject_authorization("actual-pose-frame-mismatch")
            return

        self._latched_hold = False
        self._guard.bind_execution(message.epoch, message.snapshot_hash)
        state, info = self._guard.check_bound_2d(
            self._pose_xy, self._polygons, self._rho)
        self._token_id = str(message.token_id)
        self._set_state(
            state, str(info.get("reason", "authorized")),
            bool(info.get("degraded", False)),
            float(info.get("clearance", math.nan)))
        if state == HOLD:
            self._latched_hold = True
            self._publish_stop()
            return
        first = message.path.poses[0].pose.position
        start_distance = math.hypot(
            float(first.x) - self._pose_xy[0],
            float(first.y) - self._pose_xy[1])
        if not connector_is_safe(
                start_distance, info.get("clearance", math.nan),
                self.get_parameter("path_start_tolerance").value):
            self._reject_authorization("authorization-start-disconnected")
            return
        # The controller-facing path is published only after independent
        # identity, content, frame, pose-freshness, and geometry checks.
        self._path_pub.publish(message.path)

    def _reject_authorization(self, reason):
        self._guard.clear_execution()
        self._latched_hold = True
        self._token_id = ""
        self._set_state(HOLD, reason, True, math.nan)
        self._publish_stop()
        self.get_logger().warning(f"authorization rejected: {reason}")

    def _on_pose(self, message):
        self._pose_xy = (float(message.pose.position.x),
                         float(message.pose.position.y))
        self._pose_frame = str(message.header.frame_id)
        self._pose_at = time.monotonic()
        if self._guard.execution_identity() is None:
            self._set_state(HOLD, "execution-not-bound", False, math.nan)
            return
        # HOLD truncates the controller-facing path.  Do not silently resume a
        # now-stale path after geometry or connectivity recovers; a fresh gate
        # authorization is required to clear this latch.
        if self._latched_hold:
            return
        if not self._snapshot_fresh():
            self._invalidate("monitor-snapshot-stale")
            return
        if self._pose_frame != self._frame:
            self._latched_hold = True
            self._set_state(HOLD, "actual-pose-frame-mismatch", True,
                            math.nan)
            self._publish_stop()
            return
        state, info = self._guard.check_bound_2d(
            self._pose_xy, self._polygons, self._rho)
        self._set_state(
            state, str(info.get("reason", "ok")),
            bool(info.get("degraded", False)),
            float(info.get("clearance", math.nan)))
        if state == HOLD:
            self._latched_hold = True
            self._publish_stop()

    def _snapshot_fresh(self):
        age = time.monotonic() - self._snapshot_at
        return (self._guard.epoch is not None
                and age <= float(
                    self.get_parameter("snapshot_stale_s").value))

    def _pose_fresh(self):
        return (self._pose_xy is not None
                and time.monotonic() - self._pose_at <= float(
                    self.get_parameter("pose_stale_s").value))

    def _safety_tick(self):
        if self._guard.epoch is not None and not self._snapshot_fresh():
            self._invalidate("monitor-snapshot-stale")
        elif self._guard.execution_identity() is not None \
                and not self._pose_fresh():
            self._latched_hold = True
            self._set_state(HOLD, "actual-pose-stale", True, math.nan)
        if self._state == HOLD:
            self._publish_stop()
        self._publish_state()

    def _set_state(self, state, reason, degraded, clearance):
        changed = (state != self._state or reason != self._reason
                   or bool(degraded) != self._degraded)
        self._state = state
        self._reason = str(reason)
        self._degraded = bool(degraded)
        self._clearance = float(clearance)
        if changed:
            self.get_logger().info(
                f"RuleGuard {state}: {self._reason} "
                f"clearance={self._clearance}")
        self._publish_state()

    def _publish_stop(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame
        self._path_pub.publish(message)

    def _publish_state(self):
        message = RuleGuardState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame
        message.state = _STATE_CODE[self._state]
        message.degraded = self._degraded
        message.reason = self._reason
        message.clearance = self._clearance
        message.authority_epoch = int(self._guard.epoch or 0)
        message.authority_hash = str(self._guard.snap_hash or "")
        execution = self._guard.execution_identity()
        if execution is not None:
            message.execution_epoch = int(execution[0])
            message.execution_hash = str(execution[1])
        message.token_id = self._token_id
        self._state_pub.publish(message)


def main():
    rclpy.init()
    node = RuleGuardNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
