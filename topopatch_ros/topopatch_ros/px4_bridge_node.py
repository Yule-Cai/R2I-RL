# PX4 bridge participant for RuleSync volume 2PC (mirrors uav_bridge).
#
# Single volume participant on /rulesync/uav_bridge_vote_volume (kill any
# uav_bridge first — one volume voter by design). Votes by 2D-analog
# endpoint admission (pose-inside-volume -> REJECT); applies commit rounds;
# serves applied state for reconcile. Pose from PX4 VehicleOdometry
# (ROS2/px4_msgs, multicast). No occupancy needed for admission.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy

READY, REJECT, TIMEOUT = 0, 1, 2


def px4_qos():
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    q.history = HistoryPolicy.KEEP_LAST
    return q


class Px4Bridge(Node):
    def __init__(self):
        import os as _os
        super().__init__(f"rulesync_px4_bridge_{_os.getpid()}")
        self.declare_parameter("margin_m", 0.5)
        from rulesync_interfaces.srv import BridgeVoteVolume as _BVV
        from rulesync_interfaces.srv import BridgeVolumeState as _BVS
        self.create_service(_BVV, "/rulesync/uav_bridge_vote_volume",
                            self.on_vote)
        self.create_service(_BVS, "/rulesync/uav_bridge_state",
                            self.on_state)
        from px4_msgs.msg import VehicleOdometry as _VO
        self.create_subscription(_VO, "/fmu/out/vehicle_odometry",
                                 self._on_odom, px4_qos())
        self._xyz = None
        self.active_volumes = []
        self.active_epoch = 0
        self.staged_volumes = []
        self.staged_epoch = 0
        self.get_logger().info("px4_bridge ready")

    def _on_odom(self, msg):
        # PX4 local NED -> map-ish ENU-ish xyz (x=east, y=north, z=up):
        # odom.position is NED (x=north, y=east, z=down).
        try:
            self._xyz = (float(msg.position[1]), float(msg.position[0]),
                         float(-msg.position[2]))
        except Exception:
            pass

    def _inside(self, pt, vols):
        import numpy as _np
        from rulesync3d.rulesync3d.gate_3d import GEOM_AABB, GEOM_SPHERE
        r = float(self.get_parameter("margin_m").value)
        p = _np.asarray(pt, dtype=float)
        for v in vols:
            g = int(v.geometry_type)
            if g == GEOM_AABB:
                c = _np.array([v.center.x, v.center.y, v.center.z])
                d = _np.array([v.dimensions.x, v.dimensions.y,
                               v.dimensions.z])
                if bool(((p >= c - d / 2 - r) & (p <= c + d / 2 + r))
                        .all()):
                    return True
            elif g == GEOM_SPHERE:
                c = _np.array([v.center.x, v.center.y, v.center.z])
                if float(((p - c) ** 2).sum()) <= \
                        (float(v.dimensions.x) + r) ** 2:
                    return True
        return False

    def _handle(self, vols, epoch, op):
        if op == "prepare":
            if self._xyz is None:
                return REJECT, "no pose yet"
            if self._inside(self._xyz, list(vols)):
                return REJECT, "robot inside volume"
            self.staged_volumes = list(vols)
            self.staged_epoch = int(epoch)
            return READY, "ready"
        if op == "commit":
            if int(epoch) != int(self.staged_epoch):
                return REJECT, "epoch mismatch"
            self.active_volumes = list(self.staged_volumes)
            self.active_epoch = int(self.staged_epoch)
            self.staged_volumes, self.staged_epoch = [], 0
            return READY, "applied"
        if op == "abort":
            self.staged_volumes, self.staged_epoch = [], 0
            return READY, "discarded"
        return REJECT, f"unknown op {op}"

    def on_vote(self, req, resp):
        vote, reason = self._handle(list(req.volume_rules),
                                    int(req.epoch), str(req.op))
        resp.vote, resp.reason = vote, reason
        return resp

    def on_state(self, req, resp):
        resp.active_epoch = int(self.active_epoch)
        resp.active_volumes = list(self.active_volumes)
        return resp


def main():
    rclpy.init()
    node = Px4Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
