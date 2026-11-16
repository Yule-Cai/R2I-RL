# Pose TF bridge: joint_state (+spawn offset) -> TF map->uav_base.
# Needed by sensor pipelines (octomap_server) that require TF. Non-blocking
# spins (see A1 findings). Static base->rgbd_link published separately by
# the launcher (fixed offset from the SDF joint).
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PoseTf(Node):
    def __init__(self):
        import os as _os
        super().__init__(f"pose_tf_{_os.getpid()}")
        self.declare_parameter("spawn_xyz", [10.0, 10.0, 3.0])
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(JointState, "/r2i_quad/joint_state",
                                 self._on_js, qos)
        self._br = TransformBroadcaster(self)
        self._spawn = (10.0, 10.0, 3.0)
        self._xyz = None
        self.create_timer(0.05, self._tick)

    def _on_js(self, msg):
        try:
            pos = {n: p for n, p in zip(msg.name, msg.position)}
            self._xyz = (
                self._spawn[0] + float(pos.get("jx", 0.0)),
                self._spawn[1] + float(pos.get("jy", 0.0)),
                self._spawn[2] + float(pos.get("jz", 0.0)))
        except Exception:
            pass

    def _tick(self):
        try:
            self._spawn = tuple(float(v) for v in
                                self.get_parameter("spawn_xyz").value)
        except Exception:
            pass
        if self._xyz is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "uav_base"
        t.transform.translation.x, t.transform.translation.y, \
            t.transform.translation.z = (float(v) for v in self._xyz)
        t.transform.rotation.w = 1.0
        self._br.sendTransform(t)


def main():
    rclpy.init()
    node = PoseTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
