"""pp_driver: minimal pure-pursuit path tracker for repaired-path demos.

Subs: /rulepatch/repaired_path (goal path), /odom.
Pubs: /cmd_vel as TwistStamped (matches the gz bridge ROS->GZ mapping).
Uses TF map->base_link for path tracking (paths are map-frame; raw odom
is NOT map). Deterministic; stops within goal_tol of path end.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped


class PPDriver(Node):
    def __init__(self):
        super().__init__("rulepatch_pp_driver")
        self.declare_parameter("speed", 0.35)
        self.declare_parameter("lookahead", 0.5)
        self.declare_parameter("goal_tol", 0.4)
        self.declare_parameter("topic_out", "/cmd_vel")
        self.path = None
        self._prog = 0
        self.create_subscription(Path, "/rulepatch/repaired_path",
                                 self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        import tf2_ros as _tf2
        self._tf = _tf2.Buffer()
        _tf2.TransformListener(self._tf, self)
        self.pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("topic_out").value), 10)
        self.timer = self.create_timer(0.1, self._tick)
        self._odom = None
        self.get_logger().info("pp_driver ready")

    def _on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y)
                     for p in msg.poses]
        self._prog = 0  # monotonic progress: never drive backward
        self.get_logger().info(f"tracking path n={len(self.path)}")

    def _on_odom(self, msg):
        self._odom = msg

    def _map_xy(self):
        try:
            from rclpy.time import Time as _T
            t = self._tf.lookup_transform("map", "base_link", _T())
            th = math.atan2(
                2 * (t.transform.rotation.w * t.transform.rotation.z),
                1 - 2 * t.transform.rotation.z * t.transform.rotation.z)
            return (float(t.transform.translation.x),
                    float(t.transform.translation.y), float(th))
        except Exception:
            return None

    def _tick(self):
        cmd = TwistStamped()
        pose = self._map_xy()
        if not self.path or pose is None:
            self.pub.publish(cmd)
            return
        import numpy as np
        x, y, th = pose
        p = np.array(self.path)
        gx, gy = p[-1]
        if math.hypot(gx - x, gy - y) < float(
                self.get_parameter("goal_tol").value):
            self.get_logger().info("pp_driver: goal reached")
            self.path = None
            self.pub.publish(cmd)
            return
        d = np.hypot(p[:, 0] - x, p[:, 1] - y)
        # advance progress to nearest at/after current progress (no regress)
        j = self._prog + int(np.argmin(d[self._prog:]))
        self._prog = j
        LA = float(self.get_parameter("lookahead").value)
        ahead = np.where(np.hypot(p[j:, 0] - x, p[j:, 1] - y) > LA)[0]
        k = j + int(ahead[0]) if len(ahead) else len(p) - 1
        tx, ty = p[k]
        ang = math.atan2(ty - y, tx - x) - th
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        cmd.twist.linear.x = float(self.get_parameter("speed").value)
        cmd.twist.angular.z = float(max(-1.0, min(1.0, 2.0 * ang)))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PPDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
