"""Mask-delay proxy for fault F1 (delayed mask publication).

Subscribes the bridge mask (/rulesync/keepout_mask) and republishes
each version to --out-topic after --delay seconds. The planner under
test is remapped to the delayed topic; filter-info flows direct, so the
planner authentically waits on a late mask.

Usage:
  python3 -m topopatch_ros.mask_delay --delay 5.0 \
      --out-topic /mask_delayed
"""
import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delay", type=float, default=5.0)
    ap.add_argument("--in-topic", default="/rulesync/keepout_mask")
    ap.add_argument("--out-topic", default="/mask_delayed")
    a = ap.parse_args()
    rclpy.init()
    node = Node("mask_delay_proxy")
    q = QoSProfile(depth=10,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    buf = []

    def cb(msg):
        buf.append((time.time(), msg))
        node.get_logger().info(
            f"mask rx nonzero={sum(1 for v in msg.data if v > 0)}")

    node.create_subscription(OccupancyGrid, a.in_topic, cb, q)
    # Match the costmap filter's transient-local subscription, else the
    # delayed mask is discovery-incompatible and never delivers (observed:
    # DURABILITY_QOS_POLICY warnings, planner plans maskless forever).
    pub = node.create_publisher(OccupancyGrid, a.out_topic, q)

    def tick():
        now = time.time()
        due = [m for t, m in buf if now - t >= a.delay]
        for m in due:
            pub.publish(m)
            node.get_logger().info("mask relayed after delay")
        buf[:] = [(t, m) for t, m in buf if now - t < a.delay]

    node.create_timer(0.5, tick)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
