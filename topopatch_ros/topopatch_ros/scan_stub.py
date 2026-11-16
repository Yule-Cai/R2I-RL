"""Static laser stub for headless stacks (paper sim).

Publishes a 360-ray all-clear LaserScan at 10 Hz so costmap obstacle
layers and bridge feasibility checks that wait on scan data unblock.
Pure test fixture (not part of the method).

Usage: python3 -m topopatch_ros.scan_stub [--frame base_scan]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="base_link",
                    help="must be TF-connected; headless tree only has "
                    "map->odom->base_link (no RSP), so base_link")
    ap.add_argument("--rate", type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    node = Node("scan_stub")
    pub = node.create_publisher(LaserScan, "/scan", 10)
    n = 360
    while rclpy.ok():
        m = LaserScan()
        m.header.frame_id = a.frame
        m.header.stamp = node.get_clock().now().to_msg()
        m.angle_min = -math.pi
        m.angle_max = math.pi
        m.angle_increment = 2 * math.pi / n
        m.range_min = 0.1
        m.range_max = 10.0
        m.ranges = [8.0] * n
        m.intensities = [0.0] * n
        pub.publish(m)
        time.sleep(1.0 / a.rate)


if __name__ == "__main__":
    main()
