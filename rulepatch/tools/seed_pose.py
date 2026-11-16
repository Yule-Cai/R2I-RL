"""Seed AMCL initial pose and wait for localization (demo bringup step).

Must run BEFORE lifecycle activation: costmap configure waits on map->odom.
Usage: python3 seed_pose.py [--x -2.0 --y -0.5]
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=-2.0)
    ap.add_argument("--y", type=float, default=-0.5)
    a = ap.parse_args()
    rclpy.init()
    n = Node("rp_seed_pose", parameter_overrides=[])
    # Use sim time so the stamp matches AMCL's TF buffer (stamp 0 causes
    # "extrapolation into the future" and no map->odom TF).
    from rclpy.parameter import Parameter
    n2 = Node("rp_seed_pose_t", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    pub = n2.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    box = {}
    n2.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                           lambda m: box.__setitem__("ok", True), 10)
    try:
        def _pub():
            m = PoseWithCovarianceStamped()
            m.header.stamp = n2.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.pose.pose.position.x, m.pose.pose.position.y = a.x, a.y
            m.pose.pose.orientation.w = 1.0
            m.pose.covariance[0] = m.pose.covariance[7] = 0.25
            m.pose.covariance[35] = 0.0685
            pub.publish(m)
        for _ in range(4):
            _pub()
            time.sleep(0.5)
        t0 = time.time()
        last = 0.0
        while "ok" not in box and time.time() - t0 < 90.0:
            rclpy.spin_once(n2, timeout_sec=0.5)
            if time.time() - last > 5.0:
                _pub()
                last = time.time()
        print(f"[seed] localized={bool(box.get('ok'))}", flush=True)
        assert box.get("ok"), "AMCL never localized"
    finally:
        n2.destroy_node()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
