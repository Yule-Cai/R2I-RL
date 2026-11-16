"""Wait for the first message on a topic (bringup sequencing).

Usage: python3 wait_topic.py /odom [--timeout 120] [--field ...]
Prints one line and exits 0 on receipt, 1 on timeout.
"""
import argparse
import time

import rclpy
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--type", default="nav_msgs.msg.Odometry")
    a = ap.parse_args()
    pkg, cls = a.type.rsplit(".", 1)
    import importlib
    mod = importlib.import_module(pkg)
    rclpy.init()
    n = Node("rp_wait_topic")
    box = {}
    try:
        n.create_subscription(getattr(mod, cls), a.topic,
                              lambda m: box.__setitem__("t", time.time()), 10)
        t0 = time.time()
        while not box and time.time() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.5)
        if box:
            print(f"{a.topic} flowing", flush=True)
        else:
            print(f"{a.topic} TIMEOUT", flush=True)
            raise SystemExit(1)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
