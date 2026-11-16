"""Static rule publisher for scripted TopoRulePatch Gazebo cases.

Publishes one StagedSet commit (transient-local latched) with 1-3
KEEP_OUT rectangles on /rulesync/staged_set, then exits. Rectangles
given as x0,y0,x1,y1 repeated.

Usage:
  python3 -m topopatch_ros.rule_pub --epoch 1 \
      --rect 1.0,-1.0,2.0,1.0 [--rect ...]
"""
import argparse

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from rulesync_interfaces.msg import RuleSpec, StagedSet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epoch", type=int, default=1)
    ap.add_argument("--rect", action="append", default=[],
                    help="x0,y0,x1,y1 (up to 3)")
    a = ap.parse_args()
    assert 1 <= len(a.rect) <= 3, "need 1-3 --rect"
    rclpy.init()
    node = Node("topo_rule_pub")
    q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(StagedSet, "/rulesync/staged_set", q)
    msg = StagedSet()
    msg.op = "commit"
    msg.epoch = int(a.epoch)
    for i, r in enumerate(a.rect):
        x0, y0, x1, y1 = (float(v) for v in r.split(","))
        rule = RuleSpec()
        rule.rule_id = f"case-r{i}"
        rule.epoch = int(a.epoch)
        rule.rule_type = 0
        rule.severity = 1
        rule.frame_id = "map"
        for x, y in ((x0, y0), (x0, y1), (x1, y1), (x1, y0)):
            p = Point32()
            p.x, p.y = x, y
            rule.polygon.points.append(p)
        msg.rules.append(rule)
    for _ in range(5):
        pub.publish(msg)
        node.get_logger().info(
            f"staged commit epoch={msg.epoch} n_rules={len(msg.rules)}")
        rclpy.spin_once(node, timeout_sec=0.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
