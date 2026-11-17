"""Passive command recorder kept alive through the benchmark's teardown."""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    a = p.parse_args()
    log = (a.root / 'command_witness.jsonl').open('x')
    rclpy.init()
    node = Node('rulesync_command_witness')
    received = 0

    def observe(msg):
        nonlocal received
        received += 1
        log.write(json.dumps(dict(kind='command', topic='/cmd_vel', receipt_ns=time.monotonic_ns(),
                                 pid=os.getpid(), v=msg.twist.linear.x, w=msg.twist.angular.z)) + '\n')
        log.flush()

    sub = node.create_subscription(TwistStamped, '/cmd_vel', observe, 100)
    def endpoints():
        infos = node.get_publishers_info_by_topic('/cmd_vel')
        if infos:
            assert all(info.topic_type == 'geometry_msgs/msg/TwistStamped' for info in infos), 'Command publisher type mismatch'
            (a.root / 'command_witness.endpoints.json').write_text(json.dumps([dict(node_name=info.node_name, topic_type=info.topic_type, reliability=str(info.qos_profile.reliability)) for info in infos], indent=2))
    timer = node.create_timer(.2, endpoints)
    (a.root / 'command_witness.ready').write_text(json.dumps(dict(pid=os.getpid(), ros_domain_id=os.environ.get('ROS_DOMAIN_ID'), topic_type='geometry_msgs/msg/TwistStamped', monotonic_ns=time.monotonic_ns())))
    deadline = time.monotonic() + 1100
    try:
        while not (a.root / 'command_witness.stop').exists():
            assert time.monotonic() < deadline, 'Witness stop control timed out'
            rclpy.spin_once(node, timeout_sec=.02)
        settle = time.monotonic() + .5
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    assert received > 0, 'Witness received no commands;collection is unqualified'


if __name__ == '__main__':
    main()
