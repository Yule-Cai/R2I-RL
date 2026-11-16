#!/usr/bin/env python3
"""Live ROS transport smoke checks; not a navigation/crash-recovery cohort.

Run in a private local ROS domain after other managed physics runs exit.
Uses the production participant and persistent coordinator with an empty
snapshot and existing test-pose hook. No modifications to voting semantics.
"""
import argparse
import json
from pathlib import Path
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('do not overwrite transport evidence')
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '60' or os.environ.get('ROS_DISCOVERY_SERVER'):
        raise RuntimeError('requires private local domain 60 without discovery server')
    os.environ['RULESYNC_TEST_POSE'] = '1,10,2'
    import rclpy
    import rulesync3d
    sys.modules['rulesync3d.rulesync3d'] = rulesync3d
    from rclpy.executors import SingleThreadedExecutor
    from topopatch_ros.uav_bridge_node import UavBridge
    from rulesync_manager.volume_vote_coordinator import VolumeVoteCoordinator
    from rulesync_manager.vote_coordinator import VoteCoordinator
    from rulesync_manager.snapshot_identity import _vol_snapshot_hash
    from rulesync_manager.volume_vote_coordinator import build_volume_request

    rclpy.init(args=['--ros-args', '-p', 'world_boxes:="[]"'])
    coordinator, bridge, executor, thread = None, None, None, None
    checks, replies = {}, []
    def start_bridge():
        node = UavBridge()
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        worker = threading.Thread(target=ex.spin, daemon=True)
        worker.start()
        return node, ex, worker

    def stop_bridge():
        executor.shutdown()
        thread.join(timeout=5.)
        if thread.is_alive():
            raise RuntimeError('owned bridge spinner did not stop')
        bridge.destroy_node()

    def vote(op, epoch, txn, digest, expected):
        result = coordinator.round([], epoch, op, 2., txn, digest)['uav_bridge']
        replies.append(dict(op=op, epoch=epoch, txn_id=txn, vote=result[0],
                            reason=result[1], elapsed_s=result[2]))
        if result[0] != expected:
            raise AssertionError(f'{op}: {result}')

    try:
        data = dict(rule_id='roundtrip', epoch=3, rule_type=0, severity=1, frame_id='map',
            geometry_type=2, center=[1.,2.,3.], dims=[4.,5.,6.],
            vertices=[[.1,.2,.3]], valid_from=[7,8], valid_until=[9,10])
        request = build_volume_request([data], 3, 'prepare', 'txn', 'digest')
        msg = request.volume_rules[0]
        checks['typed_request_roundtrip'] = (
            request.txn_id == 'txn' and request.target_hash == 'digest' and
            msg.valid_until.nanosec == 10 and msg.vertices[0].z == .3 and
            (msg.dimensions.x, msg.dimensions.y, msg.dimensions.z) == (4.,5.,6.))
        bridge, executor, thread = start_bridge()
        coordinator = VolumeVoteCoordinator()
        checks['prewarm'] = all(coordinator.prewarm(timeout_each=10.).values())
        digest = _vol_snapshot_hash([])
        vote('prepare', 1, 'first', digest, 0)
        vote('commit', 1, 'wrong', digest, 1)
        checks['mismatch_not_applied'] = bridge.active_epoch == 0
        vote('commit', 1, 'first', digest, 0)
        checks['correct_certificate_applied'] = bridge.active_epoch == 1
        two_d = VoteCoordinator(['uav_bridge'])
        try:
            pre = two_d.prewarm(timeout_each=10.)
            reply = two_d.round([], 1, 'prepare', 2., '2d', '')['uav_bridge']
            checks['default_2d_transport_preserved'] = all(pre.values()) and reply[0] == 1
        finally:
            two_d.shutdown()
        stop_bridge()
        bridge, executor, thread = None, None, None
        reply = coordinator.round([], 2, 'prepare', .2, 'gone', digest)['uav_bridge']
        replies.append(dict(op='prepare-missing', vote=reply[0], reason=reply[1], elapsed_s=reply[2]))
        checks['missing_participant_timeout'] = reply[0] == 2
        bridge, executor, thread = start_bridge()
        checks['restart_prewarm'] = all(coordinator.prewarm(timeout_each=10.).values())
        vote('prepare', 2, 'restart', digest, 0)
        vote('commit', 2, 'restart', digest, 0)
        checks['restarted_participant_applied'] = bridge.active_epoch == 2
    finally:
        if coordinator is not None:
            coordinator.shutdown()
        if bridge is not None:
            stop_bridge()
        rclpy.try_shutdown()
        report = dict(checks=checks, replies=replies,
            passed=len(checks)==8 and all(checks.values()),
            scope='empty-snapshot live ROS transport smoke; not physical navigation or crash recovery')
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
    return int(not report['passed'])


if __name__ == '__main__':
    sys.exit(main())
