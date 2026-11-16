#!/usr/bin/env python3
"""Real ROS transaction/admission/monitor smoke test, NOT closed-loop evidence.

Run in a built, sourced workspace in an isolated ROS domain. Production nodes
run as separate processes. The map is synthetic and free; the pose is fixed.
No robot, dynamics, perception error, stopping distance or timing bound is
validated here. Transport receipts are retained, but are not performance data.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from rulesync_interfaces.msg import ExecutionAuthorization, RuleGuardState, RuleSpec
from rulesync_interfaces.srv import CommitRule, ListRules, ProposeRule, RemoveRule
from rulesync3d.authorization import path_digest, token_id
from rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash


MODES = ('direct', 'indexed')
SOURCE_PATHS = (
    'scripts/verify_polygon_index_ros.py',
    'topopatch_ros/topopatch_ros/gate_node.py',
    'rulesync3d/rulesync3d/gate2d.py',
    'rulesync3d/rulesync3d/authorization.py',
    'rulesync3d/rulesync3d/ruleguard.py',
    'rulesync/src/rulesync_manager/rulesync_manager/manager_node.py',
    'rulesync/src/rulesync_manager/rulesync_manager/snapshot_identity.py',
    'rulesync/src/rulesync_manager/rulesync_manager/vote_coordinator.py',
    'rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py',
    'rulesync/src/rulesync_guard/rulesync_guard/guard_node.py',
)


class Processes:
    def __init__(self, output):
        self.output, self.children = output, []

    def start(self, name, command):
        stream = (self.output / (name + '.log')).open('w')
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.children.append((process, stream))

    def close(self):
        for process, _ in self.children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process, stream in self.children:
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            stream.close()


class Probe(Node):
    def __init__(self):
        super().__init__('polygon_index_ros_probe')
        self.receipts, self.status, self.auth, self.state = [], {}, {}, {}
        self.paths = {mode: [] for mode in MODES}
        self.pose_xy = (-2., 0.)
        self.pub = {mode: self.create_publisher(PathMsg, f'/index_test/{mode}/candidate', 10)
                    for mode in MODES}
        self.pose = self.create_publisher(PoseStamped, '/ruleguard/pose', 20)
        self.map = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for mode in MODES:
            self.create_subscription(String, f'/index_test/{mode}/status',
                lambda msg, m=mode: self.receive(m, 'status', json.loads(msg.data)), 10)
            self.create_subscription(ExecutionAuthorization, f'/index_test/{mode}/auth',
                lambda msg, m=mode: self.receive(m, 'auth', msg), 10)
            self.create_subscription(RuleGuardState, f'/index_test/{mode}/state',
                lambda msg, m=mode: self.receive(m, 'state', msg), 20)
            self.create_subscription(PathMsg, f'/index_test/{mode}/executable',
                lambda msg, m=mode: self.receive(m, 'path', msg), 20)
        self._test_clients = {
            'list': self.create_client(ListRules, '/rulesync/list_rules'),
            'propose': self.create_client(ProposeRule, '/rulesync/propose_rule'),
            'commit': self.create_client(CommitRule, '/rulesync/commit_rule'),
            'remove': self.create_client(RemoveRule, '/rulesync/remove_rule'),
        }
        self.create_timer(.05, self.publish_pose)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .1
        grid.info.origin.position.x = grid.info.origin.position.y = -4.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 6400
        self.map.publish(grid)

    def receive(self, mode, kind, msg):
        row = dict(mode=mode, kind=kind, received_ns=time.monotonic_ns())
        if kind == 'status':
            self.status[mode] = msg
            row.update(msg)
        elif kind == 'auth':
            self.auth[mode] = msg
            row.update(epoch=msg.epoch, snapshot_hash=msg.snapshot_hash,
                       token_id=msg.token_id, path_digest=msg.path_digest)
        elif kind == 'state':
            self.state[mode] = msg
            row.update(state=msg.state, reason=msg.reason,
                       authority_epoch=msg.authority_epoch,
                       authority_hash=msg.authority_hash,
                       execution_epoch=msg.execution_epoch)
        else:
            self.paths[mode].append(msg)
            row.update(points=len(msg.poses), digest=path_digest(msg))
        self.receipts.append(row)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.pose_xy
        msg.pose.orientation.w = 1.
        self.pose.publish(msg)

    def wait(self, predicate, timeout=15.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.03)
            if predicate():
                return
        raise TimeoutError('ROS condition timeout')

    def call(self, name, request):
        client = self._test_clients[name]
        self.wait(client.service_is_ready, 45.)
        future = client.call_async(request)
        self.wait(future.done, 45.)
        response = future.result()
        if response is None:
            raise RuntimeError('empty service response: ' + name)
        return response

    def snapshot(self, epoch):
        deadline = time.monotonic() + 30.
        while time.monotonic() < deadline:
            response = self.call('list', ListRules.Request())
            if response.epoch == epoch and not response.degraded:
                assert response.active_hash == snapshot_hash(rule_snapshot_items(response.active))
                self.wait(lambda: all(m in self.state and
                    self.state[m].authority_epoch == epoch and
                    self.state[m].authority_hash == response.active_hash for m in MODES))
                return response
        raise TimeoutError('manager did not converge')

    def candidate(self, label, points, snapshot, accepted, frame='map', repeat=0):
        path = PathMsg()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        order = MODES if repeat % 2 == 0 else MODES[::-1]
        results = {}
        for mode in order:
            self.status.pop(mode, None)
            self.auth.pop(mode, None)
            before = len(self.paths[mode])
            self.pub[mode].publish(path)
            self.wait(lambda: mode in self.status)
            status = self.status[mode]
            assert status['execute'] == accepted, (label, mode, status)
            if frame == 'map':
                assert (status['epoch'], status['snapshot_hash']) == (
                    snapshot.epoch, snapshot.active_hash)
            if accepted:
                self.wait(lambda: mode in self.auth and any(
                    path_digest(p) == path_digest(path) for p in self.paths[mode][before:]))
                auth = self.auth[mode]
                assert (auth.epoch, auth.snapshot_hash, auth.path_digest, auth.token_id) == (
                    snapshot.epoch, snapshot.active_hash, path_digest(path),
                    token_id(snapshot.epoch, snapshot.active_hash, path_digest(path)))
            else:
                until = time.monotonic() + .15
                self.wait(lambda: time.monotonic() >= until)
                assert mode not in self.auth, (label, mode, 'unexpected authorization')
                assert not any(p.poses for p in self.paths[mode][before:])
            results[mode] = {k: status[k] for k in
                ('execute', 'reason', 'epoch', 'snapshot_hash', 'token_id')}
        assert results['direct'] == results['indexed'], (label, results)
        return dict(label=label, repetition=repeat, expected_accept=accepted,
                    digest=path_digest(path), results=results)


def rule(epoch, relaxed=False):
    msg = RuleSpec()
    msg.rule_id, msg.epoch = 'index-test-zone', epoch
    msg.rule_type, msg.severity, msg.frame_id = 0, 1, 'map'
    ymax = .4 if relaxed else 1.
    msg.polygon.points = [Point32(x=x, y=y, z=0.) for x, y in
        [(-.5, -.4), (.5, -.4), (.5, ymax), (-.5, ymax)]]
    msg.valid_until.sec = 3600
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, default=3)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error('repetitions must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    args.output = args.output.resolve()
    root = Path(__file__).resolve().parents[1]
    manifest = dict(scope='separate-process ROS integration; synthetic map and fixed pose; no dynamics',
        platform=platform.platform(), python=platform.python_version(),
        ros_distro=os.environ.get('ROS_DISTRO'), ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
        repetitions=args.repetitions, rho=.2, snapshot_poll_s=.2,
        conditions=['empty', 'ADD', 'same-identity-repeat', 'path-frame-mismatch',
                    'RELAX', 'REMOVE', 'stale-authorization-replay', 'pose-boundary-HOLD'],
        source_sha256={p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in SOURCE_PATHS})
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    processes, probe, rows = Processes(args.output), None, []
    try:
        os.environ['RULESYNC_TEST_POSE2D'] = '-2,0'
        os.environ['RULESYNC_TEST_GOAL2D'] = '2,0'
        processes.start('bridge', ['ros2', 'run', 'rulesync_nav2_bridge', 'bridge',
            '--ros-args', '-p', 'auto_replan:=false', '-p', 'auto_rollback:=false',
            '-p', 'grid_width:=80', '-p', 'grid_height:=80', '-p', 'grid_resolution:=0.1',
            '-p', 'grid_origin_x:=-4.0', '-p', 'grid_origin_y:=-4.0'])
        processes.start('manager', ['ros2', 'run', 'rulesync_manager', 'manager', '--ros-args',
            '-p', f'state_path:={args.output}/state.json',
            '-p', f'wal_path:={args.output}/wal.jsonl',
            '-p', f'txn_log_path:={args.output}/transactions.jsonl'])
        for mode in MODES:
            prefix = f'/index_test/{mode}'
            processes.start('gate_' + mode, ['ros2', 'run', 'topopatch_ros', 'gate_node', '--ros-args',
                '-r', f'__node:=gate_{mode}', '-p', f'use_polygon_index:={str(mode == "indexed").lower()}',
                '-p', f'in_topic:={prefix}/candidate', '-p', f'status_topic:={prefix}/status',
                '-p', f'authorization_topic:={prefix}/auth'])
            processes.start('guard_' + mode, ['ros2', 'run', 'rulesync_guard', 'ruleguard', '--ros-args',
                '-r', f'__node:=guard_{mode}', '-p', f'authorization_topic:={prefix}/auth',
                '-p', f'executable_topic:={prefix}/executable', '-p', f'state_topic:={prefix}/state'])
        rclpy.init()
        probe = Probe()
        probe.wait(lambda: all(probe.pub[m].get_subscription_count() > 0 for m in MODES), 45.)
        straight = [(-2., 0.), (-1., 0.), (1., 0.), (2., 0.)]
        detour = [(-2., 0.), (-1., 1.5), (1., 1.5), (2., 0.)]
        relaxed_path = [(-2., 0.), (-1., .9), (1., .9), (2., 0.)]
        snap = probe.snapshot(0)
        rows.append(probe.candidate('empty', straight, snap, True))
        old_auth = dict(probe.auth)
        for epoch, relaxed in [(1, False), (2, True)]:
            req = ProposeRule.Request(rule=rule(epoch, relaxed))
            assert probe.call('propose', req).accepted
            assert probe.call('commit', CommitRule.Request(epoch=epoch)).committed
            snap = probe.snapshot(epoch)
            for mode in MODES:
                probe.wait(lambda: probe.state[mode].state == RuleGuardState.HOLD)
            # Replaying a complete, valid old token must still fail current authority.
            if epoch == 1:
                for mode in MODES:
                    replay = probe.create_publisher(ExecutionAuthorization, f'/index_test/{mode}/auth', 10)
                    probe.wait(lambda: replay.get_subscription_count() >= 2)
                    receipt_start = len(probe.receipts)
                    path_start = len(probe.paths[mode])
                    replay.publish(old_auth[mode])
                    probe.wait(lambda: any(r.get('mode') == mode and
                        r.get('reason') == 'authorization-identity-mismatch'
                        for r in probe.receipts[receipt_start:]))
                    assert not any(p.poses for p in probe.paths[mode][path_start:])
                    rows.append(dict(label='stale-authorization-replay', mode=mode, rejected=True))
            rows.append(probe.candidate('crossing-e' + str(epoch), straight, snap, False))
            rows.append(probe.candidate('relax-test-e' + str(epoch), relaxed_path, snap, relaxed))
            for repetition in range(args.repetitions):
                rows.append(probe.candidate('detour-e' + str(epoch), detour, snap, True, repeat=repetition))
            rows.append(probe.candidate('frame-mismatch-e' + str(epoch), detour, snap, False, frame='odom'))
        assert probe.call('remove', RemoveRule.Request(rule_id='index-test-zone', epoch=3)).ok
        snap = probe.snapshot(3)
        for repetition in range(args.repetitions):
            rows.append(probe.candidate('removed', straight, snap, True, repeat=repetition))
        assert probe.call('propose', ProposeRule.Request(rule=rule(4))).accepted
        assert probe.call('commit', CommitRule.Request(epoch=4)).committed
        snap = probe.snapshot(4)
        rows.append(probe.candidate('restored', detour, snap, True))
        probe.pose_xy = (0., 0.)
        for mode in MODES:
            before = len(probe.paths[mode])
            probe.wait(lambda: probe.state[mode].state == RuleGuardState.HOLD and
                probe.state[mode].reason in ('boundary-violated', 'boundary-violation', 'inside-forbidden'))
            probe.wait(lambda: any(not p.poses for p in probe.paths[mode][before:]))
            rows.append(dict(label='pose-boundary-HOLD', mode=mode, stopped_path=True))
        (args.output / 'summary.json').write_text(json.dumps(
            dict(passed=True, checks=rows, paired_candidates=sum('results' in r for r in rows)), indent=2) + '\n')
        print(json.dumps(dict(passed=True, output=str(args.output), checks=len(rows))), flush=True)
    except BaseException as exc:
        (args.output / 'failure.json').write_text(json.dumps(
            dict(type=type(exc).__name__, error=str(exc), completed_checks=rows), indent=2) + '\n')
        raise
    finally:
        if probe is not None:
            (args.output / 'receipts.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in probe.receipts))
            probe.destroy_node()
        processes.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
