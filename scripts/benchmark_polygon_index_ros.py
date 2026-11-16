#!/usr/bin/env python3
"""Paired production ROS admission benchmark, not a navigation/stop-time test.

Cold caches follow real durable commits; reuse keeps the complete identity.
Instrumentation delegates unchanged production callbacks and predicates.
Synthetic free occupancy, serialized candidates, alternating backend order.
"""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import statistics
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from rulesync_interfaces.msg import ExecutionAuthorization, RuleSpec
from rulesync_interfaces.srv import CommitRule, ListRules, ProposeRule
from rulesync3d.authorization import path_digest, token_id
from rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash

from benchmark_polygon_index import polygons_for
from verify_polygon_index_ros import Processes, SOURCE_PATHS

MODES = ('direct', 'indexed')


def worker(ros_args):
    import topopatch_ros.gate_node as production
    original_gate = production.gate_full_path

    class TrackedCache:
        def __init__(self, delegate, owner):
            self.delegate, self.owner = delegate, owner

        def get(self, *args, **kwargs):
            begin = time.monotonic_ns()
            value = self.delegate.get(*args, **kwargs)
            self.owner.metrics.update(cache_ms=(time.monotonic_ns()-begin)/1e6,
                                      cache_hit=value[1])
            return value

    class MeasuredGate(production.GateNode):
        def __init__(self):
            super().__init__()
            self.metrics = {}
            self._polygon_index = TrackedCache(self._polygon_index, self)
            self.telemetry = self.create_publisher(String,
                str(self.get_parameter('status_topic').value) + '/timing', 10)

        def _on_candidate(self, msg):
            self.begin = time.monotonic_ns()
            self.metrics = dict(job=msg.header.stamp.sec, cache_hit=None, cache_ms=0.)
            self.listed = None
            super()._on_candidate(msg)

        def _on_listed(self, future):
            self.listed = time.monotonic_ns()
            super()._on_listed(future)

        def _verdict(self, *args, **kwargs):
            end = time.monotonic_ns()
            self.metrics.update(node_ms=(end-self.begin)/1e6,
                service_wait_ms=(self.listed-self.begin)/1e6 if self.listed else None,
                has_map=self._map is not None)
            super()._verdict(*args, **kwargs)
            self.telemetry.publish(String(data=json.dumps(self.metrics)))

    rclpy.init(args=ros_args)
    node = MeasuredGate()

    def measured_gate(*args, **kwargs):
        begin = time.monotonic_ns()
        try:
            return original_gate(*args, **kwargs)
        finally:
            node.metrics['full_gate_ms'] = (time.monotonic_ns()-begin)/1e6

    production.gate_full_path = measured_gate
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Probe(Node):
    def __init__(self):
        super().__init__('paired_ros_benchmark')
        self.status, self.auth, self.timing = {}, {}, {}
        self.receipts = []
        self.pub = {m: self.create_publisher(PathMsg, f'/perf/{m}/candidate', 10) for m in MODES}
        for mode in MODES:
            for kind, msg_type, suffix in [('status', String, 'status'),
                    ('timing', String, 'status/timing'), ('auth', ExecutionAuthorization, 'auth')]:
                self.create_subscription(msg_type, f'/perf/{mode}/{suffix}',
                    lambda msg, m=mode, k=kind: self.receive(m, k, msg), 10)
        self._test_clients = {
            'list': self.create_client(ListRules, '/rulesync/list_rules'),
            'propose': self.create_client(ProposeRule, '/rulesync/propose_rule'),
            'commit': self.create_client(CommitRule, '/rulesync/commit_rule')}
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 100
        grid.info.resolution = 2.
        grid.info.origin.position.x, grid.info.origin.position.y = -20., -50.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*10000
        self.map_pub.publish(grid)
        self.job = 0

    def receive(self, mode, kind, msg):
        stamp = time.monotonic_ns()  # before any validation or JSON conversion
        value = msg if kind == 'auth' else json.loads(msg.data)
        getattr(self, kind)[mode] = (stamp, value)

    def wait(self, predicate, timeout=60.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.005)
            if predicate():
                return
        raise TimeoutError('benchmark ROS condition timeout')

    def call(self, name, request):
        client = self._test_clients[name]
        self.wait(client.service_is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        assert result is not None
        return result

    def commit(self, epoch, polygons, all_rules=False):
        for i, poly in enumerate(polygons if all_rules else polygons[:1]):
            rule = RuleSpec(rule_id=f'zone-{i}', epoch=epoch, rule_type=0,
                            severity=1, frame_id='map')
            rule.polygon.points = [Point32(x=float(x), y=float(y), z=0.) for x, y in poly]
            rule.valid_until.sec = 3600
            response = self.call('propose', ProposeRule.Request(rule=rule))
            assert response.accepted, response.reason
        response = self.call('commit', CommitRule.Request(epoch=epoch))
        assert response.committed, response.reason
        deadline = time.monotonic() + 60.
        while time.monotonic() < deadline:
            result = self.call('list', ListRules.Request())
            if result.epoch == epoch and not result.degraded:
                assert len(result.active) == len(polygons)
                assert result.active_hash == snapshot_hash(rule_snapshot_items(result.active))
                return result
            until = time.monotonic() + .1
            self.wait(lambda: time.monotonic() >= until)
        raise RuntimeError('committed snapshot unavailable')

    def pair(self, points, snapshot, accepted, repeat, cold, metadata):
        self.job += 1
        path = PathMsg()
        path.header.frame_id, path.header.stamp.sec = 'map', self.job
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        digest = path_digest(path)
        expected_token = token_id(snapshot.epoch, snapshot.active_hash, digest) if accepted else ''
        result = {}
        for mode in MODES if repeat % 2 == 0 else MODES[::-1]:
            for values in (self.status, self.auth, self.timing):
                values.pop(mode, None)
            begin = time.monotonic_ns()
            self.pub[mode].publish(path)
            self.wait(lambda: mode in self.status and mode in self.timing)
            received, verdict = self.status[mode]
            timing = self.timing[mode][1]
            assert timing['job'] == self.job and timing['has_map'], timing
            assert (verdict['execute'], verdict['epoch'], verdict['snapshot_hash'], verdict['token_id']) == (
                accepted, snapshot.epoch, snapshot.active_hash, expected_token), verdict
            assert verdict['reason'] == ('gate:ok' if accepted else 'gate:KEEP_OUT-violation'), verdict
            if mode == 'indexed':
                assert timing['cache_hit'] == (not cold), timing
            if accepted:
                self.wait(lambda: mode in self.auth)
                auth = self.auth[mode][1]
                assert (auth.epoch, auth.snapshot_hash, auth.path_digest, auth.token_id) == (
                    snapshot.epoch, snapshot.active_hash, digest, expected_token)
                assert path_digest(auth.path) == digest
            else:
                until = time.monotonic() + .05
                self.wait(lambda: time.monotonic() >= until)
                assert mode not in self.auth, 'unexpected authorization'
            result[mode] = dict(timing, receipt_ms=(received-begin)/1e6,
                verdict={k: verdict[k] for k in ('execute', 'reason', 'epoch', 'snapshot_hash', 'token_id')})
        assert result['direct']['verdict'] == result['indexed']['verdict']
        return dict(metadata, job=self.job, repeat=repeat, cache='cold' if cold else 'reuse',
                    expected_accept=accepted, digest=digest, epoch=snapshot.epoch,
                    snapshot_hash=snapshot.active_hash, results=result)


def paths_for(layout, segments):
    admitted = np.column_stack((np.linspace(0., 20., segments+1), np.zeros(segments+1)))
    start, end = {'separated': ((29.5, 30.2), (30.9, 30.2)),
                  'corridor': ((.3, -1.), (1.7, -1.)),
                  'dense': ((0., -2.5), (0., -.5))}[layout]
    rejected = [start] + [end]*segments  # first continuous segment crosses, endpoints outside
    return [('admitted', admitted, True), ('early_crossing', rejected, False)]


def run_cell(args, layout, count, segments, output):
    output.mkdir()
    processes, probe = Processes(output), None
    rows = []
    try:
        os.environ['RULESYNC_TEST_POSE2D'] = '-10,10'
        os.environ['RULESYNC_TEST_GOAL2D'] = '160,10'
        processes.start('bridge', ['ros2', 'run', 'rulesync_nav2_bridge', 'bridge', '--ros-args',
            '-p', 'auto_replan:=false', '-p', 'auto_rollback:=false',
            '-p', 'grid_width:=100', '-p', 'grid_height:=100', '-p', 'grid_resolution:=2.0',
            '-p', 'grid_origin_x:=-20.0', '-p', 'grid_origin_y:=-50.0'])
        processes.start('manager', ['ros2', 'run', 'rulesync_manager', 'manager', '--ros-args',
            '-p', f'state_path:={output}/state.json', '-p', f'wal_path:={output}/wal.jsonl',
            '-p', f'txn_log_path:={output}/transactions.jsonl'])
        for mode in MODES:
            processes.start(mode, [sys.executable, str(Path(__file__).resolve()), '--worker', '--ros-args',
                '-r', f'__node:=perf_{mode}', '-p', f'use_polygon_index:={str(mode == "indexed").lower()}',
                '-p', f'in_topic:=/perf/{mode}/candidate', '-p', f'status_topic:=/perf/{mode}/status',
                '-p', f'authorization_topic:=/perf/{mode}/auth'])
        rclpy.init()
        probe = Probe()
        probe.wait(lambda: all(probe.pub[m].get_subscription_count() for m in MODES))
        # Service discovery precedes bridge constructor/reconciliation completion.
        deadline = time.monotonic() + 60.
        while True:
            initial = probe.call('list', ListRules.Request())
            if not initial.degraded:
                assert initial.epoch == 0 and not initial.active
                assert initial.active_hash == snapshot_hash(rule_snapshot_items(initial.active))
                break
            assert time.monotonic() < deadline, 'startup reconciliation timeout'
            until = time.monotonic() + .1
            probe.wait(lambda: time.monotonic() >= until)
        polygons = polygons_for(layout, count)
        epoch = 1
        snap = probe.commit(epoch, polygons, True)
        with (output/'raw.jsonl').open('w') as stream:
            for outcome, points, accepted in paths_for(layout, segments):
                for repeat in range(-args.warmups, args.repeats):
                    epoch += 1
                    old_hash = snap.active_hash
                    snap = probe.commit(epoch, polygons)
                    assert old_hash != snap.active_hash
                    for cold in (True, False):
                        row = probe.pair(points, snap, accepted, repeat, cold,
                            dict(layout=layout, rules=count, segments=segments, outcome=outcome,
                                 warmup=repeat < 0))
                        stream.write(json.dumps(row)+'\n')
                        stream.flush()
                        if repeat >= 0:
                            rows.append(row)
                print(json.dumps(dict(layout=layout, rules=count, segments=segments,
                                      outcome=outcome, complete=True)), flush=True)
    except BaseException as exc:
        (output/'failure.json').write_text(json.dumps(dict(type=type(exc).__name__, error=str(exc))))
        raise
    finally:
        if probe:
            probe.destroy_node()
        processes.close()
        if rclpy.ok():
            rclpy.shutdown()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--pilot', action='store_true')
    parser.add_argument('--repeats', type=int, default=30)
    parser.add_argument('--warmups', type=int, default=5)
    args, remaining = parser.parse_known_args()
    if args.worker:
        worker(remaining)
        return
    if remaining or args.output is None or args.repeats < 2 or args.warmups < 0:
        parser.error('require fresh --output, repeats>=2, warmups>=0')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    cells = [(l, r, n) for l in ('separated', 'corridor', 'dense')
             for r, n in ([(3, 10)] if args.pilot else [(10, 100), (100, 1000)])]
    source_paths = tuple(SOURCE_PATHS) + ('scripts/benchmark_polygon_index.py',
                                         'scripts/benchmark_polygon_index_ros.py')
    manifest = dict(pilot=args.pilot, cells=cells, repeats=args.repeats, warmups=args.warmups,
        rho=.2, map=dict(width=100, height=100, resolution=2., origin=[-20., -50.], free=True),
        scope='manager+bridge+two production gate callbacks; no controller, RuleGuard or perception',
        cold='real commit advances epoch/full hash with unchanged geometry; assert cache miss',
        reuse='same committed identity immediately after cold; assert cache hit',
        timing='same-host monotonic_ns; receipt=publish-to-original-status; node=candidate-to-verdict entry; '
               'full_gate=geometry plus sampled occupancy; service_wait includes ROS scheduling',
        pair_order='alternating by repeat; serialized input; timestamp before receipt verification',
        rejection_observation_s=.05, platform=platform.platform(), python=platform.python_version(),
        numpy=np.__version__, ros_distro=os.environ.get('ROS_DISTRO'),
        rmw=rclpy.utilities.get_rmw_implementation_identifier(), domain=os.environ.get('ROS_DOMAIN_ID'),
        source_sha256={p: hashlib.sha256((root/p).read_bytes()).hexdigest() for p in source_paths})
    installed = {}
    for module, relative in (
            ('topopatch_ros.gate_node', 'topopatch_ros/topopatch_ros/gate_node.py'),
            ('rulesync3d.gate2d', 'rulesync3d/rulesync3d/gate2d.py'),
            ('rulesync_manager.manager_node', 'rulesync/src/rulesync_manager/rulesync_manager/manager_node.py'),
            ('rulesync_nav2_bridge.bridge_node', 'rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py')):
        installed[module] = hashlib.sha256(Path(importlib.import_module(module).__file__).read_bytes()).hexdigest()
        assert installed[module] == manifest['source_sha256'][relative], 'stale installed production code'
    manifest.update(installed_sha256=installed,
        cpu_affinity=sorted(os.sched_getaffinity(0)),
        instrumentation='benchmark-only subclass/timer wrappers; original callbacks/predicates delegated unchanged',
        intervals='descriptive repeated-run distributions, not independent robot trials or hard real-time bounds')
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    rows = []
    for layout, count, segments in cells:
        rows.extend(run_cell(args, layout, count, segments,
            args.output/f'{layout}_{count}_{segments}'))
    summaries = []
    for layout, count, segments in cells:
        for outcome in ('admitted', 'early_crossing'):
            for cache in ('cold', 'reuse'):
                cell = [r for r in rows if (r['layout'], r['rules'], r['segments'], r['outcome'], r['cache']) ==
                        (layout, count, segments, outcome, cache)]
                summary = dict(layout=layout, rules=count, segments=segments, outcome=outcome,
                               cache=cache, pairs=len(cell))
                for metric in ('receipt_ms', 'node_ms', 'service_wait_ms', 'full_gate_ms', 'cache_ms'):
                    for mode in MODES:
                        values = [r['results'][mode][metric] for r in cell]
                        summary[f'{mode}_{metric}'] = dict(median=statistics.median(values),
                            p95=float(np.percentile(values, 95)), min=min(values), max=max(values))
                summary['paired_receipt_speedup_median'] = statistics.median(
                    r['results']['direct']['receipt_ms']/r['results']['indexed']['receipt_ms'] for r in cell)
                summaries.append(summary)
    (args.output/'summary.json').write_text(json.dumps(dict(passed=True, pairs=len(rows),
        candidates=2*len(rows), summaries=summaries), indent=2)+'\n')
    print(json.dumps(dict(passed=True, pairs=len(rows), output=str(args.output))), flush=True)


if __name__ == '__main__':
    main()
