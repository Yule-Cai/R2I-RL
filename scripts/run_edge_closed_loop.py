#!/usr/bin/env python3
"""All-local ROS 2 RuleSync kinematic closed loop, without Gazebo or Nav2.

Uses unchanged production manager/bridge/gate/guard in separate processes.
Only the waypoint proposer, simple controller and ideal 2-D plant are new.
No sensor/SLAM, inertia, collision/contact physics or physical brake model.
Wall-clock integration, perfect map-frame pose, local DDS only. Not a robot
trial or a substitute for Gazebo campaigns. Fresh output directories required.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Point32, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from rulesync_interfaces.msg import RuleGuardState, RuleSpec
from rulesync_interfaces.srv import CommitRule, ListRules, ProposeRule

from edge_sim_core import (feedback_fresh, pending_release_matches, release_matches,
                           step_unicycle, track_waypoints)
from rulesync3d.authorization import path_digest, token_id
from verify_polygon_index_ros import Processes, SOURCE_PATHS

PREFIX = '/edge_sim'


class Controller(Node):
    """Consumes ONLY RuleGuard output; heartbeat loss clears cached path."""
    def __init__(self):
        super().__init__('edge_sim_controller')
        self.path, self.index, self.pose = [], 0, None
        self.pending, self.pending_at, self.active_token = None, None, ''
        self.last_status = None
        self.pose_at = self.guard_at = None
        self.state = None
        self.create_subscription(PathMsg, PREFIX + '/executable', self.on_path, 10)
        self.create_subscription(RuleGuardState, PREFIX + '/guard_state', self.on_guard, 20)
        self.create_subscription(PoseStamped, PREFIX + '/pose', self.on_pose, 20)
        self.command = self.create_publisher(TwistStamped, PREFIX + '/cmd', 10)
        self.create_timer(.05, self.tick)

    def on_path(self, msg):
        if not msg.poses:
            self.path, self.pending, self.active_token = [], None, ''
        else:
            # Different ROS topics have no shared delivery order. Retain the
            # new path pending its matching state, rather than deleting it on
            # a tick that still sees the previous HOLD state.
            self.pending, self.pending_at = msg, time.monotonic()

    def on_guard(self, msg):
        self.state, self.guard_at = msg, time.monotonic()
        if msg.state == RuleGuardState.HOLD or msg.degraded:
            self.path = []
            self.active_token = ''

    def on_pose(self, msg):
        q = msg.pose.orientation
        theta = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        self.pose = (msg.pose.position.x, msg.pose.position.y, theta)
        self.pose_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        # Expiry precedes activation, including when the matching state arrives
        # on this tick. Never revive a pending path beyond its join deadline.
        if self.pending is not None and not feedback_fresh(now, self.pending_at, 2.):
            self.pending = None
        healthy = (feedback_fresh(now, self.guard_at, .5) and
                   feedback_fresh(now, self.pose_at, .3) and self.state is not None and
                   self.state.state != RuleGuardState.HOLD and not self.state.degraded)
        velocity = (0., 0.)
        if not healthy:
            self.path = []  # recovery alone MUST NOT revive a cached path
            self.active_token = ''
        else:
            if self.pending is not None:
                expected = token_id(self.state.execution_epoch, self.state.execution_hash,
                                    path_digest(self.pending))
                if pending_release_matches(now, self.pending_at,
                    (self.state.authority_epoch, self.state.authority_hash),
                    (self.state.execution_epoch, self.state.execution_hash), self.state.token_id, expected):
                    self.path = [(p.pose.position.x, p.pose.position.y) for p in self.pending.poses]
                    self.index, self.active_token, self.pending = 0, expected, None
            # Identity agreement is required on EVERY command tick, not only
            # when activating a path. State/token delivery may be asynchronous.
            if not release_matches((self.state.authority_epoch, self.state.authority_hash),
                (self.state.execution_epoch, self.state.execution_hash),
                self.state.token_id, self.active_token):
                self.path = []
                self.active_token = ''
            if self.path:
                velocity, self.index = track_waypoints(self.pose, self.path, self.index)
        status = ('feedback-stop' if not healthy else
                  'await-path-state-join' if self.pending is not None else
                  'tracking' if self.path and self.index < len(self.path) else 'idle')
        if status != self.last_status:
            self.get_logger().info(f'adapter {status}; token={self.active_token}')
            self.last_status = status
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = velocity
        self.command.publish(msg)


class Plant(Node):
    def __init__(self):
        super().__init__('edge_sim_plant')
        self.pose = (-2., 0., 0.)
        self.velocity, self.command_at = (0., 0.), None
        self.previous = time.monotonic()
        self.pub = self.create_publisher(PoseStamped, PREFIX + '/pose', 20)
        self.diagnostics = self.create_publisher(String, PREFIX + '/plant_status', 20)
        self.create_subscription(TwistStamped, PREFIX + '/cmd', self.on_command, 10)
        self.create_timer(.02, self.tick)

    def on_command(self, msg):
        v, w = msg.twist.linear.x, msg.twist.angular.z
        self.velocity = (v, w) if math.isfinite(v) and math.isfinite(w) else (0., 0.)
        self.command_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.previous
        dt = min(elapsed, .1)  # record discarded time, never jump after a long scheduler stall
        self.previous = now
        velocity = self.velocity if feedback_fresh(now, self.command_at, .3) else (0., 0.)
        self.pose = step_unicycle(self.pose, velocity, dt)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.pose[:2]
        msg.pose.orientation.z = math.sin(self.pose[2] / 2)
        msg.pose.orientation.w = math.cos(self.pose[2] / 2)
        self.pub.publish(msg)
        status = String()
        status.data = json.dumps(dict(at_ns=time.monotonic_ns(), pose=self.pose,
            velocity=velocity, elapsed_s=elapsed, integrated_s=dt,
            discarded_s=max(0., elapsed - dt)))
        self.diagnostics.publish(status)


class Scenario(Node):
    def __init__(self, output, processes):
        super().__init__('edge_sim_scenario')
        self.output, self.processes = output, processes
        self.log = (output / 'receipts.jsonl').open('w')
        self.resources = (output / 'resources.jsonl').open('w')
        self.xy, self.state, self.verdict, self.plant = None, None, None, None
        self.command, self.state_at = (0., 0.), 0.
        self.checks, self.events = [], []
        self.samples, self.discarded, self.max_dt = 0, 0., 0.
        self.rule_active, self.min_clearance = False, math.inf
        self.candidate = self.create_publisher(PathMsg, PREFIX + '/candidate', 10)
        self.create_subscription(PoseStamped, PREFIX + '/pose', self.on_pose, 20)
        self.create_subscription(RuleGuardState, PREFIX + '/guard_state', self.on_state, 20)
        self.create_subscription(String, PREFIX + '/gate_status', self.on_verdict, 10)
        self.create_subscription(String, PREFIX + '/plant_status', self.on_plant, 20)
        self.create_subscription(TwistStamped, PREFIX + '/cmd', self.on_command, 20)
        self.create_subscription(PathMsg, PREFIX + '/executable',
            lambda msg: self.receipt('executable', points=len(msg.poses)), 20)
        self._service_clients = {n: self.create_client(cls, '/rulesync/' + service) for n, cls, service in
            [('list', ListRules, 'list_rules'), ('propose', ProposeRule, 'propose_rule'),
             ('commit', CommitRule, 'commit_rule')]}
        self.map = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .1
        grid.info.origin.position.x = grid.info.origin.position.y = -4.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 6400
        self.map.publish(grid)
        self.create_timer(1., self.sample_resources)

    def receipt(self, kind, **values):
        self.log.write(json.dumps(dict(received_ns=time.monotonic_ns(), kind=kind, **values)) + '\n')

    def on_pose(self, msg):
        self.xy = (msg.pose.position.x, msg.pose.position.y)

    def on_state(self, msg):
        self.state, self.state_at = msg, time.monotonic()
        self.receipt('guard', state=msg.state, reason=msg.reason,
            epoch=msg.authority_epoch, execution_epoch=msg.execution_epoch, token=msg.token_id)

    def on_verdict(self, msg):
        self.verdict = json.loads(msg.data)
        self.receipt('gate', **self.verdict)

    def on_plant(self, msg):
        self.plant = json.loads(msg.data)
        self.samples += 1
        self.discarded += self.plant['discarded_s']
        self.max_dt = max(self.max_dt, self.plant['elapsed_s'])
        if self.rule_active:
            x, y = self.plant['pose'][:2]
            dx, dy = max(-.3 - x, 0., x - .6), max(-.5 - y, 0., y - .5)
            clearance = math.hypot(dx, dy)
            if -.3 <= x <= .6 and -.5 <= y <= .5:
                clearance = -min(x + .3, .6 - x, y + .5, .5 - y)
            self.min_clearance = min(self.min_clearance, clearance)
        self.receipt('plant', **self.plant)

    def on_command(self, msg):
        self.command = (msg.twist.linear.x, msg.twist.angular.z)
        self.receipt('command', velocity=self.command)

    def event(self, name):
        row = dict(name=name, at_ns=time.monotonic_ns(), pose=self.xy)
        self.events.append(row)
        self.receipt('event', **row)
        self.log.flush()
        print(json.dumps(row), flush=True)
        return row

    def sample_resources(self):
        row = dict(at_ns=time.monotonic_ns(), processes=[])
        # Include ros2-run launcher descendants, not just launcher RSS.
        records = {}
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                values = path.read_text().rsplit(')', 1)[1].split()
                records[int(path.parent.name)] = values
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                pass
        pids = {os.getpid()}
        while True:
            descendants = {pid for pid, values in records.items() if int(values[1]) in pids}
            if descendants <= pids:
                break
            pids |= descendants
        for pid in sorted(pids & records.keys()):
            values = records[pid]
            row['processes'].append(dict(pid=pid, cpu_ticks=int(values[11]) + int(values[12]),
                rss_bytes=int(values[21]) * os.sysconf('SC_PAGE_SIZE')))
        row['temperatures_mC'] = {p.parent.name: int(p.read_text()) for p in
            Path('/sys/class/thermal').glob('thermal_zone*/temp')}
        row['meminfo'] = Path('/proc/meminfo').read_text()
        self.resources.write(json.dumps(row) + '\n')
        self.resources.flush()
        if max(row['temperatures_mC'].values(), default=0) >= 85000:
            raise RuntimeError('thermal protection: sampled temperature >=85 C; stop trial')

    def wait(self, predicate, timeout=45., allow_dead=()):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            for process, _ in self.processes.children:
                if process.poll() is not None and process.pid not in allow_dead:
                    raise RuntimeError(f'child exited unexpectedly: pid={process.pid}, code={process.returncode}')
            if predicate():
                return
        raise TimeoutError('scenario condition timed out')

    def call(self, name, request):
        client = self._service_clients[name]
        self.wait(client.service_is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        if future.result() is None:
            raise RuntimeError('empty service response')
        return future.result()

    def snapshot(self, epoch):
        def converged():
            return self.state is not None and self.state.authority_epoch == epoch and bool(self.state.authority_hash)
        self.wait(converged)
        response = self.call('list', ListRules.Request())
        assert response.epoch == epoch and not response.degraded
        return response

    def propose_path(self, label, points, accepted, epoch, attempt=0):
        # Existing gate requires >=4 poses. Subdivide only; do not change the polyline.
        points = list(points)
        while len(points) < 4:
            index = max(range(len(points) - 1), key=lambda i: math.dist(points[i], points[i + 1]))
            points.insert(index + 1, tuple((a + b) / 2 for a, b in zip(points[index], points[index + 1])))
        msg = PathMsg()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.
            msg.poses.append(pose)
        self.verdict = None
        self.event(label + '-submitted')
        self.candidate.publish(msg)
        self.wait(lambda: self.verdict is not None)
        assert self.verdict['execute'] == accepted, self.verdict
        assert self.verdict['epoch'] == epoch, self.verdict
        if accepted:
            try:
                self.wait(lambda: self.state.state != RuleGuardState.HOLD and
                    self.state.execution_epoch == epoch and self.state.token_id == self.verdict['token_id'],
                    timeout=4.)
            except TimeoutError:
                # A newly discovered guard can get authority before its pose
                # subscriber receives feedback. Do not weaken the guard. The
                # application requests a NEW authorization from current pose.
                if (label != 'fresh-authorization-after-restart' or attempt >= 2 or
                    self.state.state != RuleGuardState.HOLD or self.state.reason not in
                    ('actual-pose-stale', 'execution-not-bound')):
                    raise
                self.event('new-proposal-after-pose-readiness-delay')
                return self.propose_path(label, [self.xy, *points[1:]], accepted, epoch, attempt + 1)
        self.checks.append(dict(name=label, accepted=accepted, epoch=epoch, passed=True))

    def stopped(self):
        return self.plant is not None and self.command == (0., 0.) and self.plant['velocity'] == [0., 0.]


def run_scenario(output):
    root = Path(__file__).resolve().parents[1]
    manifest = dict(scope='independent board-local ideal 2-D kinematic ROS simulation',
        platform=platform.platform(), python=platform.python_version(), ros=os.environ.get('ROS_DISTRO'),
        domain=os.environ.get('ROS_DOMAIN_ID'), discovery=os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE'),
        fastdds_builtin_transports=os.environ.get('FASTDDS_BUILTIN_TRANSPORTS'),
        cpu_affinity=sorted(os.sched_getaffinity(0)), thermal_abort_C=85.,
        speed_m_s=.35, controller_period_s=.05, plant_period_s=.02,
        guard_watchdog_s=.5, command_watchdog_s=.3, rho_m=.2,
        assumptions=['perfect pose', 'free static map', 'instantaneous velocity response',
                     'no physics or braking', 'manually specified candidate paths', 'local DDS'],
        source_sha256={p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in
            (*SOURCE_PATHS, 'scripts/edge_sim_core.py', 'scripts/run_edge_closed_loop.py')})
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    children, node = Processes(output), None
    started = time.monotonic()
    try:
        os.environ['RULESYNC_TEST_POSE2D'] = '-2,0'
        os.environ['RULESYNC_TEST_GOAL2D'] = '2,0'
        children.start('bridge', ['ros2', 'run', 'rulesync_nav2_bridge', 'bridge', '--ros-args',
            '-p', 'auto_replan:=false', '-p', 'auto_rollback:=false',
            '-p', 'grid_width:=80', '-p', 'grid_height:=80', '-p', 'grid_resolution:=0.1',
            '-p', 'grid_origin_x:=-4.0', '-p', 'grid_origin_y:=-4.0'])
        children.start('manager', ['ros2', 'run', 'rulesync_manager', 'manager', '--ros-args',
            '-p', f'state_path:={output}/state.json', '-p', f'wal_path:={output}/wal.jsonl',
            '-p', f'txn_log_path:={output}/transactions.jsonl'])
        children.start('gate', ['ros2', 'run', 'topopatch_ros', 'gate_node', '--ros-args',
            '-p', 'use_polygon_index:=true', '-p', f'in_topic:={PREFIX}/candidate',
            '-p', f'status_topic:={PREFIX}/gate_status', '-p', f'authorization_topic:={PREFIX}/auth'])
        guard_command = ['ros2', 'run', 'rulesync_guard', 'ruleguard', '--ros-args',
            '-p', f'pose_topic:={PREFIX}/pose', '-p', f'authorization_topic:={PREFIX}/auth',
            '-p', f'executable_topic:={PREFIX}/executable', '-p', f'state_topic:={PREFIX}/guard_state',
            '-p', 'hold_publish_s:=0.05']
        children.start('guard', guard_command)
        guard_process = children.children[-1][0]
        for role in ('controller', 'plant'):
            children.start(role, [sys.executable, str(Path(__file__).resolve()), '--role', role])
            if role == 'controller':
                controller_process = children.children[-1][0]
        rclpy.init()
        node = Scenario(output, children)
        node.wait(lambda: node.xy is not None and node.candidate.get_subscription_count() > 0)
        node.snapshot(0)
        node.propose_path('empty-snapshot-motion', [node.xy, (2., 0.)], True, 0)
        node.wait(lambda: node.xy[0] > -1.6 and node.command[0] > .1)
        node.event('motion-before-update')
        rule = RuleSpec(rule_id='edge-zone', epoch=1, rule_type=0, severity=1, frame_id='map')
        rule.polygon.points = [Point32(x=x, y=y, z=0.) for x, y in
            [(-.3, -.5), (.6, -.5), (.6, .5), (-.3, .5)]]
        rule.valid_until.sec = 3600
        assert node.call('propose', ProposeRule.Request(rule=rule)).accepted
        node.event('commit-request')
        assert node.call('commit', CommitRule.Request(epoch=1)).committed
        node.rule_active = True
        node.event('commit-response-observed')
        node.snapshot(1)
        node.wait(lambda: node.state.state == RuleGuardState.HOLD and node.stopped())
        node.event('revocation-stop-observed')
        assert node.xy[0] < -.5, node.xy  # well before represented forbidden region
        node.checks.append(dict(name='moving-old-authorization-revoked', passed=True, stopped_pose=node.xy))
        node.propose_path('crossing-denied', [node.xy, (2., 0.)], False, 1)
        until = time.monotonic() + .3
        node.wait(lambda: time.monotonic() >= until)
        assert node.stopped() and node.state.state == RuleGuardState.HOLD
        node.propose_path('detour-resume', [node.xy, (-1., 1.2), (1., 1.2), (2., 0.)], True, 1)
        node.wait(lambda: node.xy[1] > .35 and node.command[0] > .1)
        node.event('guard-process-interrupted')
        os.killpg(guard_process.pid, signal.SIGKILL)
        guard_process.wait(timeout=5)
        # Remove the deliberately dead child from health checks; it remains in cleanup list.
        node.wait(lambda: node.stopped() and time.monotonic() - node.state_at > .5,
                  timeout=5., allow_dead=(guard_process.pid,))
        stopped_xy = node.xy
        node.event('watchdog-stop-observed')
        until = time.monotonic() + .7
        node.wait(lambda: time.monotonic() >= until, allow_dead=(guard_process.pid,))
        assert math.dist(node.xy, stopped_xy) < 1e-6, (node.xy, stopped_xy)
        node.checks.append(dict(name='guard-process-loss-watchdog', passed=True, stopped_pose=node.xy))
        # Expected dead process remains evidence but no longer counts as unexpected failure.
        dead_entry = next(entry for entry in children.children if entry[0] is guard_process)
        children.children.remove(dead_entry)
        dead_entry[1].close()
        restarted_at = time.monotonic()
        children.start('guard_restart', guard_command)
        node.wait(lambda: node.state_at > restarted_at and node.state.authority_epoch == 1)
        until = time.monotonic() + .7
        node.wait(lambda: time.monotonic() >= until)
        assert node.stopped() and math.dist(node.xy, stopped_xy) < 1e-6
        node.checks.append(dict(name='restart-no-silent-resume', passed=True))
        node.propose_path('fresh-authorization-after-restart',
            [node.xy, (-1., 1.2), (1., 1.2), (2., 0.)], True, 1)
        node.wait(lambda: math.dist(node.xy, (2., 0.)) < .07 and node.stopped(), timeout=70.)
        node.event('goal-arrival')
        node.checks.append(dict(name='detour-goal-arrival', passed=True, final_pose=node.xy))
        node.propose_path('command-watchdog-motion', [node.xy, (2., -1.2)], True, 1)
        node.wait(lambda: node.xy[1] < -.25 and node.command[0] > .1)
        node.event('controller-process-interrupted')
        os.killpg(controller_process.pid, signal.SIGKILL)
        controller_process.wait(timeout=5)
        node.wait(lambda: node.plant['velocity'] == [0., 0.], timeout=5.,
                  allow_dead=(controller_process.pid,))
        stopped_xy = node.xy
        node.event('plant-command-watchdog-stop-observed')
        until = time.monotonic() + .7
        node.wait(lambda: time.monotonic() >= until, allow_dead=(controller_process.pid,))
        assert math.dist(node.xy, stopped_xy) < 1e-6
        node.checks.append(dict(name='controller-process-loss-plant-watchdog', passed=True,
            stopped_pose=node.xy))
        assert node.min_clearance > .2, node.min_clearance
        summary = dict(passed=True, checks=node.checks, events=node.events,
            elapsed_s=time.monotonic() - started, pose_samples=node.samples,
            discarded_integration_s=node.discarded, max_plant_tick_s=node.max_dt,
            min_observed_clearance_after_commit_m=node.min_clearance,
            scope=manifest['scope'])
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(dict(passed=True, checks=len(node.checks), output=str(output))), flush=True)
    except BaseException as exc:
        (output / 'failure.json').write_text(json.dumps(dict(type=type(exc).__name__, error=str(exc),
            checks=node.checks if node else [], events=node.events if node else []), indent=2) + '\n')
        raise
    finally:
        if node is not None:
            node.log.close()
            node.resources.close()
            node.destroy_node()
        children.close()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('scenario', 'plant', 'controller'), default='scenario')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--transport', choices=('UDPv4', 'DEFAULT'), default='UDPv4',
        help='simulation-local Fast DDS profile; UDPv4 avoids shared-memory transport')
    args = parser.parse_args()
    if args.role == 'scenario':
        if args.output is None:
            parser.error('--output required for scenario')
        args.output.mkdir(parents=True, exist_ok=False)
        os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = args.transport
        run_scenario(args.output.resolve())
    else:
        rclpy.init()
        node = Plant() if args.role == 'plant' else Controller()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
