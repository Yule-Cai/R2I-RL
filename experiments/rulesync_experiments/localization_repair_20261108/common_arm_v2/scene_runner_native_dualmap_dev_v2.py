"""Native Nav2 baseline runner for the high-fidelity HSSD scenes (DEV).

Baseline arms NATIVE_ASYNC / NATIVE_WAIT / NATIVE_PERIODIC.  The runner starts
the same Gazebo world, robot, AMCL and upstream Nav2 stack that the frozen
RuleSync runner uses (planner_server, controller_server, collision_monitor and
the nav2_costmap_2d Keepout Filter), but it deliberately starts *no* RuleSync
component: no manager, no nav2 bridge, no topopatch gate, no RuleGuard and no
authorization/vote path.  The rule snapshot of a case JSON is expressed
directly as the Nav2 keepout mask; nothing else interprets the rule.

Modes (``RULESYNC_NATIVE_MODE`` env or ``--mode``):
  async     apply the replacement mask immediately at the trigger; leave the
            original FollowPath action running; Nav2 defaults do whatever they
            do.
  wait      apply the replacement mask immediately at the trigger; wait until
            both costmaps report an update stamp at/after the mask stamp (the
            same freshness + probe barrier the frozen runner uses, but WITHOUT
            any proactive HOLD), then send a fresh plan/goal.
  periodic  apply the replacement mask immediately at the trigger and replace
            the active FollowPath goal with a fresh plan every 2.0 s
            (simulation time).

Recording: per-trial ``result.json`` with PASS/FAIL, mission success, body
clearances to both rule polygons, body-entry flags, mask publish timestamps,
costmap update timestamps, wall/sim durations, plus the raw event/pose logs.
The evaluation body envelope, world pose capture and robot instrumentation are
the same as the frozen runner (``gazebo_world_pose.instrument_robot``), which is
evaluation-only and never feeds the controller.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import random
from pathlib import Path
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RosTime
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import GetCostmap, ManageLifecycleNodes
from nav2_msgs.action import ComputePathToPose, FollowPath
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from gazebo_world_pose import instrument_robot

MODES = ('async', 'wait', 'periodic')
PERIODIC_REPLAN_S = 2.0
DEFAULT_MASK_DILATION_M = .40
# Evaluation-only robot body envelope used by the frozen RuleSync runner.
BODY_ENVELOPE = [[-.21, -.17], [.11, -.17], [.11, .17], [-.21, .17]]
ARRIVAL_WORLD_ERROR_M = .20
TRIGGER_WALL_TIMEOUT_S = 120.
MAX_TOTAL_WALL_S = 1080.


def write(path, obj):
    path.write_text(json.dumps(obj, indent=2) + '\n')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


# ---------------------------------------------------------------- geometry

def point_in_polygon(point, polygon):
    x, y = float(point[0]), float(point[1])
    inside, n = False, len(polygon)
    for i in range(n):
        x0, y0 = polygon[i][0], polygon[i][1]
        x1, y1 = polygon[(i + 1) % n][0], polygon[(i + 1) % n][1]
        if ((y0 > y) != (y1 > y)) and (x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0):
            inside = not inside
    return inside


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    o1, o2 = _orient(a, b, c), _orient(a, b, d)
    o3, o4 = _orient(c, d, a), _orient(c, d, b)
    if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
        return True
    epsilon = 1e-12

    def on_segment(p, q, r):
        return (min(p[0], q[0]) - epsilon <= r[0] <= max(p[0], q[0]) + epsilon and
                min(p[1], q[1]) - epsilon <= r[1] <= max(p[1], q[1]) + epsilon)

    if abs(o1) <= epsilon and on_segment(a, b, c):
        return True
    if abs(o2) <= epsilon and on_segment(a, b, d):
        return True
    if abs(o3) <= epsilon and on_segment(c, d, a):
        return True
    if abs(o4) <= epsilon and on_segment(c, d, b):
        return True
    return False


def point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx == 0. and dy == 0.:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (dx * dx + dy * dy)
    t = max(0., min(1., t))
    return math.hypot(point[0] - (a[0] + t * dx), point[1] - (a[1] + t * dy))


def segment_segment_distance(a, b, c, d):
    if segments_intersect(a, b, c, d):
        return 0.
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def polygon_intersects(polygon_a, polygon_b):
    na, nb = len(polygon_a), len(polygon_b)
    for i in range(na):
        for j in range(nb):
            if segments_intersect(polygon_a[i], polygon_a[(i + 1) % na],
                                  polygon_b[j], polygon_b[(j + 1) % nb]):
                return True
    return (point_in_polygon(polygon_a[0], polygon_b) or
            point_in_polygon(polygon_b[0], polygon_a))


def polygon_distance(polygon_a, polygon_b):
    """Exact for simple polygons evaluated as disjoint edge sets; 0 on overlap."""
    if polygon_intersects(polygon_a, polygon_b):
        return 0.
    na, nb = len(polygon_a), len(polygon_b)
    return min(segment_segment_distance(polygon_a[i], polygon_a[(i + 1) % na],
                                        polygon_b[j], polygon_b[(j + 1) % nb])
               for i in range(na) for j in range(nb))


def body_polygon(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [[x + c * px - s * py, y + s * px + c * py] for px, py in BODY_ENVELOPE]


def path_polygon_distance(path_poses, polygon):
    points = [[p.pose.position.x, p.pose.position.y] for p in path_poses]
    return min(polygon_distance([point, point, point, point], polygon) for point in points)


def mask_arrays(rule, map_record, dilation_m):
    """Rasterise the rule polygon at map resolution, then dilate by dilation_m.

    Same mask construction as the frozen RuleSync costmap-mask adapter
    (benchmark_nav2_combination.Probe.flush_masks): the raw polygon is inflated
    by 0.40 m so both families hand identical keepout geometry to the costmap.
    """
    width, height = int(map_record['width']), int(map_record['height'])
    resolution = float(map_record['resolution'])
    ox, oy = float(map_record['origin'][0]), float(map_record['origin'][1])
    source = np.zeros((height, width), dtype=bool)
    xs = [p[0] for p in rule]
    ys = [p[1] for p in rule]
    i0 = max(0, math.floor((min(xs) - ox) / resolution) - 1)
    i1 = min(width, math.ceil((max(xs) - ox) / resolution) + 2)
    j0 = max(0, math.floor((min(ys) - oy) / resolution) - 1)
    j1 = min(height, math.ceil((max(ys) - oy) / resolution) + 2)
    for j in range(j0, j1):
        y = oy + (j + .5) * resolution
        for i in range(i0, i1):
            if point_in_polygon((ox + (i + .5) * resolution, y), rule):
                source[j, i] = True
    inflated = np.zeros_like(source)
    cells = math.ceil(dilation_m / resolution)
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if math.hypot(dx, dy) * resolution > dilation_m + 1e-8:
                continue
            a, b = max(0, dy), min(height, height + dy)
            c, d = max(0, dx), min(width, width + dx)
            if b <= a or d <= c:
                continue
            inflated[a:b, c:d] |= source[a - dy:b - dy, c - dx:d - dx]
    return source, inflated


# ---------------------------------------------------------------- config

def parameters():
    """Upstream Nav2 configuration of the frozen RuleSync runner.

    Copied unchanged from the frozen ``nav2_combination_pilot.parameters()``
    (runtime image localhost/rulesync-channel-dev:20261104-trace1).  The
    RuleSync runner then edits only the AMCL block, goal tolerance, speed and
    keepout filter list; this runner applies exactly the same edits so the
    physical navigation stack is shared.
    """
    controller = dict(use_sim_time=True, controller_frequency=20.,
        enable_stamped_cmd_vel=True, costmap_update_timeout=1.,
        controller_plugins=['FollowPath'], goal_checker_plugins=['GoalChecker'],
        progress_checker_plugins=['ProgressChecker'], path_handler_plugins=['PathHandler'],
        failure_tolerance=.3,
        FollowPath=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            desired_linear_vel=.20, lookahead_dist=.5, use_velocity_scaled_lookahead_dist=False,
            use_rotate_to_heading=True, rotate_to_heading_angular_vel=.5,
            use_collision_detection=True, max_allowed_time_to_collision_up_to_carrot=1.),
        GoalChecker=dict(plugin='nav2_controller::SimpleGoalChecker', xy_goal_tolerance=.15,
            yaw_goal_tolerance=.25, stateful=True),
        ProgressChecker=dict(plugin='nav2_controller::SimpleProgressChecker',
            required_movement_radius=.1, movement_time_allowance=15.),
        PathHandler=dict(plugin='nav2_controller::FeasiblePathHandler', prune_distance=2.))
    result = {'controller_server': {'ros__parameters': controller}}
    result['planner_server'] = {'ros__parameters': dict(use_sim_time=True,
        planner_plugins=['GridBased'], expected_planner_frequency=1.,
        GridBased=dict(plugin='nav2_navfn_planner::NavfnPlanner', tolerance=.1,
            use_astar=True, allow_unknown=False))}
    for scope in ['global_costmap', 'local_costmap']:
        p = dict(use_sim_time=True, global_frame='map', robot_base_frame='base_footprint',
            update_frequency=10., publish_frequency=5., resolution=.05,
            robot_radius=.30, transform_tolerance=.3, always_send_full_costmap=True,
            plugins=['static_layer', 'obstacle_layer', 'inflation_layer'], filters=['keepout_filter'],
            static_layer=dict(plugin='nav2_costmap_2d::StaticLayer', map_subscribe_transient_local=True),
            obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer', observation_sources='scan',
                scan=dict(topic='/scan', data_type='LaserScan', clearing=True, marking=True,
                    raytrace_max_range=3., obstacle_max_range=2.5)),
            inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',
                inflation_radius=.5, cost_scaling_factor=3.),
            keepout_filter=dict(plugin='nav2_costmap_2d::KeepoutFilter', enabled=True,
                filter_info_topic='/pilot/filter_info', override_lethal_cost=False))
        if scope == 'local_costmap':
            p.update(rolling_window=True, width=6, height=6)
        result[scope] = {scope: {'ros__parameters': p}}
    result['collision_monitor'] = {'ros__parameters': dict(use_sim_time=True,
        enabled=True, enable_stamped_cmd_vel=True, base_frame_id='base_footprint',
        odom_frame_id='odom', cmd_vel_in_topic='/cmd_vel_nav', cmd_vel_out_topic='/cmd_vel',
        transform_tolerance=.3, source_timeout=.5, stop_pub_timeout=2.,
        polygons=['PolygonStop'],
        PolygonStop=dict(type='polygon', points='[[0.35,0.30],[0.35,-0.30],[-0.30,-0.30],[-0.30,0.30]]',
            action_type='stop', min_points=4, enabled=True, visualize=False),
        observation_sources=['scan'], scan=dict(type='scan', topic='/scan', enabled=True))}
    result['lifecycle_manager_pilot'] = {'ros__parameters': dict(use_sim_time=True,
        autostart=True, bond_timeout=4.,
        node_names=['planner_server', 'controller_server', 'collision_monitor'])}
    return result


def upstream_versions():
    """Provenance for the upstream components actually started."""
    out = {}
    distro = os.environ.get('ROS_DISTRO', 'lyrical')
    for package in ('nav2_planner', 'nav2_controller', 'nav2_collision_monitor',
                    'nav2_costmap_2d', 'nav2_amcl', 'nav2_lifecycle_manager',
                    'nav2_msgs', 'nav2_navfn_planner',
                    'nav2_regulated_pure_pursuit_controller'):
        path = Path('/opt/ros') / distro / 'share' / package / 'package.xml'
        try:
            out[package] = ET.parse(path).getroot().findtext('version')
        except Exception:
            out[package] = None
    out['ros_distro'] = distro
    return out


class Processes:
    def __init__(self, root, env):
        self.root, self.env, self.items = root, env, []

    def start(self, name, args, delay=0.):
        stream = (self.root / (name + '.log')).open('w')
        p = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                             env=self.env, start_new_session=True)
        self.items.append((name, p, stream))
        if delay:
            time.sleep(delay)

    def close(self):
        import signal
        for _, p, _ in reversed(self.items):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for _, p, stream in self.items:
            try:
                p.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    p.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait()
            stream.close()


# ---------------------------------------------------------------- ROS helpers

def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.01)


def wait(node, condition, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if condition():
            return
    raise TimeoutError('native condition timed out')


def call(node, future, seconds=35.):
    wait(node, future.done, seconds)
    return future.result()


def read_query(node, client, request, deadline_s=35.):
    """Bounded read-only retries; never repeat a mutation or action request."""
    deadline = time.monotonic() + deadline_s
    while time.monotonic() < deadline:
        pending = client.call_async(request)
        try:
            return call(node, pending, min(2., max(.1, deadline - time.monotonic())))
        except TimeoutError:
            client.remove_pending_request(pending)
            node.event('read_query_timeout', service=client.srv_name)
    raise TimeoutError('read query unavailable: ' + client.srv_name)


def cost_at(map_msg, x, y):
    p = map_msg.metadata
    i = math.floor((x - p.origin.position.x) / p.resolution)
    j = math.floor((y - p.origin.position.y) / p.resolution)
    if not (0 <= i < p.size_x and 0 <= j < p.size_y):
        raise ValueError('probe outside actual costmap')
    return int(map_msg.data[j * p.size_x + i])


class NativeProbe(Node):
    def __init__(self, root, map_record, case, mode, mask_dilation_m):
        super().__init__('native_nav2_baseline')
        self.root = root
        self.map_record = map_record
        self.mode = mode
        self.mask_dilation_m = mask_dilation_m
        self.spawn = [float(v) for v in case['spawn']]
        self.goal = [float(v) for v in case['goal']]
        self.initial_rule = [[float(p[0]), float(p[1])] for p in case['initial_rule']]
        self.replacement_rule = [[float(p[0]), float(p[1])] for p in case['replacement_rule']]
        self.trigger_distance = float(case['trigger']['distance_m'])
        self.task_probes = case['costmap_probes']
        self.events = (root / 'events.jsonl').open('w')
        self.world = (root / 'world_pose.jsonl').open('w')
        self.odom_log = (root / 'localization.jsonl').open('w')
        self.last_world = None
        self.world_errors = []
        self.scan_count = 0
        self.latest_scan = None
        self.odom_raw = None
        self.odom = None
        self.use_map_pose = False
        self.map_pose_ready = False
        self.last_pose_at = 0.
        self.nonzero = {'/cmd_vel_nav': 0, '/cmd_vel': 0}
        self.active = False
        self.mask_generation = None
        self.mask_events = []
        self.costmap_updates = {}
        self.plans = []
        self.plan_count = 0
        self.triggered = False
        self.trigger_displacement_m = None
        self.trigger_sim_ns = None
        self.trigger_wall = None
        self.watcher = None
        self.current_handle = None
        self.current_result = None
        self.next_periodic_sim = None
        self.clearance = {
            name: dict(min_all=math.inf, min_pre=math.inf, min_post=math.inf,
                       entered_all=False, entered_pre=False, entered_post=False)
            for name in ('initial_rule', 'replacement_rule')}
        self.clearance_samples = 0
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', self.qos)
        self.localization_map_pub = self.create_publisher(OccupancyGrid, '/localization_map', self.qos)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/pilot/mask', self.qos)
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/pilot/filter_info', self.qos)
        self.create_subscription(TFMessage, '/rulesync_test/world_pose', self.on_world, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 30)
        for topic in ('/cmd_vel_nav', '/cmd_vel'):
            self.create_subscription(TwistStamped, topic,
                                     lambda m, t=topic: self.on_cmd(t, m), 50)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.controller = ActionClient(self, FollowPath, '/follow_path')
        self.states = {name: self.create_client(GetState, '/' + name + '/get_state')
                       for name in ('planner_server', 'controller_server', 'collision_monitor')}
        self.costmaps = {name: self.create_client(GetCostmap, '/' + name + '/get_costmap')
                         for name in ('global_costmap', 'local_costmap')}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def event(self, kind, **data):
        self.events.write(json.dumps(dict(kind=kind, receipt_ns=time.monotonic_ns(),
                                          sim_ns=self.get_clock().now().nanoseconds, **data)) + '\n')
        self.events.flush()

    def on_world(self, msg):
        try:
            if len(msg.transforms) != 1:
                raise ValueError('one model transform required')
            t = msg.transforms[0]
            p, q = t.transform.translation, t.transform.rotation
            ns = stamp_ns(t.header.stamp)
            if t.header.frame_id != 'default' or t.child_frame_id != 'turtlebot3_waffle':
                raise ValueError('unexpected world/model frames: %r/%r'
                                 % (t.header.frame_id, t.child_frame_id))
            if ns <= 0 or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise ValueError('invalid model pose or simulation timestamp')
            if abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.) > 1e-3:
                raise ValueError('model pose quaternion is not normalized')
            if self.last_world and ns <= self.last_world['sim_ns']:
                raise ValueError('nonmonotonic world stamp')
            row = dict(sim_ns=ns, receipt_ns=time.monotonic_ns(), xyz=[p.x, p.y, p.z],
                       quaternion=[q.x, q.y, q.z, q.w], active=self.active,
                       parent=t.header.frame_id, child=t.child_frame_id)
            self.last_world = row
            self.world.write(json.dumps(row) + '\n')
            self.world.flush()
        except ValueError as exc:
            self.world_errors.append(str(exc))

    def on_scan(self, msg):
        self.scan_count += 1
        self.latest_scan = msg

    def on_cmd(self, topic, msg):
        if abs(msg.twist.linear.x) + abs(msg.twist.angular.z) > 1e-6:
            self.nonzero[topic] += 1

    def on_odom(self, msg):
        self.odom_raw = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if not self.use_map_pose:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', RosTime())
            if (self.get_clock().now().nanoseconds - stamp_ns(tf.header.stamp)) / 1e9 > .25:
                return
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))
        self.odom = [t.x, t.y]
        self.last_pose_at = time.monotonic()
        self.map_pose_ready = True
        self.sample_clearance(t.x, t.y, yaw)
        self.odom_log.write(json.dumps(dict(
            receipt_ns=time.monotonic_ns(), sim_ns=stamp_ns(tf.header.stamp),
            map_xy=self.odom, raw_odom_xy=self.odom_raw,
            quaternion=[q.x, q.y, q.z, q.w])) + '\n')

    def sample_clearance(self, x, y, yaw):
        body = body_polygon(x, y, yaw)
        post = self.triggered
        self.clearance_samples += 1
        for name, polygon in (('initial_rule', self.initial_rule),
                              ('replacement_rule', self.replacement_rule)):
            distance = polygon_distance(body, polygon)
            entered = polygon_intersects(body, polygon)
            stats = self.clearance[name]
            stats['min_all'] = min(stats['min_all'], distance)
            stats['entered_all'] = stats['entered_all'] or entered
            if post:
                stats['min_post'] = min(stats['min_post'], distance)
                stats['entered_post'] = stats['entered_post'] or entered
            else:
                stats['min_pre'] = min(stats['min_pre'], distance)
                stats['entered_pre'] = stats['entered_pre'] or entered

    def publish_map(self):
        m = self.map_record
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = m['resolution']
        grid.info.width = m['width']
        grid.info.height = m['height']
        grid.info.origin.position.x = m['origin'][0]
        grid.info.origin.position.y = m['origin'][1]
        grid.info.origin.orientation.w = 1.
        grid.info.map_load_time = grid.header.stamp
        grid.data = list(m['data'])
        self.map_pub.publish(grid)
        localization=OccupancyGrid()
        localization.header=grid.header
        localization.info=grid.info
        localization.data=list(m['_localization_map']['data'])
        self.localization_map_pub.publish(localization)

    def publish_mask(self, epoch):
        rule = self.initial_rule if epoch == 1 else self.replacement_rule
        source, inflated = mask_arrays(rule, self.map_record, self.mask_dilation_m)
        m = self.map_record
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = m['resolution']
        grid.info.width = m['width']
        grid.info.height = m['height']
        grid.info.origin.position.x = m['origin'][0]
        grid.info.origin.position.y = m['origin'][1]
        grid.info.origin.orientation.w = 1.
        grid.info.map_load_time = grid.header.stamp
        grid.data = (inflated.astype(np.int8) * 100).ravel().tolist()
        self.mask_pub.publish(grid)
        info = CostmapFilterInfo()
        info.header = grid.header
        info.type = 0
        info.filter_mask_topic = '/pilot/mask'
        info.base = 0.
        info.multiplier = 1.
        self.info_pub.publish(info)
        stamp = stamp_ns(grid.header.stamp)
        generation = dict(
            epoch=epoch,
            rule_source='initial_rule' if epoch == 1 else 'replacement_rule',
            stamp_ns=stamp, receipt_ns=time.monotonic_ns(), wall_ns=time.time_ns(),
            polygon=rule,
            polygon_sha256=hashlib.sha256(json.dumps(rule).encode()).hexdigest(),
            mask_sha256=hashlib.sha256(np.packbits(inflated).tobytes()).hexdigest(),
            dilation_m=self.mask_dilation_m,
            resolution=float(m['resolution']), width=int(m['width']), height=int(m['height']),
            origin=[float(m['origin'][0]), float(m['origin'][1])],
            raw_occupied_cells=int(source.sum()), mask_occupied_cells=int(inflated.sum()),
            filter_info_topic='/pilot/filter_info', mask_topic='/pilot/mask',
            filter_type=0, filter_base=0., filter_multiplier=1.)
        self.mask_generation = generation
        self.mask_events.append({k: v for k, v in generation.items() if k != 'polygon'})
        write(self.root / f'mask_epoch{epoch}.json', generation)
        # event() stamps receipt_ns/sim_ns itself; drop the duplicate receipt_ns.
        self.event('mask_published', **{k: v for k, v in generation.items()
                                        if k not in ('polygon', 'receipt_ns')})
        return generation

    def registration(self):
        """Stationary map/scan registration preflight, same as frozen runner."""
        m = self.map_record
        scan = self.latest_scan
        world = self.last_world
        q = world['quaternion']
        yaw = math.atan2(2. * (q[3] * q[2] + q[0] * q[1]), 1. - 2. * (q[1] ** 2 + q[2] ** 2))
        sx = world['xyz'][0] - .064 * math.cos(yaw)
        sy = world['xyz'][1] - .064 * math.sin(yaw)
        grid = np.asarray(m['data']).reshape(m['height'], m['width'])
        errors, endpoints = [], []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or not .10 < r < min(scan.range_max - .05, 15.):
                continue
            a = yaw + scan.angle_min + i * scan.angle_increment
            x, y = sx + r * math.cos(a), sy + r * math.sin(a)
            col = math.floor((x - m['origin'][0]) / m['resolution'])
            row = math.floor((y - m['origin'][1]) / m['resolution'])
            lo_x, hi_x = max(0, col - 20), min(m['width'], col + 21)
            lo_y, hi_y = max(0, row - 20), min(m['height'], row + 21)
            if hi_x <= lo_x or hi_y <= lo_y:
                error = 2.
            else:
                ys, xs = np.where(grid[lo_y:hi_y, lo_x:hi_x] == 100)
                error = float(np.min(np.hypot(
                    (xs + lo_x + .5) * m['resolution'] + m['origin'][0] - x,
                    (ys + lo_y + .5) * m['resolution'] + m['origin'][1] - y))) if len(xs) else 2.
            errors.append(error)
            endpoints.append([x, y, error])
        result = dict(
            finite_returns=len(errors),
            median_endpoint_map_distance_m=float(np.median(errors)) if errors else None,
            fraction_within_030m=float(np.mean(np.asarray(errors) <= .30)) if errors else 0.,
            world_xyz=world['xyz'], world_yaw=yaw, odom_xy=self.odom,
            thresholds=dict(min_returns=60, median_max_m=.25, min_fraction_within_030m=.70),
            scope='stationary one-pose registration diagnostic; not a localization-accuracy study',
            endpoints=endpoints)
        result['passed'] = bool(len(errors) >= 60 and np.median(errors) <= .25 and
                                result['fraction_within_030m'] >= .70)
        write(self.root / 'registration.json', result)
        write(self.root / 'stationary_scan.json', dict(
            angle_min=scan.angle_min, angle_increment=scan.angle_increment,
            range_min=scan.range_min, range_max=scan.range_max,
            ranges=[float(r) if math.isfinite(r) else None for r in scan.ranges]))
        return result


class CostmapWatcher:
    """Non-blocking observation of the costmap update stamps after a mask publish."""

    def __init__(self, node, epoch):
        self.node = node
        self.epoch = epoch
        self.stamp = node.mask_generation['stamp_ns']
        self.start_wall = time.monotonic()
        self.next_poll = 0.
        self.pending = {}
        self.pending_start = {}
        self.seen = {}
        self.done = False

    def poll(self):
        if self.done:
            return
        now = time.monotonic()
        if now < self.next_poll:
            return
        self.next_poll = now + .2
        for name, client in self.node.costmaps.items():
            if name in self.seen:
                continue
            future = self.pending.get(name)
            if future is not None:
                if future.done():
                    try:
                        map_msg = future.result().map
                    except Exception:
                        map_msg = None
                    self.pending.pop(name, None)
                    if map_msg is not None and stamp_ns(map_msg.metadata.update_time) >= self.stamp:
                        update_ns = stamp_ns(map_msg.metadata.update_time)
                        row = dict(update_time_ns=update_ns,
                                   receipt_ns=time.monotonic_ns(),
                                   mask_stamp_ns=self.stamp,
                                   latency_sim_s=(update_ns - self.stamp) / 1e9,
                                   latency_wall_s=now - self.start_wall)
                        self.seen[name] = row
                        self.node.event('costmap_mask_reflected', epoch=self.epoch,
                                        costmap=name,
                                        **{k: v for k, v in row.items() if k != 'receipt_ns'})
                    continue
                if now - self.pending_start.get(name, now) < 3.:
                    continue
                self.pending.pop(name, None)
            self.pending[name] = client.call_async(GetCostmap.Request())
            self.pending_start[name] = now
        if len(self.seen) == len(self.node.costmaps):
            self.done = True
            self.node.costmap_updates[self.epoch] = {
                name: {k: v for k, v in row.items() if k != 'receipt_ns'}
                for name, row in self.seen.items()}
            self.node.event('costmap_mask_reflected_all', epoch=self.epoch,
                            costmaps=sorted(self.seen))


def applied(node, epoch):
    """Mask-application barrier identical to the frozen runner's: both costmaps
    must carry an update stamp at/after the mask stamp AND the case probe cells
    must read keepout (254) inside / not-254 outside."""
    probes = node.task_probes[str(epoch)]
    inside, outside = probes['inside'], probes['outside']
    deadline = time.monotonic() + 30.
    for client in node.costmaps.values():
        if not client.wait_for_service(timeout_sec=15.):
            raise TimeoutError('costmap unavailable')
    values = {}
    while time.monotonic() < deadline:
        maps = {name: read_query(node, client, GetCostmap.Request()).map
                for name, client in node.costmaps.items()}
        values = {name: [cost_at(m, *inside), cost_at(m, *outside)]
                  for name, m in maps.items()}
        fresh = node.mask_generation is not None and all(
            stamp_ns(m.metadata.update_time) >= node.mask_generation['stamp_ns']
            for m in maps.values())
        if fresh and all(v[0] == 254 and v[1] != 254 for v in values.values()):
            for name, m in maps.items():
                p = m.metadata
                update_ns = stamp_ns(p.update_time)
                write(node.root / f'costmap_epoch{epoch}_{name}.json', dict(
                    epoch=epoch, frame=m.header.frame_id, resolution=p.resolution,
                    width=p.size_x, height=p.size_y,
                    origin=[p.origin.position.x, p.origin.position.y],
                    update_time_ns=update_ns, data=list(m.data)))
                node.costmap_updates.setdefault(epoch, {})[name] = dict(
                    update_time_ns=update_ns,
                    mask_stamp_ns=node.mask_generation['stamp_ns'],
                    latency_sim_s=(update_ns - node.mask_generation['stamp_ns']) / 1e9,
                    latency_wall_s=None)
            node.event('corresponding_costmaps_applied', epoch=epoch, values=values,
                       mask=node.mask_generation['stamp_ns'])
            return values
        spin_for(node, .05)
    raise TimeoutError('corresponding mask absent from actual costmaps: ' + str(values))


def start_navigation(node):
    """Explicit lifecycle startup after discovery, once only."""
    manager = node.create_client(ManageLifecycleNodes, '/lifecycle_manager_pilot/manage_nodes')
    if not manager.wait_for_service(timeout_sec=30.):
        raise TimeoutError('lifecycle manager service unavailable')
    for name, client in node.states.items():
        if not client.wait_for_service(timeout_sec=30.):
            raise TimeoutError(name + ' state missing')
        state = int(read_query(node, client, GetState.Request()).current_state.id)
        assert state == 1, (name, 'expected fresh unconfigured node', state)
    spin_for(node, 3.)
    request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
    response = call(node, manager.call_async(request))
    assert response.success, 'lifecycle startup refused'
    deadline = time.monotonic() + 50.
    while True:
        states = {name: int(read_query(node, client, GetState.Request()).current_state.id)
                  for name, client in node.states.items()}
        if all(s == 3 for s in states.values()):
            return states
        if time.monotonic() > deadline:
            raise TimeoutError('lifecycle not active: ' + str(states))
        spin_for(node, .1)


def plan_path(node, epoch, label):
    if not node.planner.wait_for_server(timeout_sec=10.):
        raise TimeoutError('planner missing')
    request = ComputePathToPose.Goal()
    request.planner_id = 'GridBased'
    request.goal.header.frame_id = 'map'
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x = node.goal[0]
    request.goal.pose.position.y = node.goal[1]
    request.goal.pose.orientation.w = 1.
    handle = call(node, node.planner.send_goal_async(request))
    if not handle.accepted:
        raise RuntimeError('planner goal rejected')
    result = call(node, handle.get_result_async())
    if int(result.status) != 4 or int(result.result.error_code) != 0:
        raise RuntimeError('planner failed: %s' % result.result.error_msg)
    path = result.result.path
    if not path.poses:
        raise RuntimeError('empty planner path')
    node.plan_count += 1
    points = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    row = dict(epoch=epoch, label=label, sim_ns=node.get_clock().now().nanoseconds,
               receipt_ns=time.monotonic_ns(), points=points, length_m=length)
    write(node.root / f'plan_{node.plan_count:03d}_{label}.json', row)
    node.plans.append(dict(epoch=epoch, label=label, points=len(points), length_m=length,
                           receipt_ns=row['receipt_ns']))
    node.event('plan', epoch=epoch, label=label, points=len(points), length_m=length)
    return path


def follow_path(node, path, epoch, label):
    if not node.controller.wait_for_server(timeout_sec=10.):
        raise TimeoutError('controller missing')
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = 'FollowPath'
    goal.goal_checker_id = 'GoalChecker'
    goal.progress_checker_id = 'ProgressChecker'
    goal.path_handler_id = 'PathHandler'
    handle = call(node, node.controller.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError('follow goal rejected')
    node.current_handle = handle
    node.current_result = handle.get_result_async()
    node.event('follow_goal_sent', epoch=epoch, label=label, points=len(path.poses))
    return handle


def cancel_current(node, reason):
    handle, result = node.current_handle, node.current_result
    if handle is None or result is None:
        return None
    if not result.done():
        try:
            cancel_future = handle.cancel_goal_async()
        except Exception:
            cancel_future = None
        if cancel_future is not None:
            try:
                call(node, cancel_future, 10.)
            except TimeoutError:
                node.event('cancel_timeout', reason=reason)
        end = time.monotonic() + 10.
        while not result.done() and time.monotonic() < end:
            spin_for(node, .02)
    if not result.done():
        raise RuntimeError('previous follow goal did not terminate: ' + reason)
    status = int(result.result().status)
    node.event('follow_goal_terminated', reason=reason, status=status)
    return status


def fire_trigger(node, mode):
    node.triggered = True
    node.trigger_sim_ns = node.get_clock().now().nanoseconds
    node.trigger_wall = time.monotonic()
    node.trigger_displacement_m = math.dist(node.odom, node.spawn)
    node.event('trigger', displacement_m=node.trigger_displacement_m,
               trigger_distance_m=node.trigger_distance)
    node.publish_mask(2)
    if mode == 'wait':
        applied(node, 2)
        cancel_current(node, 'wait-mode-new-goal')
        path = plan_path(node, 2, 'recovery')
        follow_path(node, path, 2, 'recovery')
        node.event('wait_mode_new_goal_sent',
                   mask_stamp_ns=node.mask_generation['stamp_ns'],
                   costmap_updates=node.costmap_updates.get(2, {}))
    else:
        node.watcher = CostmapWatcher(node, 2)


def periodic_replan(node):
    if node.current_result is not None and node.current_result.done():
        return
    epoch = 2 if node.triggered else 1
    try:
        path = plan_path(node, epoch, 'periodic')
    except Exception as exc:  # a failed periodic plan must not kill the trial
        node.event('periodic_plan_failed', error=repr(exc))
        return
    if node.current_result.done():
        return
    status = cancel_current(node, 'periodic-replan')
    if status == 4:
        node.event('periodic_replan_skipped_arrived')
        return
    follow_path(node, path, epoch, 'periodic')
    node.event('periodic_replan_sent', epoch=epoch)


def replacement_invalidates_initial_path(path, replacement_rule):
    distance = path_polygon_distance(path.poses, replacement_rule)
    return distance <= .30, distance


def evaluate(node, result, case):
    final_status = None
    final_error_code = None
    final_error_msg = None
    if node is not None and node.current_result is not None and node.current_result.done():
        try:
            final = node.current_result.result()
            final_status = int(final.status)
            final_error_code = int(final.result.error_code)
            final_error_msg = str(final.result.error_msg)
        except Exception as exc:
            final_error_msg = repr(exc)
    world_error = None
    if node is not None and node.last_world is not None:
        world_error = math.dist(node.last_world['xyz'][:2], node.goal)
    map_error = None
    if node is not None and node.odom is not None:
        map_error = math.dist(node.odom, node.goal)
    action_ok = final_status == 4
    world_ok = world_error is not None and world_error <= ARRIVAL_WORLD_ERROR_M
    world_errors = list(node.world_errors) if node is not None else []
    mission = bool(action_ok and world_ok and not world_errors)
    result.update(
        mission_success=mission,
        arrival_definition=dict(
            final_action_status_success=action_ok,
            independent_world_goal_error_m=world_error,
            world_goal_error_max_m=ARRIVAL_WORLD_ERROR_M,
            world_capture_errors=world_errors),
        final_action_status=final_status,
        final_action_error_code=final_error_code,
        final_action_error_message=final_error_msg,
        independent_goal_error_m=world_error,
        map_frame_goal_error_m=map_error,
        last_world_xyz=node.last_world['xyz'] if node is not None and node.last_world else None,
        last_map_xy=node.odom if node is not None else None)
    if mission:
        result['result'] = 'PASS'
        result['phase'] = 'arrived'
    else:
        result['result'] = 'FAIL'
        if action_ok and not world_ok:
            reason = 'goal error exceeds arrival threshold: %s' % world_error
        elif world_errors:
            reason = 'world capture errors: %s' % world_errors[:3]
        elif final_status is not None:
            reason = 'final FollowPath action terminal status %s (%s)' % (
                final_status, final_error_msg)
        else:
            reason = result.get('error', 'no terminal action result')
        if 'error' not in result:
            result['error'] = reason
    return result


def run(args):
    mode = args.mode or os.environ.get('RULESYNC_NATIVE_MODE') or 'async'
    assert mode in MODES, 'unknown native mode: ' + str(mode)
    mask_dilation = getattr(args, 'mask_dilation_m', None)
    if mask_dilation is None:
        mask_dilation = float(os.environ.get('RULESYNC_NATIVE_MASK_DILATION',
                                             DEFAULT_MASK_DILATION_M))
    assert .0 <= mask_dilation <= 1.
    assert 0 <= args.seed < 2 ** 31
    assert .05 < args.speed <= .30
    assert 0 <= args.ros_domain <= 232 and args.concurrency >= 1
    assert args.stage == 'recovery', 'native baseline arms are recovery-only'
    assert getattr(args, 'geometry_profile', 'standard') == 'standard'
    assert getattr(args, 'controller_profile', 'legacy') == 'legacy'
    assert getattr(args, 'cohort_id', None) is None
    random.seed(args.seed)
    np.random.seed(args.seed)
    t0 = time.monotonic()
    root = args.output
    root.mkdir(parents=True, exist_ok=False)
    # Only documentation camera save paths change in the run-local world copy;
    # the native arms never trigger a capture.
    tree = ET.parse(args.scene / (args.scene_id + '.sdf'))
    camera_paths = tree.findall('./world/model/link/sensor/camera/save/path')
    assert len(camera_paths) == 2
    (root / 'images').mkdir()
    for path in camera_paths:
        name = Path(path.text).name
        assert name in ('perspective', 'overview')
        assert path.text == '/experiment/current/images/' + name
        path.text = str(root / 'images' / name)
    step = tree.find('./world/physics/max_step_size')
    factor = tree.find('./world/physics/real_time_factor')
    assert step is not None and factor is not None
    assert float(step.text) == .003 and float(factor.text) == 1.
    if args.target_rtf != 1.:
        factor.text = str(args.target_rtf)
    ET.SubElement(tree.getroot().find('world'),'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
    tree.write(root / 'world.sdf', encoding='unicode', xml_declaration=True)
    map_record = json.loads((args.scene / 'map.json').read_text())
    map_record['_localization_map'] = json.loads((args.scene / 'localization_map.json').read_text())
    migration = json.loads((args.scene / 'migration_manifest.json').read_text())
    case = json.loads(args.case.read_text())
    assert case['schema_version'] == 1 and case['scene'] == args.scene_id
    assert case['map_sha256'] == sha(args.scene / 'map.json')
    assert case['trigger']['type'] == 'displacement_from_spawn'
    assert case['trigger']['distance_m'] > 0 and case['reference_length_m'] >= 6
    assert not case.get('planning_only'), 'planning-only inputs cannot release motion'
    spawn = [float(v) for v in case['spawn']]
    goal = [float(v) for v in case['goal']]
    if args.spawn is not None:
        assert math.dist(spawn, [float(v) for v in args.spawn]) < 1e-9
    if args.goal is not None:
        assert math.dist(goal, [float(v) for v in args.goal]) < 1e-9
    for polygon in (case['initial_rule'], case['replacement_rule']):
        assert len(polygon) >= 3
        assert all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in polygon)
    assert set(case['costmap_probes']) >= {'1', '2'}
    occupancy = np.asarray(map_record['data']).reshape(map_record['height'], map_record['width'])
    rows, cols = np.where(occupancy == 100)
    goal_clearance = float(np.min(np.hypot(
        (cols + .5) * map_record['resolution'] + map_record['origin'][0] - goal[0],
        (rows + .5) * map_record['resolution'] + map_record['origin'][1] - goal[1]))) \
        - math.sqrt(2) * map_record['resolution'] / 2
    assert goal_clearance >= .50, 'goal overlaps retained obstacle clearance: ' + str(goal_clearance)
    # Freeze inputs and record provenance.
    source_paths = [Path(__file__), args.scene / (args.scene_id + '.sdf'),
                    args.scene / 'map.json', args.scene / 'localization_map.json', args.scene / 'migration_manifest.json',
                    args.case, Path(instrument_robot.__code__.co_filename)]
    hashes = {}
    (root / 'source').mkdir()
    for index, path in enumerate(source_paths):
        dest = root / 'source' / ('%02d_%s' % (index, path.name))
        shutil.copy2(path, dest)
        hashes[str(path)] = sha(path)
    timing = case['timing']
    sim_budget = timing['sim_fixed_s'] + timing['distance_multiplier'] * \
        case['reference_length_m'] / args.speed
    wall_budget = timing['wall_fixed_s'] + sim_budget / timing['minimum_budget_rtf']
    assert 0 < sim_budget < 4000 and 0 < wall_budget < 11000
    params = parameters()
    params['lifecycle_manager_pilot']['ros__parameters']['autostart'] = False
    params['amcl'] = {'ros__parameters': dict(
        use_sim_time=True, base_frame_id='base_footprint', global_frame_id='map',
        odom_frame_id='odom', scan_topic='scan', map_topic='/localization_map', tf_broadcast=True,
        transform_tolerance=1., robot_model_type='nav2_amcl::DifferentialMotionModel',
        laser_model_type='likelihood_field', max_beams=60, min_particles=500,
        max_particles=2000, alpha1=.2, alpha2=.2, alpha3=.2, alpha4=.2, alpha5=.2,
        sigma_hit=.2, z_hit=.5, z_rand=.5, update_min_d=.1, update_min_a=.1,
        set_initial_pose=True, random_seed=args.seed,
        initial_pose=dict(x=spawn[0], y=spawn[1], z=0., yaw=0.))}
    params['amcl']['ros__parameters'].update(max_beams=120,sigma_hit=.05,z_hit=.9,z_rand=.1,
        alpha1=.02,alpha2=.02,alpha3=.02,alpha4=.02,alpha5=.02,
        update_min_d=.05,update_min_a=.05)
    params['lifecycle_manager_pilot']['ros__parameters']['node_names'].insert(0, 'amcl')
    params['controller_server']['ros__parameters']['GoalChecker']['xy_goal_tolerance'] = .10
    params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = args.speed
    for scope in ('global_costmap', 'local_costmap'):
        p = params[scope][scope]['ros__parameters']
        p['filters'] = ['keepout_filter', 'filter_inflation']
        p['filter_inflation'] = dict(plugin='nav2_costmap_2d::InflationLayer',
                                     inflation_radius=.8, cost_scaling_factor=3.)
    (root / 'params.yaml').write_text(yaml.safe_dump(params))
    parameters_sha = sha(root / 'params.yaml')
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.ros_domain),
               GZ_PARTITION='rulesync_native_%s_%s' % (root.parent.name, root.name),
               LP_NUM_THREADS='2', OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1')
    if args.render_mode == 'gpu':
        env.pop('LIBGL_ALWAYS_SOFTWARE', None)
        env.update(MESA_LOADER_DRIVER_OVERRIDE='iris', EGL_PLATFORM='surfaceless')
        os.environ.pop('LIBGL_ALWAYS_SOFTWARE', None)
        probe = subprocess.run(['eglinfo', '-B', '-p', 'surfaceless'], env=env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
        (root / 'gpu_renderer.log').write_text(probe.stdout)
        assert probe.returncode == 0 and 'Intel' in probe.stdout and 'Iris' in probe.stdout \
            and 'llvmpipe' not in probe.stdout, 'Hardware Intel rendering must be verified'
    else:
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    env.pop('ROS_DISCOVERY_SERVER', None)
    sim = Path('/opt/ros/lyrical/share/nav2_minimal_tb3_sim')
    env['GZ_SIM_RESOURCE_PATH'] = '/assets/' + args.scene_id + '/models:' + \
        str(sim / 'models') + ':' + str(sim.parent)
    os.environ.update(env)
    manifest = dict(
        runner='scene_runner_native_dualmap_dev_v2.py',pilot=True,formal_denominator=False,cohort_id=None, runner_sha256=sha(Path(__file__)),
        arm='NATIVE_' + mode.upper(), mode=mode,
        scope='native upstream-Nav2 baseline (NATIVE_ASYNC/WAIT/PERIODIC) on a high-fidelity HSSD scene',
        rulesync_components_started=[],
        rulesync_components_note=('no RuleSync manager, nav2 bridge, gate, guard or '
                                  'authorization/vote component is started in this process tree'),
        upstream_components=['nav2_planner/planner_server', 'nav2_controller/controller_server',
                             'nav2_collision_monitor/collision_monitor',
                             'nav2_costmap_2d::KeepoutFilter (inside global/local costmaps)',
                             'nav2_costmap_2d::StaticLayer/ObstacleLayer/InflationLayer',
                             'nav2_lifecycle_manager/lifecycle_manager',
                             'nav2_amcl/amcl', 'nav2_navfn_planner::NavfnPlanner',
                             'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'],
        upstream_versions=upstream_versions(),
        stage=args.stage, scene_id=args.scene_id,
        world=migration.get('world_provenance_label', 'HSSD ' + args.scene_id),
        source_sha256=hashes, parameters_sha256=parameters_sha,
        world_sha256=sha(args.scene / (args.scene_id + '.sdf')),
        map_sha256=sha(args.scene / 'map.json'),
        migration_manifest_sha256=sha(args.scene / 'migration_manifest.json'),
        task_id=case['id'], task_source_sha256=sha(args.case),
        spawn=spawn, goal=goal, initial_rule=case['initial_rule'],
        replacement_rule=case['replacement_rule'],
        trigger=case['trigger'], costmap_probes=case['costmap_probes'],
        requested_speed_mps=args.speed, seed=args.seed,
        controller_profile='legacy', geometry_profile='standard',
        keepout_mask=dict(dilation_m=mask_dilation, resolution=map_record['resolution'],
                          width=map_record['width'], height=map_record['height'],
                          origin=map_record['origin'][:2],
                          filter_info_topic='/pilot/filter_info', mask_topic='/pilot/mask',
                          filter_type=0, filter_base=0., filter_multiplier=1.,
                          rasterization='rule polygon rasterised at map resolution and dilated by dilation_m; '
                                        'same construction as the frozen RuleSync costmap-mask adapter'),
        body_envelope_xy=BODY_ENVELOPE, max_world_sim_gap_s=.10,
        arrival_world_error_m=ARRIVAL_WORLD_ERROR_M,
        max_follow_wall_s=wall_budget, max_follow_sim_s=sim_budget,
        periodic_replan_s=PERIODIC_REPLAN_S if mode == 'periodic' else None,
        ros_domain_id=args.ros_domain, gazebo_partition=env['GZ_PARTITION'],
        render_mode=args.render_mode, gpu_device='/dev/dri/renderD128' if args.render_mode == 'gpu' else None,
        target_real_time_factor=args.target_rtf, physics_step_size_s=float(step.text),
        declared_concurrent_jobs=args.concurrency,
        runtime_world_sha256=sha(root / 'world.sdf'),
        runtime_world_edits=['documentation camera save paths (capture never triggered)','contact observation plugin; physics unchanged'] +
                            (['target real-time factor only; fixed physics step'] if args.target_rtf != 1. else []),
        seed_consumers=['Gazebo --seed (including simulated sensor noise)', 'AMCL random_seed',
                        'Python random', 'NumPy random'],
        task_goal_known_map_clearance_m=goal_clearance, task_goal_min_clearance_m=.50,
        localization='AMCL map-to-odom; body clearance sampled from map TF; ECM pose evaluation only',
        device='one handheld; no physical robot or second edge device',
        scenario='geometry-qualified long-route replacement DEV, upstream-Nav2 baseline arm',
        documentation_capture='disabled for native baseline arms',
        runner_sha256_duplicate=sha(Path(__file__)))
    write(root / 'manifest.json', manifest)
    procs = Processes(root, env)
    node = None
    result = dict(result='FAIL', arm='NATIVE_' + mode.upper(), mode=mode, pilot=True,
                  formal_denominator=False, cohort_id=None, stage=args.stage, phase='startup',
                  scene=args.scene_id, task_id=case['id'], seed=args.seed,
                  requested_speed_mps=args.speed, spawn=spawn, goal=goal)
    sim0 = None
    invalidated_distance = None
    try:
        robot = instrument_robot(sim / 'urdf/gz_waffle.sdf.xacro', root, env)
        contact_tree=ET.parse(robot)
        for contact_link in ['base_link','camera_link','caster_back_left_link','caster_back_right_link']:
            link=contact_tree.getroot().find("model/link[@name='"+contact_link+"']")
            assert link is not None and link.findall('collision')
            sensor=ET.SubElement(link,'sensor',name='repair_contact_'+contact_link,type='contact')
            ET.SubElement(sensor,'always_on').text='true'
            ET.SubElement(sensor,'update_rate').text='20'
            ET.SubElement(sensor,'topic').text='/rulesync_dev/contact/'+contact_link
            contact=ET.SubElement(sensor,'contact')
            for collision in link.findall('collision'):
                ET.SubElement(contact,'collision').text=collision.get('name')
        contact_tree.write(robot,encoding='unicode',xml_declaration=True)
        procs.start('gazebo', ['gz', 'sim', '-r', '-s', '--headless-rendering',
                               '--seed', str(args.seed), str(root / 'world.sdf')], 5.)
        procs.start('spawn', ['ros2', 'launch', 'nav2_minimal_tb3_sim', 'spawn_tb3.launch.py',
                              'robot_name:=turtlebot3_waffle', f'robot_sdf:={robot}',
                              f'x_pose:={spawn[0]}', f'y_pose:={spawn[1]}', 'z_pose:=0.06',
                              'yaw:=0'], 1.)
        procs.start('world_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                     '/rulesync_test/world_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        procs.start('robot_state', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                                    '--ros-args', '-p', 'use_sim_time:=true', '-p',
                                    'robot_description:=' + (sim / 'urdf/turtlebot3_waffle.urdf').read_text()])
        for contact_link in ['base_link','camera_link','caster_back_left_link','caster_back_right_link']:
            procs.start('contact_'+contact_link,['gz','topic','-e','--json-output','-t','/world/default/model/turtlebot3_waffle/link/'+contact_link+'/sensor/repair_contact_'+contact_link+'/contact'])
        rclpy.init()
        node = NativeProbe(root, map_record, case, mode, mask_dilation)
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        sim0 = node.get_clock().now().nanoseconds
        wait(node, lambda: node.last_world is not None and node.scan_count >= 5 and
             node.odom_raw is not None, 120.)
        spin_for(node, 1.)
        result['phase'] = 'registration'
        registration = node.registration()
        assert math.dist(node.last_world['xyz'][:2], spawn) < .05, 'spawn drift or collision'
        assert registration['passed'], 'original map registration needs correction'
        node.event('registration_checked', passed=True, scan_messages=node.scan_count)
        node.publish_map()
        procs.start('amcl', ['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args',
                             '--params-file', str(root / 'params.yaml')])
        for package, executable in (('nav2_planner', 'planner_server'),
                                    ('nav2_controller', 'controller_server'),
                                    ('nav2_collision_monitor', 'collision_monitor')):
            cmd = ['ros2', 'run', package, executable, '--ros-args',
                   '--params-file', str(root / 'params.yaml')]
            if executable == 'controller_server':
                cmd += ['-r', 'cmd_vel:=/cmd_vel_nav']
            procs.start(executable, cmd, .3)
        procs.start('lifecycle', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
                                  '--ros-args', '-r', '__node:=lifecycle_manager_pilot',
                                  '--params-file', str(root / 'params.yaml')])
        states = start_navigation(node)
        node.event('active_states', states=states)
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        assert len(publishers) == 1 and publishers[0].node_name == 'collision_monitor'
        node.use_map_pose = True
        node.odom = None
        wait(node, lambda: node.map_pose_ready and node.odom is not None, 30.)
        node.event('map_localization_ready', xy=node.odom)
        result['phase'] = 'initial_mask'
        node.publish_mask(1)
        applied(node, 1)
        result['phase'] = 'initial_follow'
        initial_path = plan_path(node, 1, 'initial')
        invalidated, invalidated_distance = replacement_invalidates_initial_path(
            initial_path, node.replacement_rule)
        node.event('replacement_invalidates_initial_path', invalidated=bool(invalidated),
                   min_distance_m=invalidated_distance)
        assert invalidated, 'actual initial Nav2 path avoids the replacement; case needs qualification'
        follow_path(node, initial_path, 1, 'initial')
        node.active = True
        node.motion_wall_start = time.monotonic()
        node.motion_sim_start = node.get_clock().now().nanoseconds
        trigger_wall_deadline = node.motion_wall_start + TRIGGER_WALL_TIMEOUT_S
        node.next_periodic_sim = node.motion_sim_start + int(PERIODIC_REPLAN_S * 1e9)
        while True:
            spin_for(node, .02)
            if node.watcher is not None:
                node.watcher.poll()
            now_sim = node.get_clock().now().nanoseconds
            now_wall = time.monotonic()
            if node.current_result.done():
                break
            if not node.triggered:
                if node.odom is not None and \
                        math.dist(node.odom, node.spawn) >= node.trigger_distance:
                    result['phase'] = 'update'
                    fire_trigger(node, mode)
                    node.next_periodic_sim = now_sim + int(PERIODIC_REPLAN_S * 1e9)
                elif now_wall > trigger_wall_deadline:
                    raise TimeoutError('trigger not reached within %.0fs wall'
                                       % TRIGGER_WALL_TIMEOUT_S)
            if node.triggered:
                sim_elapsed = (now_sim - node.trigger_sim_ns) / 1e9
                wall_elapsed = now_wall - node.trigger_wall
                if sim_elapsed > sim_budget:
                    raise TimeoutError('post-trigger simulation budget %.1fs expired'
                                       % sim_budget)
                if wall_elapsed > wall_budget or now_wall - t0 > MAX_TOTAL_WALL_S:
                    raise TimeoutError('post-trigger wall-clock budget %.1fs expired'
                                       % wall_budget)
            if mode == 'periodic' and now_sim >= node.next_periodic_sim:
                node.next_periodic_sim += int(PERIODIC_REPLAN_S * 1e9)
                if node.triggered:
                    result['phase'] = 'recovery_follow'
                periodic_replan(node)
        node.active = False
        spin_for(node, .3)
        evaluate(node, result, case)
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = repr(exc)
        try:
            if node is not None:
                evaluate(node, result, case)
        except Exception as cleanup_exc:
            result['evaluation_error'] = repr(cleanup_exc)
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.active = False
                    spin_for(node, .1)
                    node.event('terminal', **{k: v for k, v in result.items()
                                              if k in ('result', 'phase', 'error', 'mission_success')})
                result.update(
                    wall_duration_s=time.monotonic() - t0,
                    sim_duration_s=(node.get_clock().now().nanoseconds - sim0) / 1e9
                    if sim0 is not None else None,
                    wall_motion_s=(time.monotonic() - node.motion_wall_start)
                    if getattr(node, 'motion_wall_start', None) is not None else None,
                    sim_motion_s=(node.get_clock().now().nanoseconds - node.motion_sim_start) / 1e9
                    if getattr(node, 'motion_sim_start', None) is not None else None,
                    wall_post_trigger_s=(time.monotonic() - node.trigger_wall)
                    if node.trigger_wall is not None else None,
                    sim_post_trigger_s=(node.get_clock().now().nanoseconds - node.trigger_sim_ns) / 1e9
                    if node.trigger_sim_ns is not None else None,
                    budgets=dict(sim_budget_s=sim_budget, wall_budget_s=wall_budget),
                    trigger=dict(type='displacement_from_spawn',
                                 distance_m=node.trigger_distance,
                                 fired=bool(node.triggered),
                                 displacement_m=node.trigger_displacement_m,
                                 sim_ns=node.trigger_sim_ns,
                                 latency_from_motion_start_s=(node.trigger_sim_ns - node.motion_sim_start) / 1e9
                                 if node.trigger_sim_ns is not None and
                                 getattr(node, 'motion_sim_start', None) is not None else None),
                    mask_events=node.mask_events,
                    costmap_updates=node.costmap_updates,
                    mask_reflection_done=bool(2 in node.costmap_updates),
                    mask_reflection_epoch2_costmaps=sorted(node.costmap_updates.get(2, {})),
                    plans=node.plans,
                    replans=int(sum(1 for p in node.plans if p['label'] == 'periodic')),
                    nonzero_commands=node.nonzero,
                    clearance_samples=node.clearance_samples,
                    min_clearance_to_initial_rule_m=_finite(node.clearance['initial_rule']['min_all']),
                    min_clearance_to_replacement_rule_m=_finite(node.clearance['replacement_rule']['min_all']),
                    min_clearance_post_trigger_initial_rule_m=_finite(node.clearance['initial_rule']['min_post']),
                    min_clearance_post_trigger_replacement_rule_m=_finite(node.clearance['replacement_rule']['min_post']),
                    body_entered_original_rule_polygon=node.clearance['initial_rule']['entered_all'],
                    body_entered_initial_rule_polygon=node.clearance['initial_rule']['entered_all'],
                    body_entered_replacement_rule_polygon=node.clearance['replacement_rule']['entered_all'],
                    body_entered_initial_rule_pre_trigger=node.clearance['initial_rule']['entered_pre'],
                    body_entered_initial_rule_post_trigger=node.clearance['initial_rule']['entered_post'],
                    body_entered_replacement_rule_pre_trigger=node.clearance['replacement_rule']['entered_pre'],
                    body_entered_replacement_rule_post_trigger=node.clearance['replacement_rule']['entered_post'],
                    clearance_phases={name: dict(
                        min_all_m=_finite(stats['min_all']),
                        min_pre_trigger_m=_finite(stats['min_pre']),
                        min_post_trigger_m=_finite(stats['min_post'])) for name, stats in node.clearance.items()},
                    initial_plan_min_distance_to_replacement_m=invalidated_distance,
                    initial_rule=case['initial_rule'],
                    replacement_rule=case['replacement_rule'],
                    case_sha256=sha(args.case),
                    map_sha256=sha(args.scene / 'map.json'),
                    world_sha256=sha(args.scene / (args.scene_id + '.sdf')),
                    keepout_mask=dict(dilation_m=mask_dilation,
                                      filter_info_topic='/pilot/filter_info',
                                      mask_topic='/pilot/mask', filter_type=0),
                    rulesync_components_started=[],
                    runner_sha256=sha(Path(__file__)),
                    parameters_sha256=parameters_sha,
                    world_errors=list(node.world_errors))
                node.events.close()
                node.world.close()
                node.odom_log.close()
                node.destroy_node()
                rclpy.try_shutdown()
            except Exception as exc:
                result['cleanup_error'] = repr(exc)
        procs.close()
        result['source_unchanged'] = all(
            hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest
            for p, digest in hashes.items())
        write(root / 'result.json', result)
        print(json.dumps(result), flush=True)
    return result


def _finite(value):
    return None if value is None or not math.isfinite(value) else float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--scene-id', required=True)
    parser.add_argument('--spawn', type=float, nargs=2, default=None)
    parser.add_argument('--goal', type=float, nargs=2, default=None)
    parser.add_argument('--speed', type=float, default=.20)
    parser.add_argument('--seed', type=int, default=52106)
    parser.add_argument('--ros-domain', type=int, default=231)
    parser.add_argument('--concurrency', type=int, default=1)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--geometry-profile', default='standard')
    parser.add_argument('--controller-profile', default='legacy')
    parser.add_argument('--cohort-id', default=None)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=['recovery'], default='recovery')
    parser.add_argument('--mode', choices=MODES, default=None)
    parser.add_argument('--mask-dilation-m', type=float, default=None)
    parser.add_argument('--render-mode', choices=['software', 'gpu'], default='gpu')
    parser.add_argument('--target-rtf', type=float, choices=[1., 2.], default=1.)
    args = parser.parse_args()
    raise SystemExit(0 if run(args)['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
