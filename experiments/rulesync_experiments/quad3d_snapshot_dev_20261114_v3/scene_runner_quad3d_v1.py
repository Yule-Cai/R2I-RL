"""B1 quadrotor 3D DEV runner on a prepared HSSD indoor scene.

What this is
------------
One closed-loop DEV trial in a prepared HSSD Gazebo scene (the same
``iotj_scene_build/scenes/<scene>`` prepared world and 5 cm collision
projection used by the frozen 2D runner) with a small velocity-level
quadrotor:

  takeoff / hover -> 3D path admission (epoch 1) -> goal navigation ->
  durable 3D KEEP_OUT volume commit mid-run -> RuleGuard HOLD on epoch
  disagreement -> 3D replan + re-admission (epoch 2) -> arrival / landing

Three-condition authority mechanism reused from the 3D RuleSync harness:

1. durable rule commit   -- PREPARE / independent VOTE / COMMIT with an
   fsynced WAL + state snapshot and a monotone epoch
   (``quad3d/authority3d.DurableVolumeAuthority``);
2. 3D admission          -- ``rulesync3d.gate_3d.gate_3d_path`` over the
   candidate 3D path against committed AABB volumes inflated by the body
   radius plus swept dilated-occupancy collision; admission returns a
   content-addressed token bound to (epoch, snapshot hash, path digest);
3. RuleGuard-style HOLD  -- ``rulesync3d.ruleguard.RuleGuard`` pinned to the
   executing identity; an authority epoch change during execution fails
   closed on the next monitor check, and recovery requires re-admission
   bound to a new execution identity.

Declared fidelity (also written into manifest.json)
---------------------------------------------------
* Velocity-level body: the Gazebo model is driven by a direct linear-velocity
  interface (``gz::sim::systems::VelocityControl``) at 20 Hz with capped
  speed.  There is NO PX4, no attitude loop, no rotor/aerodynamics model and
  no wind; this is not a flight-control or real-flight validation.
* Occupancy is the prepared scene's 2D 5 cm projection lifted to the declared
  flight band and dilated conservatively (over-blocking).  Rule volumes are
  separate AABBs, so a declared partial-height KEEP_OUT volume can be
  over-flown - that is the 3D-specific behaviour exercised here.  Per-height
  voxelisation of the HSSD collision triangles remains future work.
* One scene, one route, one case: a DEV integration check, not a success-rate
  measurement and not part of any formal denominator.

Outputs (fresh directory per attempt): result.json, manifest.json,
events.jsonl, trajectory.jsonl, guard_events.jsonl, authority/{wal.jsonl,
state.json}, admissions/*.json, audit.json, world.sdf (declared runtime
edit), source/ (copies with sha256), logs for gz/bridge, camera frames when
rendering is enabled.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

from quad3d import authority3d, scene3d, snapshot_history

SCHEMA_VERSION = 1
ALLOWED_SPEEDS = (0.10, 0.20, 0.25)

QUAD_MODEL = """<model name="{name}">
  <static>false</static>
  <self_collide>false</self_collide>
  <enable_wind>false</enable_wind>
  <pose>{x:.9f} {y:.9f} {z:.9f} 0 0 0</pose>
  <link name="base_link">
    <inertial>
      <mass>0.9</mass>
      <inertia>
        <ixx>0.0025</ixx><iyy>0.0025</iyy><izz>0.0045</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia>
    </inertial>
    <collision name="body">
      <geometry><box><size>0.22 0.22 0.10</size></box></geometry>
    </collision>
    <visual name="body">
      <geometry><box><size>0.18 0.18 0.06</size></box></geometry>
      <material><ambient>0.15 0.30 0.75 1</ambient>
      <diffuse>0.20 0.40 0.90 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      </material>
    </visual>
    <visual name="rotor_fl">
      <pose>0.10 0.10 0.02 0 0 0</pose>
      <geometry><cylinder><radius>0.055</radius><length>0.010</length></cylinder></geometry>
      <material><ambient>0.85 0.15 0.10 1</ambient><diffuse>0.9 0.2 0.1 1</diffuse></material>
    </visual>
    <visual name="rotor_fr">
      <pose>0.10 -0.10 0.02 0 0 0</pose>
      <geometry><cylinder><radius>0.055</radius><length>0.010</length></cylinder></geometry>
      <material><ambient>0.85 0.15 0.10 1</ambient><diffuse>0.9 0.2 0.1 1</diffuse></material>
    </visual>
    <visual name="rotor_rl">
      <pose>-0.10 0.10 0.02 0 0 0</pose>
      <geometry><cylinder><radius>0.055</radius><length>0.010</length></cylinder></geometry>
      <material><ambient>0.85 0.15 0.10 1</ambient><diffuse>0.9 0.2 0.1 1</diffuse></material>
    </visual>
    <visual name="rotor_rr">
      <pose>-0.10 -0.10 0.02 0 0 0</pose>
      <geometry><cylinder><radius>0.055</radius><length>0.010</length></cylinder></geometry>
      <material><ambient>0.85 0.15 0.10 1</ambient><diffuse>0.9 0.2 0.1 1</diffuse></material>
    </visual>
  </link>
  <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
    <topic>/quad3d/cmd_vel</topic>
  </plugin>
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_frame>quad3d/odom</odom_frame>
    <robot_base_frame>quad3d</robot_base_frame>
    <dimensions>3</dimensions>
  </plugin>
  <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
    <publish_model_pose>true</publish_model_pose>
    <publish_link_pose>false</publish_link_pose>
    <publish_visual_pose>false</publish_visual_pose>
    <publish_collision_pose>false</publish_collision_pose>
    <publish_sensor_pose>false</publish_sensor_pose>
    <use_pose_vector_msg>true</use_pose_vector_msg>
    <update_frequency>20</update_frequency>
  </plugin>
</model>
"""

MINIMAL_WORLD = """<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <light name="sun" type="directional"><cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground"><static>true</static>
      <link name="l">
        <collision name="c"><geometry><plane><normal>0 0 1</normal>
          <size>100 100</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal>
          <size>100 100</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
    {quad}
  </world>
</sdf>
"""

DEFAULT_FLIGHT = {
    "z_floor_m": 0.03,
    "z_max_m": 1.70,
    "z_plan_min_m": 0.45,
    "z_plan_max_m": 1.65,
    "z_spawn_m": 0.12,
    "cruise_z_m": 0.90,
    "body_radius_m": 0.25,
    "occ_clearance_m": 0.30,
    "voxel_res_m": 0.10,
    "speed_xy_mps": 0.20,
    "speed_z_mps": 0.20,
    "command_rate_hz": 20.0,
    "position_gain": 1.5,
    "lookahead_m": 0.25,
    "arrival_xy_tolerance_m": 0.15,
    "arrival_z_tolerance_m": 0.10,
    "hover_hold_s": 1.0,
    "hold_dwell_s": 1.0,
    "guard_rho_warn_m": 0.30,
    "guard_rho_release_m": 0.10,
}


def sha256_file(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    tmp.replace(path)


def iso_now():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def quad_xml(spawn, name="quad3d"):
    return QUAD_MODEL.format(name=name, x=spawn[0], y=spawn[1], z=spawn[2])


def write_scene_world(scene_sdf, output_sdf, quad_xyz, image_dir,
                      render_mode):
    """Copy the prepared scene world, append the quad, redirect cameras."""
    tree = ET.parse(scene_sdf)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("scene SDF has no <world>")
    if render_mode == "none":
        for model in list(world.findall("model")):
            if model.get("name") in ("perspective", "overview"):
                world.remove(model)
        cameras = []
    else:
        camera_paths = tree.findall(
            "./world/model/link/sensor/camera/save/path")
        cameras = []
        for path in camera_paths:
            name = Path(path.text).name
            if name not in ("perspective", "overview"):
                raise ValueError(f"unexpected camera name {name!r}")
            path.text = str(Path(image_dir) / name)
            cameras.append(name)
    world.append(ET.fromstring(quad_xml(quad_xyz)))
    tree.write(output_sdf, encoding="unicode", xml_declaration=True)
    return cameras


class Process:
    def __init__(self, name, command, log_path, env=None):
        self.name = name
        self.log_path = Path(log_path)
        self._log = self.log_path.open("w")
        self.process = subprocess.Popen(
            command, stdout=self._log, stderr=subprocess.STDOUT, env=env)

    def poll(self):
        return self.process.poll()

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=10)
        finally:
            self._log.close()


class RunnerError(RuntimeError):
    pass


class Mission:
    """One DEV attempt: platform bring-up, authority lifecycle, flight audit."""

    def __init__(self, args):
        self.args = args
        self.root = Path(args.output)
        self.stage = args.stage
        self.seed = int(args.seed)
        self.speed = float(args.speed)
        self.render_mode = args.render_mode
        self.wall_started = iso_now()
        self.monotonic_started = time.monotonic()
        self.processes = []
        self.events = None
        self.trajectory = None
        self.guard_events = None
        self.node = None
        self._rclpy = None
        self.result = {"result": "FAIL", "stage": self.stage,
                       "phase": "startup", "scene_id": args.scene_id,
                       "case_id": None, "error": None}
        self.case = None
        self.manifest = None
        self.cameras = []
        self.capture = []
        self.occ = None
        self.map_record = None
        self.authority = None
        self.guard = None
        self.execution = None
        self.rows = []
        self.phases = {}
        self.guard_counts = {}
        self.hold_record = None
        self.recovery = None
        self.arrival = None
        self.audit_record = None
        self.negative_control = None
        self.initial_admission = None
        self.initial_path = None
        self.recovery_path_points = None
        self.z_integral = 0.0
        self.last_velocity_time = None

    # -- logging ---------------------------------------------------------
    def event(self, name, **fields):
        row = {"receipt_ns": time.monotonic_ns(),
               "sim_ns": self.sim_ns(), "wall_time": iso_now(),
               "event": name}
        row.update(fields)
        self.events.write(json.dumps(row, sort_keys=True) + "\n")
        self.events.flush()

    def phase(self, name):
        self.result["phase"] = name
        self.event("phase", phase=name)
        print(f"[quad3d] phase {name}", flush=True)

    def sim_ns(self):
        if self.node is None:
            return None
        try:
            return int(self.node.get_clock().now().nanoseconds)
        except Exception:
            return None

    # -- setup -----------------------------------------------------------
    def setup(self):
        self.events = (self.root / "events.jsonl").open("w")
        self.trajectory = (self.root / "trajectory.jsonl").open("w")
        self.guard_events = (self.root / "guard_events.jsonl").open("w")
        for name in ("logs", "images", "admissions"):
            (self.root / name).mkdir()

    def environment(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(int(self.args.ros_domain))
        env["GZ_PARTITION"] = "rulesync_quad3d_" + self.root.name
        env["GZ_IP"] = "127.0.0.1"
        env["LP_NUM_THREADS"] = "2"
        env["OMP_NUM_THREADS"] = "2"
        env["OPENBLAS_NUM_THREADS"] = "1"
        env.pop("ROS_DISCOVERY_SERVER", None)
        env.pop("LIBGL_ALWAYS_SOFTWARE", None)
        if self.render_mode == "gpu":
            env.update(MESA_LOADER_DRIVER_OVERRIDE="iris",
                       EGL_PLATFORM="surfaceless")
        elif self.render_mode == "software":
            env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        if self.args.scene_id:
            env["GZ_SIM_RESOURCE_PATH"] = f"/assets/{self.args.scene_id}:/assets"
        else:
            env["GZ_SIM_RESOURCE_PATH"] = "/assets"
        # The runner's own ROS node must live on the same domain/rmw settings
        # as the bridge child processes; the container shell does not set
        # ROS_DOMAIN_ID, so applying the child environment to this process is
        # what keeps the node and the bridge discoverable.
        os.environ.update(env)
        os.environ.pop("ROS_DISCOVERY_SERVER", None)
        return env

    def check_gpu(self, env):
        if self.render_mode != "gpu":
            return None
        probe = subprocess.run(
            ["eglinfo", "-B", "-p", "surfaceless"], env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        (self.root / "gpu_renderer.log").write_text(probe.stdout)
        if not (probe.returncode == 0 and "Intel" in probe.stdout
                and "Iris" in probe.stdout and "llvmpipe" not in probe.stdout):
            raise RunnerError("hardware Intel rendering was not verified")
        return probe.stdout

    def start_platform(self, world_path, capture_cameras):
        env = self.environment()
        self.processes.append(Process(
            "gazebo", ["gz", "sim", "-r", "-s", "--headless-rendering",
                       "--seed", str(self.seed), str(world_path)],
            self.root / "logs/gazebo.log", env))
        bridge_topics = [
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/quad3d/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/quad3d/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/quad3d/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ]
        self.processes.append(Process(
            "bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + bridge_topics, self.root / "logs/bridge.log", env))
        self.capture_cameras = capture_cameras

    def check_processes(self):
        for process in self.processes:
            if process.poll() not in (None, 0):
                log = process.log_path.read_text(errors="replace")[-2000:]
                raise RunnerError(
                    f"{process.name} exited with {process.poll()}:\n{log}")

    def capture_images(self):
        if not self.capture_cameras:
            return []
        env = self.environment()
        for camera in self.capture_cameras:
            self.processes.append(Process(
                f"camera_{camera}",
                ["gz", "topic", "-e", "--json-output", "-n", "1",
                 "-t", f"/capture/{camera}"],
                self.root / f"logs/camera_{camera}.log", env))
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            if list((self.root / "images").glob("*")):
                break
            time.sleep(0.5)
        return [str(path.relative_to(self.root))
                for path in sorted((self.root / "images").glob("*"))]

    # -- ROS node --------------------------------------------------------
    def open_node(self):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        class QuadNode(Node):
            def __init__(self):
                super().__init__("quad3d_dev_runner")
                self.set_parameters(
                    [rclpy.parameter.Parameter("use_sim_time", value=True)])
                self.cmd_pub = self.create_publisher(
                    Twist, "/quad3d/cmd_vel", 10)
                self.create_subscription(Odometry, "/model/quad3d/odometry",
                                         self.on_odom, 20)
                self.create_subscription(TFMessage, "/model/quad3d/pose",
                                         self.on_pose, 20)
                self.pose = None
                self.odom = None
                self.pose_count = 0
                self.odom_count = 0
                self.command_count = 0
                self.last_command = [0.0, 0.0, 0.0]

            def on_pose(self, msg):
                for transform in msg.transforms:
                    if transform.child_frame_id == "quad3d":
                        p = transform.transform.translation
                        self.pose = [float(p.x), float(p.y), float(p.z)]
                        self.pose_count += 1

            def on_odom(self, msg):
                p = msg.pose.pose.position
                self.odom = [float(p.x), float(p.y), float(p.z)]
                self.odom_count += 1

            def send(self, vx, vy, vz):
                twist = Twist()
                twist.linear.x = float(vx)
                twist.linear.y = float(vy)
                twist.linear.z = float(vz)
                self.cmd_pub.publish(twist)
                self.command_count += 1
                self.last_command = [float(vx), float(vy), float(vz)]

        rclpy.init()
        self._rclpy = rclpy
        self.node = QuadNode()

    def stage_spin(self):
        if self._rclpy is None:
            import rclpy
            self._rclpy = rclpy
        self._rclpy.spin_once(self.node, timeout_sec=0.01)

    def spin(self, seconds):
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self.stage_spin()

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            self.stage_spin()
            self.check_processes()
            if predicate():
                return True
        raise RunnerError(f"timeout waiting for {description}")

    # -- case / geometry -------------------------------------------------
    def load_case(self):
        case = json.loads(Path(self.args.case).read_text())
        if case.get("schema_version") != SCHEMA_VERSION:
            raise RunnerError("unsupported 3D case schema_version")
        if case.get("kind") != "rulesync_quad3d_volume_task":
            raise RunnerError("unsupported 3D case kind")
        if case.get("scene") != self.args.scene_id:
            raise RunnerError("case scene does not match --scene-id")
        self.case = case
        flight = dict(DEFAULT_FLIGHT)
        flight.update(case["flight"])
        if abs(self.speed - float(flight["speed_xy_mps"])) > 1e-9:
            raise RunnerError("--speed does not match the case speed")
        if round(self.speed, 2) not in ALLOWED_SPEEDS:
            raise RunnerError("DEV runner only supports speeds 0.10/0.20/0.25")
        self.flight = flight
        self.scene_sdf = Path(self.args.scene) / f"{self.args.scene_id}.sdf"
        map_path = Path(self.args.scene) / "nav_v1/map.json"
        if sha256_file(map_path) != case["map_sha256"]:
            raise RunnerError("case map_sha256 does not match scene map")
        self.map_record = scene3d.load_map(map_path)
        vox = os.environ.get("RULESYNC_QUAD_VOXEL_NPZ")
        if vox and Path(vox).exists():
            self.occ = scene3d.VoxelOccupancy(
                vox, flight["occ_clearance_m"], flight["z_floor_m"],
                flight["z_max_m"])
            self.occupancy_source = "voxel:" + str(vox)
        else:
            self.occ = scene3d.MapOccupancy(
                self.map_record, flight["occ_clearance_m"],
                flight["z_floor_m"], flight["z_max_m"])
            self.occupancy_source = "projected_2d"
        self.spawn = [float(v) for v in case["spawn_xyz"]]
        self.goal = [float(v) for v in case["goal_xyz"]]
        self.rules = case["rules"]
        self.trigger = float(case["trigger"]["distance_m"])
        self.route_xy = [[float(x), float(y)]
                         for x, y in case["route"]["xy_path"]]
        self.bounds = scene3d.planning_bounds(
            self.occ, flight["z_plan_min_m"], flight["z_plan_max_m"])
        self.bounds_gate = (self.occ.xmin, self.occ.ymin,
                            flight["z_floor_m"], self.occ.xmax,
                            self.occ.ymax, flight["z_max_m"])
        self.event("occupancy_source", source=self.occupancy_source)
        self.event("case_loaded", case_id=case["id"],
                   case_sha256=sha256_file(Path(self.args.case)),
                   route_length_m=case["route"]["length_m"],
                   spawn=self.spawn, goal=self.goal)

    def open_authority(self):
        self.authority = authority3d.DurableVolumeAuthority(
            self.root / "authority")
        self.authority.recovery = self.authority.recover()
        self.event("authority_recovered", **self.authority.recovery)
        from rulesync3d.ruleguard import RuleGuard
        self.guard = RuleGuard(
            rho_warn=float(self.flight["guard_rho_warn_m"]),
            rho_release=float(self.flight["guard_rho_release_m"]))

    def inflate(self):
        return float(self.flight["body_radius_m"])

    def gate_volumes(self):
        return self.authority.gate_volumes(self.inflate())

    def feasibility_voter(self, volumes, evidence):
        """Independent pre-commit vote (one voter in DEV, the bridge role).

        A snapshot that encloses the vehicle or leaves no admitted 3D path to
        the goal is voted down; COMMIT then fails closed.
        """
        inflation = float(self.flight["body_radius_m"])
        current = (evidence or {}).get("current_xyz") or self.node.pose
        if current is None:
            return False, "no-vehicle-position"
        for volume in volumes:
            for point in (current, self.goal):
                if all(volume["bmin"][i] - inflation < point[i]
                       < volume["bmax"][i] + inflation for i in range(3)):
                    return False, "proposed-volume-encloses-body"
        start = [current[0], current[1],
                 max(float(current[2]), float(self.flight["z_plan_min_m"]))]
        goal = [self.goal[0], self.goal[1], float(self.flight["cruise_z_m"])]
        reserve = float(self.flight.get("planning_reserve_m", 0.05))
        path, planner = scene3d.plan_path(
            self.occ, self.bounds, float(self.flight["voxel_res_m"]),
            start, goal, volumes, inflation + reserve, seed=self.seed)
        if path is None:
            return False, f"no-3d-path-under-proposed-volumes ({planner})"
        ok, reason = scene3d.gate_path(
            path, self.occ, volumes, inflation, self.bounds_gate)
        return bool(ok), f"path-exists-and-admitted ({planner}, {reason})"

    def commit(self, rule_key, note):
        volumes = self.rules[rule_key]["volumes"]
        txn = self.authority.propose(volumes, note=note)
        self.event("volume_prepare", rule=rule_key, txn_id=txn["txn_id"],
                   txn_epoch=txn["epoch"])
        vote = self.authority.vote(
            txn["txn_id"], self.feasibility_voter,
            evidence={"current_xyz": list(self.node.pose)})
        self.event("volume_vote", rule=rule_key, txn_id=txn["txn_id"],
                   vote=vote["passed"], reason=vote["reason"])
        record = self.authority.commit(txn["txn_id"])
        self.event("volume_commit", rule=rule_key, epoch=record["epoch"],
                   snapshot_hash=record["snapshot_hash"],
                   volume_ids=[v.get("id") for v in record["volumes"]],
                   wal_offset=record["wal_offset"])
        self.guard.update_snapshot(record["epoch"], record["snapshot_hash"])
        return record

    def admit(self, path, tag):
        inflation = self.inflate()
        volumes = self.authority.volumes
        ok, reason = scene3d.gate_path(
            path, self.occ, volumes, inflation, self.bounds_gate)
        digest = scene3d.path_digest(path, self.sim_ns())
        record = {"tag": tag, "accepted": bool(ok), "reason": reason,
                  "epoch": self.authority.epoch,
                  "snapshot_hash": self.authority.snapshot_hash,
                  "path_digest": digest, "path_points": len(path),
                  "inflation_radius_m": inflation,
                  "occupancy_clearance_m": float(
                      self.flight["occ_clearance_m"]),
                  "volumes": [v.get("id") for v in volumes],
                  "receipt_ns": time.monotonic_ns(),
                  "sim_ns": self.sim_ns(), "wall_time": iso_now()}
        if not ok:
            self.event("admission_rejected", **record)
            write_json(self.root / f"admissions/{tag}_rejected.json", record)
            raise RunnerError(f"3D admission rejected: {reason}")
        record["token"] = self.authority.issue_token(digest)
        self.guard.clear_execution()
        record["execution_bound"] = bool(
            self.guard.bind_execution(self.authority.epoch,
                                      self.authority.snapshot_hash))
        write_json(self.root / f"admissions/{tag}_admitted.json", record)
        self.event("admission_accepted", **record)
        self.execution = record
        return record

    def verify_execution(self, path):
        record = self.execution
        digest = scene3d.path_digest(path, record["sim_ns"])
        ok_digest = digest == record["path_digest"]
        ok_token, reason, _ = self.authority.verify_token(
            digest, record["token"], record["epoch"],
            record["snapshot_hash"])
        return bool(ok_digest and ok_token), reason

    # -- control ---------------------------------------------------------
    def velocity_towards(self, current, target):
        """Position-error velocity command; bounded integral only on z.

        The direct linear-velocity interface leaves a small constant
        gravity-sag per command interval, so a bounded vertical integral
        removes the steady-state altitude offset (declared in the manifest as
        part of the velocity-level controller; it is not an attitude loop).
        """
        gain = float(self.flight["position_gain"])
        ki = 0.8
        vmax_xy = float(self.flight["speed_xy_mps"])
        vmax_z = float(self.flight["speed_z_mps"])
        now = time.monotonic()
        last = self.last_velocity_time
        dt = 0.05 if last is None else max(1e-3, min(0.10, now - last))
        self.last_velocity_time = now
        ex, ey, ez = (target[0] - current[0], target[1] - current[1],
                      target[2] - current[2])
        # conditional integration (anti-windup): integrate only while the
        # vertical command is not saturated by the speed cap
        vz_unsat = gain * ez + ki * self.z_integral
        if abs(vz_unsat) < vmax_z:
            self.z_integral = max(-0.20, min(0.20,
                                             self.z_integral + ez * dt))
        vx, vy = gain * ex, gain * ey
        horizontal = math.hypot(vx, vy)
        if horizontal > vmax_xy:
            vx, vy = vx / horizontal * vmax_xy, vy / horizontal * vmax_xy
        vz = max(-vmax_z, min(vmax_z, gain * ez + ki * self.z_integral))
        return [vx, vy, vz]

    def guard_check(self, phase, pose):
        state, info = self.guard.check_bound_3d(
            list(pose), self.gate_volumes(), 0.0)
        counts = self.guard_counts.setdefault(
            phase, {"CLEAR": 0, "WARNING": 0, "HOLD": 0})
        counts[state] = counts.get(state, 0) + 1
        if state == "HOLD":
            self.guard_events.write(json.dumps(
                {"receipt_ns": time.monotonic_ns(), "sim_ns": self.sim_ns(),
                 "wall_time": iso_now(), "phase": phase, "state": state,
                 "info": info, "pose": list(pose)}) + "\n")
            self.guard_events.flush()
            self.event("guard_hold", phase=phase, info=info, pose=list(pose))
        return state, info

    def track(self, path, phase, budget_s, arrival=None,
              update_trigger=None):
        """Pure-pursuit follow with per-tick RuleGuard + identity checks."""
        rate = float(self.flight["command_rate_hz"])
        period = 1.0 / rate
        lookahead = float(self.flight["lookahead_m"])
        wall_start = time.monotonic()
        index = min(range(len(path)),
                    key=lambda i: math.dist(self.node.pose, path[i]))
        last_command = 0.0
        last_state = None
        warned = False
        while True:
            self.stage_spin()
            self.check_processes()
            now = time.monotonic()
            if now - wall_start > float(budget_s):
                raise RunnerError(f"{phase}: wall-clock budget expired")
            current = self.node.pose
            if current is None:
                continue
            while (index + 1 < len(path)
                   and math.dist(current, path[index + 1]) < lookahead):
                index += 1
            target = path[index]
            if now - last_command >= period:
                state, info = self.guard_check(phase, current)
                if state == "HOLD":
                    self.node.send(0.0, 0.0, 0.0)
                    return "hold"
                if state == "WARNING" and not warned:
                    warned = True
                    self.event("guard_warning", phase=phase, info=info,
                               pose=list(current))
                if state != last_state:
                    self.event("guard_state", phase=phase, state=state,
                               info=info, pose=list(current))
                    last_state = state
                identity_ok, identity_reason = self.verify_execution(path)
                if not identity_ok:
                    self.event("execution_identity_rejected", phase=phase,
                               reason=identity_reason, pose=list(current))
                    self.node.send(0.0, 0.0, 0.0)
                    return "hold"
                velocity = self.velocity_towards(current, target)
                self.node.send(*velocity)
                last_command = now
                row = {"receipt_ns": time.monotonic_ns(),
                       "sim_ns": self.sim_ns(), "wall_time": iso_now(),
                       "phase": phase, "pose": list(current),
                       "odom": list(self.node.odom) if self.node.odom else None,
                       "target": list(target), "command": velocity,
                       "guard": state, "epoch": self.authority.epoch,
                       "index": index}
                self.trajectory.write(json.dumps(row, sort_keys=True) + "\n")
                self.rows.append(row)
                if len(self.rows) % 40 == 0:
                    self.trajectory.flush()
            if arrival is not None and (
                    math.dist(current[:2], arrival[:2])
                    <= float(self.flight["arrival_xy_tolerance_m"])):
                self.event("waypoint_reached", phase=phase, pose=list(current),
                           target=list(arrival), guard=last_state,
                           altitude_m=current[2])
                self.trajectory.flush()
                return "arrival"
            if (update_trigger is not None
                    and math.dist(current[:2], self.spawn[:2])
                    >= update_trigger):
                self.event("update_trigger_reached", phase=phase,
                           pose=list(current),
                           displacement_m=math.dist(current[:2],
                                                    self.spawn[:2]),
                           guard=last_state)
                self.node.send(0.0, 0.0, 0.0)
                self.spin(0.25)
                return "update"

    def takeoff(self):
        self.phase("takeoff")
        started = time.monotonic()
        cruise = float(self.flight["cruise_z_m"])
        self.wait(lambda: self.node.pose is not None, 180,
                  "first ground-truth pose")
        self.event("platform_ready", pose=list(self.node.pose),
                   odom=list(self.node.odom) if self.node.odom else None,
                   pose_messages=self.node.pose_count,
                   odom_messages=self.node.odom_count)
        rate = float(self.flight["command_rate_hz"])
        last = 0.0
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            self.stage_spin()
            self.check_processes()
            now = time.monotonic()
            current = self.node.pose
            if now - last >= 1.0 / rate:
                self.node.send(*self.velocity_towards(
                    current, [self.spawn[0], self.spawn[1], cruise]))
                last = now
            if (math.dist(current[:2], self.spawn[:2]) < 0.30
                    and abs(current[2] - cruise) < 0.06):
                break
        else:
            raise RunnerError("takeoff did not reach cruise altitude")
        hover = self.hover_hold(float(self.flight["hover_hold_s"]))
        self.event("takeoff_complete", cruise_z_m=cruise,
                   hover=hover,
                   pose=list(self.node.pose),
                   wall_s=time.monotonic() - started)
        self.phases["takeoff"] = {
            "wall_s": time.monotonic() - started,
            "sim_ns": self.sim_ns()}

    def hover_hold(self, seconds):
        """Stationary hover (position hold) used by HOLD states and takeoff."""
        rate = float(self.flight["command_rate_hz"])
        deadline = time.monotonic() + float(seconds)
        start_pose = list(self.node.pose)
        deviation = 0.0
        last = 0.0
        while time.monotonic() < deadline:
            self.stage_spin()
            now = time.monotonic()
            if now - last >= 1.0 / rate:
                self.node.send(*self.velocity_towards(self.node.pose,
                                                      start_pose))
                last = now
            deviation = max(deviation,
                            math.dist(self.node.pose, start_pose))
            time.sleep(0.005)
        return {"start_pose": start_pose, "end_pose": list(self.node.pose),
                "max_deviation_m": deviation,
                "duration_s": float(seconds)}

    def land(self):
        self.phase("landing")
        ground = float(self.flight["z_spawn_m"]) + 0.02
        deadline = time.monotonic() + 45.0
        last = 0.0
        rate = float(self.flight["command_rate_hz"])
        while time.monotonic() < deadline:
            self.stage_spin()
            now = time.monotonic()
            current = self.node.pose
            if now - last >= 1.0 / rate:
                self.node.send(*self.velocity_towards(
                    current, [self.goal[0], self.goal[1], ground]))
                last = now
            if abs(current[2] - ground) < 0.08:
                break
        self.node.send(0.0, 0.0, 0.0)
        self.spin(0.3)
        pose = list(self.node.pose)
        self.event("landed", pose=pose)
        return {"landed_pose": pose,
                "landing_xy_error_m": math.dist(pose[:2], self.goal[:2]),
                "landing_z_error_m": abs(pose[2] - ground)}

    # -- mission ---------------------------------------------------------
    def build_scene_world(self):
        self.cameras = write_scene_world(
            self.scene_sdf, self.root / "world.sdf",
            [self.spawn[0], self.spawn[1], self.spawn[2]],
            self.root / "images", self.render_mode)

    def run_platform_selftest(self):
        """Velocity-level platform qualification on a minimal world."""
        self.phase("platform_selftest")
        self.args.scene_id = None
        spawn = [0.0, 0.0, 0.50]
        self.spawn = spawn
        self.goal = [1.0, 0.0, 0.50]
        self.flight = dict(DEFAULT_FLIGHT)
        self.flight.update(cruise_z_m=0.80, hold_dwell_s=1.0,
                           hover_hold_s=1.0,
                           arrival_xy_tolerance_m=0.15)
        world = MINIMAL_WORLD.format(quad=quad_xml(spawn))
        (self.root / "world.sdf").write_text(world)
        self.manifest = {
            "schema": "rulesync.quad3d.platform_selftest.v1",
            "stage": "platform_selftest",
            "formal_denominator": False,
            "declared_fidelity": {
                "body": "velocity-level quadrotor model; no PX4, attitude "
                        "loop, rotor aerodynamics or wind",
                "scope": "platform qualification only; empty world, no "
                         "scene, no rule volumes, no paper evidence",
            },
            "runtime_world_sha256": sha256_file(self.root / "world.sdf"),
            "render_mode": self.render_mode,
            "ros_domain_id": int(self.args.ros_domain),
            "source_sha256": {str(path): sha256_file(path)
                              for path in self.source_files()},
            "runner_sha256": sha256_file(Path(__file__)),
            "wall_started": self.wall_started,
        }
        write_json(self.root / "manifest.json", self.manifest)
        env = self.environment()
        self.check_gpu(env)
        self.start_platform(self.root / "world.sdf", [])
        self.open_node()
        self.takeoff()
        hover = self.hover_hold(2.0)
        cruise = float(self.flight["cruise_z_m"])
        path = [list(self.node.pose), [1.00, 0.0, cruise]]
        rate = float(self.flight["command_rate_hz"])
        last = 0.0
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            self.stage_spin()
            now = time.monotonic()
            current = self.node.pose
            if now - last >= 1.0 / rate:
                self.node.send(*self.velocity_towards(current, path[-1]))
                last = now
            if (math.dist(current[:2], path[-1][:2]) <= 0.15
                    and abs(current[2] - cruise) <= 0.10):
                break
        self.node.send(0.0, 0.0, 0.0)
        self.spin(0.3)
        waypoint_pose = list(self.node.pose)
        waypoint_odom = list(self.node.odom) if self.node.odom else None
        offset = (math.dist(waypoint_pose, waypoint_odom)
                  if waypoint_odom else None)
        landing = self.land()
        self.result["platform_selftest"] = {
            "hover": hover,
            "waypoint_pose": waypoint_pose,
            "waypoint_error_m": math.dist(waypoint_pose, path[-1]),
            "landing": landing,
            "odom_vs_pose_offset_m": offset,
            "pose_messages": self.node.pose_count,
            "odom_messages": self.node.odom_count,
            "commands": self.node.command_count,
        }
        checks = {
            "hover_drift_ok": hover["max_deviation_m"] <= 0.05,
            "waypoint_ok": math.dist(waypoint_pose, path[-1]) <= 0.15,
            "landing_ok": landing["landing_xy_error_m"] <= 0.15,
            "pose_stream_ok": self.node.pose_count >= 100,
            "odom_stream_ok": self.node.odom_count >= 100,
        }
        self.result["platform_selftest"]["checks"] = checks
        if not all(checks.values()):
            raise RunnerError(f"platform selftest failed: {checks}")
        self.result["result"] = "PASS"

    def run_initial_commit(self):
        self.phase("initial_commit")
        started = time.monotonic()
        record = self.commit("initial", "initial_rule_commit")
        self.event("authority_ready", epoch=record["epoch"],
                   snapshot_hash=record["snapshot_hash"])
        return {"wall_s": time.monotonic() - started,
                "epoch": record["epoch"],
                "snapshot_hash": record["snapshot_hash"],
                "wal_offset": record["wal_offset"]}

    def run_initial_flight(self):
        self.phase("initial_flight")
        path, planner, ok, reason = scene3d.initial_flight_path(
            self.occ, self.bounds, self.bounds_gate, self.spawn, self.goal,
            self.flight["cruise_z_m"], self.flight["z_spawn_m"],
            self.authority.volumes, self.inflate(),
            res=self.flight["voxel_res_m"], seed=self.seed)
        self.event("initial_path_gate", accepted=bool(ok), reason=reason,
                   planner=planner, points=len(path) if path else None,
                   length_m=(float(sum(math.dist(path[i], path[i + 1])
                                       for i in range(len(path) - 1)))
                             if path else None))
        if not ok or path is None:
            raise RunnerError(f"initial 3D path rejected: {reason}")
        self.initial_path = path
        self.initial_admission = self.admit(path, "initial")
        terminal = self.track(
            path, "initial_flight",
            float(self.case["timing"]["wall_budget_s"]),
            update_trigger=self.trigger)
        if terminal != "update":
            raise RunnerError(
                f"initial flight ended as {terminal!r} before the update "
                "trigger")
        return {"terminal": terminal, "trigger_distance_m": self.trigger,
                "trigger_pose": list(self.node.pose),
                "sim_ns": self.sim_ns()}

    def run_update(self):
        self.phase("rule_update_and_hold")
        started = time.monotonic()
        pose_at_request = list(self.node.pose)
        self.hover_hold(0.3)
        self.event("proactive_hold_for_update", pose=pose_at_request)
        record = self.commit("replacement", "mid_run_replacement_commit")
        commit_sim = self.sim_ns()
        self.event("mid_run_commit", epoch=record["epoch"],
                   snapshot_hash=record["snapshot_hash"],
                   pose=list(self.node.pose))
        stale_ok, stale_reason, _ = self.authority.verify_token(
            self.execution["path_digest"], self.execution["token"],
            record["epoch"], record["snapshot_hash"])
        self.event("stale_authorization_rejected", accepted=bool(stale_ok),
                   reason=stale_reason,
                   executing_epoch=self.execution["epoch"],
                   authority_epoch=record["epoch"])
        if stale_ok:
            raise RunnerError("stale authorization was accepted (fail-open)")
        state, info = self.guard_check("rule_update_and_hold",
                                       list(self.node.pose))
        hold_sim = self.sim_ns()
        self.event("rule_guard_decision", state=state, info=info,
                   pose=list(self.node.pose))
        if state != "HOLD":
            raise RunnerError(
                f"monitor did not HOLD after the authority epoch change: "
                f"{state} {info}")
        if info.get("reason") != "epoch-disagreement":
            raise RunnerError(
                f"HOLD reason {info.get('reason')!r} is not the epoch "
                "disagreement under test")
        self.hold_record = {
            "state": state, "info": info, "pose": list(self.node.pose),
            "commit_sim_ns": commit_sim, "hold_sim_ns": hold_sim,
            "commit_to_hold_sim_s": (hold_sim - commit_sim) / 1e9,
            "intent_pose": pose_at_request,
            "wall_s": time.monotonic() - started}
        ok, reason = scene3d.gate_path(
            self.initial_path, self.occ, self.authority.volumes,
            self.inflate(), self.bounds_gate)
        self.event("post_update_initial_path_gate", accepted=bool(ok),
                   reason=reason, epoch=self.authority.epoch)
        if ok:
            raise RunnerError(
                "3D gate accepted the pre-update path after the replacement "
                "volume was committed")
        self.negative_control = {"accepted": bool(ok), "reason": reason}
        dwell = self.hover_hold(float(self.flight["hold_dwell_s"]))
        self.hold_record["dwell"] = dwell
        self.event("hold_dwell_complete", pose=list(self.node.pose),
                   dwell_s=float(self.flight["hold_dwell_s"]))
        return self.hold_record

    def recovery_path(self):
        current = list(self.node.pose)
        inflation = self.inflate()
        start = [current[0], current[1],
                 max(current[2], float(self.flight["z_plan_min_m"]))]
        goal_hover = [self.goal[0], self.goal[1],
                      float(self.flight["cruise_z_m"])]
        reserve = float(self.flight.get("planning_reserve_m", 0.05))
        path, planner = scene3d.plan_path(
            self.occ, self.bounds, float(self.flight["voxel_res_m"]),
            start, goal_hover, self.authority.volumes, inflation + reserve,
            seed=self.seed)
        if path is None:
            raise RunnerError(
                "no 3D recovery path exists under the committed replacement "
                "volume")
        smoothed = scene3d.smooth_path(
            path, self.occ, self.authority.volumes, inflation + reserve,
            bounds=self.bounds_gate)
        full = scene3d.dedupe(
            [start] + smoothed[1:]
            + [[self.goal[0], self.goal[1], float(self.flight["z_spawn_m"])]])
        full = scene3d.densify(full)
        return full, planner, path

    def run_recovery(self):
        self.phase("recovery_readmission")
        started = time.monotonic()
        path, planner, raw = self.recovery_path()
        self.event("recovery_path_planned", planner=planner,
                   raw_points=len(raw), path_points=len(path),
                   path_length_m=float(sum(
                       math.dist(path[i], path[i + 1])
                       for i in range(len(path) - 1))),
                   pose=list(self.node.pose))
        ok, reason = scene3d.gate_path(
            path, self.occ, self.authority.volumes, self.inflate(),
            self.bounds_gate)
        self.event("recovery_path_gate", accepted=bool(ok), reason=reason)
        if not ok:
            raise RunnerError(f"recovery 3D path rejected: {reason}")
        admission = self.admit(path, "recovery")
        recover_sim = self.sim_ns()
        state, info = self.guard_check("recovery_readmission",
                                       list(self.node.pose))
        self.event("readmission_monitor_decision", state=state, info=info)
        if state == "HOLD":
            raise RunnerError(
                f"monitor still HOLD after re-admission: {info}")
        self.recovery = {
            "planner": planner, "epoch": admission["epoch"],
            "snapshot_hash": admission["snapshot_hash"],
            "path_points": len(path),
            "readmission_sim_ns": recover_sim,
            "hold_to_readmission_sim_s":
                (recover_sim - self.hold_record["hold_sim_ns"]) / 1e9,
            "wall_s": time.monotonic() - started,
            "monitor_state_after_bind": state}
        self.recovery_path_points = path
        terminal = self.track(
            path, "recovery_flight",
            float(self.case["timing"]["wall_budget_s"]),
            arrival=[self.goal[0], self.goal[1],
                     float(self.flight["cruise_z_m"])])
        if terminal != "arrival":
            raise RunnerError(
                f"recovery flight ended as {terminal!r} before arrival")
        self.recovery["terminal"] = terminal
        self.recovery["arrival_sim_ns"] = self.sim_ns()
        return self.recovery

    def run_arrival(self):
        self.phase("arrival")
        pose = list(self.node.pose)
        cruise = float(self.flight["cruise_z_m"])
        goal_error = math.dist(pose[:2], self.goal[:2])
        if goal_error > float(self.flight["arrival_xy_tolerance_m"]):
            raise RunnerError(
                f"arrival xy error {goal_error:.3f} m exceeds tolerance")
        landing = self.land()
        self.arrival = {
            "goal_xy": self.goal[:2], "arrival_pose": pose,
            "independent_goal_error_m": goal_error,
            "arrival_altitude_m": pose[2],
            "arrival_cruise_altitude_offset_m": abs(pose[2] - cruise),
            "landing": landing,
            "nonzero_commands": int(self.node.command_count)}
        return self.arrival

    def audit(self):
        self.phase("audit")
        poses = [list(row["pose"]) for row in self.rows if row.get("pose")]
        history = snapshot_history.replay_snapshots(
            self.root / "authority/wal.jsonl", self.authority._hash)
        initial_snapshot = history[self.initial_admission["epoch"]]
        active_snapshot = history[self.execution["epoch"]]
        if (initial_snapshot["snapshot_hash"] != self.initial_admission["snapshot_hash"]
                or active_snapshot["snapshot_hash"] != self.execution["snapshot_hash"]
                or active_snapshot["snapshot_hash"] != self.authority.snapshot_hash
                or active_snapshot["volumes"] != self.authority.volumes):
            raise RunnerError("audit snapshot identity does not match WAL/admission")
        initial_volumes = initial_snapshot["volumes"]
        committed_volumes = active_snapshot["volumes"]
        audit = {
            "snapshot_semantics": "complete_committed_snapshot_from_WAL",
            "audited_snapshot_volume_ids": {
                str(epoch): [v.get("id") for v in snapshot["volumes"]]
                for epoch, snapshot in history.items()},
            "declared_initial_path_clearance": scene3d.clearance_profile(
                self.initial_path, initial_volumes, self.inflate()),
            "declared_recovery_path_clearance": scene3d.clearance_profile(
                self.recovery_path_points, committed_volumes, self.inflate()),
            "committed_snapshot_epoch": self.authority.epoch,
            "committed_snapshot_hash": self.authority.snapshot_hash,
            "path_digest_initial": self.initial_admission["path_digest"],
            "token_initial": self.initial_admission["token"],
            "path_digest_recovery": self.execution["path_digest"],
            "token_recovery": self.execution["token"],
        }
        replay_root = self.root / "audit_authority"
        # fresh replay of the run WAL into a separate directory
        replay = authority3d.DurableVolumeAuthority(replay_root)
        shutil.copy2(self.root / "authority/wal.jsonl",
                     replay_root / "wal.jsonl")
        replay_state = replay.recover()
        audit["wal_replay_after_run"] = replay_state
        audit["wal_matches_committed_state"] = bool(
            replay_state["state_matches_replay"])
        epoch1 = [list(row["pose"]) for row in self.rows
                  if row.get("epoch") == 1]
        epoch2 = [list(row["pose"]) for row in self.rows
                  if row.get("epoch") == 2]
        if epoch1:
            audit["executed_epoch1_vs_initial_snapshot"] = \
                scene3d.clearance_profile(
                    scene3d.resample(epoch1), initial_volumes,
                    self.inflate())
            # expected: the pre-update plan traverses the future replacement
            # zone (that is what makes the mid-run HOLD demonstration real)
            audit["executed_epoch1_inside_replacement_zone_samples"] = \
                scene3d.clearance_profile(
                    scene3d.resample(epoch1),
                    self.rules["replacement"]["volumes"],
                    self.inflate())["raw_violation_samples"]
        if epoch2:
            audit["executed_epoch2_vs_committed_snapshot"] = \
                scene3d.clearance_profile(
                    scene3d.resample(epoch2), committed_volumes,
                    self.inflate())
        if poses:
            dense = scene3d.resample(poses)
            audit["executed_physical_penetration_samples"] = \
                scene3d.physical_penetration_profile(dense, self.occ)
            audit["executed_planning_margin_breach_samples"] = \
                scene3d.occupancy_profile(dense, self.occ)
            audit["executed_samples"] = len(dense)
        audit["guard_counts"] = self.guard_counts
        audit["guard_event_states"] = [row[1] for row in self.guard.events]
        self.audit_record = audit
        write_json(self.root / "audit.json", audit)
        self.event("audit_complete", **{
            key: value for key, value in audit.items()
            if key.startswith("wal_") or key == "executed_samples"})
        # hard independent checks on the recorded ground truth
        for key in ("executed_epoch1_vs_initial_snapshot",
                    "executed_epoch2_vs_committed_snapshot"):
            if audit.get(key, {}).get("raw_violation_samples", 0) != 0:
                raise RunnerError(
                    f"executed ground-truth poses entered a KEEP_OUT volume "
                    f"committed at that time ({key})")
        if audit.get("executed_physical_penetration_samples", 0) != 0:
            raise RunnerError("executed ground-truth poses penetrated the "
                              "raw collision geometry (body margin)")
        if not audit["wal_matches_committed_state"]:
            raise RunnerError("WAL replay does not match the committed state")

    # -- manifest --------------------------------------------------------
    def build_manifest(self):
        flight = self.flight
        return {
            "schema": "rulesync.quad3d.dev.v1",
            "stage": self.stage,
            "pilot": True,
            "formal_denominator": False,
            "scene_id": self.args.scene_id,
            "case_id": self.case["id"],
            "case_sha256": sha256_file(Path(self.args.case)),
            "map_sha256": self.case["map_sha256"],
            "world_sha256": sha256_file(self.scene_sdf),
            "runtime_world_sha256": sha256_file(self.root / "world.sdf"),
            "runtime_world_edits": [
                "appended gz::sim::systems::VelocityControl quadrotor model "
                "'quad3d' at the case spawn",
                "redirected the two documentation camera save paths into the "
                "run directory",
            ],
            "seed": self.seed,
            "speed_xy_mps": float(flight["speed_xy_mps"]),
            "speed_z_mps": float(flight["speed_z_mps"]),
            "command_rate_hz": float(flight["command_rate_hz"]),
            "declared_fidelity": {
                "body": "velocity-level quadrotor model (direct linear "
                        "velocity interface, 0.22 m collision box); NO PX4, "
                        "no attitude/rotor/aerodynamic model, no wind",
                "occupancy": "conservative 2D 5 cm projection lifted to the "
                             f"declared flight band [{flight['z_floor_m']}, "
                             f"{flight['z_max_m']}] m and dilated by "
                             f"{flight['occ_clearance_m']} m; not a "
                             "per-height voxelisation of the collision "
                             "triangles",
                "rule_volumes": "axis-aligned KEEP_OUT boxes extruded from "
                                "the prepared 2D recovery-case polygons to "
                                "declared height bands",
                "authority": "DEV in-process durable PREPARE/VOTE/COMMIT "
                             "envelope with one independent voter; the "
                             "production ROS manager service path is not used",
                "scope": "single-scene single-route DEV closed loop; not a "
                         "success-rate measurement, not real flight, not part "
                         "of a formal denominator",
            },
            "declared_height_bands": {
                "initial": self.rules["initial"].get("declared_band"),
                "replacement": self.rules["replacement"].get("declared_band"),
            },
            "controller": {
                "law": "position-error proportional velocity, capped speed",
                "position_gain": float(flight["position_gain"]),
                "lookahead_m": float(flight["lookahead_m"]),
                "planning_reserve_m": float(
                    flight.get("planning_reserve_m", 0.05)),
                "arrival_xy_tolerance_m": float(
                    flight["arrival_xy_tolerance_m"]),
            },
            "guard": {
                "rho_warn_m": float(flight["guard_rho_warn_m"]),
                "rho_release_m": float(flight["guard_rho_release_m"]),
                "epoch_discipline": "execution identity pinned at admission; "
                                    "authority epoch change fails closed",
            },
            "render_mode": self.render_mode,
            "ros_domain_id": int(self.args.ros_domain),
            "gazebo_partition": "rulesync_quad3d_" + self.root.name,
            "source_sha256": {str(path): sha256_file(path)
                              for path in self.source_files()},
            "runner_sha256": sha256_file(Path(__file__)),
            "wall_started": self.wall_started,
        }

    def source_files(self):
        base = Path(__file__).resolve().parent
        files = [Path(__file__).resolve()]
        for path in sorted((base / "quad3d").glob("*.py")):
            files.append(path)
        if getattr(self.args, "case", None):
            files.append(Path(self.args.case))
        return files

    def copy_sources(self):
        source = self.root / "source"
        source.mkdir(exist_ok=True)
        for path in self.source_files():
            shutil.copy2(path, source / path.name)

    # -- teardown --------------------------------------------------------
    def close(self):
        for process in reversed(self.processes):
            try:
                process.close()
            except Exception:
                pass
        if self.node is not None:
            try:
                import rclpy
                if rclpy.ok():
                    self.node.send(0.0, 0.0, 0.0)
                self.node.destroy_node()
                rclpy.try_shutdown()
            except Exception as exc:  # pragma: no cover - teardown
                self.result["cleanup_error"] = repr(exc)
        for stream in (self.events, self.trajectory, self.guard_events):
            if stream is not None:
                stream.close()


def run(args):
    if not 0 <= int(args.seed) < 2 ** 31:
        raise SystemExit("seed out of range")
    if int(args.ros_domain) not in (230, 231, 232):
        raise SystemExit("DEV runner reserves ROS domains 230-232")
    mission = Mission(args)
    try:
        mission.result["wall_started"] = mission.wall_started
        # the launcher pre-creates the attempt directory for launch.json and
        # the host log; the runner only requires that it is still fresh
        mission.root.mkdir(parents=True, exist_ok=True)
        assert not (mission.root / "events.jsonl").exists(), \
            "output directory is not fresh"
        assert not (mission.root / "result.json").exists(), \
            "output directory is not fresh"
        mission.setup()
        mission.copy_sources()
        if mission.stage == "platform_selftest":
            mission.run_platform_selftest()
        else:
            mission.load_case()
            mission.build_scene_world()
            mission.manifest = mission.build_manifest()
            write_json(mission.root / "manifest.json", mission.manifest)
            mission.event("world_written", runtime_world_sha256=sha256_file(
                mission.root / "world.sdf"), cameras=mission.cameras)
            env = mission.environment()
            mission.check_gpu(env)
            mission.open_authority()
            mission.start_platform(mission.root / "world.sdf",
                                   mission.cameras)
            mission.open_node()
            mission.capture = mission.capture_images()
            mission.takeoff()
            mission.phases["initial_commit"] = mission.run_initial_commit()
            mission.phases["initial_flight"] = mission.run_initial_flight()
            mission.phases["update_and_hold"] = mission.run_update()
            mission.phases["recovery"] = mission.run_recovery()
            mission.phases["arrival"] = mission.run_arrival()
            mission.audit()
            mission.result["result"] = "PASS"
    except (Exception, KeyboardInterrupt) as exc:
        mission.result["error"] = repr(exc)
        mission.result["traceback"] = traceback.format_exc()
    finally:
        mission.result["wall_finished"] = iso_now()
        mission.result["wall_elapsed_s"] = (
            time.monotonic() - mission.monotonic_started)
        if mission.authority is not None:
            mission.result["authority"] = mission.authority.snapshot()
            mission.result["authority_recovery"] = getattr(
                mission.authority, "recovery", None)
        try:
            mission.close()
        except Exception as exc:  # pragma: no cover - teardown
            mission.result["cleanup_error"] = repr(exc)
        mission.result["phase_timings"] = mission.phases
        for field in ("hold_record", "recovery", "arrival", "negative_control",
                      "audit_record", "initial_admission", "manifest",
                      "capture"):
            value = getattr(mission, field, None)
            key = "hold" if field == "hold_record" else (
                "audit" if field == "audit_record" else (
                    "images" if field == "capture" else field))
            if value is not None:
                mission.result[key] = value
        mission.result["case_id"] = (
            mission.case["id"] if mission.case else None)
        if mission.node is not None:
            mission.result["pose_messages"] = mission.node.pose_count
            mission.result["odom_messages"] = mission.node.odom_count
            mission.result["commands"] = mission.node.command_count
        write_json(mission.root / "result.json", mission.result)
        print(json.dumps({"result": mission.result["result"],
                          "phase": mission.result["phase"],
                          "case_id": mission.result.get("case_id"),
                          "error": mission.result.get("error")}), flush=True)
    return mission.result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--scene-id")
    parser.add_argument("--case", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=53001)
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--ros-domain", type=int, default=231)
    parser.add_argument("--render-mode", choices=["gpu", "software", "none"],
                        default="gpu")
    parser.add_argument("--target-rtf", type=float, default=1.0)
    parser.add_argument("--stage", choices=["smoke", "platform_selftest"],
                        default="smoke")
    options = parser.parse_args()
    if options.stage == "smoke":
        for name in ("scene", "scene_id", "case"):
            if getattr(options, name) in (None, ""):
                parser.error(f"--{name.replace('_', '-')} is required for "
                             "the smoke stage")
    else:
        options.scene = None
        options.case = None
        options.scene_id = None
    raise SystemExit(0 if run(options)["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
