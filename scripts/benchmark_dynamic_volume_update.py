#!/usr/bin/env python3
"""Production 3-D transaction/core monitor on the existing simulated gantry.

The new benchmark adapter polls volume authority and drives existing joints;
it is NOT the production 2-D RuleGuard ROS node nor a free-flight experiment.
Independent ECM base_link poses are logged for evaluation, never control.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import shutil
import sys
import time
import xml.etree.ElementTree as ET

ARMS = ("gate_only", "geometry_only", "full")
SPAWN = (1., 10., 2.)
GOAL = (6., 11., 3.5)
NEW_CENTER = (3.5, 10.5, 2.75)
DIMS = (1., 1., 1.5)
HALF_BODY = (.17, .17, .06)  # Existing base_collision, NOT visual arms.
RHO = .4
SNAPSHOT_STALE_S = 1.  # Independent cached-authority RPC, not blocked ListVolumes.
WORLD = "dynamic_volume"
TOPIC = "/benchmark/quad_link_pose"


def startup_ready(node):
    # Authority availability is not participant convergence, even at epoch 0.
    return (node.xyz is not None and len(node.worlds) > 20
            and node.authority is not None and node.ready
            and all(c.service_is_ready() for c in (node.propose, node.commit, node.bridge)))


def compose_pose(origin, rotation, point, quaternion):
    """Independent ECM world->model->link composition, xyzw quaternions."""
    x, y, z, w = rotation
    a, b, c, d = quaternion
    px, py, pz = point
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    xyz = [origin[0]+px+w*tx+y*tz-z*ty,
           origin[1]+py+w*ty+z*tx-x*tz,
           origin[2]+pz+w*tz+x*ty-y*tx]
    q = [w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
         w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c]
    return xyz, q


def emit(event, **fields):
    from benchmark_rule_churn import emit as write
    return write(event, **fields)


def json_write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def close_trial(node, ros, processes, folder, record):
    """Always clean owned child groups, including after ROS signal shutdown."""
    errors = []
    if node is not None:
        node.tracking = False
        try:
            if ros.ok():
                node.control()
        except Exception as exc:
            errors.append("stop publication: "+repr(exc))
        try:
            node.destroy_node()
        except Exception as exc:
            errors.append("node destroy: "+repr(exc))
    try:
        ros.try_shutdown()
    except Exception as exc:
        errors.append("ROS shutdown: "+repr(exc))
    try:
        processes.close()
    except Exception as exc:
        errors.append("child cleanup: "+repr(exc))
    if errors:
        record.update(result="FAIL", cleanup_errors=errors)
    json_write(folder/"result.json", record)


def volume_dict(message, extra=0.):
    c = (message.center.x, message.center.y, message.center.z)
    d = (message.dimensions.x, message.dimensions.y, message.dimensions.z)
    if message.geometry_type != 0:
        raise ValueError("this pilot accepts AABB only")
    return dict(geometry_type=0,
                bmin=[x-y/2-extra for x, y in zip(c, d)],
                bmax=[x+y/2+extra for x, y in zip(c, d)])


def body_bounds(row):
    qx, qy, qz, qw = row["quaternion"]
    roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
    pitch = math.asin(max(-1., min(1., 2*(qw*qy-qz*qx))))
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    if max(abs(roll), abs(pitch), abs(yaw)) > 1e-7:
        raise ValueError("axis-aligned body audit requires verified gantry attitude")
    return ([x-h for x, h in zip(row["xyz"], HALF_BODY)],
            [x+h for x, h in zip(row["xyz"], HALF_BODY)])


def box_distance(a, b):
    gaps = [max(x-v, u-y, 0.) for x, y, u, v in
            zip(a[0], a[1], b[0], b[1])]
    return math.sqrt(math.fsum(g*g for g in gaps))


def measured_stop(rows, commit_ns):
    onset = None
    for a, b in zip(rows, rows[1:]):
        dt = (b["sim_ns"]-a["sim_ns"])/1e9
        if not 0 < dt <= .1:
            raise ValueError("invalid independent pose gap")
        if a["receipt_ns"] < commit_ns or math.dist(a["xyz"], b["xyz"])/dt > .01:
            onset = None
        else:
            onset = onset or b
            if (b["sim_ns"]-onset["sim_ns"])/1e9 >= .2:
                return dict(onset_receipt_ns=onset["receipt_ns"],
                            confirmed_receipt_ns=b["receipt_ns"],
                            onset_sim_ns=onset["sim_ns"])
    return None


def first_zero_in_capture(events, commit_ns, final_sample_ns):
    """Exclude the unconditional cleanup zero sent after capture ends."""
    return next((e for e in events if e["event"] == "command"
                 and commit_ns <= e["t_ns"] <= final_sample_ns
                 and max(abs(x) for x in e["velocity"]) == 0.), None)


def evaluate(folder):
    from rulesync3d.ruleguard import snapshot_hash, volume_snapshot_items
    events = sorted((json.loads(s) for p in folder.glob("*_events.jsonl")
                     for s in p.read_text().splitlines() if s), key=lambda x: x["t_ns"])
    commits = [e for e in events if e["event"] == "durable_commit" and e["epoch"] == 2]
    if len(commits) != 1:
        raise ValueError("exactly one epoch-2 durable COMMIT required")
    t0, digest = commits[0]["t_ns"], commits[0]["snapshot_hash"]
    wal = [json.loads(s) for s in (folder/"wal.jsonl").read_text().splitlines()]
    target = [x for x in wal if x.get("decision") == "COMMIT" and x.get("epoch") == 2]
    if len(target) != 1 or target[0]["target_hash"] != digest:
        raise ValueError("durable target/COMMIT event mismatch")
    vols = target[0]["target_volumes"]
    if snapshot_hash(volume_snapshot_items(vols)) != digest or len(vols) != 1:
        raise ValueError("WAL canonical identity failed")
    c, d = vols[0]["center"], vols[0]["dimensions"]
    box = ([x-y/2 for x, y in zip(c, d)], [x+y/2 for x, y in zip(c, d)])
    worlds = [json.loads(s) for s in (folder/"world_pose.jsonl").read_text().splitlines()]
    active = [r for r in worlds if r["active"]]
    gaps = [(b["sim_ns"]-a["sim_ns"])/1e9 for a, b in zip(active, active[1:])]
    if len(active) < 20 or not gaps or not 0 < min(gaps) <= max(gaps) <= .1:
        raise ValueError("capture gap/sample contract failed")
    post = [r for r in active if r["receipt_ns"] >= t0]
    pre = [r for r in active if r["receipt_ns"] < t0]
    if len(post) < 20 or not pre:
        raise ValueError("COMMIT must be bracketed by independent samples")
    distances = [box_distance(body_bounds(r), box) for r in post]
    entries = [r for r, dist in zip(post, distances) if dist == 0.]
    holds = [e for e in events if e["event"] == "hold"]
    if any(e["t_ns"] < t0 for e in holds):
        raise ValueError("pre-COMMIT HOLD confounds revocation")
    hold = next(iter(holds), None)
    zero = first_zero_in_capture(events, t0, active[-1]["receipt_ns"])
    observed = next((e for e in events if e["event"] == "snapshot_observed"
                     and e["identity"] == [2, digest] and e["t_ns"] >= t0), None)
    stop = measured_stop(active, t0)
    ms = lambda ns: (ns-t0)/1e6 if ns is not None else None
    return dict(durable_identity=[2, digest], commit_ns=t0,
        commit_to_observed_ms=ms(observed["t_ns"] if observed else None),
        commit_to_hold_ms=ms(hold["t_ns"] if hold else None),
        hold_reason=hold["reason"] if hold else None,
        commit_to_zero_publish_ms=ms(zero["t_ns"] if zero else None),
        commit_to_world_low_speed_onset_ms=ms(stop["onset_receipt_ns"] if stop else None),
        commit_to_world_low_speed_confirmation_ms=ms(stop["confirmed_receipt_ns"] if stop else None),
        post_commit_samples=len(post), max_sim_gap_s=max(gaps),
        body_entry_observed=bool(entries), body_entry_sample_count=len(entries),
        first_body_entry_sim_ns=entries[0]["sim_ns"] if entries else None,
        min_sampled_body_clearance_m=min(distances),
        commit_receipt_bracket_ms=(post[0]["receipt_ns"]-pre[-1]["receipt_ns"])/1e6,
        active_xyz_range_m=[max(r["xyz"][i] for r in active)-min(r["xyz"][i] for r in active) for i in range(3)],
        travel_to_low_speed_m=math.fsum(math.dist(a["xyz"], b["xyz"])
            for a, b in zip(post, post[1:]) if b["sim_ns"] <= stop["onset_sim_ns"]) if stop else None)


def instrument_world(folder):
    root = ET.Element("sdf", version="1.9")
    world = ET.SubElement(root, "world", name=WORLD)
    physics = ET.SubElement(world, "physics", name="physics", type="dart")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    for file, name in (("physics", "Physics"), ("user-commands", "UserCommands")):
        ET.SubElement(world, "plugin", filename=f"gz-sim-{file}-system", name=f"gz::sim::systems::{name}")
    source = Path(__file__).resolve().parents[1]/"rulesync3d/worlds/r2i_quad.sdf"
    model = ET.parse(source).getroot().find("model")
    ET.SubElement(model, "pose").text = "1 10 2 0 0 0"
    # Sensors are not needed for this oracle-world authorization experiment.
    # Leave physical inertials and existing velocity controllers unchanged.
    for link in model.findall("link"):
        for sensor in link.findall("sensor"):
            link.remove(sensor)
    for plugin in model.findall("plugin"):
        if "PosePublisher" in plugin.get("name", ""):
            model.remove(plugin)
    pp = ET.SubElement(model, "plugin", filename="gz-sim-pose-publisher-system", name="gz::sim::systems::PosePublisher")
    for key, value in dict(publish_link_pose="true", publish_model_pose="true",
        use_pose_vector_msg="true", update_frequency="50", topic=TOPIC).items():
        ET.SubElement(pp, key).text = value
    world.append(model)
    target = folder/"world.sdf"
    ET.ElementTree(root).write(target, encoding="unicode", xml_declaration=True)
    return target


def role_main(role, ros_args):
    if role == "manager":
        from benchmark_rule_churn import run_role
        return run_role(role, ros_args)
    import rclpy
    import rulesync3d
    # Existing UAV bridge uses the source-tree nested namespace.
    sys.modules["rulesync3d.rulesync3d"] = rulesync3d
    from topopatch_ros.uav_bridge_node import UavBridge
    rclpy.init(args=ros_args)
    node = UavBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def trial_environment(folder, local_domain=None):
    env = os.environ.copy()
    env.update(ROS_DISCOVERY_SERVER="127.0.0.1:11811", ROS_DOMAIN_ID="0",
        GZ_PARTITION="rulesync_dynamic3d", RULESYNC_CHURN_FOLDER=str(folder),
        RULESYNC_CHURN_ROLE="runner")
    if local_domain is not None:
        if not 0 <= local_domain <= 232:
            raise ValueError("local domain must be in 0..232")
        env.pop("ROS_DISCOVERY_SERVER", None)
        env.update(ROS_DOMAIN_ID=str(local_domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
            FASTDDS_BUILTIN_TRANSPORTS="UDPv4", GZ_PARTITION=f"rulesync_edge3d_{local_domain}")
    return env


def run_trial(folder, arm, exposure, local_domain=None, task_recovery=False):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Float64
    from rulesync_interfaces.msg import RuleVolume, VolumeAuthority
    from rulesync_interfaces.srv import ProposeVolume, CommitVolume, GetVolumeAuthority, BridgeVolumeState
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rulesync3d.ruleguard import RuleGuard, snapshot_hash, volume_snapshot_items, HOLD
    from rulesync3d.gate_3d import gate_3d_path
    from rulesync3d.uav_supervisor import Supervisor
    from ruleguard_controller_loop_matrix import Processes
    env = trial_environment(folder, local_domain)
    if local_domain is not None:
        os.environ.pop("ROS_DISCOVERY_SERVER", None)
    os.environ.update(env)
    processes = Processes(folder, env)
    node = None
    record = dict(arm=arm, result="FAIL", phase="startup")

    class Probe(Node):
        def __init__(self):
            super().__init__("dynamic_volume_probe")
            self.xyz, self.pose_at, self.worlds, self.active = None, 0., [], False
            self.errors, self.authority, self.volumes, self.snapshot_at = [], None, [], 0.
            self.pending, self.tracking, self.held = None, False, False
            self.ready = False
            self.last_identity = None
            self.task_recovery = task_recovery
            self.guard, self.supervisor = RuleGuard(), Supervisor(.2, .2)
            self.cmd = [self.create_publisher(Float64, f"/r2i_quad/{j}/cmd_vel", 10) for j in ("jx", "jy", "jz")]
            self.create_subscription(JointState, "/r2i_quad/joint_state", self.on_js, 100)
            self.create_subscription(TFMessage, TOPIC, self.on_world, 100)
            self.propose = self.create_client(ProposeVolume, "/rulesync/propose_volume")
            self.commit = self.create_client(CommitVolume, "/rulesync/commit_volume")
            self.listing = self.create_client(GetVolumeAuthority, "/rulesync/get_volume_authority")
            self.create_subscription(VolumeAuthority, "/rulesync/volume_authority",
                lambda msg: self.observed_message(msg, "notice"),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            self.bridge = self.create_client(BridgeVolumeState, "/rulesync/uav_bridge_state")
            self.create_timer(.2, self.poll)
            self.create_timer(.02, self.control)

        def on_js(self, msg):
            if not all(j in msg.name for j in ("jx", "jy", "jz")):
                self.errors.append("missing joint")
                return
            positions = dict(zip(msg.name, msg.position))
            self.xyz = tuple(SPAWN[i]+positions[j] for i, j in enumerate(("jx", "jy", "jz")))
            self.pose_at = time.monotonic()
            emit("joint_input", xyz=self.xyz)

        def on_world(self, msg):
            try:
                selected = [t for t in msg.transforms if t.child_frame_id == "r2i_quad/base_link"]
                if len(selected) != 1:
                    raise ValueError(f"base link missing/ambiguous: {[(t.header.frame_id, t.child_frame_id) for t in msg.transforms]}")
                t = selected[0]
                if t.header.frame_id != "r2i_quad":
                    raise ValueError(f"unverified pose parent: {t.header.frame_id}")
                p, q = t.transform.translation, t.transform.rotation
                parents = [v for v in msg.transforms if v.child_frame_id == "r2i_quad"]
                if len(parents) != 1 or parents[0].header.frame_id != WORLD:
                    raise ValueError(f"unverified model parent: {[(v.header.frame_id,v.child_frame_id) for v in msg.transforms]}")
                parent = parents[0]
                o, r = parent.transform.translation, parent.transform.rotation
                xyz, quaternion = compose_pose([o.x,o.y,o.z], [r.x,r.y,r.z,r.w],
                                               [p.x,p.y,p.z], [q.x,q.y,q.z,q.w])
                row = dict(receipt_ns=time.monotonic_ns(), sim_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec,
                    xyz=xyz, quaternion=quaternion, raw_link_xyz=[p.x,p.y,p.z],
                    model_world_xyz=[o.x,o.y,o.z], model_world_quaternion=[r.x,r.y,r.z,r.w],
                    parent=parent.header.frame_id, intermediate=t.header.frame_id,
                    child=t.child_frame_id, active=self.active)
                if not all(math.isfinite(x) for x in row["xyz"]+row["quaternion"]) or abs(math.fsum(x*x for x in row["quaternion"])-1)>.001:
                    raise ValueError("invalid world pose")
                if self.worlds and row["sim_ns"] <= self.worlds[-1]["sim_ns"]:
                    raise ValueError("regressing/duplicate sim time")
                self.worlds.append(row)
                with (folder/"world_pose.jsonl").open("a") as stream:
                    stream.write(json.dumps(row)+"\n")
            except Exception as exc:
                self.errors.append(str(exc))

        def poll(self):
            if self.pending is None and self.listing.service_is_ready():
                self.pending = self.listing.call_async(GetVolumeAuthority.Request())
                self.pending.add_done_callback(self.observed)

        def observed(self, future):
            self.pending = None
            try:
                self.observed_message(future.result().authority, "rpc")
            except Exception as exc:
                emit("authority_rpc_error", reason=str(exc))

        def observed_message(self, response, channel):
            try:
                # Ignore delayed older responses without refreshing freshness.
                if self.last_identity and int(response.epoch) < self.last_identity[0]:
                    emit("authority_older_ignored", epoch=int(response.epoch), channel=channel)
                    return
                digest = snapshot_hash(volume_snapshot_items(response.volumes))
                if not response.available or digest != response.snapshot_hash:
                    raise ValueError("unavailable/hash-invalid authority")
                identity = [int(response.epoch), digest]
                if self.last_identity and identity[0] == self.last_identity[0] and identity != self.last_identity:
                    raise ValueError("same-epoch identity conflict")
                if len({v.rule_id for v in response.volumes}) != len(response.volumes):
                    raise ValueError("duplicate rule id")
                if any(v.frame_id != "map" or v.rule_type != 0 or v.severity != 1 for v in response.volumes):
                    raise ValueError("unexpected enforcement scope/frame")
                volumes = [volume_dict(v) for v in response.volumes]
                self.authority, self.snapshot_at = identity, time.monotonic()
                self.last_identity = identity
                self.ready, self.volumes = bool(response.ready), volumes
                self.guard.update_snapshot(*identity)
                emit("snapshot_observed", identity=identity, ready=self.ready, channel=channel)
            except Exception as exc:
                self.authority, self.ready = None, False
                self.guard.invalidate_snapshot()
                emit("snapshot_invalid", reason=str(exc), channel=channel)

        def control(self):
            velocity = (0., 0., 0.)
            if self.tracking and not self.held:
                now, reason = time.monotonic(), None
                if now-self.pose_at > .5:
                    reason = "pose-stale"
                elif arm != "gate_only":
                    if self.authority is None:
                        reason = "monitor-no-snapshot"
                    elif now-self.snapshot_at > SNAPSHOT_STALE_S:
                        reason = "monitor-snapshot-stale"
                    elif not self.ready and tuple(self.authority) == self.guard.execution_identity():
                        reason = "participant-not-ready"
                    else:
                        state, info = (self.guard.check_bound_3d(self.xyz, self.volumes, RHO)
                            if arm == "full" else self.guard.check_3d(self.xyz, self.volumes, RHO, *self.authority))
                        if state == HOLD:
                            reason = info.get("reason", "hold")
                if reason:
                    self.held = True
                    self.supervisor.set_mode("HOLD")
                    self.hold_at = emit("hold", reason=reason, xyz=self.xyz,
                         authority=self.authority, execution=self.guard.execution_identity())
                else:
                    path = getattr(self, "path", None)
                    target = GOAL
                    if path:
                        from volume_task_recovery import WAYPOINT_TOLERANCE
                        while self.path_index < len(path)-1 and math.dist(self.xyz, path[self.path_index]) <= WAYPOINT_TOLERANCE:
                            self.path_index += 1
                        target = path[self.path_index]
                        if self.path_index == len(path)-1 and math.dist(self.xyz, GOAL) <= WAYPOINT_TOLERANCE:
                            self.tracking = False
                            self.supervisor.set_mode("HOLD")
                            emit("goal_reached", xyz=self.xyz, goal=GOAL,
                                execution=self.guard.execution_identity(), path_digest=self.path_digest)
                    if self.tracking:
                        velocity = self.supervisor.track(self.xyz, target)
            for pub, value in zip(self.cmd, velocity):
                pub.publish(Float64(data=float(value)))
            if self.active:
                fields = dict(execution=self.guard.execution_identity(),
                    path_digest=getattr(self, "path_digest", None)) if getattr(self, "task_recovery", False) else {}
                emit("command", velocity=velocity, **fields)

    def until(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            processes.assert_alive()
            rclpy.spin_once(node, timeout_sec=.01)
            if node.errors:
                raise RuntimeError(node.errors[:3])
            if predicate():
                return True
        return False

    def call(client, request):
        future = client.call_async(request)
        if not until(future.done, 40):
            raise RuntimeError("service timeout")
        return future.result()

    def commit(epoch, center):
        v = RuleVolume(rule_id="dynamic-aabb", epoch=epoch, rule_type=0, severity=1, frame_id="map", geometry_type=0)
        v.center.x, v.center.y, v.center.z = center
        v.dimensions.x, v.dimensions.y, v.dimensions.z = DIMS
        v.valid_until.sec = 2_000_000_000
        response = call(node.propose, ProposeVolume.Request(volume=v))
        emit("propose_reply", epoch=epoch, accepted=response.accepted, reason=response.reason)
        if not response.accepted:
            raise RuntimeError(response.reason)
        response = call(node.commit, CommitVolume.Request(epoch=epoch))
        emit("commit_reply", epoch=epoch, committed=response.committed, reason=response.reason)
        if not response.committed:
            raise RuntimeError(response.reason)

    try:
        world = instrument_world(folder)
        command = [sys.executable, str(Path(__file__).resolve())]
        if local_domain is None:
            processes.start("discovery", ["fastdds", "discovery", "-i", "0", "-l", "127.0.0.1", "-p", "11811"], delay=1.)
        processes.start("sim", ["gz", "sim", "-r", "-s", str(world)], delay=2.)
        bridge_cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/r2i_quad/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"]
        bridge_cmd += [f"/r2i_quad/{j}/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double" for j in ("jx", "jy", "jz")]
        processes.start("gz_bridge", bridge_cmd)
        processes.start("participant", command+["--role", "bridge", "--ros-args", "-p", "world_boxes:='[]'", "-p", "spawn_xyz:=[1.0,10.0,2.0]"])
        processes.start("manager", command+["--role", "manager", "--ros-args", "-p", "participants:=[uav_bridge]",
            "-p", f"state_path:={folder}/state.json", "-p", f"wal_path:={folder}/wal.jsonl", "-p", f"txn_log_path:={folder}/txn.log"])
        rclpy.init()
        node = Probe()
        if not until(lambda: startup_ready(node), 50):
            raise RuntimeError("startup service/authority/world capture timeout")
        if not until(lambda: False, .5):
            pass
        alignment = math.dist(node.xyz, node.worlds[-1]["xyz"])
        if alignment > .02 or time.monotonic()-node.pose_at > .25:
            raise RuntimeError(f"joint/world coordinate mismatch: {alignment}")
        emit("stationary_alignment", error_m=alignment, frames=[node.worlds[-1]["parent"], node.worlds[-1]["child"]])
        record["phase"] = "initial_commit"
        commit(1, (12., 12., 2.))
        if not until(lambda: node.authority and node.authority[0] == 1 and node.ready, 8):
            raise RuntimeError("initial snapshot unobserved")
        applied = call(node.bridge, BridgeVolumeState.Request())
        if [applied.active_epoch, applied.active_hash] != node.authority:
            raise RuntimeError("initial manager/participant mismatch")
        allowed, reason = gate_3d_path([node.xyz, GOAL], [volume_dict(v, RHO) for v in applied.active_volumes],
                                      lambda x,y,z: False, (0., 0., .2, 20., 20., 6.))
        if not allowed or not node.guard.bind_execution(*node.authority):
            raise RuntimeError(f"initial gate/binding failed: {reason}")
        emit("initial_admission", identity=node.authority, xyz=node.xyz, goal=GOAL, gate_reason=reason)
        node.active, node.tracking = True, True
        node.supervisor.set_mode("TRACK")
        record["phase"] = "in_flight"
        if not until(lambda: node.xyz[0] >= 1.4, 10):
            raise RuntimeError("update trigger not reached")
        record["phase"] = "update"
        emit("update_request", xyz=node.xyz)
        commit(2, NEW_CENTER)
        record["phase"] = "capture"
        end_sim = node.worlds[-1]["sim_ns"]+int(exposure*1e9)
        if task_recovery:
            from volume_task_recovery import propose_detour, release_checks, path_digest, audit_task
            record["phase"] = "stop_before_recovery"
            if not until(lambda: node.held and measured_stop(node.worlds, node.hold_at) is not None, 20):
                raise RuntimeError("confirmed pre-recovery stop absent")
            emit("stopped_before_recovery", measurement=measured_stop(node.worlds, node.hold_at))
            if not until(lambda: node.authority and node.authority[0]==2 and node.ready, 8):
                raise RuntimeError("new committed participant convergence absent")
            expected = list(node.authority)
            applied = call(node.bridge, BridgeVolumeState.Request())
            participant = [applied.active_epoch, applied.active_hash]
            if participant != expected:
                raise RuntimeError("new manager/participant identity mismatch")
            volumes = [volume_dict(v) for v in applied.active_volumes]
            old_allowed, old_reason = gate_3d_path([node.xyz, GOAL],
                [volume_dict(v,RHO) for v in applied.active_volumes],
                lambda x,y,z: False, (0.,0.,.2,20.,20.,6.))
            emit("old_route_recheck", identity=expected, allowed=old_allowed, reason=old_reason)
            if old_allowed:
                raise RuntimeError("recovery scene no longer requires a detour")
            record["phase"] = "propose_detour"
            started = time.monotonic_ns()
            path, reason = propose_detour(node.xyz, GOAL, volumes, RHO)
            emit("recovery_proposal", path=path, identity=expected,
                planner="existing-voxel-astar+shortcut", elapsed_ms=(time.monotonic_ns()-started)/1e6)
            # Refresh after planning, then re-gate from the latest joint pose.
            if not until(lambda: node.authority==expected and node.ready
                    and time.monotonic()-node.pose_at <= .25
                    and time.monotonic()-node.snapshot_at <= .25, 8):
                raise RuntimeError("fresh authority/pose absent after planning")
            applied = call(node.bridge, BridgeVolumeState.Request())
            participant = [applied.active_epoch, applied.active_hash]
            if snapshot_hash(volume_snapshot_items(applied.active_volumes)) != expected[1]:
                raise RuntimeError("participant payload hash mismatch before release")
            path[0] = tuple(node.xyz)
            pose_age = time.monotonic()-node.pose_at
            snapshot_age = time.monotonic()-node.snapshot_at
            reason = release_checks(expected, node.authority, participant, node.ready,
                pose_age, snapshot_age, path, node.xyz, GOAL,
                [volume_dict(v) for v in applied.active_volumes], RHO)
            if not node.guard.bind_execution(*expected):
                raise RuntimeError("new execution binding refused")
            node.path, node.path_index, node.path_digest = path, 1, path_digest(path)
            emit("recovery_release", identity=expected, participant=participant,
                execution=node.guard.execution_identity(), ready=node.ready, path=path,
                path_digest=node.path_digest, xyz=node.xyz, rho=RHO,
                pose_age_s=pose_age, snapshot_age_s=snapshot_age, gate_reason=reason)
            node.held = False
            node.supervisor.set_mode("TRACK")
            record["phase"] = "resumed_task"
            if not until(lambda: not node.tracking or node.held
                    or node.worlds[-1]["sim_ns"] >= end_sim, exposure*4+5):
                raise RuntimeError("recovered task wall budget exceeded")
            if node.held or node.tracking:
                raise RuntimeError("recovered task did not reach goal before budget")
            confirm_sim = node.worlds[-1]["sim_ns"]+500_000_000
            if not until(lambda: node.worlds[-1]["sim_ns"]>=confirm_sim, 8):
                raise RuntimeError("independent arrival capture absent")
        else:
            if not until(lambda: node.worlds[-1]["sim_ns"] >= end_sim, exposure*4+5):
                raise RuntimeError("post-update sim exposure timeout")
        if time.monotonic_ns()-node.worlds[-1]["receipt_ns"] > 250_000_000:
            raise RuntimeError("independent world capture stale")
        # Offline audits may take longer than the freshness bound. Validate
        # acquisition BEFORE audit, never spin to hide a genuinely stale capture.
        if task_recovery:
            record["task_metrics"] = audit_task(folder)
        record.update(result="PASS", metrics=evaluate(folder))
    except (Exception, KeyboardInterrupt) as exc:
        record["error"] = repr(exc)
    finally:
        close_trial(node, rclpy, processes, folder, record)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("manager", "bridge"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--audit", type=Path)
    parser.add_argument("--arms", choices=ARMS, nargs="+", default=list(ARMS))
    parser.add_argument("--exposure", type=float, default=12.)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--recover-task", action="store_true",
        help="full-arm stopped motion, independently admitted voxel-A* detour, then arrival")
    parser.add_argument("--local-domain", type=int,
        help="board-local UDP discovery, no external discovery server; distinct deployment profile")
    args, ros_args = parser.parse_known_args()
    if args.role:
        return role_main(args.role, ros_args)
    if args.audit:
        frozen = json.loads((args.audit/"manifest.json").read_text())["source_sha256"]
        source_ok = all(hashlib.sha256((args.audit/"source"/p).read_bytes()).hexdigest()==h for p,h in frozen.items())
        comparisons = []
        for folder in sorted(args.audit.glob("trial_*")):
            saved = json.loads((folder/"result.json").read_text())
            try:
                actual = evaluate(folder)
                task = None
                if "task_metrics" in saved:
                    from volume_task_recovery import audit_task
                    task = audit_task(folder)
                comparisons.append(dict(trial=folder.name, match=saved.get("metrics")==actual
                    and (task is None or task==saved["task_metrics"])))
            except Exception as exc:
                comparisons.append(dict(trial=folder.name, match=False, error=repr(exc)))
        audit_source = args.audit/"posthoc_source"/Path(__file__).name
        audit_source.parent.mkdir(exist_ok=True)
        shutil.copy2(Path(__file__), audit_source)
        result = dict(audit_passed=source_ok and bool(comparisons) and all(x["match"] for x in comparisons),
            frozen_source_integrity=source_ok, trials=comparisons,
            audit_source_sha256=hashlib.sha256(audit_source.read_bytes()).hexdigest(),
            evaluation_note="cleanup zero after final active world sample excluded; original result/raw/frozen source unchanged")
        json_write(args.audit/"recomputed_audit.json", result)
        print(json.dumps(result))
        return int(not result["audit_passed"])
    exposure_ok = (60 <= args.exposure <= 80 and args.arms==["full"]) if args.recover_task else 10 <= args.exposure <= 15
    if not args.output or ros_args or not exposure_ok or args.repetitions < 1:
        parser.error("fresh output; stop-only exposure 10..15, recovery full-only exposure 60..80 required")
    if args.local_domain is not None and not 0 <= args.local_domain <= 232:
        parser.error("local domain must be in 0..232")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    sources = ["scripts/benchmark_dynamic_volume_update.py", "scripts/test_dynamic_volume_update.py", "scripts/benchmark_rule_churn.py",
        "scripts/volume_task_recovery.py", "scripts/test_volume_task_recovery.py",
        "scripts/ruleguard_controller_loop_matrix.py", "rulesync3d/worlds/r2i_quad.sdf", "rulesync/vote_volume_once.py",
        "rulesync/src/rulesync_manager/rulesync_manager/manager_node.py",
        "rulesync/src/rulesync_manager/rulesync_manager/volume_authority.py",
        "rulesync/src/rulesync_manager/test/test_volume_authority.py",
        "scripts/verify_volume_authority_ros.py",
        "rulesync/src/rulesync_interfaces/msg/VolumeAuthority.msg",
        "rulesync/src/rulesync_interfaces/srv/GetVolumeAuthority.srv",
        "rulesync/src/rulesync_interfaces/CMakeLists.txt",
        "rulesync/src/rulesync_manager/rulesync_manager/snapshot_identity.py",
        "rulesync/src/rulesync_manager/rulesync_manager/validator.py",
        "rulesync/src/rulesync_manager/rulesync_manager/vote_coordinator.py",
        "rulesync/src/rulesync_manager/rulesync_manager/volume_vote_coordinator.py",
        "rulesync/vote_volume_once.py", "scripts/launch_edge_volume_physics.py",
        "topopatch_ros/topopatch_ros/uav_bridge_node.py",
        "rulesync3d/rulesync3d/gate_3d.py", "rulesync3d/rulesync3d/ruleguard.py",
        "rulesync3d/rulesync3d/uav_supervisor.py", "rulesync3d/rulesync3d/planners_3d.py",
        "rulesync3d/rulesync3d/worlds_3d.py"]
    hashes = {}
    for relative in sources:
        target = args.output/"source"/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root/relative, target)
        hashes[relative] = hashlib.sha256(target.read_bytes()).hexdigest()
    installed = {}
    for module, relative in {
        "rulesync_manager.manager_node": "rulesync/src/rulesync_manager/rulesync_manager/manager_node.py",
        "rulesync_manager.volume_authority": "rulesync/src/rulesync_manager/rulesync_manager/volume_authority.py",
        "rulesync_manager.vote_coordinator": "rulesync/src/rulesync_manager/rulesync_manager/vote_coordinator.py",
        "rulesync_manager.volume_vote_coordinator": "rulesync/src/rulesync_manager/rulesync_manager/volume_vote_coordinator.py",
        "topopatch_ros.uav_bridge_node": "topopatch_ros/topopatch_ros/uav_bridge_node.py",
        **{f"rulesync3d.{name}": f"rulesync3d/rulesync3d/{name}.py" for name in
           ("gate_3d", "ruleguard", "uav_supervisor", "planners_3d", "worlds_3d")},
    }.items():
        origin = Path(importlib.util.find_spec(module).origin)
        digest = hashlib.sha256(origin.read_bytes()).hexdigest()
        if digest != hashes[relative]:
            raise RuntimeError(f"installed/source mismatch: {module}")
        installed[module] = dict(path=str(origin), sha256=digest)
    schedule = [dict(arm=a, repetition=r) for r in range(1, args.repetitions+1) for a in args.arms]
    random.Random(20261010).shuffle(schedule)
    json_write(args.output/"manifest.json", dict(cohort=("EdgeVolume-3D-task-recovery" if args.recover_task else "EdgeVolume-3D-physics-attempt" if args.local_domain is not None
        else "DynamicVolume-3D-durable-authority"), schedule=schedule,
        local_domain=args.local_domain, cpu_affinity=sorted(os.sched_getaffinity(0)),
        platform=sys.platform, ros_distro=os.environ.get("ROS_DISTRO"),
        volume_vote_helper_override=os.environ.get("RULESYNC_VOLUME_VOTE_HELPER"),
        volume_vote_mode=os.environ.get("RULESYNC_VOLUME_VOTE_MODE", "helper"),
        exposure_sim_s=args.exposure, source_sha256=hashes, installed_modules=installed, pre_run_wall_ns=time.time_ns(),
        spawn=SPAWN, goal=GOAL, new_center=NEW_CENTER, dimensions=DIMS, body_half_extents=HALF_BODY,
        fixed=dict(speed_xy=.2, speed_z=.2, rho=RHO, trigger_x=1.4, poll_s=.2, snapshot_stale_s=SNAPSHOT_STALE_S, control_s=.02),
        ablation="same durable geometry/transport/freshness; initial admission requires READY; geometry_only omits in-flight identity mismatch (new authority may be visible before ACK); lost readiness of pinned identity holds both",
        scope="AABB only; gantry joints, no free-flight; independent ECM base_collision envelope samples; not continuous certification",
        adapter="typed durable notice + independent cached RPC fallback, stop latch; existing transaction/core gate/RuleGuard/Supervisor reused",
        task_recovery=args.recover_task,
        recovery_policy="confirmed stop; existing voxel A* at 0.20 m grid / rho+0.15 m proposer clearance; continuous gate at rho; fresh epoch/hash convergence before rebind; independent world-pose arrival" if args.recover_task else None,
        policy="all attempts retained; PASS is capture completion, not safety certification"))
    records = []
    for i, trial in enumerate(schedule, 1):
        folder = args.output/f"trial_{i:02d}"
        folder.mkdir()
        record = run_trial(folder, trial["arm"], args.exposure, args.local_domain, args.recover_task)
        record["repetition"] = trial["repetition"]
        json_write(folder/"result.json", record)
        records.append(record)
        with (args.output/"trials.jsonl").open("a") as stream:
            stream.write(json.dumps(record)+"\n")
        print(json.dumps(record), flush=True)
        if record["result"] != "PASS":
            break  # Preserve failed preflight; do not multiply invalid trials.
    unchanged = all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    json_write(args.output/"completion.json", dict(planned=len(schedule), attempted=len(records), failures=sum(r["result"]!="PASS" for r in records), source_unchanged=unchanged))
    return int(not unchanged or len(records) != len(schedule) or any(r["result"]!="PASS" for r in records))


if __name__ == "__main__":
    sys.exit(main())
