#!/usr/bin/env python3
"""Live ROS protocol checks, not closed-loop/hardware navigation evidence.

Faults are scoped to this test's manager wrapper and epoch 2. Actual production
WAL/fsync, certificate checks, volume helper and participant are reused.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def role(args, ros_args):
    if args.role == "bridge":
        from benchmark_dynamic_volume_update import role_main
        return role_main("bridge", ros_args)
    import rclpy
    if args.role == "read":
        from rclpy.node import Node
        from rulesync_interfaces.srv import GetVolumeAuthority, BridgeVolumeState
        from rulesync3d.ruleguard import snapshot_hash, volume_snapshot_items
        rclpy.init()
        node = Node("volume_authority_fresh_process_probe")
        def query(service, path):
            client = node.create_client(service, path)
            if not client.wait_for_service(timeout_sec=12):
                raise TimeoutError(f"fresh process discovery: {path}")
            future = client.call_async(service.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=12)
            if not future.done():
                raise TimeoutError(f"fresh process response: {path}")
            return future.result()
        try:
            a = query(GetVolumeAuthority, "/rulesync/get_volume_authority").authority
            b = query(BridgeVolumeState, "/rulesync/uav_bridge_state")
            print(json.dumps(dict(epoch=int(a.epoch), hash=a.snapshot_hash,
                available=bool(a.available), ready=bool(a.ready), count=len(a.volumes),
                hash_valid=snapshot_hash(volume_snapshot_items(a.volumes)) == a.snapshot_hash,
                bridge_epoch=int(b.active_epoch), bridge_hash=b.active_hash)), flush=True)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
        return
    from rclpy.executors import MultiThreadedExecutor
    from rulesync_manager.manager_node import RuleManager
    rclpy.init(args=ros_args)

    class FaultManager(RuleManager):
        def _vol_crash_hook(self, point, txn_id="", epoch=0):
            if epoch == 2 and point == args.fault:
                self.log_txn(op="authority-test-crash", point=point, epoch=epoch)
                os._exit(1)

        def _volume_round_body(self, vols, epoch, op, timeout, txn_id="", target_hash=""):
            if epoch == 2 and op == "commit" and args.fault in ("lost-ack", "notice-loss"):
                time.sleep(2.)  # Verify cached RPC while APPLY is still pending.
            result = super()._volume_round_body(vols, epoch, op, timeout, txn_id, target_hash)
            if epoch == 2 and op == "commit" and args.fault == "lost-ack":
                return {}  # Participant really applied; response is discarded.
            return result

        def _publish_volume_authority(self):
            if args.fault != "notice-loss":
                return super()._publish_volume_authority()

    node = FaultManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def trial(folder, fault, operation):
    import rclpy
    from rclpy.node import Node
    from rulesync_interfaces.msg import RuleVolume
    from rulesync_interfaces.srv import (ProposeVolume, CommitVolume, RemoveVolume,
                                        GetVolumeAuthority, BridgeVolumeState)
    from rulesync3d.ruleguard import snapshot_hash, volume_snapshot_items
    from ruleguard_controller_loop_matrix import Processes
    env = os.environ.copy()
    env.update(ROS_DISCOVERY_SERVER="127.0.0.1:11811", ROS_DOMAIN_ID="0",
               RULESYNC_TEST_POSE="1,10,2")
    os.environ.update(env)
    processes = Processes(folder, env)
    node = None
    result = dict(fault=fault, operation=operation, result="FAIL", phase="startup", samples=[])
    command = [sys.executable, str(Path(__file__).resolve())]

    def start_manager(name, injection):
        return processes.start(name, command + ["--role", "manager", "--fault", injection,
            "--ros-args", "-p", "participants:=[uav_bridge]", "-p", f"state_path:={folder}/state.json",
            "-p", f"wal_path:={folder}/wal.jsonl", "-p", f"txn_log_path:={folder}/txn.log"])

    def until(predicate, timeout=40):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return True
        raise TimeoutError("protocol condition timeout")

    def call(client, request):
        result["waiting_for"] = "discovery:"+client.srv_name
        until(client.service_is_ready)
        future = client.call_async(request)
        result["waiting_for"] = "response:"+client.srv_name
        until(future.done)
        return future.result()

    def authority():
        a = call(clients["authority"], GetVolumeAuthority.Request()).authority
        row = dict(t_ns=time.monotonic_ns(), epoch=int(a.epoch), hash=a.snapshot_hash,
                   available=bool(a.available), ready=bool(a.ready), count=len(a.volumes))
        result["samples"].append(row)
        assert a.available and snapshot_hash(volume_snapshot_items(a.volumes)) == a.snapshot_hash
        return row

    def stop_one(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)

    try:
        processes.start("discovery", ["fastdds", "discovery", "-i", "0", "-l", "127.0.0.1", "-p", "11811"], delay=1)
        processes.start("participant", command + ["--role", "bridge", "--ros-args",
                        "-p", "world_boxes:='[]'", "-p", "spawn_xyz:=[1.0,10.0,2.0]"])
        manager = start_manager("manager_initial", "none")
        rclpy.init()
        node = Node("volume_authority_protocol_probe")
        clients = {key: node.create_client(service, path) for key, service, path in (
            ("authority", GetVolumeAuthority, "/rulesync/get_volume_authority"),
            ("propose", ProposeVolume, "/rulesync/propose_volume"),
            ("commit", CommitVolume, "/rulesync/commit_volume"),
            ("remove", RemoveVolume, "/rulesync/remove_volume"),
            ("bridge", BridgeVolumeState, "/rulesync/uav_bridge_state"))}

        def fresh_manager_clients():
            nonlocal node, clients
            # A crashed server can leave unresolved futures/discovery state
            # in this probe's DDS participant. Reconnect the entire probe,
            # not just a service handle. The enforcer process remains live.
            node.destroy_node()
            rclpy.try_shutdown()
            rclpy.init()
            node = Node("volume_authority_protocol_probe_reconnected")
            clients = {}
            for key, service, path in (
                ("authority", GetVolumeAuthority, "/rulesync/get_volume_authority"),
                ("propose", ProposeVolume, "/rulesync/propose_volume"),
                ("commit", CommitVolume, "/rulesync/commit_volume"),
                ("remove", RemoveVolume, "/rulesync/remove_volume"),
                ("bridge", BridgeVolumeState, "/rulesync/uav_bridge_state")):
                clients[key] = node.create_client(service, path)

        until(lambda: authority()["ready"])

        def propose(epoch, size):
            v = RuleVolume(rule_id="a", epoch=epoch, rule_type=0, severity=1, frame_id="map", geometry_type=0)
            v.center.x, v.center.y, v.center.z = 4., 10., 2.
            v.dimensions.x, v.dimensions.y, v.dimensions.z = size, size, size
            v.valid_until.sec = 2_000_000_000
            response = call(clients["propose"], ProposeVolume.Request(volume=v))
            assert response.accepted, response.reason

        propose(1, 2.)
        assert call(clients["commit"], CommitVolume.Request(epoch=1)).committed
        baseline = authority()
        assert baseline["epoch"] == 1 and baseline["ready"]
        stop_one(manager)
        result["phase"] = "fault_setup"
        manager = start_manager("manager_fault", fault)
        fresh_manager_clients()
        until(lambda: authority()["ready"])
        result["phase"] = "fault_injection"
        if operation == "RELAX":
            propose(2, 1.)
            future = clients["commit"].call_async(CommitVolume.Request(epoch=2))
        else:
            future = clients["remove"].call_async(RemoveVolume.Request(rule_id="a", epoch=2))
        if fault in ("lost-ack", "notice-loss"):
            until(lambda: authority()["epoch"] == 2)
            pending = result["samples"][-1]
            assert not pending["ready"] and not future.done(), "published authority mistaken for release readiness"
            until(future.done)
            reply = future.result()
            assert reply.committed if operation == "RELAX" else reply.ok
            after = authority()
            assert after["ready"] == (fault != "lost-ack")
            result["rpc_during_pending_apply"] = True
            stop_one(manager)
        else:
            until(lambda: manager.poll() is not None)
            assert manager.returncode == 1
        # Lost notifications and old checkpoint cannot override retained WAL.
        result["phase"] = "recovery"
        manager = start_manager("manager_recovery", "none")
        expected_epoch = 1 if fault == "pre-wal" else 2
        # Separate-process read probes also isolate a crashed server's unresolved
        # futures from protocol correctness. Record every failed read attempt.
        result["recovery_reads"] = []
        deadline = time.monotonic()+60
        recovered = None
        while time.monotonic() < deadline:
            proc = subprocess.run(command+["--role", "read"], env=env,
                capture_output=True, text=True, timeout=30)
            result["recovery_reads"].append(dict(returncode=proc.returncode, stdout=proc.stdout, stderr=proc.stderr))
            if proc.returncode == 0:
                a = json.loads(proc.stdout.strip().splitlines()[-1])
                if a["epoch"] == expected_epoch and a["ready"]:
                    recovered = a
                    break
            time.sleep(.2)
        assert recovered is not None, "fresh-process recovery read failed"
        assert recovered["available"] and recovered["hash_valid"]
        assert [recovered["bridge_epoch"], recovered["bridge_hash"]] == [recovered["epoch"], recovered["hash"]]
        assert recovered["count"] == (0 if operation == "REMOVE" and expected_epoch == 2 else 1)
        decisions = [json.loads(s) for s in (folder/"wal.jsonl").read_text().splitlines()]
        committed = [r for r in decisions if r.get("epoch") == 2 and r.get("decision") == "COMMIT"]
        assert bool(committed) == (fault != "pre-wal")
        if committed:
            assert committed[-1]["target_hash"] == recovered["hash"]
        result.update(result="PASS", phase="complete", recovered=recovered, participant_epoch=recovered["bridge_epoch"])
    except Exception as exc:
        result["error"] = repr(exc)
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
        processes.close()
        write(folder/"result.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("manager", "bridge", "read"))
    parser.add_argument("--fault", default="none")
    parser.add_argument("--output", type=Path)
    args, ros_args = parser.parse_known_args()
    if args.role:
        return role(args, ros_args)
    if not args.output or ros_args:
        parser.error("fresh output required")
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    paths = ["scripts/verify_volume_authority_ros.py", "scripts/benchmark_dynamic_volume_update.py",
             "rulesync/src/rulesync_manager/rulesync_manager/manager_node.py",
             "rulesync/src/rulesync_manager/rulesync_manager/volume_authority.py",
             "rulesync/src/rulesync_interfaces/msg/VolumeAuthority.msg",
             "rulesync/src/rulesync_interfaces/srv/GetVolumeAuthority.srv",
             "topopatch_ros/topopatch_ros/uav_bridge_node.py", "rulesync/vote_volume_once.py"]
    hashes = {}
    for path in paths:
        target = args.output/"source"/path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root/path, target)
        hashes[path] = hashlib.sha256(target.read_bytes()).hexdigest()
    schedule = [(fault, op) for op in ("RELAX", "REMOVE")
                for fault in ("pre-wal", "post-wal", "post-dispatch", "lost-ack", "notice-loss")]
    write(args.output/"manifest.json", dict(scope="ROS protocol fault tests, not navigation experiments", schedule=schedule, source_sha256=hashes))
    rows = []
    for index, (fault, op) in enumerate(schedule, 1):
        folder = args.output/f"trial_{index:02d}"
        folder.mkdir()
        result = trial(folder, fault, op)
        rows.append(result)
        print(json.dumps({k: v for k, v in result.items() if k != "samples"}), flush=True)
        if result["result"] != "PASS":
            break  # Preserve failure; fix before collecting formal evidence.
    unchanged = all(hashlib.sha256((root/p).read_bytes()).hexdigest() == h for p,h in hashes.items())
    passed = len(rows) == len(schedule) and unchanged and all(r["result"] == "PASS" for r in rows)
    write(args.output/"completion.json", dict(passed=passed, attempted=len(rows), source_unchanged=unchanged))
    return int(not passed)


if __name__ == "__main__":
    sys.exit(main())
