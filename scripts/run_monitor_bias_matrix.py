#!/usr/bin/env python3
"""Freeze and run 30 targeted monitor-bias trials, sequentially, without retries.

The existing production monitor/controller are unchanged. Only the test pose
publication is offset. Independent Gazebo model-point pose is evaluation-only.
No successful-outcome filtering: failed runs remain in the cohort denominator.
"""
import argparse
import hashlib
import json
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time

SEED = 20261010
OFFSETS = (-.10, -.05, 0.0, .05, .10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=197)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    scripts = Path(__file__).resolve().parent
    workspace = scripts.parent
    trials = [{"regime": regime, "offset_m": offset, "repetition": rep,
               "angular_z": angular}
              for regime, angular in (("nominal", 0.0), ("strong", .25))
              for offset in OFFSETS for rep in range(1, 4)]
    random.Random(SEED).shuffle(trials)
    source_paths = [
        "scripts/run_monitor_bias_matrix.py", "scripts/gazebo_world_pose.py",
        "scripts/ruleguard_controller_loop_matrix.py", "scripts/test_gazebo_world_pose.py",
        "rulesync3d/rulesync3d/ruleguard.py", "rulesync3d/rulesync3d/gate2d.py",
        "rulesync/src/rulesync_guard/rulesync_guard/guard_node.py",
        "rulepatch_ros/rulepatch_ros/pp_driver.py",
        "topopatch_ros/topopatch_ros/gate_node.py",
        "rulesync/src/rulesync_manager/rulesync_manager/manager_node.py",
        "rulesync/src/rulesync_nav2_bridge/rulesync_nav2_bridge/bridge_node.py",
    ]
    hashes = {}
    for relative in source_paths:
        source = workspace / relative
        target = args.output / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        hashes[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    # Confirm production imports are the deployment evaluated, not silently
    # modified installation copies. Run from /tmp to avoid namespace shadowing.
    import importlib.util
    modules = {
        "rulesync_guard.guard_node": source_paths[6],
        "rulepatch_ros.pp_driver": source_paths[7],
        "topopatch_ros.gate_node": source_paths[8],
        "rulesync_manager.manager_node": source_paths[9],
        "rulesync_nav2_bridge.bridge_node": source_paths[10],
        "rulesync3d.ruleguard": source_paths[4], "rulesync3d.gate2d": source_paths[5],
    }
    installed = {}
    for module, relative in modules.items():
        deployed = Path(importlib.util.find_spec(module).origin)
        digest = hashlib.sha256(deployed.read_bytes()).hexdigest()
        if digest != hashes[relative]:
            raise RuntimeError(f"installed/source SHA mismatch: {module}")
        installed[module] = {"path": str(deployed), "sha256": digest}
    manifest = {
        "cohort": "MonitorBias-30", "pre_run_wall_ns": time.time_ns(),
        "seed": SEED, "trials": trials, "source_sha256": hashes,
        "installed_modules": installed, "domain_id": args.domain_id,
        "fixed": {"arm": "gate_guard", "speed_mps": .25, "rho_m": .20,
                  "lookahead_m": .50, "index_enabled": False,
                  "world_pose_requested_hz": 50, "run_timeout_s": 25,
                  "snapshot_poll_s": .20, "snapshot_stale_s": 1.0,
                  "snapshot_request_timeout_s": 1.0, "pose_stale_s": .50,
                  "rho_warn_m": .50},
        "scope": "constant bias on monitor-only odometry-derived input, not full SLAM noise",
        "policy": "sequential; retain all 30 attempts; no retry; no post-hoc cell selection",
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    records = []
    for index, trial in enumerate(trials, 1):
        folder = args.output / f"trial_{index:02d}"
        folder.mkdir()
        command = [sys.executable, str(scripts / "ruleguard_controller_loop_matrix.py"),
                   "--arm", "gate_guard", "--domain-id", str(args.domain_id),
                   "--world-truth", "--monitor-y-offset", str(trial["offset_m"]),
                   "--bias-angular-z", str(trial["angular_z"]),
                   "--label", f"monitor_bias_{index:02d}", "--log-root", str(folder)]
        with open(folder / "driver.log", "w", encoding="utf-8") as stream:
            completed = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
        paths = list(folder.glob("gate_guard_*/result.json"))
        record = {"index": index, **trial, "returncode": completed.returncode,
                  "result_path": str(paths[0]) if len(paths) == 1 else None}
        if len(paths) == 1:
            result = json.loads(paths[0].read_text())
            world = result.get("world_truth", {})
            record.update(result=result["result"], error=result.get("error"),
                          arrived=world.get("sampled_arrival"),
                          original_entry=world.get("sampled_original_region_entry"),
                          min_world_clearance_m=world.get("min_inflated_clearance_m"),
                          boundary_hold=result.get("hold_observed"))
        else:
            record.update(result="FAIL", error="missing or ambiguous result file")
        records.append(record)
        with open(args.output / "trials.jsonl", "a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")
        print(json.dumps(record), flush=True)
    failures = sum(row["result"] != "PASS" or row["returncode"] != 0 for row in records)
    (args.output / "completion.json").write_text(json.dumps({
        "attempted": len(records), "failed_capture_or_terminal": failures,
        "completed_wall_ns": time.time_ns(),
    }, indent=2) + "\n")
    return int(failures != 0)


if __name__ == "__main__":
    sys.exit(main())
