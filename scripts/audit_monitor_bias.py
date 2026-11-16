#!/usr/bin/env python3
"""Recompute MonitorBias-30 outcomes from frozen raw streams (standard library).

Rectangle signed distance is independently evaluated, without importing the
production polygon/monitor code. This checks recorded observations, not ROS
message delivery, swept volumes, or unobserved continuous-time excursions.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import random
import statistics


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def rectangle_clearance(x, y, rho):
    dx, dy = max(-.8 - x, 0, x - .8), max(.25 - y, 0, y - .65)
    distance = math.hypot(dx, dy) if dx or dy else -min(x + .8, .8 - x, y - .25, .65 - y)
    return distance - rho


def sustained_world_stop(active, hold):
    """Sampled model-point low-speed episode; 0.01 m/s and rad/s for 0.2 sim s.

    Receipt-time latency is host-clock empirical evidence. Neither onset nor
    command zero is a certified physical stopping bound.
    """
    if hold is None:
        return None
    def yaw(row):
        x, y, z, w = row["quaternion"]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    episode = None
    for a, b in zip(active, active[1:]):
        if a["receipt_ns"] < hold["receipt_ns"]:
            continue
        dt = (b["sim_ns"] - a["sim_ns"]) / 1e9
        speed = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt
        rotation = abs((yaw(b) - yaw(a) + math.pi) % (2 * math.pi) - math.pi) / dt
        if speed <= .01 and rotation <= .01:
            episode = episode or b
            if b["sim_ns"] - episode["sim_ns"] >= 200_000_000:
                return {"onset_receipt_s": (episode["receipt_ns"] - hold["receipt_ns"]) / 1e9,
                        "confirmation_receipt_s": (b["receipt_ns"] - hold["receipt_ns"]) / 1e9}
        else:
            episode = None
    return None


def audit(root):
    manifest = json.loads((root / "manifest.json").read_text())
    expected = [{"regime": regime, "offset_m": offset, "repetition": rep,
                 "angular_z": angular}
                for regime, angular in (("nominal", 0.0), ("strong", .25))
                for offset in (-.10, -.05, 0.0, .05, .10) for rep in range(1, 4)]
    random.Random(20261010).shuffle(expected)
    assert manifest["trials"] == expected
    for relative, digest in manifest["source_sha256"].items():
        assert hashlib.sha256((root / "source" / relative).read_bytes()).hexdigest() == digest
    records = read_rows(root / "trials.jsonl")
    assert len(records) == 30
    groups, outcomes = defaultdict(list), []
    for index, record in enumerate(records, 1):
        assert record["index"] == index
        for field, value in expected[index - 1].items():
            assert record[field] == value
        paths = list((root / f"trial_{index:02d}").glob("gate_guard_*/result.json"))
        assert len(paths) == 1
        folder = paths[0].parent
        result = json.loads(paths[0].read_text())
        assert result["monitor_y_offset_m"] == record["offset_m"]
        assert result["bias_angular_z"] == record["angular_z"]
        capture = json.loads((folder / "world_capture_manifest.json").read_text())
        for name in ("gazebo_world_pose.py", "ruleguard_controller_loop_matrix.py"):
            digest = capture["source_sha256"][f"/work/scripts/{name}"]
            assert digest == manifest["source_sha256"][f"scripts/{name}"]
        validation = json.loads((folder / "capture_validation.json").read_text())
        assert validation["alignment_verified"] and not validation["errors"]
        worlds = read_rows(folder / "world_pose.jsonl")
        active = [row for row in worlds if row["active"]]
        assert len(active) >= 20
        assert all(row["frame"] == "default" and row["child"] == "turtlebot3_waffle"
                   for row in worlds)
        assert all(a["sim_ns"] < b["sim_ns"] for a, b in zip(worlds, worlds[1:]))
        gaps = [(b["sim_ns"] - a["sim_ns"]) / 1e9 for a, b in zip(active, active[1:])]
        assert max(gaps) <= .10
        computed = [rectangle_clearance(row["x"], row["y"], result["rho"]) for row in active]
        assert all(abs(value - row["clearance_m"]) < 1e-9 for value, row in zip(computed, active))
        minimum = min(computed)
        entered = any(-.8 <= row["x"] <= .8 and .25 <= row["y"] <= .65 for row in active)
        arrived = any(math.hypot(row["x"] - 2, row["y"] + .5) <= .25 for row in active)
        summary = result["world_truth"]
        assert len(active) == summary["sample_count"]
        assert abs(minimum - summary["min_inflated_clearance_m"]) < 1e-9
        assert entered == summary["sampled_original_region_entry"] == record["original_entry"]
        assert arrived == summary["sampled_arrival"] == record["arrived"]
        odoms = read_rows(folder / "odom_and_monitor_input.jsonl")
        assert all(row["frame"] == "odom" and row["child"] == "base_footprint" for row in odoms)
        assert all(a["sim_ns"] < b["sim_ns"] for a, b in zip(odoms, odoms[1:]))
        assert all(abs(row["monitor_input_xy"][0] - row["x"]) < 1e-12
                   and abs(row["monitor_input_xy"][1] - row["y"] - record["offset_m"]) < 1e-12
                   for row in odoms)
        states = [row for row in read_rows(folder / "guard_states.jsonl") if row["active"]]
        assert all(row["authority_epoch"] == row["execution_epoch"] == 1
                   and len(row["authority_hash"]) == 64
                   and row["authority_hash"] == row["execution_hash"] for row in states)
        holds = [row for row in states if row["state"] == 2]
        boundary = [row for row in holds if row["reason"] in (
            "boundary-violated", "boundary-violation", "inside-forbidden")]
        first = boundary[0] if boundary else None
        if first:
            assert first["authority_epoch"] == first["execution_epoch"] == 1
            assert first["authority_hash"] == first["execution_hash"]
            matched = next(row for row in worlds if row["sim_ns"] == first["latest_world_sim_ns"])
            assert abs(matched["clearance_m"] - first["latest_world_clearance_m"]) < 1e-12
            assert 0 <= first["latest_world_receipt_age_s"] <= .25
        outcome = {
            "index": index, "regime": record["regime"], "offset_m": record["offset_m"],
            "result": result["result"], "returncode": record["returncode"],
            "arrived": arrived, "original_entry": entered, "min_world_clearance_m": minimum,
            "boundary_hold": bool(boundary),
            "warning_observed": any(row["state"] == 1 for row in states),
            "active_hold_reasons": dict(Counter(row["reason"] for row in holds)),
            "first_hold_latest_world_clearance_m": first["latest_world_clearance_m"] if first else None,
            "first_hold_world_sample_age_s": first["latest_world_receipt_age_s"] if first else None,
            "hold_to_zero_command_s": result["stop_latency_s"],
            "post_hold_sampled_travel_m": summary["post_hold_sampled_travel_m"],
            "world_samples": len(active), "max_sim_gap_s": max(gaps),
            "sampled_sustained_world_stop": sustained_world_stop(active, first),
        }
        outcomes.append(outcome)
        groups[(record["regime"], record["offset_m"])].append(outcome)
    cells = []
    for (regime, offset), rows in sorted(groups.items()):
        assert len(rows) == 3
        hold_values = [row["first_hold_latest_world_clearance_m"] for row in rows if row["boundary_hold"]]
        times = [row["hold_to_zero_command_s"] for row in rows if row["boundary_hold"]]
        stopped = [row["sampled_sustained_world_stop"]["onset_receipt_s"]
                   for row in rows if row["sampled_sustained_world_stop"] is not None]
        cells.append({
            "regime": regime, "offset_m": offset, "attempts": 3,
            "valid_runs": sum(row["result"] == "PASS" and row["returncode"] == 0 for row in rows),
            "arrivals": sum(row["arrived"] for row in rows),
            "original_entries": sum(row["original_entry"] for row in rows),
            "boundary_holds": len(hold_values),
            "warning_runs": sum(row["warning_observed"] for row in rows),
            "min_world_clearance_median_m": statistics.median(row["min_world_clearance_m"] for row in rows),
            "min_world_clearance_range_m": [min(row["min_world_clearance_m"] for row in rows),
                                            max(row["min_world_clearance_m"] for row in rows)],
            "first_hold_latest_world_clearance_median_m": statistics.median(hold_values) if hold_values else None,
            "hold_to_zero_command_median_s": statistics.median(times) if times else None,
            "sampled_sustained_world_stops": len(stopped),
            "world_stop_onset_median_s": statistics.median(stopped) if stopped else None,
        })
    return {"audit": "PASS", "attempts": 30, "cells": cells, "trials": outcomes,
            "failed_run_count": sum(row["result"] != "PASS" or row["returncode"] != 0 for row in outcomes),
            "limitations": "Raw sample audit only; latest-world-at-HOLD receipt is not exact decision-time truth; no physical safety proof"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    output = audit(args.root)
    (args.root / "independent_audit.json").write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps({key: value for key, value in output.items() if key != "trials"}, indent=2))
