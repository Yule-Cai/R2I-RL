#!/usr/bin/env python3
"""Reproducible offline scaling benchmark for RuleSync admission/monitoring.

The generated geometries are valid and deliberately placed away from the
candidate path/pose so every call scans the complete rule set.  This measures
the conservative worst branch for the current deterministic implementation,
without ROS, Gazebo, planner, or network overhead.
"""

import argparse
import csv
import json
from pathlib import Path
import statistics
import time

import numpy as np

from rulesync3d.gate2d import path_intersects_polygons
from rulesync3d.gate_3d import path_intersects_volumes
from rulesync3d.ruleguard import min_clearance_2d, min_clearance_3d


def percentile(values, q):
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def measure(operation, warmups, repeats):
    for _ in range(warmups):
        operation()
    samples_ms = []
    for _ in range(repeats):
        started = time.perf_counter_ns()
        operation()
        samples_ms.append((time.perf_counter_ns() - started) / 1e6)
    return {
        "median_ms": statistics.median(samples_ms),
        "p95_ms": percentile(samples_ms, 0.95),
        "min_ms": min(samples_ms),
        "max_ms": max(samples_ms),
    }


def make_path_2d(segment_count):
    return np.column_stack((
        np.linspace(0.0, 20.0, segment_count + 1),
        np.zeros(segment_count + 1),
    ))


def make_path_3d(segment_count):
    return np.column_stack((
        np.linspace(0.0, 20.0, segment_count + 1),
        np.zeros(segment_count + 1),
        np.ones(segment_count + 1),
    ))


def make_polygons(rule_count):
    return [
        np.asarray([
            (30.0 + i, 30.0), (30.4 + i, 30.0),
            (30.4 + i, 30.4), (30.0 + i, 30.4),
        ], dtype=np.float64)
        for i in range(rule_count)
    ]


def make_volumes(rule_count):
    return [
        {
            "geometry_type": 0,
            "bmin": (30.0 + i, 30.0, 30.0),
            "bmax": (30.4 + i, 30.4, 30.4),
        }
        for i in range(rule_count)
    ]


def benchmark(args):
    rows = []
    for rule_count in args.rule_counts:
        polygons = make_polygons(rule_count)
        volumes = make_volumes(rule_count)
        for segment_count in args.segment_counts:
            path_2d = make_path_2d(segment_count)
            path_3d = make_path_3d(segment_count)
            cases = (
                ("gate_2d", lambda p=path_2d, r=polygons:
                    path_intersects_polygons(
                        p, r, inflation_radius=args.inflation_radius)),
                ("gate_3d", lambda p=path_3d, r=volumes:
                    path_intersects_volumes(p, r)),
            )
            for operation_name, operation in cases:
                row = {
                    "operation": operation_name,
                    "rule_count": rule_count,
                    "segment_count": segment_count,
                    "repeats": args.repeats,
                    "inflation_radius": (
                        args.inflation_radius
                        if operation_name == "gate_2d" else 0.0),
                }
                row.update(measure(operation, args.warmups, args.repeats))
                rows.append(row)
                print(json.dumps({
                    "operation": operation_name,
                    "rule_count": rule_count,
                    "segment_count": segment_count,
                    "median_ms": row["median_ms"],
                    "p95_ms": row["p95_ms"],
                }, sort_keys=True), flush=True)

        monitor_cases = (
            ("monitor_2d", lambda r=polygons:
                min_clearance_2d((0.0, 0.0), r, 0.2)),
            ("monitor_3d", lambda r=volumes:
                min_clearance_3d((0.0, 0.0, 1.0), r, 0.2)),
        )
        for operation_name, operation in monitor_cases:
            row = {
                "operation": operation_name,
                "rule_count": rule_count,
                "segment_count": 0,
                "repeats": args.repeats,
                "inflation_radius": args.inflation_radius,
            }
            row.update(measure(operation, args.warmups, args.repeats))
            rows.append(row)
            print(json.dumps({
                "operation": operation_name,
                "rule_count": rule_count,
                "segment_count": 0,
                "median_ms": row["median_ms"],
                "p95_ms": row["p95_ms"],
            }, sort_keys=True), flush=True)
    return rows


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--rule-counts", type=int, nargs="+",
                        default=(1, 10, 100))
    parser.add_argument("--segment-counts", type=int, nargs="+",
                        default=(10, 100, 1000))
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--repeats", type=int, default=30)
    parser.add_argument("--inflation-radius", type=float, default=0.20)
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = benchmark(args)
    csv_path = args.output_dir / "authorization_scaling.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    json_path = args.output_dir / "authorization_scaling.json"
    json_path.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "csv": str(csv_path),
        "json": str(json_path),
        "rows": len(rows),
        "repeats": args.repeats,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
