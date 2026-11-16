#!/usr/bin/env python3
"""Independent GEOS audit of the continuous inflated 2-D gate.

The RuleSync predicate is compared against Shapely/GEOS line-to-polygon
distance, not against another implementation of the same segment formulas.
The deterministic campaign is stratified by geometry and inflation radius.
"""

import argparse
import csv
import json
import hashlib
from pathlib import Path
import random

from shapely.geometry import LineString, Polygon

from rulesync3d.gate2d import segment_intersects_polygon, PreparedPolygonIndex


SHAPES = {
    "axis_square": [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)],
    "rotated_diamond": [(0.0, -1.4), (-1.4, 0.0), (0.0, 1.4), (1.4, 0.0)],
    "concave_l": [(-1.2, -1.2), (-1.2, 1.2), (-0.2, 1.2),
                  (-0.2, -0.2), (1.2, -0.2), (1.2, -1.2)],
    "thin_bar": [(-0.05, -1.5), (-0.05, 1.5), (0.05, 1.5), (0.05, -1.5)],
}


def bounds(points, expansion):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (
        min(xs) - expansion,
        min(ys) - expansion,
        max(xs) + expansion,
        max(ys) + expansion,
    )


def one_cell(shape_name, points, rho, samples, seed, indexed=False):
    rng = random.Random(seed)
    polygon = Polygon(points)
    if not polygon.is_valid:
        raise ValueError(f"invalid audit polygon: {shape_name}")
    xmin, ymin, xmax, ymax = bounds(points, 1.5 + rho)
    disagreement = 0
    false_accept = 0
    false_reject = 0
    raw_margin_leaks = 0
    oracle_positive = 0
    examples = []
    prepared = PreparedPolygonIndex([points], rho) if indexed else None
    direct_disagreements = 0
    exact_checks = 0
    for index in range(samples):
        start = (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
        end = (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
        line = LineString((start, end))
        oracle = bool(line.intersects(polygon) or line.distance(polygon) <= rho)
        direct = bool(segment_intersects_polygon(
            start, end, points, inflation_radius=rho))
        stats = {}
        observed = (prepared.intersects_path([start, end], stats)
                    if prepared is not None else direct)
        direct_disagreements += int(observed != direct)
        exact_checks += stats.get('exact_checks', 1)
        raw = bool(segment_intersects_polygon(start, end, points))
        oracle_positive += int(oracle)
        raw_margin_leaks += int(oracle and not raw)
        if oracle != observed:
            disagreement += 1
            false_accept += int(oracle and not observed)
            false_reject += int(observed and not oracle)
            if len(examples) < 5:
                examples.append({
                    "index": index,
                    "start": start,
                    "end": end,
                    "oracle": oracle,
                    "observed": observed,
                    "distance": line.distance(polygon),
                })
    return {
        "shape": shape_name,
        "rho": rho,
        "seed": seed,
        "samples": samples,
        "oracle_positive": oracle_positive,
        "disagreements": disagreement,
        "false_accepts": false_accept,
        "false_rejects": false_reject,
        "raw_margin_leaks": raw_margin_leaks,
        "backend": 'indexed' if indexed else 'direct',
        "direct_disagreements": direct_disagreements,
        "exact_checks": exact_checks,
        "examples": examples,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--samples-per-cell", type=int, default=5000)
    parser.add_argument("--seed", type=int, default=20261002)
    parser.add_argument("--radii", type=float, nargs="+", default=(0.05, 0.20, 0.50))
    parser.add_argument("--indexed", action='store_true',
                        help='Audit BVH backend against both GEOS and direct scan.')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.indexed:
        manifest = args.output_dir / 'manifest.json'
        if manifest.exists():
            parser.error('choose a fresh indexed-audit output directory')
        gate = Path(__file__).resolve().parents[1] / 'rulesync3d/rulesync3d/gate2d.py'
        manifest.write_text(json.dumps({
            'backend': 'indexed', 'seed': args.seed,
            'samples_per_cell': args.samples_per_cell, 'radii': args.radii,
            'geometries': list(SHAPES),
            'scope': 'same deterministic generator as original GEOS audit, not a new independent cohort',
            'gate_sha256': hashlib.sha256(gate.read_bytes()).hexdigest(),
            'audit_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        }, indent=2) + '\n', encoding='utf-8')

    rows = []
    for shape_index, (name, points) in enumerate(SHAPES.items()):
        for radius_index, rho in enumerate(args.radii):
            seed = args.seed + shape_index * 1000 + radius_index
            row = one_cell(
                name, points, float(rho), args.samples_per_cell, seed, args.indexed)
            rows.append(row)
            print(json.dumps({
                key: row[key] for key in (
                    "shape", "rho", "samples", "disagreements",
                    "raw_margin_leaks")
            }, sort_keys=True), flush=True)

    csv_path = args.output_dir / "inflated_gate_geos_audit.csv"
    csv_rows = [{key: value for key, value in row.items() if key != "examples"}
                for row in rows]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
    summary = {
        "cells": len(rows),
        "samples": sum(row["samples"] for row in rows),
        "oracle_positive": sum(row["oracle_positive"] for row in rows),
        "disagreements": sum(row["disagreements"] for row in rows),
        "direct_disagreements": sum(row["direct_disagreements"] for row in rows),
        "exact_checks": sum(row["exact_checks"] for row in rows),
        "backend": 'indexed' if args.indexed else 'direct',
        "false_accepts": sum(row["false_accepts"] for row in rows),
        "false_rejects": sum(row["false_rejects"] for row in rows),
        "raw_margin_leaks": sum(row["raw_margin_leaks"] for row in rows),
        "seed": args.seed,
        "samples_per_cell": args.samples_per_cell,
        "radii": list(args.radii),
        "geometries": list(SHAPES),
        "cells_detail": rows,
    }
    summary_path = args.output_dir / "inflated_gate_geos_audit.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        key: summary[key] for key in (
            "samples", "disagreements", "false_accepts", "false_rejects",
            "raw_margin_leaks")
    }, sort_keys=True))
    if summary['disagreements'] or summary['direct_disagreements']:
        raise SystemExit('geometry disagreement: inspect retained examples')


if __name__ == "__main__":
    main()
