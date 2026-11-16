#!/usr/bin/env python3
"""Create paper-ready aggregates from ControllerGen-72 raw JSONL."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
import statistics


def mean(values):
    return statistics.fmean(values) if values else None


def aggregate(rows, keys):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in keys)].append(row)
    out = []
    for values, group in sorted(groups.items()):
        stop = [float(row["stop_latency_s"]) for row in group
                if row.get("stop_latency_s") is not None]
        travel = [float(row["travel_after_hold_m"]) for row in group
                  if row.get("hold_observed")]
        clearance = [float(row["min_clearance_m"]) for row in group
                     if row.get("min_clearance_m") is not None]
        record = dict(zip(keys, values))
        record.update({
            "n": len(group),
            "driver_passes": sum(row.get("result") == "PASS"
                                 for row in group),
            "claim_passes": sum(bool(row.get("matrix_claim_pass"))
                                for row in group),
            "arrivals": sum(bool(row.get("arrived")) for row in group),
            "warnings": sum(bool(row.get("warning_observed"))
                            for row in group),
            "holds": sum(bool(row.get("hold_observed")) for row in group),
            "original_entries": sum(
                bool(row.get("entered_original_forbidden")) for row in group),
            "stop_paths": sum(bool(row.get("stop_path_observed"))
                              for row in group),
            "mean_min_clearance_m": mean(clearance),
            "mean_stop_latency_s": mean(stop),
            "min_stop_latency_s": min(stop) if stop else None,
            "max_stop_latency_s": max(stop) if stop else None,
            "mean_travel_after_hold_m": mean(travel),
        })
        out.append(record)
    return out


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.results.read_text(
        encoding="utf-8").splitlines() if line.strip()]
    if len(rows) != 72:
        raise RuntimeError(f"expected the frozen 72-run design, got {len(rows)}")
    run_ids = [row["run_id"] for row in rows]
    if len(set(run_ids)) != len(run_ids):
        raise RuntimeError("duplicate run_id in raw results")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "controller_generalization_raw.csv", rows)
    detailed = aggregate(
        rows, ["geometry", "controller", "regime", "arm"])
    paper = aggregate(rows, ["regime", "arm"])
    write_csv(args.output_dir / "controller_generalization_cells.csv", detailed)
    write_csv(args.output_dir / "controller_generalization_paper_rows.csv", paper)
    summary = {
        "runs": len(rows),
        "driver_passes": sum(row.get("result") == "PASS" for row in rows),
        "claim_passes": sum(bool(row.get("matrix_claim_pass")) for row in rows),
        "geometries": sorted(set(row["geometry"] for row in rows)),
        "controllers": sorted(set(row["controller"] for row in rows)),
        "regimes": sorted(set(row["regime"] for row in rows)),
        "arms": sorted(set(row["arm"] for row in rows)),
        "paper_rows": paper,
    }
    (args.output_dir / "controller_generalization_aggregate.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "runs": summary["runs"],
        "driver_passes": summary["driver_passes"],
        "claim_passes": summary["claim_passes"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
