#!/usr/bin/env python3
"""Run the predeclared RuleGuard controller-generalization matrix.

Design: 2 mirrored rule geometries x 2 controller configurations x 3
disturbance regimes x 2 enforcement arms x 3 isolated repetitions = 72 runs.
The manifest is written before execution.  Results are appended after every
run so an interrupted campaign can resume without changing its design.
"""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import time


GEOMETRIES = (
    {
        "geometry": "above",
        "path_y": -0.5,
        "rule_y_min": 0.25,
        "rule_y_max": 0.65,
        "direction": 1.0,
    },
    {
        "geometry": "below_mirror",
        "path_y": 0.5,
        "rule_y_min": -0.65,
        "rule_y_max": -0.25,
        "direction": -1.0,
    },
)

CONTROLLERS = (
    {"controller": "slow_short", "speed": 0.20, "lookahead": 0.40},
    {"controller": "fast_long", "speed": 0.35, "lookahead": 0.70},
)

REGIMES = (
    {"regime": "nominal", "bias": 0.0, "bias_end_x": None},
    {"regime": "moderate", "bias": 0.04, "bias_end_x": 0.8},
    {"regime": "strong", "bias": 0.25, "bias_end_x": None},
)

ARMS = ("gate", "gate_guard")


def build_manifest(repetitions, domain_start):
    rows = []
    index = 0
    for geometry in GEOMETRIES:
        for controller in CONTROLLERS:
            for regime in REGIMES:
                for arm in ARMS:
                    for repetition in range(1, repetitions + 1):
                        direction = float(geometry["direction"])
                        rows.append({
                            "run_id": (
                                f"{geometry['geometry']}__"
                                f"{controller['controller']}__"
                                f"{regime['regime']}__{arm}__r{repetition}"
                            ),
                            "index": index,
                            "domain_id": domain_start + index,
                            "repetition": repetition,
                            "arm": arm,
                            "geometry": geometry["geometry"],
                            "path_y": geometry["path_y"],
                            "rule_y_min": geometry["rule_y_min"],
                            "rule_y_max": geometry["rule_y_max"],
                            "controller": controller["controller"],
                            "speed": controller["speed"],
                            "lookahead": controller["lookahead"],
                            "regime": regime["regime"],
                            "bias_angular_z": direction * regime["bias"],
                            "bias_end_x": regime["bias_end_x"],
                        })
                        index += 1
    return rows


def claim_pass(row, result):
    common = bool(result.get("gate_accepted"))
    if row["regime"] in ("nominal", "moderate"):
        return (
            common
            and bool(result.get("arrived"))
            and not bool(result.get("entered_original_forbidden"))
            and not bool(result.get("hold_observed"))
        )
    if row["arm"] == "gate":
        return common and bool(result.get("entered_original_forbidden"))
    return (
        common
        and bool(result.get("hold_observed"))
        and bool(result.get("stop_path_observed"))
        and not bool(result.get("entered_original_forbidden"))
    )


def write_csv(path, rows):
    if not rows:
        return
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def load_completed(path):
    completed = {}
    if not path.exists():
        return completed
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            row = json.loads(line)
            completed[row["run_id"]] = row
    return completed


def parse_result(stdout):
    for line in reversed(stdout.splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and "result" in value:
            return value
    raise RuntimeError("single-run driver emitted no result JSON")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--single-run-script", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--domain-start", type=int, default=150)
    parser.add_argument("--run-timeout", type=float, default=35.0)
    parser.add_argument("--process-timeout", type=float, default=110.0)
    args = parser.parse_args()

    manifest = build_manifest(args.repetitions, args.domain_start)
    if manifest and max(row["domain_id"] for row in manifest) > 232:
        parser.error("domain range exceeds Fast DDS maximum 232")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_logs = args.output_dir / "runs"
    run_logs.mkdir(exist_ok=True)
    manifest_path = args.output_dir / "controller_generalization_manifest.csv"
    write_csv(manifest_path, manifest)

    jsonl_path = args.output_dir / "controller_generalization_results.jsonl"
    completed = load_completed(jsonl_path)
    for row in manifest:
        if row["run_id"] in completed:
            continue
        command = [
            sys.executable,
            str(args.single_run_script),
            "--arm", row["arm"],
            "--domain-id", str(row["domain_id"]),
            "--log-root", str(run_logs),
            "--label", row["run_id"],
            "--run-timeout", str(args.run_timeout),
            "--path-y", str(row["path_y"]),
            "--rule-y-min", str(row["rule_y_min"]),
            "--rule-y-max", str(row["rule_y_max"]),
            "--speed", str(row["speed"]),
            "--lookahead", str(row["lookahead"]),
            "--bias-angular-z", str(row["bias_angular_z"]),
        ]
        if row["bias_end_x"] is not None:
            command += ["--bias-end-x", str(row["bias_end_x"])]
        started = time.time()
        try:
            process = subprocess.run(
                command,
                text=True,
                capture_output=True,
                timeout=args.process_timeout,
                check=False,
            )
            result = parse_result(process.stdout)
            error = "" if process.returncode == 0 else process.stderr[-2000:]
        except Exception as exc:
            result = {"result": "FAIL", "error": str(exc)}
            error = str(exc)
        record = {
            **row,
            **result,
            "run_id": row["run_id"],
            "matrix_claim_pass": claim_pass(row, result),
            "process_error": error,
            "wall_s": round(time.time() - started, 3),
        }
        with jsonl_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")
        completed[row["run_id"]] = record
        print(json.dumps({
            "completed": len(completed),
            "total": len(manifest),
            "run_id": row["run_id"],
            "claim_pass": record["matrix_claim_pass"],
            "driver_result": record.get("result"),
        }, sort_keys=True), flush=True)

    ordered = [completed[row["run_id"]] for row in manifest]
    write_csv(
        args.output_dir / "controller_generalization_results.csv", ordered)
    summary = {
        "design": "2 geometries x 2 controllers x 3 regimes x 2 arms",
        "repetitions": args.repetitions,
        "runs": len(ordered),
        "driver_passes": sum(row.get("result") == "PASS" for row in ordered),
        "claim_passes": sum(
            bool(row.get("matrix_claim_pass")) for row in ordered),
        "arrivals": sum(bool(row.get("arrived")) for row in ordered),
        "holds": sum(bool(row.get("hold_observed")) for row in ordered),
        "original_entries": sum(
            bool(row.get("entered_original_forbidden")) for row in ordered),
    }
    (args.output_dir / "controller_generalization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
