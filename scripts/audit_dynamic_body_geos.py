#!/usr/bin/env python3
"""Post-hoc GEOS check of dynamic-update body metrics against durable geometry.

Use the serialized Point32 polygon in the COMMIT WAL, not ideal CLI decimals.
This audits recorded samples; it is not another experiment or swept-body proof.
Dependencies: Shapely 2.1.2 (GEOS); run in a separate evaluation environment.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import shapely
from shapely.affinity import rotate, translate
from shapely.geometry import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"rulesync3d"))
from rulesync3d.ruleguard import rule_snapshot_items, snapshot_hash


def read(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s]


def audit(folder):
    manifest = json.loads((folder/"manifest.json").read_text())
    body = Polygon(manifest["body_envelope_xy"])
    nominal_region = Polygon(manifest["new_region_xy"])
    output = dict(shapely_version=shapely.__version__, geos_version=shapely.geos_version_string,
                  python_version=sys.version, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  checked_samples=0, errors=[], trials=[])
    for index, trial in enumerate(manifest["trials"], 1):
        path = folder/f"trial_{index:02d}"
        try:
            result = json.loads((path/"result.json").read_text())
            if result["result"] != "PASS":
                raise ValueError("no completed trial")
            metrics = result["metrics"]
            records = [r for r in read(path/"wal.jsonl")
                       if r.get("decision") == "COMMIT" and r.get("epoch") == 2]
            if len(records) != 1:
                raise ValueError("ambiguous/missing durable target")
            record = records[0]
            sha = snapshot_hash(rule_snapshot_items(record["target_rules"]))
            if sha != record["target_hash"] or metrics["durable_identity"] != [2, sha]:
                raise ValueError("WAL payload/full-hash mismatch")
            regions = [Polygon([(p[0], p[1]) for p in r["polygon"]])
                       for r in record["target_rules"] if r["rule_type"] == 0 and r["severity"] == 1]
            if len(regions) != 1 or not regions[0].is_valid:
                raise ValueError("pilot expects one valid enforced polygon")
            region = regions[0]
            worlds = [r for r in read(path/"world_pose.jsonl")
                      if r["active"] and r["receipt_ns"] >= metrics["commit_ns"]]
            distances, nominal_distances, entered = [], [], []
            for row in worlds:
                qx, qy, qz, qw = row["quaternion"]
                yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                geometry = translate(rotate(body, yaw, origin=(0, 0), use_radians=True),
                                     xoff=row["x"], yoff=row["y"])
                distances.append(geometry.distance(region))
                nominal_distances.append(geometry.distance(nominal_region))
                if geometry.intersects(region):
                    entered.append(row)
            if len(worlds) != metrics["post_commit_sample_count"]:
                raise ValueError("world sample denominator mismatch")
            if len(entered) != metrics["sampled_body_entry_count"]:
                raise ValueError("committed-geometry entry count differs")
            first = entered[0]["sim_ns"] if entered else None
            if first != metrics["first_body_entry_sim_ns"]:
                raise ValueError("committed-geometry first entry differs")
            ideal_difference = abs(min(nominal_distances)-metrics["minimum_sampled_body_distance_m"])
            if ideal_difference > 1e-12:
                raise ValueError("SAT/GEOS nominal-distance mismatch")
            output["checked_samples"] += len(worlds)
            output["trials"].append(dict(index=index, **trial, durable_hash=sha,
                sampled_body_entry_count=len(entered), first_body_entry_sim_ns=first,
                minimum_body_distance_to_committed_polygon_m=min(distances),
                maximum_point32_vs_nominal_distance_difference_m=max(
                    abs(a-b) for a, b in zip(distances, nominal_distances)),
                nominal_sat_vs_geos_min_distance_difference_m=ideal_difference))
        except Exception as exc:
            output["errors"].append(dict(index=index, error=repr(exc)))
    output["audit_passed"] = not output["errors"] and len(output["trials"]) == len(manifest["trials"])
    output["scope"] = "sampled declared body envelope against hash-verified WAL polygon; no continuous-time guarantee"
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    frozen = args.folder/"posthoc_source"/Path(__file__).name
    frozen.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(Path(__file__), frozen)
    result = audit(args.folder)
    (args.folder/"geos_body_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    sys.exit(int(not result["audit_passed"]))
