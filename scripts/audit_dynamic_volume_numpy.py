#!/usr/bin/env python3
"""Independent post-hoc body/WAL audit; does not call gate/monitor functions."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np


def audit_trial(folder):
    result = json.loads((folder/"result.json").read_text())
    if result["result"] != "PASS":
        raise ValueError("failed attempts are not valid measurement rows")
    metrics = result["metrics"]
    rows = [json.loads(s) for s in (folder/"world_pose.jsonl").read_text().splitlines()]
    post = [r for r in rows if r["active"] and r["receipt_ns"] >= metrics["commit_ns"]]
    wal = [json.loads(s) for s in (folder/"wal.jsonl").read_text().splitlines()]
    commits = [r for r in wal if r.get("decision")=="COMMIT" and r.get("epoch")==2]
    if len(commits) != 1:
        raise ValueError("ambiguous durable target")
    volume = commits[0]["target_volumes"][0]
    center, size = np.asarray(volume["center"]), np.asarray(volume["dimensions"])
    lo, hi = center-size/2, center+size/2
    model = ET.parse(folder/"world.sdf").getroot().find("world/model")
    body_size = np.asarray([float(v) for v in model.findtext("link[@name='base_link']/collision/geometry/box/size").split()])
    half = body_size/2
    distances, composition_errors = [], []
    for row in post:
        if (row["parent"], row["intermediate"], row["child"]) != ("dynamic_volume", "r2i_quad", "r2i_quad/base_link"):
            raise ValueError("unexpected transform chain")
        # This declared gantry has zero model attitude and zero yaw command.
        # Verify that fact per sample; never infer it from the picture.
        for field in ("quaternion", "model_world_quaternion"):
            if not np.allclose(row[field], [0,0,0,1], rtol=0, atol=1e-12):
                raise ValueError("non-axis-aligned attitude outside this independent audit")
        xyz = np.asarray(row["raw_link_xyz"])+np.asarray(row["model_world_xyz"])
        composition_errors.append(float(np.linalg.norm(xyz-np.asarray(row["xyz"]))))
        gaps = np.maximum.reduce([lo-(xyz+half), (xyz-half)-hi, np.zeros(3)])
        distances.append(float(np.linalg.norm(gaps)))
    entries = [r for r, distance in zip(post, distances) if distance==0.]
    actual = dict(body_entry_observed=bool(entries), body_entry_sample_count=len(entries),
                  first_body_entry_sim_ns=entries[0]["sim_ns"] if entries else None,
                  post_commit_samples=len(post))
    if any(metrics[key] != value for key,value in actual.items()):
        raise ValueError("saved entry/sample metrics disagree")
    error = abs(min(distances)-metrics["min_sampled_body_clearance_m"])
    if error > 1e-12 or max(composition_errors) > 1e-12:
        raise ValueError("saved distance/composition disagrees")
    return dict(trial=folder.name, arm=result["arm"], **actual,
        minimum_body_clearance_m=min(distances), minimum_distance_difference_m=error,
        maximum_pose_composition_difference_m=max(composition_errors),
        collision_size_from_run_sdf_m=body_size.tolist(),
        committed_target_hash=commits[0]["target_hash"], audit_passed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--output-name", default="numpy_body_audit.json")
    args = parser.parse_args()
    if Path(args.output_name).name != args.output_name:
        parser.error("output name must be a basename")
    results = [audit_trial(folder) for folder in sorted(args.folder.glob("trial_*"))]
    script = Path(__file__).resolve()
    # Keep a second-host audit alongside, not over the original auditor.
    audit_name = script.name if args.output_name == "numpy_body_audit.json" else "local_"+script.name
    saved = args.folder/"posthoc_source"/audit_name
    saved.parent.mkdir(exist_ok=True)
    shutil.copy2(script, saved)
    output = dict(audit_passed=bool(results) and all(r["audit_passed"] for r in results),
        backend="NumPy analytic AABB overlap/distance; body from SDF, target from durable WAL",
        numpy_version=np.__version__, audit_source_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
        trials=results)
    (args.folder/args.output_name).write_text(json.dumps(output, indent=2)+"\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
