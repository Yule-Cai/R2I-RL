"""Offline board-task audit; retain aborted attempts without calling them PASS."""
import argparse
import hashlib
import importlib
import json
import math
from pathlib import Path
import shutil
import sys


def equal(actual, saved):
    if isinstance(actual, dict):
        return isinstance(saved,dict) and actual.keys()==saved.keys() and all(equal(v,saved[k]) for k,v in actual.items())
    if isinstance(actual, list):
        return isinstance(saved,list) and len(actual)==len(saved) and all(equal(a,b) for a,b in zip(actual,saved))
    if isinstance(actual,float):
        return isinstance(saved,(int,float)) and math.isclose(actual,saved,rel_tol=0,abs_tol=1e-12)
    return type(actual) is type(saved) and actual==saved


def audit(root):
    manifest = json.loads((root/"manifest.json").read_text())
    source_ok = all(hashlib.sha256((root/"source"/p).read_bytes()).hexdigest()==h
        for p,h in manifest["source_sha256"].items())
    launches = list(root.glob("*-launch.json"))
    if len(launches)!=1:
        raise ValueError("one launcher record required")
    launch = json.loads(launches[0].read_text())
    records = list(root.glob("trial_*/result.json"))
    if launch["abort_reason"] or len(records)!=1:
        return dict(run=root.name,valid_motion_trial=False,source_integrity=source_ok,
            abort_reason=launch["abort_reason"],exit_code=launch["exit_code"],
            result_record_missing=not records, audit_passed=False)
    saved=json.loads(records[0].read_text())
    if saved["result"]!="PASS":
        return dict(run=root.name,valid_motion_trial=False,source_integrity=source_ok,
            error=saved.get("error"),audit_passed=False)
    try:
        import rulesync3d.ruleguard
    except ModuleNotFoundError:
        # Local source namespace differs from the board's installed flat package.
        sys.modules["rulesync3d"]=importlib.import_module("rulesync3d.rulesync3d")
    from benchmark_dynamic_volume_update import evaluate
    from volume_task_recovery import audit_task
    metrics=evaluate(records[0].parent)
    task=audit_task(records[0].parent)
    metric_ok=equal(metrics,saved["metrics"])
    task_ok=equal(task,saved["task_metrics"])
    samples=[json.loads(s) for s in next(root.glob("*-resources.jsonl")).read_text().splitlines()]
    temperature=max(v for row in samples for v in row["temperatures_mC"].values())/1000
    rss=max(sum(p["rss_bytes"] for p in row["processes"]) for row in samples)/2**20
    worlds=[json.loads(s) for s in (records[0].parent/"world_pose.jsonl").read_text().splitlines()]
    active=[r for r in worlds if r["active"]]
    ratio=(active[-1]["sim_ns"]-active[0]["sim_ns"])/(active[-1]["receipt_ns"]-active[0]["receipt_ns"])
    return dict(run=root.name,valid_motion_trial=True,source_integrity=source_ok,
        metrics_reproduced=metric_ok,task_reproduced=task_ok,
        audit_passed=source_ok and metric_ok and task_ok,
        task_metrics=task,body_entry_sample_count=metrics["body_entry_sample_count"],
        post_commit_samples=metrics["post_commit_samples"],
        min_sampled_body_clearance_m=metrics["min_sampled_body_clearance_m"],
        commit_to_hold_ms=metrics["commit_to_hold_ms"],
        commit_to_world_low_speed_onset_ms=metrics["commit_to_world_low_speed_onset_ms"],
        active_sim_wall_ratio=ratio,max_temperature_C=temperature,
        max_process_rss_sum_MiB=rss)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots",nargs="+",type=Path)
    parser.add_argument("--output-name",default="task_chain_audit.json")
    args=parser.parse_args()
    if Path(args.output_name).name!=args.output_name:
        parser.error("output name must be a basename")
    for root in args.roots:
        result=audit(root)
        result["audit_source_sha256"]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        target=root/"posthoc_source"/("mac_"+Path(__file__).name if args.output_name.startswith("mac_") else Path(__file__).name)
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(__file__,target)
        (root/args.output_name).write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps(result))
    # Excluded/aborted attempts are reported, never silently counted as successes.


if __name__=="__main__":
    main()
