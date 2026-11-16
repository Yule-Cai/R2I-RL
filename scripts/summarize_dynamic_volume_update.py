#!/usr/bin/env python3
"""Summarize a frozen cohort only after two independent audit gates pass."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import shutil
import statistics


def read_events(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def audit_ablation_trial(folder):
    """Check recorded observation/binding transitions, not just result summaries."""
    result = json.loads((folder/"result.json").read_text())
    if result["result"] != "PASS":
        raise ValueError("incomplete trial")
    events = read_events(folder/"runner_events.jsonl")
    if any(a["t_ns"] > b["t_ns"] for a, b in zip(events, events[1:])):
        raise ValueError("runner event ordering regressed")
    initial = [e for e in events if e["event"] == "initial_admission"]
    if len(initial) != 1:
        raise ValueError("ambiguous initial release")
    bound = initial[0]["identity"]
    before = [e for e in events if e["event"] == "snapshot_observed"
              and e["t_ns"] <= initial[0]["t_ns"]]
    if not before or not before[-1]["ready"] or before[-1]["identity"] != bound:
        raise ValueError("initial release lacks matching READY observation")
    commits = [e for e in read_events(folder/"manager_events.jsonl")
               if e["event"] == "durable_commit" and e["epoch"] == 2]
    if len(commits) != 1:
        raise ValueError("ambiguous durable update")
    commit = commits[0]
    target = [commit["epoch"], commit["snapshot_hash"]]
    metrics = result["metrics"]
    if target != metrics["durable_identity"] or commit["t_ns"] != metrics["commit_ns"]:
        raise ValueError("result COMMIT anchor/identity differs from manager trace")
    observations = [e for e in events if e["event"] == "snapshot_observed"
                    and e["identity"] == target and e["t_ns"] >= commit["t_ns"]]
    if not observations or bound == target:
        raise ValueError("missing changed-identity observation")
    observed = observations[0]
    observed_ms = (observed["t_ns"]-commit["t_ns"])/1e6
    if abs(observed_ms-metrics["commit_to_observed_ms"]) > 1e-9:
        raise ValueError("observation latency differs from trace")
    holds = [e for e in events if e["event"] == "hold" and e["t_ns"] >= commit["t_ns"]]
    expected = {"full": "epoch-disagreement", "geometry_only": "boundary-violated",
                "gate_only": None}[result["arm"]]
    if len(holds) != int(expected is not None):
        raise ValueError("unexpected HOLD count")
    hold = holds[0] if holds else None
    hold_ms, hold_ready = None, None
    if hold:
        current = [e for e in observations if e["t_ns"] <= hold["t_ns"]]
        if (not current or hold["reason"] != expected or hold["authority"] != target
                or hold["execution"] != bound):
            raise ValueError("HOLD reason or bound/observed identities differ")
        hold_ready = current[-1]["ready"]
        hold_ms = (hold["t_ns"]-commit["t_ns"])/1e6
        if abs(hold_ms-metrics["commit_to_hold_ms"]) > 1e-9:
            raise ValueError("HOLD latency differs from trace")
    if metrics["hold_reason"] != expected or metrics["commit_to_hold_ms"] != hold_ms:
        raise ValueError("result HOLD fields differ from trace")
    # The common readiness predicate is (!ready AND Q == X), not !ready alone.
    return dict(trial=folder.name, arm=result["arm"], repetition=result["repetition"],
        initial_bound_identity=bound, committed_observed_identity=target,
        first_new_observation_ready=observed["ready"],
        common_readiness_stop_at_new_observation=(not observed["ready"] and target == bound),
        ready_at_hold=hold_ready, hold_reason=expected,
        commit_to_observed_ms=observed_ms, commit_to_hold_ms=hold_ms,
        moving_commands_after_new_observation_before_hold=sum(
            e["event"] == "command" and any(v != 0 for v in e["velocity"])
            and observed["t_ns"] <= e["t_ns"]
            and (hold is None or e["t_ns"] < hold["t_ns"]) for e in events),
        audit_passed=True)


def audit_ablation(folder):
    # Reuse the existing source-integrity, manifest, and independent audit gates.
    summary = summarize(folder)
    trials = [audit_ablation_trial(path) for path in sorted(folder.glob("trial_*"))]
    return dict(audit_passed=True, valid_trials=len(trials),
        scope="Offline trace audit of the original cohort; not new experiments or instantaneous-state proof.",
        frozen_source_sha256=summary["source_sha256"],
        auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        common_readiness_predicate="not ready and observed_identity == bound_execution_identity",
        arms={arm: dict(n=len(selected),
            first_new_observation_not_ready=sum(not r["first_new_observation_ready"] for r in selected),
            common_readiness_stops_at_first_new_observation=sum(
                r["common_readiness_stop_at_new_observation"] for r in selected),
            hold_reasons=dict(Counter(r["hold_reason"] or "none" for r in selected)),
            commit_to_hold_ms=statistics_ms([r["commit_to_hold_ms"] for r in selected]))
            for arm in sorted({r["arm"] for r in trials})
            for selected in [[r for r in trials if r["arm"] == arm]]}, trials=trials)


def statistics_ms(values):
    values = sorted(v for v in values if v is not None)
    if not values:
        return None
    return dict(n=len(values), median=statistics.median(values), minimum=values[0],
                maximum=values[-1], p95_nearest_rank=values[math.ceil(.95*len(values))-1])


def summarize(folder):
    manifest = json.loads((folder/"manifest.json").read_text())
    completion = json.loads((folder/"completion.json").read_text())
    assert completion["source_unchanged"] and not completion["failures"]
    assert json.loads((folder/"recomputed_audit.json").read_text())["audit_passed"]
    assert json.loads((folder/"numpy_body_audit.json").read_text())["audit_passed"]
    assert all(hashlib.sha256((folder/"source"/p).read_bytes()).hexdigest() == h
               for p,h in manifest["source_sha256"].items())
    rows = [json.loads((path/"result.json").read_text()) for path in sorted(folder.glob("trial_*"))]
    assert len(rows) == completion["planned"] == completion["attempted"]
    assert all(r["result"] == "PASS" for r in rows)
    assert Counter((r["arm"], r["repetition"]) for r in rows) == Counter(
        (r["arm"], r["repetition"]) for r in manifest["schedule"])
    arms = {}
    for arm in sorted({r["arm"] for r in rows}):
        selected = [r["metrics"] for r in rows if r["arm"] == arm]
        arms[arm] = dict(n=len(selected), body_entry_trials=sum(m["body_entry_observed"] for m in selected),
            hold_trials=sum(m["commit_to_hold_ms"] is not None for m in selected),
            hold_reasons=dict(Counter(m["hold_reason"] for m in selected)),
            total_post_commit_pose_samples=sum(m["post_commit_samples"] for m in selected),
            timing_ms={key: statistics_ms([m[key] for m in selected]) for key in (
                "commit_to_observed_ms", "commit_to_hold_ms", "commit_to_zero_publish_ms",
                "commit_to_world_low_speed_onset_ms", "commit_to_world_low_speed_confirmation_ms")},
            body_clearance_m=statistics_ms([m["min_sampled_body_clearance_m"] for m in selected]),
            travel_to_low_speed_m=statistics_ms([m["travel_to_low_speed_m"] for m in selected]))
    rtfs = []
    for path in sorted(folder.glob("trial_*")):
        worlds = [json.loads(line) for line in (path/"world_pose.jsonl").read_text().splitlines()]
        active = [r for r in worlds if r["active"]]
        rtfs.append((active[-1]["sim_ns"]-active[0]["sim_ns"])/(active[-1]["receipt_ns"]-active[0]["receipt_ns"]))
    return dict(cohort=manifest["cohort"], valid_trials=len(rows), arms=arms,
        maximum_sim_sample_gap_s=max(r["metrics"]["max_sim_gap_s"] for r in rows),
        maximum_commit_receipt_bracket_ms=max(r["metrics"]["commit_receipt_bracket_ms"] for r in rows),
        total_post_commit_pose_samples=sum(r["metrics"]["post_commit_samples"] for r in rows),
        measured_sim_to_wall_time_ratio=statistics_ms(rtfs), source_sha256=manifest["source_sha256"],
        scope=manifest["scope"], note="One deterministic scenario, independently restarted repeated runs; not hardware or worst-case bounds. Timing in host wall-clock ms; stop confirmation additionally requires 0.2 sim s.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--ablation-audit-output", type=Path,
        help="Audit original transition traces into a NEW file outside the frozen cohort; no summary rewrite.")
    args = parser.parse_args()
    if args.ablation_audit_output:
        destination = args.ablation_audit_output.resolve()
        if destination.is_relative_to(args.folder.resolve()):
            parser.error("ablation audit must not modify the frozen cohort")
        output = audit_ablation(args.folder)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("x") as stream:
            stream.write(json.dumps(output, indent=2)+"\n")
        print(json.dumps({k: v for k, v in output.items() if k not in ("trials", "frozen_source_sha256")}, indent=2))
        return
    output = summarize(args.folder)
    script = Path(__file__).resolve()
    target = args.folder/"posthoc_source"/script.name
    target.parent.mkdir(exist_ok=True)
    shutil.copy2(script, target)
    output["summary_source_sha256"] = hashlib.sha256(script.read_bytes()).hexdigest()
    (args.folder/"cohort_summary.json").write_text(json.dumps(output, indent=2)+"\n")
    print(json.dumps({key: value for key, value in output.items() if key != "source_sha256"}, indent=2))


if __name__ == "__main__":
    main()
