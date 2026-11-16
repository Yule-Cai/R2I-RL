#!/usr/bin/env python3
"""Standard-library raw-event audit for the serialized update-load experiment.

Does not equate READY, a gate acceptance, a command, or wheel odometry with a
currently executable authorization. Categories partition a fixed window.
Restoration is censored at the next COMMIT: a superseded identity cannot count
as a later recovery of an earlier update.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def identity(row):
    return [row["epoch"], row["snapshot_hash"]]


def integrate(rows, start, end):
    if end <= start:
        raise ValueError("invalid measurement window")
    ordered = sorted(rows, key=lambda r: r["t_ns"])
    durable, guard, released = None, None, None
    totals = dict(valid=0, stale_release=0, blocked=0)
    segments = []
    last = start
    mismatched_releases = []

    def category():
        if durable is not None and released is not None and released != durable:
            return "stale_release"
        if (durable is not None and released == durable and guard is not None
                and guard["authority"] == durable and guard["execution"] == durable
                and guard["state"] != "HOLD" and not guard["degraded"]):
            return "valid"
        return "blocked"

    for row in ordered:
        when = row["t_ns"]
        if when > end:
            break
        if when > start:
            key = category()
            totals[key] += when - last
            if when > last:
                segments.append(dict(start_ns=last, end_ns=when, category=key))
            last = when
        if row["event"] == "durable_commit":
            durable = identity(row)
        elif row["event"] == "guard_state":
            guard = row
        elif row["event"] == "path_stop":
            released = None
        elif row["event"] == "path_release":
            released = row["execution"]
            if start <= when <= end and released != durable:
                mismatched_releases.append(dict(t_ns=when, release=released, durable=durable))
    key = category()
    totals[key] += end - last
    if end > last:
        segments.append(dict(start_ns=last, end_ns=end, category=key))
    assert sum(totals.values()) == end - start
    return dict(fractions={k: v / (end - start) for k, v in totals.items()},
                durations_ns=totals, segments=segments,
                mismatched_new_releases=mismatched_releases)


def update_metrics(rows, start, end):
    commits = sorted((r for r in rows if r["event"] == "durable_commit"),
                     key=lambda r: r["t_ns"])
    result = []
    for i, commit in enumerate(commits):
        if not start <= commit["t_ns"] < end:
            continue
        new_identity = identity(commit)
        cutoff = min(end, commits[i + 1]["t_ns"] if i + 1 < len(commits) else end)
        subset = [r for r in rows if commit["t_ns"] <= r["t_ns"] < cutoff]
        hold = next((r for r in subset if r["event"] == "guard_state"
                     and r["state"] == "HOLD"), None)
        observed = next((r for r in subset if r["event"] == "snapshot_observed"
                         and r["authority"] == new_identity
                         and r["callback_entry_ns"] >= commit["t_ns"]), None)
        restored = next((r for r in subset if r["event"] == "path_release"
                         and r["execution"] == new_identity
                         and r["state"] != "HOLD"), None)
        result.append(dict(epoch=commit["epoch"], snapshot_hash=commit["snapshot_hash"],
            commit_ns=commit["t_ns"], censor_ns=cutoff,
            hold_reason=hold["reason"] if hold else None,
            commit_to_hold_ms=(hold["t_ns"] - commit["t_ns"]) / 1e6 if hold else None,
            commit_to_observed_ms=(observed["callback_entry_ns"] - commit["t_ns"]) / 1e6
                if observed else None,
            restored_before_supersession=restored is not None,
            commit_to_release_ms=(restored["t_ns"] - commit["t_ns"]) / 1e6
                if restored else None))
    return result


def audit_trial(folder, trial, duration):
    recorded = json.loads((folder / "result.json").read_text())
    if recorded["trial"] != trial or recorded["result"] != "PASS":
        raise ValueError(f"invalid trial outcome: {recorded}")
    rows = []
    for role in ("manager", "guard", "runner"):
        role_rows = read_rows(folder / f"{role}_events.jsonl")
        if any(a["t_ns"] > b["t_ns"] for a, b in zip(role_rows, role_rows[1:])):
            raise ValueError(f"regressing {role} event sequence")
        rows.extend(role_rows)
    rows.sort(key=lambda r: r["t_ns"])
    starts = [r for r in rows if r["event"] == "measurement_start"]
    ends = [r for r in rows if r["event"] == "measurement_end"]
    if len(starts) != 1 or len(ends) != 1:
        raise ValueError("missing/duplicate measurement anchors")
    start, end = starts[0]["t_ns"], ends[0]["window_end_ns"]
    if abs((end - start) / 1e9 - duration) > 1e-8:
        raise ValueError("window differs from pre-run manifest")
    wal = [r for r in read_rows(folder / "wal.jsonl") if r.get("decision") == "COMMIT"]
    commits = [r for r in rows if r["event"] == "durable_commit"]
    if [(r["epoch"], r["target_hash"], r["txn_id"]) for r in wal] != [
            (r["epoch"], r["snapshot_hash"], r["txn_id"]) for r in commits]:
        raise ValueError("durable instrumentation disagrees with WAL")
    if any(len(r["snapshot_hash"]) != 64 for r in commits):
        raise ValueError("incomplete identity hash")
    expected_epoch = list(range(1, len(commits) + 1))
    if [r["epoch"] for r in commits] != expected_epoch:
        raise ValueError("unexpected epoch progression")
    # Verify update isolation: all full snapshots contain the same polygon.
    polygons = [r["target_rules"][0]["polygon"] for r in wal]
    if any(p != polygons[0] for p in polygons):
        raise ValueError("geometry changed rather than identity-only load")
    period = trial["period_s"]
    expected_slots = int(round(duration / period)) - 1 if period else 0
    slots = [r for r in rows if r["event"] in ("update_request", "update_slot_busy")]
    if [r["slot"] for r in slots] != list(range(1, expected_slots + 1)):
        raise ValueError("missing/duplicated fixed-grid slots")
    for row in slots:
        if row["scheduled_ns"] != start + round(row["slot"] * period * 1e9):
            raise ValueError("request grid moved post hoc")
    requests = [r for r in slots if r["event"] == "update_request"]
    outcomes = [r for r in rows if r["event"] == "commit_result"]
    if len(outcomes) != len(requests) or not all(r["committed"] for r in outcomes):
        raise ValueError("failed/unaccounted update attempt")
    validation = json.loads((folder / "capture_validation.json").read_text())
    if validation["errors"] or not validation["alignment_verified"]:
        raise ValueError("world capture validation failed")
    world = [r for r in read_rows(folder / "world_pose.jsonl")
             if start <= r["receipt_ns"] <= end]
    if len(world) < 20 or any(r["frame"] != "default" or r["child"] != "turtlebot3_waffle"
                             for r in world):
        raise ValueError("insufficient or misidentified world samples")
    gaps = [(b["sim_ns"] - a["sim_ns"]) / 1e9 for a, b in zip(world, world[1:])]
    if min(gaps) <= 0 or max(gaps) > .10:
        raise ValueError("world capture sequence/gap invalid")
    timeline = integrate(rows, start, end)
    metrics = update_metrics(rows, start, end)
    for metric in metrics:
        if metric["restored_before_supersession"] and (
                metric["commit_to_observed_ms"] is None or
                metric["commit_to_observed_ms"] > metric["commit_to_release_ms"]):
            raise ValueError("new binding published without a captured prior snapshot observation")
    intervals = [(b["commit_ns"] - a["commit_ns"]) / 1e9 for a, b in zip(metrics, metrics[1:])]
    return dict(trial=trial, window_s=duration,
        scheduled_slots=expected_slots, update_requests=len(requests),
        busy_slots=sum(r["event"] == "update_slot_busy" for r in slots),
        commit_successes_all=len(outcomes), commits_in_window=len(metrics),
        durable_commits_per_exposure_s=len(metrics) / duration,
        durable_interval_median_s=statistics.median(intervals) if intervals else None,
        durable_interval_range_s=[min(intervals), max(intervals)] if intervals else None,
        authorization_fractions=timeline["fractions"],
        mismatched_new_releases=timeline["mismatched_new_releases"], updates=metrics,
        recovered_after_window=recorded["recovery_after_window"],
        world_sample_count=len(world), world_max_sim_gap_s=max(gaps),
        world_displacement_m=math.hypot(world[-1]["x"] - world[0]["x"],
                                        world[-1]["y"] - world[0]["y"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.root / "manifest.json").read_text())
    source_ok = all(hashlib.sha256((args.root / "source" / path).read_bytes()).hexdigest() == sha
                    for path, sha in manifest["source_sha256"].items())
    completion = json.loads((args.root / "completion.json").read_text())
    results, failures = [], []
    for index, trial in enumerate(manifest["trials"], 1):
        try:
            results.append(audit_trial(args.root / f"trial_{index:02d}", trial,
                                       manifest["duration_s"]))
        except Exception as exc:
            failures.append(dict(index=index, trial=trial, error=repr(exc)))
    groups = {}
    for period in sorted({r["trial"]["period_s"] for r in results}, reverse=True):
        selected = [r for r in results if r["trial"]["period_s"] == period]
        updates = [u for r in selected for u in r["updates"]]
        release = [u["commit_to_release_ms"] for u in updates if u["restored_before_supersession"]]
        hold = [u["commit_to_hold_ms"] for u in updates if u["commit_to_hold_ms"] is not None]
        groups[str(period)] = dict(runs=len(selected), commits=len(updates),
            busy_slots=sum(r["busy_slots"] for r in selected),
            restored_before_supersession=len(release), censored=len(updates) - len(release),
            new_release_mismatches=sum(len(r["mismatched_new_releases"]) for r in selected),
            median_valid_fraction=statistics.median(r["authorization_fractions"]["valid"] for r in selected),
            median_stale_release_fraction=statistics.median(r["authorization_fractions"]["stale_release"] for r in selected),
            median_commit_to_release_ms=statistics.median(release) if release else None,
            median_commit_to_hold_ms=statistics.median(hold) if hold else None)
    passed = source_ok and completion["source_unchanged"] and not failures and completion["failures"] == 0
    report = dict(audit_passed=passed, pre_run_manifest=manifest["cohort"],
                  attempted=len(manifest["trials"]), valid=len(results), failures=failures,
                  source_hashes_verified=source_ok, groups=groups, trials=results,
                  caveat="stale released path is a revocation observation window, not valid authorization or a physical-motion proof")
    (args.root / "independent_audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(dict(audit_passed=passed, valid=len(results), failures=failures,
                         groups=groups), indent=2))
    return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
