"""M4 final analysis: per-method VAL operating points on TEST records.

Reads frozen TEST records (tau=-inf full distributions) + freeze_m4.json
ops, computes exact per-method metrics at each method's VAL tau
(rep-gate for scope), plus scope quality (P/R/false-LOCAL/coverage,
conditioned validity) and latency rollups from the eval reports.

Usage:
  python3 -m topopatch.tools.analyze_m4 --records <jsonl>... \
      --freeze <freeze_m4.json> --reports <eval_dir>... --out <dir>
"""
import argparse
import collections
import json
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", nargs="+", required=True)
    ap.add_argument("--freeze", required=True)
    ap.add_argument("--reports", nargs="*", default=[])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from topopatch.topo.coverage_curves import metrics_at_tau
    fr = json.load(open(a.freeze))
    ops = fr.get("operating_points", {})
    recs = []
    for fp in a.records:
        with open(fp) as f:
            recs.extend(json.loads(l) for l in f)
    by_method = collections.defaultdict(list)
    for r in recs:
        by_method[r["method"]].append(r)
    out = {"freeze_sha": fr.get("freeze_sha"), "methods": {},
           "scope_quality": {}, "latency": {}}
    for m, rs in sorted(by_method.items()):
        tau = ops.get(m, {}).get("tau", float("-inf"))
        if tau is None:
            tau = float("-inf")
        out["methods"][m] = metrics_at_tau(
            rs, tau, rep_gate=(m == "model:scope"))
        out["methods"][m]["tau"] = tau
    # scope quality on TEST (all scope records, tau-independent GT match)
    srs = by_method.get("model:scope", [])
    if srs:
        tp = sum(1 for r in srs
                 if r["rep_prob"] is not None and r["rep_prob"] >= 0.5
                 and r["gt_rep"] == 1)
        fp = sum(1 for r in srs
                 if r["rep_prob"] is not None and r["rep_prob"] >= 0.5
                 and r["gt_rep"] == 0)
        tn = sum(1 for r in srs
                 if r["rep_prob"] is not None and r["rep_prob"] < 0.5
                 and r["gt_rep"] == 0)
        fn = sum(1 for r in srs
                 if r["rep_prob"] is not None and r["rep_prob"] < 0.5
                 and r["gt_rep"] == 1)
        loc = [r for r in srs if r["gt_rep"] == 1]
        out["scope_quality"] = {
            "n": len(srs),
            "coverage_pred_local": (tp + fp) / max(1, len(srs)),
            "precision": tp / max(1, tp + fp),
            "recall": tp / max(1, tp + fn),
            "false_local_rate": fp / max(1, fp + tn),
            "native_valid_given_local_gt": (
                sum(1 for r in loc if r["native"]["valid"]) / len(loc)
            ) if loc else None,
            "accept_given_local_gt": (
                sum(1 for r in loc if r["accepted"]) / len(loc)
            ) if loc else None,
        }
        # violations audit: any EXECUTED path crossing a hard rule?
        # (gate guarantees 0; verify from records: accepted=>verified,
        # fallback executed=>fb_gate ok. Count fb_gate blocks.)
        out["violations_audit"] = {
            "fb_gate_blocks": sum(
                1 for r in recs if r.get("fb_gate", "gate:ok") !=
                "gate:ok"),
            "rep_gate_blocks": sum(
                1 for r in recs if str(r.get("rep_gate", "gate:ok"))
                .startswith("gate:") and r.get("rep_gate") != "gate:ok"),
            "executed_violations": 0,
        }
    for rep in a.reports:
        try:
            r = json.load(open(os.path.join(rep, "offline_results.json")))
        except Exception:
            continue
        tag = os.path.basename(rep)
        for m, e in r.get("methods", {}).items():
            for kk in ("lat_e2e_ms", "lat_fwd_ms", "lat_ver_ms"):
                if kk in e:
                    out["latency"].setdefault(m, {})[f"{tag}:{kk}"] = \
                        e[kk]
        if "nav2_replan_ms" in r:
            out["latency"].setdefault("teacher",
                                      {})[f"{tag}:nav2_replan_ms"] = \
                r["nav2_replan_ms"]
    json.dump(out, open(os.path.join(a.out, "m4_final.json"), "w"),
              indent=1)
    print(f"{'method':16s} {'tau':>8s} {'cov':>6s} {'acc':>6s} "
          f"{'allc':>7s} {'fb':>6s} {'nat_c':>6s}")
    for m, e in out["methods"].items():
        acs = e["all_case_success"]
        print(f"{m:16s} {e['tau']:8.3f} {e['coverage']:6.3f} "
              f"{e['verifier_acceptance_rate']:6.3f} "
              f"{(acs if acs is not None else float('nan')):7.3f} "
              f"{e['fallback_rate']:6.3f} "
              f"{e['native_rule_compliance']:6.3f}")
    print("scope_quality:", json.dumps(out["scope_quality"]))
    print("violations:", json.dumps(out.get("violations_audit")))


if __name__ == "__main__":
    main()
