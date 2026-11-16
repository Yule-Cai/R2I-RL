"""Selective-coverage curves from eval records (no recompute).

Input: records.jsonl from evaluate_topo.py --rollout (must contain
max_conf, accepted, reason, closed_loop, closed_loop_fb, native, len_ratio,
len_ratio_fb, topo_cls, n_rules, geoms).

Semantics: at threshold tau a model sample is ABSTAINED when
max_conf < tau; its outcome is then the fallback branch (teacher replan).
Non-model methods (copy/teacher/deform, max_conf=None) ignore tau.

Outputs <out_dir>/curves.json {method: {taus:[...], coverage, acceptance,
fallback, all_case_success, accepted_success, efficiency}} and
<out_dir>/operating_point.json selected on THESE records (intended: VAL):
argmax all-case success, tie -> max coverage.

Usage:
  python3 -m topopatch.topo.coverage_curves --records <jsonl> \
      --method model:full --out <dir> [--n-tau 21]
"""
import argparse
import json
import os

import numpy as np


def load_records(path):
    with open(path) as f:
        return [json.loads(l) for l in f]


def metrics_at_tau(recs, tau, rep_gate=False):
    n = len(recs)
    cov = acc = fb = cl_acc = cl_fb = 0
    nat_c = nat_v = 0
    lrs = []
    for r in recs:
        mc = r.get("max_conf")
        abst = mc is not None and mc < tau
        if rep_gate and r.get("rep_prob") is not None \
                and r["rep_prob"] < 0.5:
            abst = True  # M4 scope: predicted NONLOCAL -> safe fallback
        accepted = (not abst) and bool(r["accepted"])
        if not abst:
            cov += 1
        nat = r.get("native", {})
        nat_c += bool(nat.get("compliant"))
        nat_v += bool(nat.get("valid"))
        if abst or not accepted:
            fb += 1
            cl_fb += bool(r.get("closed_loop_fb"))
            lrs.append(r.get("len_ratio_fb"))
        else:
            acc += 1
            cl_acc += bool(r.get("closed_loop"))
            lrs.append(r.get("len_ratio"))
    lrs = [v for v in lrs if v is not None]
    return {
        "tau": float(tau),
        "coverage": cov / n,
        "verifier_acceptance_rate": acc / n,
        "fallback_rate": fb / n,
        "all_case_success": (cl_acc + cl_fb) / n,
        "accepted_success": (cl_acc / acc) if acc else None,
        "fallback_success": (cl_fb / fb) if fb else None,
        "native_rule_compliance": nat_c / n,  # tau-independent
        "native_valid_patch_rate": nat_v / n,
        "len_ratio_mean": float(np.mean(lrs)) if lrs else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", required=True)
    ap.add_argument("--method", default=None,
                    help="only this method (default: all model:* + teacher)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--n-tau", type=int, default=21)
    ap.add_argument("--rep-gate", action="store_true",
                    help="M4 scope: treat rep_prob<0.5 as abstention")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    recs = load_records(a.records)
    by_method = {}
    for r in recs:
        by_method.setdefault(r["method"], []).append(r)
    methods = ([a.method] if a.method else
               [m for m in by_method if m.startswith("model:")] + ["teacher"])
    curves = {}
    for m in methods:
        rs = by_method.get(m, [])
        if not rs:
            continue
        mcs = [r["max_conf"] for r in rs if r.get("max_conf") is not None]
        if mcs:
            taus = ([float("-inf")] + sorted(
                np.quantile(mcs, np.linspace(0, 1, a.n_tau)).tolist()))
        else:
            taus = [float("-inf")]
        pts = [metrics_at_tau(rs, t, rep_gate=a.rep_gate)
               for t in taus]
        curves[m] = pts
    json.dump(curves, open(os.path.join(a.out, "curves.json"), "w"),
              indent=1)
    # operating point on these records (caller passes VAL records).
    op = {}
    for m, pts in curves.items():
        if not m.startswith("model:"):
            continue
        best = max(pts, key=lambda p: (p["all_case_success"] or -1,
                                       p["coverage"]))
        op[m] = {"tau": best["tau"],
                 "all_case_success": best["all_case_success"],
                 "coverage": best["coverage"],
                 "verifier_acceptance_rate":
                     best["verifier_acceptance_rate"],
                 "fallback_rate": best["fallback_rate"]}
    json.dump(op, open(os.path.join(a.out, "operating_point.json"), "w"),
              indent=1)
    print(json.dumps(op, indent=1))


if __name__ == "__main__":
    main()
