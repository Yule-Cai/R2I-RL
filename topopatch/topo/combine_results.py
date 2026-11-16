"""Combine per-map TopoRulePatch eval records into aggregate tables.

Reads N records.jsonl files (one per evaluated map, all at the SAME frozen
tau) and reports:
  - aggregate metrics per method (micro-average over samples)
  - per-map table (unseen-map generalization claim)
  - per-complexity tables: by n_rules, by geom mix, by topo class
  - topology head: accuracy overall + hard-only, confusion sum, coverage
  - latency rollup (e2e vs Nav2 replan reference per map)

Usage:
  python3 -m topopatch.topo.combine_results rec1.jsonl ... \
      --out <dir> --nav2 manifest1.jsonl ... (optional, aligned by order)
"""
import argparse
import collections
import json
import os

import numpy as np

from topopatch.topo.coverage_curves import metrics_at_tau


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("records", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tau", type=float, default=float("-inf"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    all_recs = []
    for fp in a.records:
        with open(fp) as f:
            all_recs.extend(json.loads(l) for l in f)
    by_method = collections.defaultdict(list)
    for r in all_recs:
        by_method[r["method"]].append(r)
    out = {"tau": a.tau, "n_total": len(all_recs), "aggregate": {},
           "per_map": {}, "by_n_rules": {}, "by_topo_cls": {},
           "topo_head": {}}
    for m, rs in sorted(by_method.items()):
        out["aggregate"][m] = metrics_at_tau(rs, a.tau)
        pm = collections.defaultdict(list)
        for r in rs:
            pm[r["map_id"]].append(r)
        out["per_map"][m] = {k: metrics_at_tau(v, a.tau)
                             for k, v in sorted(pm.items())}
        pn = collections.defaultdict(list)
        for r in rs:
            pn[str(r.get("n_rules"))].append(r)
        out["by_n_rules"][m] = {k: metrics_at_tau(v, a.tau)
                                for k, v in sorted(pn.items())}
        pc = collections.defaultdict(list)
        for r in rs:
            pc[str(r.get("topo_cls"))].append(r)
        out["by_topo_cls"][m] = {k: metrics_at_tau(v, a.tau)
                                 for k, v in sorted(pc.items())}
    # topology head rollup over model records carrying topo_pred
    cm = np.zeros((3, 3), dtype=np.int64)
    n_hard = ok_hard = n_all = ok_all = 0
    for r in all_recs:
        if r.get("topo_pred") is None:
            continue
        t, p = int(r["topo_cls"]), int(r["topo_pred"])
        cm[t, p] += 1
        n_all += 1
        ok_all += (t == p)
        if t != 2:
            n_hard += 1
            ok_hard += (t == p)
    out["topo_head"] = {"confusion": cm.tolist(),
                        "acc": (ok_all / n_all) if n_all else None,
                        "acc_hard": (ok_hard / n_hard) if n_hard else None,
                        "n": n_all}
    json.dump(out, open(os.path.join(a.out, "combined.json"), "w"), indent=1)
    # compact console table
    print(f"{'method':16s} {'acc':>6s} {'allcase':>8s} {'fb':>6s} "
          f"{'nat_c':>6s}")
    for m, e in out["aggregate"].items():
        acs = e["all_case_success"]
        print(f"{m:16s} {e['verifier_acceptance_rate']:6.3f} "
              f"{(acs if acs is not None else float('nan')):8.3f} "
              f"{e['fallback_rate']:6.3f} "
              f"{e['native_rule_compliance']:6.3f}")
    print(json.dumps(out["topo_head"]))


if __name__ == "__main__":
    main()
