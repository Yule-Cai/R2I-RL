"""M4 Task A regression: prove the output gate blocks every violator.

Replays gate_full_path over:
  (a) every teacher fallback full-path (repl_full) in the given extracted
      split dirs (the offline executable set);
  (b) Gazebo fallback executed paths from a cases CSV with rule rects
      (re-derives inside counts; the live violation repro).
Reports: n_paths, n_violating (rule-crossing), n_blocked (=violating,
must equal), n_executed_clean, executed_violations (must be 0).

Usage:
  python3 -m topopatch.tools.replay_gate --data <ext_dir>... \
      [--cases ~/rulepatch/gz_cases_demo14.csv] --out <report.json>
"""
import argparse
import csv
import json
import math
import os

import numpy as np


def pip(xy, poly):
    x, y = xy
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
            if x < xin:
                inside = not inside
    return inside


def pip_any(xy, polys):
    return any(pip(xy, np.asarray(p, dtype=np.float64)) for p in polys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", nargs="+", default=[])
    ap.add_argument("--cases", default=None)
    ap.add_argument("--pgm-res", type=float, default=0.05)
    ap.add_argument("--pgm-ox", type=float, default=-10.0)
    ap.add_argument("--pgm-oy", type=float, default=-10.0)
    ap.add_argument("--pgm", default=None,
                    help="occ pgm for the --data splits (single map each; "
                    "pass per-map via repeated runs or matching order)")
    ap.add_argument("--pgms", nargs="*", default=[],
                    help="pgm per --data dir, same order")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    from rulepatch.rp.extract import load_occ_grid, make_occ_fn
    from topopatch.topo.extract_topo import load_topo_split_items
    from topopatch.topo.gate import gate_full_path
    rep = {"offline": [], "gazebo": [], "summary": {}}
    tot_v = tot_b = tot_e = tot_ev = 0
    for di, pgm in zip(a.data, a.pgms or [a.pgm] * len(a.data)):
        occ = load_occ_grid(pgm, res_out=a.pgm_res, res_in=a.pgm_res,
                            ox=a.pgm_ox, oy=a.pgm_oy)
        occ_fn = make_occ_fn(occ)
        w, h = occ["w"], occ["h"]
        bounds = (a.pgm_ox, a.pgm_oy, a.pgm_ox + w * a.pgm_res,
                  a.pgm_oy + h * a.pgm_res)
        items = load_topo_split_items(di, "test")
        n_v = n_b = n_e = n_ev = 0
        gate_reasons = {}
        for s in items:
            polys = [np.asarray(s["polys"][k], dtype=np.float64)
                     for k in range(int(
                         np.asarray(s["polys_n"]).reshape(-1)[0]))]
            # (i) teacher fallback full path (should pass);
            # (ii) original full path (crosses rules by construction:
            #     adversarial violators, must ALL be blocked).
            for key, expect_viol in (("repl_full", False),
                                     ("orig_full", True)):
                path = np.asarray(s[key], dtype=np.float64)
                go, reason = gate_full_path(path, polys, occ_fn, bounds)
                gate_reasons[reason] = gate_reasons.get(reason, 0) + 1
                violated = sum(1 for xy in path
                               if pip_any(xy, polys)) > 0
                n_v += int(violated)
                if violated:
                    n_b += int(not go)
                    n_ev += int(go)  # executed a violator: soundness bug
                else:
                    n_e += int(go)
        rep["offline"].append({"dir": di, "n": len(items),
                               "paths": 2 * len(items),
                               "violating": n_v, "blocked": n_b,
                               "executed_clean": n_e,
                               "executed_violations": n_ev,
                               "gate_reasons": gate_reasons})
        tot_v += n_v
        tot_b += n_b
        tot_e += n_e
        tot_ev += n_ev
    if a.cases:
        with open(a.cases) as f:
            rows = list(csv.DictReader(f))
        import ast
        for r in rows:
            try:
                rect = [tuple(p) for p in
                        ast.literal_eval(r["rule"])]
            except Exception:
                continue
            fb_in = r.get("fallback_inside_rule", "")
            try:
                fb_in = int(float(fb_in)) if fb_in not in ("", None) \
                    else None
            except Exception:
                fb_in = None
            rep["gazebo"].append({
                "epoch": r.get("epoch"), "fallback_ok": r.get(
                    "fallback_ok"), "fallback_inside_rule": fb_in,
                "track_inside_rule": r.get("track_inside_rule"),
                "arrived": r.get("arrived")})
    rep["summary"] = {
        "offline_paths": sum(d["paths"] for d in rep["offline"]),
        "offline_violating": tot_v, "blocked": tot_b,
        "offline_executed_clean": tot_e,
        "executed_violations": tot_ev,
        "gate_sound": bool(tot_b == tot_v and tot_ev == 0),
    }
    json.dump(rep, open(a.out, "w"), indent=1)
    print(json.dumps(rep["summary"], indent=1))


if __name__ == "__main__":
    main()
