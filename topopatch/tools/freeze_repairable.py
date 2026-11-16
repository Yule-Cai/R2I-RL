"""Freeze the repairable-runtime benchmark manifest (paper freeze).

Selects cases from extracted TEST splits using ONLY frozen teacher
geometry (no model outputs anywhere in selection):
  LOCAL iff span_m <= 6.0 AND detour <= 1.8  (repair_scope constants)
Selection is deterministic (all LOCAL, sorted by uid); seeds recorded.
Each case: uid, map_id, map_sha256, pgm, res/ox/oy, start (orig_full[0]),
goal, RuleSpec polygons (trimmed), n_rules, geoms, GT span/detour/
complexity, frozen epoch, execution config (planner/controller).

Usage:
  python3 -m topopatch.tools.freeze_repairable --data <ext>... \
      --epoch0 9000 --epoch-step 100 --out manifest.json
Epoch blocks of 100 per input dir (unique across maps).
Writes manifest.json + manifest.sha256. COMMIT BEFORE any inference
on these cases; never remove hard cases afterwards.
"""
import argparse
import hashlib
import json

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", nargs="+", required=True)
    ap.add_argument("--epoch0", type=int, default=9000)
    ap.add_argument("--epoch-stride", type=int, default=10,
                    help="per-case epoch block (rules + removals fit "
                    "inside; manager requires strict increase)")
    ap.add_argument("--exec-planner", default="GridBased")
    ap.add_argument("--exec-controller", default="pp")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    from topopatch.topo.extract_topo import load_topo_split_items
    from topopatch.topo import repair_scope as RS
    cases = []
    maps = {}
    ecounter = [a.epoch0]
    for di, datadir in enumerate(a.data):
        items = load_topo_split_items(datadir, "test")
        meta = json.load(open(f"{datadir}/META.json"))
        tag = datadir.rstrip("/").split("/")[-1]
        maps[tag] = {"map_sha256": meta.get("map_sha256"),
                     "pgm": meta.get("pgm"),
                     "n_test": len(items)}
        for s in sorted(items, key=lambda x: str(x["uid"])):
            g = RS.repairability_gt(s["orig_seg"], s["repl_seg"],
                                    tuple(s["A"]), tuple(s["B"]))
            if g["repairable"] != RS.LOCAL:
                continue
            n = int(np.asarray(s["polys_n"]).reshape(-1)[0])
            polys = [np.asarray(s["polys"][j], dtype=np.float64).tolist()
                     for j in range(n)]
            o0 = np.asarray(s["orig_full"], dtype=np.float64)[0]
            cases.append({
                "uid": str(s["uid"]), "map_tag": tag,
                "map_id": str(s["map_id"]),
                "start": [round(float(o0[0]), 3), round(float(o0[1]), 3)],
                "goal": [round(float(v), 3) for v in
                         np.asarray(s["goal"],
                                    dtype=np.float64).tolist()],
                "rules": polys, "n_rules": n,
                "geoms": str(s["geoms"]),
                "gt_span_m": round(g["span_m"], 3),
                "gt_detour": round(g["detour"], 3),
                "gt_complexity": int(g["complexity"]),
                "epoch": ecounter[0],
            })
            ecounter[0] += a.epoch_stride
    manifest = {
        "protocol": "repairable-runtime-benchmark-v1",
        "selection": "all TEST samples with span_m<=6.0 AND detour<=1.8 "
                     "(frozen repair_scope constants); no model outputs used",
        "scope_threshold": {"rep_prob": 0.5, "span_window_m": 10.0},
        "exec": {"planner": a.exec_planner,
                 "controller": a.exec_controller},
        "maps": maps,
        "n_cases": len(cases),
        "cases": cases,
    }
    blob = json.dumps(manifest, sort_keys=True).encode()
    sha = hashlib.sha256(blob).hexdigest()
    manifest["manifest_sha256"] = sha
    json.dump(manifest, open(a.out, "w"), indent=1)
    open(a.out + ".sha256", "w").write(sha + f"  {a.out}\n")
    print(f"[freeze-repairable] n={len(cases)} sha={sha}")
    by_map = {}
    for c in cases:
        by_map[c["map_tag"]] = by_map.get(c["map_tag"], 0) + 1
    print("by_map:", by_map)


if __name__ == "__main__":
    main()
