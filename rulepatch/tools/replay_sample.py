"""Deterministic h1-fail/h2-ok demonstration on a held-out test sample.

Scans the extracted test split with the fused model + verifier and reports
the first sample where the top-confidence hypothesis is rejected but a
lower-confidence hypothesis is accepted. Saves the verdict + arrays as
demo evidence. No ROS needed (same rp.* code path as the live node).

Usage:
  PYTHONPATH=rulepatch python3 rulepatch/tools/replay_sample.py \
      --data <extracted> --pgm <warehouse.pgm> --weights <best_fused.npz> \
      --out /tmp/rulepatch_replay.json
"""
import argparse
import json

import numpy as np

from rp.extract import load_occ_grid, make_occ_fn
from rp.infer_numpy import NumpyPatchNet
from rp.verifier import verify_patch, decode_ctrl

MAP_BOUNDS = (-15.1, -25.0, 15.08, 25.22)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-scan", type=int, default=2000)
    a = ap.parse_args()

    occ = load_occ_grid(a.pgm)
    occ_fn = make_occ_fn(occ)
    net = NumpyPatchNet(a.weights)
    from rp.dataset import load_split
    from rp.model import add_coords
    samples = load_split(a.data, "test")
    n = len(samples)
    found = None
    for i, s in enumerate(samples[:a.max_scan]):
        bev = add_coords(s["bev"].astype(np.float64) / 255.0)
        local = s["local"].astype(np.float64)
        ctrls, confs = net.forward(bev, local)
        center = tuple(s["center"])
        A, B = tuple(s["A"]), tuple(s["B"])
        seg = s["orig_seg"]
        rect = [tuple(p) for p in s["rect"]]
        order = list(np.argsort(-confs))
        verdicts = []
        for k in order:
            ok, reason = verify_patch(ctrls[k], center, A, B, seg, rect,
                                      occ_fn, MAP_BOUNDS)
            verdicts.append({"hyp": int(k), "conf": float(confs[k]),
                             "ok": bool(ok), "reason": reason})
        if not verdicts[0]["ok"] and any(v["ok"] for v in verdicts[1:]):
            acc = next(v for v in verdicts[1:] if v["ok"])
            found = {"uid": s["uid"], "index": i,
                     "verdicts": verdicts, "accepted_hyp": acc["hyp"]}
            np.savez(a.out + ".npz", bev=s["bev"],
                     local=s["local"], tgt=s["tgt"],
                     ctrls=ctrls, confs=confs)
            break
    if found is None:
        print(json.dumps({"found": False, "scanned": min(n, a.max_scan)}))
    else:
        found["found"] = True
        print(json.dumps(found, indent=1))
    json.dump(found or {"found": False}, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
