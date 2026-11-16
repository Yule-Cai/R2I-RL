"""Score candidate KEEP_OUT zones offline with the exact model+verifier.

Given a live Nav2 path (json list), map pgm+geometry, and fused weights,
tries frac x lateral x size candidates and reports the best zone
(verifier-accepted preferred, else highest-confidence). The demo client
then commits the winner live via --keepout (same code path decides).

Usage:
  PYTHONPATH=rulepatch python3 rulepatch/tools/pick_zone_offline.py \
      --path path.json --pgm <map.pgm> --res-in 0.05 --ox -10 --oy -10 \
      --weights <best_fused.npz> --out zone.json
"""
import argparse
import itertools
import json

import numpy as np

from rp.extract import load_occ_grid, make_occ_fn
from rp.infer_numpy import NumpyPatchNet
from rp.verifier import verify_patch, decode_ctrl
from rp.bev import (render_bev_fast, window_for_patch, local_path_points,
                    world_to_norm, add_coords)
from rp.geom import count_inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", required=True)
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res-in", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fracs", type=float, nargs="+",
                    default=[0.35, 0.5, 0.65])
    ap.add_argument("--laterals", type=float, nargs="+",
                    default=[0.0, 0.15, -0.15, 0.3, -0.3])
    ap.add_argument("--sizes", type=float, nargs="+", default=[0.6, 0.8])
    ap.add_argument("--margin-m", type=float, default=2.0)
    a = ap.parse_args()

    path = [tuple(p) for p in json.load(open(a.path))]
    occ = load_occ_grid(a.pgm, res_in=a.res_in, ox=a.ox, oy=a.oy)
    occ_fn = make_occ_fn(occ)
    fast = (occ["full"], a.res_in, a.ox, a.oy)
    net = NumpyPatchNet(a.weights)
    import math
    H = len(path)
    results = []
    for frac, lat, size in itertools.product(a.fracs, a.laterals, a.sizes):
        i = min(H - 1, max(1, int(H * frac)))
        cx, cy = path[i]
        j0, j1 = max(0, i - 5), min(H - 1, i + 5)
        dx = path[j1][0] - path[j0][0]
        dy = path[j1][1] - path[j0][1]
        L = math.hypot(dx, dy) + 1e-9
        cx += -dy / L * lat
        cy += dx / L * lat
        w = h = size
        rect = [(cx - w / 2, cy - h / 2), (cx - w / 2, cy + h / 2),
                (cx + w / 2, cy + h / 2), (cx + w / 2, cy - h / 2)]
        if count_inside(path, rect) < 5:
            continue
        # anchors like live inference (entry/exit + margin_m)
        inside = [_pip(p, rect) for p in path]
        i0 = inside.index(True)
        i1 = len(inside) - 1 - inside[::-1].index(True)
        acc, iA = 0.0, i0
        while iA > 0 and acc < a.margin_m:
            acc += math.hypot(path[iA][0] - path[iA - 1][0],
                               path[iA][1] - path[iA - 1][1])
            iA -= 1
        acc, iB = 0.0, i1
        while iB < H - 1 and acc < a.margin_m:
            acc += math.hypot(path[iB + 1][0] - path[iB][0],
                               path[iB + 1][1] - path[iB][1])
            iB += 1
        A, B = tuple(path[iA]), tuple(path[iB])
        fh, fw = fast[0].shape
        bounds = (a.ox, a.oy, a.ox + fw * a.res_in, a.oy + fh * a.res_in)
        center = window_for_patch(A, B, bounds)
        half = 12.8 / 2.0 - 0.5
        if abs(A[0] - center[0]) > half or abs(A[1] - center[1]) > half \
                or abs(B[0] - center[0]) > half \
                or abs(B[1] - center[1]) > half:
            continue
        seg = path[iA:iB + 1]
        goal = path[-1]
        bev = render_bev_fast(fast[0], fast[1], fast[2], fast[3], center,
                              seg, rect, goal)
        bev = add_coords(bev.astype(np.float64))
        local = local_path_points(seg, center)
        ctrls, confs = net.forward(bev, local.astype(np.float64))
        order = list(np.argsort(-confs))
        verdicts = []
        for k in order:
            ok, reason = verify_patch(ctrls[k], center, A, B, seg, rect,
                                      occ_fn, bounds)
            verdicts.append({"hyp": int(k), "conf": float(confs[k]),
                             "ok": bool(ok), "reason": reason})
        acc_any = next((v for v in verdicts if v["ok"]), None)
        results.append({
            "frac": frac, "lateral": lat, "size": size,
            "rect": rect, "center": center, "A": A, "B": B,
            "accepted": acc_any is not None,
            "accepted_hyp": acc_any["hyp"] if acc_any else None,
            "top_conf": float(confs[order[0]]),
            "verdicts": verdicts,
        })
    results.sort(key=lambda r: (r["accepted"], r["top_conf"]), reverse=True)
    out = {"candidates": len(results), "results": results[:12],
           "best": results[0] if results else None}
    json.dump(out, open(a.out, "w"), indent=1)
    b = out["best"]
    print(json.dumps({"candidates": len(results),
                      "best_accepted": b["accepted"] if b else None,
                      "best": {k: b[k] for k in
                               ("frac", "lateral", "size", "rect",
                                "accepted_hyp") if b} if b else None},
                     indent=1))


def _pip(xy, pts) -> bool:
    x, y = float(xy[0]), float(xy[1])
    inside, n = False, len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if ((y0 > y) != (y1 > y)) and (
                x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0):
            inside = not inside
    return inside


if __name__ == "__main__":
    main()
