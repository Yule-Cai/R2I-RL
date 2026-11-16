"""Sample deterministic evaluation goals from a Nav2 pgm (paper matrix).

Picks K free cells with clearance >= clear_m, spread by farthest-point
sampling, within [dist_min, dist_max] of start. Writes goals.json [[x,y]].

Usage:
  python3 topopatch/tools/paper_goals.py --pgm map.pgm --res 0.05 \
      --ox -10 --oy -10 --clear-m 0.5 --k 4 --seed 0 --out goals.json
"""
import argparse
import json
import math
import random

import numpy as np
from PIL import Image


def clearance(img, res, clear_m):
    free = (np.asarray(img) >= 254).astype(np.uint8)
    h, w = free.shape
    r = max(1, int(round(clear_m / res)))
    sat = np.pad(free, r + 1, mode="constant").astype(np.int64)
    sat = sat.cumsum(0).cumsum(1)
    A = sat[2 * r + 2:, 2 * r + 2:]
    B = sat[1:-(2 * r + 1), 2 * r + 2:]
    C = sat[2 * r + 2:, 1:-(2 * r + 1)]
    D = sat[1:-(2 * r + 1), 1:-(2 * r + 1)]
    return (A - B - C + D) == (2 * r + 1) ** 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--clear-m", type=float, default=0.5)
    ap.add_argument("--k", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--start", default=None,
                    help="x,y fixed start; default=auto max-clearance cell")
    ap.add_argument("--dist-min", type=float, default=3.0)
    ap.add_argument("--dist-max", type=float, default=10.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    img = Image.open(a.pgm).convert("L")
    w, h = img.size
    mask = clearance(img, a.res, a.clear_m)
    from scipy.ndimage import distance_transform_edt
    free = (np.asarray(img) >= 254)
    dt = distance_transform_edt(free) * a.res
    ys, xs = np.where(mask)
    if a.start:
        sx, sy = (float(v) for v in a.start.split(","))
    else:
        best, bd = None, -1.0
        for x, y in zip(xs.tolist(), ys.tolist()):
            if dt[y, x] > bd:
                bd = dt[y, x]
                best = (x, y)
        sx = a.ox + (best[0] + 0.5) * a.res
        sy = a.oy + (h - 1 - best[1] + 0.5) * a.res
        print(f"auto start=({sx:.2f},{sy:.2f}) clear={bd:.2f}")
    pts = []
    for x, y in zip(xs.tolist(), ys.tolist()):
        wx = a.ox + (x + 0.5) * a.res
        wy = a.oy + (h - 1 - y + 0.5) * a.res
        d = math.hypot(wx - sx, wy - sy)
        if a.dist_min <= d <= a.dist_max:
            pts.append((wx, wy))
    assert len(pts) > a.k, f"only {len(pts)} candidates"
    rng = random.Random(a.seed)
    chosen = [rng.choice(pts)]
    while len(chosen) < a.k:
        best, bd = None, -1.0
        for p in pts:
            d = min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in chosen)
            if d > bd:
                best, bd = p, d
        chosen.append(best)
    chosen = [[round(float(x), 2), round(float(y), 2)] for x, y in chosen]
    out = {"start": [round(sx, 2), round(sy, 2)], "goals": chosen,
           "pgm": a.pgm, "clear_m": a.clear_m, "seed": a.seed}
    json.dump(out, open(a.out, "w"), indent=1)
    print(json.dumps(out))


if __name__ == "__main__":
    main()
