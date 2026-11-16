"""PGM occupancy -> Gazebo SDF static-wall world (paper sim eval).

Converts a Nav2 pgm (0 occupied / 254 free, matching topopatch maps)
into a Gazebo world of merged axis-aligned wall boxes (row-run-length
merging), so closed-loop physics can run on procedural warehouse /
office / maze / narrow maps with the SAME occupancy the planner uses.

Occupied threshold: value < 250 (walls; unknown 205 stays out, matching
the procedural convention free=254/wall=0/unknown=205 *blocked* in the
planner? NOTE: planner blocks unknown too, Gazebo only has walls —
documented mismatch: unknown cells are free in physics but blocked for
planning. Conservative direction (planner more cautious), reported.)

Usage:
  python3 topopatch/tools/pgm_to_sdf.py --pgm map.pgm --res 0.05 \
      --ox -10 --oy -10 --wall-h 2.0 --out world.sdf [--max-boxes 8000]
"""
import argparse

import numpy as np
from PIL import Image


def rle_boxes(occ):
    """Row-run-length merge of occupied cells -> (x0,y0,x1,y1) in cells
    (inclusive-exclusive), then vertical merge of identical runs."""
    h, w = occ.shape
    runs = []  # (y, x0, x1)
    for y in range(h):
        row = occ[y]
        x = 0
        while x < w:
            if row[x]:
                x0 = x
                while x < w and row[x]:
                    x += 1
                runs.append((y, x0, x))
            else:
                x += 1
    # vertical merge: same (x0,x1) on consecutive rows
    boxes = []
    open_runs = {}
    for y, x0, x1 in runs:
        key = (x0, x1)
        if key in open_runs and open_runs[key][0] == y - 1:
            open_runs[key] = (y, open_runs[key][1])
        else:
            if key in open_runs:
                y0old, y1old = open_runs.pop(key)
                boxes.append((key[0], y0old, key[1], y1old))
            open_runs[key] = (y, y)
    for (x0, x1), (y1, y0) in open_runs.items():
        boxes.append((x0, y0, x1, y1))
    return boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--wall-h", type=float, default=2.0)
    ap.add_argument("--max-boxes", type=int, default=8000)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    img = np.asarray(Image.open(a.pgm).convert("L"))
    h, w = img.shape
    occ = img < 250
    boxes = rle_boxes(occ)
    print(f"[pgm2sdf] occ_cells={int(occ.sum())} boxes={len(boxes)}")
    if len(boxes) > a.max_boxes:
        # coarsen by 2 until under budget (reported; physics res
        # differs from planning res then — last resort only)
        f = 2
        while len(boxes) > a.max_boxes:
            small = occ.reshape(h // f, f, w // f, f).max(axis=(1, 3))
            boxes = rle_boxes(small)
            print(f"[pgm2sdf] factor={f} boxes={len(boxes)}")
            f *= 2
            if f > 8:
                raise SystemExit("too many boxes even at 8x")
        res, note = a.res * (f // 2), f" (coarsened {f // 2}x)"
    else:
        res, note = a.res, ""
    print(f"[pgm2sdf] writing {a.out}{note}")
    L = []
    L.append('<?xml version="1.0" ?>')
    L.append('<sdf version="1.9">')
    L.append('<world name="topopatch_paper">')
    L.append('<physics name="1ms" type="dart">'
             '<max_step_size>0.001</max_step_size>'
             '<real_time_factor>1.0</real_time_factor></physics>')
    L.append('<scene><sky><clouds><speed>12</speed></clouds></sky></scene>')
    L.append('<light type="directional" name="sun">'
             '<cast_shadows>false</cast_shadows></light>')
    L.append('<ground_plane/>')
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        # pgm row 0 = top (ymax); world x right, y up
        wx0 = a.ox + x0 * res
        wx1 = a.ox + x1 * res
        wy1 = a.oy + (h - y0) * res
        wy0 = a.oy + (h - y1) * res
        cx, cy = (wx0 + wx1) / 2, (wy0 + wy1) / 2
        sx, sy = wx1 - wx0, wy1 - wy0
        L.append(
            f'<model name="wall{i}"><static>true</static>'
            f'<link name="link"><collision name="c"><geometry>'
            f'<box><size>{sx:.3f} {sy:.3f} {a.wall_h}</size></box>'
            f'</geometry></collision>'
            f'<visual name="v"><geometry>'
            f'<box><size>{sx:.3f} {sy:.3f} {a.wall_h}</size></box>'
            f'</geometry><material><ambient>0.6 0.6 0.6 1</ambient>'
            f'</material></visual>'
            f'<pose>{cx:.3f} {cy:.3f} {a.wall_h / 2:.3f} 0 0 0</pose>'
            f'</link></model>')
    L.append('</world>')
    L.append('</sdf>')
    open(a.out, "w").write("\n".join(L))
    print(f"[pgm2sdf] done: {len(boxes)} boxes")


if __name__ == "__main__":
    main()
