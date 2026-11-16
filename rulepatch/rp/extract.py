"""Offline patch extraction: teacher pairs -> A/B-anchored BEV samples.

Input: dataset shards (load_shards records with orig/repl/rect/start/goal/uid).
Output: per-split npz (bev uint8 4x128x128, local 32x2, tgt 6x2, meta) +
split json + extraction stats. Deterministic.
"""
import json
import os

import numpy as np

from . import (BEV_PX, BEV_SIZE_M, TEST_REGION_X_MIN, TRAIN_SPLIT, VAL_SPLIT,
               TEST_SPLIT, N_LOCAL_PTS)
from .bev import (render_bev, window_for_patch, local_path_points,
                  target_control_points)
from .dataset import load_shards
from .geom import (find_anchors, count_inside, path_length,
                   resample_polyline)


def load_occ_grid(pgm_path, res_out=0.1, res_in=0.03, ox=-15.1, oy=-25.0,
                  pgm_dir=None, walls_only=False):
    """Downsample map pgm to res_out occupancy grid.

    walls_only=False (default): walls + unknown treated blocked.
    walls_only=True: only lethal walls blocked; unknown free (matches Nav2
    allow_unknown=true used by the teacher planner).
    """
    """Downsample map pgm to res_out occupancy grid (1=occupied/unknown).

    res_in/ox/oy default to warehouse.yaml; override for other maps.
    pgm_dir resolves relative image paths (map yaml `image:` field).
    """
    from PIL import Image
    import os as _os
    if pgm_dir and not _os.path.isabs(pgm_path):
        pgm_path = _os.path.join(pgm_dir, pgm_path)
    img = np.asarray(Image.open(pgm_path).convert("L"))
    h, w = img.shape
    blocked = (img == 0) if walls_only else (img < 250)
    occ_full = blocked.astype(np.uint8)[::-1]  # row 0 = ymin
    f = max(1, int(round(res_out / res_in)))
    hh, ww = (h // f) * f, (w // f) * f
    grid = occ_full[:hh, :ww].reshape(hh // f, f, ww // f, f).max(axis=(1, 3))
    return {"grid": grid, "full": occ_full, "res": res_in * f,
            "res_in": res_in, "ox": ox, "oy": oy,
            "w": ww // f, "h": hh // f}


def make_occ_fn(occ):
    """Full-resolution occupancy query (matches planner grid, no downsample
    over-blocking). Row 0 = ymin."""
    g, res, ox, oy = occ["full"], occ["res_in"], occ["ox"], occ["oy"]
    h, w = g.shape

    def fn(x, y):
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if 0 <= ix < w and 0 <= iy < h:
            return bool(g[iy, ix])
        return True  # outside known map = blocked
    return fn


def extract_sample(rec, occ_fn, map_bounds, stats=None, fast=None):
    """Returns sample dict or (None, reason).

    fast=(occ_full, res_in, ox, oy) selects the vectorized BEV renderer;
    otherwise occ_fn per-pixel rendering is used (tests / ROS node).
    """
    def drop(reason):
        if stats is not None:
            stats[reason] = stats.get(reason, 0) + 1
        return None, reason
    orig = np.asarray(rec["orig"], dtype=np.float64)
    repl = np.asarray(rec["repl"], dtype=np.float64)
    rect = [tuple(map(float, p)) for p in rec["rect"]]
    goal = tuple(map(float, rec["goal"]))
    anc, reason = find_anchors(orig, repl)
    if anc is None:
        return drop(f"anchor:{reason}")
    iA, iB, jA, jB = anc["iA"], anc["iB"], anc["jA"], anc["jB"]
    A, B = anc["A"], anc["B"]
    orig_seg = orig[iA:iB + 1]
    repl_seg = repl[jA:jB + 1]
    if count_inside(orig_seg, rect) < 3:
        return drop("orig-unaffected")
    if count_inside(repl_seg, rect) > 0:
        return drop("teacher-violates")
    ol, rl = path_length(orig_seg), path_length(repl_seg)
    if ol < 1e-6 or rl / ol > 2.5:
        return drop("detour-ratio")
    center = window_for_patch(A, B, map_bounds)
    half = BEV_SIZE_M / 2.0 - 0.5
    if abs(A[0] - center[0]) > half or abs(A[1] - center[1]) > half \
            or abs(B[0] - center[0]) > half or abs(B[1] - center[1]) > half:
        return drop("span-overflow")
    if fast is not None:
        from .bev import render_bev_fast
        bev = render_bev_fast(fast[0], fast[1], fast[2], fast[3], center,
                              orig_seg, rect, goal)
    else:
        bev = render_bev(occ_fn, center, orig_seg, rect, goal)
    local = local_path_points(orig_seg, center)
    tgt = target_control_points(repl_seg, center)
    mid_x = (A[0] + B[0]) / 2.0
    if mid_x > TEST_REGION_X_MIN:
        split = TEST_SPLIT
    else:
        digits = "".join(c for c in rec.get("uid", "0") if c.isdigit())
        split = VAL_SPLIT if (int(digits[-2:]) % 10 == 0) else TRAIN_SPLIT
    return {
        "uid": rec.get("uid", "?"), "split": split,
        "bev": (np.clip(bev, 0, 1) * 255).astype(np.uint8),
        "local": local.astype(np.float32), "tgt": tgt.astype(np.float32),
        "center": np.array(center, np.float32),
        "A": np.array(A, np.float32), "B": np.array(B, np.float32),
        "rect": np.array(rect, np.float32), "goal": np.array(goal, np.float32),
        "orig_seg": np.array(orig_seg, np.float32),
        "repl_seg": np.array(repl_seg, np.float32),
        "orig_full": np.array(orig, np.float32),
        "repl_full": np.array(repl, np.float32),
        "idx": np.array([iA, iB, jA, jB], np.int64),
    }, "ok"


def run_extraction(data_dir, pgm_path, map_bounds, out_dir, res_in=0.03,
                   ox=-15.1, oy=-25.0):
    occ = load_occ_grid(pgm_path, res_in=res_in, ox=ox, oy=oy)
    occ_fn = make_occ_fn(occ)
    fast = (occ["full"], res_in, ox, oy)
    recs = load_shards(data_dir)
    stats: dict = {"input_pairs": len(recs)}
    by = {TRAIN_SPLIT: [], VAL_SPLIT: [], TEST_SPLIT: []}
    for rec in recs:
        s, reason = extract_sample(rec, occ_fn, map_bounds, stats, fast=fast)
        if s is None:
            continue
        by[s["split"]].append(s)
    os.makedirs(out_dir, exist_ok=True)
    counts = {}
    for split, items in by.items():
        # shard splits (<=1000 samples/file; single giant npz trips zipfile
        # limits with 10k+ entries)
        counts[split] = len(items)
        nsh = max(1, (len(items) + 999) // 1000)
        for s in range(nsh):
            chunk = items[s * 1000:(s + 1) * 1000]
            fp = os.path.join(out_dir, f"{split}_{s:04d}.npz")
            kw = {"n": np.array([len(chunk)])}
            for i, sm in enumerate(chunk):
                for k in ("bev", "local", "tgt", "center", "A", "B", "rect",
                          "goal", "orig_seg", "repl_seg", "orig_full",
                          "repl_full", "idx"):
                    kw[f"{k}_{i}"] = sm[k]
            kw["uids"] = np.array([sm["uid"] for sm in chunk])
            np.savez_compressed(fp, **kw)
    stats["output"] = counts
    with open(os.path.join(out_dir, "extraction_stats.json"), "w") as f:
        json.dump(stats, f, indent=1)
    return stats
