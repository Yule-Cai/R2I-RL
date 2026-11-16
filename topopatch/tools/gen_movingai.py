"""Freeze MovingAI zero-shot external benchmark (no training data use).

Protocol (frozen BEFORE any inference on these cases; record hash):
- Maps (SHA recorded): maze512-16-0, maze512-32-0 (corridor variation),
  random512-10-0, random512-20-0 (density variation), 32room_000 (rooms).
  MovingAI octile .map; blocked = '@OTSW'; CELL_M = 0.1 (frozen scale).
- Per map: rng seed 0; Dijkstra distance fields (early-stopped at
  16 m) from rng-ordered free starts; goals drawn from the 4-16 m
  geodesic annulus (straight-line >= 3 m); nominal = full-map Theta*.
- Rule: single RECT_HALF=0.75 rect centered at the nominal midpoint
  (same style as bench auto-placement; 1 rule/case).
- Teacher replan: full-map Theta* avoiding the rule (lethal).
- Anchors: _union_span (same frozen function as the deployed node,
  margin 3.0 m). Cases with no span are recorded as unaffected (kept:
  they test abstention/behavior, reported separately).
- GT: frozen repair_scope.repairability_gt (span/detour/complexity).
  Quota per map: first 20 LOCAL + first 20 NONLOCAL in rng order
  (no cherry-picking; shortfalls reported).
- Epoch blocks stride 10 from 30000 (no overlap with repairable
  9000-11600 or paper-matrix epochs).

Zero-shot: frozen weights, frozen tau/scope, no tuning on results.
Geometric evaluation only (no Gazebo claim from MovingAI alone).
"""
import argparse
import hashlib
import json
import math

import numpy as np

CELL_M = 0.1
RECT_HALF = 0.75
SEED = 0
LOCAL_Q = 20
NONLOCAL_Q = 20
EPOCH0 = 30000
EPOCH_STRIDE = 10

MAPS = ["maze512-16-0", "maze512-32-0", "random512-10-0",
        "random512-20-0", "32room_000"]


def load_map(path):
    lines = open(path).read().splitlines()
    h = int([l for l in lines if l.startswith("height")][0].split()[1])
    w = int([l for l in lines if l.startswith("width")][0].split()[1])
    i = next(i for i, l in enumerate(lines) if l.strip() == "map")
    rows = lines[i + 1:i + 1 + h]
    grid = np.zeros((h, w), np.int16)
    for y, r in enumerate(rows):
        for x, ch in enumerate(r[:w]):
            if ch in "@OTSW":
                grid[y, x] = 100
    return grid


def path_len(pts):
    p = np.asarray(pts, dtype=np.float64)
    return float(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])).sum())


def dijkstra_limited(blocked, s, max_dist):
    """8-connected Dijkstra (octile costs) early-stopped beyond
    max_dist (cells). Returns dist dict for reached cells."""
    import heapq
    H, W = blocked.shape
    dist = {s: 0.0}
    pq = [(0.0, s)]
    nbrs = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
            (0, -1, 1.0), (0, 1, 1.0),
            (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]
    while pq:
        d, cur = heapq.heappop(pq)
        if d > dist.get(cur, float("inf")) + 1e-12:
            continue
        if d > max_dist:
            break
        for dx, dy, c in nbrs:
            nb = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nb[0] < W and 0 <= nb[1] < H):
                continue
            if blocked[nb[1], nb[0]]:
                continue
            nd = d + c
            if nd < dist.get(nb, float("inf")) - 1e-12 and nd <= max_dist:
                dist[nb] = nd
                heapq.heappush(pq, (nd, nb))
    return dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps-dir", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    import os
    from topopatch.topo.classical_local import theta_star
    from topopatch.topo import repair_scope as RS
    from topopatch_ros.topo_inference_node import _union_span

    rng = np.random.default_rng(SEED)
    cases = []
    maps_rec = {}
    epoch = EPOCH0
    for mname in MAPS:
        fp = None
        for root, _, files in os.walk(a.maps_dir):
            if mname + ".map" in files:
                fp = os.path.join(root, mname + ".map")
                break
        assert fp, mname
        sha = hashlib.sha256(open(fp, "rb").read()).hexdigest()[:16]
        grid = load_map(fp)
        H, W = grid.shape
        blocked = grid > 50
        ox = oy = 0.0
        res = CELL_M
        maps_rec[mname] = {"map_sha256": sha, "cells": [H, W],
                           "cell_m": CELL_M}
        free = np.argwhere(~blocked)
        n_starts_used = 0
        n_local = n_nonlocal = 0
        tried = 0
        # Bounded search (dense maps may never fill quotas): caps keep
        # generation tractable; shortfalls are recorded honestly.
        # Deterministic given SEED (rng permutation order fixed).
        MAX_STARTS = 250
        MAX_TRIED = 600
        # rng-ordered starts; each yields a distance field, goals drawn
        # from its 4-16 m annulus (deterministic given SEED).
        start_order = rng.permutation(len(free))[:MAX_STARTS]
        max_d_cells = 16.0 / res
        min_d_cells = 4.0 / res
        for si_k in start_order:
            if n_local >= LOCAL_Q and n_nonlocal >= NONLOCAL_Q:
                break
            if tried >= MAX_TRIED:
                break
            si = free[int(si_k)]
            s_cell = (int(si[1]), int(si[0]))
            dist = dijkstra_limited(blocked, s_cell, max_d_cells)
            n_starts_used += 1
            cands = [(c, d) for c, d in dist.items()
                     if min_d_cells <= d <= max_d_cells]
            if not cands:
                continue
            order = rng.permutation(len(cands))
            for gi_k in order[:12]:
                if n_local >= LOCAL_Q and n_nonlocal >= NONLOCAL_Q:
                    break
                if tried >= MAX_TRIED:
                    break
                tried += 1
                (gx_c, gy_c), _ = cands[int(gi_k)]
                s = (float((s_cell[0] + 0.5) * res),
                     float((s_cell[1] + 0.5) * res))
                g = (float((gx_c + 0.5) * res),
                     float((gy_c + 0.5) * res))
                if math.hypot(s[0] - g[0], s[1] - g[1]) < 3.0:
                    continue
                nom = theta_star(blocked, ox, oy, res, s, g)
                if not nom:
                    continue
                L = path_len(nom)
                if not (4.0 <= L <= 16.0):
                    continue
                mid = nom[len(nom) // 2]
                rect = [(mid[0] - RECT_HALF, mid[1] - RECT_HALF),
                        (mid[0] - RECT_HALF, mid[1] + RECT_HALF),
                        (mid[0] + RECT_HALF, mid[1] + RECT_HALF),
                        (mid[0] + RECT_HALF, mid[1] - RECT_HALF)]
                sp = _union_span([(float(x), float(y)) for x, y in nom],
                                 [rect])
                if sp is None:
                    continue
                iA, iB = sp
                A, B = tuple(nom[iA]), tuple(nom[iB])
                # teacher replan avoiding the rule
                from topopatch.topo.classical_local import build_blocked
                b2, (ox2, oy2) = build_blocked(
                    grid, ox, oy, res, [rect], radius=0.1, window=None)
                repl = theta_star(b2, ox, oy, res, s, g)
                if not repl:
                    gt = {"repairable": RS.NONLOCAL, "span_m": 0.0,
                          "detour": float("inf"), "complexity": 2,
                          "infeasible": True}
                else:
                    seg_o = nom[iA:iB + 1]
                    g2 = RS.repairability_gt(seg_o, repl, A, B)
                    gt = {"repairable": g2["repairable"],
                          "span_m": g2["span_m"],
                          "detour": g2["detour"],
                          "complexity": g2["complexity"],
                          "infeasible": False}
                is_local = (gt["repairable"] == RS.LOCAL)
                if is_local and n_local >= LOCAL_Q:
                    continue
                if not is_local and n_nonlocal >= NONLOCAL_Q:
                    continue
                cases.append({
                    "uid": f"{mname}-{n_local + n_nonlocal:03d}",
                    "map": mname,
                    "start": [round(s[0], 3), round(s[1], 3)],
                    "goal": [round(g[0], 3), round(g[1], 3)],
                    "rules": [[[round(float(x), 3), round(float(y), 3)]
                               for x, y in rect]],
                    "A": [round(float(A[0]), 3), round(float(A[1]), 3)],
                    "B": [round(float(B[0]), 3), round(float(B[1]), 3)],
                    "gt_span_m": round(float(gt["span_m"]), 3),
                    "gt_detour": (round(float(gt["detour"]), 3)
                                  if math.isfinite(gt["detour"]) else None),
                    "gt_local": bool(is_local),
                    "gt_infeasible": bool(gt["infeasible"]),
                    "epoch": epoch,
                })
                epoch += EPOCH_STRIDE
                if is_local:
                    n_local += 1
                else:
                    n_nonlocal += 1
        print(f"[gen-movingai] {mname}: local={n_local} "
              f"nonlocal={n_nonlocal} tried={tried} "
              f"starts={n_starts_used}", flush=True)
        maps_rec[mname]["n_cases"] = n_local + n_nonlocal
    manifest = {
        "protocol": "movingai-zero-shot-v1",
        "cell_m": CELL_M, "rect_half_m": RECT_HALF, "seed": SEED,
        "quotas": {"local": LOCAL_Q, "nonlocal": NONLOCAL_Q},
        "maps": maps_rec, "n_cases": len(cases), "cases": cases,
    }
    blob = json.dumps(manifest, sort_keys=True).encode()
    manifest["manifest_sha256"] = hashlib.sha256(blob).hexdigest()
    json.dump(manifest, open(a.out, "w"), indent=1)
    print(f"[gen-movingai] n={len(cases)} sha={manifest['manifest_sha256'][:8]}")


if __name__ == "__main__":
    main()
