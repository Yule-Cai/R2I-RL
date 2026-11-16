"""MovingAI zero-shot paired eval (learned deployed arm vs Theta*).

Reconstructs each scenario's nominal (full-map Theta*), slices the
span via the frozen _union_span, then runs the identical pipelines as
paired_eval.py: BEV render + NumpyTopoNet + frozen tau/scope vs
windowed Theta*, same strengthened gate.

Zero-shot: frozen weights/thresholds, no tuning on these maps.
"""
import csv
import json
import math
import time

import numpy as np

TAU_SCOPE = 1.8216713190078737
CELL_M = 0.1


def path_len(pts):
    p = np.asarray(pts, dtype=np.float64)
    return float(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])).sum())


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


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--maps-dir", required=True)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    import os
    from rulepatch.rp.bev import (add_coords, local_path_points,
                                  render_bev, _rasterize_polygon)
    from rulepatch.rp.geom import point_in_polygon
    from topopatch.topo.classical_local import (repair_local_thetastar,
                                                theta_star)
    from topopatch.topo.extract_topo import _rule_feats
    from topopatch.topo.infer_numpy_topo import NumpyTopoNet
    from topopatch.topo.verifier_topo import select_topo

    def _union_span_frozen(path, polys, margin_m=3.0):
        # Frozen M2 crossing_anchors logic (copied to avoid an rclpy
        # import in offline eval): entry/exit indices widened by
        # margin arc-length; union span over polys.
        import math as _m
        spans = []
        for poly in polys:
            inside = [bool(point_in_polygon(p, np.asarray(
                poly, dtype=np.float64))) for p in path]
            if not any(inside):
                continue
            i0 = inside.index(True)
            i1 = len(inside) - 1 - inside[::-1].index(True)
            acc, iA = 0.0, i0
            while iA > 0 and acc < margin_m:
                acc += _m.hypot(path[iA][0] - path[iA - 1][0],
                                path[iA][1] - path[iA - 1][1])
                iA -= 1
            acc, iB = 0.0, i1
            while iB < len(path) - 1 and acc < margin_m:
                acc += _m.hypot(path[iB + 1][0] - path[iB][0],
                                path[iB + 1][1] - path[iB][1])
                iB += 1
            spans.append((max(0, iA), min(len(path) - 1, iB)))
        if not spans:
            return None
        return min(s[0] for s in spans), max(s[1] for s in spans)

    m = json.load(open(a.manifest))
    net = NumpyTopoNet(a.ckpt)
    grids = {}

    def grid_of(mapname):
        if mapname not in grids:
            fp = None
            for root, _, files in os.walk(a.maps_dir):
                if mapname + ".map" in files:
                    fp = os.path.join(root, mapname + ".map")
                    break
            assert fp, mapname
            grids[mapname] = load_map(fp)
        return grids[mapname]

    rows = []
    for c in m["cases"]:
        grid = grid_of(c["map"])
        blocked = grid > 50
        ox = oy = 0.0
        res = CELL_M
        H, W = grid.shape
        bounds = (0.0, 0.0, W * res, H * res)
        s = tuple(c["start"])
        g = tuple(c["goal"])
        rects = [[(float(x), float(y)) for x, y in poly]
                 for poly in c["rules"]]
        row = {"uid": c["uid"], "map": c["map"],
               "gt_local": c["gt_local"],
               "gt_span_m": c.get("gt_span_m"),
               "gt_detour": c.get("gt_detour")}
        t0 = time.time()
        try:
            nom = theta_star(blocked, ox, oy, res, s, g)
            if not nom:
                row["nominal"] = "no-path"
                rows.append(row)
                continue
            row["teacher_len"] = round(path_len(nom), 3)
            sp = _union_span_frozen([(float(x), float(y)) for x, y in nom],
                                      rects)
            if sp is None:
                row["learned_ok"] = False
                row["learned_reason"] = "unaffected-path"
                row["classical_ok"] = False
                row["classical_reason"] = "unaffected-path"
                rows.append(row)
                continue
            iA, iB = sp
            A, B = tuple(nom[iA]), tuple(nom[iB])
            seg = np.asarray(nom[iA:iB + 1], dtype=np.float64)
            center = ((A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0)

            def occ_fn(x, y):
                ix = int((x - ox) / res)
                iy = int((y - oy) / res)
                if 0 <= ix < W and 0 <= iy < H:
                    return bool(grid[iy, ix] > 50)
                return True

            # learned arm
            t1 = time.time()
            bev = render_bev(occ_fn, center, seg, rects[0], g)
            for extra in rects[1:]:
                _rasterize_polygon(bev[2], extra, center)
            np.clip(bev, 0, 1, out=bev)
            local = local_path_points(seg, center)
            rf = _rule_feats(rects, A, B, center)
            ctrls, confs, topo, scope = net.forward(
                add_coords(bev.astype(np.float64)),
                local.astype(np.float64), rf.astype(np.float64))
            idx, dense, reason = select_topo(
                ctrls, confs, center, A, B, seg, rects, occ_fn,
                bounds, tau=TAU_SCOPE, scope=scope)
            row["learned_ok"] = bool(idx is not None)
            row["learned_reason"] = reason
            row["learned_len"] = (round(path_len(dense), 3)
                                  if idx is not None else "")
            row["learned_ms"] = round((time.time() - t1) * 1000.0, 2)
            row["learned_rep"] = (None if scope is None
                                  else round(float(scope["rep_prob"]), 4))
            # classical arm
            t2 = time.time()
            from rulepatch.rp import BEV_SIZE_M
            half = BEV_SIZE_M / 2.0
            window = (center[0] - half, center[1] - half,
                      center[0] + half, center[1] + half)
            from topopatch.topo.classical_local import build_blocked
            bwin, (oxw, oyw) = build_blocked(
                grid, ox, oy, res, rects, radius=0.1, window=window)
            pts = theta_star(bwin, oxw, oyw, res, A, B)
            if not pts:
                row["classical_ok"] = False
                row["classical_reason"] = "classical:no-path"
            else:
                from topopatch.topo.classical_local import densify_world
                from topopatch.topo.verifier_topo import (
                    verify_dense_multi)
                dense_c = densify_world(pts)
                ok_c, rs_c = verify_dense_multi(
                    np.asarray(dense_c, dtype=np.float64), A, B, seg,
                    rects, occ_fn, bounds, center)
                row["classical_ok"] = bool(ok_c)
                row["classical_reason"] = rs_c
                row["classical_len"] = (round(path_len(dense_c), 3)
                                        if ok_c else "")
            row["classical_ms"] = round((time.time() - t2) * 1000.0, 2)
            row["case_ms"] = round((time.time() - t0) * 1000.0, 2)
        except Exception as e:  # noqa: BLE001 - record, don't stop
            row["learned_ok"] = False
            row["learned_reason"] = f"error: {e}"[:120]
        rows.append(row)
        if len(rows) % 25 == 0:
            print(f"[mazeval] {len(rows)}/{len(m['cases'])}", flush=True)

    with open(a.out, "w", newline="") as f:
        keys = sorted({k for r in rows for k in r.keys()})
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})
    print(f"[mazeval] wrote {a.out} ({len(rows)} rows)", flush=True)


if __name__ == "__main__":
    main()
