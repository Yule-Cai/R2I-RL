"""Paired offline eval: frozen deployed RulePatch vs classical Theta*.

Same 261 frozen manifest cases, identical inputs per case (A/B anchors,
keepout polys, occupancy, center, orig segment, goal, bounds), same
deterministic gate (verify_dense_multi, strengthened segment version).

Learned arm replicates the deployed ROS node EXACTLY: BEV render +
NumpyTopoNet(best_fused.npz) forward + select_topo with the frozen VAL
tau (1.8216713190078737) and scope policy (rep>=0.5, span<=window).

No training, no tuning, no threshold changes. Pure evaluation.
"""
import csv
import json
import math
import time

import numpy as np

TAU_SCOPE = 1.8216713190078737
RES = 0.05
OX = OY = -10.0


def path_len(pts):
    p = np.asarray(pts, dtype=np.float64)
    return float(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])).sum())


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    from rulepatch.rp import BEV_SIZE_M
    from rulepatch.rp.bev import (add_coords, local_path_points,
                                  render_bev, _rasterize_polygon)
    from rulepatch.rp.extract import load_occ_grid, make_occ_fn
    from topopatch.topo.classical_local import repair_local_thetastar
    from topopatch.topo.extract_topo import (_rule_feats,
                                             load_topo_split_items)
    from topopatch.topo.infer_numpy_topo import NumpyTopoNet
    from topopatch.topo.verifier_topo import select_topo

    m = json.load(open(a.manifest))
    cases = {c["uid"]: c for c in m["cases"]}
    net = NumpyTopoNet(a.ckpt)

    # map tag -> (test npz dir, pgm)
    import os
    manifest_dir = os.path.dirname(a.manifest)
    data = {}
    for tag in ("topo_ext_maze-B-s202", "topo_ext_narrow-B-s302"):
        d = os.environ.get("RP_DATA_" + tag.replace("-", "_"), "")
        assert d, f"set RP_DATA_{tag} to test-split dir"
        items = {np.asarray(s["uid"]).reshape(-1)[0]
                 if np.asarray(s["uid"]).size else s["uid"]: s
                 for s in load_topo_split_items(d, "test")}
        pgm = os.environ.get("RP_PGM_" + tag.replace("-", "_"), "")
        occ = load_occ_grid(pgm, res_out=RES, res_in=RES, ox=OX, oy=OY)
        full = np.asarray(occ["full"])
        w, h = occ["w"], occ["h"]
        bounds = (OX, OY, OX + w * RES, OY + h * RES)
        data[tag] = (items, full, make_occ_fn(occ), bounds)

    rows = []
    for uid, c in cases.items():
        tag = c["map_tag"]
        items, full, occ_fn, bounds = data[tag]
        s = items[uid]
        A = tuple(np.asarray(s["A"], dtype=np.float64).ravel().tolist())
        B = tuple(np.asarray(s["B"], dtype=np.float64).ravel().tolist())
        center = tuple(np.asarray(s["center"], dtype=np.float64)
                       .ravel().tolist())
        seg = np.asarray(s["orig_seg"], dtype=np.float64)
        npol = int(np.asarray(s["polys_n"]).reshape(-1)[0])
        keepouts = [np.asarray(s["polys"][k], dtype=np.float64)
                    for k in range(npol)]
        goal = tuple(np.asarray(s["goal"], dtype=np.float64)
                     .ravel().tolist())
        teacher_len = path_len(np.asarray(s["repl_full"],
                                          dtype=np.float64))
        row = {"uid": uid, "map": tag, "teacher_len": round(teacher_len, 3)}

        # ---- learned arm (deployed node replication) ----
        t0 = time.time()
        try:
            bev = render_bev(occ_fn, center, seg, keepouts[0], goal)
            for extra in keepouts[1:]:
                _rasterize_polygon(bev[2], extra, center)
            np.clip(bev, 0, 1, out=bev)
            local = local_path_points(seg, center)
            rf = _rule_feats(keepouts, A, B, center)
            ctrls, confs, topo, scope = net.forward(
                add_coords(bev.astype(np.float64)),
                local.astype(np.float64), rf.astype(np.float64))
            fwd_ms = (time.time() - t0) * 1000.0
            idx, dense, reason = select_topo(
                ctrls, confs, center, A, B, seg, keepouts, occ_fn,
                bounds, tau=TAU_SCOPE, scope=scope)
            row["learned_ok"] = bool(idx is not None)
            row["learned_reason"] = reason
            row["learned_len"] = (round(path_len(dense), 3)
                                  if idx is not None else "")
            row["learned_ms"] = round((time.time() - t0) * 1000.0, 2)
            row["learned_infer_ms"] = round(fwd_ms, 2)
            row["learned_rep"] = (None if scope is None
                                  else round(float(scope["rep_prob"]), 4))
        except Exception as e:  # noqa: BLE001 - record, don't stop
            row["learned_ok"] = False
            row["learned_reason"] = f"error: {e}"[:120]
            row["learned_ms"] = round((time.time() - t0) * 1000.0, 2)

        # ---- classical arm (local Theta*, same budget) ----
        t1 = time.time()
        try:
            cr = repair_local_thetastar(
                full, OX, OY, RES, A, B, keepouts, center, seg, bounds)
            row["classical_ok"] = bool(cr["ok"])
            row["classical_reason"] = cr["reason"]
            row["classical_len"] = (round(path_len(cr["dense"]), 3)
                                    if cr.get("dense") is not None else "")
            row["classical_ms"] = round((time.time() - t1) * 1000.0, 2)
            row["classical_plan_ms"] = round(cr.get("latency_ms", -1), 2)
        except Exception as e:  # noqa: BLE001 - record, don't stop
            row["classical_ok"] = False
            row["classical_reason"] = f"error: {e}"[:120]
            row["classical_ms"] = round((time.time() - t1) * 1000.0, 2)

        rows.append(row)
        if len(rows) % 50 == 0:
            print(f"[paired] {len(rows)}/{len(cases)}", flush=True)

    with open(a.out, "w", newline="") as f:
        keys = sorted({k for r in rows for k in r.keys()})
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})
    print(f"[paired] wrote {a.out} ({len(rows)} rows)", flush=True)


if __name__ == "__main__":
    main()
