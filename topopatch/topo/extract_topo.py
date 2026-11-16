"""TopoRulePatch extraction: raw teacher pairs -> anchored BEV samples.

Raw schema (gen_topo_dataset shards): per-index orig_{i}/repl_{i} full
paths, polys_{i} (object array of KEEP_OUT polygons), plus uids, start,
goal, n_rules, geoms, orig_len, repl_len, map_id.

Mirrors rulepatch.rp.extract.extract_sample, generalized to 1-3 rules:
  - anchors via find_anchors(orig, repl) (same thresholds as M2)
  - orig_seg must cross the UNION of rules (>=3 pts); repl_seg must avoid
    the union (0 pts) — else the teacher pair is unusable.
  - BEV rule channel = union of all rule polygons.
  - split is assigned by the RUNNER (--split train|val|test), never by map
    region: test maps are unseen by construction, so geographic TEST_REGION
    splits would be both unnecessary and misleading here.
"""
import glob
import json
import os

import numpy as np

from rulepatch.rp import BEV_SIZE_M
from rulepatch.rp.bev import (render_bev, window_for_patch,
                              local_path_points, target_control_points,
                              _rasterize_polygon)
from rulepatch.rp.extract import load_occ_grid, make_occ_fn
from rulepatch.rp.geom import (find_anchors, path_length, point_in_polygon,
                               resample_polyline)

from .labels import label_sample

N_RULES_MAX = 3
POLY_PTS = 12


def load_topo_raw(data_dir):
    """Yields raw teacher records from topo shards."""
    recs = []
    for fp in sorted(glob.glob(os.path.join(data_dir, "shard_*.npz"))):
        d = np.load(fp, allow_pickle=True)
        n = int(np.asarray(d["n"]).reshape(-1)[0])
        for i in range(n):
            polys = [np.asarray(p, dtype=np.float64)
                     for p in d[f"polys_{i}"].tolist()]
            recs.append({
                "uid": str(d["uids"][i]),
                "map_id": str(d["map_id"][i]),
                "orig": np.asarray(d[f"orig_{i}"], dtype=np.float64),
                "repl": np.asarray(d[f"repl_{i}"], dtype=np.float64),
                "polys": polys,
                "n_rules": int(np.asarray(d["n_rules"]).reshape(-1)[i]),
                "geoms": str(d["geoms"][i]).split(","),
                "goal": np.asarray(d["goal"][i], dtype=np.float64),
            })
    return recs


def _inside_union(xy, polys):
    return any(point_in_polygon(xy, poly) for poly in polys)


def extract_topo_sample(rec, occ_fn, map_bounds, fast=None, stats=None):
    """Returns sample dict or (None, reason). fast=(occ_full,res_in,ox,oy)."""
    def drop(reason):
        if stats is not None:
            stats[reason] = stats.get(reason, 0) + 1
        return None, reason
    orig = np.asarray(rec["orig"], dtype=np.float64)
    repl = np.asarray(rec["repl"], dtype=np.float64)
    polys = [np.asarray(p, dtype=np.float64) for p in rec["polys"]]
    goal = tuple(map(float, rec["goal"]))
    anc, reason = find_anchors(orig, repl)
    if anc is None:
        return drop(f"anchor:{reason}")
    iA, iB, jA, jB = anc["iA"], anc["iB"], anc["jA"], anc["jB"]
    A, B = anc["A"], anc["B"]
    orig_seg = orig[iA:iB + 1]
    repl_seg = repl[jA:jB + 1]
    if sum(1 for xy in orig_seg if _inside_union(xy, polys)) < 3:
        return drop("orig-unaffected")
    if sum(1 for xy in repl_seg if _inside_union(xy, polys)) > 0:
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
        from rulepatch.rp.bev import render_bev_fast
        bev = render_bev_fast(fast[0], fast[1], fast[2], fast[3], center,
                              orig_seg, polys[0], goal)
    else:
        bev = render_bev(occ_fn, center, orig_seg, polys[0], goal)
    for extra in polys[1:]:  # union remaining rules into channel 2
        _rasterize_polygon(bev[2], extra, center)
    np.clip(bev, 0, 1, out=bev)
    local = local_path_points(orig_seg, center)
    tgt = target_control_points(repl_seg, center)
    topo = label_sample(A, B, repl_seg, polys)
    polys_pad = np.zeros((N_RULES_MAX, POLY_PTS, 2), dtype=np.float32)
    for k, p in enumerate(polys[:N_RULES_MAX]):
        r = resample_polyline(np.asarray(p, dtype=np.float64), POLY_PTS)
        polys_pad[k] = r.astype(np.float32)
    # union rect (verifier/augmentation use; replaces M2 single rect)
    xs = np.concatenate([p[:, 0] for p in polys])
    ys = np.concatenate([p[:, 1] for p in polys])
    union_rect = np.array([(xs.min(), ys.min()), (xs.min(), ys.max()),
                           (xs.max(), ys.max()), (xs.max(), ys.min())],
                          dtype=np.float32)
    return {
        "uid": rec.get("uid", "?"), "map_id": rec.get("map_id", "?"),
        "bev": (bev * 255).astype(np.uint8),
        "local": local.astype(np.float32), "tgt": tgt.astype(np.float32),
        "center": np.array(center, np.float32),
        "A": np.array(A, np.float32), "B": np.array(B, np.float32),
        "rect": union_rect,
        "polys": polys_pad,
        "polys_n": np.array([len(polys)], np.int64),
        "geoms": ",".join(rec.get("geoms", [])),
        "rule_feats": _rule_feats(polys, A, B, center).astype(np.float32),
        "topo_cls": np.array([topo["cls"]], np.int64),
        "topo_sig": topo["signature"], "topo_corr": int(topo["corridor"]),
        "goal": np.array(goal, np.float32),
        "orig_seg": np.array(orig_seg, np.float32),
        "repl_seg": np.array(repl_seg, np.float32),
        "orig_full": np.array(orig, np.float32),
        "repl_full": np.array(repl, np.float32),
        "idx": np.array([iA, iB, jA, jB], np.int64),
    }, "ok"


def _rule_feats(polys, A, B, center):
    """Planner-available geometric features per rule, padded to (3,6).

    Layout per rule: [area_n, cx_n, cy_n, w_n, h_n, cross_orig], with
    centroid/size in WINDOW-NORMALIZED coords (matches the augmentation
    frame in train_topo._augment: flips/rot90 act on cols 1:2, w/h swap
    on odd rotations, area/cross invariant). area_n = area/25 clipped;
    w_n/h_n = bbox sides / BEV_SIZE_M; cross_orig = fraction of
    {A, B, mid} inside the rule (rigid-invariant).
    """
    from rulepatch.rp import BEV_SIZE_M
    from rulepatch.rp.bev import world_to_norm
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    mid = (A + B) / 2.0
    feats = np.zeros((N_RULES_MAX, 6), dtype=np.float32)
    for k, poly in enumerate(polys[:N_RULES_MAX]):
        p = np.asarray(poly, dtype=np.float64)
        if len(p) < 3:
            continue
        x0, y0 = p[:, 0], p[:, 1]
        area = 0.5 * abs(float(np.dot(x0, np.roll(y0, -1))
                               - np.dot(np.roll(x0, -1), y0)))
        c = world_to_norm(p.mean(axis=0, keepdims=True), center)[0]
        w = (x0.max() - x0.min()) / BEV_SIZE_M
        h = (y0.max() - y0.min()) / BEV_SIZE_M
        cross = sum(1 for q in [A, B, mid]
                    if point_in_polygon(q, p)) / 3.0
        feats[k] = [min(area / 25.0, 1.0), c[0], c[1], w, h, cross]
    return feats


def load_topo_split_items(data_dir, split):
    """Load one extracted split ({split}.npz, stacked schema) as dicts.

    NOTE: each npz column is materialized ONCE (cols dict); per-item
    entries are views. Indexing NpzFile per (item, key) re-decodes the
    column from zip every time and pins a fresh base per view (100x
    memory blowup -> OOM-killed merge/train; incident 2026-09-25).
    """
    fp = os.path.join(data_dir, f"{split}.npz")
    z = np.load(fp, allow_pickle=True)
    cols = {k: z[k] for k in z.files if k != "n"}
    n = int(np.asarray(z["n"]).reshape(-1)[0])
    items = []
    for i in range(n):
        it = {}
        for k, v in cols.items():
            if v.shape and v.shape[0] == n:
                it[k] = v[i]
            else:
                it[k] = v
        items.append(it)
    return items


def run_topo_extraction(data_dir, pgm_path, map_bounds, out_dir, split,
                        res_in=0.05, ox=-10.0, oy=-10.0, val_mod=10):
    """Extract one raw dir into a single-split npz (+META with map hash).

    split=train: uid-hash val subdivision (int(uid)%val_mod==0 -> val).
    split=test|val: all samples take that split (map-level separation).
    """
    import hashlib
    occ = load_occ_grid(pgm_path, res_out=res_in, res_in=res_in,
                        ox=ox, oy=oy)
    occ_fn = make_occ_fn(occ)
    fast = (occ["full"], occ["res_in"], occ["ox"], occ["oy"])
    map_sha = hashlib.sha256(open(pgm_path, "rb").read()).hexdigest()[:16]
    os.makedirs(out_dir, exist_ok=True)
    recs = load_topo_raw(data_dir)
    print(f"[extract] raw records: {len(recs)}", flush=True)
    stats, buckets = {}, {}
    for rec in recs:
        s, reason = extract_topo_sample(rec, occ_fn, map_bounds, fast=fast,
                                        stats=stats)
        if s is None:
            continue
        if split == "train":
            digits = "".join(c for c in s["uid"] if c.isdigit()) or "1"
            sp = "val" if (int(digits) % val_mod == 0) else "train"
        else:
            sp = split
        s["split"] = sp
        buckets.setdefault(sp, []).append(s)
    out = {"map_sha256": map_sha, "pgm": os.path.basename(pgm_path),
           "split_arg": split, "stats": stats, "counts": {}}
    for sp, items in buckets.items():
        kw = {"n": np.array([len(items)])}
        str_keys = ("uid", "map_id", "geoms", "topo_sig", "split")
        for kk in str_keys:
            kw[kk] = np.array([it[kk] for it in items])
        arr_keys = ("bev", "local", "tgt", "center", "A", "B", "rect",
                    "polys", "polys_n", "rule_feats", "topo_cls",
                    "goal", "orig_seg", "repl_seg", "orig_full",
                    "repl_full", "idx")
        for kk in arr_keys:
            try:
                kw[kk] = np.stack([np.asarray(it[kk]) for it in items])
            except ValueError:
                kw[kk] = np.array([np.asarray(it[kk], dtype=object)
                                   for it in items], dtype=object)
        kw["topo_corr"] = np.array([it["topo_corr"] for it in items],
                                   np.int64)
        np.savez_compressed(os.path.join(out_dir, f"{sp}.npz"), **kw)
        out["counts"][sp] = len(items)
    json.dump(out, open(os.path.join(out_dir, "META.json"), "w"), indent=1)
    print(f"[extract] {out_dir}: {out['counts']} drops={stats}", flush=True)
    return out
