"""TopoRulePatch offline evaluation: corrected metric semantics + baselines.

Metric semantics (§2 of the milestone):
  native_*        : raw top-1 hypothesis quality, NO verifier, NO threshold.
  verifier_*      : after deterministic verification + operating threshold.
  post-verifier compliance is a SYSTEM property (100% by construction when
  the verifier is sound), reported as such — never as model accuracy.
  all-case success: denominator = ALL test samples (accepted + rejected +
  fallback). Never report only success conditional on acceptance.

Methods: copy, teacher(FULL_NAV2_REPLAN), deform (classical),
  m1 (frozen M2 M=1), m3unstruct (frozen M2 M=3), notopo (ablation ckpt),
  full (TopoRulePatch ckpt). Ckpts optional (skipped if absent).

Usage:
  PYTHONPATH=<repo> python3 -m topopatch.topo.evaluate_topo \
      --data <extracted_map_dir> --map-id <id> --pgm <map.pgm> \
      --res-in 0.05 --ox -10 --oy -10 --bounds ... \
      --m3unstruct <m2m3.pt> --m1 <m2m1.pt> --notopo <pt> --full <pt> \
      --tau 0.0 --rollout --records records.jsonl --out <eval_dir>
"""
import argparse
import json
import os
import time

import numpy as np
import torch

from rulepatch.rp import evaluate as EVAL
from rulepatch.rp.geom import (path_length, point_in_polygon,
                               resample_polyline)
from topopatch.topo.verifier_topo import (select_topo, verify_dense_multi,
                                          verify_patch_multi)


def _polys_of(s):
    n = int(np.asarray(s["polys_n"]).reshape(-1)[0])
    return [tuple(map(tuple, np.asarray(s["polys"][k]).tolist()))
            for k in range(n)]


def _native_checks(dense, s, occ_fn, bounds):
    """Native (pre-verifier) quality of one dense patch."""
    A, B = tuple(s["A"]), tuple(s["B"])
    polys = _polys_of(s)
    dense = np.asarray(dense, dtype=np.float64)
    compliant = all(sum(1 for xy in dense if point_in_polygon(xy, poly))
                    == 0 for poly in polys)
    coll = False
    for x, y in dense:
        hit = False
        for dx in (-0.1, 0.0, 0.1):
            for dy in (-0.1, 0.0, 0.1):
                if occ_fn(x + dx, y + dy):
                    hit = True
                    break
            if hit:
                break
        if hit:
            coll = True
            break
    ok, reason = verify_dense_multi(
        dense, A, B, s["orig_seg"], polys, occ_fn, bounds,
        tuple(s["center"]))
    return {"compliant": bool(compliant), "collisionfree": bool(not coll),
            "valid": bool(ok), "reason": reason}


def _rollout_full(orig_full, idx, dense_or_none, repl_full, goal, occ_fn):
    if dense_or_none is not None:
        full = EVAL.repaired_full(orig_full, int(idx[0]), int(idx[1]),
                                  dense_or_none)
    else:
        full = np.asarray(repl_full, dtype=np.float64)
    ok = EVAL.rollout_success(full, tuple(goal), occ_fn)
    return bool(ok), float(path_length(full) / max(
        1e-9, path_length(repl_full)))


def _max_turn(dense):
    from rulepatch.rp.geom import max_turn_deg
    try:
        return float(max_turn_deg(np.asarray(dense, dtype=np.float64)))
    except Exception:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--map-id", required=True)
    ap.add_argument("--split", default="test")
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res-in", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--bounds", type=float, nargs=4, required=True)
    ap.add_argument("--m1", default=None)
    ap.add_argument("--m3unstruct", default=None)
    ap.add_argument("--notopo", default=None)
    ap.add_argument("--full", default=None)
    ap.add_argument("--scope", default=None,
                    help="adaptive-scope ckpt (model:scope method)")
    ap.add_argument("--tau", type=float, default=float("-inf"))
    ap.add_argument("--freeze", default=None,
                    help="freeze.json: asserts map/data hashes, uses its tau")
    ap.add_argument("--rollout", action="store_true")
    ap.add_argument("--records", default=None)
    ap.add_argument("--manifest", default=None,
                    help="gen manifest.jsonl for Nav2 replan-time stats")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.freeze:
        from topopatch.topo.freeze import check_freeze
        a.tau = check_freeze(a.freeze, a.pgm, a.data, a.split)
    from rulepatch.rp.extract import load_occ_grid, make_occ_fn
    occ = load_occ_grid(a.pgm, res_in=a.res_in, ox=a.ox, oy=a.oy)
    occ_fn = make_occ_fn(occ)
    MAPB = tuple(a.bounds)
    from topopatch.topo.extract_topo import load_topo_split_items
    samples = load_topo_split_items(a.data, a.split)
    print(f"[eval] map {a.map_id}: test samples: {len(samples)}", flush=True)

    # SDF for the classical baseline (planner-available occ info only).
    import scipy.ndimage as _ndi
    _sdf = _ndi.distance_transform_edt(
        1 - occ["full"].astype(np.uint8)).astype(np.float32) * a.res_in
    from topopatch.topo.deform import make_sdf_fn
    sdf_fn = make_sdf_fn(_sdf, a.ox, a.oy, a.res_in)

    models, lat = {}, {}
    from rulepatch.rp.model import RulePatchNet, count_params
    for name, path, arch in (("m1", a.m1, "m2"), ("m3unstruct", a.m3unstruct, "m2"),
                             ("notopo", a.notopo, "topo"),
                             ("full", a.full, "topo"),
                             ("scope", a.scope, "topo")):
        if not path:
            continue
        ck = torch.load(path, map_location="cpu", weights_only=False)
        if arch == "m2":
            m = RulePatchNet(m_hyp=int(ck["cfg"]["m_hyp"]),
                             in_ch=int(ck["cfg"].get("in_ch", 6)))
            m.load_state_dict(ck["model"])
            m.eval()
            models[name] = ("m2", m, count_params(m))
        else:
            from topopatch.topo.model_topo import (TopoRulePatchNet,
                                                  count_params as cpt)
            m = TopoRulePatchNet()
            # Old (M3) ckpts lack the M4 scope heads: tolerate exactly
            # those missing keys (scope outputs unused for them).
            _missing, _unexpected = m.load_state_dict(ck["model"],
                                                      strict=False)
            _allowed = {"repair_head.weight", "repair_head.bias",
                        "span_head.weight", "span_head.bias",
                        "complexity_head.weight", "complexity_head.bias"}
            assert set(_missing) <= _allowed, _missing
            assert not _unexpected, _unexpected
            m.eval()
            models[name] = ("topo", m, cpt(m))
    for name in models:
        lat[name] = []

    from rulepatch.rp.bev import add_coords

    def fwd_m2(m, s):
        bev = torch.from_numpy(
            add_coords(s["bev"].astype(np.float32) / 255.0)).unsqueeze(0)
        loc = torch.from_numpy(s["local"]).unsqueeze(0)
        with torch.no_grad():
            ctrls, confs = m(bev, loc)
        return ctrls[0].numpy(), confs[0].numpy(), None

    def fwd_topo(m, s):
        bev = torch.from_numpy(
            add_coords(s["bev"].astype(np.float32) / 255.0)).unsqueeze(0)
        loc = torch.from_numpy(s["local"]).unsqueeze(0)
        rf = torch.from_numpy(s["rule_feats"]).unsqueeze(0)
        with torch.no_grad():
            ctrls, confs, topo, scope = m(bev, loc, rf)
        import torch as _t
        sc = {"rep_prob": float(_t.sigmoid(scope["repair_logit"])[0]),
              "span_pred": float(scope["span_pred"][0]),
              "cx_pred": int(scope["complexity_logits"][0].argmax())}
        return (ctrls[0].numpy(), confs[0].numpy(),
                torch.softmax(topo, dim=1)[0].numpy(), sc)

    def e2e_predict(kind, m, s, use_scope=False):
        """Returns dict with dense options + timings (ms)."""
        from rulepatch.rp import verifier as V
        A, B = tuple(s["A"]), tuple(s["B"])
        center = tuple(s["center"])
        polys = _polys_of(s)
        t0 = time.perf_counter()
        t_span = (time.perf_counter() - t0) * 1000.0
        t0 = time.perf_counter()
        if kind == "m2":
            ctrls, confs, topo = fwd_m2(m, s)
            scope = None
        else:
            ctrls, confs, topo, scope = fwd_topo(m, s)
            if not use_scope:
                scope = None
        t_fwd = (time.perf_counter() - t0) * 1000.0
        t0 = time.perf_counter()
        idx, dense, rs = select_topo(
            ctrls, confs, center, A, B, s["orig_seg"], polys, occ_fn,
            MAPB, tau=a.tau, scope=scope)
        t_ver = (time.perf_counter() - t0) * 1000.0
        # native top-1 (argmax conf, no threshold)
        top = int(np.argmax(confs))
        t0 = time.perf_counter()
        dense_top = V.decode_ctrl(ctrls[top], center, A=A, B=B)
        t_dec = (time.perf_counter() - t0) * 1000.0
        # diversity: mean pairwise dense distance across heads
        dd = [V.decode_ctrl(ctrls[k], center, A=A, B=B)
              for k in range(len(ctrls))]
        divs = []
        for i in range(len(dd)):
            for j in range(i + 1, len(dd)):
                divs.append(float(np.mean(np.hypot(
                    dd[i][:, 0] - dd[j][:, 0], dd[i][:, 1] - dd[j][:, 1]))))
        div = float(np.mean(divs)) if divs else 0.0
        out = {"idx": idx, "dense": dense, "reason": rs,
               "dense_top": dense_top, "topo": topo,
               "max_conf": float(np.max(confs)),
               "diversity": div,
               "t_span_ms": t_span, "t_fwd_ms": t_fwd,
               "t_ver_ms": t_ver, "t_dec_ms": t_dec}
        if scope is not None:
            out["scope"] = scope
        else:
            # scope GT is stored regardless (metrics need it for all).
            out["scope"] = {"rep_prob": None, "span_pred": None,
                            "cx_pred": None}
        from topopatch.topo.repair_scope import repairability_gt
        _sg = repairability_gt(s["orig_seg"], s["repl_seg"],
                               tuple(s["A"]), tuple(s["B"]))
        out["scope_gt"] = {"repairable": int(_sg["repairable"]),
                           "span_m": float(_sg["span_m"]),
                           "detour": float(_sg["detour"]),
                           "complexity": int(_sg["complexity"])}
        return out

    report = {"map_id": a.map_id, "split": a.split,
              "n_test": len(samples), "tau": a.tau, "methods": {}}
    records = []
    method_names = (["copy", "teacher", "deform"] +
                    [f"model:{k}" for k in models.keys()])
    for method in method_names:
        acc = comp_acc = cl_acc = cl_fb = fb = 0
        nat_c = nat_f = nat_v = 0
        topo_ok = topo_n = divs = []
        topo_ok, topo_n, divs, turns, ratios = 0, 0, [], [], []
        lats_e2e, lats_fwd, lats_ver = [], [], []
        reasons = {}
        cm = [[0, 0, 0] for _ in range(3)]
        for s in samples:
            A, B = tuple(s["A"]), tuple(s["B"])
            center = tuple(s["center"])
            polys = _polys_of(s)
            rec = {"uid": str(s["uid"]), "map_id": str(s["map_id"]),
                   "n_rules": int(np.asarray(s["polys_n"]).reshape(-1)[0]),
                   "geoms": str(s["geoms"]),
                   "topo_cls": int(s["topo_cls"][0]),
                   "topo_corr": int(s["topo_corr"])}
            if method in ("copy", "teacher"):
                patch = resample_polyline(
                    s["orig_seg"] if method == "copy" else s["repl_seg"], 61)
                nat = _native_checks(patch, s, occ_fn, MAPB)
                r_ok, r_rs = verify_dense_multi(
                    patch, A, B, s["orig_seg"], polys, occ_fn, MAPB, center)
                rec.update({"native": nat, "accepted": bool(r_ok),
                            "reason": r_rs, "max_conf": None,
                            "topo_pred": None, "diversity": 0.0,
                            "lat_e2e": 0.0})
                dense_use = patch if r_ok else None
            elif method == "deform":
                from topopatch.topo.deform import deform_patch
                t0 = time.perf_counter()
                patch = deform_patch(s["orig_seg"], polys, sdf_fn, A, B)
                t_def = (time.perf_counter() - t0) * 1000.0
                nat = _native_checks(patch, s, occ_fn, MAPB)
                r_ok, r_rs = verify_dense_multi(
                    patch, A, B, s["orig_seg"], polys, occ_fn, MAPB, center)
                rec.update({"native": nat, "accepted": bool(r_ok),
                            "reason": r_rs, "max_conf": None,
                            "topo_pred": None, "diversity": 0.0,
                            "lat_e2e": t_def})
                dense_use = patch if r_ok else None
                lats_e2e.append(t_def)
            else:
                name = method.split(":")[1]
                kind, m, _ = models[name]
                pr = e2e_predict(kind, m, s,
                                 use_scope=(name == "scope"))
                nat = _native_checks(pr["dense_top"], s, occ_fn, MAPB)
                if pr["topo"] is not None:
                    pred_mode = int(np.argmax(pr["topo"]))
                    cm[int(s["topo_cls"][0])][pred_mode] += 1
                    topo_n += 1
                    if pred_mode == int(s["topo_cls"][0]):
                        topo_ok += 1
                rec.update({"native": nat,
                            "accepted": pr["idx"] is not None,
                            "reason": pr["reason"],
                            "max_conf": pr["max_conf"],
                            "topo_pred": (int(np.argmax(pr["topo"]))
                                          if pr["topo"] is not None else None),
                            "rep_prob": pr["scope"]["rep_prob"],
                            "span_pred": pr["scope"]["span_pred"],
                            "cx_pred": pr["scope"]["cx_pred"],
                            "gt_rep": pr["scope_gt"]["repairable"],
                            "gt_span": pr["scope_gt"]["span_m"],
                            "gt_cx": pr["scope_gt"]["complexity"],
                            "diversity": pr["diversity"],
                            "lat_e2e": (pr["t_span_ms"] + pr["t_fwd_ms"]
                                        + pr["t_ver_ms"]),
                            "lat_fwd": pr["t_fwd_ms"],
                            "lat_ver": pr["t_ver_ms"]})
                lats_e2e.append(rec["lat_e2e"])
                lats_fwd.append(pr["t_fwd_ms"])
                lats_ver.append(pr["t_ver_ms"])
                divs.append(pr["diversity"])
                dense_use = pr["dense"]
            # closed-loop: accepted->repaired rollout else teacher fallback.
            # M4 Task A invariant: the fallback path is GATED (gate_full_path)
            # before execution. A fallback that crosses a hard rule (or
            # collides) is NEVER executed: safe stop (success False), never
            # a blind teacher rollout. Always ALSO store the fallback-branch
            # outcome so threshold sweeps can recompute per-tau metrics
            # exactly without rerunning rollouts.
            if a.rollout:
                from topopatch.topo.gate import gate_full_path
                # Gate the EXECUTABLE repaired path too (not just the
                # patch): splicing a verified patch can still cut a rule
                # (maze kinks). A gated repaired path is a safe stop.
                rep_go, rep_gate = True, "gate:ok"
                if dense_use is not None:
                    rep_full = EVAL.repaired_full(
                        s["orig_full"], int(s["idx"][0]), int(s["idx"][1]),
                        dense_use)
                    rep_go, rep_gate = gate_full_path(
                        rep_full, polys, occ_fn, MAPB)
                rec["rep_gate"] = rep_gate
                if rep_go:
                    cl_ok, lr = _rollout_full(
                        s["orig_full"], s["idx"], dense_use, s["repl_full"],
                        s["goal"], occ_fn)
                else:
                    cl_ok, lr = False, None
                fb_go, fb_gate = gate_full_path(
                    s["repl_full"], polys, occ_fn, MAPB)
                rec["fb_gate"] = fb_gate
                if fb_go:
                    cl_fb_ok, lr_fb = _rollout_full(
                        s["orig_full"], s["idx"], None, s["repl_full"],
                        s["goal"], occ_fn)
                    lr_fb = float(lr_fb)
                else:
                    # gated: safe stop, never executed
                    cl_fb_ok, lr_fb = False, None
                rec["closed_loop"] = bool(cl_ok)
                rec["len_ratio"] = float(lr) if lr is not None else None
                rec["closed_loop_fb"] = bool(cl_fb_ok)
                rec["len_ratio_fb"] = float(lr_fb) if lr_fb is not None \
                    else None
                if rec["accepted"]:
                    cl_acc += int(cl_ok)
                else:
                    cl_fb += int(cl_ok)
            if dense_use is not None:
                turns.append(_max_turn(dense_use))
                _lr = (rec.get("len_ratio") if a.rollout else
                       _rollout_full(
                           s["orig_full"], s["idx"], dense_use,
                           s["repl_full"], s["goal"], occ_fn)[1])
                ratios.append(_lr)
            nat_c += rec["native"]["compliant"]
            nat_f += rec["native"]["collisionfree"]
            nat_v += rec["native"]["valid"]
            acc += rec["accepted"]
            comp_acc += rec["accepted"] and rec["native"]["compliant"]
            fb += (not rec["accepted"])
            reasons[rec["reason"]] = reasons.get(rec["reason"], 0) + 1
            rec["method"] = method
            records.append(rec)
        n = len(samples)
        entry = {
            "native_rule_compliance": nat_c / n,
            "native_collisionfree": nat_f / n,
            "native_valid_patch_rate": nat_v / n,
            "verifier_acceptance_rate": acc / n,
            "post_verifier_compliance": (comp_acc / acc) if acc else 1.0,
            "fallback_rate": fb / n,
            "all_case_success": ((cl_acc + cl_fb) / n) if a.rollout else None,
            "accepted_success": (cl_acc / acc) if (acc and a.rollout) else None,
            "fallback_success": (cl_fb / fb) if (fb and a.rollout) else None,
            "len_ratio_mean": (float(np.nanmean(
                [v for v in ratios if v is not None]))
                if any(v is not None for v in ratios) else None),
            "max_turn_mean": float(np.nanmean(turns)) if turns else None,
            "reject_reasons": reasons,
        }
        if method.startswith("model:"):
            name = method.split(":")[1]
            entry["params"] = models[name][2]
            entry["topo_acc"] = (topo_ok / topo_n) if topo_n else None
            entry["confusion"] = cm
            entry["diversity_mean"] = float(np.mean(divs)) if divs else None
            for kk, vv in (("e2e_ms", lats_e2e), ("fwd_ms", lats_fwd),
                           ("ver_ms", lats_ver)):
                if vv:
                    arr = np.asarray(vv, dtype=np.float64)
                    entry[f"lat_{kk}"] = {
                        k: float(v) for k, v in
                        zip(("median", "p50", "p90", "p95", "p99"),
                            (np.median(arr), np.percentile(arr, 50),
                             np.percentile(arr, 90),
                             np.percentile(arr, 95),
                             np.percentile(arr, 99)))}
        elif method == "deform":
            if lats_e2e:
                arr = np.asarray(lats_e2e, dtype=np.float64)
                entry["lat_e2e_ms"] = {
                    k: float(v) for k, v in
                    zip(("median", "p50", "p90", "p95", "p99"),
                        (np.median(arr), np.percentile(arr, 50),
                         np.percentile(arr, 90),
                         np.percentile(arr, 95),
                         np.percentile(arr, 99)))}
        report["methods"][method] = entry
    # M4 scope metrics (post-pass over records): repairability quality
    # for methods carrying scope predictions (model:scope).
    for method in report["methods"]:
        rs = [r for r in records
              if r.get("method") == method
              and r.get("rep_prob") is not None]
        if not rs:
            continue
        tp = sum(1 for r in rs
                 if r["rep_prob"] >= 0.5 and r["gt_rep"] == 1)
        fp = sum(1 for r in rs
                 if r["rep_prob"] >= 0.5 and r["gt_rep"] == 0)
        tn = sum(1 for r in rs
                 if r["rep_prob"] < 0.5 and r["gt_rep"] == 0)
        fn = sum(1 for r in rs
                 if r["rep_prob"] < 0.5 and r["gt_rep"] == 1)
        loc = [r for r in rs if r["gt_rep"] == 1]
        report["methods"][method]["scope"] = {
            "n": len(rs),
            "coverage_pred_local": (tp + fp) / len(rs),
            "precision": tp / max(1, tp + fp),
            "recall": tp / max(1, tp + fn),
            "false_local_rate": fp / max(1, fp + tn),
            "tnr": tn / max(1, tn + fp),
            "native_valid_given_local_gt": (
                sum(1 for r in loc if r["native"]["valid"]) / len(loc)
            ) if loc else None,
            "accept_given_local_gt": (
                sum(1 for r in loc if r["accepted"]) / len(loc)
            ) if loc else None,
        }
    if a.records:
        with open(a.records, "w") as f:
            for rec in records:
                f.write(json.dumps(_jsonable(rec)) + "\n")
    # Nav2 replan-time reference from gen manifest (same box, honest).
    if a.manifest:
        try:
            ts = []
            with open(a.manifest) as f:
                for line in f:
                    d = json.loads(line)
                    if "plan_t" in d and len(d["plan_t"]) > 1:
                        ts.append(float(d["plan_t"][1]))
            if ts:
                arr = np.asarray(ts)
                report["nav2_replan_ms"] = {
                    k: float(v) for k, v in
                    zip(("median", "p50", "p90", "p95", "p99", "n"),
                        (np.median(arr), np.percentile(arr, 50),
                         np.percentile(arr, 90), np.percentile(arr, 95),
                         np.percentile(arr, 99), len(arr)))}
        except Exception as e:
            report["nav2_replan_ms"] = {"error": str(e)}
    # multimodality-style deltas vs frozen unstructured M3
    mm = report["methods"]
    if "model:full" in mm and "model:m1" in mm:
        report["gain_vs_m1_pp"] = (
            mm["model:full"]["verifier_acceptance_rate"]
            - mm["model:m1"]["verifier_acceptance_rate"]) * 100.0
    json.dump(report, open(os.path.join(a.out, "offline_results.json"),
                           "w"), indent=1)
    print(json.dumps({k: {kk: vv for kk, vv in v.items()
                          if kk not in ("reject_reasons", "confusion")}
                      for k, v in report["methods"].items()}, indent=1))


def _jsonable(rec):
    out = {}
    for k, v in rec.items():
        if isinstance(v, np.ndarray):
            out[k] = v.tolist()
        elif isinstance(v, (np.integer, np.floating, np.bool_)):
            out[k] = v.item()
        elif isinstance(v, (np.str_, bytes)):
            out[k] = str(v)
        else:
            out[k] = v
    return out


def _max_turn(dense):
    from rulepatch.rp.geom import max_turn_deg
    try:
        return float(max_turn_deg(np.asarray(dense, dtype=np.float64)))
    except Exception:
        return float("nan")


if __name__ == "__main__":
    main()
