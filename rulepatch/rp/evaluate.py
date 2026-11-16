"""Offline held-out evaluation: baselines + RulePatchNet vs success targets.

Baselines: FULL_NAV2_REPLAN (teacher), COPY_ORIGINAL,
SINGLE_PATCH_NET (M=1), RULEPATCHNET (M=3), RULEPATCHNET_NO_RULE_CHANNEL.
Metrics: rule compliance (accepted), valid patch rate, accepted
learned-patch rate, fallback rate, closed-loop (pure-pursuit rollout) goal
success, path length ratio, inference mean/p95 latency, params/RAM.

Usage:
  PYTHONPATH=rulepatch python3 -m rp.evaluate --data <extracted> --pgm <map.pgm>
      --m3 <run_dir/best.pt> --m1 <run_dir/best.pt> --m3norule <run/best.pt>
      --out <eval_dir>
"""
import argparse
import json
import os
import time

import numpy as np
import torch

from . import BEV_SIZE_M
from .extract import load_occ_grid, make_occ_fn
from .geom import path_length, point_in_polygon, resample_polyline
from .model import RulePatchNet, count_params
from .verifier import verify_dense, select_hypothesis, decode_ctrl

MAP_BOUNDS = (-15.1, -25.0, 15.08, 25.22)  # warehouse (origin + W*res)

def rollout_success(full_path, goal, occ_fn, speed=0.4, dt=0.1,
                    lookahead=0.5, max_steps=600, goal_tol=0.5):
    """Deterministic unicycle pure-pursuit proxy for closed-loop following."""
    p = np.asarray(full_path, dtype=np.float64)
    if len(p) < 2:
        return False
    x, y, th = float(p[0, 0]), float(p[0, 1]), 0.0
    dx, dy = p[1] - p[0]
    if abs(dx) + abs(dy) > 1e-9:
        th = float(np.arctan2(dy, dx))
    prog = 0
    for _ in range(max_steps):
        if occ_fn(x, y):
            return False
        if float(np.hypot(goal[0] - x, goal[1] - y)) < goal_tol:
            return True
        d = np.hypot(p[prog:, 0] - x, p[prog:, 1] - y)
        prog += int(np.argmin(d))  # nearest at/after progress (no regression)
        ahead = np.where(np.hypot(p[prog:, 0] - x, p[prog:, 1] - y)
                         > lookahead)[0]
        if len(ahead) == 0:
            tx, ty = goal
        else:
            tx, ty = float(p[prog + ahead[0], 0]), float(p[prog + ahead[0], 1])
        ang = float(np.arctan2(ty - y, tx - x) - th)
        ang = (ang + np.pi) % (2 * np.pi) - np.pi
        th += np.clip(2.0 * ang, -1.0, 1.0) * dt
        x += speed * np.cos(th) * dt
        y += speed * np.sin(th) * dt
    return bool(float(np.hypot(goal[0] - x, goal[1] - y)) < goal_tol)


def repaired_full(orig_full, iA, iB, dense_patch):
    o = np.asarray(orig_full, dtype=np.float64)
    d = np.asarray(dense_patch, dtype=np.float64)
    return np.concatenate([o[:iA + 1], d, o[iB:]], axis=0)


def load_samples(data_dir, split="test"):
    from .dataset import load_split
    items = load_split(data_dir, split)
    return items, [s["uid"] for s in items]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--m3", default=None)
    ap.add_argument("--m1", default=None)
    ap.add_argument("--m3norule", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rollout", action="store_true",
                    help="run pure-pursuit closed-loop proxy (slower)")
    ap.add_argument("--walls-only", action="store_true",
                    help="occupancy = lethal walls only (Nav2 allow_unknown "
                    "semantics); default blocks unknown too")
    ap.add_argument("--res-in", type=float, default=0.03)
    ap.add_argument("--ox", type=float, default=-15.1)
    ap.add_argument("--oy", type=float, default=-25.0)
    ap.add_argument("--bounds", type=float, nargs=4,
                    default=list(MAP_BOUNDS))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    occ = load_occ_grid(a.pgm, res_in=a.res_in, ox=a.ox, oy=a.oy,
                        walls_only=a.walls_only)
    occ_fn = make_occ_fn(occ)
    MAPB = tuple(a.bounds)
    samples, uids = load_samples(a.data, "test")
    print(f"[eval] test samples: {len(samples)}", flush=True)

    models = {}
    for name, path in (("m3", a.m3), ("m1", a.m1), ("m3norule", a.m3norule)):
        if path:
            ck = torch.load(path, map_location="cpu", weights_only=False)
            mhyp = ck["cfg"]["m_hyp"]
            m = RulePatchNet(m_hyp=mhyp,
                             in_ch=int(ck["cfg"].get("in_ch", 6)))
            m.load_state_dict(ck["model"])
            m.eval()
            models[name] = (m, count_params(m))
    results = {}
    lat = {}
    for name, (m, np_) in models.items():
        lat[name] = []
    with torch.no_grad():
        for name, (m, _) in models.items():
            from .model import add_coords
            for s in samples[:50]:
                bev = torch.from_numpy(
                    add_coords(s["bev"].astype(np.float32) / 255.0)).unsqueeze(0)
                loc = torch.from_numpy(s["local"]).unsqueeze(0)
                t0 = time.perf_counter()
                m(bev, loc)
                lat[name].append((time.perf_counter() - t0) * 1000.0)

    def model_pred(name, s):
        from .model import add_coords
        m, _ = models[name]
        bev = add_coords(s["bev"].astype(np.float32) / 255.0)
        bev = torch.from_numpy(bev).unsqueeze(0)
        loc = torch.from_numpy(s["local"]).unsqueeze(0)
        ctrls, confs = m(bev, loc)
        return ctrls[0].detach().numpy(), confs[0].detach().numpy()

    def assess(patch_dense, s, accepted_src):
        A, B = tuple(s["A"]), tuple(s["B"])
        rect = [tuple(p) for p in s["rect"]]
        ok, reason = verify_dense(
            patch_dense, A, B, s["orig_seg"], rect, occ_fn, MAPB,
            tuple(s["center"]))
        comp = sum(1 for xy in np.asarray(patch_dense) if point_in_polygon(xy, rect)) == 0
        out = {"accepted": bool(ok), "reason": reason, "compliant": bool(comp)}
        if ok and a.rollout:
            iA, iB = int(s["idx"][0]), int(s["idx"][1])
            full = repaired_full(s["orig_full"], iA, iB, patch_dense)
            out["closed_loop"] = rollout_success(full, tuple(s["goal"]), occ_fn)
            out["len_ratio"] = path_length(full) / max(1e-9, path_length(s["repl_full"]))
        return out

    # baselines + models
    report = {"n_test": len(samples), "methods": {}}
    for method in (["copy", "teacher"] +
                   ([f"model:{k}" for k in models.keys()])):
        acc = comp = cl = fb = 0
        ratios = []
        reasons: dict = {}
        for s in samples:
            A, B = tuple(s["A"]), tuple(s["B"])
            center = tuple(s["center"])
            if method == "copy":
                patch = resample_polyline(s["orig_seg"], 61)
                r = assess(patch, s, method)
            elif method == "teacher":
                patch = resample_polyline(s["repl_seg"], 61)
                r = assess(patch, s, method)
            else:
                name = method.split(":")[1]
                ctrls, confs = model_pred(name, s)
                idx, dense, rs = select_hypothesis(
                    ctrls, confs, center, A, B, s["orig_seg"],
                    [tuple(p) for p in s["rect"]], occ_fn, MAPB)
                if idx is None:
                    r = {"accepted": False, "reason": rs, "compliant": True}
                else:
                    r = assess(dense, s, method)
            acc += r["accepted"]
            comp += r["compliant"] and r["accepted"]
            fb += (not r["accepted"])
            if r.get("closed_loop"):
                cl += 1
            if r.get("len_ratio"):
                ratios.append(r["len_ratio"])
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        n = len(samples)
        report["methods"][method] = {
            "valid_patch_rate": acc / n,
            "accepted_learned_patch_rate": acc / n,
            "rule_compliance_accepted": (comp / acc) if acc else 1.0,
            "fallback_rate": fb / n,
            "closed_loop_success": (cl / acc) if (acc and a.rollout) else None,
            "len_ratio_mean": float(np.mean(ratios)) if ratios else None,
            "reject_reasons": reasons,
        }
    for name, v in lat.items():
        report["methods"][f"model:{name}"]["lat_ms_mean"] = float(np.mean(v))
        report["methods"][f"model:{name}"]["lat_ms_p95"] = float(np.percentile(v, 95))
        report["methods"][f"model:{name}"]["params"] = models[name][1]
    # multimodality gain
    if "model:m3" in report["methods"] and "model:m1" in report["methods"]:
        g = (report["methods"]["model:m3"]["accepted_learned_patch_rate"]
             - report["methods"]["model:m1"]["accepted_learned_patch_rate"])
        report["multimodality_gain_pp"] = g * 100.0
    json.dump(report, open(os.path.join(a.out, "offline_results.json"), "w"), indent=1)
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "reject_reasons"}
                      for k, v in report["methods"].items()}, indent=1))


if __name__ == "__main__":
    main()
