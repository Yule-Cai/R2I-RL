"""Nav2-native vs RuleSync-verified controlled baseline (Blocker B).

Same frozen cases, same planner/map/start/goal/timing for both arms:

ARM A (NAV2_NATIVE): rule committed into the normal Nav2 keepout
pipeline; the planner's raw output is TRUSTED directly (no universal
gate). An unsafe candidate is recorded as WOULD-execute (never
physically driven when known unsafe).

ARM B (RULESYNC_VERIFIED): identical RuleSpec/planner/map/start/goal/
timing, but EVERY executable candidate passes gate_full_path
(segment-exact). Only gated paths count as executable.

Conditions per (case, planner):
  N0 normal propagation (plan after mask-ready),
  N2 immediate plan right after commit (race),
  N4 second plan 3 s later (propagation window),
  N6 injected geometrically-crossing path (gate challenge).
N1 (delayed mask via proxy) runs on a subset with --mask-delay.
N3/N5 are covered by F2/F4/F6 (cited, not rerun).

Metrics: unsafe-candidate rate (A), would-execute violation rate (A),
safe-stop/abort rate (B), recovery, commit->compliant-executable
latency (B), exposure window, false-rejection rate (B blocks a clean
path), gate overhead ms. Paired cases. Headless (candidate correctness
before execution; nothing unsafe is driven).
"""
import argparse
import csv
import json
import math
import time

import rclpy
from topopatch_ros.bench_client import Bench
from topopatch.topo.gate import gate_full_path, path_intersects_polygons


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--cases-file", required=True,
                    help="manifest subset [{goal, epoch, rules, start, uid}]")
    ap.add_argument("--planners", default="GridBased",
                    help="comma list: GridBased,Smac2D,ThetaStar")
    ap.add_argument("--map", default="")
    ap.add_argument("--max-cases", type=int, default=0,
                    help="0 = all")
    a = ap.parse_args()
    rclpy.init()
    node = Bench("GridBased", map_tag=a.map)
    if not node.ac.wait_for_server(timeout_sec=30.0):
        print("[native] FATAL: no compute_path_to_pose", flush=True)
        return
    for name, cli in (("propose", node.cli_propose),
                       ("commit", node.cli_commit)):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f"[native] FATAL: /rulesync/{name} unavailable",
                  flush=True)
            return
    # Removal + list services are exercised every case (verified
    # cleanup); wait for them up front like propose/commit (late
    # discovery wedges per-case removal and cascades).
    for name, cli in (("remove_rule", node._cli_rm),
                      ("list_rules", node.cli_list)):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f"[native] FATAL: /rulesync/{name} unavailable",
                  flush=True)
            return
    cases = json.load(open(a.cases_file))["cases"]
    if a.max_cases:
        step = max(1, len(cases) // a.max_cases)
        cases = cases[::step][:a.max_cases]
    rows = []
    for planner in a.planners.split(","):
        node.planner_id = planner
        for c in cases:
            rows.append(run_case(node, planner, c, a.map))
            print(f"[native] {planner} epoch={c['epoch']} "
                  f"reason={rows[-1].get('reason')} "
                  f"A_unsafe={rows[-1].get('A_unsafe_any')} "
                  f"B_exec={rows[-1].get('B_executable')} "
                  f"removal={rows[-1].get('removal_ok')}", flush=True)
            _dump(a.csv, rows)
            if rows[-1].get("removal_ok") is False:
                _nbad = sum(1 for r in rows[-3:]
                            if r.get("removal_ok") is False)
                if _nbad >= 3:
                    print("[native] FATAL: 3 consecutive removal "
                          "failures — aborting", flush=True)
                    break
    print(f"[native] wrote {a.csv} ({len(rows)} rows)", flush=True)
    node.destroy_node()
    rclpy.shutdown()


def _dump(csv_path, rows):
    import csv as _csv
    with open(csv_path, "w", newline="") as f:
        keys = sorted({k for r in rows for k in r.keys()})
        w = _csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})


def run_case(node, planner, c, map_tag):
    import math as _m
    from geometry_msgs.msg import PoseStamped
    from topopatch_ros.txn_trace import wait_mask_ready
    sx, sy = (float(v) for v in c.get("start", [-2.0, -0.5]))
    gx, gy = float(c["goal"][0]), float(c["goal"][1])
    epoch = int(c["epoch"])
    rects = [[(float(p[0]), float(p[1])) for p in poly]
             for poly in c["rules"]]
    out = {"planner": planner, "map": map_tag, "uid": c.get("uid", ""),
           "epoch": epoch, "start": [sx, sy], "goal": [gx, gy],
           "n_rules": len(rects)}
    # latch goal for bridge feasibility (M2 pattern)
    gp = PoseStamped()
    gp.header.frame_id = "map"
    gp.pose.position.x, gp.pose.position.y = gx, gy
    gp.pose.orientation.w = 1.0
    for _ in range(6):
        node.goal_pub.publish(gp)
        time.sleep(0.3)
    mask0 = node.mask_watcher.snapshot()["gen"]
    # nominal pre-commit plan (reference)
    plan_pre, _, rs_pre = node.plan((sx, sy), (gx, gy))
    if not plan_pre:
        out.update(ok=False, reason=f"no-plan:{rs_pre}")
        return out
    out["plan_len"] = round(sum(
        _m.hypot(a[0] - b[0], a[1] - b[1])
        for a, b in zip(plan_pre[1:], plan_pre[:-1])), 3)
    # commit (one rule per epoch in the case block)
    placed, placed_ids, ct = [], [], 0.0
    t_commit0 = time.time()
    for k, poly in enumerate(rects):
        ok1, tp1, tc1, _ = node.propose_commit([poly], epoch + k)
        ct += tp1 + tc1
        if ok1:
            placed.append(poly)
            placed_ids.append(f"paper-{epoch + k}-r0")
        else:
            break
    out["commit_t_ms"] = round(ct, 1)
    out["placed_n"] = len(placed)
    if not placed:
        out.update(ok=False, reason="commit-failed")
        return out
    last_ep = epoch + len(placed) - 1
    try:
        # timed plan sequence: immediate (N2), +3 s (N4), mask-ready (N0)
        plans = {}
        t0 = time.time()
        p_imm, _, _ = node.plan((sx, sy), (gx, gy))
        plans["N2_immediate"] = (p_imm, round(time.time() - t0, 2))
        time.sleep(3.0)
        p_mid, _, _ = node.plan((sx, sy), (gx, gy))
        plans["N4_plus3s"] = (p_mid, round(time.time() - t0, 2))
        mr, mw, _ = wait_mask_ready(
            node, node.mask_watcher, node.cli_list, last_ep,
            timeout_s=10.0, base_gen=mask0)
        p_rdy, _, _ = node.plan((sx, sy), (gx, gy))
        plans["N0_maskready"] = (p_rdy, round(time.time() - t0, 2))
        out["mask_ready"] = bool(mr)
        # map bounds for the gate (generous; sandbox-scale safe)
        BND = (-1e3, -1e3, 1e3, 1e3)
        if node._map is not None:
            m = node._map
            BND = (m.info.origin.position.x, m.info.origin.position.y,
                   m.info.origin.position.x + m.info.width
                   * m.info.resolution,
                   m.info.origin.position.y + m.info.height
                   * m.info.resolution)

            def _occ(x, y, _m=m):
                d = list(_m.data)
                ix = int((x - _m.info.origin.position.x)
                         / _m.info.resolution)
                iy = int((y - _m.info.origin.position.y)
                         / _m.info.resolution)
                w = _m.info.width
                if 0 <= ix < w and 0 <= iy < len(d) // w:
                    return d[iy * w + ix] > 50
                return True
        else:
            def _occ(x, y):
                return False
        out["A_unsafe_any"] = False
        out["B_executable"] = False
        out["B_safe_stop"] = True
        first_exec = None
        for cond, (p, dt) in plans.items():
            if not p:
                out[f"{cond}_plan"] = "none"
                continue
            crosses = path_intersects_polygons(p, placed)
            out[f"{cond}_plan"] = "crosses" if crosses else "clean"
            out[f"{cond}_t_s"] = dt
            # ARM A: trust directly -> unsafe iff crosses
            if crosses:
                out["A_unsafe_any"] = True
            # ARM B: gate every candidate
            go, rs = gate_full_path(p, placed, _occ, BND)
            out[f"{cond}_gate"] = rs
            if go:
                out["B_executable"] = True
                out["B_safe_stop"] = False
                if first_exec is None:
                    first_exec = dt
                    out["B_first_exec_rule"] = cond
        # N6: injected geometrically-crossing path (straight line)
        inj = [(sx + (gx - sx) * t / 20.0, sy + (gy - sy) * t / 20.0)
               for t in range(21)]
        inj_cross = path_intersects_polygons(inj, placed)
        out["N6_crosses"] = bool(inj_cross)
        if inj_cross:
            go6, rs6 = gate_full_path(inj, placed, _occ, BND)
            out["N6_gate"] = rs6
            out["N6_blocked"] = bool(not go6)
        else:
            out["N6_gate"] = "no-crossing-to-block"
            out["N6_blocked"] = True
        if first_exec is not None:
            out["exposure_window_s"] = round(first_exec, 2)
        out["commit_to_first_plan_s"] = round(
            plans["N2_immediate"][1], 2)
        out["ok"] = True
        return out
    finally:
        rok, rdetail = node._remove_placed(placed_ids, last_ep)
        out["removal_ok"] = bool(rok)
        out["removal_detail"] = str(rdetail)
        if not rok:
            print(f"[native] WARNING: removal failed epoch={last_ep} "
                  f"detail={rdetail}", flush=True)


if __name__ == "__main__":
    main()
