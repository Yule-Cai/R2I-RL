"""Pre-score Gazebo demo scenes offline (M2 methodology, documented).

For a goal on the sandbox map: get the Nav2 plan via action, generate
candidate KEEP_OUT rects around the plan midpoint, score each with the
fused TopoRulePatch model + deterministic verifier, and write the best
ACCEPT scene (plus all-fail rate) to cases.json for the live driver.

Usage (onexplayer, minimal planner stack up on default domain):
  PYTHONPATH=<repo>:<repo>/rulepatch source-ros &&
  python3 -m topopatch.tools.prescore_gazebo --pgm <tb3_sandbox.pgm> \\
      --weights <best_fused.npz> --tau 1.026 --goal 2.23,-0.88 \\
      --epoch 101 --out cases.json
"""
import argparse
import json
import math
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--tau", type=float, default=1.0264495134353637)
    ap.add_argument("--goal", required=True)
    ap.add_argument("--start", default="-2.0,-0.5")
    ap.add_argument("--epoch", type=int, default=101)
    ap.add_argument("--sizes", default="1.0,1.5")
    ap.add_argument("--offsets", default="-0.6,-0.3,0.0,0.3,0.6")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    gx, gy = (float(v) for v in a.goal.split(","))
    sx, sy = (float(v) for v in a.start.split(","))

    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = rclpy.create_node("prescore_gazebo")
    ac = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    assert ac.wait_for_server(timeout_sec=30.0), "no planner action"

    def ps(p):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        m.pose.orientation.w = 1.0
        return m

    goal = ComputePathToPose.Goal()
    goal.start, goal.goal = ps((sx, sy)), ps((gx, gy))
    goal.planner_id = "GridBased"
    goal.use_start = True
    f = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=30.0)
    gh = f.result()
    assert gh is not None and gh.accepted, "plan rejected"
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
    r = rf.result()
    assert r is not None
    plan = [(p.pose.position.x, p.pose.position.y)
            for p in r.result.path.poses]
    assert len(plan) >= 4, "plan too short"
    node.destroy_node()
    rclpy.shutdown()

    mid = plan[len(plan) // 2]
    j0, j1 = max(0, len(plan) // 2 - 5), min(len(plan) - 1,
                                             len(plan) // 2 + 5)
    dx = plan[j1][0] - plan[j0][0]
    dy = plan[j1][1] - plan[j0][1]
    L = math.hypot(dx, dy) + 1e-9
    nx, ny = -dy / L, dx / L

    from rulepatch.rp.extract import load_occ_grid, make_occ_fn
    from rulepatch.rp.bev import (add_coords, local_path_points, render_bev,
                                  window_for_patch)
    from rulepatch.rp import BEV_SIZE_M
    from topopatch.topo.extract_topo import _rule_feats
    from topopatch.topo.infer_numpy_topo import NumpyTopoNet
    from topopatch.topo.verifier_topo import select_topo
    occ = load_occ_grid(a.pgm, res_out=a.res, res_in=a.res, ox=a.ox,
                        oy=a.oy)
    occ_fn = make_occ_fn(occ)
    MAPB = (a.ox, a.oy, a.ox + 384 * a.res, a.oy + 384 * a.res)
    net = NumpyTopoNet(a.weights)

    scored = []
    for size in [float(v) for v in a.sizes.split(",")]:
        for off in [float(v) for v in a.offsets.split(",")]:
            cx, cy = mid[0] + nx * off, mid[1] + ny * off
            h = size / 2
            poly = [(cx - h, cy - h), (cx - h, cy + h),
                    (cx + h, cy + h), (cx + h, cy - h)]
            # anchors: span around the rule-affected region of the plan
            from rulepatch.rp.geom import count_inside
            inside = [i for i, p in enumerate(plan)
                      if count_inside([p], poly) > 0]
            if len(inside) < 3:
                scored.append({"size": size, "off": off,
                               "skip": "rule-misses-path"})
                continue
            iA = max(0, inside[0] - 8)
            iB = min(len(plan) - 1, inside[-1] + 8)
            A, B = tuple(plan[iA]), tuple(plan[iB])
            if math.hypot(B[0] - A[0], B[1] - A[1]) < 1.0:
                scored.append({"size": size, "off": off,
                               "skip": "span-short"})
                continue
            center = window_for_patch(A, B, MAPB)
            seg = plan[iA:iB + 1]
            bev = render_bev(occ_fn, center, seg, poly, (gx, gy))
            np.clip(bev, 0, 1, out=bev)
            local = local_path_points(seg, center)
            rf = _rule_feats([poly], A, B, center)
            ctrls, confs, topo = net.forward(
                add_coords(bev.astype(np.float64)),
                local.astype(np.float64), rf.astype(np.float64))
            idx, dense, reason = select_topo(
                ctrls, confs, center, A, B, seg, [poly], occ_fn, MAPB,
                tau=a.tau)
            scored.append({"size": size, "off": off, "poly": poly,
                           "idx": idx, "reason": reason,
                           "max_conf": float(np.max(confs)),
                           "topo_pred": int(np.argmax(topo))})
    scored_ok = [s for s in scored if s.get("idx") is not None]
    scored_ok.sort(key=lambda s: -s["max_conf"])
    case = {"goal": [gx, gy], "epoch": a.epoch, "plan": plan,
            "n_candidates": len(scored),
            "n_accept": len(scored_ok),
            "scored": [{k: v for k, v in s.items() if k != "poly"}
                       for s in scored]}
    if scored_ok:
        best = scored_ok[0]
        case["rule"] = best["poly"]
        case["expect"] = "accept"
        case["expect_conf"] = best["max_conf"]
        case["expect_topo"] = best["topo_pred"]
    else:
        case["rule"] = None
        case["expect"] = "all-fail"
    json.dump(case, open(a.out, "w"), indent=1)
    print(json.dumps({k: v for k, v in case.items()
                      if k not in ("plan", "scored")}, indent=1))


if __name__ == "__main__":
    main()
