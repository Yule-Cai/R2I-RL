"""Probe Nav2 plan feasibility on a topo map (diagnostic, runs onexplayer).

Samples K random free-clear (s,g) pairs and reports plan success/timeout
rate without any rules. Usage:
  python3 probe_plan.py --pgm <map.pgm> --res 0.05 --ox -10 --oy -10 \
      --clear-m 0.36 --k 20 --dist-min 3 --dist-max 7 --timeout 8
"""
import argparse
import random
import time

import numpy as np
from PIL import Image


def clearance_mask(img, res, clear_m):
    free = (np.asarray(img) >= 254).astype(np.uint8)
    h, w = free.shape
    r = max(1, int(round(clear_m / res)))
    sat = np.pad(free, r + 1, mode="constant").astype(np.int64)
    sat = sat.cumsum(0).cumsum(1)
    A = sat[2 * r + 2:, 2 * r + 2:]
    B = sat[1:-(2 * r + 1), 2 * r + 2:]
    C = sat[2 * r + 2:, 1:-(2 * r + 1)]
    D = sat[1:-(2 * r + 1), 1:-(2 * r + 1)]
    return (A - B - C + D) == (2 * r + 1) ** 2


def publish_empty(mask_pub, info_pub):
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import CostmapFilterInfo
    g = OccupancyGrid()
    g.header.frame_id = "map"
    g.info.width, g.info.height, g.info.resolution = 384, 384, 0.05
    g.info.origin.position.x, g.info.origin.position.y = -10.0, -10.0
    g.data = [0] * (384 * 384)
    info = CostmapFilterInfo()
    info.header.frame_id = "map"
    info.filter_mask_topic = "/rulesync/keepout_mask"
    for _ in range(2):
        mask_pub.publish(g)
        info_pub.publish(info)
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--ox", type=float, default=-10.0)
    ap.add_argument("--oy", type=float, default=-10.0)
    ap.add_argument("--clear-m", type=float, default=0.36)
    ap.add_argument("--k", type=int, default=20)
    ap.add_argument("--dist-min", type=float, default=3.0)
    ap.add_argument("--dist-max", type=float, default=7.0)
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--with-rule", action="store_true",
                    help="publish a rect KEEP_OUT across path midpoint, "
                    "then plan the replan (tests mask+filter+repl path)")
    a = ap.parse_args()
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    img = np.asarray(Image.open(a.pgm).convert("L"))
    h, w = img.shape
    clear = clearance_mask(img, a.res, a.clear_m)
    ys, xs = np.where(clear)
    rng = random.Random(a.seed)

    def cell_to_world(x, y):
        return (a.ox + (x + 0.5) * a.res,
                a.oy + (h - 1 - y + 0.5) * a.res)

    rclpy.init()
    node = rclpy.create_node("probe_plan")
    ac = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    assert ac.wait_for_server(timeout_sec=30.0), "no action server"
    mask_pub = info_pub = None
    if a.with_rule:
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.msg import CostmapFilterInfo
        q = QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        mask_pub = node.create_publisher(
            OccupancyGrid, "/rulesync/keepout_mask", q)
        info_pub = node.create_publisher(
            CostmapFilterInfo, "/keepout_costmap_filter_info", q)

    def publish_rect(cx, cy, sx=1.5, sy=1.5):
        from nav_msgs.msg import OccupancyGrid
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.info.width, g.info.height, g.info.resolution = 384, 384, a.res
        g.info.origin.position.x, g.info.origin.position.y = a.ox, a.oy
        data = [0] * (384 * 384)
        for gy in range(384):
            wy = a.oy + (384 - 1 - gy + 0.5) * a.res
            for gx in range(384):
                wx = a.ox + (gx + 0.5) * a.res
                if abs(wx - cx) <= sx / 2 and abs(wy - cy) <= sy / 2:
                    data[gy * 384 + gx] = 100
        g.data = data
        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.type, info.base, info.multiplier = 0, 0.0, 1.0
        info.filter_mask_topic = "/rulesync/keepout_mask"
        for _ in range(3):
            mask_pub.publish(g)
            info_pub.publish(info)
            time.sleep(0.15)
    ok = tout = fail = 0
    ts = []
    for t in range(a.k):
        for _ in range(60):
            i, j = rng.randrange(len(xs)), rng.randrange(len(xs))
            s = cell_to_world(int(xs[i]), int(ys[i]))
            g = cell_to_world(int(xs[j]), int(ys[j]))
            dd = ((s[0] - g[0]) ** 2 + (s[1] - g[1]) ** 2) ** 0.5
            if a.dist_min <= dd <= a.dist_max:
                break
        goal = ComputePathToPose.Goal()
        from geometry_msgs.msg import PoseStamped

        def ps(p):
            m = PoseStamped()
            m.header.frame_id = "map"
            m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
            m.pose.orientation.w = 1.0
            return m

        goal.goal = ps(g)
        goal.start = ps(s)
        goal.planner_id = "GridBased"
        goal.use_start = True
        # first get the clean path; with --with-rule, block its midpoint
        # with a rect mask and replan (mirrors the generator's repl stage)
        t0 = time.time()
        fh = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fh, timeout_sec=a.timeout)
        if fh.result() is None:
            tout += 1
            continue
        gh = fh.result()
        if not gh.accepted:
            fail += 1
            continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=a.timeout)
        dt = time.time() - t0
        if rf.result() is None:
            tout += 1
            continue
        path = [(p.pose.position.x, p.pose.position.y)
                for p in rf.result().result.path.poses]
        if a.with_rule and len(path) >= 4:
            mx = sum(p[0] for p in path) / len(path)
            my = sum(p[1] for p in path) / len(path)
            publish_rect(mx, my)
            time.sleep(0.45)
            t1 = time.time()
            fh2 = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fh2,
                                             timeout_sec=a.timeout)
            dt = time.time() - t1
            gh2 = fh2.result() if fh2.done() else None
            if gh2 is None or not gh2.accepted:
                fail += 1
                publish_empty(mask_pub, info_pub)
                continue
            rf2 = gh2.get_result_async()
            rclpy.spin_until_future_complete(node, rf2,
                                             timeout_sec=a.timeout)
            dt = time.time() - t1
            r2 = rf2.result()
            publish_empty(mask_pub, info_pub)
            if r2 is None or len(r2.result.path.poses) < 4:
                fail += 1
            else:
                ok += 1
                ts.append(dt)
        elif len(path) >= 4:
            ok += 1
            ts.append(dt)
        else:
            fail += 1
    print(f"[probe] ok={ok} timeout={tout} reject={fail} "
          f"med_t={sorted(ts)[len(ts)//2] if ts else -1:.2f}s")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
