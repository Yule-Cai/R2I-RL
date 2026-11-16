"""RulePatchNet teacher-pair generator (runs on onexplayer, ROS_DOMAIN_ID=7).

Real Nav2 Navfn plans on warehouse map:
  sample free (s,g) -> plan orig -> rect keepout crossing orig ->
  publish keepout mask+filter-info -> settle -> plan replan (must avoid rect).
Saves shards (npz) + manifest.jsonl + CHECKSUMS. Checkpoint-resumable.

Usage:
  source /opt/ros/lyrical/setup.bash && export ROS_DOMAIN_ID=7 &&
  python3 gen_dataset.py --n-target 11000 --seed 7 --out-dir ~/rulepatch/data
"""
import argparse
import hashlib
import json
import math
import os
import random
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.action import ComputePathToPose

MAP_PGM = "/opt/ros/lyrical/share/nav2_bringup/maps/warehouse.pgm"
MAP_RES, MAP_OX, MAP_OY = 0.03, -15.1, -25.0


def _cfg(args):
    """Override map geometry from CLI (sandbox runs)."""
    global MAP_PGM, MAP_RES, MAP_OX, MAP_OY
    if args.map_pgm:
        MAP_PGM = args.map_pgm
        MAP_RES, MAP_OX, MAP_OY = args.map_res, args.map_ox, args.map_oy
MASK_TOPIC = "/rulesync/keepout_mask"
INFO_TOPIC = "/keepout_costmap_filter_info"


def pip(xy, pts) -> bool:
    x, y = float(xy[0]), float(xy[1])
    inside, n = False, len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if ((y0 > y) != (y1 > y)) and (
            x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0
        ):
            inside = not inside
    return inside


def count_inside(path, poly) -> int:
    return sum(1 for xy in path if pip(xy, poly))


class Gen(Node):
    def __init__(self):
        super().__init__("rp_gen_dataset")
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub = self.create_publisher(OccupancyGrid, MASK_TOPIC, q)
        self.info_pub = self.create_publisher(CostmapFilterInfo, INFO_TOPIC, q)
        self.ac = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        if not self.ac.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("compute_path_to_pose unavailable")

    def pose(self, p):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        m.pose.orientation.w = 1.0
        return m

    def plan(self, s, g, timeout=30.0):
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal = self.pose(s), self.pose(g)
        goal.planner_id = "GridBased"
        goal.use_start = True
        t0 = time.time()
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None, -1.0
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        r = rf.result()
        if r is None:
            return None, -1.0
        path = [(p.pose.position.x, p.pose.position.y)
                for p in r.result.path.poses]
        return (path if len(path) >= 4 else None), time.time() - t0

    def publish_mask(self, rect_or_none):
        w, h = self.map_w, self.map_h
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.info.width, g.info.height, g.info.resolution = w, h, MAP_RES
        g.info.origin.position.x, g.info.origin.position.y = MAP_OX, MAP_OY
        data = [0] * (w * h)
        if rect_or_none is not None:
            xs = [p[0] for p in rect_or_none]
            ys = [p[1] for p in rect_or_none]
            gx0 = max(0, int((min(xs) - MAP_OX) / MAP_RES))
            gy0 = max(0, int((min(ys) - MAP_OY) / MAP_RES))
            gx1 = min(w - 1, int((max(xs) - MAP_OX) / MAP_RES))
            gy1 = min(h - 1, int((max(ys) - MAP_OY) / MAP_RES))
            for gy in range(gy0, gy1 + 1):
                for gx in range(gx0, gx1 + 1):
                    if pip((MAP_OX + (gx + 0.5) * MAP_RES,
                            MAP_OY + (gy + 0.5) * MAP_RES), rect_or_none):
                        data[gy * w + gx] = 100
        g.data = data
        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.type, info.base, info.multiplier = 0, 0.0, 1.0
        info.filter_mask_topic = MASK_TOPIC
        for _ in range(3):
            self.mask_pub.publish(g)
            self.info_pub.publish(info)
            time.sleep(0.15)


def load_free(rng_seed, clear_m=0.36):
    from PIL import Image
    img = np.asarray(Image.open(MAP_PGM).convert("L"))
    h, w = img.shape
    free = (img >= 254).astype(np.uint8)  # 254/255 free; 205 unknown excluded
    # (unknown endpoints make Navfn return error 208; verified empirically)
    # box clearance via integral image; window sums (h,w)
    r = max(1, int(round(clear_m / MAP_RES)))
    sat = np.pad(free, r + 1, mode="constant").cumsum(0).cumsum(1)
    A = sat[2 * r + 2:, 2 * r + 2:]
    B = sat[:-(2 * r + 2), 2 * r + 2:]
    C = sat[2 * r + 2:, :-(2 * r + 2)]
    D = sat[:-(2 * r + 2), :-(2 * r + 2)]
    box = A - B - C + D
    assert box.shape == (h, w), box.shape
    clear = (box == (2 * r + 1) ** 2).astype(np.uint8)
    ys, xs = np.where(clear > 0)
    print(f"[gen] map {w}x{h}, free-clear cells: {len(xs)}", flush=True)
    return img, xs, ys, w, h


def cell_to_world(x, y, h):
    return MAP_OX + (x + 0.5) * MAP_RES, MAP_OY + (h - 1 - y + 0.5) * MAP_RES


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-target", type=int, default=11000)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out-dir", default="/home/yule/rulepatch/data")
    ap.add_argument("--settle", type=float, default=0.45)
    ap.add_argument("--shard-every", type=int, default=500)
    ap.add_argument("--plan-timeout", type=float, default=30.0)
    ap.add_argument("--map-pgm", default="")
    ap.add_argument("--map-res", type=float, default=0.03)
    ap.add_argument("--map-ox", type=float, default=-15.1)
    ap.add_argument("--map-oy", type=float, default=-25.0)
    ap.add_argument("--clear-m", type=float, default=0.36)
    ap.add_argument("--rect-min", type=float, default=1.5)
    ap.add_argument("--rect-max", type=float, default=3.0)
    ap.add_argument("--off-max", type=float, default=0.3)
    ap.add_argument("--dist-min", type=float, default=6.0)
    ap.add_argument("--dist-max", type=float, default=15.0)
    ap.add_argument("--no-east-bias", action="store_true")
    args = ap.parse_args()
    _cfg(args)

    os.makedirs(args.out_dir, exist_ok=True)
    ckpt_path = os.path.join(args.out_dir, "ckpt.json")
    rng_path = os.path.join(args.out_dir, "ckpt_rng.pkl")
    man_path = os.path.join(args.out_dir, "manifest.jsonl")
    rng = random.Random(args.seed)
    accepted, attempts, skipped = 0, 0, 0
    STATS: dict = {}
    import pickle
    if os.path.exists(ckpt_path) and os.path.exists(rng_path):
        ck = json.load(open(ckpt_path))
        accepted, attempts, skipped = ck["accepted"], ck["attempts"], ck["skipped"]
        rng.setstate(pickle.load(open(rng_path, "rb")))
        print(f"[gen] resume: accepted={accepted} attempts={attempts}", flush=True)

    rclpy.init()
    gen = Gen()
    try:
        img, fxs, fys, W, H = load_free(args.seed, clear_m=args.clear_m)
        gen.map_w, gen.map_h = W, H
        # east pool (world x>4) to guarantee held-out test-region volume
        # (warehouse only; disabled for other maps via --no-east-bias).
        wx_all = MAP_OX + (fxs + 0.5) * MAP_RES
        east_mask = wx_all > 4.0
        exs, eys = fxs[east_mask], fys[east_mask]
        print(f"[gen] east-pool cells: {len(exs)}", flush=True)
        buf = []  # pending records for current shard
        man = open(man_path, "a")
        flush_counter = [0]
        # resume shard counter from existing shards
        import glob as _glob
        _existing = _glob.glob(os.path.join(args.out_dir, "shard_*.npz"))
        if _existing:
            nums = [int(os.path.basename(p)[6:10]) for p in _existing]
            flush_counter[0] = max(nums) + 1

        def flush():
            nonlocal buf
            if not buf:
                return
            sid = flush_counter[0]
            flush_counter[0] += 1
            fp = os.path.join(args.out_dir, f"shard_{sid:04d}.npz")
            kw = {"n": np.array([len(buf)]),
                  "uids": np.array([b["uid"] for b in buf]),
                  "start": np.array([b["start"] for b in buf], np.float32),
                  "goal": np.array([b["goal"] for b in buf], np.float32),
                  "orig_len": np.array([b["orig_len"] for b in buf], np.float32),
                  "repl_len": np.array([b["repl_len"] for b in buf], np.float32)}
            for i, b in enumerate(buf):
                kw[f"orig_{i}"] = np.asarray(b["orig"], np.float32)
                kw[f"repl_{i}"] = np.asarray(b["repl"], np.float32)
                kw[f"rect_{i}"] = np.asarray(b["rect"], np.float32)
            np.savez_compressed(fp, **kw)
            buf = []

        while accepted < args.n_target:
            attempts += 1
            # sample s,g with configured separation (connected plans)
            # 30%: both endpoints eastern (feeds held-out test region x>9)
            pool = (exs, eys) if (not args.no_east_bias and exs is not None
                                  and len(exs) > 0
                                  and rng.random() < 0.30) else (fxs, fys)
            px, py = pool
            for _ in range(60):
                i, j = rng.randrange(len(px)), rng.randrange(len(px))
                s = cell_to_world(int(px[i]), int(py[i]), H)
                g = cell_to_world(int(px[j]), int(py[j]), H)
                dd = math.hypot(s[0] - g[0], s[1] - g[1])
                if args.dist_min <= dd <= args.dist_max:
                    break
            else:
                STATS['sample'] = STATS.get('sample', 0) + 1
                skipped += 1
                continue
            gen.publish_mask(None)  # clear previous keepout
            time.sleep(args.settle)
            orig, t0 = gen.plan(s, g, args.plan_timeout)
            if orig is None:
                STATS['orig_plan_fail'] = STATS.get('orig_plan_fail', 0) + 1
                skipped += 1
                continue
            # keepout rect at a random 30-70% point along orig (anchor variety)
            kf = rng.uniform(0.30, 0.70)
            ki = min(len(orig) - 1, max(1, int(len(orig) * kf)))
            mid = orig[ki]
            j0, j1 = max(0, ki - 5), min(len(orig) - 1, ki + 5)
            # rect: random size/offset crossing the path at mid
            dx = orig[min(len(orig) - 1, j1)][0] - orig[max(0, j0)][0]
            dy = orig[min(len(orig) - 1, j1)][1] - orig[max(0, j0)][1]
            L = math.hypot(dx, dy) + 1e-9
            nx, ny = -dy / L, dx / L  # path normal
            off = rng.uniform(-args.off_max, args.off_max)
            cx, cy = mid[0] + nx * off, mid[1] + ny * off
            rw, rh = (rng.uniform(args.rect_min, args.rect_max),
                        rng.uniform(args.rect_min, args.rect_max))
            rect = [(cx - rw / 2, cy - rh / 2), (cx - rw / 2, cy + rh / 2),
                    (cx + rw / 2, cy + rh / 2), (cx + rw / 2, cy - rh / 2)]
            if pip(s, rect) or pip(g, rect):
                STATS['endpoint_in_rect'] = STATS.get('endpoint_in_rect', 0) + 1
                skipped += 1
                continue
            if count_inside(orig, rect) < 5:
                STATS['rect_miss'] = STATS.get('rect_miss', 0) + 1
                skipped += 1
                continue
            gen.publish_mask(rect)
            time.sleep(args.settle)
            repl, t1 = gen.plan(s, g, args.plan_timeout)
            if repl is None:
                STATS['repl_plan_fail'] = STATS.get('repl_plan_fail', 0) + 1
                skipped += 1
                continue
            if count_inside(repl, rect) > 0:
                STATS['replan_inside'] = STATS.get('replan_inside', 0) + 1
                skipped += 1  # keepout not yet absorbed or no detour; skip
                continue
            ol = sum(math.hypot(a[0] - b[0], a[1] - b[1])
                     for a, b in zip(orig[1:], orig[:-1]))
            rl = sum(math.hypot(a[0] - b[0], a[1] - b[1])
                     for a, b in zip(repl[1:], repl[:-1]))
            if not (ol > 1.0 and rl < 3.0 * ol):
                STATS['length'] = STATS.get('length', 0) + 1
                skipped += 1
                continue
            uid = f"rp{accepted:06d}"
            rec = {"uid": uid, "start": s, "goal": g, "orig": orig,
                   "repl": repl, "rect": rect, "orig_len": ol, "repl_len": rl}
            buf.append(rec)
            midx = sum(p[0] for p in orig) / len(orig)
            man.write(json.dumps({
                "uid": uid, "split_proposal": "test" if midx > 9.0 else "train",
                "n_orig": len(orig), "n_repl": len(repl),
                "orig_len": round(ol, 2), "repl_len": round(rl, 2),
                "plan_t": [round(t0, 3), round(t1, 3)],
                "attempt": attempts}) + "\n")
            man.flush()
            accepted += 1
            if accepted % args.shard_every == 0:
                flush()
                man.flush()
            if accepted % 100 == 0:
                rate = accepted / max(1, attempts)
                print(f"[gen] accepted={accepted} attempts={attempts} "
                      f"skipped={skipped} rate={rate:.2f} stats={STATS}", flush=True)
                json.dump({"accepted": accepted, "attempts": attempts,
                           "skipped": skipped}, open(ckpt_path, "w"))
                pickle.dump(rng.getstate(), open(rng_path, "wb"))
        flush()
        man.close()
        gen.publish_mask(None)
        print(f"[gen] DONE accepted={accepted} attempts={attempts} skipped={skipped} "
              f"stats={STATS}", flush=True)
    finally:
        gen.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
