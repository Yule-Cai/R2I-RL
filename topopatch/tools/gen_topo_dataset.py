"""TopoRulePatch teacher-pair generator (runs on onexplayer, ROS_DOMAIN_ID=7).

Extends the M2 pattern (frozen `rulepatch/tools/gen_dataset.py`, NOT
modified) to: multiple KEEP_OUT geometries (rect / convex blob /
non-convex notch / narrow closure wall / elongated barrier) with 1-3
simultaneously active rules per pair, on any pgm map.

Flow per pair: sample free (s,g) -> plan orig -> place k rule polygons
crossing orig (pairwise separated) -> publish union keepout mask -> settle
-> plan replan (must avoid ALL rules). Saves shards (npz) + manifest.
Checkpoint-resumable.

Usage:
  source /opt/ros/lyrical/setup.bash && export ROS_DOMAIN_ID=7 &&
  python3 gen_topo_dataset.py --map-pgm <map.pgm> --map-res 0.05 \
      --map-ox -10.0 --map-oy -10.0 --map-id office-A-s101 \
      --n-target 2000 --seed 11 --out-dir ~/rulepatch/topo_data/office-A-s101
"""
import argparse
import hashlib
import json
import math
import os
import random
import time

import numpy as np

MAP_PGM = ""
MAP_RES, MAP_OX, MAP_OY = 0.05, -10.0, -10.0
MASK_TOPIC = "/rulesync/keepout_mask"
INFO_TOPIC = "/keepout_costmap_filter_info"

GEOM_TYPES = ("rect", "convex", "notch", "narrow", "elongated")


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


def _rot(pts, cx, cy, theta):
    c, s = math.cos(theta), math.sin(theta)
    return [(cx + (x - cx) * c - (y - cy) * s,
             cy + (x - cx) * s + (y - cy) * c) for x, y in pts]


def sample_rect(rng, cx, cy, smin, smax):
    w = rng.uniform(smin, smax)
    h = rng.uniform(smin, smax)
    return ([(cx - w / 2, cy - h / 2), (cx - w / 2, cy + h / 2),
             (cx + w / 2, cy + h / 2), (cx + w / 2, cy - h / 2)],
            "rect")


def sample_convex(rng, cx, cy, smin, smax):
    rx = rng.uniform(smin, smax) / 2.0
    ry = rng.uniform(smin, smax) / 2.0
    th = rng.uniform(0, math.pi)
    k = rng.randint(5, 9)
    pts = []
    for i in range(k):
        a = 2 * math.pi * i / k
        pts.append((rx * math.cos(a), ry * math.sin(a)))
    return _rot(pts, cx, cy, th), "convex"


def sample_notch(rng, cx, cy, smin, smax):
    # non-convex: rect with a rectangular bite (8 vertices).
    w = rng.uniform(smin, smax)
    h = rng.uniform(smin, smax)
    bw, bh = w * rng.uniform(0.3, 0.5), h * rng.uniform(0.3, 0.5)
    x0, y0 = cx - w / 2, cy - h / 2
    pts = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h),
           (x0 + w - bw, y0 + h), (x0 + w - bw, y0 + h - bh),
           (x0 + bw, y0 + h - bh), (x0 + bw, y0 + h), (x0, y0 + h)]
    return _rot(pts, cx, cy, rng.uniform(0, math.pi)), "notch"


def sample_narrow(rng, cx, cy, heading, smin, smax):
    # thin wall perpendicular to path: closes a corridor.
    length = rng.uniform(max(2.0, smin), smax + 1.5)
    width = rng.uniform(0.3, 0.5)
    pts = [(-length / 2, -width / 2), (-length / 2, width / 2),
           (length / 2, width / 2), (length / 2, -width / 2)]
    return _rot(pts, cx, cy, heading + math.pi / 2), "narrow"


def sample_elongated(rng, cx, cy, heading, smin, smax, off_max):
    # long thin barrier along the path, slight lateral offset.
    length = rng.uniform(4.0, 7.0)
    width = rng.uniform(0.4, 0.6)
    pts = [(-length / 2, -width / 2), (-length / 2, width / 2),
           (length / 2, width / 2), (length / 2, -width / 2)]
    return _rot(pts, cx, cy, heading), "elongated"


def sample_n_rules(rng):
    r = rng.random()
    if r < 0.5:
        return 1
    if r < 0.8:
        return 2
    return 3


class Gen:
    """ROS Nav2 planning + keepout-mask driver (M2-proven pattern)."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.msg import CostmapFilterInfo
        from nav2_msgs.action import ComputePathToPose
        self._rclpy = rclpy
        self._ComputePathToPose = ComputePathToPose
        rclpy.init()
        self.node = Node("rp_gen_topo_dataset")
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub = self.node.create_publisher(OccupancyGrid,
                                                   MASK_TOPIC, q)
        self.info_pub = self.node.create_publisher(CostmapFilterInfo,
                                                   INFO_TOPIC, q)
        self.ac = ActionClient(self.node, ComputePathToPose,
                               "compute_path_to_pose")
        if not self.ac.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("compute_path_to_pose unavailable")
        from geometry_msgs.msg import PoseStamped
        self._PoseStamped = PoseStamped
        self._OccupancyGrid = OccupancyGrid
        self._CostmapFilterInfo = CostmapFilterInfo

    def pose(self, p):
        m = self._PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        m.pose.orientation.w = 1.0
        return m

    def plan(self, s, g, timeout=30.0):
        goal = self._ComputePathToPose.Goal()
        goal.start, goal.goal = self.pose(s), self.pose(g)
        goal.planner_id = "GridBased"
        goal.use_start = True
        t0 = time.time()
        f = self.ac.send_goal_async(goal)
        self._rclpy.spin_until_future_complete(self.node, f,
                                              timeout_sec=timeout)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None, -1.0
        rf = gh.get_result_async()
        self._rclpy.spin_until_future_complete(self.node, rf,
                                              timeout_sec=timeout)
        r = rf.result()
        if r is None:
            return None, -1.0
        path = [(p.pose.position.x, p.pose.position.y)
                for p in r.result.path.poses]
        return (path if len(path) >= 4 else None), time.time() - t0

    def publish_mask(self, polys_or_none):
        from nav_msgs.msg import OccupancyGrid  # noqa (kept for clarity)
        w, h = self.map_w, self.map_h
        g = self._OccupancyGrid()
        g.header.frame_id = "map"
        g.info.width, g.info.height, g.info.resolution = w, h, MAP_RES
        g.info.origin.position.x, g.info.origin.position.y = MAP_OX, MAP_OY
        data = [0] * (w * h)
        if polys_or_none:
            xs = [p[0] for poly in polys_or_none for p in poly]
            ys = [p[1] for poly in polys_or_none for p in poly]
            gx0 = max(0, int((min(xs) - MAP_OX) / MAP_RES))
            gy0 = max(0, int((min(ys) - MAP_OY) / MAP_RES))
            gx1 = min(w - 1, int((max(xs) - MAP_OX) / MAP_RES))
            gy1 = min(h - 1, int((max(ys) - MAP_OY) / MAP_RES))
            for gy in range(gy0, gy1 + 1):
                for gx in range(gx0, gx1 + 1):
                    wx = MAP_OX + (gx + 0.5) * MAP_RES
                    wy = MAP_OY + (gy + 0.5) * MAP_RES
                    for poly in polys_or_none:
                        if pip((wx, wy), poly):
                            data[gy * w + gx] = 100
                            break
        g.data = data
        info = self._CostmapFilterInfo()
        info.header.frame_id = "map"
        info.type, info.base, info.multiplier = 0, 0.0, 1.0
        info.filter_mask_topic = MASK_TOPIC
        for _ in range(3):
            self.mask_pub.publish(g)
            self.info_pub.publish(info)
            time.sleep(0.15)

    def shutdown(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


def clearance_mask(img, res, clear_m=0.36):
    """Boolean mask of cells with a full (2r+1)^2 free box around them.

    Centered box sums via integral image (M2's formula has an off-by-one
    — it sums (2r+2)^2; corrected here; M2 file left frozen). img values
    >=254 count as free.
    """
    import numpy as _np
    free = (np.asarray(img) >= 254).astype(_np.uint8)
    h, w = free.shape
    r = max(1, int(round(clear_m / res)))
    sat = _np.pad(free, r + 1, mode="constant").astype(_np.int64)
    sat = sat.cumsum(0).cumsum(1)
    A = sat[2 * r + 2:, 2 * r + 2:]
    B = sat[1:-(2 * r + 1), 2 * r + 2:]
    C = sat[2 * r + 2:, 1:-(2 * r + 1)]
    D = sat[1:-(2 * r + 1), 1:-(2 * r + 1)]
    box = A - B - C + D
    assert box.shape == (h, w), box.shape
    return (box == (2 * r + 1) ** 2)


def load_free(clear_m=0.36):
    from PIL import Image
    img = np.asarray(Image.open(MAP_PGM).convert("L"))
    h, w = img.shape
    clear = clearance_mask(img, MAP_RES, clear_m)
    ys, xs = np.where(clear)
    print(f"[gen] map {w}x{h}, free-clear cells: {len(xs)}", flush=True)
    return img, xs, ys, w, h


def cell_to_world(x, y, h):
    return MAP_OX + (x + 0.5) * MAP_RES, MAP_OY + (h - 1 - y + 0.5) * MAP_RES


def try_place_rules(rng, orig, s, g, smin, smax, off_max):
    """Place 1-3 separated polygons crossing orig. Returns list or None."""
    n = sample_n_rules(rng)
    fracs = [0.5] if n == 1 else sorted(
        rng.uniform(0.30, 0.70) for _ in range(n))
    placed = []
    for fi, kf in enumerate(fracs):
        ki = min(len(orig) - 1, max(1, int(len(orig) * kf)))
        mid = orig[ki]
        j0, j1 = max(0, ki - 5), min(len(orig) - 1, ki + 5)
        dx = orig[min(len(orig) - 1, j1)][0] - orig[max(0, j0)][0]
        dy = orig[min(len(orig) - 1, j1)][1] - orig[max(0, j0)][1]
        heading = math.atan2(dy, dx)
        L = math.hypot(dx, dy) + 1e-9
        nx, ny = -dy / L, dx / L
        for _ in range(12):  # separation retries
            gtype = GEOM_TYPES[rng.randrange(len(GEOM_TYPES))]
            if gtype == "rect":
                off = rng.uniform(-off_max, off_max)
                poly, gt = sample_rect(
                    rng, mid[0] + nx * off, mid[1] + ny * off, smin, smax)
            elif gtype == "convex":
                off = rng.uniform(-off_max, off_max)
                poly, gt = sample_convex(
                    rng, mid[0] + nx * off, mid[1] + ny * off, smin, smax)
            elif gtype == "notch":
                off = rng.uniform(-off_max, off_max)
                poly, gt = sample_notch(
                    rng, mid[0] + nx * off, mid[1] + ny * off, smin, smax)
            elif gtype == "narrow":
                poly, gt = sample_narrow(
                    rng, mid[0], mid[1], heading, smin, smax)
            else:
                off = rng.uniform(-0.2, 0.2)
                poly, gt = sample_elongated(
                    rng, mid[0] + nx * off, mid[1] + ny * off,
                    heading, smin, smax, off_max)
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            if any(math.hypot(cx - q[0][0], cy - q[0][1]) < 1.2
                   for q in placed):
                continue  # too close to an already placed rule
            if pip(s, poly) or pip(g, poly):
                continue
            if count_inside(orig, poly) < 3:
                continue
            placed.append(((cx, cy), poly, gt))
            break
        else:
            return None
    out = [{"poly": poly, "geom": gt} for (_, poly, gt) in placed]
    # final gate: orig must cross >=1 rule (guaranteed per-rule above, but
    # keep the explicit check for the record).
    if sum(count_inside(orig, r["poly"]) for r in out) < 5:
        return None
    return out


def main():
    global MAP_PGM, MAP_RES, MAP_OX, MAP_OY
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-target", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--map-id", required=True)
    ap.add_argument("--map-pgm", required=True)
    ap.add_argument("--map-res", type=float, default=0.05)
    ap.add_argument("--map-ox", type=float, default=-10.0)
    ap.add_argument("--map-oy", type=float, default=-10.0)
    ap.add_argument("--settle", type=float, default=0.45)
    ap.add_argument("--shard-every", type=int, default=500)
    ap.add_argument("--plan-timeout", type=float, default=30.0)
    ap.add_argument("--clear-m", type=float, default=0.36)
    ap.add_argument("--rect-min", type=float, default=1.0)
    ap.add_argument("--rect-max", type=float, default=2.0)
    ap.add_argument("--off-max", type=float, default=0.3)
    ap.add_argument("--dist-min", type=float, default=4.0)
    ap.add_argument("--dist-max", type=float, default=12.0)
    args = ap.parse_args()
    MAP_PGM, MAP_RES, MAP_OX, MAP_OY = (args.map_pgm, args.map_res,
                                       args.map_ox, args.map_oy)

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
    elif os.path.exists(man_path) and os.path.getsize(man_path) > 0:
        # Manifest without checkpoint = unflushed rows from a killed run;
        # appending would duplicate counter-based uids. Quarantine it.
        import time as _t
        bak = man_path + f".bak-{int(_t.time())}"
        os.rename(man_path, bak)
        print(f"[gen] quarantined orphan manifest -> {bak}", flush=True)

    gen = Gen()
    try:
        img, fxs, fys, W, H = load_free(clear_m=args.clear_m)
        gen.map_w, gen.map_h = W, H
        buf = []
        man = open(man_path, "a")
        flush_counter = [0]
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
                  "map_id": np.array([args.map_id for b in buf]),
                  "start": np.array([b["start"] for b in buf], np.float32),
                  "goal": np.array([b["goal"] for b in buf], np.float32),
                  "n_rules": np.array([b["n_rules"] for b in buf], np.int64),
                  "geoms": np.array([",".join(b["geoms"]) for b in buf]),
                  "orig_len": np.array([b["orig_len"] for b in buf], np.float32),
                  "repl_len": np.array([b["repl_len"] for b in buf], np.float32)}
            for i, b in enumerate(buf):
                kw[f"orig_{i}"] = np.asarray(b["orig"], np.float32)
                kw[f"repl_{i}"] = np.asarray(b["repl"], np.float32)
                kw[f"polys_{i}"] = np.array(
                    [np.asarray(p, np.float32) for p in b["polys"]],
                    dtype=object)
            np.savez_compressed(fp, **kw)
            buf = []

        while accepted < args.n_target:
            attempts += 1
            for _ in range(60):
                i, j = rng.randrange(len(fxs)), rng.randrange(len(fxs))
                s = cell_to_world(int(fxs[i]), int(fys[i]), H)
                g = cell_to_world(int(fxs[j]), int(fys[j]), H)
                dd = math.hypot(s[0] - g[0], s[1] - g[1])
                if args.dist_min <= dd <= args.dist_max:
                    break
            else:
                STATS['sample'] = STATS.get('sample', 0) + 1
                skipped += 1
                continue
            gen.publish_mask(None)
            time.sleep(args.settle)
            orig, t0 = gen.plan(s, g, args.plan_timeout)
            if orig is None:
                STATS['orig_plan_fail'] = STATS.get('orig_plan_fail', 0) + 1
                skipped += 1
                continue
            rules = try_place_rules(rng, orig, s, g, args.rect_min,
                                    args.rect_max, args.off_max)
            if rules is None:
                STATS['place_fail'] = STATS.get('place_fail', 0) + 1
                skipped += 1
                continue
            gen.publish_mask([r["poly"] for r in rules])
            time.sleep(args.settle)
            repl, t1 = gen.plan(s, g, args.plan_timeout)
            if repl is None:
                STATS['repl_plan_fail'] = STATS.get('repl_plan_fail', 0) + 1
                skipped += 1
                continue
            bad = [r for r in rules if count_inside(repl, r["poly"]) > 0]
            if bad:
                STATS['replan_inside'] = STATS.get('replan_inside', 0) + 1
                skipped += 1
                continue
            ol = sum(math.hypot(a[0] - b[0], a[1] - b[1])
                     for a, b in zip(orig[1:], orig[:-1]))
            rl = sum(math.hypot(a[0] - b[0], a[1] - b[1])
                     for a, b in zip(repl[1:], repl[:-1]))
            if not (ol > 1.0 and rl < 3.0 * ol):
                STATS['length'] = STATS.get('length', 0) + 1
                skipped += 1
                continue
            uid = f"{args.map_id}-rp{accepted:06d}"
            rec = {"uid": uid, "start": s, "goal": g, "orig": orig,
                   "repl": repl, "polys": [r["poly"] for r in rules],
                   "geoms": [r["geom"] for r in rules],
                   "n_rules": len(rules), "orig_len": ol, "repl_len": rl}
            buf.append(rec)
            man.write(json.dumps({
                "uid": uid, "map_id": args.map_id,
                "n_rules": len(rules),
                "geom_types": [r["geom"] for r in rules],
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
                      f"skipped={skipped} rate={rate:.2f} stats={STATS}",
                      flush=True)
                json.dump({"accepted": accepted, "attempts": attempts,
                           "skipped": skipped}, open(ckpt_path, "w"))
                pickle.dump(rng.getstate(), open(rng_path, "wb"))
        flush()
        man.close()
        gen.publish_mask(None)
        print(f"[gen] DONE accepted={accepted} attempts={attempts} skipped={skipped} "
              f"stats={STATS}", flush=True)
    finally:
        gen.shutdown()


if __name__ == "__main__":
    main()
