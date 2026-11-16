"""Strong classical local-repair baseline (P0 reviewer risk).

Question: why use a learned local repair model instead of a classical
planner run locally between the same anchors?

This module implements local Theta* (any-angle A* with line-of-sight
shortcuts) between the SAME anchors A/B the learned model uses, on the
SAME information budget:
  - same occupancy grid / resolution / origin as the offline items,
  - same RuleSpec keepout polygons as lethal,
  - same collision margin (0.1 m radius, matching the verifier's
    0.1 m offset sampling),
  - same map bounds,
  - search restricted to the model's BEV window (BEV_SIZE_M around the
    case center) — the classical baseline may not roam the whole map.

It is deliberately given LESS shape information than the learned model
(anchors + occ + keepout only; no original-segment shape, no BEV
context features). If it still matches the learned repair, the paper's
learning claim must be reduced honestly.

Outputs are verified with the SAME deterministic gate
(verify_dense_multi) as learned hypotheses, so valid-repair rates are
directly comparable. No learning, no tuning, no randomness.
"""
import heapq
import math
import time

import numpy as np

COLL_RADIUS_M = 0.1


def _inflate_blocked_numpy(occ_grid, keepout_mask, radius_cells):
    """Inflate with pure numpy (no scipy dependency)."""
    b = (np.asarray(occ_grid) > 50) | (np.asarray(keepout_mask, bool))
    for _ in range(int(radius_cells)):
        up = np.zeros_like(b)
        up[1:, :] |= b[:-1, :]
        up[:-1, :] |= b[1:, :]
        up[:, 1:] |= b[:, :-1]
        up[:, :-1] |= b[:, 1:]
        b = b | up
    return b


def build_blocked(occ_full, ox, oy, res, keepouts,
                  radius=COLL_RADIUS_M, window=None):
    """occ_full: (H, W) occupancy values. window: (xmin, ymin, xmax, ymax)
    crop (BEV window); returns (blocked, (ox_w, oy_w)) in window frame."""
    H, W = occ_full.shape
    xs = ox + (np.arange(W) + 0.5) * res
    ys = oy + (np.arange(H) + 0.5) * res
    km = np.zeros((H, W), bool)
    if keepouts:
        from rulepatch.rp.geom import point_in_polygon
        for poly in keepouts:
            p = np.asarray(poly, dtype=np.float64)
            x0, x1 = float(p[:, 0].min()), float(p[:, 0].max())
            y0, y1 = float(p[:, 1].min()), float(p[:, 1].max())
            ix = np.where((xs >= x0 - radius) & (xs <= x1 + radius))[0]
            iy = np.where((ys >= y0 - radius) & (ys <= y1 + radius))[0]
            for j in iy:
                for i in ix:
                    if point_in_polygon((xs[i], ys[j]), p):
                        km[j, i] = True
    r = int(math.ceil(radius / res))
    blocked = _inflate_blocked_numpy(occ_full, km, r)
    if window is None:
        return blocked, (ox, oy)
    xmin, ymin, xmax, ymax = window
    ix0 = max(0, int((xmin - ox) / res))
    iy0 = max(0, int((ymin - oy) / res))
    ix1 = min(W, int(math.ceil((xmax - ox) / res)))
    iy1 = min(H, int(math.ceil((ymax - oy) / res)))
    return blocked[iy0:iy1, ix0:ix1], (ox + ix0 * res, oy + iy0 * res)


def _los(blocked, ox, oy, res, a, b):
    """Line-of-sight on the blocked grid (supercover sampling)."""
    ax, ay = (a[0] - ox) / res, (a[1] - oy) / res
    bx, by = (b[0] - ox) / res, (b[1] - oy) / res
    H, W = blocked.shape
    n = int(math.ceil(max(abs(bx - ax), abs(by - ay)) * 2)) + 1
    for t in np.linspace(0.0, 1.0, n):
        ix, iy = int(ax + (bx - ax) * t), int(ay + (by - ay) * t)
        if ix < 0 or iy < 0 or ix >= W or iy >= H:
            return False
        if blocked[iy, ix]:
            return False
    return True


def theta_star(blocked, ox, oy, res, A, B, window=None):
    """Theta* from A to B in world frame. Returns list of (x, y) world
    points including A and B, or None if no path. 8-connected grid with
    line-of-sight parent shortcuts (any-angle)."""
    H, W = blocked.shape
    def to_cell(p):
        return (int((p[0] - ox) / res), int((p[1] - oy) / res))

    def to_world(c):
        return (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res)

    s = to_cell(A)
    g = to_cell(B)
    if not (0 <= s[0] < W and 0 <= s[1] < H and 0 <= g[0] < W
            and 0 <= g[1] < H):
        return None
    if blocked[s[1], s[0]] or blocked[g[1], g[0]]:
        return None
    Aw, Bw = to_world(s), to_world(g)

    def h(c):
        w = to_world(c)
        return math.hypot(w[0] - Bw[0], w[1] - Bw[1])

    openh = [(h(s), 0, s)]
    came = {s: None}
    gs = {s: 0.0}
    closed = set()
    nbrs = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
            (0, 1), (1, -1), (1, 0), (1, 1)]
    while openh:
        _, _, cur = heapq.heappop(openh)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == g:
            cells = [cur]
            while came[cells[-1]] is not None:
                cells.append(came[cells[-1]])
            cells.reverse()
            pts = [Aw] + [to_world(c) for c in cells[1:-1]] + [Bw]
            return pts
        cw = to_world(cur)
        for dx, dy in nbrs:
            nb = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nb[0] < W and 0 <= nb[1] < H):
                continue
            if blocked[nb[1], nb[0]] or nb in closed:
                continue
            parent = came[cur]
            pw = to_world(parent) if parent is not None else cw
            nw = to_world(nb)
            if parent is not None and _los(blocked, ox, oy, res, pw, nw):
                # Shortcut: cost via parent directly (exact re-walk).
                g2 = _path_cost(gs, came, to_world, parent) + math.hypot(
                    pw[0] - nw[0], pw[1] - nw[1])
                cand_parent = parent
            else:
                step = math.hypot(dx * res, dy * res)
                g2 = gs[cur] + step
                cand_parent = cur
            if nb not in gs or g2 < gs[nb] - 1e-9:
                gs[nb] = g2
                came[nb] = cand_parent
                heapq.heappush(openh, (g2 + h(nb), len(gs), nb))
    return None


def _path_cost(gs, came, to_world, node):
    # Reconstruct exact g cost start->node by walking parents.
    cost, cur = 0.0, node
    cells = []
    while cur is not None:
        cells.append(cur)
        cur = came.get(cur)
    cells.reverse()
    for a, b in zip(cells[:-1], cells[1:]):
        wa, wb = to_world(a), to_world(b)
        cost += math.hypot(wa[0] - wb[0], wa[1] - wb[1])
    return cost


def densify_world(pts, step=0.2):
    out = [tuple(pts[0])]
    for a, b in zip(pts[:-1], pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(d / step)))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t))
    return out


def repair_local_thetastar(occ_full, ox, oy, res, A, B, keepouts,
                           center, orig_seg_world, map_bounds):
    """Full pipeline: windowed Theta* + densify + same-gate verify.
    Returns dict(ok, reason, dense, latency_ms, ...)."""
    from rulepatch.rp import BEV_SIZE_M
    from topopatch.topo.verifier_topo import verify_dense_multi
    t0 = time.time()
    half = BEV_SIZE_M / 2.0
    window = (center[0] - half, center[1] - half,
              center[0] + half, center[1] + half)
    blocked, (oxw, oyw) = build_blocked(
        occ_full, ox, oy, res, keepouts, radius=COLL_RADIUS_M,
        window=window)
    pts = theta_star(blocked, oxw, oyw, res, tuple(A), tuple(B))
    t_plan = (time.time() - t0) * 1000.0
    if not pts:
        return {"ok": False, "reason": "classical:no-path",
                "latency_ms": t_plan}
    dense = densify_world(pts)

    def occ_fn(x, y):
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        H, W = occ_full.shape
        if 0 <= ix < W and 0 <= iy < H:
            return bool(occ_full[iy, ix] > 50)
        return True

    ok, reason = verify_dense_multi(
        np.asarray(dense, dtype=np.float64), tuple(A), tuple(B),
        np.asarray(orig_seg_world, dtype=np.float64), keepouts,
        occ_fn, map_bounds, tuple(center))
    return {"ok": bool(ok), "reason": reason, "dense": dense,
            "latency_ms": t_plan, "n_pts": len(dense)}
