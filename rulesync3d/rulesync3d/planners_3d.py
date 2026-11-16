# Classical 3D candidate generators (UNTRUSTED proposers only).
#
# A. Voxel A*: 26-connected 3D search on the (inflated) occupancy grid.
# B. RRT-Connect: bidirectional sampling-based planner with exact
#    segment collision/volume checking (no grid optimism).
# Both take the same information budget (occ, volumes, bounds, margins)
# and return world-frame waypoint lists or None. No learning.
import heapq
import math
import time

import numpy as np


def voxelize_bounds(boxes, res, bounds, margin=0.0):
    """Rasterize AABB boxes [(xmin,ymin,zmin,xmax,ymax,zmax)] to a bool
    grid over bounds at res, inflated by margin. Returns (grid, origin).
    origin = (xmin, ymin, zmin) of bounds."""
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    nx = int(math.ceil((xmax - xmin) / res))
    ny = int(math.ceil((ymax - ymin) / res))
    nz = int(math.ceil((zmax - zmin) / res))
    grid = np.zeros((nz, ny, nx), bool)
    for b in boxes:
        x0, y0, z0, x1, y1, z1 = b
        ix0 = max(0, int((x0 - margin - xmin) / res))
        iy0 = max(0, int((y0 - margin - ymin) / res))
        iz0 = max(0, int((z0 - margin - zmin) / res))
        ix1 = min(nx - 1, int(math.ceil((x1 + margin - xmin) / res)))
        iy1 = min(ny - 1, int(math.ceil((y1 + margin - ymin) / res)))
        iz1 = min(nz - 1, int(math.ceil((z1 + margin - zmin) / res)))
        grid[iz0:iz1 + 1, iy0:iy1 + 1, ix0:ix1 + 1] = True
    return grid, (xmin, ymin, zmin)


def keepouts_from_geoms(geoms, margin=0.0):
    """Case volume-geom dicts ({type, center, dims, vertices}) -> AABB
    keepout boxes [(xmin,ymin,zmin,xmax,ymax,zmax)] inflated by margin.
    CONVEX via vertex bbox (conservative); exact convex enforcement
    lives in gate_3d."""
    boxes = []
    for g in geoms or []:
        t = int(g["type"])
        if t == 0:
            cx, cy, cz = (float(v) for v in g["center"])
            dx, dy, dz = (float(v) for v in g["dims"])
            boxes.append((cx - dx / 2 - margin, cy - dy / 2 - margin,
                          cz - dz / 2 - margin, cx + dx / 2 + margin,
                          cy + dy / 2 + margin, cz + dz / 2 + margin))
        elif t == 1:
            cx, cy, cz = (float(v) for v in g["center"])
            rr = float(g["dims"][0]) + margin
            boxes.append((cx - rr, cy - rr, cz - rr,
                          cx + rr, cy + rr, cz + rr))
        else:
            vs = np.asarray(g["vertices"], dtype=float)
            lo, hi = vs.min(axis=0) - margin, vs.max(axis=0) + margin
            boxes.append((float(lo[0]), float(lo[1]), float(lo[2]),
                          float(hi[0]), float(hi[1]), float(hi[2])))
    return boxes


def shortcut_smooth(path, occ_fn, vols=(), step=0.05):
    """Greedy line-of-sight shortcutting (executor tuning, NOT science).

    Dense A* cell chains / jittery RRT chains stall the velocity tracker
    (fixed total-time budget vs waypoint count). Every shortcut segment
    is verified free (occupancy stepping + exact volume segment test),
    so the smoothed path satisfies the same validity contract the gate
    checks; the driver re-gates the smoothed path before execution.
    Deterministic given the input path.
    """
    from rulesync3d.rulesync3d.gate_3d import path_intersects_volumes
    import numpy as _np
    p = [tuple(float(v) for v in q) for q in path]
    if len(p) <= 2:
        return p

    def seg_free(a, b):
        d = math.dist(a, b)
        n = max(1, int(math.ceil(d / step)))
        for t in _np.linspace(0.0, 1.0, n + 1):
            r = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
                 a[2] + (b[2] - a[2]) * t)
            if occ_fn(*r):
                return False
        if vols and path_intersects_volumes([a, b], vols):
            return False
        return True

    out = [p[0]]
    i = 0
    while i < len(p) - 1:
        j = len(p) - 1
        while j > i + 1 and not seg_free(p[i], p[j]):
            j -= 1
        out.append(p[j])
        i = j
    return out


NB26 = [(dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == dy == dz == 0)]


def voxel_astar(grid, origin, res, start, goal, max_expand=400000):
    """26-connected A* (octile-3D costs). Returns world waypoints or None."""
    nz, ny, nx = grid.shape
    ox, oy, oz = origin

    def cell(p):
        return (int((p[0] - ox) / res), int((p[1] - oy) / res),
                int((p[2] - oz) / res))

    def world(c):
        return (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res,
                oz + (c[2] + 0.5) * res)

    s, g = cell(start), cell(goal)
    for c in (s, g):
        if not (0 <= c[0] < nx and 0 <= c[1] < ny and 0 <= c[2] < nz):
            return None
    if grid[s[2], s[1], s[0]] or grid[g[2], g[1], g[0]]:
        return None
    gw = world(g)

    def h(c):
        w = world(c)
        return math.sqrt((w[0] - gw[0]) ** 2 + (w[1] - gw[1]) ** 2
                         + (w[2] - gw[2]) ** 2)

    openh = [(h(s), 0, s)]
    came = {s: None}
    gs = {s: 0.0}
    closed = set()
    n_exp = 0
    while openh and n_exp < max_expand:
        _, _, cur = heapq.heappop(openh)
        if cur in closed:
            continue
        closed.add(cur)
        n_exp += 1
        if cur == g:
            cells = [cur]
            while came[cells[-1]] is not None:
                cells.append(came[cells[-1]])
            cells.reverse()
            pts = [tuple(start)] + [world(c) for c in cells[1:-1]] \
                + [tuple(goal)]
            return pts
        for dx, dy, dz in NB26:
            nb = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
            if not (0 <= nb[0] < nx and 0 <= nb[1] < ny
                    and 0 <= nb[2] < nz):
                continue
            if grid[nb[2], nb[1], nb[0]] or nb in closed:
                continue
            step = math.sqrt((dx * res) ** 2 + (dy * res) ** 2
                             + (dz * res) ** 2)
            g2 = gs[cur] + step
            if nb not in gs or g2 < gs[nb] - 1e-9:
                gs[nb] = g2
                came[nb] = cur
                heapq.heappush(openh, (g2 + h(nb), len(gs), nb))
    return None


def _seg_free(p, q, occ_fn, vols, step=0.05):
    from rulesync3d.rulesync3d.gate_3d import path_intersects_volumes
    d = math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))
    n = max(1, int(math.ceil(d / step)))
    for t in np.linspace(0.0, 1.0, n + 1):
        r = tuple(a + (b - a) * t for a, b in zip(p, q))
        if occ_fn(*r):
            return False
    if vols and path_intersects_volumes([p, q], vols):
        return False
    return True


def rrt_connect(start, goal, occ_fn, bounds, vols=(),
                max_iter=4000, step=0.4, seed=0, goal_bias=0.08,
                t_limit_s=10.0):
    """Bidirectional RRT-Connect in xyz. Exact segment checks.
    Returns world waypoints or None. Deterministic given seed."""
    rng = np.random.default_rng(seed)
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    t0 = time.time()
    if not _seg_free(start, start, occ_fn, ()) or \
            not _seg_free(goal, goal, occ_fn, ()):
        # endpoint check ignores vols here (volumes handled per edge);
        # occupied endpoints are fatal
        return None
    Ts, Ps = {tuple(start): None}, [tuple(start)]
    Tg, Pg = {tuple(goal): None}, [tuple(goal)]

    def nearest(P, q):
        if len(P) > 64:
            A = np.asarray(P)
            d2 = ((A - q) ** 2).sum(axis=1)
            return P[int(np.argmin(d2))]
        return min(P, key=lambda p: (p[0] - q[0]) ** 2
                   + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2)

    def steer(fr, to):
        d = math.sqrt(sum((a - b) ** 2 for a, b in zip(fr, to)))
        if d < 1e-9:
            return fr
        t = min(1.0, step / d)
        return tuple(a + (b - a) * t for a, b in zip(fr, to))

    def chain(tree, node):
        out = [node]
        seen = {node}
        while tree[out[-1]] is not None:
            nxt = tree[out[-1]]
            if nxt in seen:  # safety: never loop on corrupt parents
                break
            seen.add(nxt)
            out.append(nxt)
        return out

    extend_s = True
    for _ in range(max_iter):
        if time.time() - t0 > t_limit_s:
            break
        q = (tuple(goal) if rng.random() < goal_bias else
             (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax),
              rng.uniform(zmin, zmax)))
        Tgrow, Pgrow = (Ts, Ps) if extend_s else (Tg, Pg)
        Tconn, Pconn = (Tg, Pg) if extend_s else (Ts, Ps)
        ng = nearest(Pgrow, q)
        nn = steer(ng, q)
        if _seg_free(ng, nn, occ_fn, vols):
            Tgrow[nn] = ng
            Pgrow.append(nn)
            # connect the other tree toward nn
            cur = nearest(Pconn, nn)
            while True:
                if math.dist(cur, nn) < 1e-9:
                    ch_s = chain(Ts, nn)
                    ch_s = ch_s[::-1] if ch_s[-1] == tuple(start) \
                        else ch_s
                    ch_g = chain(Tg, nn)
                    ch_g = ch_g[1:] if ch_g and ch_g[0] == nn else ch_g
                    return ch_s + ch_g
                nxt = steer(cur, nn)
                if nxt == cur or not _seg_free(cur, nxt, occ_fn, vols):
                    break
                Tconn[nxt] = cur
                Pconn.append(nxt)
                cur = nxt
        extend_s = not extend_s
    return None
