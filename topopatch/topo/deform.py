"""Classical local path-deformation baseline (deterministic, numpy only).

Elastic-band-lite: init = original segment resampled dense; minimize
  J = w_len * length + w_curv * rough + w_rule * keepout_repulsion
      + w_occ * sdf_repulsion
with fixed-step gradient descent (analytic gradients), endpoints pinned
to A/B, fixed iteration budget. Same anchors + rule mask as learned
methods. Deterministic: no RNG anywhere.

Known limitation (documented, not hidden): initialized from the original
segment, the optimizer cannot *choose* a detour topology — in symmetric
cases repulsion gradients cancel and it stalls. This is precisely the
gap explicit topology reasoning (TopoRulePatch) addresses. See the
milestone report discussion of prior rule-compliant trajectory-repair
work (elastic bands, CHOMP/STOMP, TrajOpt): local optimizers deform
within a homotopy class; they do not select one.
"""
import math

import numpy as np

# Frozen defaults (tuned on a handful of synthetic cases; frozen for M3).
W_LEN = 1.0
W_CURV = 2.0
W_RULE = 8.0
RULE_MARGIN = 0.4
W_OCC = 4.0
OCC_SAFE = 0.3
LR = 0.02
ITERS = 120


def _seg_closest(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy + 1e-12
    t = min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    qx, qy = ax + t * dx, ay + t * dy
    return qx, qy, math.hypot(px - qx, py - qy)


def _bilinear(grid, xs, ys):
    """Sample float grid at fractional index arrays (exact on nodes)."""
    h, w = grid.shape
    xs = np.clip(xs, 0, w - 1)
    ys = np.clip(ys, 0, h - 1)
    x0, y0 = xs.astype(int), ys.astype(int)
    x1, y1 = np.minimum(x0 + 1, w - 1), np.minimum(y0 + 1, h - 1)
    fx, fy = xs - x0, ys - y0
    return ((grid[y0, x0] * (1 - fx) + grid[y0, x1] * fx) * (1 - fy)
            + (grid[y1, x0] * (1 - fx) + grid[y1, x1] * fx) * fy)


def deform_patch(orig_seg_world, rule_polys, occ_sdf_fn, A, B,
                 w_len=W_LEN, w_curv=W_CURV, w_rule=W_RULE,
                 rule_margin=RULE_MARGIN, w_occ=W_OCC, occ_safe=OCC_SAFE,
                 lr=LR, iters=ITERS):
    """Returns dense (61,2) world-frame patch. Deterministic."""
    from rulepatch.rp.geom import point_in_polygon, resample_polyline
    p = resample_polyline(np.asarray(orig_seg_world, dtype=np.float64),
                          61).astype(np.float64)
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    polys = [np.asarray(poly, dtype=np.float64) for poly in rule_polys]
    n = len(p)
    for it in range(iters):
        g = np.zeros_like(p)
        # length term: sum |d_i|, d_i = p[i+1]-p[i]
        d = p[1:] - p[:-1]
        nl = np.hypot(d[:, 0], d[:, 1]) + 1e-9
        u = d / nl[:, None]
        g[:-1] += -u
        g[1:] += u
        g *= w_len
        # curvature term: sum |p[i+1]-2p[i]+p[i-1]|^2
        lap = p[2:] - 2 * p[1:-1] + p[:-2]
        gc = np.zeros_like(p)
        gc[:-2] += 2 * lap
        gc[1:-1] += -4 * lap
        gc[2:] += 2 * lap
        g += w_curv * gc
        # keepout repulsion: ReLU(margin - d_signed)^2 per rule, where
        # d_signed is negative inside the polygon. Unsigned edge distance
        # would attract interior points to the incenter; the sign flip
        # below restores the outward-pushing gradient.
        for poly in polys:
            m = len(poly)
            for k in range(n):
                px, py = p[k]
                best = (1e9, 0.0, 0.0)
                for i in range(m):
                    qx, qy, dd = _seg_closest(
                        px, py, poly[i][0], poly[i][1],
                        poly[(i + 1) % m][0], poly[(i + 1) % m][1])
                    if dd < best[0]:
                        best = (dd, qx, qy)
                    if best[0] == 0.0:
                        break
                dd, qx, qy = best
                inside = point_in_polygon((px, py), poly)
                ds = -dd if inside else dd
                if ds < rule_margin:
                    # d/dp of (margin - ds)^2 = -2(margin-ds) * grad(ds);
                    # grad(ds) = +(p-q)/d outside, -(p-q)/d inside.
                    s = (-2.0 * (rule_margin - ds) / max(dd, 1e-6)
                         * (-1.0 if inside else 1.0))
                    g[k, 0] += w_rule * s * (px - qx)
                    g[k, 1] += w_rule * s * (py - qy)
        # occupancy repulsion via SDF callback: ReLU(safe - sdf)^2.
        # occ_sdf_fn(xs, ys) -> (vals, (gx, gy)) with analytic grid grads.
        vals, grads = occ_sdf_fn(p[:, 0], p[:, 1])
        act = vals < occ_safe
        if np.any(act):
            s = -2.0 * (occ_safe - vals[act])  # d/dsdf of penalty
            g[act] += w_occ * (s[:, None] * grads[act])
        # pin endpoints
        g[0] = 0.0
        g[-1] = 0.0
        p = p - lr * g
        p[0] = A
        p[-1] = B
        # early exit: all points clear of all rules by margin and SDF safe
        if (it + 1) % 20 == 0:
            clear = True
            for poly in polys:
                m = len(poly)
                for k in range(n):
                    px, py = p[k]
                    if point_in_polygon((px, py), poly):
                        clear = False
                        break
                    dd = min(_seg_closest(
                        px, py, poly[i][0], poly[i][1],
                        poly[(i + 1) % m][0],
                        poly[(i + 1) % m][1])[2] for i in range(m))
                    if dd < rule_margin:
                        clear = False
                        break
                if not clear:
                    break
            if clear:
                vals, _ = occ_sdf_fn(p[:, 0], p[:, 1])
                if bool(np.all(vals >= occ_safe)):
                    break
    # Chaikin corner-cutting (documented part of the baseline pipeline:
    # classical elastic bands are always followed by a smoothing pass in
    # practice) + uniform resample so the verifier's spacing check sees
    # the same discretization as for learned methods. The verifier still
    # decides acceptance — smoothing can only move points, never bypass
    # the KEEP_OUT / collision / curvature gates.
    from rulepatch.rp.geom import resample_polyline as _resample
    for _ in range(2):
        q = [p[0]]
        for i in range(len(p) - 1):
            q.append(0.75 * p[i] + 0.25 * p[i + 1])
            q.append(0.25 * p[i] + 0.75 * p[i + 1])
        q.append(p[-1])
        p = np.asarray(q)
    p[0] = A
    p[-1] = B
    return _resample(p, 61)


def make_sdf_fn(sdf_grid, ox, oy, res):
    """Build occ_sdf_fn(xs, ys) -> (vals, grads) from an SDF grid (meters).

    sdf_grid: HxW float32, row 0 = ymin. grads via central differences
    scaled to per-meter. Queries outside grid return (0.0, zero-grad)
    (treated as free — verifier still enforces map bounds separately).
    """
    g = np.asarray(sdf_grid, dtype=np.float64)
    h, w = g.shape
    gy, gx = np.gradient(g, res, edge_order=1)  # per-meter grads

    def fn(xs, ys):
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        ix = (xs - ox) / res
        iy = (ys - oy) / res
        vals = _bilinear(g, ix, iy)
        gxq = _bilinear(gx, ix, iy)
        gyq = _bilinear(gy, ix, iy)
        inside = (ix >= 0) & (ix <= w - 1) & (iy >= 0) & (iy <= h - 1)
        vals = np.where(inside, vals, 0.0)
        grads = np.stack([np.where(inside, gxq, 0.0),
                          np.where(inside, gyq, 0.0)], axis=-1)
        return vals, grads

    return fn
