"""BEV window rendering + local-frame conversions (numpy only)."""
import numpy as np

from . import BEV_PX, BEV_SIZE_M, BEV_RES, N_LOCAL_PTS
from .geom import resample_polyline, point_in_polygon


def add_coords(bev):
    """Append normalized coordinate channels. bev: (..., C,H,W) numpy."""
    b = np.asarray(bev, dtype=np.float32)
    _, H, W = b.shape[-3:]
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    cx = (xx / (W - 1)) * 2 - 1
    cy = 1 - (yy / (H - 1)) * 2
    shape = b.shape[:-3] + (1, H, W)
    cx = np.broadcast_to(cx, shape).copy()
    cy = np.broadcast_to(cy, shape).copy()
    return np.concatenate([b, cx, cy], axis=-3)

def window_for_patch(A, B, map_bounds, margin=1.0):
    """Axis-aligned window center covering A..B plus margin; shift into map.

    map_bounds = (xmin, ymin, xmax, ymax). Returns (cx, cy).
    """
    mx = (A[0] + B[0]) / 2.0
    my = (A[1] + B[1]) / 2.0
    half = BEV_SIZE_M / 2.0
    xmin, ymin, xmax, ymax = map_bounds
    cx = min(max(mx, xmin + half), xmax - half)
    cy = min(max(my, ymin + half), ymax - half)
    return float(cx), float(cy)


def world_to_norm(pts, center):
    p = np.asarray(pts, dtype=np.float64)
    half = BEV_SIZE_M / 2.0
    return (p - np.array(center, dtype=np.float64)) / half  # [-1,1] in window


def norm_to_world(pts, center):
    p = np.asarray(pts, dtype=np.float64)
    half = BEV_SIZE_M / 2.0
    return p * half + np.array(center, dtype=np.float64)


def _rasterize_polyline(chan, pts_norm, thickness=2, value=1.0):
    h, w = chan.shape
    for x, y in pts_norm:
        px = int(round((x * 0.5 + 0.5) * (w - 1)))
        py = int(round((0.5 - y * 0.5) * (h - 1)))
        for dy in range(-thickness, thickness + 1):
            for dx in range(-thickness, thickness + 1):
                xx, yy = px + dx, py + dy
                if 0 <= xx < w and 0 <= yy < h:
                    chan[yy, xx] = value


def _rasterize_polygon(chan, poly_world, center, value=1.0):
    n = chan.shape[0]
    half = BEV_SIZE_M / 2.0
    xs = [p[0] for p in poly_world]
    ys = [p[1] for p in poly_world]
    # pixel bbox of polygon
    u0 = int(max(0, ((min(xs) - (center[0] - half)) / BEV_SIZE_M) * n))
    u1 = int(min(n - 1, ((max(xs) - (center[0] - half)) / BEV_SIZE_M) * n))
    # v axis: world y max -> row 0
    v0 = int(max(0, ((center[1] + half - max(ys)) / BEV_SIZE_M) * n))
    v1 = int(min(n - 1, ((center[1] + half - min(ys)) / BEV_SIZE_M) * n))
    for v in range(v0, v1 + 1):
        wy = center[1] + half - (v + 0.5) * BEV_RES
        for u in range(u0, u1 + 1):
            wx = center[0] - half + (u + 0.5) * BEV_RES
            if point_in_polygon((wx, wy), poly_world):
                chan[v, u] = value


def render_bev_fast(occ_full, res_in, ox, oy, center, orig_seg_world,
                    rule_poly_world, goal_xy):
    """Vectorized render_bev (identical output geometry). occ_full: HxW uint8
    grid at res_in with origin (ox, oy) (row 0 = ymin). Returns 4xPxPx f32."""
    n = BEV_PX
    half = BEV_SIZE_M / 2.0
    H, W = occ_full.shape
    uu = (np.arange(n) + 0.5) * BEV_RES + (center[0] - half)
    vv = (center[1] + half) - (np.arange(n) + 0.5) * BEV_RES
    ix = np.clip(((uu - ox) / res_in).astype(int), 0, W - 1)
    iy = np.clip(((vv - oy) / res_in).astype(int), 0, H - 1)
    occ = occ_full[iy[:, None], ix[None, :]].astype(np.float32)
    op = np.zeros((n, n), dtype=np.float32)
    _rasterize_polyline(op, world_to_norm(orig_seg_world, center))
    rule = np.zeros((n, n), dtype=np.float32)
    _rasterize_polygon(rule, rule_poly_world, center)
    gx, gy = goal_xy
    gu = (gx - (center[0] - half)) / BEV_SIZE_M * n
    gv = ((center[1] + half) - gy) / BEV_SIZE_M * n
    yy, xx = np.mgrid[0:n, 0:n]
    sig = 0.4 / BEV_RES
    goal = np.exp(-((xx - gu) ** 2 + (yy - gv) ** 2) / (2 * sig * sig)).astype(np.float32)
    return np.stack([np.clip(occ, 0, 1), op, rule, np.clip(goal, 0, 1)], axis=0)


def render_bev(occ_fn, center, orig_seg_world, rule_poly_world, goal_xy):
    """Render 4-channel BEV (float32 [0,1], PX x PX).

    occ_fn(x, y) -> 1 if occupied else 0 (world coords).
    Channels: [occupancy, original-path, rule-mask, goal-blob].
    """
    n = BEV_PX
    half = BEV_SIZE_M / 2.0
    occ = np.zeros((n, n), dtype=np.float32)
    for v in range(n):
        wy = center[1] + half - (v + 0.5) * BEV_RES
        for u in range(n):
            wx = center[0] - half + (u + 0.5) * BEV_RES
            if occ_fn(wx, wy):
                occ[v, u] = 1.0
    op = np.zeros((n, n), dtype=np.float32)
    _rasterize_polyline(op, world_to_norm(orig_seg_world, center))
    rule = np.zeros((n, n), dtype=np.float32)
    _rasterize_polygon(rule, rule_poly_world, center)
    goal = np.zeros((n, n), dtype=np.float32)
    gx, gy = goal_xy
    # gaussian blob sigma=0.4m
    gu = (gx - (center[0] - half)) / BEV_SIZE_M * n
    gv = ((center[1] + half) - gy) / BEV_SIZE_M * n
    yy, xx = np.mgrid[0:n, 0:n]
    sig = 0.4 / BEV_RES
    goal = np.exp(-((xx - gu) ** 2 + (yy - gv) ** 2) / (2 * sig * sig)).astype(np.float32)
    goal = np.clip(goal, 0.0, 1.0)
    return np.stack([occ, op, rule, goal], axis=0)


def local_path_points(orig_seg_world, center, n=N_LOCAL_PTS):
    seg = resample_polyline(orig_seg_world, n)
    return world_to_norm(seg, center).astype(np.float32)


def target_control_points(repl_seg_world, center, k=6):
    seg = resample_polyline(repl_seg_world, k)
    return world_to_norm(seg, center).astype(np.float32)
