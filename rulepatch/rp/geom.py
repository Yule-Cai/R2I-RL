"""Deterministic 2D geometry helpers (numpy only, no ROS)."""
import math

import numpy as np


def point_in_polygon(xy, pts) -> bool:
    x, y = float(xy[0]), float(xy[1])
    inside, n = False, len(pts)
    for i in range(n):
        x0, y0 = float(pts[i][0]), float(pts[i][1])
        x1, y1 = float(pts[(i + 1) % n][0]), float(pts[(i + 1) % n][1])
        if ((y0 > y) != (y1 > y)) and (
            x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0
        ):
            inside = not inside
    return inside


def polygon_area(pts) -> float:
    a = 0.0
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return abs(a) * 0.5


def path_length(pts) -> float:
    p = np.asarray(pts, dtype=np.float64)
    if len(p) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))))


def resample_polyline(pts, n: int) -> np.ndarray:
    """Arc-length uniform resample to exactly n points (Nx2 float64)."""
    p = np.asarray(pts, dtype=np.float64)
    assert p.ndim == 2 and p.shape[1] == 2 and len(p) >= 2 and n >= 2
    seg = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    if total < 1e-9:
        return np.repeat(p[:1], n, axis=0).copy()
    targets = np.linspace(0.0, total, n)
    out = np.zeros((n, 2))
    j = 0
    for i, t in enumerate(targets):
        while j < len(seg) - 1 and cum[j + 1] < t:
            j += 1
        denom = max(cum[j + 1] - cum[j], 1e-12)
        f = (t - cum[j]) / denom
        out[i] = p[j] + f * (p[j + 1] - p[j])
    return out


def dist_point_to_polyline(xy, poly) -> float:
    """Min distance from point to polyline segments."""
    p = np.asarray(poly, dtype=np.float64)
    q = np.asarray(xy, dtype=np.float64)
    if len(p) == 1:
        return float(np.hypot(*(q - p[0])))
    d = p[1:] - p[:-1]
    denom = np.sum(d * d, axis=1) + 1e-12
    t = np.clip(np.sum((q - p[:-1]) * d, axis=1) / denom, 0.0, 1.0)
    proj = p[:-1] + t[:, None] * d
    return float(np.min(np.hypot(proj[:, 0] - q[0], proj[:, 1] - q[1])))


def nearest_index_on_polyline(xy, poly) -> int:
    p = np.asarray(poly, dtype=np.float64)
    q = np.asarray(xy, dtype=np.float64)
    return int(np.argmin(np.hypot(p[:, 0] - q[0], p[:, 1] - q[1])))


def count_inside(path, poly) -> int:
    return sum(1 for xy in path if point_in_polygon(xy, poly))


def find_anchors(orig, repl, div_thresh=0.35, rejoin_tol=0.175,
                 min_straight=1.0):
    """Divergence (A) / reconvergence (B) anchors of repl vs orig.

    Returns dict with iA, iB (orig indices), jA, jB (repl indices),
    A, B coords, or (None, reason) when no clean local patch exists.
    """
    from . import DIV_THRESH as DT, REJOIN_TOL as RT, MIN_PATCH_STRAIGHT as MPS
    div_thresh = float(div_thresh or DT)
    rejoin_tol = float(rejoin_tol or RT)
    min_straight = float(min_straight or MPS)
    o = np.asarray(orig, dtype=np.float64)
    r = np.asarray(repl, dtype=np.float64)
    if len(o) < 4 or len(r) < 4:
        return None, "path too short"
    d = np.array([dist_point_to_polyline(q, r) for q in o])
    above = np.where(d > div_thresh)[0]
    if len(above) == 0:
        return None, "no divergence (paths coincide)"
    # A = last coinciding orig point before the divergent span;
    # B = first coinciding orig point after it. (Stepping back/forward
    # guarantees the repl actually rejoins at the anchors by construction;
    # the tol checks below remain as safety.)
    pre = np.where(d[:int(above[0])] < rejoin_tol)[0]
    if len(pre) == 0:
        return None, "no rejoin at A"
    post = np.where(d[int(above[-1]):] < rejoin_tol)[0]
    if len(post) == 0:
        return None, "no rejoin at B"
    iA, iB = int(pre[-1]), int(above[-1] + post[0])
    if iB - iA < 2:
        return None, "divergent span too short"
    A, B = o[iA], o[iB]
    if float(np.hypot(*(B - A))) < min_straight:
        return None, "patch straight-line below minimum"
    # Rejoin safety: segment-distance (consistent with d above; vertex
    # distance would be overly strict on sloped ramps).
    jA = nearest_index_on_polyline(A, r)
    jB = nearest_index_on_polyline(B, r)
    if jB <= jA:
        return None, "repl indices not ordered"
    if dist_point_to_polyline(A, r) > rejoin_tol:
        return None, "no rejoin at A"
    if dist_point_to_polyline(B, r) > rejoin_tol:
        return None, "no rejoin at B"
    return {
        "iA": iA, "iB": iB, "jA": jA, "jB": jB,
        "A": (float(A[0]), float(A[1])), "B": (float(B[0]), float(B[1])),
    }, "ok"


def catmull_rom(ctrl, samples_per_seg=12) -> np.ndarray:
    """Centripetal-free Catmull-Rom through control points (uniform)."""
    c = np.asarray(ctrl, dtype=np.float64)
    assert len(c) >= 2
    ext = np.concatenate([c[:1], c, c[-1:]], axis=0)
    out = [c[:1]]
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        for s in range(1, samples_per_seg + 1):
            t = s / samples_per_seg
            t2, t3 = t * t, t * t * t
            pt = 0.5 * ((2 * p1) + (-p0 + p2) * t
                        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)
            out.append(pt[None, :])
    return np.concatenate(out, axis=0)


def max_turn_deg(poly) -> float:
    p = np.asarray(poly, dtype=np.float64)
    if len(p) < 3:
        return 0.0
    v1 = p[1:-1] - p[:-2]
    v2 = p[2:] - p[1:-1]
    n1 = np.hypot(v1[:, 0], v1[:, 1]) + 1e-12
    n2 = np.hypot(v2[:, 0], v2[:, 1]) + 1e-12
    cosang = np.clip(np.sum(v1 * v2, axis=1) / (n1 * n2), -1.0, 1.0)
    return float(np.max(np.degrees(np.arccos(cosang))))


def rect_corners(cx, cy, w, h, theta=0.0):
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for sx, sy in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
        dx, dy = sx * w / 2, sy * h / 2
        out.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    return out
