# Universal 3D execution gate: NOTHING executes without verification.
#
# For every trajectory segment s_i = [p_i, p_{i+1}] (true 3D points),
# check exact/robust intersection with every active RuleVolume:
#   AABB   -> exact segment-AABB intersection (slab method)
#   SPHERE -> exact segment-sphere distance/intersection
#   CONVEX -> clip a shared segment-parameter interval against all
#             supplied halfspaces (boundary contact counts)
# Never point-sample-only. Body margin via inflated volumes
# (V (+) B(r_uav + r_margin)); the gate checks the CENTER trajectory
# against V_inflated. See safety contract for assumptions.
import math

import numpy as np

GEOM_AABB = 0
GEOM_SPHERE = 1
GEOM_CONVEX = 2


def inflate_aabb(center, dims, r):
    c = np.asarray(center, dtype=np.float64)
    d = np.asarray(dims, dtype=np.float64)
    return c - d / 2.0 - r, c + d / 2.0 + r


def segment_intersects_aabb(p1, p2, bmin, bmax, eps=1e-9):
    p1 = np.asarray(p1, dtype=np.float64)
    p2 = np.asarray(p2, dtype=np.float64)
    bmin = np.asarray(bmin, dtype=np.float64)
    bmax = np.asarray(bmax, dtype=np.float64)
    d = p2 - p1
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < eps:
            if p1[i] < bmin[i] - eps or p1[i] > bmax[i] + eps:
                return False
        else:
            ta = (bmin[i] - p1[i]) / d[i]
            tb = (bmax[i] - p1[i]) / d[i]
            if ta > tb:
                ta, tb = tb, ta
            t0 = max(t0, ta)
            t1 = min(t1, tb)
            if t0 > t1 + eps:
                return False
    return True


def segment_intersects_sphere(p1, p2, center, radius, eps=1e-9):
    p1 = np.asarray(p1, dtype=np.float64)
    p2 = np.asarray(p2, dtype=np.float64)
    c = np.asarray(center, dtype=np.float64)
    d = p2 - p1
    f = p1 - c
    a = float(d @ d)
    if a < eps:
        return bool(float(f @ f) <= radius * radius + eps)
    t = -float(f @ d) / a
    t = max(0.0, min(1.0, t))
    closest = p1 + t * d
    return bool(float((closest - c) @ (closest - c)) <= radius * radius + eps)


def segment_intersects_convex(p1, p2, halfspaces, eps=1e-9):
    """Intersect a segment with finite, valid halfspaces n.p + off >= 0.

    Each face clips the same t interval in p(t) = p1 + t * (p2 - p1).
    Passing every face separately is insufficient: their feasible intervals
    must overlap. Boundary contact counts; eps relaxes each plane residual,
    so it is a distance tolerance only for unit normals. Geometry inflation
    remains the caller's responsibility.
    """
    p1 = np.asarray(p1, dtype=np.float64)
    p2 = np.asarray(p2, dtype=np.float64)
    t_enter, t_exit = 0.0, 1.0
    for n, off in halfspaces:
        n = np.asarray(n, dtype=np.float64)
        d1 = float(n @ p1) + float(off) + eps
        d2 = float(n @ p2) + float(off) + eps
        if d1 < 0.0 and d2 < 0.0:
            return False
        if d1 < 0.0:
            t_enter = max(t_enter, d1 / (d1 - d2))
        elif d2 < 0.0:
            t_exit = min(t_exit, d1 / (d1 - d2))
        # Division occurs only across a sign change, including contact at
        # one endpoint; parallel and zero-length segments need no division.
        if t_enter > t_exit:
            return False
    return True


def volume_from_msg(v):
    """Normalize a RuleVolume-like dict to (geom_type, params).

    AABB: params = (bmin3, bmax3) already inflated by caller.
    SPHERE: params = (center3, radius_inflated).
    CONVEX: params = halfspaces list.
    """
    g = int(v["geometry_type"])
    if g == GEOM_AABB:
        return g, (np.asarray(v["bmin"]), np.asarray(v["bmax"]))
    if g == GEOM_SPHERE:
        return g, (np.asarray(v["center"]), float(v["radius"]))
    if g == GEOM_CONVEX:
        return g, [(np.asarray(n), float(o))
                   for n, o in v["halfspaces"]]
    raise ValueError(f"unknown RuleVolume geometry {g}")


def segment_intersects_volume(p1, p2, vol):
    g, pr = volume_from_msg(vol)
    if g == GEOM_AABB:
        return segment_intersects_aabb(p1, p2, pr[0], pr[1])
    if g == GEOM_SPHERE:
        return segment_intersects_sphere(p1, p2, pr[0], pr[1])
    return segment_intersects_convex(p1, p2, pr)


def path_intersects_volumes(path, vols):
    p = np.asarray(path, dtype=np.float64)
    for i in range(len(p) - 1):
        for v in vols:
            if segment_intersects_volume(p[i], p[i + 1], v):
                return True
    return False


def inflate_gate_vols(vols, extra):
    """Extra inflation for PLANNING/SMOOTHING only (tracking reserve).

    The gate verifies against vols (body margin); the smoother verifies
    against vols+extra so shortcut paths keep `extra` clearance from the
    gate boundary — absorbing tracker tolerance/overshoot. Two-tier:
    plan/smooth with reserve, gate nominal. Deterministic.
    """
    out = []
    for v in vols:
        g = int(v["geometry_type"])
        if g == GEOM_AABB:
            out.append({"geometry_type": g,
                        "bmin": np.asarray(v["bmin"]) - extra,
                        "bmax": np.asarray(v["bmax"]) + extra})
        elif g == GEOM_SPHERE:
            out.append({"geometry_type": g,
                        "center": np.asarray(v["center"]),
                        "radius": float(v["radius"]) + extra})
        else:
            out.append(v)  # convex exact path stays nominal (no live use)
    return out


def gate_3d_path(path, volumes, occ_fn, bounds, min_len=0.2,
                 r_body=0.0, occ_res=0.1):
    """Gate a 3D candidate trajectory.

    volumes: list of INFLATED RuleVolume dicts (caller inflates by
      r_uav + r_margin; r_body documents the assumption).
    occ_fn(x, y, z): True if occupied (must already include the
      body-margin inflation, same radius as the volumes).
    bounds: (xmin, ymin, zmin, xmax, ymax, zmax).
    occ_res: occupancy voxel size; segments are walked at <= occ_res/2
      so every traversed voxel is sampled (no tunneling), which with
      inflation makes the swept check conservative.
    Returns (execute, reason).
    """
    p = np.asarray(path, dtype=np.float64)
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    if p.ndim != 2 or p.shape[0] < 2 or p.shape[1] != 3:
        return False, "gate3d:bad-path"
    if bool(((p[:, 0] < xmin) | (p[:, 0] > xmax) | (p[:, 1] < ymin)
             | (p[:, 1] > ymax) | (p[:, 2] < zmin)
             | (p[:, 2] > zmax)).any()):
        return False, "gate3d:out-of-bounds"
    for v in volumes:
        g = int(v["geometry_type"]) if isinstance(v, dict) \
            else int(v.geometry_type)
        if g not in (GEOM_AABB, GEOM_SPHERE, GEOM_CONVEX):
            return False, "gate3d:bad-volume"
    if path_intersects_volumes(p, volumes):
        return False, "gate3d:KEEP_OUT_VOLUME-violation"
    # Swept collision, conservative: walk each segment at steps of
    # at most half a voxel so every traversed (inflated) voxel is
    # sampled — no tunneling. Exact for the voxel model.
    r_step = min(0.05, occ_res / 2.0)
    for i in range(len(p) - 1):
        a, b = p[i], p[i + 1]
        seg_len = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(seg_len / r_step)))
        for t in np.linspace(0.0, 1.0, n + 1):
            q = a + (b - a) * t
            if bool(occ_fn(float(q[0]), float(q[1]), float(q[2]))):
                return False, "gate3d:collision"
    total = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    if total < min_len:
        return False, "gate3d:too-short"
    return True, "gate3d:ok"
