"""Deterministic patch verifier (numpy only, no learning).

Rejects: collision, KEEP_OUT violation, anchor mismatch, out-of-bounds,
excessive curvature / bad spacing, no-progress patches.
"""
import numpy as np

from . import BEV_SIZE_M
from .geom import (point_in_polygon, path_length, max_turn_deg,
                   catmull_rom, dist_point_to_polyline)


# Thresholds (frozen for the milestone; recorded in the report).
ANCHOR_TOL = 0.30       # patch endpoints must match A/B within this (m)
MAX_TURN_DEG = 60.0     # per-vertex turn on 0.2m-resampled decode
MIN_SPACING = 1e-4      # degenerate-sample guard (dense decode is ~0.1m)
MAX_SPACING = 1.00      # max consecutive sample spacing (m)
MIN_PROGRESS = 1.0      # min patch length (m)
MIN_NOVELTY = 0.20      # max deviation from original segment (m)


def decode_ctrl(ctrl_norm, center, samples_per_seg=12, A=None, B=None):
    """Decode control points to a dense world-frame patch.

    Endpoints are pinned to the input anchors A/B when given (anchors are
    problem inputs, not predictions; the network shapes the interior).
    Without A/B the raw endpoints are used (legacy/tests).
    """
    from .bev import norm_to_world, world_to_norm
    c = np.asarray(ctrl_norm, dtype=np.float64)
    if A is not None and B is not None:
        c = c.copy()
        c[0] = world_to_norm([A], center)[0]
        c[-1] = world_to_norm([B], center)[0]
    w = norm_to_world(c, center)
    return catmull_rom(w, samples_per_seg=samples_per_seg)


def verify_patch(ctrl_norm, center, A, B, orig_seg_world, rule_poly_world,
                 occ_fn, map_bounds):
    """Return (ok: bool, reason: str). First failure reason wins."""
    try:
        dense = decode_ctrl(ctrl_norm, center, A=A, B=B)
    except Exception as e:
        return False, f"decode-fail: {e}"
    return verify_dense(dense, A, B, orig_seg_world, rule_poly_world,
                        occ_fn, map_bounds, center)


def verify_dense(dense, A, B, orig_seg_world, rule_poly_world,
                 occ_fn, map_bounds, center=None):
    """Check a dense world-frame patch. Returns (ok, reason)."""
    import numpy as _np
    dense = _np.asarray(dense, dtype=_np.float64)
    half = BEV_SIZE_M / 2.0
    xmin, ymin, xmax, ymax = map_bounds
    if len(dense) < 4:
        return False, "decode too short"
    # 1. anchors
    if float(np.hypot(dense[0][0] - A[0], dense[0][1] - A[1])) > ANCHOR_TOL:
        return False, "bad anchor A"
    if float(np.hypot(dense[-1][0] - B[0], dense[-1][1] - B[1])) > ANCHOR_TOL:
        return False, "bad anchor B"
    # 2. bounds (window if center given + map)
    for x, y in dense:
        if center is not None and not (
                center[0] - half <= x <= center[0] + half
                and center[1] - half <= y <= center[1] + half):
            return False, "out of window bounds"
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            return False, "out of map bounds"
    # 3. KEEP_OUT violation (strict)
    for x, y in dense:
        if point_in_polygon((x, y), rule_poly_world):
            return False, "KEEP_OUT violation"
    # 4. collision (occupancy + 1-cell 0.1m margin ring)
    for x, y in dense:
        hit = False
        for dx in (-0.1, 0.0, 0.1):
            for dy in (-0.1, 0.0, 0.1):
                if occ_fn(x + dx, y + dy):
                    hit = True
                    break
            if hit:
                break
        if hit:
            return False, "collision"
    # 5. spacing on resampled decode
    seg = np.hypot(np.diff(dense[:, 0]), np.diff(dense[:, 1]))
    if len(seg) and (float(np.min(seg)) < MIN_SPACING or float(np.max(seg)) > MAX_SPACING):
        return False, "bad spacing"
    # 6. curvature
    if max_turn_deg(dense) > MAX_TURN_DEG:
        return False, "excessive curvature"
    # 7. progress + novelty
    if path_length(dense) < MIN_PROGRESS:
        return False, "no progress"
    o = np.asarray(orig_seg_world, dtype=np.float64)
    novelty = max(dist_point_to_polyline(q, o) for q in dense)
    if novelty < MIN_NOVELTY:
        return False, "no-progress patch"
    return True, "ok"


def select_hypothesis(ctrls_norm, confs, center, A, B, orig_seg_world,
                      rule_poly_world, occ_fn, map_bounds):
    """Highest-confidence-first verifier pass. Returns (idx, dense, reason).

    idx=None means all proposals failed -> caller falls back to Nav2 replan.
    """
    order = list(np.argsort(-np.asarray(confs, dtype=np.float64)))
    last = "no hypotheses"
    for i in order:
        ok, reason = verify_patch(ctrls_norm[i], center, A, B, orig_seg_world,
                                  rule_poly_world, occ_fn, map_bounds)
        if ok:
            return int(i), decode_ctrl(ctrls_norm[i], center, A=A, B=B), "ok"
        last = reason
    return None, None, f"all-fail: {last}"
