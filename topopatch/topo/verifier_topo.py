"""Multi-rule deterministic verifier for TopoRulePatch (numpy only).

Same check sequence and thresholds as frozen M2
(`rulepatch.rp.verifier`, imported read-only), except the KEEP_OUT check
loops over ALL rule polygons instead of one. M2's file is never modified.

Returns (ok, reason) with M2-compatible reason strings.
"""
import numpy as np

from rulepatch.rp import verifier as V
from rulepatch.rp.geom import (dist_point_to_polyline, max_turn_deg,
                               path_length, point_in_polygon)


def verify_dense_multi(dense, A, B, orig_seg_world, rule_polys, occ_fn,
                       map_bounds, center=None):
    """Check a dense world-frame patch against all rules."""
    from rulepatch.rp import BEV_SIZE_M
    dense = np.asarray(dense, dtype=np.float64)
    half = BEV_SIZE_M / 2.0
    xmin, ymin, xmax, ymax = map_bounds
    if len(dense) < 4:
        return False, "decode too short"
    if float(np.hypot(dense[0][0] - A[0], dense[0][1] - A[1])) > V.ANCHOR_TOL:
        return False, "bad anchor A"
    if float(np.hypot(dense[-1][0] - B[0], dense[-1][1] - B[1])) > V.ANCHOR_TOL:
        return False, "bad anchor B"
    for x, y in dense:
        if center is not None and not (
                center[0] - half <= x <= center[0] + half
                and center[1] - half <= y <= center[1] + half):
            return False, "out of window bounds"
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            return False, "out of map bounds"
    # Continuous segment-level KEEP_OUT check (P0 gate audit): exact
    # segment-polygon intersection, not point sampling. Strictly
    # stronger than the old per-sample check (any sampled inside point
    # is also a segment intersection), so this can only turn
    # accept->reject, never reject->accept.
    from topopatch.topo.gate import path_intersects_polygons
    if path_intersects_polygons(dense, rule_polys):
        return False, "KEEP_OUT violation"
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
    seg = np.hypot(np.diff(dense[:, 0]), np.diff(dense[:, 1]))
    if len(seg) and (float(np.min(seg)) < V.MIN_SPACING
                     or float(np.max(seg)) > V.MAX_SPACING):
        return False, "bad spacing"
    if max_turn_deg(dense) > V.MAX_TURN_DEG:
        return False, "excessive curvature"
    if path_length(dense) < V.MIN_PROGRESS:
        return False, "no progress"
    o = np.asarray(orig_seg_world, dtype=np.float64)
    novelty = max(dist_point_to_polyline(q, o) for q in dense)
    if novelty < V.MIN_NOVELTY:
        return False, "no-progress patch"
    return True, "ok"


def verify_patch_multi(ctrl_norm, center, A, B, orig_seg_world, rule_polys,
                       occ_fn, map_bounds):
    """Decode (anchor-pinned, M2 semantics) then multi-rule verify."""
    try:
        dense = V.decode_ctrl(ctrl_norm, center, A=A, B=B)
    except Exception as e:
        return False, f"decode-fail: {e}"
    return verify_dense_multi(dense, A, B, orig_seg_world, rule_polys,
                              occ_fn, map_bounds, center)


def select_topo(ctrls_norm, confs, center, A, B, orig_seg_world,
                rule_polys, occ_fn, map_bounds, tau=float("-inf"),
                scope=None):
    """Confidence-ordered pass with abstention threshold tau.

    Candidate = highest-confidence hypothesis; if its conf < tau, abstain
    immediately (fallback). Else verify in conf order; first pass wins.
    Returns (idx, dense, reason); idx=None -> fallback.

    M4 scope (optional dict with rep_prob, span_pred): a NONLOCAL
    prediction (rep_prob < 0.5 or span_pred > SPAN_WINDOW) abstains BEFORE
    any repair is attempted — safe fallback is correct behavior there,
    not failure.
    """
    if scope is not None:
        from topopatch.topo.repair_scope import SPAN_WINDOW
        rp = float(scope.get("rep_prob", 1.0))
        sp = float(scope.get("span_pred", 0.0))
        if rp < 0.5:
            return None, None, f"abstain-nonlocal: rep {rp:.3f} < 0.5"
        if sp > SPAN_WINDOW:
            return None, None, f"abstain-nonlocal: span {sp:.2f}m"
    order = list(np.argsort(-np.asarray(confs, dtype=np.float64)))
    if not order:
        return None, None, "all-fail: no hypotheses"
    if float(confs[order[0]]) < float(tau):
        return None, None, f"abstain: conf {float(confs[order[0]]):.3f} < tau"
    last = "no hypotheses"
    for i in order:
        ok, reason = verify_patch_multi(
            ctrls_norm[i], center, A, B, orig_seg_world, rule_polys,
            occ_fn, map_bounds)
        if ok:
            return int(i), V.decode_ctrl(ctrls_norm[i], center, A=A, B=B), "ok"
        last = reason
    return None, None, f"all-fail: {last}"
