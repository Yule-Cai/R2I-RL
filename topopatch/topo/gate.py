"""Universal output gate (M4 Task A): NOTHING executes without verification.

Invariant: ANY path — learned patch, classical deform, teacher replan,
Nav2 fallback replan — must pass the deterministic RuleSpec check before
execution (offline rollout or live pp_driver). A path that crosses any
active hard KEEP_OUT (or collides, or leaves bounds) is NEVER executed:
bounded replan retries, then safe stop / transaction abort.

Two entry points:
  gate_patch(...) : dense patch with anchors (wraps verify_dense_multi).
  gate_full_path(...) : full-length path (fallback replans, teacher
    full paths): no anchors apply; checks rule compliance (0 points of
    the path inside any hard polygon), collision-free (0.1 m radius
    sampling, same as the verifier), inside map bounds, and minimum
    length. Returns (execute: bool, reason: str).

Reasons are stable strings for records and status messages.
"""
try:  # Installed RuleSync runtime package.
    import rulesync3d.gate2d as _gate2d
except ImportError:  # Repository-root source execution.
    import rulesync3d.rulesync3d.gate2d as _gate2d

COLL_RADIUS = _gate2d.COLL_RADIUS
COLL_STEPS = _gate2d.COLL_STEPS
gate_full_path = _gate2d.gate_full_path
path_intersects_polygons = _gate2d.path_intersects_polygons
segment_intersects_polygon = _gate2d.segment_intersects_polygon
segments_intersect = _gate2d.segments_intersect


def gate_patch(dense, A, B, orig_seg, rule_polys, occ_fn, map_bounds,
               center=None):
    """Gate a learned/classical patch (anchors apply)."""
    from topopatch.topo.verifier_topo import verify_dense_multi
    ok, reason = verify_dense_multi(
        dense, tuple(A), tuple(B), orig_seg, rule_polys, occ_fn,
        map_bounds, center)
    if ok:
        return True, "gate:ok"
    return False, f"gate:{reason}"


# Bounded fallback policy (frozen for M4): at most this many rule-aware
# replans (spaced for mask propagation) before safe stop. Retries with an
# identical deterministic planner are only useful across costmap/mask
# propagation delays — hence the small bound, then STOP (no motion).
MAX_FALLBACK_REPLANS = 2
FALLBACK_RETRY_GAP_S = 3.0
