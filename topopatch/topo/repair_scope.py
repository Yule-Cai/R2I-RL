"""Adaptive repair scope (M4 Task B): deterministic GT + policy constants.

Ground truth is derived ONLY from teacher geometry (no learning, no
threshold tuning on test):
  span_m   = |B - A| (anchor span, meters)
  detour   = len(repl_seg) / len(orig_seg) (teacher cost of locality)
  LOCAL      iff span_m <= SPAN_MAX and detour <= DETOUR_MAX
  NONLOCAL   otherwise (structurally needs global rerouting; a NONLOCAL
             prediction followed by safe fallback is CORRECT, not failure)
  complexity = detour bucket: [1.0, 1.3) SIMPLE, [1.3, 1.8) MODERATE,
             [1.8, inf) COMPLEX.

Frozen constants below. Inference policy: attempt local repair only if
P(LOCAL) >= 0.5 AND predicted span <= SPAN_WINDOW (fits the BEV window
with margin); else abstain-NONLOCAL -> gated fallback.
"""
import numpy as np

SPAN_MAX = 6.0       # teacher span above this is structurally nonlocal
DETOUR_MAX = 1.8     # teacher detour above this is structurally nonlocal
SPAN_WINDOW = 10.0   # predicted span above this cannot fit BEV+margin
COMPLEX_EDGES = (1.3, 1.8)

LOCAL = 1
NONLOCAL = 0


def repairability_gt(orig_seg, repl_seg, A, B):
    """Deterministic (repairable, span_m, detour, complexity) from segs."""
    from rulepatch.rp.geom import path_length
    o = np.asarray(orig_seg, dtype=np.float64)
    r = np.asarray(repl_seg, dtype=np.float64)
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    span_m = float(np.hypot(*(B - A)))
    ol, rl = float(path_length(o)), float(path_length(r))
    detour = float(rl / max(ol, 1e-9))
    local = bool(span_m <= SPAN_MAX and detour <= DETOUR_MAX)
    if detour < COMPLEX_EDGES[0]:
        cx = 0
    elif detour < COMPLEX_EDGES[1]:
        cx = 1
    else:
        cx = 2
    return {"repairable": LOCAL if local else NONLOCAL,
            "span_m": span_m, "detour": detour, "complexity": cx}
