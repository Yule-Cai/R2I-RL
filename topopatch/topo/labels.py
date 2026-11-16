"""Deterministic topology labels for teacher repair samples (numpy only).

Representation (geometry-consistent; NOT screen coordinates):
  Let A,B be the divergence/reconvergence anchors and u = (B-A)/|B-A| the
  directed travel axis with left normal n = (-uy, ux).
  For the teacher repair segment R, let M* be the point of R with maximum
  absolute lateral offset s(P) = dot(P - A, n) from the directed axis AB.
  - SIDE_POS (1): s(M*) > +deadband  (detour on the +n side of travel)
  - SIDE_NEG (0): s(M*) < -deadband  (detour on the -n side of travel)
  - AMBIGUOUS (2): |s(M*)| <= deadband (too straight to call), or the
    anchor span is below minimum, or a multi-rule clearance conflict
    (below) makes the assignment unreliable. Ambiguous samples are
    explicitly marked, never force-labeled.

Multi-rule corridor signature (compact, deterministic):
  For each KEEP_OUT rule k with centroid Ck: clearance ck = min distance
  from R to polygon k, side_k = sign(dot(M* - Ck, n)) with the same
  deadband (0 if within deadband). Corridor flag = True when >= 2 rules
  have opposite non-zero sides (repair threads between them). Signature
  string: "S:{P|N|A}|R0:{P|N|0}/c0.42,R1:..." (clearances in meters,
  2 decimals). A per-rule clearance < TIGHT_M marks the sample AMBIGUOUS
  (too tight to call reliably).

Only information available to the planner is used (anchors, teacher path,
rule geometry). No simulator futures, no learning.
"""
import math

import numpy as np

from topopatch import TOPO_AMBIGUOUS, TOPO_SIDE_NEG, TOPO_SIDE_POS

DEADBAND_M = 0.25      # lateral deadband for side assignment (m)
TIGHT_M = 0.15         # per-rule clearance below which label is ambiguous (m)
MIN_SPAN_M = 1.0       # anchor span below which label is ambiguous (m)


def _axis(A, B):
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    d = B - A
    L = float(np.hypot(d[0], d[1]))
    if L < 1e-9:
        return None, None, 0.0
    u = d / L
    n = np.array([-u[1], u[0]])
    return u, n, L


def detour_side(A, B, repl_seg, deadband=DEADBAND_M):
    """Return (cls, signed_offset_m, margin_m).

    cls in {SIDE_NEG, SIDE_POS, AMBIGUOUS}; signed_offset is s(M*);
    margin = |s(M*)| - deadband (negative when ambiguous-by-deadband).
    """
    u, n, L = _axis(A, B)
    r = np.asarray(repl_seg, dtype=np.float64)
    if u is None or len(r) == 0:
        return TOPO_AMBIGUOUS, 0.0, -float(deadband)
    A = np.asarray(A, dtype=np.float64)
    s = (r - A) @ n
    i = int(np.argmax(np.abs(s)))
    smax = float(s[i])
    if L < MIN_SPAN_M:
        return TOPO_AMBIGUOUS, smax, -float(deadband)
    if smax > deadband:
        return TOPO_SIDE_POS, smax, smax - deadband
    if smax < -deadband:
        return TOPO_SIDE_NEG, smax, -smax - deadband
    return TOPO_AMBIGUOUS, smax, abs(smax) - deadband


def _seg_point_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy + 1e-12
    t = min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def rule_clearance(repl_seg, poly):
    """Min distance from repair segment to polygon edges (m)."""
    r = np.asarray(repl_seg, dtype=np.float64)
    p = np.asarray(poly, dtype=np.float64)
    if len(r) == 0 or len(p) < 3:
        return 0.0
    best = float("inf")
    n = len(p)
    for q in r:
        for i in range(n):
            d = _seg_point_dist(float(q[0]), float(q[1]),
                                float(p[i][0]), float(p[i][1]),
                                float(p[(i + 1) % n][0]),
                                float(p[(i + 1) % n][1]))
            if d < best:
                best = d
    return float(best)


def label_sample(A, B, repl_seg, rule_polys, deadband=DEADBAND_M,
                 tight=TIGHT_M):
    """Deterministic topology label for one teacher sample.

    rule_polys: list of polygons (each >=3 points). Returns dict:
      cls, signed_offset_m, margin_m, corridor (bool),
      per_rule: [{side, clearance_m}], signature (str), ambiguous (bool).
    """
    u, n, L = _axis(A, B)
    r = np.asarray(repl_seg, dtype=np.float64)
    A = np.asarray(A, dtype=np.float64)
    if u is None or len(r) == 0 or not rule_polys:
        return {"cls": TOPO_AMBIGUOUS, "signed_offset_m": 0.0,
                "margin_m": -float(deadband), "corridor": False,
                "per_rule": [], "signature": "S:A|",
                "ambiguous": True}
    s = (r - A) @ n
    i = int(np.argmax(np.abs(s)))
    Mstar = r[i]
    smax = float(s[i])
    per_rule = []
    for poly in rule_polys:
        p = np.asarray(poly, dtype=np.float64)
        Ck = p.mean(axis=0)
        sk = float((Mstar - Ck) @ n)
        if sk > deadband:
            side = "P"
        elif sk < -deadband:
            side = "N"
        else:
            side = "0"
        ck = rule_clearance(r, p)
        per_rule.append({"side": side, "clearance_m": ck})
    nonzero = [q["side"] for q in per_rule if q["side"] != "0"]
    corridor = len(set(nonzero)) > 1
    tight_hit = any(q["clearance_m"] < tight for q in per_rule)
    if L < MIN_SPAN_M or abs(smax) <= deadband or tight_hit:
        cls = TOPO_AMBIGUOUS
    else:
        cls = TOPO_SIDE_POS if smax > 0 else TOPO_SIDE_NEG
    sig = ("S:" + ("P" if cls == TOPO_SIDE_POS else
                   "N" if cls == TOPO_SIDE_NEG else "A") + "|"
           + ",".join(f"R{k}:{q['side']}/c{q['clearance_m']:.2f}"
                      for k, q in enumerate(per_rule)))
    return {"cls": cls, "signed_offset_m": smax,
            "margin_m": (abs(smax) - deadband),
            "corridor": bool(corridor), "per_rule": per_rule,
            "signature": sig,
            "ambiguous": bool(cls == TOPO_AMBIGUOUS)}
