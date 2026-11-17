"""Canonical snapshot identity + durable-target selection (NO ROS dependency).

Pure-python helpers shared by the 2D RuleSpec and 3D RuleVolume transaction
paths. Imported by manager_node (never the reverse) so unit tests can load
this file by path without rclpy. Floats use the schema-defined 1e-12 decimal
normalization; rows use canonical JSON framing.

Hash contract (both dimensions):
  h_e = full sha256 over a schema-tagged, sorted canonical serialization.
  The COMPLETE ACTIVE SET is hashed, never bare IDs: same epoch + same
  IDs + different geometry MUST hash differently.
"""
import hashlib as _hl
import json as _json
import math as _mm


HASH_SCHEMA = "rulesync.snapshot.v2"


def _cf(x):
    value = float(x)
    # Canonicalize signed zero and absorb only sub-1e-12 arithmetic noise from
    # equivalent center/dimension <-> bound conversions. This still separates
    # adjacent float32 coordinates used by planar RuleSpec geometry.
    if value == 0.0:
        value = 0.0
    return f"{value:.12f}"


def _cpt(q):
    if isinstance(q, dict):
        vals = [q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0)]
    elif hasattr(q, "x"):
        vals = [q.x, q.y, getattr(q, "z", 0.0)]
    else:
        vals = list(q)
    while len(vals) < 3:
        vals.append(0.0)
    return ",".join(_cf(v) for v in vals[:3])


def _cts(t):
    if t is None:
        return "none"
    if hasattr(t, "sec"):
        return f"{int(t.sec)}.{int(getattr(t, 'nanosec', 0) or 0)}"
    if isinstance(t, (list, tuple)) and len(t) == 2:
        return f"{int(t[0])}.{int(t[1])}"
    return str(t)


def _canon_vol_msg(v):
    vs = getattr(v, "vertices", []) or []
    try:
        verts = [_cpt(q) for q in list(vs)]
    except Exception:
        verts = []
    return (
        str(getattr(v, "rule_id", "?")), int(getattr(v, "epoch", 0)),
        int(getattr(v, "rule_type", 0)),
        int(getattr(v, "severity", 1)),
        str(getattr(v, "frame_id", "")),
        int(getattr(v, "geometry_type", 0)),
        _cpt((v.center.x, v.center.y, v.center.z)),
        _cpt((v.dimensions.x, v.dimensions.y, v.dimensions.z)),
        ";".join(verts),
        _cts(getattr(v, "valid_from", None)),
        _cts(getattr(v, "valid_until", None)),
    )


def _vol_snapshot_hash(vols):
    items = sorted(_json.dumps(
        list(_canon_vol_msg(v)), ensure_ascii=False, separators=(",", ":"))
        for v in vols)
    payload = HASH_SCHEMA + "\n" + "\n".join(items)
    return _hl.sha256(payload.encode()).hexdigest()


def _rule_snapshot_hash(rules):
    items = []
    for r in rules:
        pts = list(getattr(getattr(r, "polygon", None), "points", [])
                   or [])
        fields = [
            str(getattr(r, "rule_id", "?")), int(getattr(r, "epoch", 0)),
            int(getattr(r, "rule_type", 0)),
            int(getattr(r, "severity", 1)),
            str(getattr(r, "frame_id", "")),
            ";".join(_cpt((p.x, p.y, getattr(p, "z", 0.0)))
                     for p in pts),
            _cf(getattr(r, "speed_limit", 0.0) or 0.0),
            _cts(getattr(r, "valid_from", None)),
            _cts(getattr(r, "valid_until", None))]
        items.append(_json.dumps(
            fields, ensure_ascii=False, separators=(",", ":")))
    payload = HASH_SCHEMA + "\n" + "\n".join(sorted(items))
    return _hl.sha256(payload.encode()).hexdigest()


def _rget(r, name, default=None):
    if isinstance(r, dict):
        return r.get(name, default)
    return getattr(r, name, default)


def _rule_xy(r):
    """Polygon x/y pairs from a RuleSpec-like (msg or _rule_to_dict dict)."""
    try:
        if isinstance(r, dict):
            poly = r.get("polygon", []) or []
            return [(float(p[0]), float(p[1])) for p in list(poly)]
        pts = list(getattr(getattr(r, "polygon", None), "points", [])
                   or [])
        return [(float(p.x), float(p.y)) for p in pts]
    except Exception:
        return []


def _rule_measure(r):
    # Restrictiveness measure for TIGHTEN/RELAX classification.
    # KEEP_OUT (type 0): polygon area (bigger = tighter).
    # SPEED_LIMIT (type 1): negated limit (lower limit = tighter).
    try:
        rt = int(_rget(r, "rule_type", 0))
    except Exception:
        rt = 0
    if rt == 1:
        try:
            return -float(_rget(r, "speed_limit", 0.0) or 0.0)
        except Exception:
            return 0.0
    pts = _rule_xy(r)
    if len(pts) < 3:
        return 0.0
    area = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        area += x0 * y1 - x1 * y0
    return abs(area) / 2.0


def _rule_op_class(before, after):
    """Classify a complete-snapshot RuleSpec transition (never trust
    'newer' or 'bigger' alone): ADD / REMOVE / TIGHTEN / RELAX / REPLACE."""
    b, a = set(before), set(after)
    added, removed = a - b, b - a
    if added and not removed:
        return "ADD"
    if removed and not added:
        return "REMOVE"
    if added and removed:
        return "REPLACE"
    dirs = []
    for k in a & b:
        try:
            dm = _rule_measure(after[k]) - _rule_measure(before[k])
        except Exception:
            dm = 0.0
        if abs(dm) > 1e-9:
            dirs.append(1 if dm > 0 else -1)
    if dirs and all(d > 0 for d in dirs):
        return "TIGHTEN"
    if dirs and all(d < 0 for d in dirs):
        return "RELAX"
    return "REPLACE"


def _vol_measure(v):
    # Conservative volume extent for TIGHTEN/RELAX classification
    # (AABB dims product; sphere 4/3 pi r^3; convex via vertex bbox).
    g = int(getattr(v, "geometry_type", 0))
    if g == 1:
        r = float(v.dimensions.x)
        return (4.0 / 3.0) * _mm.pi * r ** 3
    if g == 2:
        xs = [float(p.x) for p in (v.vertices or [])] or [0.0]
        ys = [float(p.y) for p in (v.vertices or [])] or [0.0]
        zs = [float(p.z) for p in (v.vertices or [])] or [0.0]
        return max(0.0, (max(xs) - min(xs)) * (max(ys) - min(ys))
                   * (max(zs) - min(zs)))
    d = v.dimensions
    return max(0.0, float(d.x) * float(d.y) * float(d.z))


def _vol_op_class(before, after):
    """Classify a complete-snapshot transition (never trust 'newer' or
    'bigger' alone): ADD / REMOVE / TIGHTEN / RELAX / REPLACE."""
    b, a = set(before), set(after)
    added, removed = a - b, b - a
    if added and not removed:
        return "ADD"
    if removed and not added:
        return "REMOVE"
    if added and removed:
        return "REPLACE"
    dirs = []
    for k in a & b:
        try:
            dm = _vol_measure(after[k]) - _vol_measure(before[k])
        except Exception:
            dm = 0.0
        if abs(dm) > 1e-9:
            dirs.append(1 if dm > 0 else -1)
    if dirs and all(d > 0 for d in dirs):
        return "TIGHTEN"
    if dirs and all(d < 0 for d in dirs):
        return "RELAX"
    return "REPLACE"


def cert_matches(staged_txn, staged_epoch, staged_hash,
                 req_txn, req_epoch, req_hash):
    """Exact prepared-identity match for a COMMIT certificate.

    Stale/mismatched certificates MUST be rejected: any field difference
    (including epoch) fails closed. Mirrors the bridge-side check so both
    sides enforce the identical predicate.
    """
    try:
        if int(req_epoch) != int(staged_epoch):
            return False
    except Exception:
        return False
    return (str(req_txn or "") == str(staged_txn or "")
            and str(req_hash or "") == str(staged_hash or ""))


def select_durable_target(snapshot_epoch, snapshot_dicts, wal_records,
                           kind):
    """Pure durable-target selection (coordinator decision is authority).

    Newest COMMIT record of `kind` ("rule-decision" or "vol-decision")
    with epoch >= snapshot epoch wins; otherwise the snapshot stands.
    Returns (epoch, [complete target dicts]). Bridge state is NOT an
    input: participant state is evidence only and can never win here.
    """
    best = None
    for rec in wal_records or []:
        try:
            if rec.get("kind") == kind and rec.get("decision") == "COMMIT":
                if best is None or int(rec.get("epoch", 0)) > int(
                        best.get("epoch", 0)):
                    best = rec
        except Exception:
            continue
    if best is not None and int(best.get("epoch", 0)) >= int(
            snapshot_epoch):
        key = ("target_volumes" if kind == "vol-decision"
               else "target_rules")
        try:
            dicts = list(best.get(key, []) or [])
        except Exception:
            dicts = []
        return int(best.get("epoch", snapshot_epoch)), dicts
    return int(snapshot_epoch), list(snapshot_dicts or [])


def recovery_action(durable_epoch, durable_hash, durable_ids,
                    bridge_reachable, bridge_epoch, bridge_hash,
                    bridge_ids):
    """Pure recovery branch (decision-driven; adopt-bridge REMOVED).

    Returns one of:
      "hold-degraded"  -- bridge silent, or any disagreement: the manager
                          must NOT promote participant state and must NOT
                          clear DEGRADED. Caller replays the durable target
                          (idempotent) and stays DEGRADED until explicit
                          convergence.
      "converge"       -- bridge (epoch, hash, ids) explicitly equals the
                          durable target: the ONLY branch that may clear
                          DEGRADED.
    There is deliberately NO "adopt-bridge" outcome: an uncommitted
    participant REMOVE/RELAX can never become authoritative here.
    """
    try:
        if not bridge_reachable:
            return "hold-degraded"
        if (int(bridge_epoch) == int(durable_epoch)
                and str(bridge_hash or "")
                and str(bridge_hash) == str(durable_hash)
                and list(bridge_ids or []) == list(durable_ids or [])):
            return "converge"
    except Exception:
        return "hold-degraded"
    return "hold-degraded"
