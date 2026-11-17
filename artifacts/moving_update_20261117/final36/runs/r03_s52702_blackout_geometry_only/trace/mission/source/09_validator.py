"""Deterministic RuleSpec validator (no ROS dependency; unit-tested)."""
import math
from fractions import Fraction

KEEP_OUT = 0
SPEED_LIMIT = 1


def is_finite_number(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(float(x))


def _orientation(a, b, c):
    """Orientation sign, with an exact fallback for near-collinear floats."""
    left = (b[0] - a[0]) * (c[1] - a[1])
    right = (b[1] - a[1]) * (c[0] - a[0])
    det = left - right
    if abs(det) > 8 * math.ulp(1.0) * (abs(left) + abs(right)):
        return (det > 0) - (det < 0)
    ax, ay, bx, by, cx, cy = map(Fraction, (*a, *b, *c))
    exact = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    return (exact > 0) - (exact < 0)


def _edges_intersect(a, b, c, d):
    if (max(a[0], b[0]) < min(c[0], d[0])
            or max(c[0], d[0]) < min(a[0], b[0])
            or max(a[1], b[1]) < min(c[1], d[1])
            or max(c[1], d[1]) < min(a[1], b[1])):
        return False
    return (_orientation(a, b, c) * _orientation(a, b, d) <= 0
            and _orientation(c, d, a) * _orientation(c, d, b) <= 0)


def validate_polygon(points) -> tuple[bool, str]:
    """Accept a finite simple ring, either winding, optionally explicitly closed.

    Validation never reorders or changes the submitted representation/hash.
    Holes, self-contact, crossing/overlapping edges and duplicate vertices are
    not supported. Straight-through collinear boundary vertices are allowed.
    """
    try:
        raw = list(points)
    except Exception:
        return False, "polygon points unreadable"
    try:
        pts = [(float(p.x), float(p.y)) for p in raw]
    except Exception:
        try:
            pts = [(float(p[0]), float(p[1])) for p in raw]
        except Exception:
            return False, "polygon points unreadable"
    if len(pts) < 3:
        return False, f"polygon needs >=3 points, got {len(pts)}"
    for (x, y) in pts:
        if not (is_finite_number(x) and is_finite_number(y)):
            return False, "polygon contains NaN/Inf"
        if abs(x) > 1e4 or abs(y) > 1e4:
            return False, "polygon coordinate out of range"
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        return False, "polygon needs >=3 distinct points"
    if len(set(pts)) != len(pts):
        return False, "polygon has duplicate vertices / zero-length edge"
    origin_x, origin_y = pts[0]
    terms = []
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        terms.append((x0 - origin_x) * (y1 - origin_y)
                     - (x1 - origin_x) * (y0 - origin_y))
    area = math.fsum(terms)
    if abs(area) < 1e-9:
        return False, "degenerate (zero-area) polygon"
    count = len(pts)
    for i, b in enumerate(pts):
        a, c = pts[i - 1], pts[(i + 1) % count]
        if (_orientation(a, b, c) == 0
                and ((a[0] - b[0]) * (c[0] - b[0])
                     + (a[1] - b[1]) * (c[1] - b[1])) > 0):
            return False, "polygon has overlapping adjacent edges"
        for j in range(i + 1, count):
            if j == i + 1 or (i == 0 and j == count - 1):
                continue
            if _edges_intersect(b, c, pts[j], pts[(j + 1) % count]):
                return False, "polygon has self-intersection / self-contact"
    return True, "ok"


def validate_rule(rule, committed_epoch: int,
                  allowed_frames=("map",)) -> tuple[bool, str]:
    if not getattr(rule, "rule_id", ""):
        return False, "empty rule_id"
    if int(getattr(rule, "epoch", 0)) <= int(committed_epoch):
        return False, (f"stale epoch {getattr(rule, 'epoch', 0)} "
                       f"<= committed {committed_epoch}")
    rt = int(getattr(rule, "rule_type", -1))
    if rt not in (KEEP_OUT, SPEED_LIMIT):
        return False, f"unknown rule_type {rt}"
    if int(getattr(rule, "severity", -1)) not in (0, 1):
        return False, "severity must be 0 or 1"
    if getattr(rule, "frame_id", "") not in allowed_frames:
        return False, f"frame_id '{getattr(rule, 'frame_id', '')}' not allowed"
    poly = getattr(rule, "polygon", None)
    pts = getattr(poly, "points", []) if poly is not None else []
    ok, reason = validate_polygon(pts)
    if not ok:
        return False, reason
    if rt == SPEED_LIMIT and not float(getattr(rule, "speed_limit", 0.0)) > 0.0:
        return False, "speed_limit must be > 0 for SPEED_LIMIT"
    vf = getattr(getattr(rule, "valid_from", None), "sec", 0)
    vu = getattr(getattr(rule, "valid_until", None), "sec", 0)
    if vu <= vf:
        return False, "valid_until must be > valid_from"
    return True, "ok"
