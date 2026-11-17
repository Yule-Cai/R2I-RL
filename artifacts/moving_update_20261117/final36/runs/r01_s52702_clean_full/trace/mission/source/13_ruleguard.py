# RuleGuard: deterministic runtime execution monitor (NOT a planner).
#
# The path gate G_d verifies PLANNED paths; tracking can deviate.
# RuleGuard monitors ACTUAL executed pose against the committed snapshot:
#   clearance(p) = signed_distance(p, R_e) - rho
#     CLEAR   : clearance > rho_warn
#     WARNING : 0 < clearance <= rho_warn (event recorded)
#     HOLD    : clearance <= 0 (release needs hysteresis: clearance >
#               rho_release on 2 consecutive checks — no chatter)
# Epoch discipline: an admitted execution is pinned once to its authorizing
# (epoch, snapshot hash).  The monitor independently tracks the currently
# committed identity and authorizes motion only while the two identities agree;
# an authority change during execution therefore fails closed on the next
# monitor check. Invalid geometry/snapshot also fails closed. Velocity is
# accepted for future use (recorded, v1 unused; SPEED_LIMIT is not yet part of
# the execution-monitor predicate).
# 3D: exact AABB/SPHERE; CONVEX via bbox superset (clearance underestimated
# = fail-safe direction; exact convex lives in the path gate). 2D: exact
# polygon signed distance (ray-cast inside test + segment distances).
import hashlib
import json
import math

CLEAR, WARNING, HOLD = "CLEAR", "WARNING", "HOLD"
HASH_SCHEMA = "rulesync.snapshot.v2"

GEOM_AABB, GEOM_SPHERE, GEOM_CONVEX = 0, 1, 2


def snapshot_hash(items):
    """Canonical hash h_e over a rule snapshot (dimension-independent).

    items: iterable of canonical field tuples (any length; canon_volume
    / rule_snapshot_items output). Order-free (sorted)."""
    parts = sorted(json.dumps(
        list(it), ensure_ascii=False, separators=(",", ":")) for it in items)
    payload = HASH_SCHEMA + "\n" + "\n".join(parts)
    return hashlib.sha256(payload.encode()).hexdigest()


def _canon_float(x):
    # Absorb sub-1e-12 arithmetic noise from equivalent geometry conversions,
    # while retaining finer resolution than any planar Point32 coordinate.
    value = float(x)
    if value == 0.0:
        value = 0.0
    return f"{value:.12f}"


def _canon_pt(p):
    p = list(p)
    while len(p) < 3:
        p.append(0.0)
    return ",".join(_canon_float(v) for v in p[:3])


def _get(d, *names, default=None):
    # Uniform accessor over dicts and ROS-msg-like objects.
    for nm in names:
        if isinstance(d, dict):
            if nm in d:
                return d[nm]
        elif hasattr(d, nm):
            return getattr(d, nm)
    return default


def _pt3(q):
    # A geometry point in any supported shape -> [x, y, z] floats.
    if isinstance(q, dict):
        return [float(q.get("x", 0.0)), float(q.get("y", 0.0)),
                float(q.get("z", 0.0))]
    if hasattr(q, "x"):
        return [float(q.x), float(q.y), float(getattr(q, "z", 0.0))]
    q = list(q)
    while len(q) < 3:
        q.append(0.0)
    return [float(v) for v in q[:3]]


def _canon_ts(t):
    if t is None:
        return "none"
    s = _get(t, "sec", default=None)
    ns = _get(t, "nanosec", default=None)
    if s is not None:
        return f"{int(s)}.{int(ns or 0)}"
    if isinstance(t, (list, tuple)) and len(t) == 2:
        return f"{int(t[0])}.{int(t[1])}"
    return str(t)


def canon_volume(v):
    """Any RuleVolume representation -> canonical hash tuple covering ALL
    authorization-relevant semantics.

    Accepted schemas: ROS RuleVolume msg (center/dims/vertices objects);
    msg-style dicts (center/dims/vertices lists); gate dicts (bmin/bmax
    arrays, or center array + radius scalar). ALL parties hashing one
    snapshot must use the SAME inflation state (all-INFLATED or all-RAW,
    never mixed): AABB gate dicts derive center/dims exactly from
    bmin/bmax; sphere gate dicts map radius -> dims[0] per msg convention.
    Two volumes differing in ANY field (incl. every convex vertex and
    validity bounds) hash differently."""
    g = int(_get(v, "geometry_type", default=0))
    center = [0.0, 0.0, 0.0]
    dims = [0.0, 0.0, 0.0]
    verts = []
    _has_bmin = ("bmin" in v) if isinstance(v, dict) else hasattr(v, "bmin")
    if _has_bmin:
        bmin = [float(x) for x in list(_get(v, "bmin"))]
        bmax = [float(x) for x in list(_get(v, "bmax"))]
        center = [(a + b) / 2.0 for a, b in zip(bmin, bmax)]
        dims = [b - a for a, b in zip(bmin, bmax)]
    else:
        c = _get(v, "center", default=None)
        if c is not None:
            center = _pt3(c)
        d = _get(v, "dimensions", "dims", default=None)
        if d is not None:
            # _pt3 handles ROS Vector3 (x/y/z attrs, not iterable),
            # lists, and dicts uniformly.
            dims = _pt3(d)
        r = _get(v, "radius", default=None)
        if r is not None and g == GEOM_SPHERE:
            dims = [float(r), 0.0, 0.0]
    vv = _get(v, "vertices", default=[]) or []
    try:
        verts = [_pt3(q) for q in list(vv)]
    except Exception:
        verts = []
    return (
        str(_get(v, "rule_id", default="?")),
        int(_get(v, "epoch", default=0)),
        int(_get(v, "rule_type", default=0)),
        int(_get(v, "severity", default=1)),
        str(_get(v, "frame_id", default="")),
        g,
        _canon_pt(center),
        _canon_pt(dims),
        ";".join(_canon_pt(q) for q in verts),
        _canon_ts(_get(v, "valid_from", default=None)),
        _canon_ts(_get(v, "valid_until", default=None)),
    )


def volume_snapshot_items(vols):
    """RuleVolume-likes (any schema) -> canonical hash items."""
    return [canon_volume(v) for v in vols]


def rule_snapshot_items(rules):
    """2D RuleSpec-likes (dict or msg) -> canonical hash items covering
    ALL authorization-relevant semantics: id, epoch, type, severity,
    frame, EVERY polygon vertex, speed_limit, validity window."""
    out = []
    for r in rules:
        pts = _get(r, "polygon", default=None)
        if pts is not None and not isinstance(pts, (list, tuple)):
            pts = getattr(pts, "points", [])
        if pts is None:
            pts = _get(r, "points", default=[]) or []
        try:
            plist = [_pt3(q) for q in list(pts)]
        except Exception:
            plist = []
        out.append((
            str(_get(r, "rule_id", default="?")),
            int(_get(r, "epoch", default=0)),
            int(_get(r, "rule_type", default=0)),
            int(_get(r, "severity", default=1)),
            str(_get(r, "frame_id", default="")),
            ";".join(_canon_pt(q) for q in plist),
            _canon_float(_get(r, "speed_limit", default=0.0) or 0.0),
            _canon_ts(_get(r, "valid_from", default=None)),
            _canon_ts(_get(r, "valid_until", default=None)),
        ))
    return out


def sd_aabb(p, bmin, bmax):
    # Signed distance to AABB (negative inside). Exact.
    qx = max(bmin[0] - p[0], 0.0, p[0] - bmax[0])
    qy = max(bmin[1] - p[1], 0.0, p[1] - bmax[1])
    qz = max(bmin[2] - p[2], 0.0, p[2] - bmax[2])
    outside = math.sqrt(qx * qx + qy * qy + qz * qz)
    if outside > 0:
        return outside
    inside = -min(p[0] - bmin[0], bmax[0] - p[0], p[1] - bmin[1],
                  bmax[1] - p[1], p[2] - bmin[2], bmax[2] - p[2])
    return inside


def sd_sphere(p, c, r):
    return math.sqrt((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2
                     + (p[2] - c[2]) ** 2) - r


def _seg_dist_2d(p, a, b):
    ax, ay, bx, by, px, py = a[0], a[1], b[0], b[1], p[0], p[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx
                                                  + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _inside_poly_2d(p, poly):
    inside, n = False, len(poly)
    x, y = p[0], p[1]
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if ((y0 > y) != (y1 > y)) and \
                (x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0):
            inside = not inside
    return inside


def sd_poly_2d(p, poly):
    d = min(_seg_dist_2d(p, poly[i], poly[(i + 1) % len(poly)])
            for i in range(len(poly)))
    return -d if _inside_poly_2d(p, poly) else d


def min_clearance_3d(p, vols, rho):
    """Min over volumes of (signed distance - rho). +inf if no volumes."""
    best = math.inf
    for v in vols:
        g = int(v["geometry_type"])
        if g == GEOM_AABB:
            sd = sd_aabb(p, v["bmin"], v["bmax"])
        elif g == GEOM_SPHERE:
            sd = sd_sphere(p, v["center"], v["radius"])
        else:  # CONVEX fallback: bbox superset (fail-safe: underestimates)
            vs = v["vertices"]
            xs = [q[0] for q in vs]
            ys = [q[1] for q in vs]
            zs = [q[2] for q in vs]
            sd = sd_aabb(p, (min(xs), min(ys), min(zs)),
                         (max(xs), max(ys), max(zs)))
        best = min(best, sd - rho)
    return best


def min_clearance_2d(p, polys, rho):
    best = math.inf
    for poly in polys:
        best = min(best, sd_poly_2d(p, poly) - rho)
    return best


class RuleGuard:
    """Stateful execution monitor with hysteresis + epoch discipline."""

    def __init__(self, rho_warn=0.5, rho_release=0.2):
        self.rho_warn = float(rho_warn)
        self.rho_release = float(rho_release)
        self.epoch = None
        self.snap_hash = None
        self._exec_epoch = None
        self._exec_hash = None
        self._hold_streak = 0
        self._release_streak = 0
        self.events = []  # (seq, state, clearance, epoch)

    def update_snapshot(self, epoch, snap_hash):
        """Install the monitor's independently observed committed identity.

        This deliberately does not rewrite the identity of an execution that
        is already in flight. If authority advances from e to e+1, the next
        check_bound* call compares the pinned execution identity with the new
        monitor identity and returns HOLD.
        """
        new_epoch, new_hash = int(epoch), str(snap_hash)
        changed = (new_epoch != self.epoch or new_hash != self.snap_hash)
        self.epoch, self.snap_hash = new_epoch, new_hash
        if changed:
            self._hold_streak = 0
            self._release_streak = 0

    def invalidate_snapshot(self):
        """Forget monitor authority without altering an in-flight binding.

        Service loss, a manager DEGRADED response, or a canonical-hash
        mismatch makes the independently observed authority unknown.  The
        next bound check therefore returns HOLD rather than continuing under
        the last cached snapshot.
        """
        self.epoch = None
        self.snap_hash = None
        self._hold_streak = 0
        self._release_streak = 0

    def bind_execution(self, auth_epoch, auth_hash):
        """Pin one admitted execution to the identity that authorized it.

        Binding is retained even when the monitor has no snapshot or observes
        a different identity. This separation prevents callers from passing
        the monitor's current identity back into every check and accidentally
        making agreement tautological.
        """
        self._exec_epoch = int(auth_epoch)
        self._exec_hash = str(auth_hash)
        return (self.epoch is not None and self.snap_hash is not None
                and self._exec_epoch == int(self.epoch)
                and self._exec_hash == str(self.snap_hash))

    def clear_execution(self):
        self._exec_epoch = None
        self._exec_hash = None

    def execution_identity(self):
        if self._exec_epoch is None or self._exec_hash is None:
            return None
        return self._exec_epoch, self._exec_hash

    def check_geometry(self, clearance):
        """ARM B — geometry-only monitor: same warning/HOLD geometry as
        check(), but deliberately WITHOUT epoch/hash binding. Used to
        separate 'distance threshold effect' from 'snapshot-bound
        execution authority' (Arm C). No hysteresis state is shared with
        check() (independent streaks would confound the comparison)."""
        if not math.isfinite(clearance):
            return CLEAR, {"arm": "B", "empty_set": True}
        if clearance <= 0:
            return HOLD, {"arm": "B", "reason": "boundary-violated"}
        if clearance <= self.rho_warn:
            return WARNING, {"arm": "B", "reason": "near-boundary"}
        return CLEAR, {"arm": "B"}

    def check(self, clearance, auth_epoch, auth_hash, velocity=None):
        """Core decision on a precomputed clearance (dimension-free).

        Returns (state, info). Any inconsistency -> HOLD + degraded.
        """
        seq = len(self.events)
        if self.epoch is None or self.snap_hash is None:
            self.events.append((seq, HOLD, clearance, None))
            return HOLD, {"degraded": True, "reason": "monitor-no-snapshot"}
        if int(auth_epoch) != int(self.epoch) or \
                str(auth_hash) != str(self.snap_hash):
            self.events.append((seq, HOLD, clearance, self.epoch))
            return HOLD, {"degraded": True, "reason": "epoch-disagreement",
                          "monitor_epoch": self.epoch,
                          "authority_epoch": int(auth_epoch)}
        if not math.isfinite(clearance):
            # No active volumes/polys and consistent epochs: vacuous CLEAR
            # is UNSOUND for a monitor (nothing to watch) — report CLEAR
            # with explicit empty-set flag (caller decides motion policy;
            # RuleSync authorizes via gate+epoch, not via empty monitor).
            self.events.append((seq, CLEAR, clearance, self.epoch))
            return CLEAR, {"degraded": False, "empty_set": True}
        if clearance <= 0:
            self._hold_streak += 1
            self._release_streak = 0
            self.events.append((seq, HOLD, clearance, self.epoch))
            return HOLD, {"degraded": False, "reason": "boundary-violated"}
        if self._hold_streak > 0:
            if clearance > self.rho_release:
                self._release_streak += 1
                if self._release_streak >= 2:
                    self._hold_streak = 0
                    self._release_streak = 0
                else:
                    self.events.append((seq, HOLD, clearance, self.epoch))
                    return HOLD, {"degraded": False,
                                  "reason": "hysteresis-hold"}
            else:
                self._release_streak = 0
                self.events.append((seq, HOLD, clearance, self.epoch))
                return HOLD, {"degraded": False, "reason": "hysteresis-hold"}
        if clearance <= self.rho_warn:
            self.events.append((seq, WARNING, clearance, self.epoch))
            return WARNING, {"degraded": False, "reason": "near-boundary"}
        self.events.append((seq, CLEAR, clearance, self.epoch))
        return CLEAR, {"degraded": False}

    def check_bound(self, clearance, velocity=None):
        """Check against the identity pinned by bind_execution()."""
        if self._exec_epoch is None or self._exec_hash is None:
            seq = len(self.events)
            self.events.append((seq, HOLD, clearance, self.epoch))
            return HOLD, {"degraded": True,
                          "reason": "execution-not-bound"}
        return self.check(clearance, self._exec_epoch, self._exec_hash,
                          velocity)

    def check_3d(self, xyz, vols, rho, auth_epoch, auth_hash,
                 velocity=None):
        try:
            c = min_clearance_3d(tuple(xyz), vols, float(rho))
        except Exception:
            self.events.append((len(self.events), HOLD, float("nan"),
                                self.epoch))
            return HOLD, {"degraded": True, "reason": "invalid-geometry"}
        st, info = self.check(c, auth_epoch, auth_hash, velocity)
        info["clearance"] = c
        return st, info

    def check_bound_3d(self, xyz, vols, rho, velocity=None):
        try:
            c = min_clearance_3d(tuple(xyz), vols, float(rho))
        except Exception:
            self.events.append((len(self.events), HOLD, float("nan"),
                                self.epoch))
            return HOLD, {"degraded": True, "reason": "invalid-geometry"}
        st, info = self.check_bound(c, velocity)
        info["clearance"] = c
        return st, info

    def check_2d(self, xy, polys, rho, auth_epoch, auth_hash,
                 velocity=None):
        try:
            c = min_clearance_2d(tuple(xy), polys, float(rho))
        except Exception:
            self.events.append((len(self.events), HOLD, float("nan"),
                                self.epoch))
            return HOLD, {"degraded": True, "reason": "invalid-geometry"}
        st, info = self.check(c, auth_epoch, auth_hash, velocity)
        info["clearance"] = c
        return st, info

    def check_bound_2d(self, xy, polys, rho, velocity=None):
        try:
            c = min_clearance_2d(tuple(xy), polys, float(rho))
        except Exception:
            self.events.append((len(self.events), HOLD, float("nan"),
                                self.epoch))
            return HOLD, {"degraded": True, "reason": "invalid-geometry"}
        st, info = self.check_bound(c, velocity)
        info["clearance"] = c
        return st, info
