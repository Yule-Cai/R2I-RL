"""Standalone deterministic 2-D path-admission primitives.

This module is part of the packaged RuleSync runtime.  It deliberately depends
only on NumPy so the ROS admission node does not need the TopoPatch or
RulePatch research/training trees at execution time.
"""

import numpy as np
import math


COLL_RADIUS = 0.1
COLL_STEPS = (-0.1, 0.0, 0.1)


def _point_in_polygon(xy, poly):
    """Return whether ``xy`` is inside ``poly`` using deterministic ray casting.

    Boundary contact is handled separately by the exact segment predicates and
    is therefore rejected fail-closed.
    """
    x, y = float(xy[0]), float(xy[1])
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = float(poly[i][0]), float(poly[i][1])
        x1, y1 = float(poly[(i + 1) % n][0]), float(poly[(i + 1) % n][1])
        if ((y0 > y) != (y1 > y)) and (
            x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0
        ):
            inside = not inside
    return inside


def _collision_free(path, occ_fn):
    for x, y in np.asarray(path, dtype=np.float64):
        for dx in COLL_STEPS:
            for dy in COLL_STEPS:
                if occ_fn(float(x) + dx, float(y) + dy):
                    return False
    return True


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - \
        (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a, b, c, eps=1e-9):
    return (min(a[0], c[0]) - eps <= b[0] <= max(a[0], c[0]) + eps
            and min(a[1], c[1]) - eps <= b[1] <= max(a[1], c[1]) + eps)


def segments_intersect(p1, p2, p3, p4, eps=1e-9):
    """Return whether two closed 2-D segments intersect."""
    o1 = _orient(p1, p2, p3)
    o2 = _orient(p1, p2, p4)
    o3 = _orient(p3, p4, p1)
    o4 = _orient(p3, p4, p2)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and \
       ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    if abs(o1) <= eps and _on_segment(p1, p3, p2, eps):
        return True
    if abs(o2) <= eps and _on_segment(p1, p4, p2, eps):
        return True
    if abs(o3) <= eps and _on_segment(p3, p1, p4, eps):
        return True
    if abs(o4) <= eps and _on_segment(p3, p2, p4, eps):
        return True
    return False


def _point_segment_distance(point, start, end):
    """Euclidean distance from a point to a closed line segment."""
    px, py = float(point[0]), float(point[1])
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])
    dx, dy = bx - ax, by - ay
    norm_sq = dx * dx + dy * dy
    if norm_sq <= 1e-24:
        return math.hypot(px - ax, py - ay)
    fraction = ((px - ax) * dx + (py - ay) * dy) / norm_sq
    fraction = min(1.0, max(0.0, fraction))
    qx, qy = ax + fraction * dx, ay + fraction * dy
    return math.hypot(px - qx, py - qy)


def segment_distance(p1, p2, p3, p4):
    """Minimum Euclidean distance between two closed 2-D segments."""
    if segments_intersect(p1, p2, p3, p4):
        return 0.0
    return min(
        _point_segment_distance(p1, p3, p4),
        _point_segment_distance(p2, p3, p4),
        _point_segment_distance(p3, p1, p2),
        _point_segment_distance(p4, p1, p2),
    )


def segment_intersects_polygon(p1, p2, poly, inflation_radius=0.0):
    """Return whether a segment touches an inflated polygon.

    ``inflation_radius`` defines the Euclidean Minkowski sum of the polygon
    with a closed disk.  The test is continuous: a segment is rejected when
    it enters the polygon or comes within the configured radius of any edge.
    """
    poly = np.asarray(poly, dtype=np.float64)
    rho = float(inflation_radius)
    if not np.isfinite(rho) or rho < 0.0:
        raise ValueError("inflation_radius must be finite and non-negative")
    if _point_in_polygon(p1, poly) or _point_in_polygon(p2, poly):
        return True
    n = poly.shape[0]
    for i in range(n):
        edge_start, edge_end = poly[i], poly[(i + 1) % n]
        if segments_intersect(p1, p2, edge_start, edge_end):
            return True
        if rho > 0.0 and segment_distance(
                p1, p2, edge_start, edge_end) <= rho + 1e-12:
            return True
    return False


def path_intersects_polygons(path, polys, inflation_radius=0.0):
    """Check every path segment against every inflated polygon."""
    path_array = np.asarray(path, dtype=np.float64)
    polygons = [np.asarray(poly, dtype=np.float64) for poly in polys]
    for i in range(len(path_array) - 1):
        for polygon in polygons:
            if segment_intersects_polygon(
                    path_array[i], path_array[i + 1], polygon,
                    inflation_radius=inflation_radius):
                return True
    return False


class PreparedPolygonIndex:
    """Immutable broad-phase BVH; the original predicate remains decisive.

    Bounds are expanded by rho plus the existing coordinate-contact tolerance,
    then rounded outward. An overlapping box is only a candidate, never proof
    of a violation. No polygon buffering or approximate inflation is used.
    Construction costs are separate from reuse costs. Dense overlap still
    requires a direct-scan number of narrow-phase checks in the worst case.
    Near-horizontal edges whose legacy ray-cast perturbation can extend beyond
    the box padding are checked without pruning, preserving legacy behavior.
    """

    def __init__(self, polygons, inflation_radius=0.0, leaf_size=4):
        rho = float(inflation_radius)
        if not math.isfinite(rho) or rho < 0:
            raise ValueError("inflation_radius must be finite and non-negative")
        if leaf_size < 1 or int(leaf_size) != leaf_size:
            raise ValueError("leaf_size must be a positive integer")
        owned = []
        for polygon in polygons:
            p = np.array(polygon, dtype=np.float64, copy=True)
            if (p.ndim != 2 or p.shape[0] < 3 or p.shape[1] != 2
                    or not np.isfinite(p).all()):
                raise ValueError("index requires finite polygon rings")
            p.flags.writeable = False
            owned.append(p)
        self.polygons = tuple(owned)
        self.inflation_radius = rho
        # _point_in_polygon adds 1e-12 to dy. For a non-horizontal edge,
        # its perturbed ray crossing can exceed the true x extent by
        # |dx*1e-12/(dy+1e-12)|. Do not prune such rings when this exceeds
        # the existing 1e-9 coordinate padding (including a zero denominator).
        # This preserves the original predicate rather than silently changing
        # its decisions. It does not certify arbitrary floating-point geometry.
        unpruned = []
        for j, p in enumerate(owned):
            edges = np.roll(p, -1, axis=0) - p
            dx, dy = edges[:, 0], edges[:, 1]
            if np.any((dy != 0) &
                      (np.abs(dx) * 1e-12 >= np.abs(dy + 1e-12) * 1e-9)):
                unpruned.append(j)
        self._unpruned = tuple(unpruned)
        padding = rho + 1e-9
        self._bounds = np.asarray([
            (*np.nextafter(p.min(axis=0) - padding, -np.inf),
             *np.nextafter(p.max(axis=0) + padding, np.inf))
            for p in owned], dtype=np.float64).reshape(-1, 4)
        if not np.isfinite(self._bounds).all():
            raise ValueError("non-finite expanded bounds")

        def build(ids):
            boxes = self._bounds[ids]
            bounds = (float(boxes[:, 0].min()), float(boxes[:, 1].min()),
                      float(boxes[:, 2].max()), float(boxes[:, 3].max()))
            if len(ids) <= leaf_size:
                return bounds, tuple(int(i) for i in ids), None, None
            centers = boxes[:, :2] / 2 + boxes[:, 2:] / 2
            axis = int(np.argmax(np.ptp(centers, axis=0)))
            order = np.argsort(centers[:, axis], kind='stable')
            split = len(ids) // 2
            return (bounds, (), build(ids[order[:split]]),
                    build(ids[order[split:]]))

        prunable = np.asarray([i for i in range(len(owned))
                               if i not in self._unpruned], dtype=int)
        self._root = build(prunable) if len(prunable) else None

    @staticmethod
    def _overlap(a, b):
        return (a[0] <= b[2] and a[2] >= b[0]
                and a[1] <= b[3] and a[3] >= b[1])

    def _query(self, box, stats):
        pending = [self._root] if self._root is not None else []
        candidates = list(self._unpruned)
        while pending:
            node = pending.pop()
            if stats is not None:
                stats['nodes_visited'] += 1
            bounds, ids, left, right = node
            if not self._overlap(box, bounds):
                continue
            if left is None:
                candidates.extend(i for i in ids
                                  if self._overlap(box, self._bounds[i]))
            else:
                pending.extend((right, left))
        return sorted(candidates)

    def intersects_path(self, path, stats=None):
        p = np.asarray(path, dtype=np.float64)
        if (p.ndim != 2 or p.shape[1] != 2 or len(p) < 2
                or not np.isfinite(p).all()):
            raise ValueError("index requires a finite continuous polyline")
        if stats is not None:
            stats.update(nodes_visited=0, candidate_pairs=0, exact_checks=0)
        if self._root is None and not self._unpruned:
            return False
        lower = np.minimum(p[:-1], p[1:])
        upper = np.maximum(p[:-1], p[1:])
        if self._unpruned:
            relevant = range(len(p) - 1)
        else:
            root = self._root[0]
            relevant = np.flatnonzero(
                (lower[:, 0] <= root[2]) & (upper[:, 0] >= root[0])
                & (lower[:, 1] <= root[3]) & (upper[:, 1] >= root[1]))
        for i in relevant:
            box = (*lower[i], *upper[i])
            candidates = self._query(box, stats)
            if stats is not None:
                stats['candidate_pairs'] += len(candidates)
            for j in candidates:
                if stats is not None:
                    stats['exact_checks'] += 1
                if segment_intersects_polygon(
                        p[i], p[i + 1], self.polygons[j],
                        inflation_radius=self.inflation_radius):
                    return True
        return False


class SnapshotPolygonIndexCache:
    """One-entry cache for a *verified* complete snapshot, never authority.

    Call only after independently verifying the response hash and common frame.
    Every candidate must still query usable committed state. Changes in epoch,
    complete hash, frame or margin rebuild the index, including REMOVE/RELAX.
    Cache hits do not authorize motion, renew freshness or reuse an old verdict.
    """

    def __init__(self):
        self._key = None
        self._index = None

    def get(self, epoch, snapshot_hash, frame, polygons, inflation_radius):
        key = (int(epoch), str(snapshot_hash), str(frame),
               float(inflation_radius))
        if (key[0] < 0 or len(key[1]) != 64
                or any(c not in '0123456789abcdef' for c in key[1])
                or not key[2] or not math.isfinite(key[3]) or key[3] < 0):
            raise ValueError("invalid verified snapshot cache key")
        if key == self._key:
            return self._index, True
        index = PreparedPolygonIndex(polygons, key[3])
        self._key, self._index = key, index
        return index, False


def gate_full_path(path, rule_polys, occ_fn, map_bounds,
                   inflation_radius=0.0):
    """Gate a full executable path and return ``(accepted, reason)``."""
    path_array = np.asarray(path, dtype=np.float64)
    try:
        rho = float(inflation_radius)
    except (TypeError, ValueError):
        return False, "gate:bad-inflation"
    if not np.isfinite(rho) or rho < 0.0:
        return False, "gate:bad-inflation"
    xmin, ymin, xmax, ymax = map_bounds
    if (path_array.ndim != 2 or path_array.shape[0] < 4
            or path_array.shape[1] != 2 or not np.isfinite(path_array).all()):
        return False, "gate:bad-path"
    if bool(((path_array[:, 0] < xmin) | (path_array[:, 0] > xmax)
             | (path_array[:, 1] < ymin) | (path_array[:, 1] > ymax)).any()):
        return False, "gate:out-of-bounds"
    prepared = rule_polys if isinstance(rule_polys, PreparedPolygonIndex) else None
    if prepared is not None and prepared.inflation_radius != rho:
        return False, "gate:index-margin-mismatch"
    polygons = (prepared.polygons if prepared is not None else
                [np.asarray(poly, dtype=np.float64) for poly in rule_polys])
    if any(poly.ndim != 2 or poly.shape[0] < 3 or poly.shape[1] != 2
           or not np.isfinite(poly).all()
           for poly in polygons):
        return False, "gate:bad-rule"
    intersects = (prepared.intersects_path(path_array) if prepared is not None
                  else path_intersects_polygons(
                      path_array, polygons, inflation_radius=rho))
    if intersects:
        return False, "gate:KEEP_OUT-violation"
    if not _collision_free(path_array, occ_fn):
        return False, "gate:collision"
    return True, "gate:ok"
