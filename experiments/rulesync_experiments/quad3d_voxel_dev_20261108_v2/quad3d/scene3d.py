"""3D geometry adapters for the B1 quadrotor DEV runner.

Everything that decides admissibility is imported from the installed
``rulesync3d`` package so the DEV runner is a thin, inspectable adapter:

* ``rulesync3d.gate_3d.gate_3d_path``   -- exact segment-vs-volume + swept voxel
  collision gate (the admission predicate).
* ``rulesync3d.planners_3d``            -- untrusted 3D candidate generators
  (26-connected voxel A*, line-of-sight smoothing).
* ``rulesync3d.ruleguard``              -- snapshot canonical hash, signed
  distance, and the epoch-disciplined RuleGuard monitor.

Declared simulation model:
* Explicit projected_2d or source-bound per-height mesh-surface voxels.
* Unknown space outside the grid is blocked. Mesh interiors are not inferred.
* KEEP_OUT volume checks are exact; occupancy checks use the existing swept
  sampling gate and do not acquire a new continuous collision guarantee.
"""
import copy
import math

import numpy as np

from rulesync3d import authorization as auth
from rulesync3d import gate_3d
from rulesync3d import planners_3d
from rulesync3d import ruleguard as rg


def load_map(map_path):
    import json
    from pathlib import Path
    record = json.loads(Path(map_path).read_text())
    for key in ("width", "height", "resolution", "origin", "data"):
        if key not in record:
            raise ValueError(f"map record missing {key}")
    return record


class MapOccupancy:
    """Ground-slab 2D occupancy lifted to a declared z band.

    ``blocked`` is True for occupied or unknown map cells, dilated by
    ``ceil(clearance_m / resolution)`` cells (square dilation, the same
    conservative style as the route prequalification).  Everything outside the
    map, below ``z_floor`` or above ``z_ceiling`` is blocked.
    """

    def __init__(self, map_record, clearance_m, z_floor, z_ceiling):
        self.resolution = float(map_record["resolution"])
        self.origin = [float(v) for v in map_record["origin"][:2]]
        self.width = int(map_record["width"])
        self.height = int(map_record["height"])
        data = np.asarray(map_record["data"], dtype=np.int16).reshape(
            self.height, self.width)
        raw = data != 0
        self.raw_blocked = raw.copy()
        cells = 0 if clearance_m <= 0 else int(
            math.ceil(float(clearance_m) / self.resolution))
        if cells > 0:
            padded = np.pad(raw, cells, mode="constant", constant_values=True)
            out = np.zeros_like(raw)
            for dy in range(0, 2 * cells + 1):
                for dx in range(0, 2 * cells + 1):
                    out |= padded[dy:dy + self.height, dx:dx + self.width]
            raw = out
        self.blocked = raw
        # physical collision margin: the quad body is 0.22 m, so a centre
        # within 0.12 m of raw occupied/unknown geometry is a collision
        body_cells = int(math.ceil(0.12 / self.resolution))
        padded = np.pad(self.raw_blocked, body_cells, mode="constant",
                        constant_values=True)
        physical = np.zeros_like(raw)
        for dy in range(0, 2 * body_cells + 1):
            for dx in range(0, 2 * body_cells + 1):
                physical |= padded[dy:dy + self.height,
                                   dx:dx + self.width]
        self.physical_blocked = physical
        self.clearance_m = float(clearance_m)
        self.z_floor = float(z_floor)
        self.z_ceiling = float(z_ceiling)
        self.xmin = self.origin[0]
        self.ymin = self.origin[1]
        self.xmax = self.origin[0] + self.width * self.resolution
        self.ymax = self.origin[1] + self.height * self.resolution

    def blocked_xy(self, x, y):
        ix = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        iy = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return bool(self.blocked[iy, ix])

    def occ_fn(self, x, y, z):
        if not (self.z_floor <= float(z) <= self.z_ceiling):
            return True
        return self.blocked_xy(x, y)

    def physical_xy(self, x, y):
        ix = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        iy = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return bool(self.physical_blocked[iy, ix])

    def physical_fn(self, x, y, z):
        if not (self.z_floor <= float(z) <= self.z_ceiling):
            return True
        return self.physical_xy(x, y)

    def cells(self, xs, ys):
        """Blocked mask on the (ys, xs) grid of sample points."""
        ix = np.floor((np.asarray(xs) - self.origin[0])
                      / self.resolution).astype(int)
        iy = np.floor((np.asarray(ys) - self.origin[1])
                      / self.resolution).astype(int)
        x_ok = (ix >= 0) & (ix < self.width)
        y_ok = (iy >= 0) & (iy < self.height)
        ixc = np.clip(ix, 0, self.width - 1)
        iyc = np.clip(iy, 0, self.height - 1)
        out = self.blocked[np.ix_(iyc, ixc)].copy()
        out[~np.outer(y_ok, x_ok)] = True
        return out



class VoxelOccupancy:
    """Per-height voxel occupancy (npz from build_scene_voxel_v1.py).

    Same interface as MapOccupancy for the quad3d runner, but blocked state is
    resolved per z layer instead of lifting the ground projection.  Dilation is
    a metric L-infinity box, quantized separately for XY and Z spacing.
    """

    def __init__(self, npz_path, clearance_m, z_floor, z_ceiling):
        from .voxel_contract import validate_grid
        from .voxel_geometry import dilate_metric_box
        meta = validate_grid(npz_path)
        with np.load(npz_path, allow_pickle=False) as data:
            raw = data["occ"].copy()
        self.z0, self.dz = meta['z0'], meta['dz']
        self.resolution, self.origin = meta['res'], meta['origin']
        self.nz, self.height, self.width = raw.shape
        self.raw_blocked = raw.copy()
        spacing = (self.dz, self.resolution, self.resolution)
        self.blocked_vox = dilate_metric_box(raw, clearance_m, spacing)
        # Preserves the pre-existing conservative .12 m centre-body margin,
        # now quantized in metres independently on all three axes.
        self.physical_vox = dilate_metric_box(raw, .12, spacing)
        self.clearance_m = float(clearance_m)
        self.z_floor = float(z_floor)
        self.z_ceiling = float(z_ceiling)
        self.xmin = self.origin[0]
        self.ymin = self.origin[1]
        self.xmax = self.origin[0] + self.width * self.resolution
        self.ymax = self.origin[1] + self.height * self.resolution
        self.voxel_path = str(npz_path)

    def _idx(self, x, y, z):
        ix = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        iy = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        kz = int(math.floor((float(z) - self.z0) / self.dz))
        if ix < 0 or iy < 0 or kz < 0 or ix >= self.width or iy >= self.height or kz >= self.nz:
            return None
        return kz, iy, ix

    def blocked_xy(self, x, y):
        ix = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        iy = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return bool(self.blocked_vox[:, iy, ix].any())

    def occ_fn(self, x, y, z):
        if not (self.z_floor <= float(z) <= self.z_ceiling):
            return True
        c = self._idx(x, y, z)
        if c is None:
            return True
        return bool(self.blocked_vox[c])

    def physical_xy(self, x, y):
        ix = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        iy = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return bool(self.physical_vox[:, iy, ix].any())

    def physical_fn(self, x, y, z):
        if not (self.z_floor <= float(z) <= self.z_ceiling):
            return True
        c = self._idx(x, y, z)
        if c is None:
            return True
        return bool(self.physical_vox[c])

    def cells(self, xs, ys):
        ix = np.floor((np.asarray(xs) - self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((np.asarray(ys) - self.origin[1]) / self.resolution).astype(int)
        x_ok = (ix >= 0) & (ix < self.width)
        y_ok = (iy >= 0) & (iy < self.height)
        ixc = np.clip(ix, 0, self.width - 1)
        iyc = np.clip(iy, 0, self.height - 1)
        out = self.blocked_vox[np.ix_(np.arange(self.nz), iyc, ixc)].any(axis=0).copy()
        out[~np.outer(y_ok, x_ok)] = True
        return out


def extrude_polygon(polygon_xy, zmin, zmax):
    """Axis-aligned extrusion of a 2D rule polygon to a declared z band."""
    pts = np.asarray(polygon_xy, dtype=np.float64)
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    return {"geometry_type": 0,
            "bmin": [float(lo[0]), float(lo[1]), float(zmin)],
            "bmax": [float(hi[0]), float(hi[1]), float(zmax)]}


def inflate_volumes(volumes, radius):
    out = []
    for volume in volumes:
        out.append({"geometry_type": 0,
                    "bmin": [float(v) - float(radius)
                             for v in volume["bmin"]],
                    "bmax": [float(v) + float(radius)
                             for v in volume["bmax"]]})
    return out


def planning_bounds(occ, z_plan_min, z_plan_max, inset=0.0):
    return (occ.xmin + inset, occ.ymin + inset, float(z_plan_min),
            occ.xmax - inset, occ.ymax - inset, float(z_plan_max))


def build_grid(occ, bounds, res, keepout_boxes=(), keepout_margin=0.0):
    """3D occupancy grid for the untrusted planner (True = blocked)."""
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    nz = int(math.ceil((zmax - zmin) / res))
    ny = int(math.ceil((ymax - ymin) / res))
    nx = int(math.ceil((xmax - xmin) / res))
    xs = xmin + (np.arange(nx) + 0.5) * res
    ys = ymin + (np.arange(ny) + 0.5) * res
    zs = zmin + (np.arange(nz) + 0.5) * res
    if isinstance(occ, VoxelOccupancy):
        ixv = np.floor((xs - occ.origin[0]) / occ.resolution).astype(int)
        iyv = np.floor((ys - occ.origin[1]) / occ.resolution).astype(int)
        kzv = np.floor((zs - occ.z0) / occ.dz).astype(int)
        x_ok = (ixv >= 0) & (ixv < occ.width)
        y_ok = (iyv >= 0) & (iyv < occ.height)
        z_ok = (kzv >= 0) & (kzv < occ.nz)
        ixc = np.clip(ixv, 0, occ.width - 1)
        iyc = np.clip(iyv, 0, occ.height - 1)
        kzc = np.clip(kzv, 0, occ.nz - 1)
        grid = occ.blocked_vox[np.ix_(kzc, iyc, ixc)]
        outside = (~z_ok[:, None, None]) | (~y_ok[None, :, None]) | (~x_ok[None, None, :])
        grid = grid | outside
        grid[(zs < occ.z_floor) | (zs > occ.z_ceiling), :, :] = True
    else:
        mask2d = occ.cells(xs, ys)
        grid = np.repeat(mask2d[None, :, :], nz, axis=0)
        outside = ((zs < occ.z_floor) | (zs > occ.z_ceiling))
        grid[outside, :, :] = True
    if keepout_boxes:
        boxes = planners_3d.keepouts_from_geoms(
            [{"type": 0,
              "center": [(b["bmin"][i] + b["bmax"][i]) / 2 for i in range(3)],
              "dims": [b["bmax"][i] - b["bmin"][i] for i in range(3)]}
             for b in keepout_boxes], margin=keepout_margin)
        blocked, _ = planners_3d.voxelize_bounds(boxes, res, bounds,
                                                 margin=0.0)
        grid |= blocked
    return grid, (xmin, ymin, zmin)


_NEIGHBOURS_26 = [
    (dx, dy, dz)
    for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
    if not (dx == dy == dz == 0)]


def astar_3d(grid, origin, res, start, goal, max_expand=500000):
    """26-connected voxel A* with corner-cutting prevention.

    ``rulesync3d.planners_3d.voxel_astar`` allows a diagonal move whose
    linear interpolation leaves the free-cell union (it would cross a blocked
    corner), which the exact gate then rejects.  This DEV planner is
    gate-consistent: a diagonal move is only accepted when every cell of the
    2x2(x2) cell block it spans is free, so each straight segment stays inside
    free cells and the gate's swept sampling cannot find a blocked point.

    Returns world-frame waypoints (exact start/goal endpoints) or None.
    """
    import heapq
    nz, ny, nx = grid.shape
    ox, oy, oz = origin

    def cell(point):
        return (int((point[0] - ox) / res), int((point[1] - oy) / res),
                int((point[2] - oz) / res))

    def world(c):
        return (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res,
                oz + (c[2] + 0.5) * res)

    start_cell, goal_cell = cell(start), cell(goal)
    for c in (start_cell, goal_cell):
        if not (0 <= c[0] < nx and 0 <= c[1] < ny and 0 <= c[2] < nz):
            return None
    if grid[start_cell[2], start_cell[1], start_cell[0]] or \
            grid[goal_cell[2], goal_cell[1], goal_cell[0]]:
        return None
    goal_world = world(goal_cell)

    def heuristic(c):
        w = world(c)
        return math.sqrt((w[0] - goal_world[0]) ** 2
                         + (w[1] - goal_world[1]) ** 2
                         + (w[2] - goal_world[2]) ** 2)

    def free(c):
        return (0 <= c[0] < nx and 0 <= c[1] < ny and 0 <= c[2] < nz
                and not grid[c[2], c[1], c[0]])

    def block_free(c, dx, dy, dz):
        for ax in ({0, dx}):
            for ay in ({0, dy}):
                for az in ({0, dz}):
                    if ax == 0 and ay == 0 and az == 0:
                        continue
                    if not free((c[0] + ax, c[1] + ay, c[2] + az)):
                        return False
        return True

    open_heap = [(heuristic(start_cell), 0.0, start_cell)]
    came = {start_cell: None}
    cost = {start_cell: 0.0}
    closed = set()
    expanded = 0
    while open_heap and expanded < max_expand:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == goal_cell:
            cells = [current]
            while came[cells[-1]] is not None:
                cells.append(came[cells[-1]])
            cells.reverse()
            points = [tuple(float(v) for v in start)]
            points += [world(c) for c in cells[1:-1]]
            points.append(tuple(float(v) for v in goal))
            return [list(p) for p in points]
        for dx, dy, dz in _NEIGHBOURS_26:
            neighbour = (current[0] + dx, current[1] + dy,
                         current[2] + dz)
            if neighbour in closed or not free(neighbour):
                continue
            if (dx or dy or dz) and not block_free(current, dx, dy, dz):
                continue
            step = math.sqrt((dx * res) ** 2 + (dy * res) ** 2
                             + (dz * res) ** 2)
            new_cost = cost[current] + step
            if neighbour not in cost or new_cost < cost[neighbour] - 1e-9:
                cost[neighbour] = new_cost
                came[neighbour] = current
                heapq.heappush(open_heap,
                               (new_cost + heuristic(neighbour), new_cost,
                                neighbour))
    return None


def plan_path(occ, bounds, res, start, goal, volumes, inflation, seed=0):
    """Gate-consistent 26-connected voxel A* against occupancy + volumes."""
    del seed
    grid, origin = build_grid(occ, bounds, res, volumes, inflation)
    path = astar_3d(grid, origin, res, start, goal)
    if path is None:
        return None, "gate_consistent_astar_26conn"
    return path, "gate_consistent_astar_26conn"


def smooth_path(path, occ, volumes, inflation, bounds=None, step=0.02):
    """Greedy line-of-sight shortcutting using the EXACT admission predicate.

    A shortcut segment is accepted only if ``gate_3d.gate_3d_path`` accepts it
    (same swept occupancy sampling and exact segment-vs-inflated-AABB test the
    final gate uses), so concatenating accepted shortcuts cannot introduce a
    segment the gate would later reject.  ``step`` is kept for API symmetry and
    not used for the validity decision.
    """
    del step
    points = [[float(value) for value in point] for point in path]
    if len(points) <= 2:
        return points
    if bounds is None:
        bounds = (occ.xmin, occ.ymin, occ.z_floor, occ.xmax, occ.ymax,
                  occ.z_ceiling)
    gate_vols = inflate_volumes(volumes, inflation) if volumes else []

    def segment_free(a, b):
        ok, _ = gate_3d.gate_3d_path(
            np.asarray([a, b], dtype=np.float64), gate_vols, occ.occ_fn,
            bounds, min_len=0.0, r_body=inflation,
            occ_res=occ.resolution)
        return bool(ok)

    out = [points[0]]
    index = 0
    while index < len(points) - 1:
        candidate = len(points) - 1
        while candidate > index + 1 and not segment_free(
                points[index], points[candidate]):
            candidate -= 1
        out.append(points[candidate])
        index = candidate
    return out


def gate_path(path, occ, volumes, inflation, bounds):
    """Admission predicate: exact 3D rule volumes + swept occupancy."""
    gate_vols = inflate_volumes(volumes, inflation) if volumes else []
    return gate_3d.gate_3d_path(
        np.asarray(path, dtype=np.float64), gate_vols, occ.occ_fn, bounds,
        min_len=0.2, r_body=inflation, occ_res=occ.resolution)


def dedupe(path, eps=1e-9):
    out = []
    for point in path:
        point = [float(v) for v in point]
        if not out or math.dist(out[-1], point) > eps:
            out.append(point)
    return out


def build_flight_path(route_xy, spawn_z, goal_z, cruise_z):
    """Ground takeoff -> xy route at cruise z -> ground descent at goal."""
    if len(route_xy) < 2:
        raise ValueError("route needs at least two xy points")
    sx, sy = float(route_xy[0][0]), float(route_xy[0][1])
    gx, gy = float(route_xy[-1][0]), float(route_xy[-1][1])
    path = [[sx, sy, float(spawn_z)]]
    if abs(cruise_z - spawn_z) > 1e-6:
        path.append([sx, sy, float(cruise_z)])
    for x, y in route_xy[1:]:
        path.append([float(x), float(y), float(cruise_z)])
    if abs(goal_z - cruise_z) > 1e-6:
        path.append([gx, gy, float(goal_z)])
    return dedupe(path)


def densify(points, spacing=0.10):
    """Insert intermediate waypoints so pure pursuit can step continuously."""
    out = [[float(value) for value in points[0]]]
    for a, b in zip(points[:-1], points[1:]):
        length = math.dist(a, b)
        count = max(1, int(math.ceil(length / float(spacing))))
        for k in range(1, count + 1):
            t = k / count
            out.append([a[i] + (b[i] - a[i]) * t for i in range(3)])
    return out


def initial_flight_path(occ, bounds, bounds_gate, spawn_xy, goal_xy,
                        cruise_z, z_spawn, volumes, inflation, res=None,
                        seed=0, reserve=0.05):
    """Re-plan and gate the initial 3D path under the initial rule snapshot.

    The prepared 2D route is provenance/qualification input; the flown 3D path
    is re-planned for the declared flight band with the same gate-consistent
    planner and exact admission predicate the runner uses.  Returns
    ``(path, planner, gate_ok, gate_reason)``.
    """
    res = float(res or 0.10)
    start = [float(spawn_xy[0]), float(spawn_xy[1]), float(cruise_z)]
    goal = [float(goal_xy[0]), float(goal_xy[1]), float(cruise_z)]
    # two-tier margin: plan/smooth with the tracking reserve, gate nominal
    raw, planner = plan_path(occ, bounds, res, start, goal, volumes,
                             float(inflation) + float(reserve), seed=seed)
    if raw is None:
        return None, planner, False, "no-3d-path-under-initial-snapshot"
    smoothed = smooth_path(raw, occ, volumes,
                           float(inflation) + float(reserve),
                           bounds=bounds_gate)
    path = dedupe([[float(spawn_xy[0]), float(spawn_xy[1]),
                    float(z_spawn)]] + smoothed
                  + [[float(goal_xy[0]), float(goal_xy[1]),
                      float(z_spawn)]])
    path = densify(path)
    ok, reason = gate_path(path, occ, volumes, inflation, bounds_gate)
    return path, planner, bool(ok), reason


def path_object(points, frame_id="map", stamp_ns=0):
    """nav_msgs/Path-like duck object for ``rulesync3d.authorization``."""
    from types import SimpleNamespace
    stamp = SimpleNamespace(sec=int(stamp_ns) // 10 ** 9,
                            nanosec=int(stamp_ns) % 10 ** 9)
    header = SimpleNamespace(frame_id=str(frame_id), stamp=stamp)
    poses = []
    for point in points:
        poses.append(SimpleNamespace(
            header=copy.deepcopy(header),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=float(point[0]),
                                         y=float(point[1]),
                                         z=float(point[2])),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    return SimpleNamespace(header=header, poses=poses)


def path_digest(points, stamp_ns, frame_id="map"):
    return auth.path_digest(path_object(points, frame_id, stamp_ns))


def clearance_profile(points, volumes, inflation):
    """Min signed distance of every sampled point to the raw/inflated volumes.

    Returns (min_raw, min_inflated, raw_violation_samples,
    margin_violation_samples, samples).  Ground-truth poses are used by the
    caller; this is the independent post-hoc check, not the monitor.
    """
    if not volumes:
        return None
    raw = [{"geometry_type": 0, "bmin": [float(v) for v in volume["bmin"]],
            "bmax": [float(v) for v in volume["bmax"]]} for volume in volumes]
    inflated = inflate_volumes(volumes, inflation)
    min_raw = math.inf
    min_infl = math.inf
    raw_viol = 0
    margin_viol = 0
    for point in points:
        raw_c = rg.min_clearance_3d(tuple(point), raw, 0.0)
        inf_c = rg.min_clearance_3d(tuple(point), inflated, 0.0)
        min_raw = min(min_raw, raw_c)
        min_infl = min(min_infl, inf_c)
        raw_viol += int(raw_c < 0.0)
        margin_viol += int(inf_c < 0.0)
    return {"min_raw_clearance_m": min_raw,
            "min_inflated_clearance_m": min_infl,
            "raw_violation_samples": raw_viol,
            "margin_violation_samples": margin_viol,
            "samples": len(points)}


def occupancy_profile(points, occ):
    return sum(1 for point in points if occ.occ_fn(*point))


def physical_penetration_profile(points, occ):
    """Executed-track samples within the body margin of raw geometry."""
    return sum(1 for point in points if occ.physical_fn(*point))


def resample(points, spacing=0.05):
    """Dense polyline resampling so audits do not skip short excursions."""
    out = []
    for a, b in zip(points[:-1], points[1:]):
        length = math.dist(a, b)
        n = max(1, int(math.ceil(length / spacing)))
        for i in range(n):
            t = i / n
            out.append([a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t,
                        a[2] + (b[2] - a[2]) * t])
    out.append([float(v) for v in points[-1]])
    return out
