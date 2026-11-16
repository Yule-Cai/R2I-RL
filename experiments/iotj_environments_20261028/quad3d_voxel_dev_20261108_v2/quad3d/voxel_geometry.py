"""Conservative surface voxelisation; closed-cell contact is occupied.

Clip each collision triangle to each intersected z slab, then intersect its
XY convex projection with CLOSED raster cells. This retains thin, vertical,
sloping and sub-cell faces. Mesh interiors are not inferred to be solids.
"""
import math

import numpy as np
import shapely


def clip_z(vertices, plane, above):
    result = []
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        ia = a[2] >= plane if above else a[2] <= plane
        ib = b[2] >= plane if above else b[2] <= plane
        if ia:
            result.append(a)
        if ia != ib:
            result.append(a + (b - a) * ((plane - a[2]) / (b[2] - a[2])))
    return np.asarray(result, dtype=float).reshape(-1, 3)


def rasterize_triangles(occ, triangles, origin, res, z0, dz):
    """OR surfaces into a (z,y,x) grid; contact at a cell face counts twice.

    A 1e-10 m geometric tolerance retains floating-point boundary contact.
    No area/vertical-face heuristic is used.
    """
    nz, ny, nx = occ.shape
    ox, oy = origin[:2]
    eps = 1e-10
    triangles = np.asarray(triangles, dtype=float).reshape(-1, 3, 3)
    if not np.isfinite(triangles).all():
        raise ValueError('non-finite collision mesh')
    for triangle in triangles:
        k0 = max(0, math.floor((triangle[:, 2].min() - z0 - eps) / dz))
        k1 = min(nz - 1, math.floor((triangle[:, 2].max() - z0 + eps) / dz))
        for k in range(k0, k1 + 1):
            poly = clip_z(clip_z(triangle, z0 + k * dz - eps, True),
                          z0 + (k + 1) * dz + eps, False)
            if not len(poly):
                continue
            geom = shapely.convex_hull(shapely.MultiPoint(poly[:, :2]))
            xlo, ylo, xhi, yhi = geom.bounds
            i0 = max(0, math.floor((xlo - ox - eps) / res))
            i1 = min(nx - 1, math.floor((xhi - ox + eps) / res))
            j0 = max(0, math.floor((ylo - oy - eps) / res))
            j1 = min(ny - 1, math.floor((yhi - oy + eps) / res))
            if i1 < i0 or j1 < j0:
                continue
            # Bound temporary geometry allocations for large floor triangles.
            rows_per_chunk = max(1, 16000 // (i1 - i0 + 1))
            for first in range(j0, j1 + 1, rows_per_chunk):
                last = min(j1 + 1, first + rows_per_chunk)
                xx, yy = np.meshgrid(ox + np.arange(i0, i1 + 1) * res,
                                     oy + np.arange(first, last) * res)
                boxes = shapely.box(xx, yy, xx + res, yy + res)
                occ[k, first:last, i0:i1 + 1] |= shapely.dwithin(geom, boxes, eps)
    return occ


def dilate_metric_box(raw, radius, spacing):
    """Conservative L-infinity margin, quantized independently per axis.

    `spacing` follows array order (dz,dy,dx). Each extra centre offset is
    ceil(radius/spacing)*spacing, hence less than one cell over the requested
    radius on each axis. Unknown space outside the stored grid is occupied.
    """
    from scipy.ndimage import maximum_filter
    radius = float(radius)
    if not math.isfinite(radius) or radius < 0:
        raise ValueError('margin must be finite and nonnegative')
    sizes = [2 * int(math.ceil(max(0., radius / s - 1e-12))) + 1
             for s in spacing]
    return maximum_filter(np.asarray(raw, dtype=bool), size=sizes,
                          mode='constant', cval=True).astype(bool)


def band_mask(occ, z0, dz, lower, upper):
    """Union cells overlapping the requested half-open interval [lower,upper)."""
    if not (np.isfinite([z0, dz, lower, upper]).all() and dz > 0 and lower < upper):
        raise ValueError('invalid height band')
    starts = z0 + np.arange(occ.shape[0]) * dz
    active = (starts < upper - 1e-10) & (starts + dz > lower + 1e-10)
    return np.asarray(occ, dtype=bool)[active].any(axis=0)
