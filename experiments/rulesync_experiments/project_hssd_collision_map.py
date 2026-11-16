"""Project the complete static HSSD world at the TurtleBot operating height."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial.transform import Rotation
import shapely

from prepare_hssd_scene import accessor, read_glb, sha


def load_triangles(path):
    doc, blob = read_glb(path)
    pieces = []

    def visit(index, parent):
        node = doc["nodes"][index]
        if "matrix" in node:
            transform = np.asarray(node["matrix"], dtype=float).reshape(4, 4, order="F")
        else:
            transform = np.eye(4)
            rotation = Rotation.from_quat(node.get("rotation", [0, 0, 0, 1])).as_matrix()
            transform[:3, :3] = rotation * node.get("scale", [1, 1, 1])
            transform[:3, 3] = node.get("translation", [0, 0, 0])
        placement = parent @ transform
        if "mesh" in node:
            for primitive in doc["meshes"][node["mesh"]]["primitives"]:
                assert primitive.get("mode", 4) == 4
                coords = accessor(doc, blob, primitive["attributes"]["POSITION"])
                indices = accessor(doc, blob, primitive["indices"]).reshape(-1, 3)
                world = coords @ placement[:3, :3].T + placement[:3, 3]
                pieces.append(world[indices])
        for child in node.get("children", []):
            visit(child, placement)

    for index in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        visit(index, np.eye(4))
    return np.concatenate(pieces)


def clip(vertices, z, keep_above):
    polygon = []
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        inside_a = a[2] >= z if keep_above else a[2] <= z
        inside_b = b[2] >= z if keep_above else b[2] <= z
        if inside_a:
            polygon.append(a)
        if inside_a != inside_b:
            polygon.append(a + (b - a) * ((z - a[2]) / (b[2] - a[2])))
    return np.asarray(polygon).reshape(-1, 3)


def obstacles(triangles, bottom, top):
    mask = (triangles[:, :, 2].max(axis=1) >= bottom) & (triangles[:, :, 2].min(axis=1) <= top)
    projected = []
    for triangle in triangles[mask]:
        clipped = clip(clip(triangle, bottom, True), top, False)
        if len(clipped):
            xy = clipped[:, :2]
            hull = shapely.convex_hull(shapely.MultiPoint(xy))
            if hull.geom_type == "Polygon" and hull.area < 1e-10:
                distances = np.sum((xy[:, None] - xy[None, :]) ** 2, axis=-1)
                a, b = np.unravel_index(distances.argmax(), distances.shape)
                hull = shapely.LineString([xy[a], xy[b]])
            projected.append(hull)
    return shapely.union_all(projected)


def main(scene, output):
    record = json.loads((scene / "migration_manifest.json").read_text())
    assert record["world_sha256"] == sha(scene / f"hssd_{record['scene']}.sdf")
    cached = {}
    pieces = []
    for row in record["models"]:
        asset = row["collision"]
        if asset not in cached:
            path = scene / "assets" / asset
            assert record["asset_files_sha256"][asset] == sha(path)
            cached[asset] = load_triangles(path)
        pose = row["pose"]
        rotation = Rotation.from_euler("xyz", pose[3:]).as_matrix()
        pieces.append((cached[asset] * row["geometry_scale"]) @ rotation.T + pose[:3])
    stage = pieces[0]
    ground = stage[np.max(np.abs(stage[:, :, 2]), axis=1) <= .025, :, :2]
    floor = shapely.union_all(shapely.polygons(ground))
    assert floor.area > 10
    blocked = obstacles(np.concatenate(pieces), .035, .28)

    res = .05
    xmin, ymin, xmax, ymax = floor.bounds
    origin = [(math.floor(v / res) - 2) * res for v in (xmin, ymin)]
    width, height = [math.ceil((v - o) / res) + 2 for v, o in zip((xmax, ymax), origin)]
    xx, yy = np.meshgrid(origin[0] + (np.arange(width) + .5) * res,
                         origin[1] + (np.arange(height) + .5) * res)
    cells = shapely.box(xx - res/2, yy - res/2, xx + res/2, yy + res/2)
    # Build spatial indices for the repeated predicates against the same mesh projections.
    shapely.prepare(floor)
    shapely.prepare(blocked)
    grid = np.where(shapely.covers(floor, cells), 0, -1)
    grid[shapely.intersects(blocked, cells)] = 100
    distances = ndimage.distance_transform_edt(grid == 0) * res
    # The runner subtracts half the cell diagonal from occupied-center distance.
    safe = distances >= .50 + res / math.sqrt(2)
    assert safe.any()
    points = np.column_stack((xx[safe], yy[safe]))
    first = np.unravel_index(distances.argmax(), distances.shape)
    selected = [np.array([xx[first], yy[first]])]
    for _ in range(2):
        separation = np.min([np.sum((points - p) ** 2, axis=1) for p in selected], axis=0)
        selected.append(points[np.argmax(separation)])

    output.mkdir(exist_ok=False)
    map_path = output / "map.json"
    map_path.write_text(json.dumps(dict(width=width, height=height, resolution=res,
                                      origin=[*origin, 0.], data=grid.ravel().tolist())) + "\n")
    view = np.flipud(np.where(grid < 0, 205, np.where(grid == 0, 254, 0)))
    Image.fromarray(view.astype("uint8")).save(output / "map.pgm")
    data = dict(world_sha256=record["world_sha256"], map_sha256=sha(map_path),
                resolution=res, floor_support_area_m2=floor.area, obstacle_slab_m=[.035, .28],
                triangles=sum(map(len, pieces)), safe_poses_xy=[p.tolist() for p in selected],
                cells={str(k): int((grid == k).sum()) for k in [-1, 0, 100]})
    (output / "projection.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    main(args.scene, args.output)
