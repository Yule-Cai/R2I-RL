"""Validate and archive the complete provenance of a DEV occupancy input."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def member(root, relative):
    root = Path(root).resolve()
    path = (root / relative).resolve()
    if path != root and root not in path.parents:
        raise ValueError('input path escapes its declared source root')
    return path


def validate_grid(path, expected_map=None):
    with np.load(path, allow_pickle=False) as data:
        raw = data['occ']
        z0, dz, res = [float(data[k]) for k in ('z0', 'dz', 'res')]
        origin = np.asarray(data['origin'], dtype=float)
        if raw.dtype != np.bool_ or raw.ndim != 3 or min(raw.shape) <= 0:
            raise ValueError('occupancy must be a nonempty boolean (z,y,x) grid')
        if origin.shape != (2,) or not np.isfinite([z0, dz, res, *origin]).all() or min(dz, res) <= 0:
            raise ValueError('invalid voxel grid coordinates')
        if expected_map is not None:
            m = expected_map
            if raw.shape[1:] != (int(m['height']), int(m['width'])):
                raise ValueError('voxel/map dimensions differ')
            if not math.isclose(res, float(m['resolution']), rel_tol=0, abs_tol=1e-12):
                raise ValueError('voxel/map resolutions differ')
            if not np.allclose(origin, m['origin'][:2], rtol=0, atol=1e-12):
                raise ValueError('voxel/map origins differ')
            if len(m['origin']) > 2 and abs(m['origin'][2]) > 1e-12:
                raise ValueError('rotated map is not supported')
        return dict(shape=list(raw.shape), z0=z0, dz=dz, res=res,
                    origin=origin.tolist())


def validate_manifest(manifest_path, scene_dir, project_root=None):
    scene_dir = Path(scene_dir).resolve()
    project_root = Path(project_root or Path(__file__).resolve().parents[2]).resolve()
    mp = Path(manifest_path).resolve()
    record = json.loads(mp.read_text())
    if record.get('schema') != 'rulesync.mesh_surface_voxel.v2' or record.get('scene') != scene_dir.name:
        raise ValueError('voxel schema or scene identity mismatch')
    required = {'migration_manifest.json', 'nav_v1/map.json', scene_dir.name + '.sdf'}
    if not required.issubset(record.get('scene_sources', {})):
        raise ValueError('incomplete scene source list')
    migration = json.loads((scene_dir / 'migration_manifest.json').read_text())
    collisions = {'assets/' + row['collision'] for row in migration['models']}
    if not collisions.issubset(record['scene_sources']):
        raise ValueError('missing collision asset binding')
    required_code = {'quad3d_voxel_dev_20261108_v2/build_scene_voxel_v2.py',
                     'quad3d_voxel_dev_20261108_v2/quad3d/voxel_geometry.py',
                     'quad3d_voxel_dev_20261108_v2/quad3d/mesh_reader.py'}
    if not required_code.issubset(record.get('code_sources', {})):
        raise ValueError('incomplete voxel generator source list')
    for relative, digest in record['scene_sources'].items():
        if sha(member(scene_dir, relative)) != digest:
            raise ValueError('scene source hash mismatch: ' + relative)
    for relative, digest in record['code_sources'].items():
        if sha(member(project_root, relative)) != digest:
            raise ValueError('voxel generator hash mismatch: ' + relative)
    if sha(scene_dir / (scene_dir.name + '.sdf')) != migration['world_sha256']:
        raise ValueError('migration world hash mismatch')
    for relative in collisions:
        if record['scene_sources'][relative] != migration['asset_files_sha256'][relative[len('assets/'):]]:
            raise ValueError('migration collision hash mismatch')
    grid_path = member(mp.parent, record['grid_file'])
    if sha(grid_path) != record['grid_sha256']:
        raise ValueError('voxel NPZ hash mismatch')
    m = json.loads((scene_dir / 'nav_v1/map.json').read_text())
    if validate_grid(grid_path, m) != record['grid']:
        raise ValueError('voxel grid metadata mismatch')
    return grid_path, dict(mode='voxel', manifest_sha256=sha(mp),
                           grid_sha256=record['grid_sha256'], manifest=record,
                           description='Per-height collision-mesh surface voxels; closed-cell '
                           'intersection; metric L-infinity dilation quantized per axis; '
                           'mesh solid interiors are not inferred')


def load_occupancy(scene_dir, map_record, flight, mode, manifest_path=None):
    from .scene3d import MapOccupancy, VoxelOccupancy
    args = (flight['occ_clearance_m'], flight['z_floor_m'], flight['z_max_m'])
    if mode == 'projected_2d':
        if manifest_path:
            raise ValueError('projected_2d must not carry a voxel manifest')
        return MapOccupancy(map_record, *args), dict(
            mode=mode, description='Ground-slab 2D projection lifted to flight band; '
            'not a per-height obstacle representation')
    if mode != 'voxel' or not manifest_path:
        raise ValueError('explicit occupancy mode and voxel manifest required')
    grid, binding = validate_manifest(manifest_path, scene_dir)
    shape = binding['manifest']['grid']
    if args[1] < shape['z0'] or args[2] > shape['z0'] + shape['shape'][0] * shape['dz']:
        raise ValueError('declared flight band exceeds voxel coverage')
    return VoxelOccupancy(grid, *args), binding
