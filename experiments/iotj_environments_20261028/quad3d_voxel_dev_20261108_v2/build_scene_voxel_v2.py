"""Build a new, source-bound surface voxel input; never overwrite old evidence."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

BASE = Path(__file__).resolve().parent
PROJECT = BASE.parent
from quad3d.mesh_reader import load_triangles
from quad3d.voxel_geometry import rasterize_triangles, band_mask
from quad3d.voxel_contract import sha, member, validate_grid, validate_manifest


def build(scene, output, z0=0., dz=.10, nz=18):
    scene, output = Path(scene).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    migration = json.loads((scene / 'migration_manifest.json').read_text())
    m = json.loads((scene / 'nav_v1/map.json').read_text())
    if nz <= 0 or dz <= 0 or not np.isfinite([z0, dz]).all():
        raise ValueError('invalid height grid')
    scene_sources = {name: sha(scene / name) for name in
                     ('migration_manifest.json', 'nav_v1/map.json', scene.name + '.sdf')}
    if scene_sources[scene.name + '.sdf'] != migration['world_sha256']:
        raise ValueError('world hash mismatch')
    code_sources = {str(path.relative_to(PROJECT)): sha(path) for path in
                    (Path(__file__), BASE / 'quad3d/voxel_geometry.py',
                     BASE / 'quad3d/mesh_reader.py')}
    cache = {}
    occ = np.zeros((nz, int(m['height']), int(m['width'])), dtype=bool)
    triangles = 0
    for row in migration['models']:
        asset = row['collision']
        if asset not in cache:
            path = member(scene / 'assets', asset)
            digest = sha(path)
            if digest != migration['asset_files_sha256'][asset]:
                raise ValueError('collision asset hash mismatch: ' + asset)
            scene_sources['assets/' + asset] = digest
            cache[asset] = np.asarray(load_triangles(path))
        tri = cache[asset] * np.asarray(row.get('geometry_scale', [1., 1., 1.]))
        tri = tri @ Rotation.from_euler('xyz', row['pose'][3:]).as_matrix().T
        tri = tri + np.asarray(row['pose'][:3])
        triangles += len(tri)
        # Retain all collision surfaces, including floor/low objects. No z<2cm removal.
        rasterize_triangles(occ, tri, m['origin'], m['resolution'], z0, dz)
    # Detect a source rewrite while the input was being generated.
    for name, digest in scene_sources.items():
        if sha(member(scene, name)) != digest:
            raise ValueError('scene changed during voxel build: ' + name)
    for name, digest in code_sources.items():
        if sha(PROJECT / name) != digest:
            raise ValueError('generator changed during voxel build: ' + name)
    grid_path = output / 'occupancy.npz'
    np.savez_compressed(grid_path, occ=occ, z0=z0, dz=dz,
                        res=float(m['resolution']), origin=np.asarray(m['origin'][:2]))
    record = dict(schema='rulesync.mesh_surface_voxel.v2', scene=scene.name,
                  scene_sources=scene_sources, code_sources=code_sources,
                  grid_file=grid_path.name, grid_sha256=sha(grid_path),
                  grid=validate_grid(grid_path, m), triangles=triangles,
                  surface_semantics='Closed triangle/cell intersection; solid interiors not inferred',
                  floor_policy='All collision faces retained; floor contact is occupied',
                  formal_denominator=False)
    mp = output / 'occupancy_manifest.json'
    mp.write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
    validate_manifest(mp, scene, PROJECT)
    cells = np.asarray(m['data']).reshape(m['height'], m['width'])
    comparison = {}
    for a, b in ((.03,.28),(.30,.80),(.80,1.50),(1.50,1.80)):
        band = band_mask(occ, z0, dz, a, b)
        comparison[f'[{a},{b})'] = dict(
            free2d_voxel_surface=int(((cells == 0) & band).sum()),
            blocked2d_no_voxel_surface=int(((cells == 100) & ~band).sum()))
    (output / 'exploratory_comparison.json').write_text(json.dumps(dict(
        scope='Raster representation comparison; not ground-truth classification error',
        occupancy_manifest_sha256=sha(mp), bands=comparison), indent=2)+'\n')
    return record


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    print(json.dumps(build(a.scene, a.output), indent=2))
