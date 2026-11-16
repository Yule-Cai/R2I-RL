"""Independent SAT oracle and regression checks; no ROS, devices or simulator."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import sys
import unittest
from unittest.mock import patch

import numpy as np

from quad3d.voxel_geometry import rasterize_triangles, dilate_metric_box, band_mask
from quad3d.voxel_contract import sha, validate_manifest, load_occupancy, validate_grid
from quad3d.scene3d import VoxelOccupancy, build_grid, plan_path, gate_path

BASE = Path(__file__).resolve().parent


def sat_triangle_box(tri, low, high):
    """Separating-axis test, independently of the slab-clipping rasterizer."""
    half = (high - low) / 2
    v = tri - (high + low) / 2
    edges = [v[1]-v[0], v[2]-v[1], v[0]-v[2]]
    axes = [*np.eye(3), np.cross(edges[0], edges[1])]
    axes += [np.cross(e, axis) for e in edges for axis in np.eye(3)]
    for axis in axes:
        if np.linalg.norm(axis) < 1e-15:
            continue
        projections = v @ axis
        extent = half @ np.abs(axis)
        if projections.min() > extent + 1e-12 or projections.max() < -extent - 1e-12:
            return False
    return True


class VoxelTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def npz(self, raw=None, **kwargs):
        p = self.root / 'occ.npz'
        np.savez(p, occ=raw if raw is not None else np.zeros((21,21,21),bool),
                 z0=kwargs.get('z0',0.), dz=kwargs.get('dz',.10),
                 res=kwargs.get('res',.05), origin=kwargs.get('origin',[0.,0.]))
        return p

    def test_small_nonzero_face_is_not_lost(self):
        tri = [[[.001,.001,.80],[.009,.001,.80],[.001,.009,.80]]]
        occ = np.zeros((18,4,4),bool)
        rasterize_triangles(occ,tri,[0.,0.],.05,0.,.1)
        self.assertTrue(occ[7,0,0] and occ[8,0,0])

    def test_vertical_thin_and_sloped_against_sat(self):
        cases = [np.array([[.025,.01,.01],[.025,.19,.01],[.025,.01,.39]]),
                 np.array([[.001,.001,.01],[.199,.001,.39],[.001,.199,.21]]),
                 np.array([[.10,.10,.10],[.10,.10,.10],[.10,.10,.10]])]
        rng = np.random.default_rng(54217)
        cases += list(rng.uniform(-.05,.45,(35,3,3)))
        for tri in cases:
            actual = np.zeros((4,4,4),bool)
            rasterize_triangles(actual,[tri],[0.,0.],.05,0.,.1)
            expected = np.zeros_like(actual)
            for k,j,i in np.ndindex(actual.shape):
                lo = np.array([i*.05,j*.05,k*.1])
                expected[k,j,i] = sat_triangle_box(tri,lo,lo+[.05,.05,.1])
            np.testing.assert_array_equal(actual,expected)

    def test_anisotropic_margin_is_quantized_in_metres(self):
        raw = np.zeros((21,21,21),bool);raw[10,10,10]=True
        out = dilate_metric_box(raw,.15,(.1,.05,.05))
        # Inspect the isolated interior component, excluding occupied outer padding.
        z = np.flatnonzero(out[:,10,10][4:17])+4
        self.assertEqual((int(z.min()),int(z.max())),(8,12))
        self.assertFalse(out[7,10,10])
        self.assertTrue(out[10,10,13]);self.assertFalse(out[10,10,14])
        self.assertTrue(out[0,10,10])

    def test_zero_margin_preserves_raw_and_negative_rejected(self):
        raw=np.zeros((5,5,5),bool);raw[2,2,2]=True
        np.testing.assert_array_equal(dilate_metric_box(raw,0.,(.1,.05,.05)),raw)
        with self.assertRaises(ValueError):dilate_metric_box(raw,-1.,(.1,.05,.05))

    def test_rectangular_cartesian_projection(self):
        raw=np.zeros((21,21,21),bool);raw[10,7,5]=True
        v=VoxelOccupancy(self.npz(raw),0.,0.,2.1)
        result=v.cells([.275,.475],[.275,.375,.475])
        self.assertEqual(result.shape,(3,2));self.assertTrue(result[1,0])
        self.assertFalse(result[0,0]);self.assertFalse(result[1,1])

    def test_per_height_grid_not_projection(self):
        raw=np.zeros((21,21,21),bool);raw[10,7,5]=True
        v=VoxelOccupancy(self.npz(raw),0.,0.,2.1)
        grid,_=build_grid(v,(.25,.35,.9,.30,.40,1.2),.05)
        self.assertFalse(grid[0,0,0]);self.assertTrue(grid[2,0,0])
        self.assertTrue(v.occ_fn(-1.,0.,1.));self.assertTrue(v.occ_fn(0.,0.,3.))

    def test_half_open_band_does_not_include_next_layer(self):
        raw=np.zeros((18,1,1),bool);raw[15,0,0]=True
        self.assertFalse(band_mask(raw,0.,.1,.8,1.5)[0,0])
        self.assertTrue(band_mask(raw,0.,.1,1.5,1.8)[0,0])

    def test_voxel_obstacle_can_be_overflown_but_not_crossed(self):
        raw=np.zeros((18,160,160),bool);raw[:10,:,70:80]=True
        v=VoxelOccupancy(self.npz(raw),.15,.03,1.7)
        bounds=(0.,0.,.45,8.,8.,1.65)
        start,goal=[1.,3.,.9],[6.,3.,.9]
        through,_=gate_path([start,goal],v,[],0.,bounds)
        self.assertFalse(through)
        path,_=plan_path(v,bounds,.10,start,goal,[],0.)
        self.assertIsNotNone(path)
        accepted,reason=gate_path(path,v,[],0.,bounds)
        self.assertTrue(accepted,reason)
        self.assertGreater(max(p[2] for p in path),1.2)

    def test_bad_grid_and_coordinates_rejected(self):
        with self.assertRaises(ValueError):validate_grid(self.npz(np.zeros((2,2,2),int)))
        with self.assertRaises(ValueError):validate_grid(self.npz(dz=0.))
        with self.assertRaises(ValueError):validate_grid(self.npz(origin=[float('nan'),0.]))

    def test_explicit_mode_and_missing_voxel_fail_closed(self):
        flight=dict(occ_clearance_m=.3,z_floor_m=.03,z_max_m=1.7)
        m=dict(resolution=.05,width=2,height=2,origin=[0,0,0],data=[0]*4)
        for mode,mp in [(None,None),('voxel',None),('voxel',self.root/'missing.json'),
                        ('projected_2d',self.root/'x')]:
            with self.assertRaises((ValueError,FileNotFoundError)):
                load_occupancy(self.root,m,flight,mode,mp)

    def fixture_manifest(self):
        scene=self.root/'hssd_test';(scene/'assets').mkdir(parents=True)
        m=dict(resolution=.05,width=21,height=21,origin=[0,0,0],data=[0]*441)
        (scene/'nav_v1').mkdir();(scene/'nav_v1/map.json').write_text(json.dumps(m))
        (scene/'hssd_test.sdf').write_text('world');(scene/'assets/mesh.glb').write_bytes(b'mesh')
        migration=dict(world_sha256=sha(scene/'hssd_test.sdf'),models=[dict(collision='mesh.glb')],
                       asset_files_sha256={'mesh.glb':sha(scene/'assets/mesh.glb')})
        (scene/'migration_manifest.json').write_text(json.dumps(migration))
        grid=self.npz();names=['migration_manifest.json','nav_v1/map.json','hssd_test.sdf','assets/mesh.glb']
        code_names=['quad3d_voxel_dev_20261108_v2/build_scene_voxel_v2.py',
                    'quad3d_voxel_dev_20261108_v2/quad3d/voxel_geometry.py',
                    'quad3d_voxel_dev_20261108_v2/quad3d/mesh_reader.py']
        project=BASE.parent
        record=dict(schema='rulesync.mesh_surface_voxel.v2',scene=scene.name,
                    scene_sources={n:sha(scene/n) for n in names},
                    code_sources={n:sha(project/n) for n in code_names},grid_file=grid.name,
                    grid_sha256=sha(grid),grid=validate_grid(grid,m))
        mp=self.root/'manifest.json';mp.write_text(json.dumps(record))
        return scene,mp,record

    def test_manifest_validates_and_rejects_wrong_scene(self):
        scene,mp,r=self.fixture_manifest();validate_manifest(mp,scene,BASE.parent)
        r['scene']='wrong';mp.write_text(json.dumps(r))
        with self.assertRaises(ValueError):validate_manifest(mp,scene,BASE.parent)

    def test_mutated_voxel_and_mesh_rejected(self):
        scene,mp,r=self.fixture_manifest()
        (scene/'assets/mesh.glb').write_bytes(b'changed')
        with self.assertRaises(ValueError):validate_manifest(mp,scene,BASE.parent)
        (scene/'assets/mesh.glb').write_bytes(b'mesh')
        with (self.root/'occ.npz').open('ab') as f:f.write(b'changed')
        with self.assertRaises(ValueError):validate_manifest(mp,scene,BASE.parent)

    def test_incomplete_asset_and_code_bindings_rejected(self):
        scene,mp,r=self.fixture_manifest();del r['scene_sources']['assets/mesh.glb']
        mp.write_text(json.dumps(r))
        with self.assertRaises(ValueError):validate_manifest(mp,scene,BASE.parent)

    def test_launcher_passes_explicit_mode_and_defers_for_active_queue(self):
        # Syntax/contract checks inspect the actual launcher, not a second executor.
        text=(BASE/'run_hssd_quad3d_attempt.py').read_text()
        self.assertIn("RULESYNC_QUAD_OCCUPANCY_MODE=",text)
        self.assertIn("RULESYNC_QUAD_VOXEL_MANIFEST=",text)
        self.assertIn("another RuleSync container is active",text)
        self.assertIn('verify_source_pin(options.source_pin_sha)',text)

    def test_actual_launcher_refuses_active_formal_container(self):
        import run_hssd_quad3d_attempt as launch
        argv=['launch','--stage','platform_selftest','--source-pin-sha',sha(BASE/'source_pin_v2.json')]
        with patch.object(launch,'ROOT',BASE.parent), patch.object(sys,'argv',argv), \
                patch.object(launch,'running_containers',return_value=['rulesync-dm2970-abcdef']), \
                patch.object(launch,'container',side_effect=AssertionError('must not launch')):
            with self.assertRaisesRegex(RuntimeError,'another RuleSync container'):
                launch.main()

    def test_builder_and_input_snapshot_end_to_end_without_simulator(self):
        import build_scene_voxel_v2 as builder
        import scene_runner_quad3d_v1 as runner
        scene,_,_=self.fixture_manifest()
        migration=json.loads((scene/'migration_manifest.json').read_text())
        migration['models']=[dict(collision='mesh.glb',pose=[0]*6,geometry_scale=[1]*3)]
        (scene/'migration_manifest.json').write_text(json.dumps(migration))
        tri=np.array([[[.001,.001,.8],[.009,.001,.8],[.001,.009,.8]]])
        out=self.root/'built'
        with patch.object(builder,'load_triangles',return_value=tri):
            record=builder.build(scene,out)
        self.assertFalse(record['formal_denominator'])
        with np.load(out/'occupancy.npz') as data:
            self.assertTrue(data['occ'][8,0,0])
        mp=out/'occupancy_manifest.json'
        _,binding=validate_manifest(mp,scene,BASE.parent)
        obj=object.__new__(runner.Mission)
        from types import SimpleNamespace
        obj.args=SimpleNamespace(scene=scene);obj.root=self.root/'trial';obj.root.mkdir()
        obj.occupancy_source='voxel';obj.occupancy_binding=binding
        with patch.dict('os.environ',{'RULESYNC_QUAD_VOXEL_MANIFEST':str(mp)}):
            obj.copy_occupancy_inputs()
        self.assertEqual(sha(obj.root/'input_voxel/occupancy.npz'),record['grid_sha256'])
        with self.assertRaises(FileExistsError):builder.build(scene,out)


if __name__=='__main__':
    unittest.main(verbosity=2)
