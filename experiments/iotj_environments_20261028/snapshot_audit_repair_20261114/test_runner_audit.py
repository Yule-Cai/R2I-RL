"""Exercise the actual repaired Mission.audit on captured evidence and a violation."""
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest

BASE = Path(__file__).resolve().parent
PROJECT = BASE.parent
REPO = PROJECT.parents[1]
sys.path.insert(0, str(REPO / 'rulesync3d'))
sys.path.insert(0, str(PROJECT / 'quad3d_snapshot_dev_20261114_v3'))
spec = importlib.util.spec_from_file_location('repaired_runner', PROJECT / 'quad3d_snapshot_dev_20261114_v3/scene_runner_quad3d_v1.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class ActualRunnerAuditTests(unittest.TestCase):
    def mission(self, directory):
        raw = REPO / 'output/snapshot_audit_repair_20261114/raw'
        binding = next(x for x in json.loads((raw / 'bindings.json').read_text())
                       if x['row']['scene'] == 'hssd_102816786' and x['row']['route'] == '03'
                       and x['row']['speed'] == .2 and x['row']['seed'] == '52601' and x['row']['occ'] == 'voxel')
        source = raw / binding['run']
        mission = runner.Mission.__new__(runner.Mission)
        mission.root = Path(directory)
        (mission.root / 'authority').mkdir()
        shutil.copyfile(source / 'authority/wal.jsonl', mission.root / 'authority/wal.jsonl')
        state = json.loads((source / 'authority/state.json').read_text())
        case = json.loads(next((source / 'source').glob('*.json')).read_text())
        mission.rules, mission.flight = case['rules'], case['flight']
        mission.authority = SimpleNamespace(epoch=state['epoch'], snapshot_hash=state['snapshot_hash'],
                                            volumes=state['volumes'], _hash=runner.authority3d.DurableVolumeAuthority._hash)
        mission.initial_admission = json.loads((source / 'admissions/initial_admitted.json').read_text())
        mission.execution = json.loads((source / 'admissions/recovery_admitted.json').read_text())
        mission.rows = [json.loads(line) for line in (source / 'trajectory.jsonl').read_text().splitlines()]
        mission.initial_path = [x['pose'] for x in mission.rows if x['epoch'] == 1]
        mission.recovery_path_points = [x['pose'] for x in mission.rows if x['epoch'] == 2]
        mission.occ = SimpleNamespace(physical_fn=lambda *point: False, occ_fn=lambda *point: False)
        mission.guard_counts = {}
        mission.guard = SimpleNamespace(events=[])
        mission.phase = lambda *args: None
        mission.event = lambda *args, **kwargs: None
        return mission

    def test_captured_retired_rule_crossing_passes_corrected_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = self.mission(directory)
            mission.audit()
            audit = json.loads((Path(directory) / 'audit.json').read_text())
            self.assertEqual(audit['audited_snapshot_volume_ids']['2'], ['R2_replacement'])
            self.assertEqual(audit['executed_epoch2_vs_committed_snapshot']['raw_violation_samples'], 0)

    def test_actual_active_rule_crossing_still_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = self.mission(directory)
            box = mission.authority.volumes[0]
            row = next(x for x in mission.rows if x['epoch'] == 2)
            row['pose'] = [(a + b) / 2 for a, b in zip(box['bmin'], box['bmax'])]
            with self.assertRaisesRegex(runner.RunnerError, 'KEEP_OUT'):
                mission.audit()

    def test_stale_execution_identity_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = self.mission(directory)
            mission.execution['snapshot_hash'] = 'wrong'
            with self.assertRaisesRegex(runner.RunnerError, 'snapshot identity'):
                mission.audit()


if __name__ == '__main__':
    unittest.main()
