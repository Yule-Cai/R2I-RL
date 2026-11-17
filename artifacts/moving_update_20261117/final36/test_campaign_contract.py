"""Check fault-window boundaries, paired settings and preregistered geometry."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'source'))
from channel_window_schedule_v1 import choose


class CampaignContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((ROOT/'manifest.json').read_text())

    def test_paired_faults_and_runtime_settings(self):
        groups = {}
        for run in self.manifest['runs']:
            groups.setdefault(run['pair_id'], []).append(run)
        self.assertEqual(len(groups),18)
        for pair in groups.values():
            self.assertEqual({r['arm'] for r in pair},{'full','geometry_only'})
            left,right = [copy.deepcopy(r) for r in pair]
            for name in ('id','order','arm','schedule','schedule_sha256'):
                left.pop(name); right.pop(name)
            self.assertEqual(left,right)
            configs = [json.loads((ROOT/r['schedule']).read_text()) for r in pair]
            for config in configs:
                config.pop('monitor_arm')
            self.assertEqual(configs[0],configs[1])

    def test_blackout_closed_open_boundaries(self):
        run = next(r for r in self.manifest['runs'] if r['condition']=='blackout')
        config = json.loads((ROOT/run['schedule']).read_text())
        anchor = dict(t_ns=10_000_000_000)
        for timestamp, expected in [(anchor['t_ns']-1,'forward'),
                (anchor['t_ns'],'suppress'),(anchor['t_ns']+2_500_000_000-1,'suppress'),
                (anchor['t_ns']+2_500_000_000,'forward')]:
            self.assertEqual(choose(config,7,timestamp,anchor)['action'],expected)
        self.assertEqual(choose(config,7,anchor['t_ns'],None)['action'],'forward')

    def test_clean_condition_never_suppresses(self):
        run = next(r for r in self.manifest['runs'] if r['condition']=='clean')
        config = json.loads((ROOT/run['schedule']).read_text())
        for offset in (-1,0,1,2_499_999_999,2_500_000_000):
            self.assertEqual(choose(config,3,10**10+offset,dict(t_ns=10**10))['action'],'forward')

    def test_identity_refresh_has_identical_polygons(self):
        for route in (1,2,3):
            case = json.loads((ROOT/f'inputs/supplement_route_{route:02}.json').read_text())
            self.assertEqual(case['expect_path_invalidated'],route!=1)
            if route == 1:
                self.assertEqual(case['initial_rule'],case['replacement_rule'])
                self.assertEqual(case['costmap_probes']['1'],case['costmap_probes']['2'])
            else:
                original = json.loads((ROOT/f'inputs/route_{route:02}.json').read_text())
                self.assertEqual(case['replacement_rule'],original['replacement_rule'])

    def test_frozen_inputs_have_not_changed(self):
        for path,digest in self.manifest['inputs_sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest,path)


if __name__ == '__main__':
    unittest.main()
