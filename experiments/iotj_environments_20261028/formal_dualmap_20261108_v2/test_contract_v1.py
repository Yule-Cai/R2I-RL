import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from contract_v1 import COHORT,atomic,check_files,inspect,load,reconcile,selected,sha


class Integrity(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.row=dict(key='full|scene|route|0.20|52501|bound',id='identity',arm='full',scene='scene',case_id='case',seed=52501,speed=.2,phase='formal')
        self.m=dict(image_id='image',cases={'case':dict(path='case.json',sha256='case-sha')},runners={'rulesync':'runner.py'},sources_sha256={'runner.py':'runner-sha'},scenes={})
    def tearDown(self):self.tmp.cleanup()
    def launch(self):
        out=self.root/'out';out.mkdir()
        atomic(out/'launch.json',dict(manifest_sha256='digest',key=self.row['key'],phase='formal',row=self.row,image_id='image',
            arguments=dict(seed=52501,speed=.2,arm='full',scene_id='scene'),input_sha256={}))
        return out
    def complete(self,result='FAIL'):
        out=self.launch();trial=out/'position_1';(trial/'source').mkdir(parents=True)
        (trial/'source/runner.py').write_text('science source')
        self.m['sources_sha256']['runner.py']=sha(trial/'source/runner.py')
        raw=dict(result=result,formal_denominator=True,cohort_id=COHORT,source_unchanged=True)
        atomic(trial/'result.json',raw)
        atomic(trial/'manifest.json',dict(formal_denominator=True,cohort_id=COHORT,runtime_binding=dict(manifest_sha256='digest',key=self.row['key']),
            seed=52501,requested_speed_mps=.2,runner_sha256=self.m['sources_sha256']['runner.py'],task_source_sha256='case-sha',baseline_arm='full',monitor_arm='full',gate_rho_m=.3))
        atomic(out/'closed.json',dict(result=raw,returncode=0,inputs_unchanged=True,manifest_sha256='digest'))
        return out
    def test_absent_is_not_a_failure(self):self.assertEqual(inspect(self.root/'absent',self.m,'digest',self.row,'formal')['kind'],'absent')
    def test_nonempty_unbound_is_preserved_and_blocked(self):
        p=self.root/'out';p.mkdir();f=p/'important.log';f.write_text('preserve')
        with self.assertRaises(ValueError):inspect(p,self.m,'digest',self.row,'formal')
        self.assertEqual(f.read_text(),'preserve')
    def test_identified_interruption_never_becomes_absent(self):
        out=self.launch();(out/'prefix.jsonl').write_text('{"observed":true}\n')
        r=inspect(out,self.m,'digest',self.row,'formal');self.assertEqual(r['kind'],'executed_interrupted');self.assertFalse(r['passed'])
        self.assertTrue((out/'prefix.jsonl').exists())
    def test_wrong_manifest_cannot_reuse_an_output(self):
        out=self.launch()
        with self.assertRaises(AssertionError):inspect(out,self.m,'other',self.row,'formal')
    def test_snapshot_mutation_blocks_reuse(self):
        out=self.launch();f=out/'input_snapshot/input';f.parent.mkdir();f.write_text('expected')
        l=json.loads((out/'launch.json').read_text());l['input_sha256']={'input':sha(f)};atomic(out/'launch.json',l);f.write_text('changed')
        with self.assertRaises(AssertionError):inspect(out,self.m,'digest',self.row,'formal')
    def test_raw_failure_is_not_overridden_by_wrapper_exit_zero(self):
        out=self.complete();r=inspect(out,self.m,'digest',self.row,'formal');self.assertFalse(r['passed']);self.assertEqual(r['kind'],'verified_archive')
    def test_formal_identity_is_required_inside_scientific_result(self):
        out=self.complete();p=out/'position_1/manifest.json';r=json.loads(p.read_text());r['formal_denominator']=False;atomic(p,r)
        with self.assertRaises(AssertionError):inspect(out,self.m,'digest',self.row,'formal')
    def test_closed_without_result_remains_failure(self):
        out=self.launch();atomic(out/'closed.json',dict(result=None,inputs_unchanged=True,manifest_sha256='digest'))
        self.assertEqual(inspect(out,self.m,'digest',self.row,'formal')['kind'],'executed_no_result')
    def test_closed_failure_cannot_silently_turn_into_pass(self):
        old=dict(status='done',kind='verified_archive',passed=False);new=dict(old,passed=True)
        with self.assertRaises(AssertionError):reconcile(old,new)
    def test_preflight_key_cannot_enter_formal_rows(self):
        m=dict(rows=[self.row],preflight_rows=[dict(self.row,key='preflight')])
        with self.assertRaises(AssertionError):selected(m,'preflight','formal')
    def test_global_file_change_is_rejected(self):
        f=self.root/'source';f.write_text('one');m=dict(sources_sha256={'source':sha(f)},cases={},scenes={});f.write_text('two')
        with self.assertRaises(AssertionError):check_files(self.root,m)
    def test_manifest_hash_is_verified_before_loading(self):
        p=self.root/'manifest.json';p.write_text('{}')
        with self.assertRaises(AssertionError):load(p,'wrong')
    def test_control_and_cleanup_try_blocks_unchanged(self):
        here=Path(__file__).resolve().parent;prior=here.parent/'localization_repair_20261108'
        full=prior/'common_arm_v1/scene_runner_rulesync_dualmap_dev_v1.py';native=prior/'common_arm_v2/scene_runner_native_dualmap_dev_v2.py'
        if not full.exists():
            prior=here.parents[1]/'input_requalification_20261108_v1'
            full=prior/'common_arm_v1/tools/scene_runner_rulesync_dualmap_dev_v1.py';native=prior/'common_arm_v2/scene_runner_native_dualmap_dev_v2.py'
        pairs=[(full,here/'scene_runner_rulesync_formal_v1.py'),(native,here/'scene_runner_native_formal_v1.py')]
        for before,after in pairs:
            def tries(p):
                run=next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='run')
                return [ast.dump(n,include_attributes=False) for n in run.body if isinstance(n,ast.Try)]
            self.assertEqual(tries(before),tries(after))
    def test_native_trigger_capture_survives_in_formal_variant(self):
        here=Path(__file__).resolve().parent;prior=here.parent/'localization_repair_20261108/common_arm_v2/scene_runner_native_dualmap_dev_v2.py'
        if not prior.exists():prior=here.parents[1]/'input_requalification_20261108_v1/common_arm_v2/scene_runner_native_dualmap_dev_v2.py'
        def fire(p):return ast.dump(next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='fire_trigger'),include_attributes=False)
        self.assertEqual(fire(prior),fire(here/'scene_runner_native_formal_v1.py'))
    def test_native_records_zero_and_nonzero_commands_without_changing_counts(self):
        source=Path(__file__).with_name('scene_runner_native_formal_v1.py')
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='NativeProbe')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='on_cmd')
        env={'stamp_ns':lambda s:s.sec*10**9+s.nanosec}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),env)
        events=[];node=SimpleNamespace(nonzero={'/cmd_vel':0},event=lambda kind,**kw:events.append(dict(kind=kind,**kw)))
        for v,w in [(0.,0.),(.2,0.),(0.,.5)]:
            msg=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=v),angular=SimpleNamespace(z=w)),header=SimpleNamespace(stamp=SimpleNamespace(sec=3,nanosec=7)))
            env['on_cmd'](node,'/cmd_vel',msg)
        self.assertEqual(node.nonzero['/cmd_vel'],2)
        self.assertEqual([(e['v'],e['w']) for e in events],[(0.,0.),(.2,0.),(0.,.5)])
        self.assertTrue(all(e['topic']=='/cmd_vel' and e['header_sim_ns']==3000000007 for e in events))


if __name__=='__main__':unittest.main()
