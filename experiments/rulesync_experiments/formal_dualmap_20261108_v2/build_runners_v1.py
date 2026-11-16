"""Add strict cohort identity to the already checked navigation code, in new files."""
import ast
import hashlib
import json
from pathlib import Path


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def change(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b,1)


def main():
    here=Path(__file__).resolve().parent;prior=here.parent/'localization_repair_20261108'
    full=prior/'common_arm_v1/scene_runner_rulesync_dualmap_dev_v1.py'
    native=prior/'common_arm_v2/scene_runner_native_dualmap_dev_v2.py'
    assert sha(full)=='d5d31a1e89a28b964d5a2d2d1a68cbd8da235bdb1a6b1543236be6519bade54a'
    assert sha(native)=='04f7a32b8154a28b03c930ea2839ed5071859b0eed1fc479b5b4fec80f8726cc'
    s=full.read_text()
    s=change(s,'base=bench.base','from contract_v1 import runtime_binding, COHORT\nbase=bench.base')
    s=change(s,"    assert args.cohort_id is None, 'Common repair qualification is DEV only'",'    binding=runtime_binding(args,Path(__file__))')
    s=change(s,"assert args.cohort_id=='FurnishedGeneralization1350_20261030'",'assert args.cohort_id==COHORT')
    s=change(s,'assert args.seed in (51001,51002,51003,51004,51005)','assert args.seed in (52501,52502,52503,52504,52505)')
    s=change(s,"case.get('formal_scope')=='new_layout_motion_under_fixed_configuration'","case.get('formal_scope')=='dualmap_planning_input_qualified'")
    s=change(s,'rep=args.seed-51001 if formal else 0','rep=args.seed-52501 if formal else 0')
    s=change(s,'manifest=dict(pilot=not formal','manifest=dict(runtime_binding=binding,pilot=not formal')
    s=change(s,"result=dict(result='FAIL',pilot=not formal","result=dict(runtime_binding=binding,result='FAIL',pilot=not formal")
    a=here/'scene_runner_rulesync_formal_v1.py';assert not a.exists();ast.parse(s);a.write_text(s)
    s=native.read_text()
    s=change(s,'from gazebo_world_pose import instrument_robot','from gazebo_world_pose import instrument_robot\nfrom contract_v1 import runtime_binding')
    s=change(s,"    mode = args.mode or os.environ.get('RULESYNC_NATIVE_MODE') or 'async'", "    binding=runtime_binding(args,Path(__file__))\n    formal=args.experiment_phase=='formal'\n    mode = args.mode or os.environ.get('RULESYNC_NATIVE_MODE') or 'async'")
    s=change(s,"    assert getattr(args, 'cohort_id', None) is None", "    assert bool(args.cohort_id)==formal")
    s=change(s,"runner='scene_runner_native_dualmap_dev_v2.py',pilot=True,formal_denominator=False,cohort_id=None,", "runner='scene_runner_native_formal_v1.py',runtime_binding=binding,pilot=not formal,formal_denominator=formal,cohort_id=args.cohort_id,")
    s=change(s,"result = dict(result='FAIL', arm='NATIVE_' + mode.upper(), mode=mode, pilot=True,\n                  formal_denominator=False, cohort_id=None,", "result = dict(runtime_binding=binding,result='FAIL', arm='NATIVE_' + mode.upper(), mode=mode, pilot=not formal,\n                  formal_denominator=formal, cohort_id=args.cohort_id,")
    s=change(s,'    def on_cmd(self, topic, msg):',"    def on_cmd(self, topic, msg):\n        self.event('command',topic=topic,v=float(msg.twist.linear.x),w=float(msg.twist.angular.z),header_sim_ns=stamp_ns(msg.header.stamp))")
    b=here/'scene_runner_native_formal_v1.py';assert not b.exists();ast.parse(s);b.write_text(s)
    (here/'runner_lineage_v1.json').write_text(json.dumps(dict(parents={str(p):sha(p) for p in [full,native]},
        outputs={p.name:sha(p) for p in [a,b]},scope='Strict input/cohort binding and accurate formal identity metadata; physical navigation, masks, gate/guard semantics unchanged'),indent=2)+'\n')
    print(sha(a),sha(b))


if __name__=='__main__':main()
