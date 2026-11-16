"""One immutable, uniquely named attempt; errors never delete or replace evidence."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from contract_v1 import COHORT,atomic,check_files,load,selected,sha

ROOT=Path('/home/gamer/Projects/rulesync-iotj-environments-20261028')


def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--key',required=True);p.add_argument('--phase',choices=['formal','preflight'],required=True);p.add_argument('--domain',type=int,required=True)
    a=p.parse_args();m=load(a.manifest,a.sha256);row=selected(m,a.key,a.phase)
    inputs=check_files(ROOT,m,row)
    assert a.domain in (220,221,222)
    sys.path.insert(0,str(ROOT));import run_native_furnished_recovery_dev_v2 as base
    assert subprocess.check_output(['podman','image','inspect',base.IMAGE,'--format','{{.Id}}'],text=True).strip()==m['image_id']
    cohort_root=a.manifest.parent;out=cohort_root/'runs'/a.phase/row['id'];out.mkdir(parents=True,exist_ok=False)
    c=m['cases'][row['case_id']];case=json.loads((ROOT/c['path']).read_text())
    runner=m['runners']['native' if row['arm'].startswith('native_') else 'rulesync']
    inside=lambda n:'/experiment/'+str(n)
    args=dict(scene=inside(m['scenes'][row['scene']]['prepared']),output=inside((out/'position_1').relative_to(ROOT)),
        case=inside(c['path']),spawn=case['spawn'],goal=case['goal'],seed=row['seed'],stage='recovery',speed=row['speed'],scene_id=row['scene'],
        geometry_profile='standard',controller_profile='legacy',cohort_id=COHORT if a.phase=='formal' else None,
        concurrency=3,render_mode='gpu',target_rtf=1.,ros_domain=a.domain,arm=row['arm'],experiment_phase=a.phase,
        cohort_manifest=inside(a.manifest.relative_to(ROOT)),cohort_sha256=a.sha256,source_root='/experiment',attempt_key=row['key'])
    if row['arm'].startswith('native_'):args['mode']=row['arm'].removeprefix('native_')
    inputs[str(a.manifest.relative_to(ROOT))]=a.sha256
    for n,h in inputs.items():
        target=out/'input_snapshot'/n;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/n,target)
        assert sha(target)==h
    name='rulesync-dm2970-'+row['id'][:18]
    launch=dict(cohort=COHORT,manifest_sha256=a.sha256,key=row['key'],row=row,phase=a.phase,
        image_id=m['image_id'],container=name,arguments=args,input_sha256=inputs)
    atomic(out/'launch.json',launch)
    code="""import json,runpy,sys
from pathlib import Path
from types import SimpleNamespace
runner=Path(sys.argv[1]);sys.path.insert(0,str(runner.parent))
a=SimpleNamespace(**json.loads(sys.argv[2]))
for k in ['scene','output','case']:setattr(a,k,Path(getattr(a,k)))
r=runpy.run_path(str(runner))['run'](a)
raise SystemExit(0 if r['result']=='PASS' else 1)
"""
    cmd=base.container(name,['python3','-u','-c',code,inside(runner),json.dumps(args)],gpu=True)
    # This outer watchdog exceeds the largest retained scientific follow budget
    # plus startup. The scientific clocks and budgets themselves are unchanged.
    cmd[cmd.index('--timeout')+1]='1500'
    i=cmd.index('--entrypoint');cmd[i:i]=['-v',str(ROOT/m['scenes'][row['scene']]['asset_root'])+':/assets/'+row['scene']+':ro',
        '--env','GZ_IP=127.0.0.1','--env','RULESYNC_BASELINE='+row['arm'],'--env','RULESYNC_NATIVE_MODE='+row['arm'].removeprefix('native_')]
    started=time.monotonic()
    with (out/'host.log').open('x') as log:proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
    result=out/'position_1/result.json'
    unchanged=all(sha(ROOT/n)==h for n,h in inputs.items())
    atomic(out/'closed.json',dict(returncode=proc.returncode,wall_s=time.monotonic()-started,
        result=json.loads(result.read_text()) if result.exists() else None,inputs_unchanged=unchanged,manifest_sha256=a.sha256))
    print(json.dumps(dict(id=row['id'],returncode=proc.returncode,result=json.loads(result.read_text()) if result.exists() else None,inputs_unchanged=unchanged)),flush=True)
    assert unchanged


if __name__=='__main__':main()
