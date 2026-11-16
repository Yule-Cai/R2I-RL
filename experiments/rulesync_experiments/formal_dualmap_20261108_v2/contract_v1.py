"""Pinned input identity and lossless attempt inspection; no ROS dependencies."""
import hashlib
import json
import os
from pathlib import Path

COHORT='HSSD_DualMap_2970_20261108_v2'
ARMS=('full','geometry_only','rho0','native_async','native_wait','native_periodic')


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()


def atomic(path,value):
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_name(p.name+'.tmp-'+str(os.getpid()))
    with tmp.open('x') as f:
        json.dump(value,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    tmp.replace(p)


def load(path,digest):
    assert sha(path)==digest,'Manifest SHA mismatch'
    m=json.loads(Path(path).read_text())
    assert m['cohort']==COHORT and len(m['rows'])==m['matrix_total']==2970
    assert sum(m['matrix_per_arm'].values())==2970
    assert len({r['key'] for r in m['rows']})==2970
    assert len({r['id'] for r in m['rows']+m['preflight_rows']})==2976
    assert len(m['preflight_rows'])==6
    return m


def selected(m,key,phase):
    assert phase in ('formal','preflight')
    rows=m['rows'] if phase=='formal' else m['preflight_rows']
    matches=[r for r in rows if r['key']==key];assert len(matches)==1,'Unknown/ambiguous trial key'
    return matches[0]


def reconcile(previous,current):
    if previous and previous.get('status')=='done':
        assert current.get('status')=='done','Previously executed evidence disappeared'
        assert (previous['kind'],previous['passed'])==(current['kind'],current['passed']),'Closed outcome changed; preserve and investigate, never silently replace'
    return current


def check_files(root,m,row=None):
    names=list(m['sources_sha256'])
    if row:
        c=m['cases'][row['case_id']]
        names += [c['path']]+list(m['scenes'][row['scene']]['files_sha256'])
    else:
        names += [c['path'] for c in m['cases'].values()]
        names += [n for s in m['scenes'].values() for n in s['files_sha256']]
    expected=dict(m['sources_sha256'])
    expected.update({c['path']:c['sha256'] for c in m['cases'].values()})
    for s in m['scenes'].values():expected.update(s['files_sha256'])
    for n in names:assert sha(Path(root)/n)==expected[n],('Frozen file changed',n)
    return {n:expected[n] for n in names}


def runtime_binding(args,runner):
    root=Path(args.source_root);m=load(args.cohort_manifest,args.cohort_sha256)
    row=selected(m,args.attempt_key,args.experiment_phase)
    assert (row['scene'],row['seed'],row['speed'],row['arm'])==(args.scene_id,args.seed,args.speed,args.arm)
    formal=args.experiment_phase=='formal'
    assert args.cohort_id==(COHORT if formal else None)
    assert args.stage=='recovery' and args.controller_profile=='legacy' and args.geometry_profile=='standard'
    assert args.render_mode=='gpu' and args.target_rtf==1.
    assert args.concurrency==(3 if formal else 3)
    assert args.ros_domain in (220,221,222)
    expected_runner=m['runners']['native' if row['arm'].startswith('native_') else 'rulesync']
    assert sha(runner)==m['sources_sha256'][expected_runner]
    assert Path(args.case)==root/m['cases'][row['case_id']]['path']
    assert Path(args.scene)==root/m['scenes'][row['scene']]['prepared']
    check_files(root,m,row)
    for p,h in m['runtime_core_sha256'].items():assert sha(p)==h,('Runtime core changed',p)
    return dict(cohort=COHORT,manifest_sha256=args.cohort_sha256,key=row['key'],id=row['id'],
        arm=row['arm'],phase=args.experiment_phase,formal_denominator=formal)


def inspect(out,m,digest,row,phase):
    """Executed or ambiguous launched prefixes are never silently retried."""
    out=Path(out)
    if not out.exists():return dict(kind='absent')
    launch=out/'launch.json'
    if not launch.exists():
        if not any(out.iterdir()):return dict(kind='empty_prelaunch')
        raise ValueError('Nonempty output without launch identity: '+str(out))
    l=json.loads(launch.read_text())
    assert (l['manifest_sha256'],l['key'],l['phase'])==(digest,row['key'],phase)
    assert l['row']==row and l['image_id']==m['image_id']
    a=l['arguments'];c=m['cases'][row['case_id']]
    assert (a['seed'],a['speed'],a['arm'],a['scene_id'])==(row['seed'],row['speed'],row['arm'],row['scene'])
    for n,h in l['input_sha256'].items():assert sha(out/'input_snapshot'/n)==h,('Snapshot mismatch',n)
    close=out/'closed.json';raw=out/'position_1/result.json';manifest=out/'position_1/manifest.json'
    if not close.exists():return dict(kind='executed_interrupted',status='done',passed=False,error='Launched/ambiguous interrupted attempt retained; no retry',evidence=str(out))
    record=json.loads(close.read_text());result=json.loads(raw.read_text()) if raw.exists() else None
    assert record['result']==result and record['manifest_sha256']==digest
    assert record['inputs_unchanged'],'Source/input changed during attempted trial'
    if result is None:return dict(kind='executed_no_result',status='done',passed=False,error='No raw scientific result; prefix retained',evidence=str(out))
    assert manifest.exists(),'Raw result without runtime manifest'
    r=json.loads(manifest.read_text());formal=phase=='formal'
    assert r['formal_denominator'] is formal and r['cohort_id']==(COHORT if formal else None)
    assert result['formal_denominator'] is formal and result['cohort_id']==(COHORT if formal else None)
    assert r['runtime_binding']['manifest_sha256']==digest and r['runtime_binding']['key']==row['key']
    assert r['seed']==row['seed'] and r['requested_speed_mps']==row['speed']
    runner=m['runners']['native' if row['arm'].startswith('native_') else 'rulesync']
    assert r['runner_sha256']==m['sources_sha256'][runner] and r['task_source_sha256']==c['sha256']
    assert any(sha(p)==r['runner_sha256'] for p in (out/'position_1/source').glob('*.py'))
    if row['arm'].startswith('native_'):assert r['mode']==row['arm'].removeprefix('native_')
    else:assert (r['baseline_arm'],r['monitor_arm'],r['gate_rho_m'])==(row['arm'],'geometry_only' if row['arm']=='geometry_only' else 'full',0. if row['arm']=='rho0' else .3)
    assert result.get('source_unchanged') is True
    passed=result['result']=='PASS';assert not passed or record['returncode']==0
    return dict(kind='verified_archive',status='done',passed=passed,error=result.get('error'),phase=result.get('phase'),evidence=str(out))
