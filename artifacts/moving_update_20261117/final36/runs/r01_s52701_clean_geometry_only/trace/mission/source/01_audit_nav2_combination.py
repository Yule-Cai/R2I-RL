#!/usr/bin/env python3
"""Offline paired-Nav2 audit. GEOS geometry never calls the admission backend."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

import shapely
from shapely.geometry import Polygon, LineString
from shapely.affinity import rotate, translate


def read(path): return [json.loads(s) for s in path.read_text().splitlines() if s]


def audit_trial(root):
    manifest=json.loads((root/'manifest.json').read_text()); saved=json.loads((root/'result.json').read_text())
    source_ok=all(hashlib.sha256((root/'source'/f'{i:02}_{Path(p).name}').read_bytes()).hexdigest()==h
        for i,(p,h) in enumerate(manifest['source_sha256'].items()))
    source_ok &= hashlib.sha256((root/'params.yaml').read_bytes()).hexdigest()==manifest['parameters_sha256']
    if saved['result']!='PASS':
        return dict(**manifest['trial'],completed=False,audit_passed=False,source_integrity=source_ok,
            recorded_failure=saved.get('error'))
    rows=read(root/'events.jsonl'); world=read(root/'world_pose.jsonl')
    commits=[r for r in read(root/'manager_events.jsonl') if r['event']=='durable_commit' and r['epoch']==2]
    assert len(commits)==1,'unique durable commit required'
    commit=commits[0]
    wal=[r for r in read(root/'wal.jsonl') if r.get('decision')=='COMMIT' and r.get('epoch')==2]
    assert len(wal)==1 and wal[0]['target_hash']==commit['snapshot_hash']
    regions=[Polygon([(p[0],p[1]) for p in r['polygon']]) for r in wal[0]['target_rules']
        if r['rule_type']==0 and r['severity']==1]
    assert len(regions)==1 and regions[0].is_valid
    region=regions[0]; body=Polygon(manifest['body_envelope_xy'])
    active=[r for r in world if r['active']]
    assert len(active)>=20
    gaps=[(b['sim_ns']-a['sim_ns'])/1e9 for a,b in zip(active,active[1:])]
    assert min(gaps)>0 and max(gaps)<=manifest['max_world_sim_gap_s']
    post=[r for r in active if r['receipt_ns']>=commit['t_ns']]
    assert len(post)>=20
    distances=[]; entries=0
    for r in post:
        assert (r['parent'],r['child'])==('default','turtlebot3_waffle')
        x,y,z,w=r['quaternion']; assert abs(x*x+y*y+z*z+w*w-1)<1e-3
        roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
        assert abs(roll)<.05 and abs(pitch)<.05
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        b=translate(rotate(body,yaw,origin=(0,0),use_radians=True),r['xyz'][0],r['xyz'][1])
        entries+=int(b.intersects(region)); distances.append(b.distance(region))
    req=[r for r in rows if r['kind']=='update_requested']; assert len(req)==1
    release=[r for r in rows if r['kind']=='task_release' and r['epoch']==2]; assert len(release)==1
    release=release[0]; request_ns=req[0]['receipt_ns']
    assert release['identity']==[2,commit['snapshot_hash']]
    admitted=[r for r in rows if r['kind']=='gate_verdict' and r['execute']
        and request_ns<r['receipt_ns']<release['receipt_ns']]
    assert len(admitted)==1 and admitted[0]['epoch']==2
    assert admitted[0]['snapshot_hash']==commit['snapshot_hash']
    # Separate DDS topics may be received out of order; both evidence records
    # must precede release, rather than requiring their receipt order to match.
    forwarded=[r for r in rows if r['kind']=='monitor_path'
        and request_ns<r['receipt_ns']<release['receipt_ns']]
    assert forwarded and forwarded[-1]['digest']==release['digest']
    applies=[r for r in rows if r['kind']=='corresponding_costmaps_applied' and r['epoch']==2]
    assert len(applies)==1 and commit['t_ns']<applies[0]['receipt_ns']<release['receipt_ns']
    for name in ['global_costmap','local_costmap']:
        m=json.loads((root/f'costmap_epoch2_{name}.json').read_text())
        assert m['frame']=='map' and m['update_time_ns']>=applies[0]['mask']['stamp_ns']
        i=math.floor((.2-m['origin'][0])/m['resolution']); j=math.floor((-.5-m['origin'][1])/m['resolution'])
        assert m['data'][j*m['width']+i]==254
    path=[r for r in rows if r['kind']=='planner_result' and r['receipt_ns']>request_ns]
    assert len(path)==1 and LineString(path[0]['xy']).distance(region)>manifest['gate_rho_m']
    terminal=next(r for r in rows if r['kind']=='old_action_terminal')
    assert terminal['status']==5 and terminal['receipt_ns']<release['receipt_ns']
    hold=next(r for r in rows if r['kind']=='interlock_hold' and r['receipt_ns']>=request_ns)
    command=[r for r in rows if r['kind']=='command' and r['topic']=='/cmd_vel']
    zero=next(r for r in command if r['receipt_ns']>=hold['receipt_ns'] and abs(r['v'])+abs(r['w'])<=1e-6)
    between=[r for r in command if zero['receipt_ns']<=r['receipt_ns']<release['receipt_ns']]
    residual=sum(abs(r['v'])+abs(r['w'])>1e-6 for r in between)
    injected=[r for r in rows if r['kind']=='injected_candidate_denied']
    assert len(injected)==int(manifest['trial']['condition']=='invalid_candidate')
    states=next(r for r in rows if r['kind']=='active_states')['states']; assert all(s==3 for s in states.values())
    final=world[-1]; direction=manifest['trial']['direction']
    goal_error=math.hypot(final['xyz'][0]-2*direction,final['xyz'][1]+.5)
    assert abs(goal_error-saved['independent_goal_error_m'])<.005 and goal_error<=.20
    return dict(**manifest['trial'],completed=True,source_integrity=bool(source_ok),
        audit_passed=bool(source_ok and residual==0),sampled_body_entries=entries,
        post_commit_samples=len(post),min_body_gap_m=min(distances),max_sim_gap_s=max(gaps),
        request_to_hold_ms=(hold['receipt_ns']-request_ns)/1e6,
        hold_to_zero_receipt_ms=(zero['receipt_ns']-hold['receipt_ns'])/1e6,
        request_to_release_ms=(release['receipt_ns']-request_ns)/1e6,
        commit_to_release_ms=(release['receipt_ns']-commit['t_ns'])/1e6,
        nonzero_after_first_zero_before_release=residual,independent_goal_error_m=goal_error)


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root',type=Path)
    parser.add_argument('--trial',action='store_true'); args=parser.parse_args()
    roots=[args.root] if args.trial else sorted(args.root.glob('trial_*'))
    results=[]
    for root in roots:
        try: results.append(dict(directory=root.name,**audit_trial(root)))
        except Exception as e: results.append(dict(directory=root.name,audit_passed=False,error=repr(e)))
    groups=[]
    for arm in ['full','geometry_only']:
        for condition in ['normal','delayed_mask','invalid_candidate','restart']:
            rows=[r for r in results if r.get('arm')==arm and r.get('condition')==condition]
            valid=[r for r in rows if r.get('audit_passed')]
            if rows:
                groups.append(dict(arm=arm,condition=condition,attempted=len(rows),valid=len(valid),
                    body_entry_trials=sum(r['sampled_body_entries']>0 for r in valid),
                    request_to_release_median_ms=statistics.median(r['request_to_release_ms'] for r in valid) if valid else None))
    output=dict(audit_passed=bool(results) and all(r['audit_passed'] for r in results),
        shapely_version=shapely.__version__,geos_version=shapely.geos_version_string,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        trials=results,groups=groups,
        scope='sampled declared body envelope, actual costmaps, action/event ordering; shared transaction backend')
    (args.root/'independent_audit.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2)); return 0 if output['audit_passed'] else 1


if __name__=='__main__': raise SystemExit(main())
