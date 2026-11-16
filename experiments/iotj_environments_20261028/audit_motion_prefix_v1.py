"""Independent recorded-prefix checks, including missions without a renewed release.

Measurements describe recorded samples and receipt-order witnesses. Missing logs
and unexercised epochs remain explicit; a failed mission is never relabeled.
"""
import hashlib
import json
import math
from pathlib import Path

from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Polygon


def sha(p):
    digest=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(2**20),b''):digest.update(chunk)
    return digest.hexdigest()


def prefix(path):
    rows=[];errors=[]
    if not path.exists():return rows,dict(status='missing',valid_rows=0)
    for number,line in enumerate(path.read_text().splitlines(),1):
        if not line.strip():continue
        try:rows.append(json.loads(line))
        except (ValueError,TypeError) as error:
            errors.append(dict(line=number,error=str(error)));break
    return rows,dict(status='valid_prefix_only' if errors else 'complete_file',valid_rows=len(rows),errors=errors)


def assess(manifest,events,world,manager,wal):
    commits=sorted([c for c in manager if c.get('event')=='durable_commit'],key=lambda c:c['t_ns'])
    epochs=[];valid_commits={}
    body=Polygon(manifest['body_envelope_xy']);assert body.is_valid and not body.is_empty
    for epoch in [1,2]:
        matches=[c for c in commits if c['epoch']==epoch]
        decisions=[d for d in wal if d.get('decision')=='COMMIT' and d.get('epoch')==epoch]
        row=dict(epoch=epoch,durable_commit_records=len(matches),wal_commit_records=len(decisions),
            authority_witness='unavailable',geometry_status='not_assessed',sampled_body_entries=None,
            assessed_samples=0,invalid_pose_samples=0)
        if len(matches)!=1 or len(decisions)!=1:epochs.append(row);continue
        c,d=matches[0],decisions[0]
        if c['snapshot_hash']!=d['target_hash']:
            row['authority_witness']='hash_disagreement';epochs.append(row);continue
        regions=[Polygon([(p[0],p[1]) for p in rule['polygon']]) for rule in d['target_rules']
                 if rule['rule_type']==0 and rule['severity']==1]
        if not regions or not all(p.is_valid and not p.is_empty for p in regions):
            row['authority_witness']='unsupported_or_invalid_geometry';epochs.append(row);continue
        valid_commits[epoch]=c;row['authority_witness']='matched_manager_and_wal'
        next_commit=min([x['t_ns'] for x in commits if x['t_ns']>c['t_ns']],default=math.inf)
        samples=[w for w in world if c['t_ns']<=w['receipt_ns']<next_commit]
        gaps=[(z['sim_ns']-y['sim_ns'])/1e9 for y,z in zip(samples,samples[1:])]
        entries=0;distances=[]
        for w in samples:
            x,y,z,q=w['quaternion'];norm=x*x+y*y+z*z+q*q
            roll=math.atan2(2*(q*x+y*z),1-2*(x*x+y*y))
            pitch=math.asin(max(-1.,min(1.,2*(q*y-z*x))))
            valid=((w['parent'],w['child'])==('default','turtlebot3_waffle')
                   and all(math.isfinite(v) for v in w['quaternion']+w['xyz'])
                   and abs(norm-1)<1e-3 and abs(roll)<.05 and abs(pitch)<.05)
            if not valid:row['invalid_pose_samples']+=1;continue
            yaw=math.atan2(2*(q*z+x*y),1-2*(y*y+z*z))
            placed=translate(rotate(body,yaw,origin=(0,0),use_radians=True),*w['xyz'][:2])
            entries+=int(any(placed.intersects(region) for region in regions))
            distances.append(min(placed.distance(region) for region in regions))
        row.update(recorded_samples=len(samples),assessed_samples=len(distances),
            sampled_body_entries=entries if distances else None,
            min_sampled_body_gap_m=min(distances) if distances else None,
            max_sim_gap_s=max(gaps) if gaps else None,
            geometry_status=('sampled_prefix_with_valid_spacing' if gaps and min(gaps)>0
                and max(gaps)<=manifest['max_world_sim_gap_s'] and not row['invalid_pose_samples']
                else 'sparse_or_invalid_prefix') if samples else 'not_assessed')
        paths=[p for p in events if p['kind']=='planner_result' and c['t_ns']<=p['receipt_ns']<next_commit]
        row['planner_paths_recorded']=len(paths)
        row['recorded_planner_paths_clear']=all(LineString(p['xy']).distance(region)>manifest['gate_rho_m']
            for p in paths for region in regions) if paths else None
        epochs.append(row)
    releases=sorted([e for e in events if e['kind']=='task_release'],key=lambda r:r['receipt_ns']);release_rows=[]
    for r in releases:
        c=valid_commits.get(r['epoch']);before=[e for e in events if e['receipt_ns']<r['receipt_ns']]
        previous_commits=[x for x in commits if x['t_ns']<r['receipt_ns']]
        gate=[e for e in before if e['kind']=='gate_verdict' and e['execute'] and e['epoch']==r['epoch']]
        forwarded=[e for e in before if e['kind']=='monitor_path']
        applied=[e for e in before if e['kind']=='corresponding_costmaps_applied' and e['epoch']==r['epoch']]
        valid=bool(c and previous_commits and previous_commits[-1]==c
            and r['identity']==[r['epoch'],c['snapshot_hash']]
            and gate and gate[-1]['snapshot_hash']==c['snapshot_hash']
            and forwarded and forwarded[-1]['digest']==r['digest']
            and applied and c['t_ns']<applied[-1]['receipt_ns'])
        release_rows.append(dict(epoch=r['epoch'],receipt_ns=r['receipt_ns'],witness_valid=valid,
            commit_to_release_receipt_ms=(r['receipt_ns']-c['t_ns'])/1e6 if c else None))
    commands=sorted([e for e in events if e['kind']=='command' and e['topic']=='/cmd_vel'],key=lambda r:r['receipt_ns'])
    unwitnessed=[]
    for cmd in commands:
        if abs(cmd['v'])+abs(cmd['w'])<=1e-6:continue
        prior_c=[c for c in commits if c['t_ns']<cmd['receipt_ns']]
        prior_r=[r for r in release_rows if r['receipt_ns']<cmd['receipt_ns']]
        if not prior_c or not prior_r or not prior_r[-1]['witness_valid'] or prior_c[-1]['epoch']!=prior_r[-1]['epoch']:
            unwitnessed.append(cmd['receipt_ns'])
    intervals=[]
    for request in [e for e in events if e['kind']=='update_requested']:
        holds=[e for e in events if e['kind']=='interlock_hold' and e['receipt_ns']>=request['receipt_ns']]
        end=min([r['receipt_ns'] for r in releases if r['receipt_ns']>request['receipt_ns']],default=math.inf)
        zeros=[c for c in commands if holds and holds[0]['receipt_ns']<=c['receipt_ns']<end and abs(c['v'])+abs(c['w'])<=1e-6]
        interval=[c for c in commands if zeros and zeros[0]['receipt_ns']<=c['receipt_ns']<end]
        held_world=[w for w in world if request['receipt_ns']<=w['receipt_ns']<end]
        onset=None;stop=None
        for x,y in zip(held_world,held_world[1:]):
            dt=(y['sim_ns']-x['sim_ns'])/1e9
            if not 0<dt<=.10 or math.dist(x['xyz'][:2],y['xyz'][:2])/dt>.01:onset=None;continue
            if onset is None:onset=x
            if (y['sim_ns']-onset['sim_ns'])/1e9>=.20:
                stop=dict(onset_after_request_ms=(onset['receipt_ns']-request['receipt_ns'])/1e6,
                    confirmation_after_request_ms=(y['receipt_ns']-request['receipt_ns'])/1e6,
                    sustained_sim_s=(y['sim_ns']-onset['sim_ns'])/1e9,speed_threshold_mps=.01);break
        intervals.append(dict(request_ns=request['receipt_ns'],hold_observed=bool(holds),first_zero_observed=bool(zeros),
            renewed_release_observed=end<math.inf,commands_in_interval=len(interval),
            request_to_hold_receipt_ms=(holds[0]['receipt_ns']-request['receipt_ns'])/1e6 if holds else None,
            hold_to_zero_receipt_ms=(zeros[0]['receipt_ns']-holds[0]['receipt_ns'])/1e6 if zeros else None,
            request_to_release_receipt_ms=(end-request['receipt_ns'])/1e6 if end<math.inf else None,
            last_recorded_event_ns=max([e['receipt_ns'] for e in events],default=None),
            sustained_observed_stop=stop,
            nonzero_after_first_zero=sum(abs(c['v'])+abs(c['w'])>1e-6 for c in interval) if interval else None))
    return dict(epochs=epochs,releases=release_rows,command_samples=len(commands),
        observed_nonzero_without_current_release_witness=len(unwitnessed) if commands else None,
        unwitnessed_command_receipt_ns=unwitnessed,interlock_intervals=intervals,
        world_samples=len(world),terminal_world_goal_error_m=math.dist(world[-1]['xyz'][:2],manifest['goal']) if world else None,
        scope='recorded sampled body geometry and receipt-order witnesses;no continuous safety or contact certificate')


def audit(trial):
    result=dict(trial=str(trial),audit_source_sha256=sha(Path(__file__)))
    if not (trial/'manifest.json').exists():return dict(result,assessment='not_assessed_missing_manifest')
    manifest=json.loads((trial/'manifest.json').read_text())
    result.update(cohort_id=manifest.get('cohort_id'),formal_denominator=manifest.get('formal_denominator',False))
    paths={trial/'source'/f'{i:02}_{Path(p).name}':h for i,(p,h) in enumerate(manifest['source_sha256'].items())}
    paths[trial/'params.yaml']=manifest['parameters_sha256']
    if manifest.get('runtime_world_sha256'):paths[trial/'world.sdf']=manifest['runtime_world_sha256']
    if manifest.get('task_source_sha256'):paths[trial/'source/scene_case.json']=manifest['task_source_sha256']
    missing=[str(p.relative_to(trial)) for p in paths if not p.is_file()]
    mismatched=[str(p.relative_to(trial)) for p,h in paths.items() if p.is_file() and sha(p)!=h]
    runner_present=any(sha(p)==manifest['runner_sha256'] for p in (trial/'source').glob('*.py'))
    result.update(source_snapshot_integrity=not missing and not mismatched and runner_present,
        runner_snapshot_present=runner_present,missing_snapshot_files=missing,mismatched_snapshot_files=mismatched)
    logs={};coverage={}
    for key,name in [('events','events.jsonl'),('world','world_pose.jsonl'),('manager','manager_events.jsonl'),('wal','wal.jsonl')]:logs[key],coverage[key]=prefix(trial/name)
    result['log_coverage']=coverage
    case_path=trial/'source/scene_case.json';checks=[]
    if case_path.exists():
        case=json.loads(case_path.read_text())
        for epoch,key in [(1,'initial_rule'),(2,'replacement_rule')]:
            decisions=[w for w in logs['wal'] if w.get('decision')=='COMMIT' and w.get('epoch')==epoch]
            status='not_recorded'
            if len(decisions)==1:
                rules=decisions[0]['target_rules']
                same=(len(rules)==1 and rules[0]['frame_id']=='map' and rules[0]['rule_type']==0 and rules[0]['severity']==1
                    and Polygon([(x[0],x[1]) for x in rules[0]['polygon']]).equals_exact(Polygon(case[key]),1e-6))
                status='matches_case' if same else 'mismatch'
            elif decisions:status='non_unique'
            checks.append(dict(epoch=epoch,status=status))
    result['intended_case_commit_geometry']=checks
    result['metrics']=assess(manifest,**logs)
    result['assessment']='recorded_prefix_only' if any(x['status']!='complete_file' for x in coverage.values()) else 'closed_recorded_trace'
    return result
