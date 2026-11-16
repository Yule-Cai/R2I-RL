"""Measure the causal identity ablation without treating outcomes as validity.

Use actual COMMIT, guard response/decision and native output times. A monitor
that never intervenes within the fixed window is censored, not assigned a fake
5s stopping time. Navigation failures and physical entries remain outcomes.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate, translate
from audit_commit_window_nav2_v1 import audit as audit_delivery
from audit_motion_prefix_v1 import assess


def read(p):return [json.loads(x) for x in p.read_text().splitlines()]


def audit(root,native):
    mission=root/'mission'
    metadata=json.loads((mission/'channel_adapter.json').read_text())
    manifest=json.loads((mission/'manifest.json').read_text())
    config=json.loads((mission/'snapshot_channel/schedule.json').read_text())
    task=json.loads((mission/'source/scene_case.json').read_text())
    assert config['condition']=='clean' and metadata['external_update_exposure_s']==5.
    arm=metadata['monitor_arm'];assert arm==manifest['trial']['arm']
    assert arm in ('full','geometry_only')
    delivery=audit_delivery(root,root/'service_cdr_decoded.json');assert delivery['passed']
    events=read(mission/'events.jsonl');guard=read(mission/'guard_events.jsonl')
    world=read(mission/'world_pose.jsonl');manager=read(mission/'manager_events.jsonl');wal=read(mission/'wal.jsonl')
    commits=[r for r in manager if r['event']=='durable_commit' and r['epoch']==2];assert len(commits)==1
    commit=commits[0];t0=commit['t_ns'];end=t0+5*10**9
    windows=[r for r in events if r['kind']=='revocation_window'];assert len(windows)==1
    assert windows[0]['commit_ns']==t0 and windows[0]['planned_end_ns']==end
    ending=[r for r in events if r['kind']=='revocation_window_end'];assert len(ending)==1 and ending[0]['receipt_ns']>=end
    external=[r for r in events if r['kind']=='external_update_without_anticipatory_stop'];assert len(external)==1 and external[0]['receipt_ns']<t0
    before_hold=[r for r in events if r['kind']=='interlock_hold' and external[0]['receipt_ns']<=r['receipt_ns']<t0]
    initial=[r for r in events if r['kind']=='task_release' and r['epoch']==1];assert len(initial)==1
    old=initial[0]['identity'];assert old[0]==1
    target=[2,commit['snapshot_hash']];assert old!=target
    decision=next(r for r in wal if r.get('decision')=='COMMIT' and r.get('epoch')==2)
    assert decision['target_hash']==commit['snapshot_hash']
    regions=[Polygon([(p[0],p[1]) for p in r['polygon']]) for r in decision['target_rules'] if r['rule_type']==0 and r['severity']==1]
    assert regions and all(p.is_valid for p in regions)
    paths=[r for r in events if r['kind']=='planner_result' and r['receipt_ns']<t0];assert len(paths)==1
    invalidated=any(LineString(paths[0]['xy']).distance(p)<=manifest['gate_rho_m'] for p in regions)
    assert invalidated,'Update does not invalidate the actual initially admitted path'
    decoded=json.loads((root/'service_cdr_decoded.json').read_text())['decoded']
    callbacks=[r for r in guard if r['event']=='channel_snapshot_response_callback' and r['current_generation']
        and r['epoch']==2 and r['active_hash']==target[1] and t0<=r['t_ns']<end
        and not decoded[r['cdr_sha256']]['fields']['degraded']]
    assert callbacks,'No fresh nondegraded new-snapshot callback in the fixed exposure'
    observed=callbacks[0]['t_ns']
    holds=[r for r in guard if r['event']=='hold_decision' and t0<=r['t_ns']<end]
    first=holds[0] if holds else None
    state=('preexisting_hold' if before_hold else 'new_hold' if first else 'right_censored_at_window_end')
    raw_native=json.loads((native/'summary.json').read_text());assert raw_native['passed'] and raw_native['require_complete_values']
    commands=list(csv.DictReader((native/'command_native_timeline.csv').open()))
    output=[r for r in commands if observed<=int(r['rmw_publish_ns'])<end]
    nonzero=[r for r in output if r['observed_nonzero']=='True']
    zero_after_hold=next((r for r in commands if first and int(r['rmw_publish_ns'])>=first['t_ns'] and r['observed_nonzero']=='False'),None)
    body=Polygon(manifest['body_envelope_xy'])
    def gap(w):
        x,y,z,q=w['quaternion'];yaw=math.atan2(2*(q*z+x*y),1-2*(y*y+z*z))
        placed=translate(rotate(body,yaw,origin=(0,0),use_radians=True),*w['xyz'][:2])
        return min(placed.distance(p) for p in regions),any(placed.intersects(p) for p in regions)
    selected=[w for w in world if t0<=w['receipt_ns']<end]
    assert len(selected)>=20 and selected[-1]['receipt_ns']>=end-10**8
    gaps=[(b['sim_ns']-a['sim_ns'])/1e9 for a,b in zip(selected,selected[1:])]
    assert gaps and min(gaps)>0 and max(gaps)<=manifest['max_world_sim_gap_s']
    geometry=[gap(w) for w in selected]
    distance=sum(math.dist(a['xyz'][:2],b['xyz'][:2]) for a,b in zip(selected,selected[1:]))
    after_obs=[w for w in selected if w['receipt_ns']>=observed]
    observed_distance=sum(math.dist(a['xyz'][:2],b['xyz'][:2]) for a,b in zip(after_obs,after_obs[1:]))
    at_hold=min(selected,key=lambda r:abs(r['receipt_ns']-first['t_ns'])) if first else None
    authority_states=[r for r in events if r['kind']=='guard_state' and observed<=r['receipt_ns']<end
        and r['authority']==target and r['execution']==old]
    prefix=assess(manifest,events,world,manager,wal)
    source_ok=all(hashlib.sha256((mission/'source'/f'{i:02}_{Path(p).name}').read_bytes()).hexdigest()==h
        for i,(p,h) in enumerate(manifest['source_sha256'].items()))
    assert source_ok and hashlib.sha256((mission/'params.yaml').read_bytes()).hexdigest()==manifest['parameters_sha256']
    result=json.loads((mission/'result.json').read_text())
    return dict(evidence_valid=True,arm=arm,seed=manifest['seed'],scene=task['scene'],task_id=task['id'],
        parameter_sha256=manifest['parameters_sha256'],initial_path_invalidated=True,
        new_snapshot_hash=target[1],first_fresh_observation_after_commit_ms=(observed-t0)/1e6,
        intervention_status=state,first_hold_reason=first['reason'] if first else None,
        commit_to_first_monitor_hold_ms=(first['t_ns']-t0)/1e6 if first and not before_hold else None,
        callback_to_first_monitor_hold_ms=(first['t_ns']-observed)/1e6 if first and first['t_ns']>=observed and not before_hold else None,
        hold_to_native_zero_ms=(int(zero_after_hold['rmw_publish_ns'])-first['t_ns'])/1e6 if zero_after_hold is not None else None,
        prior_hold_reasons=[r['reason'] for r in before_hold],
        fixed_window_s=5.,actual_common_stop_overshoot_ms=(ending[0]['receipt_ns']-end)/1e6,
        observed_window_travel_m=distance,travel_after_fresh_observation_m=observed_distance,
        post_observation_nonzero_native_publications=len(nonzero),
        first_hold_body_gap_m=gap(at_hold)[0] if at_hold else None,
        first_hold_nearest_world_receipt_gap_ms=abs(at_hold['receipt_ns']-first['t_ns'])/1e6 if at_hold else None,
        sampled_body_entries_during_window=sum(x[1] for x in geometry),minimum_body_gap_during_window_m=min(x[0] for x in geometry),
        fresh_authority_with_old_execution_state_counts={str(s):sum(r['state']==s for r in authority_states) for s in [0,1,2]},
        guard_receipts_verified=delivery['field_exact_guard_receipts'],native_commands_verified=raw_native['final_publications'],
        observed_window_samples=len(selected),max_window_sim_gap_s=max(gaps),
        task_complete=result['result']=='PASS',retained_failure=result.get('error'),
        readmission_witnesses=prefix['releases'],
        scope='paired causal monitor ablation in a5s fixed window;no invented stopping time for censored outcomes')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--native',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=audit(a.root,a.native);a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.output/'audit_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in result.items() if k!='readmission_witnesses'},indent=2))


if __name__=='__main__':main()
