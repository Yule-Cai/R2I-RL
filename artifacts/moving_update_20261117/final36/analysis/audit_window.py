"""Audit fixed moving windows without conflating callbacks, commands and motion."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from geometry_audit import convex_distance, convex_intersects, rectangle, transform


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def audit(root, native):
    mission = root/'mission'
    manifest = json.loads((mission/'manifest.json').read_text())
    events = read(mission/'events.jsonl')
    guard = read(mission/'guard_events.jsonl')
    manager = read(mission/'manager_events.jsonl')
    wal = read(mission/'wal.jsonl')
    world = read(mission/'world_pose.jsonl')
    config = json.loads((mission/'snapshot_channel/schedule.json').read_text())
    case = json.loads((mission/'source/scene_case.json').read_text())
    commits = [r for r in manager if r['event']=='durable_commit' and r['epoch']==2]
    assert len(commits)==1, 'Missing or ambiguous COMMIT'
    commit = commits[0]; start=commit['t_ns']; end=start+5*10**9
    target=[2,commit['snapshot_hash']]
    window = [r for r in events if r['kind']=='revocation_window']
    assert len(window)==1 and window[0]['commit_ns']==start and window[0]['planned_end_ns']==end
    endings = [r for r in events if r['kind']=='revocation_window_end']
    assert len(endings)==1 and endings[0]['receipt_ns']>=end
    external = [r for r in events if r['kind']=='external_update_without_anticipatory_stop']
    assert len(external)==1 and external[0]['receipt_ns']<start
    initial = [r for r in events if r['kind']=='task_release' and r['epoch']==1]
    assert len(initial)==1 and initial[0]['receipt_ns']<external[0]['receipt_ns']
    old_identity=initial[0]['identity']
    assert old_identity!=target
    prior_holds = [r for r in events if r['kind']=='interlock_hold'
                   and initial[0]['receipt_ns']<=r['receipt_ns']<start]
    assert not prior_holds, ('Preexisting HOLD before COMMIT',prior_holds)
    decision = [r for r in wal if r.get('decision')=='COMMIT' and r.get('epoch')==2]
    assert len(decision)==1 and decision[0]['target_hash']==target[1]
    regions=[rectangle(r['polygon']) for r in decision[0]['target_rules']
             if r['rule_type']==0 and r['severity']==1]
    assert regions
    path=next(r for r in events if r['kind']=='planner_result' and r['receipt_ns']<start)
    invalidated=any(convex_distance([a,b],polygon)<=manifest['gate_rho_m']
                    for a,b in zip(path['xy'],path['xy'][1:]) for polygon in regions)
    assert invalidated==case['expect_path_invalidated'], 'Actual path violates fixed geometry condition'
    # This witness is emitted only when the real guard accepted a usable
    # response. callback_entry_ns identifies callback entry; the post-callback
    # t_ns is kept separately rather than being called the instant of knowledge.
    observations=[r for r in guard if r['event']=='snapshot_observed' and r['authority']==target
                  and start<=r['callback_entry_ns']<end]
    observed=observations[0]['callback_entry_ns'] if observations else None
    holds=[r for r in guard if r['event']=='hold_decision' and start<=r['t_ns']<end]
    first=holds[0] if holds else None
    hold_time=first['t_ns'] if first else None
    adapter=[r for r in events if r['kind']=='interlock_hold' and start<=r['receipt_ns']<end]
    cancels=[r for r in events if r['kind']=='cancel_requested' and start<=r['receipt_ns']<end]
    monitor_paths=[r for r in events if r['kind']=='monitor_path' and start<=r['receipt_ns']<end]
    old_clear_states=[r for r in events if r['kind']=='guard_state' and observed is not None
        and observed<=r['receipt_ns']<end and r['authority']==target
        and r['execution']==old_identity and r['state'] in (0,1)]
    refreshed_while_held=[r for r in observations if hold_time is not None
        and hold_time<r['callback_entry_ns']<end]
    renewals=[r for r in events if r['kind']=='task_release' and r['epoch']==2]
    assert len(renewals)<=1
    if renewals:
        assert renewals[0]['receipt_ns']>=end and renewals[0]['identity']==target
    native_summary=json.loads((native/'summary.json').read_text())
    assert native_summary['passed'] and native_summary['publication_values_unobserved']==0
    commands=list(csv.DictReader((native/'command_native_timeline.csv').open()))
    def count_nonzero(threshold):
        if threshold is None:return None
        return sum(r['observed_nonzero']=='True' and threshold<=int(r['rmw_publish_ns'])<end for r in commands)
    first_zero=next((r for r in commands if hold_time is not None
        and hold_time<=int(r['rmw_publish_ns'])<end and r['observed_nonzero']=='False'),None)
    zero_ns=int(first_zero['rmw_publish_ns']) if first_zero else None
    nonzero_after_hold=[r for r in commands if hold_time is not None
        and hold_time<=int(r['rmw_publish_ns'])<end and r['observed_nonzero']=='True']
    last_nonzero_ns=int(nonzero_after_hold[-1]['rmw_publish_ns']) if nonzero_after_hold else None
    stable_zero=next((r for r in commands if hold_time is not None
        and max(hold_time,last_nonzero_ns+1 if last_nonzero_ns else hold_time)<=int(r['rmw_publish_ns'])<end
        and r['observed_nonzero']=='False'),None)
    stable_zero_ns=int(stable_zero['rmw_publish_ns']) if stable_zero else None
    selected=[r for r in world if start<=r['receipt_ns']<end]
    assert len(selected)>=20 and selected[-1]['receipt_ns']>=end-10**8
    gaps=[(b['sim_ns']-a['sim_ns'])/1e9 for a,b in zip(selected,selected[1:])]
    assert gaps and min(gaps)>0 and max(gaps)<=manifest['max_world_sim_gap_s']
    body=manifest['body_envelope_xy']
    distances=[];entries=0
    for pose in selected:
        x,y,z,w=pose['quaternion']
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        footprint=transform(body,*pose['xyz'][:2],yaw)
        distances.append(min(convex_distance(footprint,p) for p in regions))
        entries+=any(convex_intersects(footprint,p) for p in regions)
    def travel(rows):
        return sum(math.dist(a['xyz'][:2],b['xyz'][:2]) for a,b in zip(rows,rows[1:]))
    source_ok=all(hashlib.sha256((mission/'source'/f'{i:02}_{Path(p).name}').read_bytes()).hexdigest()==digest
                  for i,(p,digest) in enumerate(manifest['source_sha256'].items()))
    assert source_ok
    assert hashlib.sha256((mission/'params.yaml').read_bytes()).hexdigest()==manifest['parameters_sha256']
    result=json.loads((mission/'result.json').read_text())
    return dict(assessed=True,arm=config['monitor_arm'],condition=config['condition'],seed=manifest['seed'],
        task_id=manifest['task_id'],expected_path_invalidated=case['expect_path_invalidated'],
        actual_path_invalidated=invalidated,parameters_sha256=manifest['parameters_sha256'],
        commit_ns=start,window_end_ns=end,window_end_overshoot_ms=(endings[0]['receipt_ns']-end)/1e6,
        target_identity=target,old_identity=old_identity,
        first_target_snapshot_callback_entry_ns=observed,
        target_callback_to_postvalidation_log_ms=(observations[0]['t_ns']-observed)/1e6 if observations else None,
        commit_to_target_callback_ms=(observed-start)/1e6 if observed else None,
        first_hold_ns=hold_time,first_hold_reason=first['reason'] if first else None,
        intervention_status='HOLD' if first else 'right_censored_at_5s',
        commit_to_hold_ms=(hold_time-start)/1e6 if first else None,
        hold_before_target_callback=hold_time<observed if first and observed else None,
        target_callback_to_hold_ms=(hold_time-observed)/1e6 if first and observed and hold_time>=observed else None,
        hold_to_adapter_ms=(adapter[0]['receipt_ns']-hold_time)/1e6 if first and adapter else None,
        adapter_hold_ns=adapter[0]['receipt_ns'] if adapter else None,
        controller_cancel_request_ns=cancels[0]['receipt_ns'] if cancels else None,
        first_zero_publication_ns=zero_ns,
        first_zero_simulator_take_ns=int(first_zero['simulator_bridge_take_ns']) if first_zero and first_zero['simulator_bridge_take_ns'] else None,
        hold_to_zero_publication_ms=(zero_ns-hold_time)/1e6 if zero_ns else None,
        last_nonzero_after_hold_ns=last_nonzero_ns,
        stable_zero_publication_ns=stable_zero_ns,
        hold_to_stable_zero_publication_ms=(stable_zero_ns-hold_time)/1e6 if stable_zero_ns else None,
        stable_zero_observation_duration_ms=(end-stable_zero_ns)/1e6 if stable_zero_ns else None,
        nonzero_publications_after_commit=count_nonzero(start),
        nonzero_publications_after_target_callback=count_nonzero(observed),
        nonzero_publications_after_hold=count_nonzero(hold_time),
        nonempty_path_receipts_during_exposure=len(monitor_paths),
        old_execution_clear_state_receipts_after_target_callback=len(old_clear_states),
        accepted_target_callbacks_after_hold_before_readmission=len(refreshed_while_held),
        fresh_readmission=bool(renewals),renewed_release_ns=renewals[0]['receipt_ns'] if renewals else None,
        post_window_renewal_ms=(renewals[0]['receipt_ns']-end)/1e6 if renewals else None,
        window_body_samples=len(selected),maximum_sample_gap_sim_s=max(gaps),
        sampled_body_entries=entries,minimum_body_clearance_m=min(distances),
        window_travel_m=travel(selected),
        travel_after_target_callback_m=travel([r for r in selected if observed and r['receipt_ns']>=observed]) if observed else None,
        task_complete=result['result']=='PASS',retained_task_error=result.get('error'),
        native_final_publications=native_summary['final_publications'],source_integrity=True,
        timing_scope='Measured software callback, HOLD, publication and middleware take; not physical stop')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--native',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        result=audit(args.root,args.native)
    except Exception as exc:
        result=dict(assessed=False,error=repr(exc))
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))
