"""Local review of every selected row, including failed recorded prefixes."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_motion_prefix_v1 import audit,prefix
prior=Path(__file__).resolve().parents[1]/'localization_repair_20261108'
sys.path.insert(0,str(prior));sys.path.insert(0,str(prior/'common_arm_v1'))
from audit_repair_bundle import world_preserved,xml_value
from audit_six_arm_dev_v1 import native_prefix
from contract_v1 import COHORT,load,sha


def normalized(params):
    p=copy.deepcopy(params);a=p['amcl']['ros__parameters'];a['random_seed']=0
    a['initial_pose'].update(x=0.,y=0.)
    p['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=0.
    return p


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--phase',choices=['preflight','formal'],required=True);p.add_argument('--startup',action='store_true');p.add_argument('--case-id');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=a.root;m=load(root/'manifest.json',a.sha256)
    rows=m['preflight_rows'] if a.phase=='preflight' else m['rows'][:6] if a.startup else m['rows']
    if a.case_id:rows=[r for r in rows if r['case_id']==a.case_id]
    assert rows
    repo=Path(__file__).resolve().parents[3]
    reference=repo/'output/input_requalification_20261108_v1/common_disposable/input_requalification_20261108_v1/common_arm_v1/runs/common_full_v1/position_1'
    ref_params=normalized(yaml.safe_load((reference/'params.yaml').read_text()));ref_robot=xml_value(ET.parse(reference/'robot_world_pose.sdf').getroot())
    reviews=[]
    for row in rows:
        folder=root/'runs'/a.phase/row['id'];trial=folder/'position_1'
        item=dict(id=row['id'],key=row['key'],arm=row['arm'],case_id=row['case_id'],raw_result=None,checks={},assessment='unassessed_missing_runtime')
        checks=item['checks']
        if not (folder/'launch.json').exists():reviews.append(item);continue
        launch=json.loads((folder/'launch.json').read_text())
        checks['launch_identity']=(launch['manifest_sha256']==a.sha256 and launch['row']==row and launch['phase']==a.phase and launch['image_id']==m['image_id'])
        expected=dict(m['sources_sha256']);expected.update(m['scenes'][row['scene']]['files_sha256']);c=m['cases'][row['case_id']];expected[c['path']]=c['sha256']
        manifest_name=next(n for n in launch['input_sha256'] if n.endswith('/manifest.json') and n.startswith(root.name+'/'));expected[manifest_name]=a.sha256
        checks['complete_frozen_input_list']=launch['input_sha256']==expected
        checks['input_snapshots']=all(sha(folder/'input_snapshot'/n)==h for n,h in expected.items())
        if not (trial/'manifest.json').exists():reviews.append(item);continue
        runtime=json.loads((trial/'manifest.json').read_text());formal=a.phase=='formal'
        raw=json.loads((trial/'result.json').read_text()) if (trial/'result.json').exists() else None;item['raw_result']=raw
        closed=json.loads((folder/'closed.json').read_text()) if (folder/'closed.json').exists() else None
        checks['raw_closed_match']=bool(closed and raw is not None and closed['result']==raw and closed['inputs_unchanged'])
        binding=dict(cohort=COHORT,manifest_sha256=a.sha256,key=row['key'],id=row['id'],arm=row['arm'],phase=a.phase,formal_denominator=formal)
        checks['runtime_identity']=runtime['runtime_binding']==binding and bool(raw and raw['runtime_binding']==binding)
        checks['formal_fields']=bool(raw and all(x['formal_denominator'] is formal and x['cohort_id']==(COHORT if formal else None) and x['pilot'] is (not formal) for x in [runtime,raw]))
        runner=m['runners']['native' if row['arm'].startswith('native_') else 'rulesync']
        checks['runner_case_seed_speed']=(runtime['runner_sha256']==m['sources_sha256'][runner] and runtime['task_source_sha256']==c['sha256'] and runtime['seed']==row['seed'] and runtime['requested_speed_mps']==row['speed'])
        params=yaml.safe_load((trial/'params.yaml').read_text())
        checks['common_Nav2_profile']=normalized(params)==ref_params
        checks['actual_parameter_file_bindings']=(params['amcl']['ros__parameters']['random_seed']==row['seed'] and params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']==row['speed'] and sha(trial/'params.yaml')==runtime['parameters_sha256'])
        checks['robot_xml_unchanged']=xml_value(ET.parse(trial/'robot_world_pose.sdf').getroot())==ref_robot
        worldfile=m['scenes'][row['scene']]['prepared']+'/'+row['scene']+'.sdf'
        checks['world_only_declared_edits']=world_preserved(trial/'world.sdf',folder/'input_snapshot'/worldfile)
        case=json.loads((folder/'input_snapshot'/c['path']).read_text())
        events,coverage=prefix(trial/'events.jsonl');commands=[e for e in events if e['kind']=='command' and e['topic']=='/cmd_vel']
        item['max_observed_abs_final_v_mps']=max((abs(e['v']) for e in commands),default=None)
        item['command_value_coverage']='observed' if commands else 'unassessed_native_counter_only' if row['arm'].startswith('native_') else 'missing_expected_command_log'
        checks['requested_speed_respected']=bool(commands) and item['max_observed_abs_final_v_mps']<=row['speed']+1e-5
        checks['complete_events']=coverage['status']=='complete_file'
        if row['arm'].startswith('native_'):
            checks['source_snapshot']=all(sha(trial/'source'/f'{i:02}_{Path(n).name}')==h for i,(n,h) in enumerate(runtime['source_sha256'].items()))
            checks['native_role']=runtime['mode']==row['arm'].removeprefix('native_') and runtime['rulesync_components_started']==[]
            reviewed=native_prefix(trial,case,runtime);reviewed['raw_summary_trigger_displacement_is_final_pose_not_trigger']=False
            triggers=reviewed['actual_trigger_events']
            checks['trigger_summary_correct']=bool(raw and len(triggers)==1 and raw['trigger']['displacement_m']==triggers[0]['displacement_m'])
            checks['published_masks_match_case']=all(reviewed['masks_match_case'])
            checks['world_frame_fields_recorded']=reviewed['per_pose_frame_fields_recorded']
            checks['update_exercised']=len(triggers)==1 and all(e['assessed_samples']>0 for e in reviewed['epochs'])
            checks['no_observed_updated_body_entry']=reviewed['epochs'][1]['sampled_body_entries']==0
            world,_=prefix(trial/'world_pose.jsonl');speeds=[]
            for i in range(10,len(world)):
                before,after=world[i-10],world[i];dt=(after['sim_ns']-before['sim_ns'])/1e9
                if .15<=dt<=.25:speeds.append(math.dist(before['xyz'][:2],after['xyz'][:2])/dt)
            item['max_observed_world_window_speed_mps']=max(speeds) if speeds else None
            item['world_speed_scope']='0.15–0.25s recorded world displacement proxy; not reconstructed command values'
        else:
            reviewed=audit(trial);checks['source_snapshot']=reviewed['source_snapshot_integrity']
            checks['RuleSync_core_unchanged']=runtime['source_sha256']==m['runtime_core_sha256']
            checks['role_and_margin']=(runtime['baseline_arm'],runtime['monitor_arm'],runtime['gate_rho_m'])==(row['arm'],'geometry_only' if row['arm']=='geometry_only' else 'full',0. if row['arm']=='rho0' else .3)
            checks['committed_rules_match_case']=all(e['status']=='matches_case' for e in reviewed['intended_case_commit_geometry'])
            checks['renewed_release_witness']=len(reviewed['metrics']['releases'])==2 and all(e['witness_valid'] for e in reviewed['metrics']['releases'])
            e2=reviewed['metrics']['epochs'][1];checks['update_exercised']=e2['authority_witness']=='matched_manager_and_wal' and e2['assessed_samples']>0
            checks['no_observed_updated_body_entry']=e2['sampled_body_entries']==0
        item['recorded_prefix_review']=reviewed
        item['assessment']='independent_recorded_prefix_review';item['checks_passed']=all(checks.values());reviews.append(item)
    passed=all(r.get('checks_passed',False) for r in reviews)
    report=dict(manifest_sha256=a.sha256,phase=a.phase,script_sha256=sha(Path(__file__)),reviewed=len(reviews),ids=[r['id'] for r in rows],
        raw_pass=sum((r['raw_result'] or {}).get('result')=='PASS' for r in reviews),eligible_for_formal=passed,checks_passed=sum(r.get('checks_passed',False) for r in reviews),
        scope='Source/configuration, updated contract witnesses and sampled recorded prefixes; task failures retained, not continuous physical safety or complete identity-ablation proof',rows=reviews)
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['rows','ids']},indent=2))


if __name__=='__main__':main()
