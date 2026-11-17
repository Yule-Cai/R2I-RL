#!/usr/bin/env python3
"""Independently verify the complete final moving-update cohort without ROS."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
ART=ROOT/'artifacts/moving_update_20261117/final36'
EXPECTED='5947f35dd0ca096052d4f7293bf3aacf75e52ae02a2defeede633f0c7991baf6'


def main():
    assert hashlib.sha256((ART/'manifest.json').read_bytes()).hexdigest()==EXPECTED
    manifest=json.loads((ART/'manifest.json').read_text())
    for name,digest in manifest['inputs_sha256'].items():
        assert hashlib.sha256((ART/name).read_bytes()).hexdigest()==digest,name
    sys.path.insert(0,str(ART/'analysis'))
    spec=importlib.util.spec_from_file_location('final_moving_artifact',ART/'analysis/verify_artifact.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.verify(ART)
    assert result['passed'] and result['verified_runs']==result['included_attempts']==36
    assert result['final_commands']==58211
    assert sum(r['task_complete'] for r in result['records'])==35
    summary=json.loads((ART/'aggregate.json').read_text())
    assert summary['assessed_pairs']==18 and summary['sampled_body_entries']==0
    assert sum(r['window']['fresh_readmission'] for r in summary['records'])==36
    failures=[r for r in summary['records'] if not r['window']['task_complete']]
    assert len(failures)==1 and failures[0]['id']=='r01_s52703_blackout_full'
    assert '0.21082709241385617' in failures[0]['window']['retained_task_error']
    print(json.dumps(dict(passed=True,verified_runs=36,paired_settings=18,
                         renewed_releases=36,task_arrivals=35,task_failures=1,
                         final_commands=58211,sampled_body_entries=0,
                         verified_artifact_files=result['verified_files'],
                         new_robot_experiments=0),indent=2))


if __name__=='__main__':main()
