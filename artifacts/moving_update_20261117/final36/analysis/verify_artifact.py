"""Recompute exported moving-window evidence without ROS or robot simulation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from audit_window import audit as audit_window
from audit_delivery_window import audit as audit_delivery


def equal(actual,expected,path='root'):
    if isinstance(actual,dict):
        assert set(actual)==set(expected),(path,set(actual)^set(expected))
        for key in actual:equal(actual[key],expected[key],path+'.'+key)
    elif isinstance(actual,list):
        assert len(actual)==len(expected),path
        for i,(a,b) in enumerate(zip(actual,expected)):equal(a,b,f'{path}[{i}]')
    elif isinstance(actual,float):
        assert isinstance(expected,(int,float)) and math.isclose(actual,expected,rel_tol=1e-10,abs_tol=1e-9),(path,actual,expected)
    else:assert actual==expected,(path,actual,expected)


def verify(root,allow_partial=False):
    files=json.loads((root/'artifact_files.json').read_text())
    for name,record in files.items():
        path=root/name
        assert path.is_file() and path.stat().st_size==record['bytes'],name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256'],name
    manifest=json.loads((root/'manifest.json').read_text())
    artifact_index=json.loads((root/'artifact_index.json').read_text())
    included=artifact_index['runs']
    assert len(included)==len(set(included))
    if not allow_partial:assert len(included)==manifest['planned_runs']==36
    verified=[]
    with tempfile.TemporaryDirectory(prefix='rulesync_native_recheck_') as temp:
        for run_id in included:
            run=root/'runs'/run_id
            if not artifact_index.get('assessment',{}).get(run_id,True):
                assert (run/'closed.json').exists(),run_id
                verified.append(dict(id=run_id,assessed=False,passed=None))
                continue
            native=Path(temp)/run_id
            command=[sys.executable,str(root/'analysis/audit_native_subset.py'),
                     '--root',str(run/'trace'),'--output',str(native),'--require-complete-values']
            process=subprocess.run(command,capture_output=True,text=True)
            assert process.returncode==0,(run_id,process.stdout,process.stderr)
            assert (native/'command_native_timeline.csv').read_bytes()==(run/'native_audit/command_native_timeline.csv').read_bytes(),run_id
            window=audit_window(run/'trace',native)
            equal(window,json.loads((run/'window_audit_final.json').read_text()),run_id+'.window')
            delivery=audit_delivery(run/'trace',run/'trace/service_cdr_decoded.json')
            equal(delivery,json.loads((run/'delivery_audit_final.json').read_text()),run_id+'.delivery')
            verified.append(dict(id=run_id,assessed=True,passed=True,final_commands=window['native_final_publications'],
                                 task_complete=window['task_complete']))
    return dict(passed=True,verified_files=len(files),included_attempts=len(verified),
                verified_runs=sum(r['assessed'] for r in verified),
                unassessed_attempts=sum(not r['assessed'] for r in verified),planned_runs=36,
                final_commands=sum(r.get('final_commands',0) for r in verified),records=verified,
                scope='Independent native-record, decoded-field, timing and sampled-footprint rechecks; no robot experiment')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parent.parent)
    parser.add_argument('--allow-partial',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();result=verify(args.root,args.allow_partial)
    if args.output:
        with args.output.open('x') as stream:
            json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))
