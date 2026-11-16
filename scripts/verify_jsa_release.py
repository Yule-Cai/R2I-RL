#!/usr/bin/env python3
"""Verify scientific source data and archived receiver evidence without ROS/Gazebo."""
from pathlib import Path
from collections import Counter
from types import SimpleNamespace
import csv,hashlib,importlib.util,json,sys,tarfile
ROOT=Path(__file__).resolve().parents[1];ART=ROOT/'artifacts/jsa_20261116';DATA=ART
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def obj(v):
 if isinstance(v,dict):return SimpleNamespace(**{k:obj(x) for k,x in v.items()})
 if isinstance(v,list):return [obj(x) for x in v]
 return v
def main():
 manifest=json.loads((ART/'release_manifest.json').read_text())
 checked=0
 for name,record in manifest['files'].items():
  if record.get('rebuild_output'):continue
  p=ROOT/name;assert p.is_file() and p.stat().st_size==record['bytes'] and sha(p)==record['sha256'],name;checked+=1
 data=json.loads((DATA/'source_data_manifest.json').read_text());assert len(data)==93
 for name,r in data.items():assert sha(DATA/name)==r['sha256'] and (DATA/name).stat().st_size==r['bytes']
 freeze=json.loads((ART/'input_freeze_v1.json').read_text());assert len(freeze['files_sha256'])==197
 for name,digest in freeze['files_sha256'].items():assert sha(ART/'frozen_inputs'/name)==digest,name
 cohort=ART/'frozen_inputs/formal_dualmap_20261108_v2/manifest.json';assert sha(cohort)=='bb749f5d886ef5a0009966ce4d180b3c58e2c1d503213a918cedd94534dfef20'
 rows=list(csv.DictReader((DATA/'source_data/formal2970.csv').open()));assert len(rows)==len({r['id'] for r in rows})==2970
 assert sum(r['raw_task_result']=='PASS' for r in rows)==1997 and sum(r['epoch2_assessed']=='True' for r in rows)==2943
 m=json.loads(cohort.read_text());assert {r['id'] for r in rows}=={r['id'] for r in m['rows']}
 archive=ART/'receiver12_raw.tar.gz';assert sha(archive)=='5cc90acf9a44655738b42e165e55f20ec4a2c44fa8a90566b910d921cf42f43f'
 auth_path=DATA/'evidence/code_data_story_audit_20261115/frozen_source/15_authorization.py';s=importlib.util.spec_from_file_location('frozen_auth',auth_path);module=importlib.util.module_from_spec(s);s.loader.exec_module(module)
 with tarfile.open(archive) as t:
  inventory=json.load(t.extractfile('files.json'));assert len(inventory)==500
  saved={}
  for member in t:
   if member.name=='files.json':continue
   assert member.isfile() and member.name in inventory and '..' not in Path(member.name).parts
   b=t.extractfile(member).read();assert dict(bytes=len(b),sha256=hashlib.sha256(b).hexdigest())==inventory[member.name]
   saved[member.name]=b
  assert set(saved)==set(inventory)
  progress=json.loads(saved['authorization_completion_20261114/progress.json']);assert progress['closed']==12
  outcomes=Counter();refusals=0
  for rec in progress['rows']:
   base=rec['output'];outcomes[rec['result']['result']]+=1
   message=json.loads(saved[base+'/injected_authorization.json']);a=obj(message)
   assert module.verify_authorization(a.path,a.path_digest,a.token_id,a.epoch,a.snapshot_hash)[0]
   events=[json.loads(l) for l in saved[base+'/events.jsonl'].splitlines()]
   assert any(e['kind']=='guard_state' and e.get('reason')=='authorization-identity-mismatch' for e in events);refusals+=1
  assert outcomes=={'PASS':8,'FAIL':4} and refusals==12
 sys.path.insert(0,str(DATA))
 from verify_system_evidence import verify
 from verify_binding_latch import verify as binding
 system=verify();latch=binding()
 report=dict(release_files_checked=checked,scientific_source_files=93,frozen_inputs_verified=197,cohort_rows=2970,raw_receiver_files=500,receiver_refusals=refusals,receiver_original_outcomes=dict(outcomes),system=system,latch=latch,new_robot_experiments=0)
 (ART/'verification_latest.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
