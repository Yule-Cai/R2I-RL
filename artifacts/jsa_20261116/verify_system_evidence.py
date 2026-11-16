"""Recompute the added scientific endpoints without mutating pinned inputs."""
from pathlib import Path
from collections import Counter
import ast,hashlib,json,math,statistics,csv,tarfile,importlib.util
from types import SimpleNamespace
HERE=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def quantile(values,p):
 a=sorted(values);pos=(len(a)-1)*p;i=int(pos);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(pos-i)
def objectify(v):
 if isinstance(v,dict):return SimpleNamespace(**{k:objectify(x) for k,x in v.items()})
 if isinstance(v,list):return [objectify(x) for x in v]
 return v
def check_diagnostic_archive(path,index,expected):
 with tarfile.open(path) as archive:
  inv=json.load(archive.extractfile('files.json'));payload={}
  for member in archive:
   if member.name=='files.json':continue
   assert member.isfile() and '..' not in Path(member.name).parts
   b=archive.extractfile(member).read();assert dict(bytes=len(b),sha256=hashlib.sha256(b).hexdigest())==inv[member.name]
   payload[member.name]=b
 assert set(payload)==set(inv) and len(payload)==expected['files_verified']
 load=lambda name:json.loads(payload[name])
 rows=lambda name:[json.loads(line) for line in payload[name].splitlines()]
 prefix='authorization_diagnostic_20261115/'
 record=load(prefix+f'run_{index}.json');base=record['output']
 raw=load(base+'/result.json');assert raw['result']==expected['new_raw_task_result'] and not raw['formal_denominator']
 meta=load(base+'/manifest.json')
 for i,(name,digest) in enumerate(meta['source_sha256'].items()):assert hashlib.sha256(payload[base+f'/source/{i:02d}_'+Path(name).name]).hexdigest()==digest
 pin=load(prefix+'source_pin.json');assert hashlib.sha256(payload[prefix+'source_pin.json']).hexdigest()==expected['source_pin_sha256']
 for name,digest in pin['files'].items():assert hashlib.sha256(payload[prefix+name]).hexdigest()==digest
 injected=load(base+'/injected_authorization.json')
 authfile=HERE/'evidence/code_data_story_audit_20261115/frozen_source/15_authorization.py'
 assert sha(authfile)=='8c6ca852b65b35ec6513c56d6192d0b1fc4b63abc112ad91629e24fb4c491be6'
 spec=importlib.util.spec_from_file_location('pinned_diagnostic_auth',authfile);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 a=objectify(injected);assert module.verify_authorization(a.path,a.path_digest,a.token_id,a.epoch,a.snapshot_hash)[0]
 ledger=rows(base+'/observer_ledger.jsonl');guard=rows(base+'/guard_publication_ledger.jsonl');events=rows(base+'/events.jsonl')
 barrier=next(r for r in ledger if r['event']=='positive_control_barrier');sent=next(r for r in ledger if r['event']=='probe_send_begin');end=next(r for r in ledger if r['event']=='probe_window_closed')
 assert barrier['mono_ns']<sent['mono_ns']<end['mono_ns']
 for observer in ['primary','redundant']:
  assert any(r['event']=='path_receipt' and r['observer']==observer and r['points']>0 and r['digest']==barrier['digest'] and r['mono_ns']<barrier['mono_ns'] for r in ledger)
  assert not any(r['event']=='path_receipt' and r['observer']==observer and r['points']>0 and sent['mono_ns']<=r['mono_ns']<=end['mono_ns'] for r in ledger)
 entry=next(r for r in guard if r['event']=='authorization_enter' and r['message']==injected and sent['mono_ns']<=r['mono_ns']<=end['mono_ns']);aid=entry['authorization_id']
 assert any(r['event']=='authorization_exit' and r['authorization_id']==aid and r['reason']=='authorization-identity-mismatch' for r in guard)
 assert not any(r['event']=='path_publish_begin' and r['active_authorization_id']==aid and r['points']>0 for r in guard)
 renewed=next(r for r in guard if r['event']=='authorization_enter' and r['mono_ns']>entry['mono_ns']);assert renewed['message']['epoch']==2
 assert not any(r['event']=='path_publish_begin' and r['points']>0 and entry['mono_ns']<=r['mono_ns']<renewed['mono_ns'] for r in guard)
 assert any(r['event']=='path_publish_begin' and r['active_authorization_id']==renewed['authorization_id'] and r['points']>0 for r in guard)
 assert any(r['kind']=='task_release' and r['epoch']==2 and r['receipt_ns']>renewed['mono_ns'] for r in events)
 return len(payload)
def verify():
 data=HERE/'source_data'
 inherited=json.loads((HERE/'evidence/inherited_source_data_manifest_v2.json').read_text());current=json.loads((HERE/'source_data_manifest.json').read_text())
 for n,r in inherited.items():assert current[n]==r and sha(HERE/n)==r['sha256']
 trial={r['id']:r for r in csv.DictReader((data/'formal2970.csv').open())}
 timing=json.loads((data/'formal_recovery_timing.json').read_text());rr=json.loads((data/'formal_recovery_timing_rows.json').read_text());assert len(rr)==len({r['id'] for r in rr})==1485
 for r in rr:assert (r['arm'],r['task_result'])==(trial[r['id']]['arm'],trial[r['id']]['raw_task_result'])
 for arm,n,missing in [('full',675,2),('geometry_only',405,0),('rho0',405,1)]:
  rows=[r for r in rr if r['arm']==arm];assert len(rows)==n
  a=[r['commit_to_renewed_release_receipt_ms'] for r in rows if r['commit_to_renewed_release_receipt_ms'] is not None];assert len(a)==n-missing
  expected=timing['by_arm'][arm]['commit_to_renewed_release_receipt_ms']
  for key,q in [('p50_ms',.5),('p95_ms',.95),('p99_ms',.99)]:assert abs(quantile(a,q)-expected[key])<1e-9
  label={'full':'Full','geometry_only':'Geometry only','rho0':'Zero margin'}[arm]
  cells=[f'{label} & {len(a)}/{n} & {missing}']+[f'{expected[k]/1000:.3f}' for k in ['p50_ms','p95_ms','p99_ms']]
  assert [*cells[0].split(' & '), *cells[1:]] in list(csv.reader((HERE/'recovery_timing_reference.csv').open()))
 # All accepted and rejected performance conditions, including warmups.
 perf=data/'ros_admission_performance';base=perf/'ros_perf_formal_v1';manifest=json.loads((base/'manifest.json').read_text());summary=json.loads((base/'summary.json').read_text())
 for n,h in manifest['source_sha256'].items():assert sha(perf/'source'/n)==h
 allrows=[]
 for layout,rules,segments in manifest['cells']:
  rows=[json.loads(l) for l in (base/f'{layout}_{rules}_{segments}'/'raw.jsonl').read_text().splitlines()];assert len(rows)==140
  assert {(r['outcome'],r['repeat'],r['cache']) for r in rows}=={(o,i,c) for o in ['admitted','early_crossing'] for i in range(-5,30) for c in ['cold','reuse']}
  for r in rows:
   d,q=[r['results'][k]['verdict'] for k in ['direct','indexed']];assert d==q and d['execute']==r['expected_accept']
   assert r['results']['indexed']['cache_hit']==(r['cache']=='reuse')
   token=hashlib.sha256(f"{r['epoch']}|{r['snapshot_hash']}|{r['digest']}".encode()).hexdigest()[:24]
   assert d['token_id']==(token if r['expected_accept'] else '')
  allrows+=rows
 measured=[r for r in allrows if not r['warmup']];assert len(allrows)==840 and len(measured)==720 and summary['pairs']==720 and summary['candidates']==1440
 for cell in summary['summaries']:
  values=[r for r in measured if (r['layout'],r['rules'],r['segments'],r['outcome'],r['cache'])==(cell['layout'],cell['rules'],cell['segments'],cell['outcome'],cell['cache'])];assert len(values)==30
  for mode in ['direct','indexed']:
   for field in ['receipt_ms','node_ms','service_wait_ms','full_gate_ms','cache_ms']:
    a=[r['results'][mode][field] for r in values]
    for name,want in [('median',statistics.median(a)),('p95',quantile(a,.95)),('min',min(a)),('max',max(a))]:assert abs(want-cell[f'{mode}_{field}'][name])<1e-8
  if cell['outcome']=='admitted' and cell['cache']=='reuse':
   label={'separated':'Separated','corridor':'Corridor','dense':'Dense overlap'}[cell['layout']]
   values=[cell[f'{mode}_receipt_ms'][stat] for mode in ['direct','indexed'] for stat in ['median','p95']]
   assert [str(cell['rules']),str(cell['segments']),label]+[f'{v:.1f}' for v in values] in list(csv.reader((HERE/'admission_cost_reference.csv').open()))
 # Reconstruct the actual durable/observation/HOLD events for the matched moving trace.
 moving=[]
 for arm in ['full','geometry_only']:
  d=data/'moving_update'/arm;read=lambda n:[json.loads(l) for l in (d/n).read_text().splitlines()]
  review=json.loads((d/'review.json').read_text());manager=read('manager_events.jsonl');guard=read('guard_events.jsonl');events=read('events.jsonl');wal=read('wal.jsonl')
  commit=next(r for r in manager if r['event']=='durable_commit' and r['epoch']==2);t0=commit['t_ns'];end=t0+5_000_000_000
  assert any(r.get('decision')=='COMMIT' and r.get('epoch')==2 and r['target_hash']==commit['snapshot_hash'] for r in wal)
  external=next(r for r in events if r['kind']=='external_update_without_anticipatory_stop');assert external['receipt_ns']<t0
  assert not any(r['kind']=='interlock_hold' and external['receipt_ns']<=r['receipt_ns']<t0 for r in events)
  decoded=json.loads((d/'service_cdr_decoded.json').read_text())['decoded']
  obs=next(r for r in guard if r['event']=='channel_snapshot_response_callback' and r['current_generation'] and r['epoch']==2 and r['active_hash']==commit['snapshot_hash'] and t0<=r['t_ns']<end and not decoded[r['cdr_sha256']]['fields']['degraded'])
  assert abs((obs['t_ns']-t0)/1e6-review['first_fresh_observation_after_commit_ms'])<1e-8
  holds=[r for r in guard if r['event']=='hold_decision' and t0<=r['t_ns']<end]
  if arm=='full':assert holds and holds[0]['reason']=='epoch-disagreement' and abs((holds[0]['t_ns']-t0)/1e6-84.643806)<1e-8
  else:assert not holds and review['intervention_status']=='right_censored_at_window_end'
  commands=list(csv.DictReader((d/'command_native_timeline.csv').open()));assert sum(r['observed_nonzero']=='True' and obs['t_ns']<=int(r['rmw_publish_ns'])<end for r in commands)==review['post_observation_nonzero_native_publications']
  moving.append(review)
 assert moving[0]['seed']==moving[1]['seed']==52031 and moving[0]['parameter_sha256']==moving[1]['parameter_sha256'] and all(r['task_complete'] for r in moving)
 # Added diagnostic runs have their own denominator and never rewrite original labels.
 diag_path=data/'diagnostic3_review.json'
 if diag_path.exists():
  diag=json.loads(diag_path.read_text());assert len(diag)==3
  original={r['trial_id']:r for r in json.loads((data/'authorization12_review.json').read_text())['rows']}
  flags={i for i,r in original.items() if 'probe caused executable path release' in (r['raw_error'] or '')}
  assert {r['original_failed_trial_id'] for r in diag}==flags
  for r in diag:
   assert r['independently_verified'] and r['original_result_unchanged']=='FAIL' and r['unchanged_runtime_core_files']==20
   assert r['explicit_identity_rejection'] and r['fresh_readmission_and_release'] and r['no_implicit_release_before_new_authorization']
   assert not r['nonempty_publications_caused_by_stale_authorization'] and not r['nonempty_primary_receipts_during_probe'] and not r['nonempty_redundant_receipts_during_probe']
   p=data/'diagnostic_raw'/f"diagnostic_{r['index']}_raw.tar.gz";assert sha(p)==r['raw_archive_sha256']
   check_diagnostic_archive(p,r['index'],r)
  diagnostic=dict(trials=3,task_outcomes=dict(Counter(r['new_raw_task_result'] for r in diag)),explicit_refusals=3,new_readmissions=3,old_failed_outcomes_preserved=True)
 else:diagnostic=dict(status='pending')
 result=dict(performance_pairs=720,performance_calls=1440,performance_cells=24,full_recovery_timed=673,full_recovery_missing=2,moving_update_pairs=1,diagnostic=diagnostic,source_data_files=len(current),inherited_source_files_unchanged=len(inherited))
 (HERE/'qa').mkdir(exist_ok=True)
 (HERE/'qa/system_evidence_check.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':print(json.dumps(verify(),indent=2))
