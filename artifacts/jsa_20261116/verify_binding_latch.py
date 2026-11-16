"""Re-examine retained ROS records and frozen handlers; no ROS/Gazebo launch."""
import ast,copy,hashlib,importlib.util,json,math,tarfile
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'qa'
PINNED=ROOT/'evidence/code_data_story_audit_20261115/frozen_source'
def obj(v):
 if isinstance(v,dict):return NS(**{k:obj(x) for k,x in v.items()})
 if isinstance(v,list):return [obj(x) for x in v]
 return v

def verify():
 source=PINNED/'15_authorization.py';assert hashlib.sha256(source.read_bytes()).hexdigest()=='8c6ca852b65b35ec6513c56d6192d0b1fc4b63abc112ad91629e24fb4c491be6'
 spec=importlib.util.spec_from_file_location('review_auth',source);auth=importlib.util.module_from_spec(spec);spec.loader.exec_module(auth)
 assert hashlib.sha256((PINNED/'11_guard_node.py').read_bytes()).hexdigest()=='368563f9e62929d3554d22ede3ccc9bddc73edd5e3c518d83ded51cd5796771f'
 tree=ast.parse((PINNED/'11_guard_node.py').read_text());methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
 ns=dict(verify_authorization=auth.verify_authorization,math=math,HOLD='HOLD')
 for name in ['_on_authorization','_reject_authorization']:exec(compile(ast.Module(body=[methods[name]],type_ignores=[]),'<frozen_receiver>','exec'),ns)
 class Facade:
  _on_authorization=ns['_on_authorization'];_reject_authorization=ns['_reject_authorization']
  def __init__(self):self.cleared=False;self._guard=NS(clear_execution=self.clear);self._latched_hold=False;self._token_id='positive';self.publications=[];self.reasons=[]
  def clear(self):self.cleared=True
  def _set_state(self,state,reason,degraded,clearance):self.reasons.append(reason)
  def _publish_stop(self):self.publications.append(dict(points=0))
  def get_logger(self):return NS(warning=lambda s:None)
 result=[]
 for i in range(3):
  archive=ROOT/'source_data/diagnostic_raw'/f'diagnostic_{i}_raw.tar.gz'
  with tarfile.open(archive) as t:
   rec=json.load(t.extractfile(f'authorization_diagnostic_20261115/run_{i}.json'));base=rec['output']
   rows=lambda n:[json.loads(l) for l in t.extractfile(base+'/'+n).read().splitlines()]
   ledger=rows('observer_ledger.jsonl');guard=rows('guard_publication_ledger.jsonl');events=rows('guard_events.jsonl');injected=json.load(t.extractfile(base+'/injected_authorization.json'))
   sent=next(r for r in ledger if r['event']=='probe_send_begin');old=next(r for r in guard if r['event']=='authorization_enter' and r['message']==injected and r['mono_ns']>=sent['mono_ns']);exit=next(r for r in guard if r['event']=='authorization_exit' and r['authorization_id']==old['authorization_id']);renew=next(r for r in guard if r['event']=='authorization_enter' and r['mono_ns']>exit['mono_ns'])
   fresh=[r for r in events if r['event']=='snapshot_observed' and exit['mono_ns']<r['t_ns']<renew['mono_ns'] and r['authority']==[renew['message']['epoch'],renew['message']['snapshot_hash']]]
   pubs=[r for r in guard if r['event']=='path_publish_begin' and r['points']>0 and exit['mono_ns']<=r['mono_ns']<renew['mono_ns']];assert fresh and not pubs
   a=obj(renew['message']);assert auth.verify_authorization(a.path,a.path_digest,a.token_id,a.epoch,a.snapshot_hash)[0]
   bad=copy.deepcopy(a);bad.path.poses[len(bad.path.poses)//2].pose.position.x+=.01
   decision=auth.verify_authorization(bad.path,bad.path_digest,bad.token_id,bad.epoch,bad.snapshot_hash);assert decision[:2]==(False,'authorization-path-digest-mismatch')
   assert (bad.epoch,bad.snapshot_hash,bad.path_digest,bad.token_id)==(a.epoch,a.snapshot_hash,a.path_digest,a.token_id)
   fake=Facade();fake._on_authorization(bad);assert fake.cleared and fake._latched_hold and fake.reasons==['authorization-path-digest-mismatch'] and all(p['points']==0 for p in fake.publications)
   result.append(dict(diagnostic_index=i,fresh_usable_snapshot_observations_without_new_authorization=len(fresh),observed_latch_interval_s=(renew['mono_ns']-exit['mono_ns'])/1e9,nonempty_publications_before_new_authorization=0,offline_content_mutation_refused=decision[1],actual_frozen_receiver_method_rejects_content_mutation=True))
 summary=dict(rows=result,live_latch_evidence='Existing ROS diagnostic records: repeated usable observations while awaiting a new authorization; not a new outage-and-restoration experiment',content_evidence='Captured-message mutation and frozen handler calls with recording infrastructure; not a live ROS content-ablation campaign',new_ros_experiments=0)
 expected=json.loads((ROOT/'evidence/binding_and_latch_expected.json').read_text());assert summary==expected
 assert [r['fresh_usable_snapshot_observations_without_new_authorization'] for r in result]==[11,11,8]
 OUT.mkdir(exist_ok=True)
 (OUT/'binding_and_latch_check.json').write_text(json.dumps(summary,indent=2)+'\n');return summary
if __name__=='__main__':print(json.dumps(verify(),indent=2))
