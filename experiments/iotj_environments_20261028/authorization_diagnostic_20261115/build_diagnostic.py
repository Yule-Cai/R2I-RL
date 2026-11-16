"""Build fresh diagnostic-only copies; never edit the original probe or runtime."""
from pathlib import Path
import ast,hashlib,json,shutil
HERE=Path(__file__).resolve().parent;OLD=HERE.parent/'authorization_probe_20261114'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a[:80],s.count(a));return s.replace(a,b)
assert sha(OLD/'scene_runner_authorization_probe.py')=='65bf9136d1cd406d8d6e823b6821d9ac64d4018363bd620cf590aa5fd527f2cb'
s=(OLD/'scene_runner_authorization_probe.py').read_text()
s=once(s,'from probe_logic import make_probe','from probe_logic import make_probe\nfrom observer_ledger import ObserverLedger, info_fields, probe_classification\nfrom rulesync3d.authorization import path_digest')
s=once(s,'        self.map_record=map_record','        self.observer_ledger=ObserverLedger(root)\n        self.map_record=map_record')
s=once(s,'    def sample_gpu(self):', '''    def on_executable(self, message, info=None):
        self.observer_ledger.record('path_receipt', observer='primary',
            points=len(message.poses), digest=path_digest(message),
            metadata=info_fields(info), message=message_to_ordereddict(message))
        return super().on_executable(message)

    def sample_gpu(self):''')
s=once(s,"[Path(__file__),Path(__file__).with_name('probe_logic.py')]","[Path(__file__),Path(__file__).with_name('probe_logic.py'),Path(__file__).with_name('observer_ledger.py'),Path(__file__).with_name('diagnostic_guard_role.py')]")
s=once(s,"        def captured_forward(message):\n            node.auth_probe_forwards.append(dict(receipt_ns=time.monotonic_ns(),points=len(message.poses)))",'''        def captured_forward(message, info):
            row=node.observer_ledger.record('path_receipt', observer='redundant',
                points=len(message.poses), digest=path_digest(message),
                metadata=info_fields(info), message=message_to_ordereddict(message))
            node.auth_probe_forwards.append(dict(receipt_ns=row['mono_ns'],
                points=len(message.poses), digest=row['digest'], metadata=row['metadata']))''')
s=once(s,"        procs.start('guard',[sys.executable,str(role),'--role','guard','--ros-args',","        procs.start('guard',[sys.executable,str(Path(__file__).with_name('diagnostic_guard_role.py')),'--role','guard','--ros-args',")
s=once(s,"            sent=time.monotonic_ns()",'''            fresh_digest=path_digest(fresh_authorization.path)
            base.wait(node,lambda:node.observer_ledger.has_positive('primary',fresh_digest)
                and node.observer_ledger.has_positive('redundant',fresh_digest),5.)
            publishers=node.get_publishers_info_by_topic('/rulepatch/executable_path')
            assert len(publishers)==1, 'diagnostic requires one executable-path publisher'
            node.observer_ledger.record('positive_control_barrier',digest=fresh_digest,
                token_id=str(fresh_authorization.token_id),epoch=int(fresh_authorization.epoch),
                publishers=[dict(node_name=p.node_name,node_namespace=p.node_namespace,
                    publisher_gid=bytes(p.endpoint_gid).hex()) for p in publishers])
            sent=time.monotonic_ns()
            node.observer_ledger.record('probe_send_begin',sent_ns=sent,
                message=message_to_ordereddict(injected))''')
s=once(s,"            assert not any(f['points']>0 for f in forwards), 'probe caused executable path release'",'''            diagnostic=probe_classification(node.observer_ledger.records,sent,time.monotonic_ns())
            diagnostic.update(probe_kind=kind,states=states,forwarded_paths=forwards,
                positive_control_barrier=True,sent_ns=sent,source='persistent dual observer ledger')
            write(root/'authorization_probe_diagnostic.json',diagnostic)
            node.observer_ledger.record('probe_window_closed',sent_ns=sent,
                receipt_only_endpoint_clear=diagnostic['receipt_only_endpoint_clear'])
            assert not any(f['points']>0 for f in forwards), 'probe caused executable path release' ''')
ast.parse(s);(HERE/'scene_runner_diagnostic.py').write_text(s)
shutil.copy2(OLD/'probe_logic.py',HERE/'probe_logic.py')
l=(OLD/'run_authorization_probe.py').read_text().replace('authorization_probe_20261114','authorization_diagnostic_20261115').replace('scene_runner_authorization_probe.py','scene_runner_diagnostic.py')
l=once(l,'65bf9136d1cd406d8d6e823b6821d9ac64d4018363bd620cf590aa5fd527f2cb',sha(HERE/'scene_runner_diagnostic.py'))
l=once(l,"name=name+\"-authprobe\"","name=name+\"-authdiag\"")
l=once(l,"f\"{opt.mech_mode}|{opt.monitor_arm}|{opt.case_id}|{opt.speed}|{opt.seed}\"","f\"diagnostic-v1|{opt.probe_kind}|{opt.mech_mode}|{opt.monitor_arm}|{opt.case_id}|{opt.speed}|{opt.seed}|{opt.source_pin_sha}\"")
l=once(l,"assert opt.tag.startswith('authprobe_')","assert opt.tag.startswith('authprobe_diagnostic_20261115_')")
ast.parse(l);(HERE/'run_diagnostic.py').write_text(l)
files={p.name:sha(p) for p in sorted(HERE.glob('*.py'))}
pin=dict(scope='Three fixed failed settings; diagnostic instrumentation only; independent DEV, no replacement of original outcomes',files=files,original_runner_sha256=sha(OLD/'scene_runner_authorization_probe.py'),runtime_predicates_modified=False)
(HERE/'source_pin.json').write_text(json.dumps(pin,indent=2)+'\n')
old=json.loads((HERE.parent/'authorization_completion_20261114/manifest.json').read_text());reviews=json.loads((HERE.parents[2]/'output/jsa_review_triage_20261115/observer_diagnosis.json').read_text());bad={r['trial_id'] for r in reviews['rows'] if 'probe caused executable path release' in (r['error'] or '')}
rows=[]
for r in old['rows']:
 if r['id'] not in bad:continue
 x={k:v for k,v in r.items() if k!='id'};x.update(original_failed_trial_id=r['id'],source_pin_sha256=sha(HERE/'source_pin.json'),diagnostic_version='v1')
 x['id']=hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':')).encode()).hexdigest();rows.append(x)
assert len(rows)==3
manifest=dict(cohort='AuthorizationDiagnostic3_DEV_20261115_v1',scope='Three predeclared observer-flag settings; same cases, seed, speed and runtime; no retries; no new performance-rate denominator',rows=rows)
(HERE/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(dict(source_pin_sha256=sha(HERE/'source_pin.json'),manifest_sha256=sha(HERE/'manifest.json'),trials=len(rows))))
