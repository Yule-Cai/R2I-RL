"""Three bounded workers, one state writer, cooperative drain, no scientific retries."""
import argparse
from concurrent.futures import ThreadPoolExecutor,wait,FIRST_COMPLETED
import datetime
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tarfile
import time

from contract_v1 import ARMS,atomic,check_files,inspect,load,reconcile,sha

ROOT=Path('/home/gamer/Projects/rulesync-iotj-environments-20261028')


def archive_rows(out,m,digest,rows,label):
    target=out/'archives'/label;receipt=target.with_suffix('.json');target=target.with_suffix('.tar.gz')
    if receipt.exists():
        prior=json.loads(receipt.read_text());assert target.exists() and sha(target)==prior['archive_sha256']
        assert prior['manifest_sha256']==digest and prior['ids']==[r['id'] for r in rows]
        return prior
    assert not target.exists()
    files=set([out/'manifest.json',out/'pin.json'])
    files.update(p for p in (out/'tools').glob('*.py'))
    for row in rows:
        run=out/'runs'/row['phase']/row['id'];rec=inspect(run,m,digest,row,row['phase']);assert rec.get('status')=='done'
        files.update(p for p in run.rglob('*') if p.is_file())
    checks={str(p.relative_to(ROOT)):sha(p) for p in sorted(files)}
    target.parent.mkdir(exist_ok=True);partial=target.with_suffix('.partial')
    assert not partial.exists(),'Partial archive retained; inspect before rebuilding'
    with tarfile.open(partial,'w:gz') as t:
        for n in checks:t.add(ROOT/n,arcname=n,recursive=False)
    assert all(sha(ROOT/n)==h for n,h in checks.items())
    partial.rename(target)
    value=dict(archive=str(target.relative_to(ROOT)),archive_sha256=sha(target),manifest_sha256=digest,
        ids=[r['id'] for r in rows],files_sha256=checks,raw_bytes=sum((ROOT/n).stat().st_size for n in checks),archive_bytes=target.stat().st_size)
    atomic(receipt,value);return value


def approval(path,digest,ids):
    if not path.exists():return False
    a=json.loads(path.read_text())
    assert a['manifest_sha256']==digest and a['eligible_for_formal'] is True
    assert sorted(a['ids'])==sorted(ids) and a['reviewed']==len(ids)
    return True


def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--phase',choices=['preflight','formal'],required=True);p.add_argument('--max-new',type=int)
    a=p.parse_args();out=a.manifest.parent;m=load(a.manifest,a.sha256);check_files(ROOT,m)
    runtime=out/'runtime';runtime.mkdir(exist_ok=True)
    lock=(runtime/'queue.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    live=subprocess.check_output(['podman','ps','--format','{{.Names}}'],text=True).splitlines()
    assert not any(n.startswith('rulesync-dm2970-') for n in live),'Previous child still alive; do not reconcile/restart over it'
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:tokens=(proc/'cmdline').read_bytes().split(b'\0')
        except OSError:continue
        assert str(out/'tools/launch_v1.py').encode() not in tokens,'Previous owned launcher still alive; inspect before resuming'
    rows=m['rows'] if a.phase=='formal' else m['preflight_rows'];ids={r['id'] for r in rows}
    if a.phase=='formal':
        assert approval(runtime/'preflight_approval.json',a.sha256,[r['id'] for r in m['preflight_rows']]),'Independent preflight approval required'
    state_path=runtime/(a.phase+'_state.json');status_path=runtime/(a.phase+'_status.json');ledger=runtime/(a.phase+'_ledger.jsonl')
    state=json.loads(state_path.read_text()) if state_path.exists() else dict(manifest_sha256=a.sha256,attempts={})
    assert state['manifest_sha256']==a.sha256 and set(state['attempts']).issubset(ids)
    stop=False;active={};dispatched=0;fatal=None
    def drain(*_):
        nonlocal stop;stop=True
    signal.signal(signal.SIGTERM,drain);signal.signal(signal.SIGINT,drain)
    def event(kind,**fields):
        with ledger.open('a') as f:
            f.write(json.dumps(dict(kind=kind,at=datetime.datetime.now().astimezone().isoformat(),**fields))+'\n');f.flush();os.fsync(f.fileno())
    def progress(phase):
        by={arm:dict(done=0,passed=0,failed=0,total=sum(r['arm']==arm for r in rows)) for arm in ARMS}
        for row in rows:
            v=state['attempts'].get(row['id'])
            if v and v.get('status')=='done':
                b=by[row['arm']];b['done']+=1;b['passed' if v['passed'] else 'failed']+=1
        value=dict(cohort=m['cohort'],manifest_sha256=a.sha256,scope=a.phase,phase=phase,total=len(rows),
            done=sum(b['done'] for b in by.values()),active=len(active),active_keys=[r['key'] for r,d in active.values()],
            by_arm=by,error=fatal,independent_audit_complete=False,updated_at=datetime.datetime.now().astimezone().isoformat())
        atomic(status_path,value);return value
    for row in rows:
        path=out/'runs'/a.phase/row['id'];rec=inspect(path,m,a.sha256,row,a.phase)
        if rec.get('status')=='done':state['attempts'][row['id']]=reconcile(state['attempts'].get(row['id']),rec)
        elif row['id'] in state['attempts']:
            event('state_without_attempt_evidence',id=row['id'],prior=state['attempts'][row['id']]);raise ValueError('State/evidence mismatch; no silent reset')
        elif rec['kind']=='empty_prelaunch':
            dest=out/'preserved_empty'/f'{row["id"]}-{time.time_ns()}';dest.parent.mkdir(exist_ok=True);path.rename(dest)
            event('empty_prelaunch_preserved',id=row['id'],path=str(dest))
    atomic(state_path,state)
    sentinel=[r['id'] for r in m['rows'][:6]]
    startup_ok=a.phase!='formal' or approval(runtime/'startup_approval.json',a.sha256,sentinel)
    if not startup_ok and all(i in state['attempts'] for i in sentinel):
        progress('startup_checkpoint_pending_audit');return
    atomic(runtime/'process.json',dict(pid=os.getpid(),proc_stat=Path('/proc/self/stat').read_text(),
        cmdline=Path('/proc/self/cmdline').read_bytes().replace(b'\0',b' ').decode(),manifest_sha256=a.sha256,phase=a.phase))
    event('queue_start',manifest_sha256=a.sha256,phase=a.phase,verified_closed=len(state['attempts']),startup_approved=startup_ok)
    progress('running')
    def attempt(row,domain):
        cmd=[sys.executable,str(out/'tools/launch_v1.py'),'--manifest',str(a.manifest),'--sha256',a.sha256,'--phase',a.phase,'--key',row['key'],'--domain',str(domain)]
        try:q=subprocess.run(cmd,capture_output=True,text=True,timeout=1650)
        except subprocess.TimeoutExpired as exc:
            atomic(runtime/('launcher_timeout_'+row['id']+'.json'),dict(error=repr(exc),stdout=str(exc.stdout),stderr=str(exc.stderr)))
            raise RuntimeError('Launcher timeout; preserve and inspect live container before resuming')
        atomic(runtime/('launcher_'+row['id']+'.json'),dict(returncode=q.returncode,stdout=q.stdout,stderr=q.stderr))
        rec=inspect(out/'runs'/a.phase/row['id'],m,a.sha256,row,a.phase)
        assert rec.get('status')=='done','Launch failed before an identified attempt; stop without false completion'
        return rec
    domains=[220,221,222]
    with ThreadPoolExecutor(max_workers=3) as pool:
        for cid in dict.fromkeys(r['case_id'] for r in rows):
            group=[r for r in rows if r['case_id']==cid]
            todo=[r for r in group if r['id'] not in state['attempts']]
            if not startup_ok:todo=[r for r in todo if r['id'] in sentinel]
            if not todo:
                if all(r['id'] in state['attempts'] for r in group):
                    archived=archive_rows(out,m,a.sha256,group,a.phase+'__'+cid)
                    event('case_archive_verified',case_id=cid,sha256=archived['archive_sha256'])
                continue
            # Asset verification occurs at an empty case boundary; no hashing
            # workload is introduced into a live motion interval.
            prefix=m['scenes'][group[0]['scene']]['asset_root']+'/'
            for n,h in m['asset_files_sha256'].items():
                if n.startswith(prefix):assert sha(ROOT/n)==h,('Asset changed',n)
            pending=list(todo)
            while pending or active:
                if (runtime/'maintenance_hold.json').exists():stop=True
                while pending and domains and not stop and not fatal and (a.max_new is None or dispatched<a.max_new):
                    memory=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))*1024
                    if memory<6*2**30 or shutil.disk_usage(out).free<20*2**30:break
                    row=pending.pop(0);domain=domains.pop(0);event('attempt_dispatch',id=row['id'],key=row['key'],domain=domain)
                    active[pool.submit(attempt,row,domain)]=(row,domain);dispatched+=1
                if not active:
                    if stop or fatal or (a.max_new is not None and dispatched>=a.max_new):break
                    progress('waiting_for_resources');time.sleep(5);continue
                done,_=wait(active,timeout=10,return_when=FIRST_COMPLETED)
                for f in done:
                    row,domain=active.pop(f);domains.append(domain)
                    try:
                        rec=f.result();state['attempts'][row['id']]=rec;event('attempt_closed',id=row['id'],key=row['key'],result=rec);atomic(state_path,state)
                    except Exception as exc:fatal=repr(exc);event('attempt_blocked',id=row['id'],error=fatal)
                progress('blocked' if fatal else 'draining' if stop else 'running')
            if fatal or stop:break
            if all(r['id'] in state['attempts'] for r in group):
                result=archive_rows(out,m,a.sha256,group,a.phase+'__'+cid)
                event('case_archived',case_id=cid,sha256=result['archive_sha256'])
            if not startup_ok or (a.max_new is not None and dispatched>=a.max_new):break
    final='blocked' if fatal else 'paused' if stop else 'closed_pending_audit' if len(state['attempts'])==len(rows) else 'startup_checkpoint_pending_audit' if not startup_ok else 'paused_at_limit'
    value=progress(final);event('queue_exit',phase=final,done=value['done']);print(json.dumps(value),flush=True)
    if fatal:raise SystemExit(1)


if __name__=='__main__':main()
