"""Collect completed attempts and audit them; never launch a robot experiment."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
REMOTE='/home/gamer/Projects/rulesync-ras-final-moving36-20261117'


def run(command,log):
    with log.open('x') as stream:
        return subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT).returncode


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--sync',action='store_true')
    parser.add_argument('--ids',nargs='*')
    args=parser.parse_args()
    manifest=json.loads((ROOT/'manifest.json').read_text())
    selected=[r for r in manifest['runs'] if not args.ids or r['id'] in args.ids]
    if args.sync:
        for item in selected:
            remote_path=REMOTE+'/runs/'+item['id']
            exists=subprocess.run(['ssh','-T','onexplayer','/usr/bin/test','-f',remote_path+'/closed.json']).returncode==0
            if not exists:continue
            local=ROOT/'runs'/item['id'];local.mkdir(parents=True,exist_ok=True)
            subprocess.run(['rsync','-az','onexplayer:'+remote_path+'/',str(local)+'/'],check=True)
    records=[]
    for item in selected:
        root=ROOT/'runs'/item['id']
        if not (root/'closed.json').exists():
            records.append(dict(id=item['id'],completed=False,assessed=False,reason='No closed attempt'))
            continue
        native=root/'native_audit'
        if not native.exists() and not (root/'native_audit.log').exists():
            run([sys.executable,str(ROOT/'analysis/audit_native.py'),'--root',str(root/'trace'),
                 '--output',str(native),'--require-complete-values'],root/'native_audit.log')
        window=root/'window_audit_final.json'
        if not window.exists() and (native/'summary.json').exists():
            run([sys.executable,str(ROOT/'analysis/audit_window.py'),'--root',str(root/'trace'),
                 '--native',str(native),'--output',str(window)],root/'window_audit.log')
        delivery=root/'delivery_audit_final.json'
        if not delivery.exists() and (root/'trace/service_cdr_decoded.json').exists():
            run([sys.executable,str(ROOT/'analysis/audit_delivery_window.py'),'--root',str(root/'trace'),
                 '--decoded',str(root/'trace/service_cdr_decoded.json'),'--output',str(delivery)],root/'delivery_audit_final.log')
        record=dict(id=item['id'],completed=True,
            window=json.loads(window.read_text()) if window.exists() else None,
            delivery=json.loads(delivery.read_text()) if delivery.exists() else None,
            native=json.loads((native/'summary.json').read_text()) if (native/'summary.json').exists() else None)
        record['assessed']=bool(record['window'] and record['window'].get('assessed')
                                and record['delivery'] and record['delivery'].get('passed'))
        records.append(record)
        print(json.dumps(dict(id=item['id'],assessed=record['assessed'],
            window_assessed=record['window'].get('assessed') if record['window'] else False,
            delivery_passed=record['delivery'].get('passed') if record['delivery'] else False)),flush=True)
    output=ROOT/'audit_index.json'
    output.write_text(json.dumps(dict(manifest_sha256=hashlib.sha256((ROOT/'manifest.json').read_bytes()).hexdigest(),
        planned=36,selected=len(selected),records=records),indent=2)+'\n')


if __name__=='__main__':main()
