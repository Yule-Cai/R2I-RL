"""Run the retained independent ROS deserializer after the campaign has ended."""
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
status=json.loads((ROOT/'status.json').read_text())
assert status['phase'] in ('complete','bounded_limit_reached','inspection_required','stopped_by_marker')
reports=json.loads((ROOT/'decode_results.json').read_text()) if (ROOT/'decode_results.json').exists() else []
for run in json.loads((ROOT/'manifest.json').read_text())['runs']:
    root=ROOT/'runs'/run['id']/'trace'
    if not (root/'mission/snapshot_channel/proxy.jsonl').exists():continue
    output=root/'service_cdr_decoded.json'
    if output.exists():continue
    command=[sys.executable,str(ROOT/'analysis/decode_snapshot_proxy_cdr_v1.py'),
        '--root',str(root/'mission'),'--output',str(output),
        '--log','snapshot_channel/proxy.jsonl','--log','guard_events.jsonl']
    with (root/'service_decode.log').open('x') as stream:
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
    reports.append(dict(id=run['id'],returncode=result.returncode))
    print(json.dumps(reports[-1]),flush=True)
(ROOT/'decode_results.json').write_text(json.dumps(reports,indent=2)+'\n')
