"""Attribute retained task endpoints to their recorded terminal/monitor layer."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent
records=[]
for planned in json.loads((ROOT/'manifest.json').read_text())['runs']:
    root=ROOT/'runs'/planned['id']/'trace/mission'
    if not (root/'result.json').exists():continue
    result=json.loads((root/'result.json').read_text())
    events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    guard=[json.loads(line) for line in (root/'guard_events.jsonl').read_text().splitlines()]
    renewals=[r for r in events if r['kind']=='task_release' and r['epoch']==2]
    renewal=renewals[0]['receipt_ns'] if renewals else None
    later_holds=[r for r in guard if r['event']=='hold_decision' and renewal and r['t_ns']>=renewal]
    terminal=next((r for r in events if r['kind']=='recovery_action_terminal'),None)
    records.append(dict(id=planned['id'],condition=planned['condition'],arm=planned['arm'],
        original_task_result=result['result'],raw_error=result.get('error'),
        renewed_release_ns=renewal,first_post_renewal_hold=later_holds[0] if later_holds else None,
        recovery_action_terminal=terminal,
        interpretation='Recorded trigger and terminal layer; upstream loss/scheduling root cause is not inferred'))
output=ROOT/'task_outcome_attribution.json'
output.write_text(json.dumps(records,indent=2)+'\n')
print(json.dumps(dict(recorded=len(records),pass_count=sum(r['original_task_result']=='PASS' for r in records),
                     fail_count=sum(r['original_task_result']!='PASS' for r in records))))
