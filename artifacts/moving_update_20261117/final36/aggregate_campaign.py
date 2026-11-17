"""Recompute paired outcomes from audited records, preserving each denominator."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def distribution(values):
    values=[v for v in values if v is not None]
    return dict(n=len(values),median=statistics.median(values),minimum=min(values),maximum=max(values)) if values else dict(n=0,median=None,minimum=None,maximum=None)


def aggregate(partial=False):
    manifest=json.loads((ROOT/'manifest.json').read_text())
    index=json.loads((ROOT/'audit_index.json').read_text())
    preflight=json.loads((ROOT/'preflight.json').read_text())
    assert index['manifest_sha256']==sha(ROOT/'manifest.json')==preflight['manifest_sha256']
    by_id={r['id']:r for r in index['records']}
    records=[]
    for planned in manifest['runs']:
        record=by_id[planned['id']]
        if not record.get('completed'):
            records.append(dict(id=planned['id'],planned=planned,completed=False,assessed=False))
            continue
        root=ROOT/'runs'/planned['id']
        raw=json.loads((root/'trace/mission/manifest.json').read_text())
        assert raw['source_sha256']==preflight['source_sha256'],planned['id']
        assert raw['task_source_sha256']==planned['case']['sha256']
        assert raw['runner_sha256']==sha(ROOT/'source/scene_runner_generalization_v2.py')
        assert sha(root/'trace/mission/snapshot_channel/schedule.json')==planned['schedule_sha256']
        assert raw['seed']==planned['seed'] and raw['requested_speed_mps']==planned['speed_mps']
        record=dict(record,planned=planned)
        records.append(record)
    if not partial:
        assert len(records)==36 and all(r['completed'] for r in records),'Campaign incomplete'
    pairs=[]
    for pair_id in dict.fromkeys(r['pair_id'] for r in manifest['runs']):
        pair=[r for r in records if r['planned']['pair_id']==pair_id]
        assert len(pair)==2 and {r['planned']['arm'] for r in pair}=={'full','geometry_only'}
        assessed=all(r.get('assessed') for r in pair)
        if assessed:
            left,right=[r['window'] for r in pair]
            assert left['parameters_sha256']==right['parameters_sha256'],pair_id
            assert left['expected_path_invalidated']==right['expected_path_invalidated']
        pairs.append(dict(id=pair_id,assessed=assessed,run_ids=[r['id'] for r in pair]))
    groups=[]
    for condition in ('clean','blackout'):
        for arm in ('full','geometry_only'):
            rows=[r for r in records if r['planned']['condition']==condition and r['planned']['arm']==arm]
            assessed=[r['window'] for r in rows if r.get('assessed')]
            groups.append(dict(condition=condition,arm=arm,planned=len(rows),
                completed=sum(r['completed'] for r in rows),assessed=len(assessed),
                hold=sum(r['first_hold_ns'] is not None for r in assessed),
                hold_reasons=dict(collections.Counter(r['first_hold_reason'] for r in assessed)),
                old_execution_clear=sum(r['old_execution_clear_state_receipts_after_target_callback']>0 for r in assessed),
                post_target_nonzero=sum(r['nonzero_publications_after_target_callback']>0 for r in assessed),
                post_hold_nonzero=sum((r['nonzero_publications_after_hold'] or 0)>0 for r in assessed),
                fresh_readmission=sum(r['fresh_readmission'] for r in assessed),
                task_complete=sum(r['task_complete'] for r in assessed),
                task_fail=sum(not r['task_complete'] for r in assessed),
                nonempty_exposure_path_receipts=sum(r['nonempty_path_receipts_during_exposure'] for r in assessed),
                accepted_callbacks_while_held=sum(r['accepted_target_callbacks_after_hold_before_readmission'] for r in assessed),
                sampled_body_entries=sum(r['sampled_body_entries'] for r in assessed),
                target_callback_ms=distribution([r['commit_to_target_callback_ms'] for r in assessed]),
                hold_ms=distribution([r['commit_to_hold_ms'] for r in assessed]),
                callback_to_hold_ms=distribution([r['target_callback_to_hold_ms'] for r in assessed]),
                hold_to_zero_publication_ms=distribution([r['hold_to_zero_publication_ms'] for r in assessed]),
                hold_to_stable_zero_publication_ms=distribution([r['hold_to_stable_zero_publication_ms'] for r in assessed]),
                post_window_renewal_ms=distribution([r['post_window_renewal_ms'] for r in assessed])))
    result=dict(campaign=manifest['campaign'],manifest_sha256=sha(ROOT/'manifest.json'),
        planned=36,completed=sum(r['completed'] for r in records),assessed=sum(r.get('assessed',False) for r in records),
        planned_pairs=18,assessed_pairs=sum(p['assessed'] for p in pairs),partial=partial,
        groups=groups,pairs=pairs,records=records,
        native_commands_verified=sum(r['native']['final_publications'] for r in records if r.get('assessed')),
        sampled_body_entries=sum(r['window']['sampled_body_entries'] for r in records if r.get('assessed')),
        analysis_sha256={p.name:sha(p) for p in sorted((ROOT/'analysis').glob('*.py'))})
    path=ROOT/('aggregate_partial.json' if partial else 'aggregate.json')
    path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('records','pairs','analysis_sha256')},indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--partial',action='store_true')
    aggregate(parser.parse_args().partial)
