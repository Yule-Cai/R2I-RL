"""Create a compact, independently recheckable export of retained raw records."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'analysis'))
from export_native_subset import export


def copy(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,target)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--allow-partial',action='store_true')
    args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((ROOT/'manifest.json').read_text())
    for directory in ('source','baseline','inputs','configs','analysis'):
        for path in (ROOT/directory).glob('*'):
            selected=path.suffix in ('.py','.json','.diff','.csv')
            if directory=='analysis':
                selected=path.suffix=='.py' or path.name in (
                    'geometry_crosscheck.json','native_subset_qualification.json',
                    'observer_trace_index.csv','observer_logging_provenance.json')
            if path.is_file() and selected:
                copy(path,out/path.relative_to(ROOT))
    for name in ('manifest.json','preflight.json','PROTOCOL_ZH.md','campaign.py','prepare_final.py', 'test_relay_zero_delay.py', 'qualified_source_comparison.json',
                 'test_campaign_contract.py','decode_all.py','aggregate_campaign.py',
                 'collect_and_audit.py','export_review_artifact.py','audit_index.json'):
        copy(ROOT/name,out/name)
    for name in (('aggregate.json' if (ROOT/'aggregate.json').exists() else 'aggregate_partial.json'),'task_outcome_attribution.json'):
        if (ROOT/name).exists():copy(ROOT/name,out/name)
    included=[];assessment={};export_errors={}
    for run in manifest['runs']:
        source=ROOT/'runs'/run['id']
        final=source/'window_audit_final.json'
        delivery=source/'delivery_audit_final.json'
        if not (source/'closed.json').exists():continue
        target=out/'runs'/run['id']
        assessed=bool(final.exists() and delivery.exists() and json.loads(final.read_text()).get('assessed')
                      and json.loads(delivery.read_text()).get('passed'))
        assessment[run['id']]=assessed
        if assessed:
            export(source/'trace',target/'trace')
        else:
            target.mkdir(parents=True,exist_ok=False)
            export_errors[run['id']]='Attempt retained without a complete native/window/delivery assessment'
        for name in ('launch.json','closed.json','window_audit_final.json','delivery_audit_final.json'):
            if (source/name).exists():copy(source/name,target/name)
        for name in ('summary.json','command_native_timeline.csv'):
            if (source/'native_audit'/name).exists():copy(source/'native_audit'/name,target/'native_audit'/name)
        for name in ('trace_result.json','trace_commands.jsonl','command_witness.jsonl',
                     'command_witness.ready','command_witness.endpoints.json','cgroup_resources.json',
                     'service_cdr_decoded.json'):
            if (source/'trace'/name).exists():copy(source/'trace'/name,target/'trace'/name)
        mission=source/'trace/mission'
        for name in ('manifest.json','result.json','events.jsonl','guard_events.jsonl','manager_events.jsonl',
                     'wal.jsonl','world_pose.jsonl','localization.jsonl','params.yaml','channel_adapter.json',
                     'snapshot_channel/proxy.jsonl','snapshot_channel/schedule.json'):
            if (mission/name).exists():copy(mission/name,target/'trace/mission'/name)
        for path in (mission/'source').iterdir() if (mission/'source').exists() else []:
            if path.is_file():copy(path,target/'trace/mission/source'/path.name)
        included.append(run['id'])
    if not args.allow_partial:assert len(included)==36,'Refuse incomplete final export'
    index=dict(scope='Compact review artifact with unchanged native record subsets, raw application events and complete provenance',
        runs=included,assessment=assessment,export_errors=export_errors,planned_runs=36,partial=len(included)!=36,
        full_native_traces='Retained separately; SHA-256, original event count and original trace metadata accompany every native subset')
    (out/'artifact_index.json').write_text(json.dumps(index,indent=2)+'\n')
    (out/'README.md').write_text('''# RuleSync moving-update evidence

This archive supports independent recomputation of the moving-update experiment.
`manifest.json` fixes the 36 planned attempts and 18 pairs. Every exported run
includes raw WAL, manager/guard/adapter events, sampled poses, snapshot responses,
the frozen runtime extracts, exact final-command correlations and original outcomes.

Run the evidence verification with Python 3.9 or newer:

```sh
python3 analysis/verify_artifact.py
```

No ROS installation, simulation or external Python dependency is needed for this
verification. The retained independent decoder can additionally deserialize the
captured application CDR using the pinned ROS image; decoded fields and canonical
hashes are supplied here. These are application serializations, not packet capture.

`native_relevant.jsonl` contains unchanged original records for command and
authorization topics plus all endpoint initialization records. The subset verifier
reconstructs publication/take/value associations and compares the complete command
timeline with the independently audited full trace. Every subset links the full
native trace SHA-256 and its unmodified trace metadata. Full CTF and native traces
are retained separately; this archive does not claim to contain every middleware event.

The time window ends five seconds after COMMIT. The common stop at its end is
separate from measured monitor intervention. Callback entry, HOLD, command
publication and simulator middleware receipt are software endpoints. Sampled body
nonentry and task arrival are distinct outcomes; failed tasks remain failed.

Requests before initial release or after the measured window without a response
are indexed separately from the active experiment. Every active-period request and every captured response
used by the measured impairment is verified. Historical receiver-probe outcomes
and their separate diagnostics are linked by `analysis/observer_trace_index.csv`.

The original simulator assets are identified by `preflight.json` and the retained
scene source manifests. Obtain them from their upstream distributions under their
original licenses. `campaign.py` records the workstation-specific execution setup;
the evidence verification above does not launch that campaign.
''')
    if args.allow_partial:
        with (out/'README.md').open('a') as stream:
            stream.write('\nThis is an internal PARTIAL export for audit qualification. Use `--allow-partial` when verifying.\n')
    files={str(path.relative_to(out)):dict(bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
           for path in sorted(out.rglob('*')) if path.is_file()}
    (out/'artifact_files.json').write_text(json.dumps(files,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),runs=len(included),files=len(files))))


if __name__=='__main__':main()
