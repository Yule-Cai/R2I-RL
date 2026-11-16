"""Stream-verify all selected 360-run bytes and audit injection semantics."""
import argparse
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import tarfile


def benchmark_contract(text):
    tree = ast.parse(text)
    node = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'admit')
    publishes = [x for x in ast.walk(node) if isinstance(x, ast.Call)
                 and isinstance(x.func, ast.Attribute) and x.func.attr == 'publish']
    return dict(publish_arguments=[ast.unparse(x.args[0]) for x in publishes],
                epoch_references=sum(isinstance(x, ast.Name) and x.id == 'epoch' for x in ast.walk(node)),
                has_authorization_message='ExecutionAuthorization' in ast.unparse(node))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    receipt = json.loads((args.input / 'receipt.json').read_text())
    archive = args.input / 'selected_raw_evidence.tar.gz'
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == receipt['sha256']
    with tarfile.open(archive) as tar:
        inventory = json.load(tar.extractfile('files.json'))
        bindings = json.load(tar.extractfile('bindings.json'))
    records = {b['run']: dict(key=b['key'], source_files={}, hashes={}) for b in bindings}
    seen = set()
    with tarfile.open(archive, 'r|gz') as tar:
        for member in tar:
            assert member.isfile() and '..' not in Path(member.name).parts
            if member.name not in inventory:
                assert member.name in ['files.json', 'bindings.json']
                continue
            payload = tar.extractfile(member).read()
            digest = hashlib.sha256(payload).hexdigest()
            assert dict(bytes=len(payload), sha256=digest) == inventory[member.name], member.name
            seen.add(member.name)
            run = next((name for name in records if member.name.startswith(name + '/')), None)
            if run is None:  # launcher record sits one directory above position_1
                continue
            target = records[run]
            relative = member.name[len(run) + 1:]
            target['hashes'][relative] = digest
            if relative == 'result.json':
                target['result'] = json.loads(payload)
            elif relative == 'manifest.json':
                target['manifest'] = json.loads(payload)
            elif relative.startswith('source/'):
                target['source_files'][Path(relative).name] = digest
                if relative.endswith('_benchmark_nav2_combination.py'):
                    target['admit_contract'] = benchmark_contract(payload.decode())
                if relative.endswith('scene_runner_mechpairs_v1.py'):
                    text = payload.decode()
                    target['old_path_admit_call'] = 'bench.admit(node,initial_path,1)' in text
                    target['same_path_epoch_wait_call'] = 'bench.admit(node,path2,1)' in text
            elif relative == 'events.jsonl':
                selected = []
                for line in payload.splitlines():
                    event = json.loads(line)
                    if event.get('kind') in ['gate_verdict', 'stale_candidate_verdict', 'out_of_order_replay',
                                             'commit_reply', 'task_release', 'post_update_watch']:
                        selected.append(event)
                target['events'] = selected
    assert seen == set(inventory)
    reviewed = []
    for run, target in sorted(records.items()):
        raw, manifest = target['result'], target['manifest']
        assert raw['runtime_binding']['key'] == target['key'] == manifest['runtime_binding']['key']
        assert not raw['formal_denominator'] and not manifest['formal_denominator']
        for i, (original, digest) in enumerate(manifest['source_sha256'].items()):
            copied = f'{i:02d}_' + Path(original).name
            assert target['source_files'][copied] == digest, (run, copied)
        contract = target['admit_contract']
        assert contract == dict(publish_arguments=['path'], epoch_references=1, has_authorization_message=False)
        mode, arm, case, speed, seed = target['key'].split('|')
        row = dict(run=run, mode=mode, arm=arm, case=case, speed=float(speed), seed=int(seed),
                   original_result=raw['result'], source_verified=True,
                   original_runner_sha256=target['source_files']['scene_scene_runner_mechpairs_v1.py'],
                   result_sha256=target['hashes']['result.json'], events_sha256=target['hashes']['events.jsonl'])
        if mode == 'stale_candidate':
            marker = next(e for e in target['events'] if e['kind'] == 'stale_candidate_verdict')
            prior = [e for e in target['events'] if e['kind'] == 'gate_verdict' and e['receipt_ns'] < marker['receipt_ns']]
            gate = prior[-1]
            assert target['same_path_epoch_wait_call']
            row.update(classification='Path republished; epoch argument only changes observer wait',
                       actual_stale_authorization_message_sent=False,
                       epoch2_geometry_admitted=raw['stale_candidate_geometry'] == 'epoch2_admitted',
                       last_gate_execute=gate['execute'], last_gate_epoch=gate.get('epoch'),
                       last_gate_reason=gate['reason'], original_verdict=raw['stale_candidate_verdict'])
        elif mode == 'out_of_order':
            marker = next(e for e in target['events'] if e['kind'] == 'out_of_order_replay')
            prior = [e for e in target['events'] if e['kind'] == 'gate_verdict' and e['receipt_ns'] < marker['receipt_ns']]
            gate = prior[-1]
            assert target['old_path_admit_call']
            row.update(classification='Old Path resubmitted to current gate; not an old release/CLEAR response replay',
                       actual_stale_authorization_message_sent=False,
                       last_gate_execute=gate['execute'], last_gate_epoch=gate.get('epoch'),
                       last_gate_reason=gate['reason'], original_verdict=raw['out_of_order_stale_release'])
        else:
            row.update(classification='External-update observation; no claimed identity-specific latency advantage',
                       hold_observed=raw.get('external_hold_observed'), hold_latency_s=raw.get('hold_latency_s'))
        reviewed.append(row)
    assert len(reviewed) == 360 and len({r['run'] for r in reviewed}) == 360
    # Exact, balanced declared DEV matrix, not success-selected rows.
    settings = {(r['case'], r['speed'], r['seed']) for r in reviewed}
    assert len(settings) == 60
    expected = {(m, a, c, s, seed) for m in ['external_update', 'stale_candidate', 'out_of_order']
                for a in ['full', 'geometry_only'] for c, s, seed in settings}
    assert {(r['mode'], r['arm'], r['case'], r['speed'], r['seed']) for r in reviewed} == expected
    stale = [r for r in reviewed if r['mode'] == 'stale_candidate']
    order = [r for r in reviewed if r['mode'] == 'out_of_order']
    summary = dict(rows=360, verified_files=len(seen), raw_archive_sha256=receipt['sha256'],
                   original_outcomes=dict(Counter(r['original_result'] for r in reviewed)),
                   source_versions=dict(Counter(r['original_runner_sha256'] for r in reviewed)),
                   stale_candidate_rows=120,
                   stale_candidate_actual_gate_accepts=sum(r['last_gate_execute'] for r in stale),
                   stale_candidate_actual_gate_rejections=dict(Counter(r['last_gate_reason'] for r in stale if not r['last_gate_execute'])),
                   stale_candidate_geometry_positive_controls=sum(r['epoch2_geometry_admitted'] for r in stale),
                   out_of_order_rows=120,
                   out_of_order_gate_rejections=dict(Counter(r['last_gate_reason'] for r in order)),
                   qualified_real_old_authorization_or_CLEAR_replays=0,
                   original_results_unchanged=True,
                   scope='Source/injection/event semantics review; not a complete trajectory/collision safety audit.')
    dest = args.input / 'review_v1'
    dest.mkdir(exist_ok=False)
    (dest / 'all360_review.json').write_text(json.dumps(reviewed, indent=2) + '\n')
    (dest / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
