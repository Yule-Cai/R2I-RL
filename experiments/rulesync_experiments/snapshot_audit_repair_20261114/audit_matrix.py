"""Local independent geometry/WAL reanalysis. Never rewrites raw outcomes."""
import argparse
from collections import Counter
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

from snapshot_history import replay_snapshots

RG_SHA = '8fd86f7b1fa3887bc12928475d4aab56c3b79fdadddf8e05750d09484e4ee0ff'


def load_hash_module(path):
    assert hashlib.sha256(path.read_bytes()).hexdigest() == RG_SHA
    spec = importlib.util.spec_from_file_location('pinned_identity', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def inside(point, box, margin=0.0):
    return all(box['bmin'][i] - margin < point[i] < box['bmax'][i] + margin
               for i in range(3))


def segment_penetrates(a, b, box, margin=0.0):
    """Open AABB interior via slab intervals; touching is not penetration."""
    lo, hi = 0.0, 1.0
    for i in range(3):
        lower, upper = box['bmin'][i] - margin, box['bmax'][i] + margin
        delta = b[i] - a[i]
        if delta == 0:
            if not lower < a[i] < upper:
                return False
        else:
            t0, t1 = sorted(((lower - a[i]) / delta, (upper - a[i]) / delta))
            lo, hi = max(lo, t0), min(hi, t1)
            if lo >= hi:
                return False
    return lo < hi


def profile(rows, volumes, margin):
    return dict(samples=len(rows),
                raw_inside=sum(any(inside(x['pose'], v) for v in volumes) for x in rows),
                margin_inside=sum(any(inside(x['pose'], v, margin) for v in volumes) for x in rows),
                raw_segment_penetrations=sum(any(segment_penetrates(a['pose'], b['pose'], v)
                                                 for v in volumes) for a, b in zip(rows, rows[1:])),
                margin_segment_penetrations=sum(any(segment_penetrates(a['pose'], b['pose'], v, margin)
                                                    for v in volumes) for a, b in zip(rows, rows[1:])))


def audit_one(root, binding, hash_fn):
    run = root / binding['run']
    raw = json.loads((run / 'result.json').read_text())
    manifest = json.loads((run / 'manifest.json').read_text())
    cases = list((run / 'source').glob('*.json'))
    assert len(cases) == 1
    case = json.loads(cases[0].read_text())
    for original, digest in manifest['source_sha256'].items():
        assert hashlib.sha256((run / 'source' / Path(original).name).read_bytes()).hexdigest() == digest
    assert hashlib.sha256(cases[0].read_bytes()).hexdigest() == manifest['case_sha256']
    assert manifest['speed_xy_mps'] == binding['row']['speed']
    history = replay_snapshots(run / 'authority/wal.jsonl', hash_fn)
    state = json.loads((run / 'authority/state.json').read_text())
    last = history[max(history)]
    assert state['epoch'] == max(history)
    assert state['volumes'] == last['volumes'] and state['snapshot_hash'] == last['snapshot_hash']
    for f in (run / 'admissions').glob('*_admitted.json'):
        admission = json.loads(f.read_text())
        snapshot = history[admission['epoch']]
        assert admission['snapshot_hash'] == snapshot['snapshot_hash']
        assert admission['volumes'] == [v['id'] for v in snapshot['volumes']]
    rows = [json.loads(line) for line in (run / 'trajectory.jsonl').read_text().splitlines()]
    commits = sorted(history.values(), key=lambda v: v['receipt_ns'])
    for row in rows:
        assert len(row['pose']) == 3 and all(math.isfinite(v) for v in row['pose'])
        current = max((v for v in commits if v['receipt_ns'] <= row['receipt_ns']), key=lambda v: v['epoch'])
        assert row['epoch'] == current['epoch'], 'trajectory epoch differs from WAL time'
    by_epoch = {}
    for epoch in sorted({row['epoch'] for row in rows}):
        subset = [row for row in rows if row['epoch'] == epoch]
        by_epoch[str(epoch)] = profile(subset, history[epoch]['volumes'], case['flight']['body_radius_m'])
    old_audit = raw.get('audit') or {}
    old_e2 = old_audit.get('executed_epoch2_vs_committed_snapshot') or {}
    e2 = [row for row in rows if row['epoch'] == 2]
    retired = []
    if 2 in history:
        retired = [v for v in history[1]['volumes'] if v not in history[2]['volumes']]
    retired_profile = profile(e2, retired, case['flight']['body_radius_m'])
    actual_bad = any(p['raw_inside'] or p['raw_segment_penetrations'] for p in by_epoch.values())
    false_positive = bool(old_e2.get('raw_violation_samples', 0) > 0
                          and e2 and not actual_bad and retired_profile['raw_inside'] > 0)
    events = [json.loads(line) for line in (run / 'events.jsonl').read_text().splitlines()]
    votes = [e for e in events if e.get('event') == 'volume_vote']
    return dict(**binding, original_result=raw['result'], original_phase=raw['phase'],
                original_error=raw.get('error'), source_and_wal_verified=True,
                snapshot_volume_ids={str(k): [v.get('id') for v in s['volumes']] for k, s in history.items()},
                profiles=by_epoch, retired_epoch1_profile_in_epoch2=retired_profile,
                original_epoch2_raw_violation_samples=old_e2.get('raw_violation_samples'),
                old_audit_retired_rule_false_positive=false_positive,
                last_vote=({k: v for k, v in votes[-1].items() if k in ['rule', 'vote', 'reason']} if votes else None),
                actual_committed_raw_penetration=actual_bad,
                epoch2_assessed=bool(e2),
                cross_epoch_segments_not_interpolated=sum(a['epoch'] != b['epoch'] for a, b in zip(rows, rows[1:])),
                goal_error_m=(raw.get('arrival') or {}).get('independent_goal_error_m'),
                original_physical_penetration_samples=old_audit.get('executed_physical_penetration_samples'),
                scope='Recorded ground-truth polyline vs active AABBs; does not certify continuous motion or re-audit mesh collisions')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--identity-source', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    inventory = json.loads((args.input / 'files.json').read_text())
    for name, record in inventory.items():
        data = (args.input / name).read_bytes()
        assert len(data) == record['bytes'] and hashlib.sha256(data).hexdigest() == record['sha256']
    module = load_hash_module(args.identity_source)
    hash_fn = lambda volumes: module.snapshot_hash(module.volume_snapshot_items(volumes))
    bindings = json.loads((args.input / 'bindings.json').read_text())
    reviewed = [audit_one(args.input, binding, hash_fn) for binding in bindings]
    summary = dict(rows=len(reviewed), verified_files=len(inventory),
                   original_outcomes=dict(Counter(x['original_result'] for x in reviewed)),
                   snapshot_epoch2_assessed=sum(x['epoch2_assessed'] for x in reviewed),
                   snapshot_epoch2_unassessed=sum(not x['epoch2_assessed'] for x in reviewed),
                   old_audit_retired_rule_false_positives=sum(x['old_audit_retired_rule_false_positive'] for x in reviewed),
                   observed_active_raw_penetration_trials=sum(x['actual_committed_raw_penetration'] for x in reviewed),
                   observed_active_margin_penetration_trials=sum(any(p['margin_inside'] or p['margin_segment_penetrations']
                                                                     for p in x['profiles'].values()) for x in reviewed),
                   original_outcomes_unchanged=True,
                   identity_source_sha256=RG_SHA,
                   note='Raw FAIL stays FAIL; corrected snapshot geometry is a separate retrospective assessment.')
    (args.output / 'all140_review.json').write_text(json.dumps(reviewed, indent=2) + '\n')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    keys = ['scene', 'occ', 'speed', 'route', 'seed', 'result']
    columns = ['original_phase', 'epoch2_assessed', 'old_audit_retired_rule_false_positive',
               'actual_committed_raw_penetration', 'goal_error_m']
    with (args.output / 'all140_review.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys + columns)
        writer.writeheader()
        writer.writerows(dict(**{k: x['row'][k] for k in keys}, **{k: x[k] for k in columns}) for x in reviewed)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
