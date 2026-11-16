#!/usr/bin/env python3
"""Validate frozen schedule and two-host audit equality, then summarize evidence."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import random
import statistics


def summarize(root):
    manifest = json.loads((root / 'cohort_manifest.json').read_text())
    mac = json.loads((root / 'independent_audit.json').read_text())
    host = json.loads((root / 'handheld_independent_audit.json').read_text())
    rows = [dict(arm=a, condition=c, direction=d, rep=r)
            for a, c, d, r in itertools.product(
                ['full', 'geometry_only'],
                ['normal', 'delayed_mask', 'invalid_candidate', 'restart'],
                [-1, 1], range(1, 4))]
    random.Random(20261012).shuffle(rows)
    assert manifest['seed'] == 20261012 and manifest['planned'] == 48
    assert manifest['no_retries'] and manifest['pilots_excluded']
    assert manifest['schedule'] == rows
    assert mac['audit_passed'] and host['audit_passed']
    assert mac['trials'] == host['trials'] and mac['groups'] == host['groups']
    attempts = [json.loads(s) for s in (root / 'attempts.jsonl').read_text().splitlines()]
    assert len(attempts) == len(mac['trials']) == 48
    params_hashes = set()
    for i, (planned, audited, attempt) in enumerate(zip(rows, mac['trials'], attempts), 1):
        assert audited['directory'] == f'trial_{i:02}'
        assert attempt['attempt'] == i and attempt['result'] == 'PASS'
        trial = root / audited['directory']
        saved = json.loads((trial / 'result.json').read_text())
        detail = json.loads((trial / 'manifest.json').read_text())
        assert detail['trial'] == planned
        assert all(audited[k] == planned[k] == attempt[k] for k in planned)
        assert not detail['pilot'] and not saved['pilot'] and saved['source_unchanged']
        assert detail['source_sha256']['/experiment/benchmark_nav2_combination.py'] == manifest['runner_sha256']
        assert hashlib.sha256((trial / 'source/00_benchmark_nav2_combination.py').read_bytes()).hexdigest() == manifest['runner_sha256']
        params_hashes.add(detail['parameters_sha256'])
        # Acquisition events use monotonic time, not wall time. The frozen
        # runner writes the manifest before its trial loop; do not compare
        # incompatible clocks to invent an independent predeclaration anchor.
        assert manifest['predeclared_wall_ns'] > 0
    assert len(params_hashes) == 1, 'Both arms must use identical Nav2 parameters'
    stats = {}
    for key in ['min_body_gap_m', 'max_sim_gap_s', 'independent_goal_error_m',
                'request_to_hold_ms', 'hold_to_zero_receipt_ms',
                'request_to_release_ms', 'commit_to_release_ms']:
        values = [r[key] for r in mac['trials']]
        stats[key] = dict(min=min(values), median=statistics.median(values), max=max(values))
    return dict(planned=48, completed=48, independent_audits_passed=48,
                two_host_numeric_equality=True,
                mac_geos=mac['geos_version'], handheld_geos=host['geos_version'],
                runner_sha256=manifest['runner_sha256'], shared_parameters_sha256=params_hashes.pop(),
                post_commit_samples=sum(r['post_commit_samples'] for r in mac['trials']),
                sampled_body_entries=sum(r['sampled_body_entries'] for r in mac['trials']),
                nonzero_after_first_zero_before_release=sum(r['nonzero_after_first_zero_before_release'] for r in mac['trials']),
                groups=mac['groups'], statistics=stats)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = summarize(args.root)
    text = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text)
