#!/usr/bin/env python3
"""ROS-free audit of raw edge-loop receipts; outputs JSON, no plot or manuscript edits."""
import argparse
import hashlib
import json
import math
from pathlib import Path


EXPECTED = {'empty-snapshot-motion', 'moving-old-authorization-revoked', 'crossing-denied',
    'detour-resume', 'guard-process-loss-watchdog', 'restart-no-silent-resume',
    'fresh-authorization-after-restart', 'detour-goal-arrival', 'command-watchdog-motion',
    'controller-process-loss-plant-watchdog'}


def clearance_rectangle(x, y):
    if -.3 <= x <= .6 and -.5 <= y <= .5:
        return -min(x + .3, .6 - x, y + .5, .5 - y)
    return math.hypot(max(-.3 - x, 0., x - .6), max(-.5 - y, 0., y - .5))


def audit(directory, source_root):
    summary = json.loads((directory / 'summary.json').read_text())
    manifest = json.loads((directory / 'manifest.json').read_text())
    assert summary['passed'] and len(summary['checks']) == len(EXPECTED)
    assert {row['name'] for row in summary['checks']} == EXPECTED
    assert all(row['passed'] for row in summary['checks'])
    assert not (directory / 'failure.json').exists()
    assert manifest['discovery'] == 'LOCALHOST'
    assert manifest['ros'] == 'jazzy'
    hashes_match = all(hashlib.sha256((source_root / path).read_bytes()).hexdigest() == digest
        for path, digest in manifest['source_sha256'].items())
    assert hashes_match, 'deployed source mismatch'
    rows = [json.loads(line) for line in (directory / 'receipts.jsonl').read_text().splitlines()]
    events = {row['name']: row for row in rows if row['kind'] == 'event'}
    plant = [row for row in rows if row['kind'] == 'plant']
    assert len(plant) == summary['pose_samples']
    assert abs(max(row['elapsed_s'] for row in plant) - summary['max_plant_tick_s']) < 1e-9
    assert abs(sum(row['discarded_s'] for row in plant) - summary['discarded_integration_s']) < 1e-6
    timeline = ['motion-before-update', 'commit-request', 'commit-response-observed',
        'revocation-stop-observed', 'crossing-denied-submitted', 'detour-resume-submitted',
        'guard-process-interrupted', 'watchdog-stop-observed',
        'fresh-authorization-after-restart-submitted', 'goal-arrival',
        'command-watchdog-motion-submitted', 'controller-process-interrupted',
        'plant-command-watchdog-stop-observed']
    assert all(events[a]['at_ns'] <= events[b]['at_ns'] for a, b in zip(timeline, timeline[1:]))
    active = [row for row in plant if row['at_ns'] >= events['commit-response-observed']['at_ns']]
    minimum = min(clearance_rectangle(*row['pose'][:2]) for row in active)
    assert minimum > .2
    # Stream callback may process a pre-response plant message after response;
    # independently timed oracle can differ, but neither is allowed below rho.
    assert summary['min_observed_clearance_after_commit_m'] > .2
    assert math.dist(events['goal-arrival']['pose'], (2., 0.)) < .07
    pairs = [('guard-process-interrupted', 'watchdog-stop-observed'),
             ('controller-process-interrupted', 'plant-command-watchdog-stop-observed')]
    stops = {}
    for interrupted, observed in pairs:
        end = events[observed]['at_ns']
        zeros = [row for row in plant if end <= row['at_ns'] <= end + 500_000_000]
        assert len(zeros) >= 10 and all(row['velocity'] == [0., 0.] for row in zeros)
        assert max(math.dist(row['pose'][:2], zeros[0]['pose'][:2]) for row in zeros) < 1e-6
        stops[interrupted] = (end - events[interrupted]['at_ns']) / 1e6
    assert any(row['kind'] == 'guard' and row['state'] == 2 and
        row['reason'] == 'epoch-disagreement' and events['commit-request']['at_ns'] <=
        row['received_ns'] <= events['revocation-stop-observed']['at_ns'] for row in rows)
    assert any(row['kind'] == 'gate' and not row['execute'] and
        row['reason'] == 'gate:KEEP_OUT-violation' and
        events['crossing-denied-submitted']['at_ns'] <= row['received_ns'] <=
        events['detour-resume-submitted']['at_ns'] for row in rows)
    # Recovery must remain stationary until a newly submitted candidate. This
    # directly checks receipts, rather than trusting only the summary verdict.
    recovery = [row for row in plant if events['watchdog-stop-observed']['at_ns'] <=
        row['at_ns'] <= events['fresh-authorization-after-restart-submitted']['at_ns']]
    assert len(recovery) >= 10 and recovery[-1]['at_ns'] - recovery[0]['at_ns'] >= 700_000_000
    assert all(row['velocity'] == [0., 0.] for row in recovery)
    assert max(math.dist(row['pose'][:2], recovery[0]['pose'][:2]) for row in recovery) < 1e-6
    resources = [json.loads(line) for line in (directory / 'resources.jsonl').read_text().splitlines()]
    peak_rss = max(sum(p['rss_bytes'] for p in row['processes']) for row in resources)
    peak_temp = max(value for row in resources for value in row['temperatures_mC'].values()) / 1000.
    min_available = min(int(line.split()[1]) * 1024 for row in resources
        for line in row['meminfo'].splitlines() if line.startswith('MemAvailable:'))
    if manifest.get('thermal_abort_C') is not None:
        assert peak_temp < manifest['thermal_abort_C']
    return dict(run=directory.name, passed=True, checks=len(EXPECTED), source_hashes_match=hashes_match,
        cpu_affinity=manifest.get('cpu_affinity'),
        fastdds_builtin_transports=manifest.get('fastdds_builtin_transports'),
        pose_readiness_new_proposals=sum(row.get('name') == 'new-proposal-after-pose-readiness-delay'
            for row in rows if row['kind'] == 'event'),
        elapsed_s=summary['elapsed_s'], pose_samples=len(plant), resource_samples=len(resources),
        min_observed_clearance_m=minimum, max_plant_tick_s=summary['max_plant_tick_s'],
        discarded_integration_s=summary['discarded_integration_s'],
        peak_summed_process_rss_MiB=peak_rss / 2**20,
        min_system_available_GiB=min_available / 2**30, highest_sampled_temperature_C=peak_temp,
        commit_response_to_observed_stop_ms=(events['revocation-stop-observed']['at_ns'] -
            events['commit-response-observed']['at_ns']) / 1e6,
        process_interruption_to_observed_zero_ms=stops,
        caveats=['summed RSS double-counts shared pages', 'sampled pose clearance only',
                 'timings are receipt observations, not hard bounds or physical braking',
                 'one fixed layout, repeated fresh starts'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', type=Path, nargs='+')
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    print(json.dumps([audit(directory, args.source_root) for directory in args.directories], indent=2))


if __name__ == '__main__':
    main()
