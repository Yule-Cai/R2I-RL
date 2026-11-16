"""Independent standard-library audit of recorded rows, not ROS replay.

Recompute every descriptive statistic and content-addressed token; assert the
complete predeclared matrix/cache state. Actual ROS message/path validation
was performed during each trial, not reconstructed by this offline checker.
"""
import collections
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parent
CAMPAIGN = ROOT / 'ros_perf_formal_v1'
manifest = json.loads((CAMPAIGN/'manifest.json').read_text())
summary = json.loads((CAMPAIGN/'summary.json').read_text())
assert summary['passed'] and summary['pairs'] == 720 and summary['candidates'] == 1440
for path, expected in manifest['source_sha256'].items():
    assert hashlib.sha256((ROOT/'source'/path).read_bytes()).hexdigest() == expected
all_rows = []
for layout, rules, segments in manifest['cells']:
    rows = [json.loads(line) for line in
            (CAMPAIGN/f'{layout}_{rules}_{segments}'/'raw.jsonl').read_text().splitlines()]
    assert len(rows) == 140 and [r['job'] for r in rows] == list(range(1, 141))
    expected = {(o, t, c) for o in ('admitted', 'early_crossing')
                for t in range(-5, 30) for c in ('cold', 'reuse')}
    assert {(r['outcome'], r['repeat'], r['cache']) for r in rows} == expected
    epochs = collections.defaultdict(list)
    for r in rows:
        assert r['warmup'] == (r['repeat'] < 0)
        assert r['results']['indexed']['cache_hit'] == (r['cache'] == 'reuse')
        d, q = (r['results'][mode]['verdict'] for mode in ('direct', 'indexed'))
        assert d == q and d['execute'] == r['expected_accept']
        assert (d['epoch'], d['snapshot_hash']) == (r['epoch'], r['snapshot_hash'])
        token = hashlib.sha256(f"{r['epoch']}|{r['snapshot_hash']}|{r['digest']}".encode()).hexdigest()[:24]
        assert d['token_id'] == (token if r['expected_accept'] else '')
        assert len(r['snapshot_hash']) == len(r['digest']) == 64
        for mode in ('direct', 'indexed'):
            v = r['results'][mode]
            assert v['has_map'] and v['job'] == r['job']
            for metric in ('receipt_ms', 'node_ms', 'service_wait_ms', 'full_gate_ms', 'cache_ms'):
                assert math.isfinite(v[metric]) and v[metric] >= 0
            assert v['node_ms'] >= v['service_wait_ms']
        epochs[r['epoch']].append(r)
    assert len(epochs) == len({v[0]['snapshot_hash'] for v in epochs.values()}) == 70
    for v in epochs.values():
        assert len(v) == 2 and {r['cache'] for r in v} == {'cold', 'reuse'}
        assert v[0]['snapshot_hash'] == v[1]['snapshot_hash']
    all_rows.extend(rows)
formal = [r for r in all_rows if not r['warmup']]
assert len(all_rows) == 840 and len(formal) == 720 and len(summary['summaries']) == 24
for s in summary['summaries']:
    cell = [r for r in formal if (r['layout'], r['rules'], r['segments'], r['outcome'], r['cache']) ==
            (s['layout'], s['rules'], s['segments'], s['outcome'], s['cache'])]
    assert len(cell) == s['pairs'] == 30
    for metric in ('receipt_ms', 'node_ms', 'service_wait_ms', 'full_gate_ms', 'cache_ms'):
        for mode in ('direct', 'indexed'):
            values = sorted(r['results'][mode][metric] for r in cell)
            pos = .95*(len(values)-1)
            i, fraction = int(pos), pos-int(pos)
            recomputed = dict(median=statistics.median(values), min=values[0], max=values[-1],
                p95=values[i]*(1-fraction)+values[min(i+1, len(values)-1)]*fraction)
            for stat, value in recomputed.items():
                assert abs(value-s[f'{mode}_{metric}'][stat]) < 1e-8
    speedup = statistics.median(r['results']['direct']['receipt_ms']/
                               r['results']['indexed']['receipt_ms'] for r in cell)
    assert abs(speedup-s['paired_receipt_speedup_median']) < 1e-8
result = dict(passed=True, total_pairs=840, measured_pairs=720, warmup_pairs=120,
              measured_calls=1440, cells=24, source_hashes=len(manifest['source_sha256']),
              scope='offline record/statistics/token/cache audit; not transport replay')
(ROOT/'independent_audit.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
