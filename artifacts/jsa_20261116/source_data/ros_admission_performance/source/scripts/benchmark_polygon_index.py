#!/usr/bin/env python3
"""Paired, same-host direct/BVH benchmark; no ROS or real-time claims.

Three all-admitted layouts distinguish separated rules, corridor pruning,
and the deliberately adversarial dense-overlap case. The manifest is written
before timing. Raw paired runs, construction, cold calls and reuse are retained.
Never compare these timings to historical data from another host.
"""

import argparse
import csv
import hashlib
import json
import platform
from pathlib import Path
import statistics
import time

import numpy as np

from rulesync3d.gate2d import PreparedPolygonIndex, path_intersects_polygons


def polygons_for(layout, count):
    if layout == 'separated':
        return [np.array([(30+i, 30), (30.4+i, 30),
                          (30.4+i, 30.4), (30+i, 30.4)])
                for i in range(count)]
    if layout == 'corridor':
        return [np.array([(x-.2, y-.2), (x+.2, y-.2),
                          (x+.2, y+.2), (x-.2, y+.2)])
                for i in range(count)
                for x, y in [(1 + 18*i/max(count-1, 1),
                               1.0 if i % 2 else -1.0)]]
    base = np.array([(-2, -2), (22, -2), (22, -1),
                     (-1, -1), (-1, 2), (-2, 2)], dtype=float)
    return [base + (0, i*1e-4) for i in range(count)]


def elapsed_ms(fn):
    begin = time.perf_counter_ns()
    value = fn()
    return (time.perf_counter_ns() - begin)/1e6, value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--rule-counts', type=int, nargs='+', default=[1, 10, 100])
    parser.add_argument('--segment-counts', type=int, nargs='+', default=[10, 100, 1000])
    parser.add_argument('--layouts', nargs='+', choices=['separated', 'corridor', 'dense'],
                        default=['separated', 'corridor', 'dense'])
    parser.add_argument('--warmups', type=int, default=2)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--rho', type=float, default=.2)
    args = parser.parse_args()
    if (args.repeats < 2 or args.warmups < 0
            or min(args.rule_counts + args.segment_counts) < 1):
        parser.error('require positive sizes, repeats >=2 and warmups >=0')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if (args.output_dir/'manifest.json').exists():
        parser.error('choose a fresh output directory; do not overwrite a campaign')
    source = Path(__file__).resolve().parents[1]/'rulesync3d/rulesync3d/gate2d.py'
    manifest = {
        'rule_counts': args.rule_counts, 'segment_counts': args.segment_counts,
        'layouts': args.layouts, 'rho': args.rho, 'warmups': args.warmups,
        'repeats': args.repeats, 'platform': platform.platform(),
        'python': platform.python_version(), 'numpy': np.__version__,
        'clock': 'same-host perf_counter_ns', 'pair_order': 'alternating',
        'scope': 'offline all-admitted continuous geometry; no ROS/occupancy/transport',
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'benchmark_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    (args.output_dir/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    rows, summaries = [], []
    with (args.output_dir/'raw.csv').open('w', newline='') as stream:
        fields = ['layout', 'rule_count', 'segment_count', 'repeat', 'first',
                  'build_ms', 'direct_ms', 'indexed_ms', 'cold_ms',
                  'direct_reject', 'indexed_reject']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for layout in args.layouts:
            for r in args.rule_counts:
                polygons = polygons_for(layout, r)
                for n in args.segment_counts:
                    path = np.column_stack([np.linspace(0., 20., n+1), np.zeros(n+1)])
                    prepared = PreparedPolygonIndex(polygons, args.rho)
                    direct = lambda: path_intersects_polygons(path, polygons, args.rho)
                    indexed = lambda: prepared.intersects_path(path)
                    assert not direct() and not indexed(), 'all-admitted design violated'
                    for _ in range(args.warmups):
                        direct(); indexed()
                    cell = []
                    for repeat in range(args.repeats):
                        first = 'direct' if repeat % 2 == 0 else 'indexed'
                        order = [direct, indexed] if first == 'direct' else [indexed, direct]
                        timings = [elapsed_ms(fn) for fn in order]
                        d, q = timings if first == 'direct' else timings[::-1]
                        build_ms, cold_index = elapsed_ms(
                            lambda: PreparedPolygonIndex(polygons, args.rho))
                        call_ms, cold_reject = elapsed_ms(lambda: cold_index.intersects_path(path))
                        assert d[1] == q[1] == cold_reject == False
                        row = dict(layout=layout, rule_count=r, segment_count=n,
                                   repeat=repeat, first=first, build_ms=build_ms,
                                   direct_ms=d[0], indexed_ms=q[0],
                                   cold_ms=build_ms+call_ms,
                                   direct_reject=d[1], indexed_reject=q[1])
                        cell.append(row); rows.append(row); writer.writerow(row)
                        stream.flush()
                    stats = {}
                    prepared.intersects_path(path, stats)
                    summary = dict(layout=layout, rule_count=r, segment_count=n,
                                   repeats=args.repeats, direct_pairs=r*n, **stats)
                    for key in ['direct', 'indexed', 'build', 'cold']:
                        vals = [row[key+'_ms'] for row in cell]
                        summary[key+'_median_ms'] = statistics.median(vals)
                        summary[key+'_p95_ms'] = float(np.percentile(vals, 95))
                    summary['reuse_speedup'] = (summary['direct_median_ms']
                                                / summary['indexed_median_ms'])
                    summaries.append(summary)
                    print(json.dumps(summary, sort_keys=True), flush=True)
    with (args.output_dir/'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader(); writer.writerows(summaries)
    (args.output_dir/'summary.json').write_text(json.dumps(summaries, indent=2)+'\n')


if __name__ == '__main__':
    main()
