"""Correlate real Nav2 output publications with middleware takes and observer logs."""
import argparse
import bisect
import collections
import csv
import hashlib
import json
from pathlib import Path


def rows(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def one(values):
    assert len(values) == 1, ('Absent or ambiguous native evidence', len(values))
    return values[0]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--require-complete-values', action='store_true')
    a = p.parse_args()
    trace = json.loads((a.root / 'trace_result.json').read_text())
    assert trace['trace']['discarded_messages'] == []
    assert trace['trace']['clocks']['monotonic']['frequency'] == 10**9
    native_file = a.root / 'native_events.jsonl'
    nodes, pubs, subs, kinds, count = {}, [], [], collections.Counter(), 0
    for e in rows(native_file):
        count += 1
        kinds[e['name']] += 1
        pid = e['context']['vpid']
        if e['name'] == 'ros2:rcl_node_init':
            nodes[(pid, e['payload']['node_handle'])] = e['payload']['node_name']
        if e['name'] == 'ros2:rcl_publisher_init' and e['payload']['topic_name'] == '/cmd_vel':
            pubs.append(e)
        if e['name'] == 'ros2:rcl_subscription_init' and e['payload']['topic_name'] == '/cmd_vel':
            subs.append(e)
    assert count == trace['trace']['events']
    publisher = one(pubs)
    pid, ph = publisher['context']['vpid'], publisher['payload']
    assert nodes[(pid, ph['node_handle'])] == 'collision_monitor'
    receiver = {}
    for e in subs:
        receiver[nodes[(e['context']['vpid'], e['payload']['node_handle'])]] = e
    assert 'nav2_combination_pilot' in receiver and 'ros_gz_bridge' in receiver
    rcl, rmw, taken = [], [], {name: [] for name in receiver}
    for e in rows(native_file):
        ep = e['payload']
        if e['name'] == 'ros2:rcl_publish' and e['context']['vpid'] == pid and ep['publisher_handle'] == ph['publisher_handle']:
            rcl.append(e)
        if e['name'] == 'ros2:rmw_publish' and e['context']['vpid'] == pid and ep['rmw_publisher_handle'] == ph['rmw_publisher_handle']:
            rmw.append(e)
        if e['name'] == 'ros2:rmw_take' and ep['taken'] == 1:
            for name, sub in receiver.items():
                if e['context']['vpid'] == sub['context']['vpid'] and ep['rmw_subscription_handle'] == sub['payload']['rmw_subscription_handle']:
                    taken[name].append(e)
    assert rmw and len(rcl) == len(rmw)
    publications = {e['payload']['timestamp']: e for e in rmw}
    assert len(publications) == len(rmw), 'Nonunique middleware source timestamps'
    for name in taken:
        taken[name].sort(key=lambda e: e['clock_value'])
        assert len({e['payload']['source_timestamp'] for e in taken[name]}) == len(taken[name])
        assert all(e['payload']['source_timestamp'] in publications for e in taken[name])
    observer_name = 'rulesync_command_witness' if (a.root / 'command_witness.jsonl').exists() else 'nav2_combination_pilot'
    assert observer_name in taken
    observer_takes = taken[observer_name]
    take_times = [e['clock_value'] for e in observer_takes]
    callback_by_source = {}
    callback_file = a.root / 'command_witness.jsonl' if observer_name == 'rulesync_command_witness' else a.root / 'mission/events.jsonl'
    for e in rows(callback_file):
        if e['kind'] != 'command' or e['topic'] != '/cmd_vel':
            continue
        ix = bisect.bisect_right(take_times, e['receipt_ns']) - 1
        assert ix >= 0
        stamp = observer_takes[ix]['payload']['source_timestamp']
        assert stamp not in callback_by_source, 'Ambiguous observer callback pairing'
        callback_by_source[stamp] = e
    assert len(callback_by_source) == len(observer_takes)
    manager = list(rows(a.root / 'mission/manager_events.jsonl'))
    commit2 = one([e for e in manager if e['event'] == 'durable_commit' and e['epoch'] == 2])['t_ns']
    renewals = [e for e in rows(a.root / 'mission/events.jsonl') if e['kind'] == 'task_release' and e['epoch'] == 2]
    assert len(renewals) <= 1
    release2 = renewals[0]['receipt_ns'] if renewals else float('inf')
    assert min(e['clock_value'] for e in rmw) < commit2 < max(e['clock_value'] for e in rmw)
    assert release2 >= commit2
    by_take = {name: {e['payload']['source_timestamp']: e for e in events} for name, events in taken.items()}
    output = []
    previous = -1
    for e in rmw:
        c = one([r for r in rcl if previous < r['clock_value'] <= e['clock_value'] and r['payload']['message'] == e['payload']['message']])
        previous = e['clock_value']
        stamp = e['payload']['timestamp']
        obs = by_take[observer_name].get(stamp)
        bridge = by_take['ros_gz_bridge'].get(stamp)
        callback = callback_by_source.get(stamp)
        if obs:
            assert e['clock_value'] <= obs['clock_value'] <= callback['receipt_ns']
        if bridge:
            assert e['clock_value'] <= bridge['clock_value']
        nonzero = abs(callback['v']) + abs(callback['w']) > 1e-6 if callback else None
        output.append(dict(source_timestamp=stamp, rcl_publish_ns=c['clock_value'], rmw_publish_ns=e['clock_value'],
                           observer_take_ns=obs['clock_value'] if obs else None,
                           observer_callback_ns=callback['receipt_ns'] if callback else None,
                           simulator_bridge_take_ns=bridge['clock_value'] if bridge else None,
                           v=callback['v'] if callback else None, w=callback['w'] if callback else None,
                           observed_nonzero=nonzero, commit2_to_publish_ms=(e['clock_value']-commit2)/1e6,
                           before_renewed_release=e['clock_value'] < release2))
    if a.require_complete_values:
        assert all(r['observed_nonzero'] is not None for r in output), 'Incomplete output value coverage'
    a.output.mkdir(parents=True, exist_ok=False)
    with (a.output / 'command_native_timeline.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    summary = dict(passed=True, scope='same-host native command audit; task failures and absent renewals retained; no physical-actuator application',
                   native_events=count, native_event_kinds=dict(kinds), final_publisher='collision_monitor',
                   final_publications=len(rmw), observer_takes=len(observer_takes), observer_callbacks=len(callback_by_source),
                   value_observer=observer_name, require_complete_values=a.require_complete_values,
                   primary_benchmark_native_takes=len(taken['nav2_combination_pilot']),
                   simulator_bridge_takes=len(taken['ros_gz_bridge']),
                   publication_values_unobserved=sum(r['observed_nonzero'] is None for r in output),
                   observed_nonzero_publications_after_commit_before_release=sum(r['observed_nonzero'] is True and r['commit2_to_publish_ms'] >= 0 and r['before_renewed_release'] for r in output),
                   late_observer_receipts_of_precommit_nonzero_publications=sum(r['observed_nonzero'] is True and r['commit2_to_publish_ms'] < 0 and (r['observer_callback_ns'] or 0) >= commit2 for r in output),
                   trace_discarded_messages=0, independent_scene_audit_passed=None,
                   clock='native monotonic raw nanoseconds checked against manager/observer monotonic clock',
                   correlation='PID-scoped handles+exact unique middleware source timestamps;not Twist-value matching',
                   audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (a.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
