"""Independently recompute actual response faults in the measured window.

Verify captured application CDR fields through the independent ROS decoder and
join replies using native request IDs. Requests outside the active measurement are
reported separately; they cannot supply missing evidence inside the window.
"""
import argparse
import base64
import collections
import hashlib
import json
from pathlib import Path


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def index(records,event,key):
    selected=[r for r in records if r.get('event')==event]
    result={r[key]:r for r in selected}
    assert len(selected)==len(result),('Duplicate event/key',event,key)
    return result


def audit(root,decoded_path):
    mission=root/'mission'
    proxy=rows(mission/'snapshot_channel/proxy.jsonl')
    guard=rows(mission/'guard_events.jsonl')
    manager=rows(mission/'manager_events.jsonl')
    config=json.loads((mission/'snapshot_channel/schedule.json').read_text())
    commits=[r for r in manager if r['event']=='durable_commit' and r['epoch']==2]
    assert len(commits)==1
    start=commits[0]['t_ns'];end=start+5*10**9
    decoded=json.loads(decoded_path.read_text())
    assert decoded['all_snapshot_hashes_verified']

    def fields(row):
        raw=base64.b64decode(row['cdr_base64'],validate=True)
        digest=hashlib.sha256(raw).hexdigest()
        assert digest==row['cdr_sha256']
        record=decoded['decoded'][digest]
        canonical=json.dumps(record['fields'],sort_keys=True,separators=(',',':'),allow_nan=False)
        assert hashlib.sha256(canonical.encode()).hexdigest()==record['canonical_sha256']
        assert (record['fields']['epoch'],record['fields']['active_hash'])==(row['epoch'],row['active_hash'])
        return record['fields']

    for row in proxy+guard:
        if 'cdr_base64' in row:fields(row)
    requests=index(proxy,'request','seq')
    sources=index(proxy,'source_response','seq')
    decisions=index(proxy,'decision','seq')
    dispatched=index(proxy,'dispatch','seq')
    begin=index(proxy,'service_send_begin','seq')
    ended=index(proxy,'service_send_returned','seq')
    sent=index(guard,'channel_snapshot_request_sent','rmw_request_sequence')
    received=index(guard,'channel_snapshot_response_callback','rmw_request_sequence')
    by_ordinal={r['guard_sequence']:r for r in sent.values()}
    assert len(by_ordinal)==len(sent)
    assert not any(r['event'] in ('selection_error','callback_error') for r in proxy)
    assert not any(r['event']=='channel_snapshot_response_error' for r in guard)
    events=rows(mission/'events.jsonl')
    initial=[r for r in events if r['kind']=='task_release' and r['epoch']==1]
    assert len(initial)==1 and initial[0]['receipt_ns']<start
    active_start=initial[0]['receipt_ns']
    critical_requests={seq for seq,r in by_ordinal.items() if active_start<=r['t_ns']<end}
    assert critical_requests<=set(requests)
    assert critical_requests<=set(sources) and critical_requests<=set(decisions)
    counts=collections.Counter();matched=0;window_delivered=0
    startup_forwarded_without_guard_callback=[]
    post_window_forwarded_without_guard_callback=[]
    outside_active_forward_not_dispatched=[]
    for seq,decision in decisions.items():
        assert seq in requests and seq in sources
        source=sources[seq]
        stamp=source['monotonic_ns'];elapsed=stamp-start
        active=0<=elapsed<2_500_000_000
        expected='suppress' if config['condition']=='blackout' and active else 'forward'
        assert decision['action']==expected and decision['delay_ns']==0
        assert decision['source_observed_ns']==stamp and decision['window_active']==active
        assert decision['origin_seq']==seq
        assert requests[seq]['monotonic_ns']<=stamp<=decision['monotonic_ns']
        assert decision['selected_cdr_sha256']==source['cdr_sha256']
        if decision['anchor_commit_ns'] is None:
            assert stamp<start
        else:
            assert decision['anchor_commit_ns']==start and decision['source_elapsed_ns']==elapsed
        if source['epoch']==2:
            assert stamp>=start and source['active_hash']==commits[0]['snapshot_hash']
        in_window=start<=stamp<end
        if in_window:counts[expected]+=1
        if expected=='suppress':
            assert seq not in dispatched and seq not in begin and seq not in ended
            assert decision['planned_delivery_ns'] is None
        elif seq in begin:
            assert seq in dispatched and seq in ended
            native=begin[seq]['rmw_request_sequence']
            assert native==ended[seq]['rmw_request_sequence']
            assert native in sent
            assert sent[native]['guard_sequence']==seq
            if native in received:
                reply=received[native]
                assert fields(reply)==fields(source)
                assert reply['guard_sequence']==seq
                assert stamp<=dispatched[seq]['monotonic_ns']<=begin[seq]['monotonic_ns']<=reply['t_ns']
                matched+=1
                if in_window:window_delivered+=1
            else:
                assert seq not in critical_requests,'Forwarded critical reply missing from actual guard'
                target=startup_forwarded_without_guard_callback if by_ordinal[seq]['t_ns']<active_start else post_window_forwarded_without_guard_callback
                target.append(seq)
        else:
            assert seq not in critical_requests,'Critical forward decision not dispatched'
            outside_active_forward_not_dispatched.append(seq)
    for seq in critical_requests:
        if decisions[seq]['action']=='forward':
            assert seq in begin and begin[seq]['rmw_request_sequence'] in received
    tail_pending=[seq for seq in by_ordinal if seq not in decisions]
    assert all(by_ordinal[seq]['t_ns']<active_start or by_ordinal[seq]['t_ns']>=end for seq in tail_pending)
    startup_pending=[seq for seq in tail_pending if by_ordinal[seq]['t_ns']<active_start]
    post_window_pending=[seq for seq in tail_pending if by_ordinal[seq]['t_ns']>=end]
    if config['condition']=='blackout':assert counts['suppress']>0
    else:assert counts['suppress']==0
    assert window_delivered>0
    return dict(passed=True,condition=config['condition'],critical_requests_verified=len(critical_requests),
        all_captured_serializations_field_verified=True,field_exact_guard_receipts=matched,
        measured_window_actions=dict(counts),measured_window_delivered=window_delivered,
        startup_requests_without_source_response=startup_pending,
        startup_forwarded_without_guard_callback=startup_forwarded_without_guard_callback,
        post_window_forwarded_without_guard_callback=post_window_forwarded_without_guard_callback,
        outside_active_forward_not_dispatched=outside_active_forward_not_dispatched,
        post_window_requests_without_source_response=post_window_pending,
        post_window_pending_request_times_ns={str(seq):by_ordinal[seq]['t_ns'] for seq in post_window_pending},
        verified_active_interval_start_ns=active_start,
        auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        interval='COMMIT-anchored source-response omission in [0,2.5s); measured exposure [0,5s)',
        scope='Actual application service responses; not IP packet loss')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--decoded',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:result=audit(args.root,args.decoded)
    except Exception as exc:result=dict(passed=False,error=repr(exc))
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))
