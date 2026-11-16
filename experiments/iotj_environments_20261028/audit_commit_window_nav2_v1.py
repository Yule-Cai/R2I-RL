"""Independently audit actual COMMIT-window delivery and real guard receipts.

This does not call the schedule implementation. Recompute decisions separately,
join replies by real RMW request IDs and compare every decoded response field.
Task/geometry and native command audits are distinct evidence requirements.
"""
import argparse
import base64
import collections
import hashlib
import json
from pathlib import Path


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines()]


def indexed(items,event,key):
    selected=[x for x in items if x.get('event')==event]
    result={x[key]:x for x in selected}
    assert len(result)==len(selected),('Duplicate',event)
    return result


def audit(root,decoded_path):
    mission=root/'mission'
    proxy=rows(mission/'snapshot_channel/proxy.jsonl')
    guard=rows(mission/'guard_events.jsonl')
    manager=rows(mission/'manager_events.jsonl')
    config=json.loads((mission/'snapshot_channel/schedule.json').read_text())
    assert config['schema_version']==2 and config['scope']=='DEV_commit_anchored_delivery'
    commits=[r for r in manager if r['event']=='durable_commit' and r['epoch']==config['anchor_epoch']]
    assert len(commits)==1
    commit=commits[0];t0=commit['t_ns']
    decoded=json.loads(decoded_path.read_text());assert decoded['all_snapshot_hashes_verified']
    def fields(r):
        raw=base64.b64decode(r['cdr_base64'],validate=True);digest=hashlib.sha256(raw).hexdigest()
        assert digest==r['cdr_sha256']
        d=decoded['decoded'][digest]
        canonical=json.dumps(d['fields'],sort_keys=True,separators=(',',':'),allow_nan=False)
        assert hashlib.sha256(canonical.encode()).hexdigest()==d['canonical_sha256']
        assert (d['fields']['epoch'],d['fields']['active_hash'])==(r['epoch'],r['active_hash'])
        return d['fields']
    requests=indexed(proxy,'request','seq')
    sources=indexed(proxy,'source_response','seq')
    decisions=indexed(proxy,'decision','seq')
    dispatched=indexed(proxy,'dispatch','seq')
    ready=indexed(proxy,'callback_return_ready','seq')
    send_begin=indexed(proxy,'service_send_begin','seq')
    send_end=indexed(proxy,'service_send_returned','seq')
    client_requests=indexed(guard,'channel_snapshot_request_sent','rmw_request_sequence')
    client_replies=indexed(guard,'channel_snapshot_response_callback','rmw_request_sequence')
    assert requests and set(requests)==set(sources)==set(decisions)
    assert set(requests)==set(range(1,len(requests)+1))
    assert set(dispatched)==set(ready)==set(send_begin)==set(send_end)
    assert len(client_requests)==len(requests),'Unmatched client/proxy request population'
    assert not any(r.get('event') in ('selection_error','callback_error') for r in proxy)
    assert not any(r.get('event')=='channel_snapshot_response_error' for r in guard)
    first_by_epoch={};counts=collections.Counter();delivery=[];selected_by_seq={}
    for seq in sorted(requests):
        source=sources[seq];d=decisions[seq];r=requests[seq]
        assert r['monotonic_ns']<=source['monotonic_ns']<=d['monotonic_ns']
        source_fields=fields(source)
        first_by_epoch.setdefault(source['epoch'],seq)
        elapsed=source['monotonic_ns']-t0
        active=round(config['window_start_s']*1e9)<=elapsed<round(config['window_end_s']*1e9)
        # The anchor is absent before the manager has appended its event. Once
        # observed it must be exactly the real hook event, never a guessed time.
        if d['anchor_commit_ns'] is not None:
            assert d['anchor_commit_ns']==t0 and d['source_elapsed_ns']==elapsed
        else:
            assert source['monotonic_ns']<t0 and d['source_elapsed_ns'] is None
        assert d['source_observed_ns']==source['monotonic_ns'] and d['window_active']==active
        assert d['condition']==config['condition'] and d['anchor_epoch']==config['anchor_epoch']
        key=f"{config['seed']}|{config['channel']}|{seq}"
        digest=hashlib.sha256(key.encode()).hexdigest();u=int(digest[:16],16)/2**64
        assert d['random_key_sha256']==digest
        expected='forward';delay=0.
        if active:
            if config['condition']=='blackout':expected='suppress'
            elif config['condition']=='response_loss' and u<config['response_loss_probability']:expected='suppress'
            elif config['condition']=='stale_cache':expected='cached'
            elif config['condition']=='delay_jitter':delay=config['delay_base_s']+config['delay_jitter_s']*u
        assert d['action']==expected and d['delay_ns']==round(delay*1e9)
        origin=first_by_epoch[config['anchor_epoch']-1] if expected=='cached' else seq
        original=sources[origin];selected_by_seq[seq]=fields(original)
        assert d['origin_seq']==origin and d['selected_cdr_sha256']==original['cdr_sha256']
        assert (d['selected_epoch'],d['selected_hash'])==(original['epoch'],original['active_hash'])
        if source['epoch']==config['anchor_epoch']:
            assert source['active_hash']==commit['snapshot_hash'] and source['monotonic_ns']>=t0
        counts[expected]+=1
        if expected=='suppress':
            assert seq not in dispatched and d['planned_delivery_ns'] is None
        else:
            assert seq in dispatched
            planned=d['planned_delivery_ns']
            assert source['monotonic_ns']<=planned-d['delay_ns']<=d['monotonic_ns']
            assert planned<=dispatched[seq]['monotonic_ns']<=ready[seq]['monotonic_ns']<=send_begin[seq]['monotonic_ns']<=send_end[seq]['monotonic_ns']
            native=send_begin[seq]['rmw_request_sequence']
            assert native==send_end[seq]['rmw_request_sequence']
            assert native in client_requests and native in client_replies
            callback=client_replies[native]
            assert callback['guard_sequence']==client_requests[native]['guard_sequence']
            assert fields(callback)==selected_by_seq[seq]
            assert send_begin[seq]['monotonic_ns']<=callback['t_ns']
            # Verify arrival-order ordinals against real reply headers for ALL
            # responses before using that relation for unanswered requests.
            assert client_requests[native]['guard_sequence']==seq
            delivery.append(dict(seq=seq,rmw_request_sequence=native,action=expected,
                epoch=original['epoch'],current_generation=callback['current_generation'],
                source_to_dispatch_ms=(dispatched[seq]['monotonic_ns']-source['monotonic_ns'])/1e6,
                source_to_callback_ms=(callback['t_ns']-source['monotonic_ns'])/1e6))
    delivered_ids={r['rmw_request_sequence'] for r in send_begin.values()}
    assert delivered_ids==set(client_replies)
    unanswered=set(client_requests)-delivered_ids
    assert {client_requests[n]['guard_sequence'] for n in unanswered}=={s for s in decisions if decisions[s]['action']=='suppress'}
    if config['condition']!='clean':
        assert any(d['window_active'] for d in decisions.values()),'No in-window observation'
    if config['condition']=='blackout':assert counts['suppress']>0
    adapter=json.loads((mission/'channel_adapter.json').read_text())
    for name,digest in adapter['extra_source_sha256'].items():
        assert hashlib.sha256((mission/'source'/('channel_'+name)).read_bytes()).hexdigest()==digest
    process=json.loads((mission/'channel_process_exit.json').read_text());assert process['relay_exit_before_forced_cleanup']==0
    stop=[r for r in proxy if r['event']=='stop'];assert len(stop)==1 and stop[0]['scheduled_pending']==0
    assert stop[0]['requests']==len(requests) and stop[0]['deliberately_unanswered']==len(unanswered)
    events=rows(mission/'events.jsonl');release=[r for r in events if r['kind']=='task_release' and r['epoch']==2]
    assert len(release)==1
    observations=[r for r in events if r['kind']=='channel_admission_observation_ready' and r['epoch']==2]
    assert len(observations)==1 and observations[0]['snapshot_hash']==commit['snapshot_hash']
    assert t0<=observations[0]['receipt_ns']<=release[0]['receipt_ns']
    startup_ready_verified = False
    if adapter.get("startup_collection"):
        startup = json.loads((mission / "channel_startup.json").read_text())
        witness = json.loads((mission / "snapshot_channel/ready.json").read_text())
        first_request = min(r["t_ns"] for r in client_requests.values())
        assert witness["ready"] is True and witness["monotonic_ns"] == startup["relay_ready_ns"]
        assert max(startup["relay_ready_ns"], startup["wait_started_ns"]) <= startup["ready_observed_ns"] <= first_request
        startup_ready_verified = True
    terminal_drain_verified = False
    if adapter.get("shutdown_collection"):
        drain = json.loads((mission / "channel_drain.json").read_text())
        terminal = [r for r in events if r["kind"] == "terminal"]
        frozen = [r for r in guard if r["event"] == "channel_drain_polling_frozen"]
        assert len(terminal) == len(frozen) == 1 and drain["complete"]
        assert terminal[0]["receipt_ns"] == drain["terminal_ns"]
        assert drain["terminal_ns"] <= drain["request_ns"] <= drain["frozen_ns"] <= drain["finished_ns"]
        assert frozen[0]["t_ns"] == drain["frozen_ns"]
        assert frozen[0]["last_guard_sequence"] == drain["last_guard_sequence"] == len(client_requests) == drain["requests"]
        assert drain["receipts"] == len(client_replies) and not drain["outstanding"]
        assert all(r["t_ns"] <= drain["frozen_ns"] for r in client_requests.values())
        assert all(r["t_ns"] <= drain["finished_ns"] for r in client_replies.values())
        terminal_drain_verified = True
    result=dict(passed=True,condition=config['condition'],requests=len(requests),field_exact_guard_receipts=len(delivery),
                response_actions=dict(counts),unanswered_requests=len(unanswered),
                actual_rmw_request_id_join=True,source_and_guard_all_fields_identical=True,
                commit_anchor_ns=t0,commit_to_renewed_release_ms=(release[0]['receipt_ns']-t0)/1e6,
                guard_hold_reasons=sorted({r['reason'] for r in guard if r.get('event')=='guard_decision' and r['state']=='HOLD'}),
                late_retired_generation_callbacks=sum(not r['current_generation'] for r in delivery),
                terminal_drain_verified=terminal_drain_verified,
                startup_ready_verified=startup_ready_verified,
                delivery=delivery,monitor_arm=adapter.get('monitor_arm','full'),
                scope='same-host Nav2 snapshot-delivery audit;monitor arm recorded independently of delivery validity')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--decoded',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=audit(a.root,a.decoded);a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.output/'audit_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in result.items() if k!='delivery'},indent=2))


if __name__=='__main__':main()
