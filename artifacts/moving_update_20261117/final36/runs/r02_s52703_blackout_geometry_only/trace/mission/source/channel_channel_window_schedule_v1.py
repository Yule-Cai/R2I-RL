"""Deterministic response faults anchored to an observed durable-COMMIT event.

This module has no ROS dependency and never writes manager state or event logs.
Intervals use source-response observation time on the shared monotonic clock.
"""
import hashlib
import json


CONDITIONS = {'clean', 'delay_jitter', 'response_loss', 'blackout', 'stale_cache'}


def read_anchor(path, epoch):
    if not path.exists():
        return None
    # A concurrently appended final line can be incomplete. Only complete JSONL
    # records are eligible; never manufacture an anchor from file mtime.
    text = path.read_text()
    lines = text.splitlines() if text.endswith('\n') else text.splitlines()[:-1]
    matches = [r for r in map(json.loads, lines)
               if r.get('event') == 'durable_commit' and int(r['epoch']) == epoch]
    assert len(matches) <= 1, 'Ambiguous durable COMMIT anchor'
    return matches[0] if matches else None


def choose(config, seq, observed_ns, anchor):
    assert config['schema_version'] == 2 and config['scope'] == 'DEV_commit_anchored_delivery'
    condition = config['condition']
    assert condition in CONDITIONS
    elapsed = observed_ns-int(anchor['t_ns']) if anchor else None
    active = elapsed is not None and round(config['window_start_s']*1e9) <= elapsed < round(config['window_end_s']*1e9)
    # No arm name in this key. Equivalent logical requests get the same seeded
    # decision; actual request rates/windows are still recorded for every arm.
    key = f"{config['seed']}|{config['channel']}|{seq}"
    digest = hashlib.sha256(key.encode()).hexdigest()
    u = int(digest[:16], 16)/2**64
    spec = dict(action='forward', delay_s=0.)
    if active:
        if condition == 'delay_jitter':
            spec['delay_s'] = config['delay_base_s']+config['delay_jitter_s']*u
        elif condition == 'response_loss' and u < config['response_loss_probability']:
            spec['action'] = 'suppress'
        elif condition == 'blackout':
            spec['action'] = 'suppress'
        elif condition == 'stale_cache':
            spec.update(action='cached', cache_epoch=int(config['anchor_epoch'])-1)
    context = dict(condition=condition, anchor_commit_ns=int(anchor['t_ns']) if anchor else None,
                   anchor_epoch=config['anchor_epoch'], source_observed_ns=observed_ns,
                   source_elapsed_ns=elapsed, window_active=active, random_key_sha256=digest)
    spec['_context'] = context
    return spec
