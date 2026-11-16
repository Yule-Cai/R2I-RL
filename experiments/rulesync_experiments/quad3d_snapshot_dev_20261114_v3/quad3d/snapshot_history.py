"""Recover complete committed snapshots, never a union of retired rules."""
import json


def replay_snapshots(path, hash_fn):
    pending = {}
    history = {}
    last_epoch = 0
    for expected_seq, line in enumerate(path.read_text().splitlines(), 1):
        record = json.loads(line)
        if record['seq'] != expected_seq:
            raise ValueError('non-contiguous WAL sequence')
        op = record['op']
        txn = record.get('txn_id')
        if op == 'GENESIS':
            if expected_seq != 1 or record['epoch'] != 0:
                raise ValueError('invalid genesis')
            volumes = record['volumes']
            if hash_fn(volumes) != record['snapshot_hash']:
                raise ValueError('genesis hash mismatch')
            history[0] = dict(record, volumes=volumes)
        elif op == 'PREPARE':
            if txn in pending or record['base_epoch'] != last_epoch:
                raise ValueError('prepare identity mismatch')
            if record['base_snapshot_hash'] != history[last_epoch]['snapshot_hash']:
                raise ValueError('prepare base hash mismatch')
            pending[txn] = dict(record, vote=None)
        elif op == 'VOTE':
            prepared = pending[txn]
            if record['txn_epoch'] != prepared['txn_epoch']:
                raise ValueError('vote epoch mismatch')
            prepared['vote'] = record['vote']
        elif op == 'ABORT':
            pending.pop(txn)
        elif op == 'COMMIT':
            prepared = pending.pop(txn)
            epoch = record['epoch']
            if prepared['vote'] != 'PASS' or epoch != last_epoch + 1:
                raise ValueError('commit lacks passing vote or monotone epoch')
            if epoch != prepared['txn_epoch'] or record['txn_epoch'] != epoch:
                raise ValueError('commit epoch mismatch')
            # staged_volumes is a COMPLETE snapshot, including for a replacement.
            volumes = prepared['staged_volumes']
            if record['volume_ids'] != [str(v.get('id', '?')) for v in volumes]:
                raise ValueError('commit volume identities mismatch')
            if hash_fn(volumes) != record['snapshot_hash']:
                raise ValueError('committed content hash mismatch')
            history[epoch] = dict(record, volumes=volumes)
            last_epoch = epoch
        else:
            raise ValueError('unknown WAL operation: ' + op)
    if not history:
        raise ValueError('missing genesis')
    return history
