"""Build explicit stale authorization probes without changing production code."""
import copy
from rulesync3d.authorization import token_id, verify_authorization


def make_probe(fresh, historical, kind):
    for message in (fresh, historical):
        ok, reason, _ = verify_authorization(message.path, message.path_digest,
                                              message.token_id, message.epoch, message.snapshot_hash)
        if not ok:
            raise ValueError('invalid control envelope: ' + reason)
    if not int(historical.epoch) < int(fresh.epoch):
        raise ValueError('historical epoch must be older')
    if kind == 'stale_identity':
        message = copy.deepcopy(fresh)
        message.epoch = historical.epoch
        message.snapshot_hash = historical.snapshot_hash
        message.token_id = token_id(message.epoch, message.snapshot_hash, message.path_digest)
    elif kind == 'late_authorization':
        message = copy.deepcopy(historical)
    else:
        raise ValueError('unknown probe kind')
    ok, reason, _ = verify_authorization(message.path, message.path_digest,
                                          message.token_id, message.epoch, message.snapshot_hash)
    if not ok:
        raise ValueError('invalid probe: ' + reason)
    return message
