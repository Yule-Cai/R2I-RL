"""Thread-safe immutable view: durable volume authority != release readiness.

Only the coordinator's retained durable decision installs a new identity.
Participant acknowledgements may change readiness, never authority. This view
allows readers to revoke an old execution while APPLY/ACK is still pending.
Transport authentication and permanent durable-storage loss are out of scope.
"""
from dataclasses import dataclass, replace
import json
import re
from threading import RLock


@dataclass(frozen=True)
class AuthorityView:
    available: bool = False
    epoch: int = 0
    snapshot_hash: str = ""
    ready: bool = False
    payload_json: str = "[]"

    def volumes(self):
        # Each caller gets its own copy, not a mutable alias to authority.
        return json.loads(self.payload_json)


class VolumeAuthority:
    def __init__(self):
        self._lock = RLock()
        self._view = AuthorityView()

    def read(self):
        with self._lock:
            return self._view

    def install_durable(self, epoch, snapshot_hash, volumes):
        epoch, digest = int(epoch), str(snapshot_hash)
        if epoch < 0 or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("invalid durable identity")
        payload = json.dumps(list(volumes), sort_keys=True, allow_nan=False)
        with self._lock:
            old = self._view
            if epoch < old.epoch:
                raise ValueError("durable epoch regression")
            if old.available and epoch == old.epoch:
                if digest != old.snapshot_hash or payload != old.payload_json:
                    raise ValueError("conflicting identity at one epoch")
                return old  # Duplicate recovery does not erase valid ACKs.
            self._view = AuthorityView(True, epoch, digest, False, payload)
            return self._view

    def mark_converged(self, epoch, snapshot_hash, ready):
        with self._lock:
            old = self._view
            if (not old.available or int(epoch) != old.epoch
                    or str(snapshot_hash) != old.snapshot_hash):
                return False  # An old ACK cannot release a newer decision.
            self._view = replace(old, ready=bool(ready))
            return True

    def invalidate(self):
        with self._lock:
            self._view = replace(self._view, available=False, ready=False)
