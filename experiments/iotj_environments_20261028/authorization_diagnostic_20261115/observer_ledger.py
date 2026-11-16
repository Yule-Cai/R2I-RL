"""Persistent dual-observer records and a positive-control receive barrier."""
import json
import os
from pathlib import Path
import threading
import time


def info_fields(info):
    result = {}
    for name in ['source_timestamp', 'received_timestamp',
                 'publication_sequence_number', 'reception_sequence_number']:
        value = getattr(info, name, None)
        result[name] = int(value) if value is not None else None
    gid = getattr(info, 'publisher_gid', None)
    result['publisher_gid'] = bytes(gid).hex() if gid is not None else None
    return result


class ObserverLedger:
    def __init__(self, root):
        self.path = Path(root)/'observer_ledger.jsonl'
        self.records = []
        self.lock = threading.Lock()
        self.counter = 0

    def record(self, event, **fields):
        with self.lock:
            self.counter += 1
            row = dict(event=event, sequence=self.counter,
                       mono_ns=time.monotonic_ns(), wall_ns=time.time_ns(),
                       pid=os.getpid(), **fields)
            with self.path.open('a') as stream:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
                stream.flush()
            self.records.append(row)
        return row

    def has_positive(self, observer, digest):
        return any(r['event']=='path_receipt' and r.get('observer')==observer
                   and r.get('digest')==digest and r.get('points', 0)>0
                   for r in self.records)


def probe_classification(rows, sent_ns, end_ns):
    """Receipt flags remain conservative until matched to publisher evidence."""
    nonempty = [r for r in rows if r.get('event')=='path_receipt'
                and sent_ns <= r['mono_ns'] <= end_ns and r.get('points', 0)>0]
    return dict(nonempty_receipts=nonempty,
                receipt_only_endpoint_clear=not nonempty,
                requires_publication_attribution=bool(nonempty))
