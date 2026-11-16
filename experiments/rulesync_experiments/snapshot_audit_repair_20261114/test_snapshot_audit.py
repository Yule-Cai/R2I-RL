import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from audit_matrix import inside, segment_penetrates, profile
from snapshot_history import replay_snapshots


def digest(volumes):
    return hashlib.sha256(json.dumps(volumes, sort_keys=True).encode()).hexdigest()


A = dict(id='old', bmin=[0, 0, 0], bmax=[1, 1, 1])
B = dict(id='new', bmin=[3, 0, 0], bmax=[4, 1, 1])


def wal_snapshots(snapshots):
    records = [dict(seq=1, op='GENESIS', epoch=0, volumes=[], snapshot_hash=digest([]), receipt_ns=1)]
    old = []
    for epoch, volumes in enumerate(snapshots, 1):
        txn = f't{epoch}'
        records.extend([
            dict(op='PREPARE', txn_id=txn, base_epoch=epoch - 1, base_snapshot_hash=digest(old),
                 txn_epoch=epoch, staged_volumes=volumes),
            dict(op='VOTE', txn_id=txn, txn_epoch=epoch, vote='PASS'),
            dict(op='COMMIT', txn_id=txn, epoch=epoch, txn_epoch=epoch,
                 volume_ids=[v['id'] for v in volumes], snapshot_hash=digest(volumes))])
        old = volumes
    for i, row in enumerate(records, 1):
        row['seq'] = i
        row['receipt_ns'] = i
    return records


class SnapshotAuditTests(unittest.TestCase):
    def replay(self, records):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wal.jsonl'
            path.write_text(''.join(json.dumps(x) + '\n' for x in records))
            return replay_snapshots(path, digest)

    def test_replacement_retires_old_rule(self):
        history = self.replay(wal_snapshots([[A], [B]]))
        point = [.5, .5, .5]
        self.assertTrue(inside(point, A))
        self.assertFalse(any(inside(point, v) for v in history[2]['volumes']))

    def test_additive_snapshot_retains_both_rules(self):
        history = self.replay(wal_snapshots([[A], [A, B]]))
        self.assertTrue(any(inside([.5, .5, .5], v) for v in history[2]['volumes']))

    def test_active_rule_violation_still_detected(self):
        rows = [dict(pose=[3.5, .5, .5])]
        self.assertEqual(profile(rows, [B], .25)['raw_inside'], 1)

    def test_crossing_with_both_endpoints_outside(self):
        self.assertTrue(segment_penetrates([-1, .5, .5], [2, .5, .5], A))

    def test_grazing_is_not_interior_penetration(self):
        self.assertFalse(segment_penetrates([-1, 0, .5], [2, 0, .5], A))

    def test_degenerate_segment_inside(self):
        self.assertTrue(segment_penetrates([.5, .5, .5], [.5, .5, .5], A))

    def test_inflation_detects_body_margin(self):
        self.assertFalse(inside([1.1, .5, .5], A))
        self.assertTrue(inside([1.1, .5, .5], A, .25))

    def test_hash_tampering_fails(self):
        records = wal_snapshots([[A]])
        records[-1]['snapshot_hash'] = 'bad'
        with self.assertRaises(ValueError):
            self.replay(records)

    def test_failed_vote_cannot_commit(self):
        records = wal_snapshots([[A]])
        records[-2]['vote'] = 'FAIL'
        with self.assertRaises(ValueError):
            self.replay(records)

    def test_nonmonotone_commit_fails(self):
        records = wal_snapshots([[A], [B]])
        records[-1]['epoch'] = 1
        with self.assertRaises(ValueError):
            self.replay(records)

    def test_missing_wal_record_fails(self):
        records = wal_snapshots([[A]])
        del records[-2]
        with self.assertRaises(ValueError):
            self.replay(records)

    def test_wrong_base_identity_fails(self):
        records = wal_snapshots([[A], [B]])
        records[-3]['base_snapshot_hash'] = 'wrong'
        with self.assertRaises(ValueError):
            self.replay(records)

    def test_staged_content_tampering_fails(self):
        records = copy.deepcopy(wal_snapshots([[A]]))
        records[1]['staged_volumes'][0]['bmax'][0] = 5
        with self.assertRaises(ValueError):
            self.replay(records)


if __name__ == '__main__':
    unittest.main()
