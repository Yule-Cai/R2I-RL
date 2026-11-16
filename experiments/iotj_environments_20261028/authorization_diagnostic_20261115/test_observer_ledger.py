import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from observer_ledger import ObserverLedger,info_fields,probe_classification

class DiagnosticTests(unittest.TestCase):
 def test_positive_control_requires_both_receivers(self):
  with tempfile.TemporaryDirectory() as d:
   l=ObserverLedger(d);l.record('path_receipt',observer='primary',points=10,digest='fresh')
   self.assertTrue(l.has_positive('primary','fresh'));self.assertFalse(l.has_positive('redundant','fresh'))
   l.record('path_receipt',observer='redundant',points=10,digest='old')
   self.assertFalse(l.has_positive('redundant','fresh'))
   l.record('path_receipt',observer='redundant',points=10,digest='fresh')
   self.assertTrue(l.has_positive('redundant','fresh'))
 def test_nonempty_receipt_is_saved_as_unattributed(self):
  r=probe_classification([dict(event='path_receipt',mono_ns=20,points=5,digest='same')],10,30)
  self.assertFalse(r['receipt_only_endpoint_clear']);self.assertTrue(r['requires_publication_attribution'])
  self.assertEqual(len(r['nonempty_receipts']),1)
 def test_records_are_persistent_before_assertion(self):
  with tempfile.TemporaryDirectory() as d:
   l=ObserverLedger(d)
   try:
    row=l.record('path_receipt',observer='redundant',points=5,digest='same');raise AssertionError('simulated')
   except AssertionError:pass
   self.assertEqual(json.loads((Path(d)/'observer_ledger.jsonl').read_text()),row)
 def test_empty_stop_is_not_a_nonempty_release(self):
  r=probe_classification([dict(event='path_receipt',mono_ns=20,points=0)],10,30)
  self.assertTrue(r['receipt_only_endpoint_clear'])
 def test_metadata_does_not_fabricate_missing_sequence(self):
  x=info_fields(SimpleNamespace(source_timestamp=123,publisher_gid=[1,2]))
  self.assertEqual(x['publisher_gid'],'0102');self.assertIsNone(x['publication_sequence_number'])

if __name__=='__main__':unittest.main()
