"""A pre-release startup query cannot replace a missing active-period receipt."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('fixed_delivery',ROOT/'analysis/audit_delivery_window.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
RUN=ROOT/'runs/r01_s52703_clean_full/trace'


class ScopeTests(unittest.TestCase):
    def test_startup_reply_is_retained_separately(self):
        result=module.audit(RUN,RUN/'service_cdr_decoded.json')
        self.assertTrue(result['passed'])
        self.assertEqual(result['startup_forwarded_without_guard_callback'],[1])

    def test_missing_active_receipt_still_fails(self):
        result=module.audit(RUN,RUN/'service_cdr_decoded.json')
        active=result['verified_active_interval_start_ns']
        guard=module.rows(RUN/'mission/guard_events.jsonl')
        ordinal=next(r['guard_sequence'] for r in guard if r.get('event')=='channel_snapshot_request_sent' and r['t_ns']>=active)
        original=module.rows
        def altered(path):
            records=original(path)
            if path.name=='guard_events.jsonl':
                records=[r for r in records if not (r.get('event')=='channel_snapshot_response_callback' and r.get('guard_sequence')==ordinal)]
            return records
        with patch.object(module,'rows',side_effect=altered):
            with self.assertRaisesRegex(AssertionError,'Forwarded critical reply missing'):
                module.audit(RUN,RUN/'service_cdr_decoded.json')


if __name__=='__main__':unittest.main()
