"""ROS-free tests of body, time-domain and stop-censoring contracts."""
import math
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from benchmark_dynamic_rule_update import (BODY, transformed_body,
    convex_intersects, body_distance, measured_stop, geometry_core_class,
    timed_guard_class, evaluate, audit_run)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rulesync3d.rulesync3d.ruleguard import RuleGuard


class DynamicUpdateTests(unittest.TestCase):
    def test_touch_is_intersection(self):
        box = [(0, 0), (1, 0), (1, 1), (0, 1)]
        self.assertTrue(convex_intersects(box, [(1, 0), (2, 0), (2, 1), (1, 1)]))
        self.assertFalse(convex_intersects(box, [(1.01, 0), (2, 0), (2, 1), (1.01, 1)]))

    def test_body_can_enter_before_center(self):
        row = dict(x=-.35, y=-.5, quaternion=[0, 0, 0, 1])
        self.assertEqual(body_distance(transformed_body(row)), 0)
        self.assertLess(row["x"], -.30)

    def test_distance_and_rotation(self):
        row = dict(x=-1., y=-.5, quaternion=[0, 0, 0, 1])
        self.assertAlmostEqual(body_distance(transformed_body(row)), .59)
        row["quaternion"] = [0, 0, math.sqrt(.5), math.sqrt(.5)]
        self.assertAlmostEqual(transformed_body(row)[0][0], -.83)
        self.assertGreater(.28, max(math.hypot(x, y) for x, y in BODY))

    def test_nonplanar_pose_rejected(self):
        with self.assertRaises(ValueError):
            transformed_body(dict(x=0, y=0, quaternion=[math.sin(.1), 0, 0, math.cos(.1)]))

    def samples(self, xs):
        return [dict(x=x, y=0, sim_ns=1_000_000_000+i*50_000_000,
                     receipt_ns=2_000_000_000+i*100_000_000) for i, x in enumerate(xs)]

    def test_stop_sustain_uses_sim_clock(self):
        rows = self.samples([0]*8)
        result = measured_stop(rows, 2_000_000_000)
        self.assertEqual(result["onset_receipt_ns"], 2_100_000_000)
        self.assertEqual(result["confirmed_receipt_ns"], 2_500_000_000)

    def test_precommit_stop_not_counted(self):
        self.assertIsNone(measured_stop(self.samples([0]*8), 2_650_000_000))

    def test_motion_or_short_pause_not_a_stop(self):
        self.assertIsNone(measured_stop(self.samples([0, .01, .02, .03, .04, .05, .06]), 0))
        self.assertIsNone(measured_stop(self.samples([0, 0, 0, 0, .1, .2]), 0))

    def test_gap_rejected(self):
        rows = self.samples([0, 0])
        rows[-1]["sim_ns"] += 200_000_000
        with self.assertRaises(ValueError):
            measured_stop(rows, 0)

    def test_metrics_use_full_hash_and_independent_body(self):
        with tempfile.TemporaryDirectory() as folder:
            events = [dict(event="durable_commit", t_ns=2_000_000_000,
                           epoch=2, snapshot_hash="complete-hash"),
                      dict(event="snapshot_observed", t_ns=2_100_000_000,
                           authority=[2, "wrong-hash"]),
                      dict(event="snapshot_observed", t_ns=2_200_000_000,
                           authority=[2, "complete-hash"])]
            (Path(folder)/"manager_events.jsonl").write_text("".join(json.dumps(r)+"\n" for r in events))
            worlds = [dict(x=-.35, y=-.5, quaternion=[0, 0, 0, 1], active=True,
                          sim_ns=1_000_000_000+i*20_000_000,
                          receipt_ns=2_000_000_000+i*20_000_000) for i in range(25)]
            guard = [dict(state=2, reason="epoch-disagreement", receipt_ns=2_100_000_000)]
            cmds = [dict(linear_x=0, angular_z=0, receipt_ns=2_140_000_000)]
            result = evaluate(folder, worlds, guard, cmds)
            self.assertEqual(result["commit_to_observed_ms"], 200)
            self.assertTrue(result["sampled_body_entry"])
            self.assertEqual(result["commit_to_hold_receipt_ms"], 100)
            self.assertEqual(result["commit_to_zero_command_receipt_ms"], 140)
            self.assertIsNotNone(result["world_low_speed"])
            self.assertEqual(result["sampled_body_entry_count"], 25)
            events.append(dict(event="guard_decision", state="HOLD", t_ns=1_900_000_000))
            (Path(folder)/"manager_events.jsonl").write_text("".join(json.dumps(r)+"\n" for r in events))
            result = evaluate(folder, worlds, guard, cmds)
            self.assertTrue(result["hold_active_before_commit"])
            self.assertIsNone(result["commit_to_hold_receipt_ms"])

    def test_interrupted_and_unattempted_rows_not_successes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/"manifest.json").write_text(json.dumps(dict(trials=[
                dict(arm="full"), dict(arm="gate_only")], source_sha256={})))
            (root/"trial_01").mkdir()
            result = audit_run(root)
            self.assertEqual(result["attempted"], 1)
            self.assertEqual(result["completed"], 0)
            self.assertEqual(result["not_attempted"], [2])
            self.assertFalse(result["audit_passed"])

    def test_geometry_core_retains_original_pin(self):
        full, geometry = RuleGuard(), geometry_core_class(RuleGuard)()
        for guard in (full, geometry):
            guard.update_snapshot(1, "old")
            guard.bind_execution(1, "old")
            guard.update_snapshot(2, "new")
        self.assertEqual(full.check_bound(.4)[0], "HOLD")
        self.assertEqual(geometry.check_bound(.4)[0], "WARNING")
        self.assertEqual(geometry.execution_identity(), (1, "old"))
        self.assertEqual(geometry.check_bound(-.01)[0], "HOLD")
        geometry.invalidate_snapshot()
        self.assertEqual(geometry.check_bound(.4)[1]["reason"], "monitor-no-snapshot")
        geometry.clear_execution()
        self.assertEqual(geometry.check_bound(.4)[1]["reason"], "execution-not-bound")

    def test_geometry_core_keeps_same_geometric_hysteresis(self):
        full, geometry = RuleGuard(), geometry_core_class(RuleGuard)()
        for guard in (full, geometry):
            guard.update_snapshot(1, "same")
            guard.bind_execution(1, "same")
        for clearance in (.6, .4, -.01, .1, .3, .3, .6):
            self.assertEqual(full.check_bound(clearance), geometry.check_bound(clearance))

    def test_callback_only_suppresses_identity_not_freshness_stop(self):
        # Minimal production-shaped node permits testing without importing ROS.
        class Node:
            def __init__(self):
                self._guard = RuleGuard()
                self._guard.update_snapshot(1, "old")
                self._guard.bind_execution(1, "old")
                self._latched_hold = False
                self._state, self._reason = "CLEAR", "ok"
                self._snapshot_pending_seq = 4
                self._snapshot_at = 0
                self.stops = 0

            def _on_snapshot(self, future, sequence):
                self._guard.update_snapshot(2, "new")
                self._snapshot_at = 1e30
                self._latched_hold = True
                self._set_state("HOLD", "epoch-disagreement", True, math.nan)
                self._publish_stop()

            def _set_state(self, state, reason, degraded, clearance):
                self._state, self._reason = state, reason

            def _publish_stop(self):
                self.stops += 1

        with patch.dict(os.environ, RULESYNC_DYNAMIC_ARM="geometry_only"), \
                patch("benchmark_dynamic_rule_update.emit"):
            node = timed_guard_class(Node, RuleGuard)()
            node._guard.update_snapshot(1, "old")
            node._guard.bind_execution(1, "old")
            node._on_snapshot(None, 4)
            self.assertFalse(node._latched_hold)
            self.assertEqual(node.stops, 0)
            self.assertEqual(node._guard.execution_identity(), (1, "old"))
            node._set_state("HOLD", "monitor-snapshot-stale", True, math.nan)
            node._publish_stop()
            self.assertEqual(node._state, "HOLD")
            self.assertEqual(node.stops, 1)
        with patch.dict(os.environ, RULESYNC_DYNAMIC_ARM="full"), \
                patch("benchmark_dynamic_rule_update.emit"):
            node = timed_guard_class(Node, RuleGuard)()
            node._on_snapshot(None, 4)
            self.assertTrue(node._latched_hold)
            self.assertEqual(node.stops, 1)


if __name__ == "__main__":
    unittest.main()
