"""ROS-free measurement tests; these are tests, not experimental rows."""
import tempfile
import ast
import json
from pathlib import Path
import time
from types import SimpleNamespace as N, MethodType
import unittest
import xml.etree.ElementTree as ET

from benchmark_dynamic_volume_update import (
    body_bounds, box_distance, measured_stop, instrument_world, compose_pose, first_zero_in_capture, startup_ready)
from summarize_dynamic_volume_update import audit_ablation_trial


def probe_callbacks(arm="full"):
    from rulesync3d.rulesync3d.ruleguard import RuleGuard, snapshot_hash, volume_snapshot_items, HOLD
    from rulesync3d.rulesync3d.uav_supervisor import Supervisor
    from benchmark_dynamic_volume_update import volume_dict, RHO, SNAPSHOT_STALE_S, GOAL
    tree = ast.parse(Path(__file__).with_name("benchmark_dynamic_volume_update.py").read_text())
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "Probe")
    scope = dict(time=time, snapshot_hash=snapshot_hash, volume_snapshot_items=volume_snapshot_items,
        volume_dict=volume_dict, RHO=RHO, SNAPSHOT_STALE_S=SNAPSHOT_STALE_S, GOAL=GOAL, HOLD=HOLD,
        emit=lambda *a, **kw: None, arm=arm)
    functions = [n for n in cls.body if getattr(n, "name", "") in ("observed_message", "control")]
    exec(compile(ast.Module(body=functions, type_ignores=[]), "actual_probe_callbacks", "exec"), scope)
    node = N(authority=None, last_identity=None, snapshot_at=0., ready=False,
        guard=RuleGuard(), supervisor=Supervisor(.2, .2), pose_at=time.monotonic(),
        xyz=(1.,10.,2.), tracking=True, held=False, active=False, cmd=[], volumes=[])
    for fn in functions:
        setattr(node, fn.name, MethodType(scope[fn.name], node))
    return node


def notice(epoch=1, ready=True, available=True, shift=0.):
    from rulesync3d.rulesync3d.ruleguard import snapshot_hash, volume_snapshot_items
    v = N(rule_id="a", epoch=epoch, rule_type=0, severity=1, frame_id="map", geometry_type=0,
        center=N(x=12.+shift, y=12., z=2.), dimensions=N(x=1., y=1., z=1.), vertices=[],
        valid_from=N(sec=0, nanosec=0), valid_until=N(sec=2_000_000_000, nanosec=0))
    return N(epoch=epoch, ready=ready, available=available, volumes=[v],
             snapshot_hash=snapshot_hash(volume_snapshot_items([v])))


class MeasurementTests(unittest.TestCase):
    def test_available_authority_is_not_startup_readiness(self):
        client = N(service_is_ready=lambda: True)
        node = N(xyz=(1,10,2), worlds=[None]*21, authority=[0,"h"], ready=False,
                 propose=client, commit=client, bridge=client)
        self.assertFalse(startup_ready(node))
        node.ready = True
        self.assertTrue(startup_ready(node))

    def test_startup_needs_all_services_and_pose_capture(self):
        client = N(service_is_ready=lambda: True)
        node = N(xyz=(1,10,2), worlds=[None]*20, authority=[0,"h"], ready=True,
                 propose=client, commit=client, bridge=client)
        self.assertFalse(startup_ready(node))
        node.worlds.append(None)
        node.bridge = N(service_is_ready=lambda: False)
        self.assertFalse(startup_ready(node))

    def test_durable_notice_revokes_but_does_not_rebind_execution(self):
        node = probe_callbacks()
        node.observed_message(notice(1), "rpc")
        self.assertTrue(node.guard.bind_execution(*node.authority))
        pinned = node.guard.execution_identity()
        node.observed_message(notice(2, ready=False), "notice")
        self.assertFalse(node.ready)
        self.assertEqual(node.guard.execution_identity(), pinned)
        node.control()
        self.assertTrue(node.held)

    def test_geometry_ablation_uses_same_new_geometry_not_identity(self):
        node = probe_callbacks("geometry_only")
        node.observed_message(notice(1), "rpc")
        node.guard.bind_execution(*node.authority)
        node.observed_message(notice(2, ready=False), "notice")
        node.control()
        self.assertFalse(node.held)  # Still far from new geometry.

    def test_lost_ack_of_pinned_identity_holds_both_monitors(self):
        for arm in ("full", "geometry_only"):
            node = probe_callbacks(arm)
            node.observed_message(notice(1), "rpc")
            node.guard.bind_execution(*node.authority)
            node.observed_message(notice(1, ready=False), "notice")
            node.control()
            self.assertTrue(node.held)

    def test_old_rpc_response_neither_rebinds_nor_refreshes(self):
        node = probe_callbacks()
        node.observed_message(notice(2), "notice")
        before = node.snapshot_at
        node.observed_message(notice(1), "rpc")
        self.assertEqual(node.authority[0], 2)
        self.assertEqual(node.snapshot_at, before)

    def test_conflict_invalidates_and_older_response_cannot_restore(self):
        node = probe_callbacks()
        node.observed_message(notice(2), "notice")
        node.observed_message(notice(2, shift=1.), "rpc")
        self.assertIsNone(node.authority)
        node.observed_message(notice(1), "rpc")
        self.assertIsNone(node.authority)

    def test_rpc_alone_can_observe_new_durable_identity(self):
        node = probe_callbacks()
        node.observed_message(notice(1), "rpc")
        node.guard.bind_execution(*node.authority)
        node.observed_message(notice(2, ready=False), "rpc")
        node.control()
        self.assertTrue(node.held)

    def test_loss_of_both_channels_expires_for_both_monitors(self):
        for arm in ("full", "geometry_only"):
            node = probe_callbacks(arm)
            node.observed_message(notice(1), "rpc")
            node.guard.bind_execution(*node.authority)
            node.snapshot_at -= 2.
            node.control()
            self.assertTrue(node.held)

    def test_cleanup_command_is_not_an_intervention(self):
        rows = [dict(event="command", t_ns=40, velocity=[0,0,0])]
        self.assertIsNone(first_zero_in_capture(rows, 10, 30))
        self.assertEqual(first_zero_in_capture(rows, 10, 40), rows[0])

    def test_independent_pose_composition(self):
        self.assertEqual(compose_pose([1,10,2], [0,0,0,1], [.5,.2,.3], [0,0,0,1]),
                         ([1.5,10.2,2.3], [0,0,0,1]))

    def test_rotated_parent_composition(self):
        xyz, q = compose_pose([1,10,2], [0,0,1,0], [1,2,3], [0,0,0,1])
        self.assertEqual(xyz, [0,8,5])
        self.assertEqual(q, [0,0,1,0])

    def test_contact_is_entry(self):
        self.assertEqual(box_distance(([0,0,0], [1,1,1]), ([1,0,0], [2,1,1])), 0)

    def test_xyz_distance(self):
        self.assertEqual(box_distance(([0,0,0], [1,1,1]), ([2,3,3], [3,4,4])), 3)

    def test_body_collision_extents(self):
        lo, hi = body_bounds(dict(xyz=[1,2,3], quaternion=[0,0,0,1]))
        self.assertEqual(lo, [.83,1.83,2.94])
        self.assertEqual(hi, [1.17,2.17,3.06])

    def test_attitude_scope(self):
        with self.assertRaises(ValueError):
            body_bounds(dict(xyz=[0,0,0], quaternion=[0,0,.1,.995]))

    def test_stop_needs_three_dimensions(self):
        rows = [dict(sim_ns=i*20_000_000, receipt_ns=i*20_000_000,
                     xyz=[0,0,i*.01]) for i in range(30)]
        self.assertIsNone(measured_stop(rows, 1))

    def test_stop_sustain(self):
        rows = [dict(sim_ns=i*20_000_000, receipt_ns=i*20_000_000,
                     xyz=[0,0,0]) for i in range(30)]
        stop = measured_stop(rows, 100_000_000)
        self.assertEqual(stop["onset_receipt_ns"], 120_000_000)
        self.assertEqual(stop["confirmed_receipt_ns"], 320_000_000)

    def test_gap_rejected(self):
        rows = [dict(sim_ns=i*200_000_000, receipt_ns=i*200_000_000, xyz=[0,0,0]) for i in range(3)]
        with self.assertRaises(ValueError):
            measured_stop(rows, 0)

    def test_world_link_capture_not_model_origin(self):
        with tempfile.TemporaryDirectory() as folder:
            tree = ET.parse(instrument_world(Path(folder)))
            model = tree.getroot().find("world/model")
            pp = [p for p in model.findall("plugin") if "PosePublisher" in p.get("name")][0]
            self.assertEqual(pp.findtext("publish_link_pose"), "true")
            self.assertEqual(pp.findtext("publish_model_pose"), "true")
            self.assertEqual(len(model.findall("plugin")), 6)
            self.assertEqual(model.findtext("link[@name='base_link']/collision/geometry/box/size"), "0.34 0.34 0.12")
            self.assertFalse(model.findall("link/sensor"))


class AblationTraceTests(unittest.TestCase):
    def fixture(self, folder, arm="full"):
        old, new = [1, "old"], [2, "new"]
        reason = "epoch-disagreement" if arm == "full" else "boundary-violated"
        events = [dict(event="snapshot_observed", t_ns=1, identity=old, ready=True),
            dict(event="initial_admission", t_ns=2, identity=old),
            dict(event="snapshot_observed", t_ns=12, identity=new, ready=False),
            dict(event="command", t_ns=13, velocity=[.2, 0, 0]),
            dict(event="hold", t_ns=20, authority=new, execution=old, reason=reason)]
        result = dict(result="PASS", arm=arm, repetition=1, metrics=dict(
            durable_identity=new, commit_ns=10, commit_to_observed_ms=.000002,
            commit_to_hold_ms=.00001, hold_reason=reason))
        (folder/"result.json").write_text(json.dumps(result))
        (folder/"runner_events.jsonl").write_text("\n".join(map(json.dumps, events)))
        (folder/"manager_events.jsonl").write_text(json.dumps(
            dict(event="durable_commit", t_ns=10, epoch=2, snapshot_hash="new")))
        return events, result

    def test_new_unready_authority_does_not_satisfy_common_readiness_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for arm in ("full", "geometry_only"):
                self.fixture(folder, arm)
                output = audit_ablation_trial(folder)
                self.assertFalse(output["common_readiness_stop_at_new_observation"])
                self.assertFalse(output["ready_at_hold"])
                self.assertEqual(output["moving_commands_after_new_observation_before_hold"], 1)

    def test_initial_unready_release_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            events, _ = self.fixture(folder)
            events[0]["ready"] = False
            (folder/"runner_events.jsonl").write_text("\n".join(map(json.dumps, events)))
            with self.assertRaisesRegex(ValueError, "initial release"):
                audit_ablation_trial(folder)

    def test_fabricated_hold_timing_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            _, result = self.fixture(folder)
            result["metrics"]["commit_to_hold_ms"] = 1000
            (folder/"result.json").write_text(json.dumps(result))
            with self.assertRaisesRegex(ValueError, "HOLD latency"):
                audit_ablation_trial(folder)


if __name__ == "__main__":
    unittest.main()
