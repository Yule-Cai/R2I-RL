"""ROS-free recovery checks and falsification of event-audit acceptance."""
import ast
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as N

from volume_task_recovery import (audit_task, backends, path_digest, propose_detour,
    release_checks)
from benchmark_dynamic_volume_update import close_trial

START = (1.45, 10.09, 3.)
GOAL = (6., 11., 3.5)
VOLUMES = [dict(geometry_type=0, bmin=[3.,10.,2.], bmax=[4.,11.,3.5])]
IDENTITY = [2,"b"*64]


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.path, _ = propose_detour(START, GOAL, VOLUMES, .4)

    def checks(self, **changes):
        fields = dict(expected=IDENTITY, authority=IDENTITY, participant=IDENTITY,
            ready=True, pose_age=.01, snapshot_age=.02, path=self.path,
            position=START, goal=GOAL, volumes=VOLUMES, rho=.4)
        fields.update(changes)
        return release_checks(**fields)

    def test_existing_proposer_detours_and_gate_admits(self):
        self.assertGreater(len(self.path),2)
        self.assertEqual(tuple(self.path[0]),START)
        self.assertEqual(tuple(self.path[-1]),GOAL)
        self.assertEqual(self.checks(),"gate3d:ok")

    def test_capture_freshness_is_checked_before_offline_audit(self):
        tree = ast.parse(Path(__file__).with_name("benchmark_dynamic_volume_update.py").read_text())
        trial = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="run_trial")
        stale = [n.lineno for n in ast.walk(trial) if isinstance(n,ast.Raise)
            and "independent world capture stale" in ast.unparse(n)]
        audit = [n.lineno for n in ast.walk(trial) if isinstance(n,ast.Call)
            and isinstance(n.func,ast.Name) and n.func.id=="audit_task"]
        self.assertEqual(len(stale),1)
        self.assertEqual(len(audit),1)
        self.assertLess(stale[0],audit[0])

    def test_direct_route_is_rejected(self):
        with self.assertRaises(ValueError):
            self.checks(path=[START,GOAL])

    def test_not_ready_cannot_release(self):
        with self.assertRaises(ValueError): self.checks(ready=False)

    def test_same_epoch_wrong_hash_cannot_release(self):
        with self.assertRaises(ValueError): self.checks(participant=[2,"c"*64])

    def test_old_authority_cannot_release(self):
        with self.assertRaises(ValueError): self.checks(authority=[1,"a"*64])

    def test_stale_pose_cannot_release(self):
        with self.assertRaises(ValueError): self.checks(pose_age=.26)

    def test_expired_snapshot_cannot_release(self):
        with self.assertRaises(ValueError): self.checks(snapshot_age=1.01)

    def test_changed_endpoint_cannot_release(self):
        with self.assertRaises(ValueError): self.checks(position=(1.6,10.1,3.))

    def fixture(self):
        release = dict(event="recovery_release", t_ns=5, identity=IDENTITY,
            participant=IDENTITY, execution=IDENTITY, ready=True, path=self.path,
            path_digest=path_digest(self.path), xyz=START, rho=.4,
            pose_age_s=.01,snapshot_age_s=.02)
        events = [dict(event="durable_commit",t_ns=1,epoch=2,snapshot_hash=IDENTITY[1]),
            dict(event="hold",t_ns=2),dict(event="stopped_before_recovery",t_ns=3),
            dict(event="old_route_recheck",t_ns=4,allowed=False), release,
            dict(event="command",t_ns=6,velocity=[.1,0,0],execution=IDENTITY,
                 path_digest=release["path_digest"]),dict(event="goal_reached",t_ns=7,goal=GOAL)]
        rows = [dict(active=True,receipt_ns=0,sim_ns=0,xyz=START)] + [
            dict(active=True,receipt_ns=i,sim_ns=i*20_000_000,xyz=GOAL) for i in range(8,14)]
        wal = [dict(decision="COMMIT",epoch=2,target_hash=IDENTITY[1],target_volumes=[
            dict(center=[3.5,10.5,2.75],dimensions=[1.,1.,1.5])])]
        return events, rows, wal

    def audit(self, fixture):
        events, rows, wal = fixture
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            for name, value in (("runner_events.jsonl",events),("world_pose.jsonl",rows),("wal.jsonl",wal)):
                (folder/name).write_text("".join(json.dumps(r)+"\n" for r in value))
            return audit_task(folder)

    def test_task_audit_accepts_complete_chain(self):
        self.assertTrue(self.audit(self.fixture())["task_completed"])

    def test_task_audit_denies_pre_release_motion(self):
        data = self.fixture()
        data[0].append(dict(event="command",t_ns=4,velocity=[.1,0,0]))
        with self.assertRaises(ValueError): self.audit(data)

    def test_task_audit_denies_command_identity_mismatch(self):
        data = self.fixture()
        data[0][5]["execution"] = [1,"a"*64]
        with self.assertRaises(ValueError): self.audit(data)

    def test_task_audit_denies_unconfirmed_arrival(self):
        data = self.fixture()
        data[1][-1]["xyz"] = START
        with self.assertRaises(ValueError): self.audit(data)

    def test_task_audit_denies_tampered_path(self):
        data = self.fixture()
        data[0][4]["path_digest"] = "bad"
        with self.assertRaises(ValueError): self.audit(data)

    def test_task_audit_denies_rebind_before_stop(self):
        data = self.fixture()
        data[0][2]["t_ns"] = 6
        with self.assertRaises(ValueError): self.audit(data)

    def test_shutdown_context_skips_publish_but_closes_children(self):
        calls = []
        node = N(tracking=True, control=lambda: calls.append("publish"),
            destroy_node=lambda: calls.append("destroy"))
        ros = N(ok=lambda:False, try_shutdown=lambda:calls.append("shutdown"))
        with tempfile.TemporaryDirectory() as directory:
            close_trial(node,ros,N(close=lambda:calls.append("children")),Path(directory),dict(result="FAIL"))
            self.assertTrue((Path(directory)/"result.json").exists())
        self.assertEqual(calls,["destroy","shutdown","children"])

    def test_failed_cleanup_publish_cannot_orphan_children(self):
        calls = []
        def fail(): raise RuntimeError("invalid publisher")
        node = N(tracking=True,control=fail,destroy_node=lambda:calls.append("destroy"))
        ros = N(ok=lambda:True,try_shutdown=lambda:calls.append("shutdown"))
        record = dict(result="PASS")
        with tempfile.TemporaryDirectory() as directory:
            close_trial(node,ros,N(close=lambda:calls.append("children")),Path(directory),record)
        self.assertEqual(record["result"],"FAIL")
        self.assertEqual(calls,["destroy","shutdown","children"])
        self.assertEqual(len(record["cleanup_errors"]),1)


if __name__ == "__main__":
    unittest.main()
