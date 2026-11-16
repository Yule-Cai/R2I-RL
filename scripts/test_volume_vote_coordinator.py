"""ROS-free boundary tests executing the real request/round method ASTs.

Fake message/future transports only; separate board tests use ROS messages.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as N
import time
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1] / 'rulesync/src/rulesync_manager/rulesync_manager'


def method(file, name, scope):
    node = next(n for n in ast.walk(ast.parse((ROOT/file).read_text()))
                if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(ROOT/file), 'exec'), scope)
    return scope[name]


class VolumeTests(unittest.TestCase):
    def test_complete_request_and_certificate(self):
        builder = method('volume_vote_coordinator.py', 'build_volume_request',
            dict(BridgeVoteVolume=N(Request=N), RuleVolume=lambda: N(center=N(), dimensions=N()),
                 Point=N, Time=N))
        data = dict(rule_id='v', epoch=3, rule_type=0, severity=1, frame_id='map',
            geometry_type=2, center=[1., 2., 3.], dims=[4., 5., 6.],
            vertices=[[.1, .2, .3]], valid_from=[7, 8], valid_until=[9, 10])
        req = builder([data], 3, 'prepare', 'txn', 'hash')
        self.assertEqual((req.txn_id, req.epoch, req.target_hash), ('txn', 3, 'hash'))
        self.assertEqual(req.volume_rules[0].valid_until.nanosec, 10)
        self.assertEqual(req.volume_rules[0].vertices[0].z, .3)
        self.assertEqual(req.volume_rules[0].dimensions.y, 5.)

    def run_round(self, future=None, error=None, timeout=.01):
        cli = Mock()
        if error:
            cli.call_async.side_effect = error
        else:
            cli.call_async.return_value = future
        coordinator = N(_clients={'uav_bridge': cli}, _request_builder=Mock(return_value='request'))
        round_fn = method('vote_coordinator.py', 'round', dict(time=time, TIMEOUT=2))
        return round_fn(coordinator, [], 3, 'commit', timeout, 'txn', 'hash')

    def test_real_round_forwards_ready_and_reject(self):
        for vote in (0, 1):
            result = self.run_round(N(done=lambda: True,
                result=lambda: N(vote=vote, reason='reply')))
            self.assertEqual(result['uav_bridge'][:2], (vote, 'reply'))

    def test_call_exception_fails_closed(self):
        self.assertEqual(self.run_round(error=RuntimeError('disconnected'))['uav_bridge'][0], 2)

    def test_response_exception_fails_closed(self):
        future = Mock()
        future.done.return_value = True
        future.result.side_effect = RuntimeError('bad response')
        self.assertEqual(self.run_round(future)['uav_bridge'][0], 2)

    def test_deadline_cancels_pending_no_late_release(self):
        future = Mock()
        future.done.return_value = False
        result = self.run_round(future)
        self.assertEqual(result['uav_bridge'][0], 2)
        future.cancel.assert_called_once()
        future.result.assert_not_called()

    def test_missing_client_result_has_no_ready_vote(self):
        node = N(_volume_coord=N(round=Mock(return_value={})), log_txn=Mock())
        fn = method('manager_node.py', '_volume_round_body', dict(time=time, TIMEOUT=2))
        self.assertEqual(fn(node, [], 3, 'prepare', .01, 'txn', 'hash'), {'uav_bridge': 2})

    def test_invalid_volume_votes_fail_closed(self):
        class FakeBase:
            def round(self, *args, **kwargs):
                return {'uav_bridge': (9, 'invalid', .1)}
        scope = dict(VoteCoordinator=FakeBase, TIMEOUT=2)
        tree = ast.parse((ROOT/'volume_vote_coordinator.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(ROOT), 'exec'), scope)
        obj = object.__new__(scope['VolumeVoteCoordinator'])
        self.assertEqual(obj.round([])['uav_bridge'], (2, 'invalid-volume-vote', .1))

    def test_manager_forwards_certificate_without_helper_retry(self):
        coord = N(round=Mock(return_value={'uav_bridge': (1, 'certificate mismatch', .01)}))
        node = N(_volume_coord=coord, log_txn=Mock())
        fn = method('manager_node.py', '_volume_round_body', dict(time=time, TIMEOUT=2))
        self.assertEqual(fn(node, [], 3, 'commit', .1, 'txn', 'hash'), {'uav_bridge': 1})
        coord.round.assert_called_once_with([], 3, 'commit', .1, 'txn', 'hash')


if __name__ == '__main__':
    unittest.main()
