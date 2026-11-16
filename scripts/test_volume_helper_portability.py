"""ROS-free tests of the actual manager helper subprocess boundary."""
import ast
import json
import os
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock, patch


def actual_round():
    root = Path(__file__).resolve().parents[1]
    path = root / 'rulesync/src/rulesync_manager/rulesync_manager/manager_node.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == '_volume_round_body')
    import time
    scope = {"time": time, "TIMEOUT": 2}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
    return scope['_volume_round_body']


class HelperTests(unittest.TestCase):
    def run_helper(self, reply, override=True):
        volume = N(rule_id="rule's name", epoch=2, rule_type=0, severity=1,
            frame_id='map', geometry_type=0, center=N(x=1., y=2., z=3.),
            dimensions=N(x=1., y=1., z=1.), vertices=[],
            valid_from=N(sec=0, nanosec=0), valid_until=N(sec=3600, nanosec=0))
        node = N(get_logger=Mock(return_value=Mock()), log_txn=Mock())
        env = dict(os.environ, ROS_DOMAIN_ID='59', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
        env.pop('RULESYNC_VOLUME_VOTE_HELPER', None)
        if override:
            env['RULESYNC_VOLUME_VOTE_HELPER'] = '/tmp/space path/helper.py'
        with patch.dict(os.environ, env, clear=True), patch('subprocess.run', return_value=reply) as run:
            votes = actual_round()(node, [volume], 2, 'prepare', 30., 'txn', 'digest')
            return votes, run.call_args

    def test_portable_payload_and_certificate_remain_separate_arguments(self):
        votes, call = self.run_helper(N(returncode=0, stdout='VOTE=0 REASON=ready\n'))
        self.assertEqual(votes, {'uav_bridge': 0})
        command = call.args[0]
        self.assertEqual(command[1], '/tmp/space path/helper.py')
        self.assertEqual(json.loads(command[5])[0]['rule_id'], "rule's name")
        self.assertEqual(command[6:], ['txn', 'digest'])
        self.assertEqual(call.kwargs['env']['ROS_DOMAIN_ID'], '59')
        self.assertEqual(call.kwargs['env']['FASTDDS_BUILTIN_TRANSPORTS'], 'UDPv4')

    def test_legacy_default_profile_is_preserved(self):
        _, call = self.run_helper(N(returncode=0, stdout='VOTE=1 REASON=reject'), override=False)
        self.assertEqual(call.args[0][:2], ['bash', '-c'])
        self.assertIn('/opt/ros/lyrical/setup.bash', call.args[0][2])

    def test_nonzero_exit_cannot_release_even_if_stdout_says_ready(self):
        votes, _ = self.run_helper(N(returncode=1, stdout='VOTE=0 REASON=ready'))
        self.assertEqual(votes, {'uav_bridge': 2})

    def test_missing_or_invalid_vote_fails_closed(self):
        for stdout in ('', 'VOTE=9 REASON=bad', 'VOTE=no REASON=bad'):
            votes, _ = self.run_helper(N(returncode=0, stdout=stdout))
            self.assertEqual(votes, {'uav_bridge': 2})


if __name__ == '__main__':
    unittest.main()
