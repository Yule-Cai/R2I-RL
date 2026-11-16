import os
import unittest
from pathlib import Path
from unittest.mock import patch

from benchmark_dynamic_volume_update import trial_environment


class ProfileTests(unittest.TestCase):
    def test_default_retains_historical_discovery(self):
        env = trial_environment(Path('/tmp/trial'))
        self.assertEqual(env['ROS_DOMAIN_ID'], '0')
        self.assertEqual(env['ROS_DISCOVERY_SERVER'], '127.0.0.1:11811')

    def test_board_profile_removes_inherited_discovery_server(self):
        with patch.dict(os.environ, {'ROS_DISCOVERY_SERVER': 'old-host:11811'}):
            env = trial_environment(Path('/tmp/trial'), 59)
        self.assertNotIn('ROS_DISCOVERY_SERVER', env)
        self.assertEqual(env['ROS_DOMAIN_ID'], '59')
        self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        self.assertEqual(env['FASTDDS_BUILTIN_TRANSPORTS'], 'UDPv4')
        self.assertEqual(env['RULESYNC_CHURN_FOLDER'], '/tmp/trial')

    def test_invalid_domain_rejected(self):
        for domain in (-1, 233):
            with self.assertRaises(ValueError):
                trial_environment(Path('/tmp/trial'), domain)


if __name__ == '__main__':
    unittest.main()
