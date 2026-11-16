import tempfile
import unittest
from pathlib import Path

from launch_edge_sim import output_path, runtime_command, check_temperature_limit


class LauncherTests(unittest.TestCase):
    def test_historical_temperature_guard_retained(self):
        check_temperature_limit({'cpu': 84.9})
        for temperatures in ({'cpu': 85.}, {'cpu': 90.}, {}):
            with self.assertRaises(RuntimeError):
                check_temperature_limit(temperatures)

    def test_observe_only_has_no_artificial_temperature_abort(self):
        for temperatures in ({'cpu': 90.}, {}):
            check_temperature_limit(temperatures, None)

    def test_output_stays_inside_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(output_path(root, 'edge-v16'), root.resolve() / 'evidence/edge-v16')

    def test_invalid_names(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in ('../old', '/tmp/new', 'with space', '', 'a' * 65):
                with self.assertRaises(ValueError):
                    output_path(Path(directory), name)

    def test_existing_evidence_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'evidence/old').mkdir(parents=True)
            with self.assertRaises(FileExistsError):
                output_path(root, 'old')

    def test_symlink_outside_workspace_rejected(self):
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryDirectory() as outside:
            root = Path(directory)
            (root / 'evidence').symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                output_path(root, 'new')

    def test_runtime_paths_are_arguments_not_shell_interpolation(self):
        root = Path('/tmp/workspace with spaces')
        command = runtime_command(root, root / 'evidence/new')
        self.assertIn(str(root / 'install/setup.bash'), command)
        self.assertIn(str(root / 'source/scripts/run_edge_closed_loop.py'), command)
        self.assertIn('0-3', command)
        self.assertIn('UDPv4', command)
        self.assertNotIn(str(root), command[4])


if __name__ == '__main__':
    unittest.main()
