"""ROS-free tests of the independent capture's frame and matching contract."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from gazebo_world_pose import (instrument_robot, matched_errors, validate_transform,
                              validate_capture_health, validate_active_sequence)


class WorldPoseContractTests(unittest.TestCase):
    def test_exact_entity_and_world(self):
        validate_transform("default", "turtlebot3_waffle", 1,
                           (-2, -.5, .01), (0, 0, 0, 1))
        for parent, child in (("odom", "turtlebot3_waffle"),
                              ("", "turtlebot3_waffle"),
                              ("default", "other_robot"),
                              ("default", "turtlebot3_waffle::base_link")):
            with self.subTest(parent=parent, child=child):
                with self.assertRaises(ValueError):
                    validate_transform(parent, child, 1, (0, 0, 0), (0, 0, 0, 1))

    def test_invalid_values(self):
        for stamp, xyz, q in ((0, (0, 0, 0), (0, 0, 0, 1)),
                              (1, (math.nan, 0, 0), (0, 0, 0, 1)),
                              (1, (math.inf, 0, 0), (0, 0, 0, 1)),
                              (1, (0, 0, 0), (0, 0, 0, 0)),
                              (1, (0, 0, 0), (0, 0, 0, 2))):
            with self.subTest(stamp=stamp, xyz=xyz, q=q):
                with self.assertRaises(ValueError):
                    validate_transform("default", "turtlebot3_waffle", stamp, xyz, q)

    def test_missing_stale_or_rejected_capture_fails(self):
        validate_capture_health([{"receipt_ns": 1}], [], 100_000_001)
        for rows, errors, now in (([], [], 1),
                                  ([{"receipt_ns": 1}], [], 300_000_001),
                                  ([{"receipt_ns": 1}], ["wrong frame"], 1),
                                  ([{"receipt_ns": 2}], [], 1)):
            with self.subTest(rows=rows, errors=errors, now=now):
                with self.assertRaises(RuntimeError):
                    validate_capture_health(rows, errors, now)

    def test_missing_duplicate_regressing_and_gapped_sequence_fails(self):
        good = [{"sim_ns": i * 21_000_000} for i in range(1, 21)]
        self.assertEqual(len(validate_active_sequence(good)), 19)
        duplicate = good[:-1] + [{"sim_ns": good[-2]["sim_ns"]}]
        regressing = good[:-1] + [{"sim_ns": 1}]
        gapped = good[:-1] + [{"sim_ns": good[-2]["sim_ns"] + 101_000_000}]
        for rows in ([], good[:19], duplicate, regressing, gapped):
            with self.subTest(rows=rows):
                with self.assertRaises(RuntimeError):
                    validate_active_sequence(rows)

    def test_match_by_sim_not_receipt_stamp(self):
        worlds = [{"sim_ns": 1_000_000_000, "x": 1, "y": 0},
                  {"sim_ns": 1_030_000_000, "x": 2, "y": 0}]
        odoms = [{"sim_ns": 1_025_000_000, "x": 2, "y": .1},
                 {"sim_ns": 1_070_000_000, "x": 999, "y": 0}]
        errors, gaps = matched_errors(worlds, odoms)
        self.assertEqual(len(errors), 1)
        self.assertAlmostEqual(errors[0], .1)
        self.assertAlmostEqual(gaps[0], .005)
        self.assertEqual(matched_errors([], odoms), ([], []))

    def test_model_instrumentation_is_additive(self):
        with tempfile.TemporaryDirectory() as folder:
            original = ('<sdf><model name="turtlebot3_waffle">'
                        '<link name="base_footprint"/>'
                        '<plugin name="drive" filename="drive.so"/>'
                        '</model></sdf>')
            def xacro(command, **kwargs):
                Path(command[2]).write_text(original)
            with patch("gazebo_world_pose.subprocess.run", side_effect=xacro):
                output = instrument_robot("original.xacro", folder, {})
            model = ET.parse(output).getroot().find("model")
            self.assertIsNotNone(model.find("plugin[@name='drive']"))
            added = model.find("plugin[@name='gz::sim::systems::PosePublisher']")
            self.assertEqual(added.findtext("publish_model_pose"), "true")
            self.assertEqual(added.findtext("publish_link_pose"), "false")
            self.assertEqual(added.findtext("topic"), "/rulesync_test/world_pose")


if __name__ == "__main__":
    unittest.main()
