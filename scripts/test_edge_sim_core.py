import math
import unittest

from edge_sim_core import (feedback_fresh, pending_release_matches, release_matches,
                           step_unicycle, track_waypoints)


class EdgeCoreTests(unittest.TestCase):
    def test_straight(self):
        self.assertEqual(step_unicycle((0., 0., 0.), (.5, 0.), 2.), (1., 0., 0.))

    def test_arc(self):
        x, y, _ = step_unicycle((0., 0., 0.), (1., 1.), math.pi / 2)
        self.assertAlmostEqual(x, 1.)
        self.assertAlmostEqual(y, 1.)

    def test_zero_command_does_not_translate(self):
        self.assertEqual(step_unicycle((1., 2., 0.), (0., 0.), .1), (1., 2., 0.))

    def test_reject_invalid_dt(self):
        with self.assertRaises(ValueError):
            step_unicycle((0., 0., 0.), (0., 0.), -.1)

    def test_turn_before_advancing(self):
        velocity, index = track_waypoints((0., 0., 0.), [(0., 1.)], 0)
        self.assertEqual(velocity[0], 0.)
        self.assertGreater(velocity[1], 0.)
        self.assertEqual(index, 0)

    def test_goal_stop(self):
        velocity, index = track_waypoints((0., 0., 0.), [(0., 0.)], 0)
        self.assertEqual(velocity, (0., 0.))
        self.assertEqual(index, 1)

    def test_feedback_expiry_and_clock(self):
        self.assertFalse(feedback_fresh(1., None, .5))
        self.assertTrue(feedback_fresh(1., .6, .5))
        self.assertFalse(feedback_fresh(1.2, .6, .5))
        self.assertFalse(feedback_fresh(.5, .6, .5))

    def test_path_before_matching_state_is_not_released(self):
        self.assertFalse(release_matches((1, 'h'), (0, 'old'), 'old-token', 'new-token'))
        self.assertTrue(release_matches((1, 'h'), (1, 'h'), 'new-token', 'new-token'))

    def test_wrong_content_token_is_not_released(self):
        self.assertFalse(release_matches((1, 'h'), (1, 'h'), 'different-path', 'new-token'))

    def test_empty_token_is_not_released(self):
        self.assertFalse(release_matches((1, 'h'), (1, 'h'), '', ''))

    def test_expired_pending_path_matching_state_cannot_activate(self):
        self.assertFalse(pending_release_matches(3.001, 1., (1, 'h'), (1, 'h'), 't', 't'))

    def test_pending_path_deadline_is_inclusive(self):
        self.assertTrue(pending_release_matches(3., 1., (1, 'h'), (1, 'h'), 't', 't'))

    def test_active_identity_change_blocks_even_with_same_token(self):
        self.assertFalse(release_matches((2, 'new'), (1, 'old'), 't', 't'))
        self.assertFalse(release_matches((1, 'new'), (1, 'old'), 't', 't'))

    def test_future_or_missing_pending_timestamp_cannot_activate(self):
        for received in (None, 2.):
            self.assertFalse(pending_release_matches(1., received, (1, 'h'), (1, 'h'), 't', 't'))


if __name__ == '__main__':
    unittest.main()
