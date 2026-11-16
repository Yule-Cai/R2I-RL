"""Unit tests for deterministic RuleSpec validation (no ROS needed)."""
import sys
import types
import unittest


def _load_validator():
    # Import validator.py standalone by path (no rclpy needed).
    import importlib.util
    path = __import__("pathlib").Path(__file__).resolve().parent.parent / \
        "rulesync_manager" / "validator.py"
    spec = importlib.util.spec_from_file_location("rs_validator", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakePoint:
    def __init__(self, x, y):
        self.x, self.y = x, y


class FakeRule:
    def __init__(self, **kw):
        self.rule_id = kw.get("rule_id", "r1")
        self.epoch = kw.get("epoch", 1)
        self.rule_type = kw.get("rule_type", 0)
        self.severity = kw.get("severity", 1)
        self.frame_id = kw.get("frame_id", "map")
        self.polygon = types.SimpleNamespace(
            points=kw.get("points", [FakePoint(0, 0), FakePoint(1, 0),
                                     FakePoint(1, 1)]))
        self.speed_limit = kw.get("speed_limit", 0.0)
        self.valid_from = types.SimpleNamespace(sec=0)
        self.valid_until = types.SimpleNamespace(sec=100)


class ValidatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.v = _load_validator()

    def test_valid_keepout(self):
        ok, _ = self.v.validate_rule(FakeRule(), 0)
        self.assertTrue(ok)

    def test_valid_speed(self):
        ok, _ = self.v.validate_rule(
            FakeRule(rule_type=1, speed_limit=0.5), 0)
        self.assertTrue(ok)

    def test_nan_rejected(self):
        ok, reason = self.v.validate_rule(FakeRule(
            points=[FakePoint(0, 0), FakePoint(float("nan"), 0),
                    FakePoint(1, 1)]), 0)
        self.assertFalse(ok)
        self.assertIn("NaN", reason)

    def test_two_points_rejected(self):
        ok, _ = self.v.validate_rule(
            FakeRule(points=[FakePoint(0, 0), FakePoint(1, 1)]), 0)
        self.assertFalse(ok)

    def test_zero_area_rejected(self):
        ok, _ = self.v.validate_rule(FakeRule(
            points=[FakePoint(0, 0), FakePoint(1, 1), FakePoint(2, 2)]), 0)
        self.assertFalse(ok)

    def test_stale_epoch(self):
        ok, reason = self.v.validate_rule(FakeRule(epoch=3), 5)
        self.assertFalse(ok)
        self.assertIn("stale", reason)

    def test_bad_frame(self):
        ok, _ = self.v.validate_rule(FakeRule(frame_id="nope"), 0)
        self.assertFalse(ok)

    def test_bad_window(self):
        r = FakeRule()
        r.valid_until = types.SimpleNamespace(sec=0)
        ok, _ = self.v.validate_rule(r, 0)
        self.assertFalse(ok)

    def test_speed_needs_limit(self):
        ok, _ = self.v.validate_rule(FakeRule(rule_type=1, speed_limit=0.0), 0)
        self.assertFalse(ok)

    def test_unknown_type(self):
        ok, _ = self.v.validate_rule(FakeRule(rule_type=7), 0)
        self.assertFalse(ok)

    def test_invalid_polygon_topologies(self):
        cases = {
            "nonzero-area crossing": [(0, 0), (3, 3), (0, 3), (2, 0)],
            "symmetric bow tie": [(0, 0), (2, 2), (0, 2), (2, 0)],
            "adjacent duplicate": [(0, 0), (2, 0), (2, 0), (2, 2), (0, 2)],
            "nonadjacent duplicate": [(0, 0), (2, 0), (2, 2), (0, 0), (0, 2)],
            "vertex on nonadjacent edge": [(0, 0), (3, 0), (3, 3), (1, 0), (0, 3)],
            "adjacent backtrack": [(0, 0), (3, 0), (1, 0), (3, 2), (0, 2)],
            "closing-edge crossing": [(0, 0), (3, 0), (0, 3), (3, 3)],
            "nonadjacent overlap": [(0, 0), (4, 0), (4, 3), (1, 3), (1, 0), (3, 0), (3, 2), (0, 2)],
            "double closure": [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0), (0, 0)],
        }
        for name, points in cases.items():
            for ring in (points, list(reversed(points))):
                with self.subTest(case=name, ring=ring):
                    self.assertFalse(self.v.validate_polygon(ring)[0])

    def test_simple_polygons_preserve_representation(self):
        cases = [
            [(0, 0), (2, 0), (2, 2), (0, 2)],
            [(0, 0), (3, 0), (3, 3), (1, 1), (0, 3)],
            [(0, 0), (1, 0), (2, 0), (2, 2), (0, 2)],
            [(0, 0), (2, 0), (2, 1e-8), (0, 1e-8)],
        ]
        for points in cases:
            for ring in (points, list(reversed(points)), points + [points[0]]):
                original = list(ring)
                with self.subTest(ring=ring):
                    self.assertEqual(self.v.validate_polygon(ring), (True, "ok"))
                    self.assertEqual(ring, original)

    def test_rule_rejects_self_crossing_polygon(self):
        rule = FakeRule(points=[FakePoint(x, y) for x, y in
                               [(0, 0), (3, 3), (0, 3), (2, 0)]])
        ok, reason = self.v.validate_rule(rule, 0)
        self.assertFalse(ok)
        self.assertIn("self-intersection", reason)

    def test_infinite_coordinates_rejected(self):
        self.assertFalse(self.v.validate_polygon([(0, 0), (float("inf"), 0), (1, 1)])[0])

    def test_iterator_input_does_not_drop_first_vertex(self):
        crossing = [(0, 0), (3, 3), (0, 3), (2, 0)]
        self.assertFalse(self.v.validate_polygon(iter(crossing))[0])
        triangle = [(0, 0), (2, 0), (0, 2)]
        self.assertTrue(self.v.validate_polygon(iter(triangle))[0])

    def test_nearly_collinear_orientation_matches_exact_sign(self):
        from fractions import Fraction
        a, b, c = (0.0, 0.0), (1000.0, 1000.0), (1000.0, 1000.0000000000001)
        exact = Fraction(b[0]) * Fraction(c[1]) - Fraction(b[1]) * Fraction(c[0])
        self.assertEqual(self.v._orientation(a, b, c), (exact > 0) - (exact < 0))


if __name__ == "__main__":
    unittest.main()
