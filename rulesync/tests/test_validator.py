"""Unit tests for deterministic RuleSpec validation (no ROS needed)."""
import sys
import types
import unittest


def _load_validator():
    # Import validator.py standalone by path (no rclpy needed).
    import importlib.util
    path = __import__("pathlib").Path(__file__).resolve().parent.parent / \
        "src" / "rulesync_manager" / "rulesync_manager" / "validator.py"
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


if __name__ == "__main__":
    unittest.main()
