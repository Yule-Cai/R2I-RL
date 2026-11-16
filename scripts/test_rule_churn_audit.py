"""ROS-free tests for full-identity timeline integration and censoring."""
import unittest
from audit_rule_churn import integrate, update_metrics


def commit(t, epoch, sha="a"):
    return dict(event="durable_commit", t_ns=t, epoch=epoch, snapshot_hash=sha)


def state(t, epoch, sha="a", value="CLEAR"):
    return dict(event="guard_state", t_ns=t, authority=[epoch, sha],
                execution=[epoch, sha], state=value, degraded=False, reason="test")


def release(t, epoch, sha="a"):
    return dict(event="path_release", t_ns=t, execution=[epoch, sha], state="CLEAR")


class TimelineTests(unittest.TestCase):
    def test_partition_and_prefix(self):
        rows = [commit(1, 1), state(2, 1), release(3, 1), commit(20, 2),
                state(25, 2, value="HOLD"), dict(event="path_stop", t_ns=30),
                state(40, 2), release(45, 2), commit(90, 3)]
        result = integrate(rows, 10, 60)
        self.assertEqual(result["durations_ns"], dict(valid=25, stale_release=10, blocked=15))
        self.assertEqual(sum(result["fractions"].values()), 1)
        self.assertEqual(result["mismatched_new_releases"], [])

    def test_hash_not_epoch_only(self):
        rows = [commit(1, 1, "b"), state(2, 1), release(12, 1)]
        result = integrate(rows, 10, 20)
        self.assertEqual(result["durations_ns"]["valid"], 0)
        self.assertEqual(len(result["mismatched_new_releases"]), 1)

    def test_ready_is_not_release(self):
        result = integrate([commit(1, 1), state(2, 1)], 10, 20)
        self.assertEqual(result["fractions"]["blocked"], 1)

    def test_hold_with_current_path_is_blocked(self):
        rows = [commit(1, 1), release(2, 1), state(3, 1, value="HOLD")]
        self.assertEqual(integrate(rows, 10, 20)["fractions"]["blocked"], 1)

    def test_superseded_and_window_censoring(self):
        rows = [commit(1, 1), commit(20, 2), commit(30, 3), release(40, 2), release(61, 3)]
        result = update_metrics(rows, 10, 60)
        self.assertEqual(len(result), 2)
        self.assertTrue(all(not r["restored_before_supersession"] for r in result))
        self.assertEqual([r["censor_ns"] for r in result], [30, 60])

    def test_precommit_hold_not_postcommit_latency(self):
        rows = [commit(1, 1), state(19, 1, value="HOLD"), commit(20, 2), release(40, 2)]
        result = update_metrics(rows, 10, 60)[0]
        self.assertIsNone(result["commit_to_hold_ms"])
        self.assertTrue(result["restored_before_supersession"])

    def test_invalid_window(self):
        with self.assertRaises(ValueError):
            integrate([], 10, 10)


if __name__ == "__main__":
    unittest.main()
