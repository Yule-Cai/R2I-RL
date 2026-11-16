"""Transaction tracing helpers (latency optimization campaign).

- monotonic_ms(): monotonic clock in ms (all txn timestamps use it).
- MaskWatcher: subscribes /rulesync/keepout_mask, records generation
  counter + last nonzero count + last-change t_mono.
- wait_mask_ready(...): epoch-aware mask-applied readiness. Polls
  ListRules until committed_epoch >= epoch AND a fresh mask generation
  is observed, up to timeout_s. Returns (ready, wait_ms, detail).
  Bounded and non-weakening: callers still run the deterministic gate;
  a False return only records mask_not_ready (the gate then decides).
"""
import time

MONO = time.monotonic


def monotonic_ms():
    return time.monotonic() * 1000.0


class MaskWatcher:
    def __init__(self, node, topic="/rulesync/keepout_mask"):
        from nav_msgs.msg import OccupancyGrid
        self.gen = 0
        self.nonzero = 0
        self.t_mono = None
        self._last_sig = None
        node.create_subscription(OccupancyGrid, topic, self._cb, 10)

    def _cb(self, msg):
        try:
            nz = sum(1 for v in msg.data if v > 0)
        except Exception:
            return
        sig = (len(msg.data), nz)
        if sig != self._last_sig:
            self._last_sig = sig
            self.gen += 1
            self.nonzero = nz
            self.t_mono = time.monotonic()

    def snapshot(self):
        return {"gen": self.gen, "nonzero": self.nonzero,
                "t_mono": self.t_mono}


def wait_mask_ready(node, watcher, list_client, epoch, timeout_s=3.0,
                    base_gen=None):
    """Wait until manager committed_epoch >= epoch AND mask gen advanced
    past the pre-commit baseline. Returns (ready, wait_ms, detail).

    base_gen: mask generation snapshot taken AT COMMIT time (default:
    current generation — but by fallback time the mask has usually
    already landed, so callers should snapshot right after commit).
    Bounded; never weakens the gate (caller still gates)."""
    from rulesync_interfaces.srv import ListRules
    t0 = time.monotonic()
    if base_gen is None:
        base_gen = watcher.snapshot()["gen"]
    detail = {"committed": None, "gen0": base_gen, "gen1": base_gen}
    if not list_client.wait_for_service(timeout_sec=min(5.0, timeout_s)):
        detail["error"] = "list_rules undiscovered"
        return False, (time.monotonic() - t0) * 1000.0, detail
    while time.monotonic() - t0 < timeout_s:
        fut = list_client.call_async(ListRules.Request())
        rclpy_spin(node, fut, timeout=2.0)
        try:
            res = fut.result()
        except Exception:
            res = None
        if res is not None:
            detail["committed"] = int(res.epoch)
            if int(res.epoch) >= int(epoch):
                snap = watcher.snapshot()
                detail["gen1"] = snap["gen"]
                if snap["gen"] > base_gen:
                    return True, (time.monotonic() - t0) * 1000.0, \
                        detail
        time.sleep(0.2)
    snap = watcher.snapshot()
    detail["gen1"] = snap["gen"]
    return False, (time.monotonic() - t0) * 1000.0, detail


def rclpy_spin(node, fut, timeout=2.0):
    import rclpy
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
