"""Persistent vote clients for RuleSync 2PC rounds (latency optimization).

Replaces per-round bash + vote_once.py subprocesses (each costs a fresh
interpreter + rclpy.init + DDS discovery, ~2-4 s per round) with
persistent BridgeVote service clients. Transport only: same service,
same request/response semantics, same timeout values, same fail-closed
mapping (any error/missing vote -> TIMEOUT -> abort unless all READY).

Why in-process works HERE (the old code documents in-process vote
collection as unreliable): vote traffic runs on an ISOLATED node with
a DEDICATED spinner thread. No service future is ever waited inside
the executor/callback-group that must service it — the manager worker
thread only polls future.done() while the coordinator thread spins.
No nested spins, no shared callback groups with the serving path,
long-lived or otherwise. Discovery happens once (prewarm) and is kept
alive by continuous spinning.
"""
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point32
from builtin_interfaces.msg import Time

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import BridgeVote

READY, REJECT, TIMEOUT = 0, 1, 2


def build_request(rules, epoch, op, txn_id="", target_hash=""):
    """rules: list of plain dicts (JSON-able, thread-safe to pass).

    Carries the full RuleSpec semantics (polygon z + validity window
    round-trip: the snapshot hash covers them, so dropping them would
    break manager/bridge hash agreement) plus the commit-certificate
    identity (txn_id/epoch/target_hash) for prepared-identity checks.
    """
    req = BridgeVote.Request()
    req.epoch = int(epoch)
    req.op = str(op)
    try:
        req.txn_id = str(txn_id)
        req.target_hash = str(target_hash)
    except Exception:
        pass
    rs = []
    for rd in rules:
        r = RuleSpec()
        r.rule_id = rd["rule_id"]
        r.epoch = int(rd["epoch"])
        r.rule_type = int(rd["rule_type"])
        r.severity = int(rd.get("severity", 1))
        r.frame_id = rd.get("frame_id", "map")
        r.polygon.points = [Point32(x=float(p[0]), y=float(p[1]),
                                    z=float(p[2]) if len(p) > 2 else 0.0)
                            for p in rd.get("points", [])]
        r.speed_limit = float(rd.get("speed_limit", 0.0))
        vf = rd.get("valid_from", [0, 0])
        vu = rd.get("valid_until", [3600, 0])
        try:
            r.valid_from = Time(sec=int(vf[0]), nanosec=int(vf[1]))
            r.valid_until = Time(sec=int(vu[0]), nanosec=int(vu[1]))
        except Exception:
            r.valid_from = Time(sec=0)
            r.valid_until = Time(sec=3600)
        rs.append(r)
    req.staged_rules = rs
    return req


class VoteCoordinator:
    """Owns persistent vote clients + spinner thread (one per manager)."""

    def __init__(self, participants, info=None):
        self._info = info or (lambda m: None)
        self._lock = threading.Lock()
        self._node = Node("rulesync_vote_coordinator")
        self._clients = {}
        for p in participants:
            self._ensure_client(p)
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self._node)
        self._thread = threading.Thread(target=self._spin,
                                        daemon=True,
                                        name="vote-coord-spin")
        self._thread.start()
        self._prewarmed = {}

    def _spin(self):
        try:
            self._ex.spin()
        except Exception:
            pass

    def _ensure_client(self, part):
        with self._lock:
            if part not in self._clients:
                self._clients[part] = self._node.create_client(
                    BridgeVote, f"/rulesync/{part}_vote")
            return self._clients[part]

    def prewarm(self, participants=None, timeout_each=10.0):
        """Discovery once at startup. Returns {part: bool}. Called
        before the manager reports SYSTEM_READY (readiness barrier)."""
        out = {}
        for p in (participants or list(self._clients)):
            cli = self._ensure_client(p)
            try:
                ok = cli.wait_for_service(timeout_sec=timeout_each)
            except Exception:
                ok = False
            out[p] = bool(ok)
            self._info(f"prewarm vote service /rulesync/{p}_vote: "
                       f"{'ready' if ok else 'MISSING'}")
        self._prewarmed.update(out)
        return dict(out)

    def round(self, rules, epoch, op, timeout, txn_id="",
              target_hash=""):
        """Send to ALL participants concurrently; poll to a monotonic
        deadline (no spinning here). Returns {part: (vote, reason, dt)}.
        Missing/error/timeout -> TIMEOUT (fail-closed preserved)."""
        t_send = time.monotonic()
        futs = {}
        for p in list(self._clients):
            try:
                futs[p] = self._clients[p].call_async(
                    build_request(rules, epoch, op, txn_id, target_hash))
            except Exception as e:
                futs[p] = None
                self._pending_err = getattr(self, "_pending_err", {})
                self._pending_err[p] = str(e)[:120]
        votes = {}
        errs = getattr(self, "_pending_err", {})
        self._pending_err = {}
        deadline = t_send + float(timeout)
        pending = {p: f for p, f in futs.items() if f is not None}
        for p, f in futs.items():
            if f is None:
                votes[p] = (TIMEOUT, "coordinator-call-failed: " +
                            errs.get(p, ""), 0.0)
        while pending and time.monotonic() < deadline:
            for p in list(pending):
                f = pending[p]
                try:
                    done = f.done()
                except Exception:
                    done = False
                if done:
                    try:
                        r = f.result()
                        votes[p] = (int(r.vote), str(r.reason),
                                    time.monotonic() - t_send)
                    except Exception as e:
                        votes[p] = (TIMEOUT,
                                    "coordinator-error: " + str(e)[:120],
                                    time.monotonic() - t_send)
                    del pending[p]
            if pending:
                time.sleep(0.005)
        for p in pending:
            try:
                pending[p].cancel()
            except Exception:
                pass
            votes[p] = (TIMEOUT, "coordinator-timeout", float(timeout))
        return votes

    def shutdown(self):
        try:
            self._ex.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
