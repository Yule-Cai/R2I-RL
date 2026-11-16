# ProtoCrash2D cell driver: executes the scripted 2D RuleSpec op (setup
# txns + test txn) against the live manager/nav2_bridge, writes the
# per-cell expectation file for the audit. Geometries are fixed squares
# far from the headless test pose/goal (RULESYNC_TEST_POSE2D/GOAL2D);
# epochs from argv. RELAX/SHRINK = same rule id, smaller KEEP_OUT
# geometry (the critical non-monotonic case); TIGHTEN = same id, bigger.
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import (ProposeRule, CommitRule, RemoveRule)
from geometry_msgs.msg import Point32
from builtin_interfaces.msg import Time


def stamp():
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


def mk(rid, epoch, pts, rtype=0, speed=0.0):
    r = RuleSpec()
    r.rule_id = rid
    r.epoch = int(epoch)
    r.rule_type = int(rtype)
    r.severity = 1
    r.frame_id = "map"
    r.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                        for x, y in pts]
    r.speed_limit = float(speed)
    r.valid_from = stamp()
    e = stamp()
    e.sec += 3600
    r.valid_until = e
    return r


P0 = [(10.0, 10.0), (12.0, 10.0), (12.0, 12.0), (10.0, 12.0)]
PBIG = [(9.0, 9.0), (13.0, 9.0), (13.0, 13.0), (9.0, 13.0)]
PSMALL = [(10.5, 10.5), (11.5, 10.5), (11.5, 11.5), (10.5, 11.5)]
P1 = [(15.0, 10.0), (17.0, 10.0), (17.0, 12.0), (15.0, 12.0)]


class Drv(Node):
    def __init__(self):
        # Unique node name per process (same ghost-poisoning rationale
        # as the audits).
        import os as _os
        super().__init__(f"proto2d_op_{_os.getpid()}")
        self.cp = self.create_client(ProposeRule,
                                     "/rulesync/propose_rule")
        self.cc = self.create_client(CommitRule,
                                     "/rulesync/commit_rule")
        self.cr = self.create_client(RemoveRule,
                                     "/rulesync/remove_rule")
        from rulesync_interfaces.srv import ListRules as _LR0
        self.cl = self.create_client(_LR0, "/rulesync/list_rules")
        for c, nm in ((self.cp, "propose"), (self.cc, "commit"),
                      (self.cr, "remove"), (self.cl, "list")):
            for _ in range(3):
                try:
                    if c.wait_for_service(timeout_sec=30.0):
                        break
                except Exception:
                    pass
            else:
                raise RuntimeError(f"no-service:{nm}")

    def _await(self, fut, timeout=90.0):
        # Bounded wait that does NOT depend on spin_until_future_complete
        # timeout semantics: when the service server DIES mid-call (the
        # crash under test), the pending client future can outlive the
        # spin timeout under the discovery-server transport (verified
        # hang: 600 s despite timeout_sec=90). Wall-clock deadline here
        # bounds it regardless of DDS state.
        import time as _t
        t0 = _t.monotonic()
        while not fut.done() and _t.monotonic() - t0 < float(timeout):
            rclpy.spin_once(self, timeout_sec=1.0)
        return bool(fut.done())

    def _res(self, fut):
        try:
            return fut.result() if fut.done() else None
        except Exception:
            return None

    def _call(self, client, req, timeout=90.0):
        # One service round-trip with a single retry on SILENCE
        # (no response). Rationale: fresh clients right after a kill
        # can route into ghost-lease advertisements and vanish into
        # the void (verified: live-idle server never sees the call);
        # a retry past lease expiry reaches the live server. Explicit
        # answers (READY/REJECT, accepted or not) are FINAL — never
        # retried (a retry must not paper over real rejections, and
        # crash cells stay fail-fast on the second silence).
        f = client.call_async(req)
        self._await(f, timeout)
        r = self._res(f)
        if r is not None:
            return r
        time.sleep(30)
        f = client.call_async(req)
        self._await(f, timeout)
        return self._res(f)

    def pc(self, rule, *, timeout=90.0):
        from rulesync_interfaces.srv import ProposeRule as _PR
        from rulesync_interfaces.srv import CommitRule as _CR
        r1 = self._call(self.cp, _PR.Request(rule=rule), timeout)
        if r1 is None or not r1.accepted:
            return False, f"propose:{getattr(r1, 'reason', 'noresp-retry')}"
        r2 = self._call(self.cc, _CR.Request(epoch=int(rule.epoch)),
                        timeout)
        if r2 is None:
            return False, "commit-noresponse"
        ok = bool(r2.committed)
        return ok, ("" if ok else f"commit:{r2.reason}")

    def rm(self, rid, epoch, timeout=90.0):
        from rulesync_interfaces.srv import RemoveRule as _RR
        r = self._call(self.cr, _RR.Request(rule_id=rid,
                                            epoch=int(epoch)), timeout)
        if r is None:
            return False, "remove-noresponse"
        return bool(r.ok), ("" if r.ok else f"remove:{r.reason}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uid", required=True)
    ap.add_argument("--op", required=True)
    ap.add_argument("--epoch", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--expect", default="/tmp/proto2d_expect.json")
    ap.add_argument("--phase", default="both",
                    help="setup: setup txns + pre-state only; test: test "
                    "txn only (reads pre-state); both: legacy one-shot. "
                    "Crash cells use setup -> arm-restart -> test so the "
                    "crash fires during the TEST txn, never the setup.")
    a = ap.parse_args()
    # Pre-state sidecar (per-cell): setup writes, test reads.
    pre_path = a.expect + ".pre"
    exp = {"uid": a.uid, "op": a.op, "test_epoch": None,
           "target_ids": [], "setup_log": [],
           "pre_ids": [], "pre_epoch": -1, "test_result": ""}
    try:
        _run(a, exp, pre_path)
    except Exception as e:
        import traceback
        traceback.print_exc()
        exp["test_result"] = f"driver-error:{e}"
    finally:
        try:
            # Never leave the audit without input: best-effort expect
            # file even on driver failure (row adjudicates INVALID).
            with open(a.expect, "w") as f:
                json.dump(exp, f)
        except Exception:
            pass
        # Graceful shutdown on EVERY exit (same ghost rationale).
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _run(a, exp, pre_path):
    rclpy.init()
    n = Drv()
    E, log = a.epoch, []
    phase = str(a.phase or "both")
    if phase in ("setup", "both"):
        # setup phase (no crash): establish pre-state per op class
        if a.op in ("TIGHTEN", "REPLACE", "REMOVE", "SETUP_ONLY"):
            ok, rs = n.pc(mk("pc-r0", E, P0))
            log.append(f"setup-r0@{E}={ok}:{rs}")
            E += 10
        if a.op in ("RELAX",):
            # RELAX = same-id shrink (the critical relaxing-update case):
            # setup big, test small. Distinct from REMOVE (deletion).
            ok, rs = n.pc(mk("pc-r0", E, PBIG))
            log.append(f"setup-r0big@{E}={ok}:{rs}")
            E += 10
        if a.op in ("REPLACE",):
            ok, rs = n.rm("pc-r0", E)
            log.append(f"setup-rm-r0@{E}={ok}:{rs}")
            E += 10
        # Pre-test baseline (for lost/promoted adjudication).
        # Same silence-retry as op calls (a ghost-routed read would
        # silently corrupt adjudication).
        from rulesync_interfaces.srv import ListRules as _LR
        fr = n._call(n.cl, _LR.Request(), 30.0)
        exp["pre_ids"] = sorted(str(r.rule_id) for r in fr.active) \
            if fr is not None else []
        exp["pre_epoch"] = int(fr.epoch) if fr is not None else -1
        exp["setup_log"] = log
        exp["next_epoch"] = E
        with open(pre_path, "w") as f:
            json.dump({"setup_log": log, "pre_ids": exp["pre_ids"],
                       "pre_epoch": exp["pre_epoch"], "next_epoch": E}, f)
    if phase == "setup":
        n.destroy_node()
        rclpy.shutdown()
        return
    if phase == "test":
        try:
            pre = json.load(open(pre_path))
            exp["setup_log"] = pre.get("setup_log", [])
            exp["pre_ids"] = pre.get("pre_ids", [])
            exp["pre_epoch"] = pre.get("pre_epoch", -1)
            E = int(pre.get("next_epoch", a.epoch))
        except Exception as e:
            exp["test_result"] = f"driver-error:missing-prestate:{e}"
            n.destroy_node()
            rclpy.shutdown()
            return
    # TEST txn (crash hooks may fire inside these calls)
    # Pre-register the attempted test epoch/ids BEFORE the op call: a hung
    # or crashed op must still adjudicate (no-durable path) instead of
    # leaving test_epoch None.
    _targets = {"ADD": ["pc-r0"], "TIGHTEN": ["pc-r0"],
                "REPLACE": ["pc-r1"], "REMOVE": [], "RELAX": ["pc-r0"]}
    if a.op in _targets:
        exp.update(test_epoch=E, target_ids=_targets[a.op])
    if a.op == "ADD":
        ok, rs = n.pc(mk("pc-r0", E, P0))
        exp.update(test_epoch=E, target_ids=["pc-r0"])
    elif a.op == "TIGHTEN":
        ok, rs = n.pc(mk("pc-r0", E, PBIG))
        exp.update(test_epoch=E, target_ids=["pc-r0"])
    elif a.op == "REPLACE":
        ok, rs = n.pc(mk("pc-r1", E, P1))
        exp.update(test_epoch=E, target_ids=["pc-r1"])
    elif a.op == "REMOVE":
        ok, rs = n.rm("pc-r0", E)
        exp.update(test_epoch=E, target_ids=[])
    elif a.op == "RELAX":
        ok, rs = n.pc(mk("pc-r0", E, PSMALL))
        exp.update(test_epoch=E, target_ids=["pc-r0"])
    elif a.op == "SETUP_ONLY":
        # Restart cells: the setup commit IS the durable under test.
        ok, rs = True, "setup-only"
        exp.update(test_epoch=E - 10, target_ids=["pc-r0"])
        exp["test_result"] = f"{ok}:{rs}"
        print(f"[proto2d-op] {a.uid} {a.op} -> {exp['test_result']}",
              flush=True)
        n.destroy_node()
        rclpy.shutdown()
        return
    exp["test_result"] = f"{ok}:{rs}"
    print(f"[proto2d-op] {a.uid} {a.op} -> {exp['test_result']}",
          flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
