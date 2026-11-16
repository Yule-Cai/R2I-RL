"""Transaction failure-injection tests A-E (needs manager + bridge running).

A. silent participant (short prepare timeout) -> old epoch stays active
B. malformed polygon -> rejected before PREPARE
C. invalid TF frame -> rejected
D. stale epoch -> rejected
E. duplicate rule_id update -> deterministic versioned update

Run: python3 failure_injection.py [--timeout 6]
Exits nonzero on first unexpected outcome. Prints PASS/FAIL per case.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32

from rulesync_interfaces.srv import (ProposeRule, CommitRule, RemoveRule,
                                      ListRules)
from rulesync_interfaces.msg import RuleSpec


def stamp(sec):
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = int(sec)
    return t


class Inj(Node):
    def __init__(self):
        super().__init__("failure_injection")
        self.p = self.create_client(ProposeRule, "/rulesync/propose_rule")
        self.c = self.create_client(CommitRule, "/rulesync/commit_rule")
        self.r = self.create_client(RemoveRule, "/rulesync/remove_rule")
        self.l = self.create_client(ListRules, "/rulesync/list_rules")
        for name, cli in (("propose", self.p), ("commit", self.c),
                          ("remove", self.r), ("list", self.l)):
            assert cli.wait_for_service(timeout_sec=15.0), name

    def call(self, cli, req, timeout=20.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        assert fut.done(), "service call timed out"
        return fut.result()

    def rule(self, rid, epoch, rtype=0, pts=((0, 0), (1, 0), (1, 1)),
             frame="map", speed=0.0):
        r = RuleSpec()
        r.rule_id = rid
        r.epoch = int(epoch)
        r.rule_type = int(rtype)
        r.severity = 1
        r.frame_id = frame
        r.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                            for x, y in pts]
        r.speed_limit = float(speed)
        r.valid_from = stamp(0)
        r.valid_until = stamp(3600)
        return r

    def epoch_now(self):
        return int(self.call(self.l, ListRules.Request()).epoch)


results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond), str(detail)))
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")


def main():
    ap = argparse.ArgumentParser()
    args = ap.parse_args()
    rclpy.init()
    n = Inj()
    try:
        e0 = n.epoch_now()
        # B: malformed polygon (2 points)
        r = n.rule("bad_poly", e0 + 1, pts=((0, 0), (1, 1)))
        resp = n.call(n.p, ProposeRule.Request(rule=r))
        check("B-malformed-rejected", not resp.accepted, resp.reason)
        # C: invalid frame
        r = n.rule("bad_frame", e0 + 1, frame="nope_frame")
        resp = n.call(n.p, ProposeRule.Request(rule=r))
        check("C-frame-rejected", not resp.accepted, resp.reason)
        # D: stale epoch (propose with epoch <= committed)
        r = n.rule("stale", max(e0, 0))
        resp = n.call(n.p, ProposeRule.Request(rule=r))
        # epoch 0 when committed 0 is stale (<=); any e<=e0 stale
        check("D-stale-rejected", not resp.accepted, resp.reason)
        # E: propose + commit v1, then deterministic update v2 same id
        r1 = n.rule("dup_rule", e0 + 1)
        assert n.call(n.p, ProposeRule.Request(rule=r1)).accepted
        assert n.call(n.c, CommitRule.Request(epoch=e0 + 1)).committed
        check("E-v1-active", n.epoch_now() == e0 + 1)
        r2 = n.rule("dup_rule", e0 + 2,
                    pts=((0, 0), (2, 0), (2, 2)))
        assert n.call(n.p, ProposeRule.Request(rule=r2)).accepted
        assert n.call(n.c, CommitRule.Request(epoch=e0 + 2)).committed
        lst = n.call(n.l, ListRules.Request())
        ids = [x.rule_id for x in lst.active]
        check("E-v2-replaces", ids.count("dup_rule") == 1 and
              n.epoch_now() == e0 + 2, ids)
        # A: silent participant -> TIMEOUT -> old epoch stays.
        import subprocess as _sp
        e_before = n.epoch_now()
        _sp.run(["ros2", "param", "set", "/rulesync_manager", "participants",
                 '["nav2_bridge", "ghost_bridge"]'], check=True,
                capture_output=True)
        _sp.run(["ros2", "param", "set", "/rulesync_manager",
                 "prepare_timeout_s", "3.0"], check=True, capture_output=True)
        try:
            ra = n.rule("ghost_test", e_before + 1)
            assert n.call(n.p, ProposeRule.Request(rule=ra)).accepted
            rc = n.call(n.c, CommitRule.Request(epoch=e_before + 1),
                        timeout=30.0)
            check("A-timeout-aborts", not rc.committed, rc.reason)
            check("A-old-epoch-survives", n.epoch_now() == e_before)
        finally:
            _sp.run(["ros2", "param", "set", "/rulesync_manager",
                     "participants", '["nav2_bridge"]'], check=True,
                    capture_output=True)
            _sp.run(["ros2", "param", "set", "/rulesync_manager",
                     "prepare_timeout_s", "15.0"], check=True,
                    capture_output=True)
    finally:
        n.destroy_node()
        rclpy.shutdown()
    failed = [name for name, ok, _ in results if not ok]
    print(f"injection: {len(results) - len(failed)}/{len(results)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
