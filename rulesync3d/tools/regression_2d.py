# Minimal 2D regression (§14): validator/gate paths with live services.
# Covered elsewhere by matrices (do NOT duplicate): normal commit flows,
# missing-ACK abort (C2 rows), restarts (restart rows), latency (CSV).
# This covers ONLY: stale-epoch reject, unknown-id remove, degenerate
# polygon, bad frame, bad speed-limit — each must fail CLOSED with a
# naming reason, plus one ADD/REMOVE sanity that state is servable.
# Usage: python3 regression_2d.py --epoch E --out rows.jsonl
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import (ProposeRule, CommitRule, RemoveRule,
                                     ListRules)
from geometry_msgs.msg import Point32
from builtin_interfaces.msg import Time


def stamp():
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


def mk(rid, epoch, pts, rtype=0, speed=0.0, frame="map"):
    r = RuleSpec()
    r.rule_id = rid
    r.epoch = int(epoch)
    r.rule_type = int(rtype)
    r.severity = 1
    r.frame_id = frame
    r.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                        for x, y in pts]
    r.speed_limit = float(speed)
    r.valid_from = stamp()
    e = stamp()
    e.sec += 3600
    r.valid_until = e
    return r


P0 = [(10.0, 10.0), (12.0, 10.0), (12.0, 12.0), (10.0, 12.0)]


class R(Node):
    def __init__(self):
        super().__init__("regression_2d")
        self.cp = self.create_client(ProposeRule, "/rulesync/propose_rule")
        self.cc = self.create_client(CommitRule, "/rulesync/commit_rule")
        self.cr = self.create_client(RemoveRule, "/rulesync/remove_rule")
        self.cl = self.create_client(ListRules, "/rulesync/list_rules")
        for c in (self.cp, self.cc, self.cr, self.cl):
            assert c.wait_for_service(timeout_sec=30.0)

    def call(self, cli, req, timeout=30.0):
        f = cli.call_async(req)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=1.0)
        return f.result() if f.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epoch", type=int, default=500000)
    ap.add_argument("--out", default="/tmp/regression_2d.jsonl")
    a = ap.parse_args()
    rclpy.init()
    n = R()
    E = int(a.epoch)
    rows = []

    def rec(check, passed, evidence):
        rows.append({"check": check, "passed": bool(passed),
                     "evidence": str(evidence)})
        print(f"[regress] {check}: {'PASS' if passed else 'FAIL'} "
              f"{evidence}", flush=True)

    # 0. sanity ADD + REMOVE (state servable)
    r = n.call(n.cp, ProposeRule.Request(
        rule=mk("rg-r0", E, P0)))
    ok_p = bool(r is not None and r.accepted)
    r = n.call(n.cc, CommitRule.Request(epoch=E)) if ok_p else None
    ok_c = bool(r is not None and r.committed)
    rec("sanity-add", ok_p and ok_c, f"propose={ok_p} commit={ok_c}")
    r = n.call(n.cr, RemoveRule.Request(rule_id="rg-r0", epoch=E + 10))
    rec("sanity-remove", bool(r is not None and r.ok),
        f"remove={getattr(r, 'ok', None)}:{getattr(r, 'reason', 'noresp')}")
    committed = E + 10
    # 1. stale epoch propose (must reject)
    r = n.call(n.cp, ProposeRule.Request(
        rule=mk("rg-stale", committed, P0)))
    rec("stale-epoch-reject",
        bool(r is not None and not r.accepted and
             "stale" in str(getattr(r, "reason", "")).lower()),
        f"accepted={getattr(r, 'accepted', None)} "
        f"reason={getattr(r, 'reason', 'noresp')}")
    # 2. unknown rule_id remove (must reject)
    r = n.call(n.cr, RemoveRule.Request(rule_id="no-such-rule",
                                        epoch=committed + 10))
    rec("unknown-id-reject", bool(r is not None and not r.ok),
        f"ok={getattr(r, 'ok', None)} "
        f"reason={getattr(r, 'reason', 'noresp')}")
    # 3. degenerate polygon (2 points; must reject)
    r = n.call(n.cp, ProposeRule.Request(
        rule=mk("rg-degen", committed + 10, [(0.0, 0.0), (1.0, 1.0)])))
    rec("degenerate-polygon-reject",
        bool(r is not None and not r.accepted),
        f"accepted={getattr(r, 'accepted', None)} "
        f"reason={getattr(r, 'reason', 'noresp')}")
    # 4. bad frame (must reject)
    r = n.call(n.cp, ProposeRule.Request(
        rule=mk("rg-frame", committed + 10, P0, frame="narnia")))
    rec("bad-frame-reject",
        bool(r is not None and not r.accepted),
        f"accepted={getattr(r, 'accepted', None)} "
        f"reason={getattr(r, 'reason', 'noresp')}")
    # 5. SPEED_LIMIT with non-positive limit (must reject)
    r = n.call(n.cp, ProposeRule.Request(
        rule=mk("rg-speed", committed + 10, P0, rtype=1, speed=0.0)))
    rec("bad-speed-reject",
        bool(r is not None and not r.accepted),
        f"accepted={getattr(r, 'accepted', None)} "
        f"reason={getattr(r, 'reason', 'noresp')}")
    with open(a.out, "w") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")
    print(f"[regress] {sum(1 for r in rows if r['passed'])}/{len(rows)} "
          f"passed", flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
