"""Transaction latency benchmark: legacy helper vs persistent votes.

Rapid propose/commit/remove loop with per-phase client timestamps,
joined server-side with the manager txn log by epoch. Compares:
  median / p90 / p95 / p99 per phase and end-to-end,
  transaction success, timeout behavior, CPU, RSS.
Mode selected by --mode legacy|persistent (sets the manager param
use_legacy_vote_helper live; the flag is read fresh every round, so
one deployment serves both arms).

Usage:
  python3 -m topopatch_ros.bench_txn --csv out.csv --mode persistent \
      --warmup 20 --n 200 --epoch0 1001
"""
import argparse
import csv
import json
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import (CommitRule, ProposeRule, RemoveRule,
                                     ListRules)

try:
    from std_msgs.msg import String as _S
except Exception:
    _S = None


def stamp_now():
    from builtin_interfaces.msg import Time
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


RECT = [(-0.5, -1.0), (-0.5, 0.0), (0.5, 0.0), (0.5, -1.0)]


class TxnBench(Node):
    def __init__(self):
        super().__init__("txn_bench")
        self.cli_propose = self.create_client(ProposeRule,
                                              "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule,
                                             "/rulesync/commit_rule")
        self.cli_remove = self.create_client(RemoveRule,
                                             "/rulesync/remove_rule")
        self.cli_list = self.create_client(ListRules,
                                           "/rulesync/list_rules")
        self.cpu = {}
        self._stop = False

    def mk_rule(self, rid, epoch, poly):
        rule = RuleSpec()
        rule.rule_id = rid
        rule.epoch = int(epoch)
        rule.rule_type = 0
        rule.severity = 1
        rule.frame_id = "map"
        rule.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                               for x, y in poly]
        rule.valid_from = stamp_now()
        end = stamp_now()
        end.sec += 3600
        rule.valid_until = end
        return rule

    def sample_proc(self, pattern, out):
        try:
            ps = subprocess.run(["pgrep", "-f", pattern], capture_output=True,
                                text=True, timeout=5.0)
            for pid in ps.stdout.split():
                try:
                    with open(f"/proc/{pid.strip()}/stat") as f:
                        st = f.read().split()
                    ut, stt = int(st[13]), int(st[14])
                    with open(f"/proc/{pid.strip()}/status") as f2:
                        rss = 0
                        for line in f2:
                            if line.startswith("VmRSS"):
                                rss = int(line.split()[1])
                                break
                    out[pattern] = {"utime": ut, "stime": stt, "rss": rss}
                except Exception:
                    pass
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--mode", required=True,
                    choices=["legacy", "persistent"])
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--epoch0", type=int, default=1001)
    ap.add_argument("--goal", default="2.0,0.5")
    a = ap.parse_args()
    gx, gy = (float(v) for v in a.goal.split(","))
    rclpy.init()
    node = TxnBench()
    # set manager vote path for this arm (read fresh every round)
    import os as _os
    import subprocess as _sp
    _env = dict(_os.environ)
    _sp.run(["ros2", "param", "set", "/rulesync_manager",
             "use_legacy_vote_helper",
             "true" if a.mode == "legacy" else "false"],
            capture_output=True, timeout=30.0, env=_env)
    for name, cli in (("propose", node.cli_propose),
                      ("commit", node.cli_commit),
                      ("remove", node.cli_remove),
                      ("list", node.cli_list)):
        assert cli.wait_for_service(timeout_sec=30.0), name
    # publish goal for bridge feasibility (latched repeatedly)
    from geometry_msgs.msg import PoseStamped as _PS
    gp = _PS()
    gp.header.frame_id = "map"
    gp.pose.position.x, gp.pose.position.y = gx, gy
    gp.pose.orientation.w = 1.0
    gpub = node.create_publisher(_PS, "/goal_pose", 10)
    for _ in range(6):
        gpub.publish(gp)
        time.sleep(0.3)
    rows = []
    total = a.warmup + a.n
    for i in range(total):
        ep = a.epoch0 + 10 * i
        phase = "warmup" if i < a.warmup else "steady"
        row = {"mode": a.mode, "phase": phase, "epoch": ep, "iter": i}
        snap0 = {}
        node.sample_proc("[r]ulesync_manager", snap0)
        node.sample_proc("[n]av2_bridge", snap0)
        t0 = time.monotonic()
        try:
            from rulesync_interfaces.srv import ProposeRule as _PR
            f = node.cli_propose.call_async(_PR.Request(
                rule=node.mk_rule(f"txn-{ep}", ep, RECT)))
            rclpy.spin_until_future_complete(node, f, timeout_sec=30.0)
            r = f.result()
            t1 = time.monotonic()
            row["propose_ms"] = round((t1 - t0) * 1000.0, 1)
            row["propose_ok"] = bool(r and r.accepted)
            f2 = node.cli_commit.call_async(
                CommitRule.Request(epoch=int(ep)))
            rclpy.spin_until_future_complete(node, f2, timeout_sec=60.0)
            r2 = f2.result()
            t2 = time.monotonic()
            row["commit_ms"] = round((t2 - t1) * 1000.0, 1)
            row["commit_ok"] = bool(r2 and r2.committed)
            f3 = node.cli_remove.call_async(
                RemoveRule.Request(rule_id=f"txn-{ep}",
                                   epoch=int(ep + 1)))
            rclpy.spin_until_future_complete(node, f3, timeout_sec=60.0)
            r3 = f3.result()
            t3 = time.monotonic()
            row["remove_ms"] = round((t3 - t2) * 1000.0, 1)
            row["remove_ok"] = bool(r3 and getattr(r3, "ok", False))
            row["txn_ms"] = round((t3 - t0) * 1000.0, 1)
            row["ok"] = bool(row["propose_ok"] and row["commit_ok"]
                             and row["remove_ok"])
        except Exception as e:
            row["ok"] = False
            row["error"] = str(e)[:150]
        snap1 = {}
        node.sample_proc("[r]ulesync_manager", snap1)
        node.sample_proc("[n]av2_bridge", snap1)
        import re as _re
        def _nm(k):
            return _re.sub(r"^\[(.)\]", r"\1", k)
        for k in ("[r]ulesync_manager", "[n]av2_bridge"):
            nk = _nm(k)
            if k in snap0 and k in snap1:
                # CPU% over interval approximated at analysis (jiffies)
                row[f"cpu_{nk}"] = (
                    snap1[k]["utime"] + snap1[k]["stime"]
                    - snap0[k]["utime"] - snap0[k]["stime"])
                row[f"rss_{nk}"] = snap1[k]["rss"]
        rows.append(row)
        if (i + 1) % 20 == 0:
            oks = sum(1 for r in rows if r.get("ok"))
            print(f"[bench_txn] {a.mode} {i + 1}/{total} ok={oks}",
                  flush=True)
    with open(a.csv, "w", newline="") as fh:
        keys = sorted({k for r in rows for k in r.keys()})
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[bench_txn] wrote {a.csv}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
