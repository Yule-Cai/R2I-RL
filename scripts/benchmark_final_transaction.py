#!/usr/bin/env python3
"""Benchmark the final durable 2-D RuleSync transaction end to end.

The runner assumes a manager and nav2 bridge are already running.  Every row
performs PROPOSE -> PREPARE/COMMIT/WAL/APPLY/ACK -> ListRules verification on a
single versioned KEEP_OUT rule.  Warm-up rows are retained in the raw CSV but
excluded from the summary statistics.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32
from rclpy.node import Node

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import CommitRule, ListRules, ProposeRule


def percentile(values, q):
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def describe(values):
    return {
        "n": len(values),
        "mean_ms": statistics.fmean(values),
        "median_ms": statistics.median(values),
        "p90_ms": percentile(values, 0.90),
        "p95_ms": percentile(values, 0.95),
        "p99_ms": percentile(values, 0.99),
        "min_ms": min(values),
        "max_ms": max(values),
    }


class TransactionBenchmark(Node):
    def __init__(self, service_timeout):
        super().__init__("rulesync_final_transaction_benchmark")
        self.service_timeout = float(service_timeout)
        self.propose = self.create_client(
            ProposeRule, "/rulesync/propose_rule")
        self.commit = self.create_client(
            CommitRule, "/rulesync/commit_rule")
        self.list_rules = self.create_client(
            ListRules, "/rulesync/list_rules")
        for name, client in (
                ("propose_rule", self.propose),
                ("commit_rule", self.commit),
                ("list_rules", self.list_rules)):
            if not client.wait_for_service(timeout_sec=self.service_timeout):
                raise RuntimeError(f"service /rulesync/{name} unavailable")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.service_timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("service call timed out")
        return future.result()

    def current_epoch(self):
        return int(self.call(self.list_rules, ListRules.Request()).epoch)

    @staticmethod
    def rule(epoch, offset):
        rule = RuleSpec()
        rule.rule_id = "txn-benchmark-keepout"
        rule.epoch = int(epoch)
        rule.rule_type = RuleSpec.KEEP_OUT
        rule.severity = RuleSpec.ENFORCED
        rule.frame_id = "map"
        x0 = 5.0 + float(offset)
        rule.polygon.points = [
            Point32(x=x0, y=4.0, z=0.0),
            Point32(x=x0, y=5.0, z=0.0),
            Point32(x=x0 + 1.0, y=5.0, z=0.0),
            Point32(x=x0 + 1.0, y=4.0, z=0.0),
        ]
        rule.speed_limit = 0.0
        now = int(time.time())
        rule.valid_from = Time(sec=now, nanosec=0)
        rule.valid_until = Time(sec=now + 3600, nanosec=0)
        return rule

    def one(self, epoch, index, measured):
        rule = self.rule(epoch, 0.01 * (index % 2))
        full_started = time.perf_counter_ns()

        propose_started = time.perf_counter_ns()
        proposed = self.call(
            self.propose, ProposeRule.Request(rule=rule))
        propose_ms = (time.perf_counter_ns() - propose_started) / 1e6
        if not proposed.accepted:
            raise RuntimeError(f"proposal rejected: {proposed.reason}")

        commit_started = time.perf_counter_ns()
        committed = self.call(
            self.commit, CommitRule.Request(epoch=int(epoch)))
        commit_ms = (time.perf_counter_ns() - commit_started) / 1e6
        if not committed.committed:
            raise RuntimeError(f"commit rejected: {committed.reason}")

        verify_started = time.perf_counter_ns()
        state = self.call(self.list_rules, ListRules.Request())
        verify_ms = (time.perf_counter_ns() - verify_started) / 1e6
        full_ms = (time.perf_counter_ns() - full_started) / 1e6
        ids = {str(item.rule_id) for item in state.active}
        converged = (
            int(state.epoch) == int(epoch)
            and not bool(state.degraded)
            and rule.rule_id in ids
            and len(str(state.active_hash)) == 64
        )
        return {
            "index": int(index),
            "phase": "measured" if measured else "warmup",
            "epoch": int(epoch),
            "propose_ms": propose_ms,
            "commit_ms": commit_ms,
            "verify_ms": verify_ms,
            "full_transaction_ms": full_ms,
            "committed": bool(committed.committed),
            "converged": bool(converged),
            "active_hash_bits": len(str(state.active_hash)) * 4,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=20)
    parser.add_argument("--rounds", type=int, default=200)
    parser.add_argument("--service-timeout", type=float, default=20.0)
    args = parser.parse_args()
    if args.warmups < 0 or args.rounds <= 0:
        parser.error("warmups must be >=0 and rounds must be >0")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = TransactionBenchmark(args.service_timeout)
    rows = []
    try:
        base_epoch = node.current_epoch()
        total = args.warmups + args.rounds
        for index in range(total):
            rows.append(node.one(
                base_epoch + index + 1,
                index,
                measured=index >= args.warmups,
            ))
            if (index + 1) % 25 == 0 or index + 1 == total:
                print(json.dumps({
                    "completed": index + 1,
                    "total": total,
                    "last_epoch": rows[-1]["epoch"],
                }), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raw_path = args.output_dir / "final_transaction_latency.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    measured = [row for row in rows if row["phase"] == "measured"]
    summary = {
        "warmups": args.warmups,
        "rounds": args.rounds,
        "successes": sum(
            1 for row in measured if row["committed"] and row["converged"]),
        "hash_bits": sorted({row["active_hash_bits"] for row in measured}),
        "propose": describe([row["propose_ms"] for row in measured]),
        "commit": describe([row["commit_ms"] for row in measured]),
        "verify": describe([row["verify_ms"] for row in measured]),
        "full_transaction": describe(
            [row["full_transaction_ms"] for row in measured]),
    }
    summary_path = args.output_dir / "final_transaction_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "raw": str(raw_path),
        "summary": str(summary_path),
        **summary,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
