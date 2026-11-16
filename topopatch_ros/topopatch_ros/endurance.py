"""1-hour endurance: repeated rule injection/removal under load.

Loop (until --minutes elapsed): commit 1 KEEP_OUT (epoch N, rect
alternating two positions) -> wait topo status (expect repair or
gated fallback) -> remove_rule (epoch N+1, M2 RemoveRule service) ->
verify ListRules empty. Each iteration samples: RSS of topo/manager/
bridge/planner processes, node count, committed epoch monotonicity,
statuses-received vs commits (missed verifier calls), iteration wall
time (deadlock watchdog: flag iterations > --iter-timeout s).

Usage:
  python3 -m topopatch_ros.endurance --csv end.csv --minutes 60
"""
import argparse
import csv
import json
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from std_msgs.msg import String

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import CommitRule, ProposeRule, RemoveRule

try:
    from rulesync_interfaces.srv import ListRules
    HAS_LIST = True
except Exception:
    HAS_LIST = False


def stamp_now():
    from builtin_interfaces.msg import Time
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


RECTS = [
    [(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5)],
    [(1.0, -0.5), (1.0, 0.5), (2.0, 0.5), (2.0, -0.5)],
]


def rss_of(pattern):
    try:
        out = subprocess.run(
            ["pgrep", "-f", pattern], capture_output=True, text=True,
            timeout=5.0)
        total = 0
        for pid in out.stdout.split():
            try:
                with open(f"/proc/{pid.strip()}/status") as f:
                    for line in f:
                        if line.startswith("VmRSS"):
                            total += int(line.split()[1])
                            break
            except Exception:
                pass
        return total
    except Exception:
        return -1


class Endurance(Node):
    def __init__(self, planner_id="GridBased", start=(-2.0, -0.5)):
        super().__init__("paper_endurance")
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        self.planner_id = planner_id
        self.start_xy = start
        self.ac = ActionClient(self, ComputePathToPose,
                               "compute_path_to_pose")
        self.cli_propose = self.create_client(ProposeRule,
                                              "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule,
                                             "/rulesync/commit_rule")
        self.cli_remove = self.create_client(RemoveRule,
                                             "/rulesync/remove_rule")
        self.statuses = []
        self.create_subscription(String, "/rulepatch/status",
                                 self._on_status, 10)
        from geometry_msgs.msg import PoseStamped as _PS
        self.goal_pub = self.create_publisher(_PS, "/goal_pose", 10)

    def _on_status(self, msg):
        try:
            self.statuses.append(json.loads(msg.data))
        except Exception:
            pass

    def ping_plan(self, goal, timeout=30.0):
        """Publish goal_pose + request a plan so topo has plan+goal.
        Returns True when a usable plan exists."""
        from geometry_msgs.msg import PoseStamped as _PS2

        def ps(p):
            m = _PS2()
            m.header.frame_id = "map"
            m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
            m.pose.orientation.w = 1.0
            return m

        self.goal_pub.publish(ps(goal))
        from nav2_msgs.action import ComputePathToPose as _CPP
        g = _CPP.Goal()
        g.start, g.goal = ps(self.start_xy), ps(goal)
        g.planner_id = self.planner_id
        g.use_start = True
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        gh = f.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        r = rf.result()
        return bool(r is not None and len(r.result.path.poses) >= 4)

    def propose_commit(self, rid, epoch, poly, timeout=60.0):
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
        fut = self.cli_propose.call_async(ProposeRule.Request(rule=rule))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        resp = fut.result()
        if resp is None or not resp.accepted:
            return False
        fut2 = self.cli_commit.call_async(
            CommitRule.Request(epoch=int(epoch)))
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=timeout)
        resp2 = fut2.result()
        return bool(resp2 and resp2.committed)

    def remove(self, rule_id, epoch, timeout=60.0):
        from rulesync_interfaces.srv import RemoveRule as _RR
        fut = self.cli_remove.call_async(
            _RR.Request(rule_id=rule_id, epoch=int(epoch)))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        resp = fut.result()
        return bool(resp and getattr(resp, "ok", False))

    def list_active(self, timeout=10.0):
        if not HAS_LIST:
            return None
        cli = self.create_client(ListRules, "/rulesync/list_rules")
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(ListRules.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            return None
        return (int(res.epoch), len(res.active))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--minutes", type=float, default=60.0)
    ap.add_argument("--epoch0", type=int, default=501)
    ap.add_argument("--iter-timeout", type=float, default=120.0)
    ap.add_argument("--planner", default="GridBased")
    ap.add_argument("--start", default="-2.0,-0.5")
    ap.add_argument("--goal", default="2.0,0.5")
    a = ap.parse_args()
    a.start = tuple(float(v) for v in a.start.split(","))
    a.goal = tuple(float(v) for v in a.goal.split(","))
    rclpy.init()
    node = Endurance(planner_id=a.planner, start=a.start)
    for name, cli in (("propose", node.cli_propose),
                      ("commit", node.cli_commit),
                      ("remove", node.cli_remove)):
        assert cli.wait_for_service(timeout_sec=30.0), name
    t_end = time.time() + a.minutes * 60.0
    rows = []
    it = 0
    last_epoch = 0
    anomalies = []
    while time.time() < t_end:
        t_iter = time.time()
        ep = a.epoch0 + 2 * it
        row = {"iter": it, "epoch": ep}
        n_status_before = len(node.statuses)
        try:
            row["plan_ok"] = bool(node.ping_plan(a.goal))
            okc = row["plan_ok"] and node.propose_commit(
                f"end-{ep}", ep, RECTS[it % 2])
            row["commit_ok"] = bool(okc)
            # Spin (not sleep!) so subscription callbacks (statuses)
            # are actually processed during the collection window.
            t_collect = time.time()
            while time.time() - t_collect < 6.0:
                rclpy.spin_once(node, timeout_sec=0.5)
            new_status = [s for s in node.statuses[n_status_before:]
                          if f"epoch={ep}" in str(s.get("note", ""))]
            row["statuses"] = len(new_status)
            if okc and not new_status:
                anomalies.append(f"iter{it}: missed verifier call")
                row["missed_verifier"] = True
            else:
                row["missed_verifier"] = False
            okr = node.remove(f"end-{ep}", ep + 1)
            row["remove_ok"] = bool(okr)
            la = node.list_active()
            row["list_epoch"] = la[0] if la else None
            row["list_active"] = la[1] if la else None
            if la and la[1] != 0:
                anomalies.append(f"iter{it}: stale rules active={la[1]}")
            if la and la[0] < last_epoch:
                anomalies.append(f"iter{it}: epoch went back "
                                 f"{last_epoch}->{la[0]}")
            last_epoch = la[0] if la else last_epoch
            for pat in ("topo_inference", "rulesync_manager",
                        "nav2_bridge", "planner_server"):
                row[f"rss_{pat}"] = rss_of(pat)
            try:
                out = subprocess.run(
                    ["ros2", "node", "list"], capture_output=True,
                    text=True, timeout=20.0)
                row["node_count"] = len(
                    [l for l in out.stdout.splitlines() if l.strip()])
            except Exception:
                row["node_count"] = None
        except Exception as e:
            import traceback
            traceback.print_exc()
            row["error"] = str(e)[:200]
            anomalies.append(f"iter{it}: exception {e}"[:120])
        row["iter_s"] = round(time.time() - t_iter, 1)
        if row["iter_s"] > a.iter_timeout:
            anomalies.append(f"iter{it}: deadlock-suspect "
                             f"{row['iter_s']}s")
        rows.append(row)
        print(f"[endurance] iter={it} commit={row.get('commit_ok')} "
              f"statuses={row.get('statuses')} remove={row.get('remove_ok')} "
              f"rss_topo={row.get('rss_topo_inference')} "
              f"t={row['iter_s']}s", flush=True)
        it += 1
    with open(a.csv, "w", newline="") as f:
        import csv as _csv
        keys = sorted({k for r in rows for k in r.keys()})
        w = _csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[endurance] iters={it} anomalies={len(anomalies)}", flush=True)
    for an in anomalies[:20]:
        print(f"[endurance] ANOMALY: {an}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
