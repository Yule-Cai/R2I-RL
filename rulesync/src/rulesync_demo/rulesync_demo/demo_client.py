"""RuleSync demo driver: goal -> keepout inject -> replan -> speed limit -> remove.

Usage (after workspace sourced + sim running):
  python3 demo_client.py --goal-x 4.0 --goal-y 1.0 [--csv /tmp/rulesync_latency.csv]

Steps: send NavigateToPose goal; propose+commit KEEP_OUT (epoch 1) crossing
the path; cancel+resend same goal to force replan on the new costmap;
propose+commit SPEED_LIMIT (epoch 2); observe; remove rules (epochs 3,4);
final goal. Latency CSV records every timestamp.
"""
import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point32, PoseStamped
from nav2_msgs.action import NavigateToPose

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import ProposeRule, CommitRule, RemoveRule


def now_s():
    return time.time()


def stamp_now():
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = int(time.time())
    return t


class Demo(Node):
    def __init__(self, csv_path):
        super().__init__("rulesync_demo_client")
        self.csv_path = csv_path
        self.rows = []
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        from geometry_msgs.msg import PoseStamped as _PS
        self.goal_pub = self.create_publisher(_PS, "/goal_pose", 10)
        self.cli_propose = self.create_client(ProposeRule, "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule, "/rulesync/commit_rule")
        self.cli_remove = self.create_client(RemoveRule, "/rulesync/remove_rule")
        for name, cli in (("propose", self.cli_propose), ("commit", self.cli_commit),
                          ("remove", self.cli_remove)):
            if not cli.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"service /rulesync/{name} unavailable")
        # Topic-transport 2PC needs no vote-service wait; the manager
        # re-emits staged sets until votes arrive or timeout hits.
        if not self.nav.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("NavigateToPose action server unavailable")

    def log(self, **kw):
        kw.setdefault("t_wall", now_s())
        self.rows.append(kw)

    def send_goal(self, x, y, note=""):
        from geometry_msgs.msg import PoseStamped as _PS
        gp = _PS()
        gp.header.frame_id = "map"
        gp.pose.position.x = float(x)
        gp.pose.position.y = float(y)
        gp.pose.orientation.w = 1.0
        self.goal_pub.publish(gp)  # feeds bridge feasibility + Nav2 input
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        t0 = now_s()
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        self.log(event="goal_sent", note=note, t_goal_sent=t0,
                 accepted=bool(handle.accepted))
        return handle

    def wait_arrival(self, gx, gy, timeout=150.0, tol=0.5):
        # Odom-proximity arrival (the bridge may have preempted our original
        # goal handle with its auto-replan; action results are unreliable).
        from nav_msgs.msg import Odometry as _Od
        pos = {}
        sub = self.create_subscription(
            _Od, "/odom", lambda m: pos.__setitem__(
                "xy", (float(m.pose.pose.position.x),
                       float(m.pose.pose.position.y))), 10)
        t0 = now_s()
        ok = False
        while now_s() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if "xy" in pos and abs(pos["xy"][0] - gx) < tol and \
                    abs(pos["xy"][1] - gy) < tol:
                ok = True
                break
        self.destroy_subscription(sub)
        self.log(event="arrival", arrived=ok)
        return ok

    def wait_goal(self, handle, timeout=120.0):
        fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        status = int(res.status) if res is not None else -1
        self.log(event="goal_done", status=status)
        return status

    def cancel_goal(self, handle):
        fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        self.log(event="goal_cancelled")

    def _rule(self, rule_id, epoch, rule_type, pts, speed=0.0):
        r = RuleSpec()
        r.rule_id = rule_id
        r.epoch = int(epoch)
        r.rule_type = int(rule_type)
        r.severity = 1
        r.frame_id = "map"
        r.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in pts]
        r.speed_limit = float(speed)
        r.valid_from = stamp_now()
        end = stamp_now()
        end.sec += 3600
        r.valid_until = end
        return r

    def propose_commit(self, rule, timeout=90.0):
        from rclpy.task import Future  # noqa (kept import local by style)
        t_recv = now_s()
        rq = ProposeRule.Request(rule=rule)
        fut = self.cli_propose.call_async(rq)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        resp = fut.result()
        t_prep0 = now_s()
        self.log(event="proposed", rule_id=rule.rule_id, t_received=t_recv,
                 accepted=bool(resp.accepted), reason=str(resp.reason))
        if not resp.accepted:
            return False
        rq2 = CommitRule.Request(epoch=int(rule.epoch))
        fut2 = self.cli_commit.call_async(rq2)
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=timeout)
        resp2 = fut2.result()
        t_commit = now_s()
        self.log(event="committed", rule_id=rule.rule_id, t_commit=t_commit,
                 committed=bool(resp2.committed), reason=str(resp2.reason),
                 rule_activation_latency=t_commit - t_recv,
                 prepare_latency=t_commit - t_prep0)
        return bool(resp2.committed)

    def current_epoch(self, timeout=20.0):
        from rulesync_interfaces.srv import ListRules as _LR
        cli = self.create_client(_LR, "/rulesync/list_rules")
        cli.wait_for_service(timeout_sec=timeout)
        fut = cli.call_async(_LR.Request())
        import time as _t
        from rclpy import spin_until_future_complete as _spin
        _spin(self, fut, timeout_sec=timeout)
        return int(fut.result().epoch)

    def remove(self, rule_id, epoch, timeout=90.0):
        rq = RemoveRule.Request(rule_id=rule_id, epoch=int(epoch))
        fut = self.cli_remove.call_async(rq)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = bool(fut.result().ok)
        self.log(event="removed", rule_id=rule_id, ok=ok)
        return ok

    def save_csv(self):
        keys = sorted({k for row in self.rows for k in row})
        with open(self.csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)
        print(f"[demo] latency CSV: {self.csv_path} ({len(self.rows)} rows)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal-x", type=float, default=4.0)
    ap.add_argument("--goal-y", type=float, default=1.0)
    ap.add_argument("--csv", default="/tmp/rulesync_latency.csv")
    ap.add_argument("--keepout", type=float, nargs=4, metavar=("X1", "Y1", "X2", "Y2"),
                    default=[2.0, -0.6, 3.0, 0.6])
    ap.add_argument("--speed-zone", type=float, nargs=4, dest="speed_zone",
                    metavar=("X1", "Y1", "X2", "Y2"),
                    default=[0.5, -1.5, 4.5, 1.5])
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--gate-x", type=float, default=1.0,
                    help="propose keepout only after odom x passes this")
    args = ap.parse_args()
    rclpy.init()
    demo = Demo(args.csv)
    try:
        e0 = demo.current_epoch()
        EK = e0 + 1
        ES, ER1, ER2 = e0 + 2, e0 + 3, e0 + 4
        print(f"[demo] starting at epoch base {e0}", flush=True)
        h = demo.send_goal(args.goal_x, args.goal_y, note="initial")
        # Gate keepout proposal on odom: propose only while the robot is
        # verifiably WEST of the zone (else feasibility rightly rejects).
        # Falls back to a timed propose so the demo never hangs.
        from nav_msgs.msg import Odometry as _Od
        gate = {"x": None}
        sub = demo.create_subscription(
            _Od, "/odom", lambda m: gate.__setitem__(
                "x", float(m.pose.pose.position.x)), 10)
        t_gate = time.time()
        while (gate["x"] is None or gate["x"] > args.gate_x) and \
                time.time() - t_gate < 90.0:
            rclpy.spin_once(demo, timeout_sec=0.5)
        demo.log(event="gate_passed", robot_x=gate["x"])
        demo.destroy_subscription(sub)
        time.sleep(1.0)
        # KEEP_OUT rect from CLI (must cross the live path for a
        # visible detour); default suits start (0,1) -> goal (4,1).
        kx1, ky1, kx2, ky2 = args.keepout
        keepout = [(kx1, ky1), (kx1, ky2), (kx2, ky2), (kx2, ky1)]
        assert demo.propose_commit(demo._rule("demo_keepout", EK, 0, keepout))
        # Bridge auto-replans on invalidation (no manual cancel+resend):
        # wait for a global plan stamped after the commit.
        from nav_msgs.msg import Path as _Path
        seen = {}
        sub = demo.create_subscription(
            _Path, "/plan", lambda m: seen.__setitem__("t", float(
                m.header.stamp.sec) + float(m.header.stamp.nanosec) / 1e9), 10)
        t_commit_done = now_s()
        t_new = None
        while now_s() - t_commit_done < 40.0:
            rclpy.spin_once(demo, timeout_sec=1.0)
            if seen.get("t", 0.0) > t_commit_done - 5.0:
                t_new = now_s()
                break
        demo.destroy_subscription(sub)
        demo.log(event="replan_observed", t_new_plan=t_new,
                 replan_latency=(t_new - t_commit_done) if t_new else None)
        h2 = None
        time.sleep(8.0)
        sx1, sy1, sx2, sy2 = args.speed_zone
        speed_zone = [(sx1, sy1), (sx1, sy2), (sx2, sy2), (sx2, sy1)]
        assert demo.propose_commit(demo._rule("demo_speed", ES, 1, speed_zone,
                                              speed=args.speed))
        time.sleep(10.0)  # observe reduced velocity (odom log + bag)
        assert demo.remove("demo_speed", ER1)
        time.sleep(4.0)
        assert demo.remove("demo_keepout", ER2)
        st = demo.wait_arrival(args.goal_x, args.goal_y, timeout=150.0)
        print(f"[demo] final arrival: {st}")
    finally:
        demo.save_csv()
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
