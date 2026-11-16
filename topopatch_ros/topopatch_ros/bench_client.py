"""Headless planner-matrix client (no physics, no odom).

For each case: plan with planner_id P -> commit 1-3 KEEP_OUT rects
across the plan -> wait topo status -> gated driver-side fallback
replan (bounded retries) -> record. No execution (execution metrics
come from physics runs). Rule rects auto-placed like the Gazebo driver.

Usage:
  python3 -m topopatch_ros.bench_client --csv out.csv --planner Smac2D \
      --goals goals.json --cases cases.json --n-rules 2 --pgm-note mapkey
goals.json: [[x,y],...]; cases.json (optional prescored): [{goal, rule}].
"""
import argparse
import csv
import json
import math
import time

import rclpy
from geometry_msgs.msg import Point32
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from rulesync_interfaces.msg import RuleSpec
from rulesync_interfaces.srv import CommitRule, ProposeRule

def _densify(path, n=32):
    """Linear densification for sparse planner outputs (ThetaStar
    straights). A 2-point straight plan is valid, just sparse."""
    import numpy as _np
    p = _np.asarray(path, dtype=_np.float64)
    if len(p) >= n:
        return [tuple(q) for q in p.tolist()]
    seg = _np.hypot(p[1:, 0] - p[:-1, 0], p[1:, 1] - p[:-1, 1])
    cum = _np.concatenate([[0.0], _np.cumsum(seg)])
    total = cum[-1]
    if total < 1e-9:
        return [tuple(p[0])] * n
    us = _np.linspace(0, total, n)
    xs = _np.interp(us, cum, p[:, 0])
    ys = _np.interp(us, cum, p[:, 1])
    return list(zip(xs.tolist(), ys.tolist()))

RECT_HALF = 0.75


def pip(xy, poly):
    x, y = xy
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
            if x < xin:
                inside = not inside
    return inside


def stamp_now():
    from builtin_interfaces.msg import Time
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


class Bench(Node):
    def __init__(self, planner_id, map_tag=""):
        super().__init__("paper_bench_client")
        self.planner_id = planner_id
        self.map_tag = map_tag
        self.ac = ActionClient(self, ComputePathToPose,
                               "compute_path_to_pose")
        self.cli_propose = self.create_client(ProposeRule,
                                              "/rulesync/propose_rule")
        self.cli_commit = self.create_client(CommitRule,
                                             "/rulesync/commit_rule")
        from geometry_msgs.msg import PoseStamped as _PSB
        self.goal_pub = self.create_publisher(_PSB, "/goal_pose", 10)
        self.statuses = []
        self.create_subscription(String, "/rulepatch/status",
                                 self._on_status, 10)
        from topopatch_ros.txn_trace import MaskWatcher
        from rulesync_interfaces.srv import ListRules as _LR
        self.mask_watcher = MaskWatcher(self)
        self.cli_list = self.create_client(_LR, "/rulesync/list_rules")
        # DDS lesson (narrow-B campaign): create ALL clients/subscriptions
        # up front. A client created minutes after startup intermittently
        # never discovers its service (remove_rule service unavailable ->
        # leaked rules -> topo too-many-rules -> no-status cascade).
        from rulesync_interfaces.srv import RemoveRule as _RRinit
        self._cli_rm = self.create_client(
            _RRinit, "/rulesync/remove_rule")
        self._map = None
        from nav_msgs.msg import OccupancyGrid as _OG
        from rclpy.qos import QoSProfile as _QP
        from rclpy.qos import DurabilityPolicy as _DP
        self.create_subscription(
            _OG, "/map", self._on_map,
            _QP(depth=10, durability=_DP.TRANSIENT_LOCAL))

    def _on_map(self, msg):
        self._map = msg

    def _on_status(self, msg):
        try:
            self.statuses.append(json.loads(msg.data))
        except Exception:
            pass

    def pose(self, p):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        m.pose.orientation.w = 1.0
        return m

    def plan(self, s, g, timeout=30.0):
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal = self.pose(s), self.pose(g)
        goal.planner_id = self.planner_id
        goal.use_start = True
        t0 = time.time()
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None, round(time.time() - t0, 3), "rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        r = rf.result()
        dt = round(time.time() - t0, 3)
        if r is None:
            return None, dt, "timeout"
        path = [(p.pose.position.x, p.pose.position.y)
                for p in r.result.path.poses]
        if len(path) < 2:
            return None, dt, "too-short"
        return _densify(path), dt, "ok"

    def propose_commit(self, rules, epoch, timeout=90.0):
        from topopatch_ros.txn_trace import monotonic_ms
        t0 = monotonic_ms()
        t_prop = 0.0
        for i, poly in enumerate(rules):
            rule = RuleSpec()
            rule.rule_id = f"paper-{epoch}-r{i}"
            rule.epoch = int(epoch)
            rule.rule_type = 0
            rule.severity = 1
            rule.frame_id = "map"
            rule.polygon.points = [
                Point32(x=float(x), y=float(y), z=0.0) for x, y in poly]
            rule.valid_from = stamp_now()
            end = stamp_now()
            end.sec += 3600
            rule.valid_until = end
            fut = self.cli_propose.call_async(
                ProposeRule.Request(rule=rule))
            rclpy.spin_until_future_complete(self, fut,
                                             timeout_sec=timeout)
            resp = fut.result()
            if resp is None or not resp.accepted:
                return False, round(monotonic_ms() - t0, 1), 0.0, \
                    "propose-failed"
        t_prop = round(monotonic_ms() - t0, 1)
        fut2 = self.cli_commit.call_async(
            CommitRule.Request(epoch=int(epoch)))
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=timeout)
        resp2 = fut2.result()
        ok = bool(resp2 and resp2.committed)
        t_commit = round(monotonic_ms() - t0 - t_prop, 1)
        return ok, t_prop, t_commit, ("ok" if ok else "commit-failed")

    def wait_status(self, epoch, timeout=40.0):
        t0 = time.time()
        want = f"epoch={epoch}"
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            for s in self.statuses:
                if want in str(s.get("note", "")):
                    return s
        return None

    def _remove_placed(self, placed_ids, epoch):
        # Best-effort removal of this case's rules so cases stay
        # independent (manager merges commits; without removal rules
        # accumulate and topo refuses with too-many-rules, cascading
        # into no-status rows). Removal epochs live inside this case's
        # block of 10 (epoch stride is frozen at 10). Never raises;
        # returns (all_ok, detail). Verifies via ListRules.
        detail = {"attempted": list(placed_ids), "removed": [],
                  "failed": [], "verified_gone": None}
        try:
            # Clients were created up front in __init__ (see DDS lesson
            # note there); just use them. (Import here only binds the
            # type name; it creates no DDS entities.)
            from rulesync_interfaces.srv import RemoveRule as _RR
            cli_rm = self._cli_rm
            if not cli_rm.wait_for_service(timeout_sec=10.0):
                detail["failed"] = list(placed_ids)
                detail["error"] = "remove_rule service unavailable"
                return False, detail
            for j, rid in enumerate(placed_ids):
                try:
                    fut = cli_rm.call_async(_RR.Request(
                        rule_id=rid, epoch=int(epoch + 3 + j)))
                    rclpy.spin_until_future_complete(self, fut,
                                                     timeout_sec=20.0)
                    r = fut.result()
                    if r is not None and bool(getattr(r, "ok", False)):
                        detail["removed"].append(rid)
                    else:
                        # Idempotent removal: 'unknown rule_id' for an
                        # already-absent rule is success IFF verification
                        # confirms it is gone (e.g., bridge auto-rollback
                        # removed it concurrently). Genuine failures
                        # (no-response, still-present) stay failures.
                        reason = (getattr(r, "reason", None)
                                  if r is not None else "no-response")
                        if str(reason) == "unknown rule_id":
                            detail["removed"].append(f"{rid}:already-gone")
                        else:
                            detail["failed"].append(f"{rid}:{reason}")
                except Exception as e:  # noqa: BLE001 - harness must not die
                    detail["failed"].append(f"{rid}:error:{e}")
            # verify via the up-front ListRules client: none of our ids
            # may remain active
            try:
                if self.cli_list.wait_for_service(timeout_sec=10.0):
                    from rulesync_interfaces.srv import (
                        ListRules as _LRReq)
                    fut = self.cli_list.call_async(_LRReq.Request())
                    rclpy.spin_until_future_complete(self, fut,
                                                     timeout_sec=10.0)
                    lr = fut.result()
                    if lr is not None:
                        active = {getattr(r, "rule_id", "")
                                  for r in getattr(lr, "active", [])}
                        detail["verified_gone"] = bool(
                            active.isdisjoint(set(placed_ids)))
                    else:
                        detail["verified_gone"] = False
                else:
                    detail["verified_gone"] = False
            except Exception as e:  # noqa: BLE001 - verification is advisory
                detail["verified_gone"] = False
                detail["verify_error"] = str(e)
            ok = (not detail["failed"]
                  and detail["verified_gone"] is True)
            return ok, detail
        except Exception as e:  # noqa: BLE001 - harness must not die
            detail["error"] = str(e)
            return False, detail

    def run_case(self, start, goal, epoch, n_rules=1, fixed_rules=None):
        from topopatch.topo.gate import (FALLBACK_RETRY_GAP_S,
                                         MAX_FALLBACK_REPLANS,
                                         gate_full_path)
        self.statuses = []
        out = {"planner": self.planner_id, "map": self.map_tag,
               "start": start,
               "goal": goal, "epoch": epoch, "n_rules": n_rules,
               "rule_src": ("manifest" if fixed_rules is not None
                            else "auto")}
        # feeds bridge feasibility (M2 pattern; without it the bridge
        # votes NO_FEASIBLE_PATH on every rule). Latched repeatedly:
        # single volatile shots race bridge subscription timing.
        from geometry_msgs.msg import PoseStamped
        import time as _tg
        gp = PoseStamped()
        gp.header.frame_id = "map"
        gp.pose.position.x, gp.pose.position.y = float(goal[0]), float(
            goal[1])
        gp.pose.orientation.w = 1.0
        for _ in range(6):
            self.goal_pub.publish(gp)
            _tg.sleep(0.3)
        t0 = time.time()
        plan, plan_t, plan_rs = self.plan(start, goal)
        out["plan_t"] = plan_t
        out["plan_result"] = plan_rs
        if not plan:
            out.update(ok=False, reason=f"no-plan:{plan_rs}")
            return out
        out["plan_len"] = round(sum(
            math.hypot(a[0] - b[0], a[1] - b[1])
            for a, b in zip(plan[1:], plan[:-1])), 3)
        # manifest rules (frozen) or auto placement across plan thirds
        if fixed_rules is not None:
            rects = [[(float(p[0]), float(p[1])) for p in poly]
                     for poly in fixed_rules]
        else:
            rects = []
            for k in range(n_rules):
                frac = (k + 1) / (n_rules + 1)
                mid = plan[min(len(plan) - 1, int(len(plan) * frac))]
                h = RECT_HALF
                rects.append([(mid[0] - h, mid[1] - h),
                              (mid[0] - h, mid[1] + h),
                              (mid[0] + h, mid[1] + h),
                              (mid[0] + h, mid[1] - h)])
        out["rules"] = [[[round(float(x), 3), round(float(y), 3)]
                         for x, y in r] for r in rects]
        # mask baseline BEFORE propose (bridge publishes during commit;
        # snapshotting later would mistake the new mask for baseline).
        mask_gen_at_commit = self.mask_watcher.snapshot()["gen"]
        # Incremental commit (one rule per epoch): the bridge validates
        # feasibility per rule; infeasible placements are skipped, not
        # forced. placed_n (1..n_rules) is recorded; 0 -> rollback case.
        placed = []
        placed_ids = []
        okc, ct, crs = True, 0.0, "ok"
        t_propose, t_commit = 0.0, 0.0
        for k, poly in enumerate(rects):
            ok1, tp1, tc1, _ = self.propose_commit([poly], epoch + k)
            t_propose += tp1
            t_commit += tc1
            ct += tp1 + tc1
            if ok1:
                placed.append(poly)
                placed_ids.append(f"paper-{epoch + k}-r0")
            else:
                crs = f"rule{k}-infeasible"
                break
        out["commit_t"] = round(ct, 2)
        out["t_propose_ms"] = round(t_propose, 1)
        out["t_commit_ms"] = round(t_commit, 1)
        out["placed_n"] = len(placed)
        if not placed:
            out.update(ok=False, reason="commit-failed",
                       bridge_reject=True, removal_ok=True,
                       removal_detail="nothing placed")
            return out
        rects = placed
        epoch = epoch + len(placed) - 1
        try:
            st = self.wait_status(epoch)
            if not st:
                out.update(ok=False, reason="no-status")
                return out
            out.update(accepted=bool(st.get("accepted")),
                   fallback=bool(st.get("fallback")),
                   reason=st.get("reason"),
                   topo_pred=st.get("topo_pred"),
                   rep_prob=st.get("rep_prob"),
                   conf=st.get("conf"),
                   latency_ms=st.get("latency_ms"),
                   infer_ms=st.get("infer_ms"))
            if out["fallback"] and not out["accepted"]:
                from topopatch_ros.txn_trace import wait_mask_ready
                _mr, _mw, _md = wait_mask_ready(
                    self, self.mask_watcher, self.cli_list, epoch,
                    timeout_s=3.0,
                    base_gen=mask_gen_at_commit)
                out["mask_ready"] = bool(_mr)
                out["mask_wait_ms"] = round(_mw, 1)
                fb, attempts, gate_rs = None, 0, "gate:not-attempted"
                gate_ms = 0.0
                for _ in range(MAX_FALLBACK_REPLANS):
                    attempts += 1
                    t1 = time.time()
                    cand, _, _ = self.plan(start, goal, timeout=30.0)
                    out.setdefault("fallback_t_first",
                                   round(time.time() - t1, 2))
                    import time as _tt
                    if not cand:
                        gate_rs = "gate:plan-failed"
                    elif self._map is None:
                        _tg = _tt.monotonic()
                        go, gate_rs = gate_full_path(
                            cand, rects, lambda x, y: False,
                            (-1e9, -1e9, 1e9, 1e9))
                        gate_ms += (_tt.monotonic() - _tg) * 1000.0
                        if go:
                            fb = cand
                            break
                    else:
                        m = self._map
                        w_, data_ = m.info.width, list(m.data)
                        res_, ox_, oy_ = (m.info.resolution,
                                          m.info.origin.position.x,
                                          m.info.origin.position.y)

                        def _occ(x, y, _w=w_, _d=data_, _r=res_,
                                 _ox=ox_, _oy=oy_):
                            ix = int((x - _ox) / _r)
                            iy = int((y - _oy) / _r)
                            if 0 <= ix < _w and 0 <= iy < len(_d) // _w:
                                return _d[iy * _w + ix] > 50
                            return True

                        _b = (ox_, oy_, ox_ + m.info.width * res_,
                              oy_ + m.info.height * res_)
                        _tg2 = _tt.monotonic()
                        go, gate_rs = gate_full_path(
                            cand, rects, _occ, _b)
                        gate_ms += (_tt.monotonic() - _tg2) * 1000.0
                        if go:
                            fb = cand
                            break
                    if attempts < MAX_FALLBACK_REPLANS:
                        time.sleep(FALLBACK_RETRY_GAP_S)
                out["fallback_attempts"] = attempts
                out["fallback_gate"] = gate_rs
                out["gate_ms"] = round(gate_ms, 1)
                out["fallback_ok"] = bool(fb)
                out["abort_recommended"] = not bool(fb)
                if fb:
                    out["fallback_len"] = round(sum(
                        math.hypot(a[0] - b[0], a[1] - b[1])
                        for a, b in zip(fb[1:], fb[:-1])), 3)
            out["commit_to_executable_ms"] = round(
                (time.time() - t0) * 1000, 1)
            out["ok"] = True
            return out
        finally:
            # Guaranteed cleanup on ALL paths (success, no-status,
            # fallback outcomes, exceptions): without this, placed rules
            # leak into later cases and topo refuses with too-many-rules.
            rok, rdetail = self._remove_placed(placed_ids, epoch)
            out["removal_ok"] = bool(rok)
            out["removal_detail"] = str(rdetail)
            if not rok:
                print(f"[bench] WARNING: removal failed epoch={epoch} "
                      f"detail={rdetail}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--planner", default="GridBased")
    ap.add_argument("--start", default="-2.0,-0.5")
    ap.add_argument("--goals", default=None,
                    help="required unless --cases-file is given")
    ap.add_argument("--epoch0", type=int, default=301)
    ap.add_argument("--n-rules", type=int, default=1)
    ap.add_argument("--map", default="")
    ap.add_argument("--cases-file", default=None,
                    help="manifest cases [{goal, epoch, rules, start?}]; "
                    "overrides --goals/--n-rules/--epoch0")
    a = ap.parse_args()
    goals_doc = json.load(open(a.goals)) if a.goals else None
    if goals_doc is None:
        assert a.cases_file, "--goals or --cases-file required"
        goals = []
    elif isinstance(goals_doc, dict):
        goals = goals_doc["goals"]
        if a.start == "-2.0,-0.5" and "start" in goals_doc:
            a.start = f"{goals_doc['start'][0]},{goals_doc['start'][1]}"
    else:
        goals = goals_doc
    rclpy.init()
    node = Bench(a.planner, map_tag=a.map)
    if not node.ac.wait_for_server(timeout_sec=30.0):
        print("[bench] FATAL: no compute_path_to_pose", flush=True)
        return
    for name, cli in (("propose", node.cli_propose),
                       ("commit", node.cli_commit)):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f"[bench] FATAL: /rulesync/{name} unavailable",
                  flush=True)
            return
    for name, cli in (("remove_rule", node._cli_rm),
                      ("list_rules", node.cli_list)):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f"[bench] FATAL: /rulesync/{name} unavailable",
                  flush=True)
            return
    sx, sy = (float(v) for v in a.start.split(","))
    rows = []
    if a.cases_file:
        import json as _jf2
        _m = _jf2.load(open(a.cases_file))
        _cases = [((c["goal"][0], c["goal"][1]), c["epoch"],
                   [tuple(map(float, c.get("start", [sx, sy])))][0],
                   c.get("rules"), c.get("uid", "")) for c in _m["cases"]]
    else:
        _cases = [((gx, gy), a.epoch0 + 10 * i, (sx, sy), None, "")
                  for i, (gx, gy) in enumerate(goals)]
    for (gx, gy), epoch, (sx, sy), _rules, _uid in _cases:
        print(f"[bench] planner={a.planner} epoch={epoch} "
              f"goal=({gx},{gy})", flush=True)
        try:
            row = node.run_case((sx, sy), (gx, gy), epoch,
                                n_rules=a.n_rules,
                                fixed_rules=_rules)
            if _uid:
                row["uid"] = _uid
        except Exception as e:
            import traceback
            traceback.print_exc()
            row = {"planner": a.planner, "goal": (gx, gy),
                   "epoch": epoch, "ok": False, "reason": f"error: {e}"}
        print(f"[bench] result: accepted={row.get('accepted')} "
              f"reason={row.get('reason')} "
              f"fallback_ok={row.get('fallback_ok')} "
              f"removal_ok={row.get('removal_ok')}", flush=True)
        rows.append(row)
        # Crash-safe streaming: rewrite the CSV after every case.
        _dump_rows(a.csv, rows)
        # Fail loud, not silent: repeated removal failures mean rules
        # are accumulating and later rows would be garbage
        # (topo too-many-rules -> no-status cascade). Abort instead.
        if row.get("removal_ok") is False:
            _nbad = sum(1 for r in rows[-3:]
                        if r.get("removal_ok") is False)
            if _nbad >= 3:
                print("[bench] FATAL: 3 consecutive removal failures "
                      "— aborting campaign (rules would accumulate)",
                      flush=True)
                break
    print(f"[bench] wrote {a.csv}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


def _dump_rows(csv_path, rows):
    import csv as _csv
    with open(csv_path, "w", newline="") as f:
        keys = sorted({k for r in rows for k in r.keys()})
        w = _csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})


if __name__ == "__main__":
    main()
