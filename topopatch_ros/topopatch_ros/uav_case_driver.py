# RuleSync-3D closed-loop case driver (single process).
#
# Per case: candidate (voxel A* or RRT) -> commit RuleVolume via the
# volume transaction layer -> gate_3d every candidate -> execute gated
# trajectories via joint-velocity tracking (supervisor limits) ->
# record track, rule-volume entry, contacts, arrival, latencies.
# Mirrors the 2D topo_case_driver evidence shape (§22 fields).
import argparse
import csv
import json
import math
import time

import rclpy
from rclpy.node import Node

from rulesync_interfaces.msg import RuleVolume
from rulesync_interfaces.srv import CommitVolume, ProposeVolume


def stamp_now():
    from builtin_interfaces.msg import Time
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


def parse_disturb(spec):
    """Parse disturbance spec into dict. Grammar (order-free, +-joined):
    'none' | 'lat:<float>' (y-bias m/s) | 'vert:<float>' (z-bias m/s) |
    'noise:<float>' (uniform ±m/s) | 'lag:<int>' (command delay steps).
    Unknown tokens ignored (loud in stdout by caller if needed).
    """
    d = {}
    for tok in str(spec or "none").split("+"):
        tok = tok.strip()
        if not tok or tok == "none":
            continue
        if ":" not in tok:
            continue
        k, v = tok.split(":", 1)
        k = k.strip()
        try:
            if k == "lag":
                d[k] = int(float(v))
            elif k in ("lateral", "lat"):
                d["lateral"] = float(v)
            elif k in ("vertical", "vert"):
                d["vertical"] = float(v)
            elif k == "noise":
                d["noise"] = float(v)
        except ValueError:
            continue
    return d


def path_length_3d(pts):
    import numpy as _np
    p = _np.asarray(pts, dtype=float)
    return float(_np.linalg.norm(_np.diff(p, axis=0), axis=1).sum())


# Planning/smoothing clearance reserve (m) beyond body margin; the gate
# stays nominal. Overridable via --smooth-reserve (recorded per run).
SMOOTH_RESERVE = 0.3


class UavDriver(Node):
    def __init__(self, spawn):
        # Unique node name per process: kill -9'd predecessors leave
        # stale GUIDs on the discovery server; reusing the node name
        # poisons DDS matching (verified: same-name aged publishers
        # degrade to trickle/zero, unique-name fresh nodes work fully).
        import os as _os
        super().__init__(f"uav_case_driver_{_os.getpid()}")
        from std_msgs.msg import Float64 as _F64
        self.cmd = {j: self.create_publisher(
            _F64, f"/r2i_quad/{j}/cmd_vel", 10)
            for j in ("jx", "jy", "jz", "jyaw")}
        from sensor_msgs.msg import JointState as _JS
        self.create_subscription(_JS, "/r2i_quad/joint_state",
                                 self._on_js, 10)
        from geometry_msgs.msg import PoseStamped as _PS
        self.goal_pub = self.create_publisher(_PS, "/uav_goal", 10)
        self.create_subscription(_PS, "/uav_goal", self._on_goal, 10)
        try:
            from ros_gz_interfaces.msg import Contacts as _CT
            self.create_subscription(_CT, "/body_contact",
                                     self._on_contact, 10)
            self.contact_available = True
        except Exception:
            self.contact_available = False
        self.cli_propose = self.create_client(
            ProposeVolume, "/rulesync/propose_volume")
        self.cli_commit = self.create_client(
            CommitVolume, "/rulesync/commit_volume")
        from rulesync_interfaces.srv import RemoveVolume as _RV
        self._cli_rm = self.create_client(_RV, "/rulesync/remove_volume")
        from rulesync_interfaces.srv import ListVolumes as _LV
        self.cli_list = self.create_client(_LV, "/rulesync/list_volumes")
        self._spawn = tuple(spawn)
        self._xyz = None
        self.track = []
        self.track_t = []
        self.contact_n = 0
        self._goal = None

    def _on_js(self, msg):
        try:
            pos = {n: p for n, p in zip(msg.name, msg.position)}
            self._xyz = (
                self._spawn[0] + float(pos.get("jx", 0.0)),
                self._spawn[1] + float(pos.get("jy", 0.0)),
                self._spawn[2] + float(pos.get("jz", 0.0)))
            self.track.append(self._xyz)
            self.track_t.append(time.time())
        except Exception:
            pass

    def _on_goal(self, msg):
        self._goal = (float(msg.pose.position.x),
                      float(msg.pose.position.y),
                      float(msg.pose.position.z))

    def _on_contact(self, msg):
        try:
            self.contact_n += len(getattr(msg, "contacts", []) or [])
        except Exception:
            pass

    def pose(self):
        for _ in range(500):
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._xyz is not None:
                return self._xyz
            time.sleep(0.01)
        return None

    def publish_goal(self, g, repeats=6):
        from geometry_msgs.msg import PoseStamped as _PS2
        gp = _PS2()
        gp.header.frame_id = "map"
        gp.pose.position.x, gp.pose.position.y, gp.pose.position.z = \
            float(g[0]), float(g[1]), float(g[2])
        gp.pose.orientation.w = 1.0
        for _ in range(repeats):
            self.goal_pub.publish(gp)
            time.sleep(0.3)

    def mk_volume(self, rid, epoch, geom):
        v = RuleVolume()
        v.rule_id = rid
        v.epoch = int(epoch)
        v.rule_type = 0
        v.severity = 1
        v.frame_id = "map"
        v.geometry_type = int(geom["type"])
        c = geom.get("center", [0, 0, 0])
        v.center.x, v.center.y, v.center.z = (float(x) for x in c)
        d = geom.get("dims", [0, 0, 0])
        v.dimensions.x, v.dimensions.y, v.dimensions.z = (
            float(x) for x in d)
        from geometry_msgs.msg import Point
        v.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                      for p in geom.get("vertices", [])]
        v.valid_from = stamp_now()
        end = stamp_now()
        end.sec += 3600
        v.valid_until = end
        return v

    def propose_commit(self, vol, timeout=60.0):
        import time as _t
        t0 = _t.monotonic() * 1000.0
        from rulesync_interfaces.srv import ProposeVolume as _PV
        f1 = self.cli_propose.call_async(_PV.Request(volume=vol))
        rclpy.spin_until_future_complete(self, f1, timeout_sec=timeout)
        r1 = f1.result()
        t_prop = round(_t.monotonic() * 1000.0 - t0, 1)
        if r1 is None:
            return False, t_prop, 0.0, "propose-noresponse"
        if not r1.accepted:
            return False, t_prop, 0.0, f"propose-rejected:{r1.reason}"
        from rulesync_interfaces.srv import CommitVolume as _CV
        f2 = self.cli_commit.call_async(
            _CV.Request(epoch=int(vol.epoch)))
        rclpy.spin_until_future_complete(self, f2, timeout_sec=timeout)
        r2 = f2.result()
        t_com = round(_t.monotonic() * 1000.0 - t0 - t_prop, 1)
        if r2 is None:
            return False, t_prop, t_com, "commit-noresponse"
        ok = bool(r2.committed)
        return ok, t_prop, t_com, ("" if ok else f"commit-failed:{r2.reason}")

    def execute(self, waypoints, timeout_s=None, exec_tag="",
                  guard=None, guard_vols=(),
                  r_body=0.4, disturb=None, rng_seed=0, vmax_xy=None,
                  vmax_z=None, monitor_every=1, guard_delay_ms=0.0,
                  guard_noise=0.0, epoch_bind=True):
        # Track waypoints with supervisor limits; HOLD on completion.
        # Budget scales with waypoint count (dense cell chains need more
        # settle steps than sparse smoothed chains); capped at 300 s.
        # guard (RuleGuard or None): execution-time monitor over ACTUAL
        # pose (PATH_GATE+RULEGUARD arm). epoch_bind=False selects the
        # geometry-only arm (no epoch/hash check). disturb: None or dict
        # with lateral/vertical/noise/lag_steps (deviation injection).
        # Characterization knobs: vmax_xy/z (supervisor caps), monitor_
        # every (check every Nth iteration), guard_delay_ms (stale pose
        # for the monitor only), guard_noise (Gaussian sigma on the
        # monitor pose only — control uses the true pose).
        from rulesync3d.rulesync3d.uav_supervisor import (
            Supervisor, waypoint_done)
        from std_msgs.msg import Float64 as _F64
        # Speed-aware budgets (characterization): slow arms (e.g. 0.75)
        # need proportionally more wall time; fixed budgets void them
        # just short of the goal (verified: all tracks stall/timeout at
        # ~3/4 route with 30 s/leg). vmin floors at 0.3 (effective speed
        # runs below the cap under accel slew).
        try:
            _vv = min(float(vmax_xy) if vmax_xy else 1.5,
                      float(vmax_z) if vmax_z else 1.0)
        except Exception:
            _vv = 1.0
        _vv = max(0.3, float(_vv))
        try:
            _pl, _pp = 0.0, (tuple(self._xyz)
                             if self._xyz is not None else None)
            for _w in waypoints:
                if _pp is not None:
                    _pl += math.dist(tuple(_pp), tuple(_w))
                _pp = tuple(_w)
        except Exception:
            _pl = 0.0
        if timeout_s is None:
            timeout_s = min(600.0, max(30.0 + 10.0 * len(waypoints),
                                       _pl / _vv + 90.0))
        sup = Supervisor(vmax_xy=vmax_xy, vmax_z=vmax_z)
        sup.set_mode("TRACK")
        t0 = time.time()
        arrived = False
        stalled = False
        guard_hold = False
        n_cmd = 0
        n_pub = 0
        n_mon = 0
        sum_cmd = [0.0, 0.0, 0.0]
        last_pub = 0.0
        lag_buf = []
        pose_hist = []  # (t_wall, xyz) ring for guard pose delay
        import random as _rnd
        _rng = _rnd.Random(int(rng_seed))
        g_warn = 0
        g_hold_n = 0
        g_inside_n = 0
        g_min_clr = float("inf")
        g_max_pen = 0.0
        g_t_warn = None
        g_t_hold = None
        g_states = []
        try:
            _prev_wp = tuple(self._xyz) \
                if self._xyz is not None else None
        except Exception:
            _prev_wp = None
        for wp in waypoints:
            t_wp = time.time()
            d_wp0 = None
            t_stall0 = None
            try:
                _seg = math.dist(
                    tuple(_prev_wp), tuple(wp)) \
                    if _prev_wp is not None else 0.0
            except Exception:
                _seg = 0.0
            _wp_budget = max(30.0, _seg / _vv + 20.0)
            while time.time() - t_wp < _wp_budget and \
                    time.time() - t0 < timeout_s:
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.008)
                if self._xyz is None:
                    continue
                if waypoint_done(self._xyz, wp):
                    break
                import math as _m
                d_wp = _m.dist(tuple(self._xyz), tuple(wp))
                if d_wp0 is None:
                    d_wp0, t_stall0 = d_wp, time.time()
                # Stall: strong command, no progress (solver poisoning
                # via sustained contact locks joints; pressing longer
                # only voids the case — abort loud instead).
                if time.time() - t_stall0 > 10.0 and \
                        (d_wp0 - d_wp) < 0.1:
                    print(f"[uav-exec] {exec_tag} STALL "
                          f"wp={[round(v, 2) for v in wp]} "
                          f"pos={[round(v, 2) for v in self._xyz]}",
                          flush=True)
                    stalled = True
                    break
                vx, vy, vz = sup.track(self._xyz, wp)
                n_cmd += 1
                # Controlled tracking-deviation injection (experiment).
                if disturb:
                    vy += float(disturb.get("lateral", 0.0))
                    vz += float(disturb.get("vertical", 0.0))
                    nz = float(disturb.get("noise", 0.0))
                    if nz > 0:
                        vx += _rng.uniform(-nz, nz)
                        vy += _rng.uniform(-nz, nz)
                        vz += _rng.uniform(-nz, nz)
                    lag = int(disturb.get("lag_steps", 0))
                    if lag > 0:
                        lag_buf.append((vx, vy, vz))
                        if len(lag_buf) > lag:
                            vx, vy, vz = lag_buf.pop(0)
                        else:
                            vx, vy, vz = lag_buf[0]
                sum_cmd[0] += abs(float(vx))
                sum_cmd[1] += abs(float(vy))
                sum_cmd[2] += abs(float(vz))
                # Execution monitor: ACTUAL pose vs committed snapshot.
                # Throttled to every monitor_every-th iteration; the guard
                # may see a delayed/noisy pose (sensing factors) while
                # control always uses the true pose.
                if guard is not None and n_cmd % max(1, int(
                        monitor_every)) == 0:
                    from rulesync3d.rulesync3d.ruleguard import HOLD as _H
                    from rulesync3d.rulesync3d.ruleguard import (
                        WARNING as _W)
                    from rulesync3d.rulesync3d.ruleguard import (
                        min_clearance_3d as _mc3)
                    _now = time.time()
                    pose_hist.append((_now, tuple(self._xyz)))
                    while len(pose_hist) > 400:
                        pose_hist.pop(0)
                    _gp = tuple(self._xyz)
                    _dd = float(guard_delay_ms or 0.0) / 1000.0
                    if _dd > 0 and pose_hist:
                        _cut = _now - _dd
                        for _tt, _pp in reversed(pose_hist):
                            if _tt <= _cut:
                                _gp = _pp
                                break
                    _ns = float(guard_noise or 0.0)
                    if _ns > 0:
                        _gp = (_gp[0] + _rng.gauss(0.0, _ns),
                               _gp[1] + _rng.gauss(0.0, _ns),
                               _gp[2] + _rng.gauss(0.0, _ns))
                    n_mon += 1
                    if bool(epoch_bind):
                        gst, ginfo = guard.check_bound_3d(
                            _gp, guard_vols, float(r_body))
                    else:
                        import math as _mi
                        _clr = _mc3(_gp, guard_vols, float(r_body))
                        gst, ginfo = guard.check_geometry(_clr)
                        ginfo["clearance"] = _clr
                    g_states.append(gst)
                    clr = float(ginfo.get("clearance", float("inf")))
                    if clr < g_min_clr:
                        g_min_clr = clr
                    if clr < 0:
                        g_inside_n += 1
                        if -clr > g_max_pen:
                            g_max_pen = -clr
                    if gst == _W:
                        g_warn += 1
                        if g_t_warn is None:
                            g_t_warn = round(
                                (time.time() - t0) * 1000.0, 1)
                    elif gst == _H:
                        g_hold_n += 1
                        if g_t_hold is None:
                            g_t_hold = round(
                                (time.time() - t0) * 1000.0, 1)
                        print(f"[uav-guard] {exec_tag} HOLD "
                              f"clr={clr:.3f} "
                              f"pos={[round(v, 2) for v in self._xyz]} "
                              f"{ginfo.get('reason')}", flush=True)
                        guard_hold = True
                        break
                # Rate-gated publishing (25 Hz): the loop spins at joint-
                # state rate (~1000 Hz); publishing cmd_vel at that rate
                # floods the bridge and delivery collapses over minutes
                # (verified: mean|vx| 0.26 at 1000 Hz vs full authority
                # at 6 Hz). Pose updates stay at full rate.
                now_pub = time.time()
                if now_pub - last_pub >= 0.04:
                    last_pub = now_pub
                    n_pub += 1
                    self.cmd["jx"].publish(_F64(data=float(vx)))
                    self.cmd["jy"].publish(_F64(data=float(vy)))
                    self.cmd["jz"].publish(_F64(data=float(vz)))
                    self.cmd["jyaw"].publish(_F64(data=0.0))
                if n_cmd <= 3 or n_cmd % 2000 == 0:
                    print(f"[uav-exec] {exec_tag} n={n_cmd} "
                          f"pos={[round(v, 2) for v in self._xyz]} "
                          f"wp={[round(v, 2) for v in wp]} "
                          f"cmd=({vx:.2f},{vy:.2f},{vz:.2f})", flush=True)
            else:
                continue
            if stalled:
                break
            try:
                _prev_wp = tuple(wp)
            except Exception:
                pass
        for c in self.cmd.values():
            c.publish(_F64(data=0.0))
        sup.set_mode("HOLD")
        if n_cmd > 0:
            print(f"[uav-exec] {exec_tag} cmds={n_cmd} pubs={n_pub} "
                  f"mean|v|=({sum_cmd[0]/n_cmd:.2f},"
                  f"{sum_cmd[1]/n_cmd:.2f},{sum_cmd[2]/n_cmd:.2f})",
                  flush=True)
        for _ in range(25):
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.04)
        if self._xyz is not None and waypoints:
            arrived = waypoint_done(self._xyz, waypoints[-1],
                                    tol_xy=0.5, tol_z=0.3)
        g_info = {"guard_warn_n": g_warn, "guard_hold_n": g_hold_n,
                  "guard_inside_n": g_inside_n,
                  "guard_min_clearance": (None if g_min_clr == float("inf")
                                          else round(g_min_clr, 3)),
                  "guard_max_penetration": round(g_max_pen, 3),
                  "guard_checks": n_mon,
                  "guard_monitor_hz": (round(n_mon / max(
                      1e-3, time.time() - t0), 1) if n_mon else 0.0),
                  "guard_t_warning_ms": g_t_warn,
                  "guard_t_hold_ms": g_t_hold,
                  "guard_hold": bool(guard_hold)}
        if guard_hold:
            self.stop()
            return arrived, round(time.time() - t0, 1), False, g_info
        if stalled:
            self.stop()
            return arrived, round(time.time() - t0, 1), True, g_info
        return arrived, round(time.time() - t0, 1), False, g_info

    def stop(self):
        from std_msgs.msg import Float64 as _F64
        for c in self.cmd.values():
            c.publish(_F64(data=0.0))

    def recreate_pubs(self):
        # DELIBERATELY REMOVED (was: destroy + recreate cmd publishers per
        # case as aging insurance). Endpoint destroy/create churn on a live
        # node is RACY here: recreated publishers sometimes never deliver
        # (verified: selftest on original pubs moves, post-recreate
        # reposition stalls with strong commands). Stable endpoints created
        # once at birth + fresh process per world is the proven pattern.
        # This no-op preserves the call site.
        return

    def actuator_selftest(self, timeout_each=12.0):
        """Fail-fast actuator check: pulse each linear axis and require
        joint motion. Catches pinned/degraded joints (e.g. jz stuck at
        its lower limit on some stacks) BEFORE burning cases. Returns
        (ok, detail). Quad ends near its start pose (reposition follows).
        """
        import time as _t
        from std_msgs.msg import Float64 as _F64
        results = {}
        for j in ("jx", "jy", "jz"):
            p0 = self._xyz
            t0 = _t.time()
            while _t.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.01)
                if self._xyz is not None:
                    p0 = self._xyz
                    break
            if p0 is None:
                return False, "no-pose"
            # Pulse TOWARD mid-range: a joint sitting at its limit cannot
            # move further (correctly reads 0) — test the free direction.
            ax = {"jx": 0, "jy": 1, "jz": 2}[j]
            sign = -1.0 if p0[ax] > 1.0 else 1.0
            t0 = _t.time()
            while _t.time() - t0 < timeout_each:
                rclpy.spin_once(self, timeout_sec=0.0)
                self.cmd[j].publish(_F64(data=0.6 * sign))
                time.sleep(0.05)
                if self._xyz is not None:
                    if abs(self._xyz[ax] - p0[ax]) > 0.15:
                        break
            moved = (abs(self._xyz[ax] - p0[ax])
                     if self._xyz is not None else 0.0)
            results[j] = round(moved, 3)
            self.stop()
            time.sleep(0.5)
        ok = all(v > 0.15 for v in results.values())
        print(f"[uav] actuator selftest {results} ok={ok}", flush=True)
        return ok, str(results)

    def plan_candidate(self, planner, start, goal, occ, occ_fn, bounds,
                         vols=(), seed=0):
        # vols: gate-ready inflated dicts for RRT edge checks; for A*
        # they are rasterized as keepouts (caller passes occ grid that
        # already includes them via keepouts_from_geoms).
        from rulesync3d.rulesync3d.planners_3d import (
            rrt_connect, voxel_astar)
        import time as _t
        t_p = _t.time()
        cand, ms = None, 0.0
        if planner == "ompl":
            from rulesync3d.rulesync3d.ompl_3d import ompl_rrt_connect
            cand = ompl_rrt_connect(start, goal, occ_fn, bounds,
                                    vols=vols, seed=seed)
        elif planner == "rrt":
            cand = rrt_connect(start, goal, occ_fn, bounds,
                               vols=vols, seed=seed)
        else:
            grid, org, res = occ
            cand = voxel_astar(grid, org, res, start, goal)
        ms = round((_t.time() - t_p) * 1000.0, 1)
        return cand, ms

    def run_case(self, case, occ, occ_meta, vols_inflated, bounds,
                 occ_res, r_body):
        # case: {uid, start, goal, volumes geoms, epoch (ASSIGNED by
        # caller: epoch_base + idx*100), planner}
        # Protocol per case (measured section starts AFTER reposition):
        #   1. reposition (unmeasured TRACK transit to case start)
        #   2. STALE candidate planned IGNORING volumes (race shape)
        #   3. commit RuleVolumes via 2PC (bridge endpoint admission)
        #   4. gate STALE candidate -> native_rule_violation evidence
        #   5. FRESH replan avoiding committed volumes -> gate -> execute
        #      or fail-closed HOLD (planner failure / gate-blocked-fresh)
        from rulesync3d.rulesync3d.gate_3d import gate_3d_path
        gx, gy, gz = (float(v) for v in case["goal"])
        sx, sy, sz = (float(v) for v in case["start"])
        epoch = int(case["epoch"])
        seed = int(case.get("seed", epoch))
        planner = case.get("planner", "astar")
        out = {"uid": case.get("uid", ""), "epoch": epoch,
               "planner": planner, "map": case.get("map", ""),
               "start": [sx, sy, sz], "goal": [gx, gy, gz]}
        t0 = time.time()
        self.track = []
        self.track_t = []
        self.contact_n = 0
        self.recreate_pubs()
        # 1. reposition (UNMEASURED) transit to spec start: planned below
        # (after occ_fn) — straight-line TRACK wedged into the slab.
        _live0 = self.pose()
        if _live0 is None:
            out.update(ok=False, reason="no-pose")
            return out
        _grid0, _org0, _res0 = occ
        out["start_live"] = [round(v, 3) for v in _live0]
        self.publish_goal((gx, gy, gz))

        def occ_fn(x, y, z):
            grid, org, res = occ
            ix, iy, iz = int((x - org[0]) / res), \
                int((y - org[1]) / res), int((z - org[2]) / res)
            H = (grid.shape[0], grid.shape[1], grid.shape[2])
            if 0 <= ix < H[2] and 0 <= iy < H[1] and 0 <= iz < H[0]:
                return bool(grid[iz, iy, ix])
            return True

        # 1b. reposition transit (planned above needs occ_fn; defined here).
        # NOTE: planned+gated path (occupancy only): straight-line TRACK
        # drove straight into the barrier slab and wedged (verified).
        from rulesync3d.rulesync3d.gate_3d import gate_3d_path as _g3
        from rulesync3d.rulesync3d.planners_3d import (
            shortcut_smooth as _ss, voxel_astar as _va)
        _repo_raw = _va(_grid0, _org0, _res0, _live0, (sx, sy, sz))
        if not _repo_raw:
            out.update(ok=False, reason="reposition-failed",
                       start_live=[round(v, 3) for v in _live0])
            return out
        _repo = _ss(_repo_raw, occ_fn, vols=())
        _go0, _rs0 = _g3(_repo, [], occ_fn, bounds, occ_res=occ_res,
                         r_body=r_body)
        if not _go0:
            out.update(ok=False, reason=f"reposition-failed:{_rs0}",
                       start_live=[round(v, 3) for v in _live0])
            return out
        arrived0, repo_t, repo_stall, _repo_g = self.execute(
            _repo, timeout_s=120.0,
            exec_tag=f"reposition-{out['uid']}")
        out["reposition_ok"] = bool(arrived0)
        out["reposition_t"] = repo_t
        out["reposition_stall"] = bool(repo_stall)
        self.track = []
        self.track_t = []
        self.contact_n = 0
        start = self.pose()
        if start is None:
            out.update(ok=False, reason="no-pose")
            return out
        if not arrived0:
            out.update(ok=False, reason="reposition-failed",
                       start_live=[round(v, 3) for v in start])
            return out

        # 2. STALE candidate (UNTRUSTED, ignores volumes)
        stale, ms_stale = self.plan_candidate(
            planner, start, (gx, gy, gz), occ, occ_fn, bounds, vols=(),
            seed=seed)
        out["planner_ms"] = ms_stale
        if not stale:
            out.update(ok=False, reason="no-plan-stale")
            return out
        out["plan_len_stale"] = round(path_length_3d(stale), 3)
        # 3. commit the RuleVolumes (1..3 per case)
        geoms = case.get("volumes", [case.get("volume")])
        geoms = [g for g in geoms if g]
        vols = [self.mk_volume(f"vol-{epoch}-{k}", epoch + k, g)
                for k, g in enumerate(geoms)]
        okc, tp, tc, placed, fail_rs = True, 0.0, 0.0, [], ""
        for v in vols:
            o1, a1, b1, rs = self.propose_commit(v)
            tp += a1
            tc += b1
            if o1:
                placed.append(v)
            else:
                fail_rs = rs
                break
        okc = len(placed) == len(vols)
        out["t_propose_ms"] = tp
        out["t_commit_ms"] = tc
        out["placed_n"] = len(placed)
        if geoms and not okc:
            self.stop()
            out.update(ok=False, reason=fail_rs or "commit-failed")
            return out
        if not geoms:
            out["reason_baseline"] = "no-volume-baseline"
        # 4. gate EVERY candidate: stale first (native violation evidence)
        t_g = time.time()
        go_s, rs_s = gate_3d_path(stale, vols_inflated, occ_fn, bounds,
                                  occ_res=occ_res, r_body=r_body)
        out["stale_gate"] = rs_s
        out["native_rule_violation"] = bool(geoms and not go_s
                                            and "KEEP_OUT_VOLUME" in rs_s)
        # 4b. THREE-ARM enforcement baseline (paired scenarios):
        #   rulesync: transaction + gate (+guard) [default, step 5 below]
        #   native:   commit, plan immediately (stale), TRUST, execute.
        #   wait:     commit, wait for downstream ready-ack, plan fresh,
        #             TRUST without gate. Optional fault injection
        #             (bad_candidate) substitutes a violating straight
        #             candidate AFTER readiness — WAIT trusts it, RULESYNC
        #             would block it. Answers "why not simply wait?".
        arm3 = str(case.get("arm3", getattr(self, "_arm3", "rulesync")))
        out["arm3"] = arm3
        fault = str(case.get("fault", "none"))
        out["fault"] = fault
        if arm3 in ("native", "wait") and geoms:
            import time as _t3
            if arm3 == "wait":
                # Downstream ready-ack: committed epoch visible via
                # list_volumes (manager authority). No gate afterwards.
                from rulesync_interfaces.srv import ListVolumes as _LV
                t_w = _t3.time()
                need = max(int(v.epoch) for v in placed)
                acked = False
                while _t3.time() - t_w < 30.0:
                    if self.cli_list.wait_for_service(timeout_sec=2.0):
                        fq = self.cli_list.call_async(_LV.Request())
                        rclpy.spin_until_future_complete(
                            self, fq, timeout_sec=5.0)
                        fr = fq.result()
                        if fr is not None and int(fr.epoch) >= need:
                            acked = True
                            break
                    _t3.sleep(0.5)
                out["wait_ack_ms"] = round(
                    (_t3.time() - t_w) * 1000.0, 1)
                out["wait_acked"] = bool(acked)
            if fault == "bad_candidate":
                # Faulty planner output AFTER readiness: straight shot
                # through the committed volumes (known-violating by
                # construction on constraining cases).
                _sx, _sy, _sz = (float(v) for v in start)
                cand = [tuple(start), (gx, gy, gz)]
                out["plan_len_3d"] = round(path_length_3d(cand), 3)
            elif arm3 == "native":
                cand = stale
                out["plan_len_3d"] = out.get("plan_len_stale")
            else:
                from rulesync3d.rulesync3d.planners_3d import (
                    keepouts_from_geoms as _kf, voxelize_bounds as _vb)
                from rulesync3d.rulesync3d.worlds_3d import BOUNDS as _B2
                grid, org, res = occ
                keep = _kf(geoms, margin=r_body)
                occ_f = occ
                if keep and planner != "rrt":
                    kgrid, _ = _vb(keep, res, _B2, margin=0.0)
                    import numpy as _np
                    occ_f = (_np.asarray(grid) | _np.asarray(kgrid),
                             org, res)
                cand, ms_f = self.plan_candidate(
                    planner, start, (gx, gy, gz), occ_f, occ_fn,
                    bounds, vols=tuple(vols_inflated), seed=seed + 1)
                out["planner_ms_fresh"] = ms_f
                if not cand:
                    self.stop()
                    out.update(ok=True, reason="no-feasible-path-hold",
                               arrived=False, safe_stop=True)
                    self._record_track(out, vols_inflated, t0)
                    for v in placed:
                        self._cleanup(v, int(v.epoch))
                    out["cleanup_ok"] = True
                    return out
                out["plan_len_3d"] = round(path_length_3d(cand), 3)
            # TRUSTED execution: no gate, no guard (that IS the experiment).
            self._last_cand = [list(map(float, q)) for q in cand]
            arrived, exec_t, stalled, _g0 = self.execute(
                cand, exec_tag=f"{arm3}-{out['uid']}",
                disturb=parse_disturb(
                    case.get("disturb", getattr(self, "_disturb",
                                                "none"))),
                rng_seed=epoch)
            out["arrived"] = bool(arrived)
            out["exec_t"] = exec_t
            out["safe_stop"] = False
            out.update(ok=True, reason=f"{arm3}-executed")
            self._record_track(out, vols_inflated, t0)
            out["gated_rule_violation"] = bool(out.get("rule_violation"))
            for v in placed:
                self._cleanup(v, int(v.epoch))
            out["cleanup_ok"] = True
            return out
        # 5. FRESH replan avoiding committed volumes, then gate + execute
        if geoms:
            from rulesync3d.rulesync3d.planners_3d import (
                keepouts_from_geoms, voxelize_bounds)
            from rulesync3d.rulesync3d.worlds_3d import BOUNDS as _B
            grid, org, res = occ
            keep = keepouts_from_geoms(geoms, margin=r_body)
            if keep and planner != "rrt":
                kgrid, _ = voxelize_bounds(keep, res, _B, margin=0.0)
                import numpy as _np
                occ_fresh = (_np.asarray(grid) | _np.asarray(kgrid),
                             org, res)
            else:
                occ_fresh = occ
            fresh, ms_fresh = self.plan_candidate(
                planner, start, (gx, gy, gz), occ_fresh, occ_fn,
                bounds, vols=tuple(vols_inflated), seed=seed + 1)
            out["planner_ms_fresh"] = ms_fresh
            if not fresh:
                # rule ACTIVE, no remaining path -> fail-closed HOLD
                # (infeasible-family evidence, not forced motion)
                self.stop()
                out.update(ok=True, reason="no-feasible-path-hold",
                           arrived=False, safe_stop=True)
                self._record_track(out, vols_inflated, t0)
                for v in placed:
                    self._cleanup(v, int(v.epoch))
                out["cleanup_ok"] = True
                return out
            out["plan_len_3d"] = round(path_length_3d(fresh), 3)
            # Executor tuning: shortcut-smooth (verified free) then gate
            # the SMOOTHED path — gated and executed paths identical.
            # Smoothing uses margin+SMOOTH_RESERVE so shortcuts keep
            # clearance from the gate boundary (tracking tube); the gate
            # itself stays nominal (body margin only).
            from rulesync3d.rulesync3d.gate_3d import inflate_gate_vols
            from rulesync3d.rulesync3d.planners_3d import shortcut_smooth
            vols_smooth = inflate_gate_vols(list(vols_inflated),
                                            SMOOTH_RESERVE)
            fresh = shortcut_smooth(fresh, occ_fn, vols=tuple(vols_smooth))
            out["plan_len_smooth"] = round(path_length_3d(fresh), 3)
            out["n_wps"] = len(fresh)
            if str(case.get("fault", "none")) == "bad_candidate":
                # Fault injection AFTER readiness, THROUGH the verified
                # path: straight known-violating candidate must be BLOCKED
                # by the gate (symmetric with the wait-arm fault case).
                out["fault_injected"] = True
                fresh = [tuple(float(v) for v in start),
                         (gx, gy, gz)]
                out["plan_len_3d"] = round(path_length_3d(fresh), 3)
            t_g2 = time.time()
            go, rs = gate_3d_path(fresh, vols_inflated, occ_fn, bounds,
                                  occ_res=occ_res, r_body=r_body)
            out["gate"] = rs
            out["gate_ms_measured"] = round(
                (time.time() - t_g2) * 1000.0, 2)
            if not go:
                self.stop()
                out.update(ok=True, reason=f"gate-blocked-fresh:{rs}",
                           arrived=False, safe_stop=True)
                self._record_track(out, vols_inflated, t0)
                for v in placed:
                    self._cleanup(v, int(v.epoch))
                out["cleanup_ok"] = True
                return out
            cand = fresh
        else:
            from rulesync3d.rulesync3d.planners_3d import shortcut_smooth
            stale_sm = shortcut_smooth(stale, occ_fn, vols=())
            t_g3 = time.time()
            go, rs = gate_3d_path(stale_sm, vols_inflated, occ_fn,
                                  bounds, occ_res=occ_res, r_body=r_body)
            out["gate"] = rs
            out["gate_ms_measured"] = round(
                (time.time() - t_g3) * 1000.0, 2)
            out["plan_len_3d"] = out["plan_len_stale"]
            out["plan_len_smooth"] = round(path_length_3d(stale_sm), 3)
            out["n_wps"] = len(stale_sm)
            if not go:  # baseline blocked by occupancy (should not happen)
                self.stop()
                out.update(ok=False, reason=f"gate-blocked-baseline:{rs}")
                return out
            cand = stale_sm
        # execute the gated candidate (PATH_GATE vs PATH_GATE+RULEGUARD arm)
        from rulesync3d.rulesync3d.ruleguard import (
            RuleGuard, snapshot_hash, volume_snapshot_items)
        self._last_cand = [list(map(float, q)) for q in cand]
        guard, g_ep, g_hash = None, 0, ""
        arm = str(case.get("arm", getattr(self, "_arm", "gate")))
        out["arm"] = arm
        disturb = parse_disturb(
            case.get("disturb", getattr(self, "_disturb", "none")))
        out["disturb"] = case.get("disturb",
                                  getattr(self, "_disturb", "none"))
        if arm in ("gate_guard", "gate_geomon") and geoms:
            guard = RuleGuard(
                rho_warn=float(case.get("warn_margin", 0.5)),
                rho_release=float(case.get("release_hyst", 0.2)))
            g_ep = max(int(v.epoch) for v in placed)
            expected_hash = snapshot_hash(volume_snapshot_items(
                vols_inflated))
            # Pin the admitted candidate once. The monitor snapshot below is
            # an independent view and may be missing/stale by construction.
            guard.bind_execution(g_ep, expected_hash)
            if bool(case.get("guard_missing", False)):
                # Mechanism case: monitor NEVER receives a snapshot
                # (epoch/hash stay None) -> every check must HOLD degraded.
                pass
            elif bool(case.get("guard_stale", False)):
                guard.update_snapshot(g_ep, "stale-injected")
            else:
                guard.update_snapshot(g_ep, expected_hash)
        out["warn_margin"] = float(case.get("warn_margin", 0.5))
        out["release_hyst"] = float(case.get("release_hyst", 0.2))
        out["monitor_every"] = int(case.get("monitor_every", 1))
        out["guard_delay_ms"] = float(case.get("guard_delay_ms", 0.0))
        out["guard_noise"] = float(case.get("guard_noise", 0.0))
        out["vmax_xy"] = case.get("vmax_xy", None)
        out["vmax_z"] = case.get("vmax_z", None)
        out["epoch_bind"] = bool(arm != "gate_geomon")
        arrived, exec_t, stalled, g_info = self.execute(
            cand, exec_tag=f"main-{out['uid']}", guard=guard,
            guard_vols=tuple(vols_inflated),
            # NOTE: vols_inflated already carry the body margin; the guard
            # uses rho=0 against them (single-margin semantics). Passing
            # r_body here would DOUBLE-inflate and hold at 0.34 m true
            # clearance (verified over-conservative in WAIT runs).
            r_body=0.0, disturb=disturb, rng_seed=epoch,
            vmax_xy=case.get("vmax_xy", None),
            vmax_z=case.get("vmax_z", None),
            monitor_every=int(case.get("monitor_every", 1)),
            guard_delay_ms=float(case.get("guard_delay_ms", 0.0)),
            guard_noise=float(case.get("guard_noise", 0.0)),
            epoch_bind=bool(arm != "gate_geomon"))
        out.update(g_info)
        if g_info.get("guard_hold"):
            self.stop()
            out.update(ok=True, reason="ruleguard-hold", arrived=False,
                       safe_stop=True)
            self._record_track(out, vols_inflated, t0)
            for v in placed:
                self._cleanup(v, int(v.epoch))
            out["cleanup_ok"] = True
            return out
        out["arrived"] = bool(arrived)
        out["exec_t"] = exec_t
        out["safe_stop"] = False
        if stalled:
            # LOUD actuator stall (solver poisoning via sustained
            # contact): not an arrival, not a gate event — the case is
            # void for motion evidence, transaction evidence stands.
            out.update(ok=True, reason="actuator-stall", arrived=False,
                       safe_stop=True)
            self._record_track(out, vols_inflated, t0)
            for v in placed:
                self._cleanup(v, int(v.epoch))
            out["cleanup_ok"] = True
            return out
        out.update(ok=True, reason="ok")
        self._record_track(out, vols_inflated, t0)
        out["gated_rule_violation"] = bool(out.get("rule_violation"))
        for v in placed:
            self._cleanup(v, int(v.epoch))
        out["cleanup_ok"] = True
        return out

    def _record_track(self, out, vols_inflated, t0):
        from rulesync3d.rulesync3d.gate_3d import (
            path_intersects_volumes)
        bad = 0
        for i in range(len(self.track) - 1):
            for v in vols_inflated:
                from rulesync3d.rulesync3d.gate_3d import (
                    segment_intersects_volume)
                if segment_intersects_volume(self.track[i],
                                             self.track[i + 1], v):
                    bad += 1
                    break
        out["track_n"] = len(self.track)
        out["track_inside_rule"] = bad
        out["rule_violation"] = bad > 0
        out["contact_n"] = int(self.contact_n)
        zs = [p[2] for p in self.track] or [0]
        out["max_altitude"] = round(max(zs), 3)
        out["min_altitude"] = round(min(zs), 3)
        out["commit_to_verified_ms"] = round(
            (time.time() - t0) * 1000.0, 1)
        # Actual speed summary from timestamped track (commanded means
        # are logged by execute(); this is ground truth for the
        # speed factor in characterization).
        try:
            import numpy as _np2
            _tp = _np2.asarray(self.track, dtype=float)
            _tt = _np2.asarray(getattr(self, "track_t", []),
                               dtype=float)
            if len(_tp) >= 2 and len(_tt) == len(_tp) and \
                    _tt[-1] > _tt[0]:
                _seg = _np2.linalg.norm(_np2.diff(_tp, axis=0), axis=1)
                out["actual_speed_mean"] = round(float(_seg.sum() / (
                    _tt[-1] - _tt[0])), 3)
                out["actual_speed_max"] = round(float(
                    (_seg / _np2.maximum(
                        _np2.diff(_tt), 1e-6)).max()), 3)
                out["track_rate_hz"] = round(float(len(_tp) / (
                    _tt[-1] - _tt[0])), 1)
        except Exception:
            pass
        # Forensic dump for ANY track violation or arrival failure:
        # executed candidate + full track + volumes (path vs track
        # deviation analysis lives on these, not on counters).
        if bad > 0 or (out.get("arrived") is False
                       and out.get("reason") == "ok"):
            try:
                import numpy as _np
                import os as _os
                d = "/home/yule/rulepatch/3d/tracks"
                _os.makedirs(d, exist_ok=True)
                uid = str(out.get("uid", "case"))
                ep = int(out.get("epoch", 0))
                _np.savez_compressed(
                    f"{d}/{uid}_{ep}.npz",
                    track=_np.asarray(self.track, dtype=float),
                    cand=_np.asarray(getattr(self, "_last_cand",
                                             []), dtype=float),
                    vols=_np.asarray(
                        [str(v) for v in vols_inflated]))
                out["track_dump"] = f"{uid}_{ep}.npz"
            except Exception as e:
                out["track_dump"] = f"dump-failed:{e}"

    def _cleanup(self, vol, epoch):
        from rulesync_interfaces.srv import RemoveVolume as _RV
        try:
            if self._cli_rm.wait_for_service(timeout_sec=10.0):
                fut = self._cli_rm.call_async(_RV.Request(
                    rule_id=vol.rule_id, epoch=int(epoch) + 10))
                rclpy.spin_until_future_complete(self, fut,
                                                 timeout_sec=20.0)
                r = fut.result()
                out_ok = bool(r is not None and r.ok)
            else:
                out_ok = False
        except Exception:
            out_ok = False
        return out_ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--cases", required=True,
                    help="json list of cases")
    ap.add_argument("--world-boxes", required=True,
                    help="json list of world AABB boxes")
    ap.add_argument("--spawn", default="10.0,10.0,3.0")
    ap.add_argument("--r-body", type=float, default=0.4)
    ap.add_argument("--occ-res", type=float, default=0.2)
    ap.add_argument("--epoch-base", type=int, default=50000,
                    help="assigned epochs: base + idx*100 in run order; "
                    "must exceed manager committed_epoch (fresh "
                    "manager+snapshot per world). Recorded per row.")
    ap.add_argument("--smooth-reserve", type=float, default=0.3,
                    help="extra planning/smoothing clearance (m) beyond "
                    "body margin; gate stays nominal.")
    ap.add_argument("--arm", default="gate",
                    help="enforcement arm: gate (PATH_GATE_ONLY) or "
                    "gate_guard (PATH_GATE+RULEGUARD). Per-case 'arm' "
                    "overrides.")
    ap.add_argument("--disturb", default="none",
                    help="tracking-deviation injection spec (see "
                    "parse_disturb). Per-case 'disturb' overrides.")
    ap.add_argument("--arm3", default="rulesync",
                    help="three-arm baseline: rulesync (gated) | native "
                    "(trust stale) | wait (ready-ack + trust fresh). "
                    "Per-case 'arm3' and 'fault' override.")
    ap.add_argument("--dump-all", action="store_true",
                    help="save every case trajectory NPZ (characterization "
                    "mode; default saves only violations/failures).")
    ap.add_argument("--dump-dir", default="",
                    help="trajectory dump directory (default "
                    "/home/yule/rulepatch/3d/tracks).")
    a = ap.parse_args()
    rclpy.init()
    spawn = tuple(float(v) for v in a.spawn.split(","))
    global SMOOTH_RESERVE
    SMOOTH_RESERVE = float(a.smooth_reserve)
    print(f"[uav] smooth_reserve={SMOOTH_RESERVE} r_body={a.r_body}",
          flush=True)
    node = UavDriver(spawn)
    node._arm = str(a.arm)
    node._disturb = str(a.disturb)
    node._arm3 = str(a.arm3)
    print(f"[uav] arm={node._arm} disturb={node._disturb}", flush=True)
    ok_st, detail = node.actuator_selftest()
    if not ok_st:
        print(f"[uav] FATAL: actuator selftest failed: {detail}",
              flush=True)
        return
    for name, cli in (("propose_volume", node.cli_propose),
                      ("commit_volume", node.cli_commit),
                      ("remove_volume", node._cli_rm),
                      ("list_volumes", node.cli_list)):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f"[uav] FATAL: /rulesync/{name} unavailable",
                  flush=True)
            return
    import ast
    boxes = ast.literal_eval(open(a.world_boxes).read())
    from rulesync3d.rulesync3d.planners_3d import voxelize_bounds
    from rulesync3d.rulesync3d.worlds_3d import BOUNDS
    grid, org = voxelize_bounds(boxes, a.occ_res, BOUNDS,
                                margin=a.r_body)
    occ = (grid, org, a.occ_res)
    cases = json.load(open(a.cases))
    # Epochs are ALLOCATED in run order (base + idx*100), never taken
    # from the scenario file: fixed scenario epochs + globally monotonic
    # committed_epoch poisoned multi-world campaigns (stale rejections).
    # Assigned epochs are infrastructure; manifest freezes everything else.
    for i, c in enumerate(cases):
        c["epoch"] = int(a.epoch_base) + i * 100
        c.setdefault("seed", int(c["epoch"]))
    rows = []
    for c in cases:
        # inflate the case volumes by body margin for the gate
        from rulesync3d.rulesync3d.gate_3d import (
            GEOM_AABB, GEOM_SPHERE, inflate_aabb)
        import numpy as _np
        geoms_in = c.get("volumes", [c.get("volume")])
        geoms_in = [g for g in geoms_in if g]
        gvs = []
        for k, g in enumerate(geoms_in):
            _rid = f"vol-{int(c['epoch'])}-{k}"
            _rep = int(c["epoch"]) + k
            if g["type"] == 0:
                bmin, bmax = inflate_aabb(g["center"], g["dims"],
                                          a.r_body)
                gvs.append({"geometry_type": 0, "bmin": bmin,
                            "bmax": bmax, "rule_id": _rid,
                            "epoch": _rep})
            elif g["type"] == 1:
                gvs.append({"geometry_type": 1,
                            "center": _np.asarray(g["center"], float),
                            "radius": float(g["dims"][0]) + a.r_body,
                            "rule_id": _rid, "epoch": _rep})
            else:
                # convex via vertex bbox (conservative) for the gate
                # dict; exact halfspace gate path is unit-tested;
                # hull-to-halfspace conversion for live volumes is
                # documented future work
                vs = _np.asarray(g["vertices"], float)
                gvs.append({"geometry_type": 0,
                            "bmin": vs.min(axis=0) - a.r_body,
                            "bmax": vs.max(axis=0) + a.r_body,
                            "rule_id": _rid, "epoch": _rep})
        try:
            row = node.run_case(c, occ, None, gvs, BOUNDS, a.occ_res,
                                a.r_body)
        except Exception as e:
            import traceback
            traceback.print_exc()
            row = {"uid": c.get("uid", ""), "ok": False,
                   "reason": f"error: {e}"}
        print(f"[uav] {c.get('uid')} reason={row.get('reason')} "
              f"arrived={row.get('arrived')} viol={row.get('rule_violation')} "
              f"contacts={row.get('contact_n')}", flush=True)
        rows.append(row)
        if a.dump_all and not row.get("track_dump"):
            # Characterization mode: forensic dump for EVERY case, not
            # just violations (schema: track/track_t/cand/vols/uid).
            try:
                import numpy as _npd
                import os as _osd
                _dd = str(a.dump_dir or
                          "/home/yule/rulepatch/3d/tracks")
                _osd.makedirs(_dd, exist_ok=True)
                _uid = str(row.get("uid", c.get("uid", "case")))
                _ep = int(row.get("epoch", c.get("epoch", 0)))
                _npd.savez_compressed(
                    f"{_dd}/{_uid}_{_ep}.npz",
                    track=_npd.asarray(node.track, dtype=float),
                    track_t=_npd.asarray(getattr(node, "track_t", []),
                                         dtype=float),
                    cand=_npd.asarray(getattr(node, "_last_cand", []),
                                      dtype=float),
                    uid=_uid, epoch=_ep)
                row["track_dump"] = f"{_uid}_{_ep}.npz"
            except Exception as e:
                row["track_dump"] = f"dump-failed:{e}"
        with open(a.csv, "w", newline="") as f:
            keys = sorted({k for r in rows for k in r.keys()})
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k) for k in keys})
    print(f"[uav] wrote {a.csv} ({len(rows)} rows)", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
