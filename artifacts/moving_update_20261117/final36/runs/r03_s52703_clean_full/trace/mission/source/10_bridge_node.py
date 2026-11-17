"""Nav2 bridge: staged RuleSpecs -> filter masks + markers (Gate M1).

Vote service: /rulesync/nav2_bridge_vote (BridgeVote).
  prepare: feasibility pre-check + rasterize staged masks (not published).
  commit:  atomically swap staged->active, publish masks + markers.
  abort:   discard staged; active untouched.
Publishes (transient-local): keepout/speed OccupancyGrid masks,
CostmapFilterInfo notices (type 0 keepout base 0 mult 1; type 1 speed-percent
base 100 mult -1, matching lyrical nav2_params.yaml), MarkerArray viz.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CostmapFilterInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rulesync_interfaces.msg import StagedSet, BridgeVoteMsg
from rulesync_interfaces.srv import BridgeVote

READY, REJECT = 0, 1


# ---------- Canonical snapshot identity (correctness revision) ----------
# Vendored mirror of the manager snapshot_identity._rule_snapshot_hash
# (the bridge process must not hard-depend on the manager package).
# Parity is enforced by shared unit vectors in test_protocol_parity.py
# (run on the machine with ROS + checkout on PYTHONPATH). Floats use the
# schema-defined 1e-12 decimal normalization and canonical JSON framing.
def _h_cf(x):
    value = float(x)
    if value == 0.0:
        value = 0.0
    return f"{value:.12f}"


def _h_cpt(q):
    if isinstance(q, dict):
        vals = [q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0)]
    elif hasattr(q, "x"):
        vals = [q.x, q.y, getattr(q, "z", 0.0)]
    else:
        vals = list(q)
    while len(vals) < 3:
        vals.append(0.0)
    return ",".join(_h_cf(v) for v in vals[:3])


def _h_cts(t):
    if t is None:
        return "none"
    if hasattr(t, "sec"):
        return f"{int(t.sec)}.{int(getattr(t, 'nanosec', 0) or 0)}"
    if isinstance(t, (list, tuple)) and len(t) == 2:
        return f"{int(t[0])}.{int(t[1])}"
    return str(t)


def bridge_rule_hash(rules):
    """Canonical hash h_e over RuleSpec msgs (same contract as manager)."""
    import hashlib as _hl
    import json as _json
    items = []
    for r in rules:
        pts = list(getattr(getattr(r, "polygon", None), "points", [])
                   or [])
        fields = [
            str(getattr(r, "rule_id", "?")), int(getattr(r, "epoch", 0)),
            int(getattr(r, "rule_type", 0)),
            int(getattr(r, "severity", 1)),
            str(getattr(r, "frame_id", "")),
            ";".join(_h_cpt((p.x, p.y, getattr(p, "z", 0.0)))
                     for p in pts),
            _h_cf(getattr(r, "speed_limit", 0.0) or 0.0),
            _h_cts(getattr(r, "valid_from", None)),
            _h_cts(getattr(r, "valid_until", None))]
        items.append(_json.dumps(
            fields, ensure_ascii=False, separators=(",", ":")))
    payload = "rulesync.snapshot.v2\n" + "\n".join(sorted(items))
    return _hl.sha256(payload.encode()).hexdigest()


def bridge_cert_ok(staged_txn, staged_epoch, staged_hash,
                   req_txn, req_epoch, req_hash):
    """Exact prepared-identity match for a COMMIT certificate (same
    predicate as manager snapshot_identity.cert_matches; tested jointly)."""
    try:
        if int(req_epoch) != int(staged_epoch):
            return False
    except Exception:
        return False
    return (str(req_txn or "") == str(staged_txn or "")
            and str(req_hash or "") == str(staged_hash or ""))

# Nav2 conventions (verified against lyrical nav2_params + filter docs):
# keepout type=0/base=0/mult=1: data 0 free, 100 keep-out.
KEEPOUT_FREE, KEEPOUT_ZONE = 0, 100
# speed type=1(base=100)/mult=-1: space = 100 - data, so full speed is 0
# and a fraction f of max speed encodes as 100*(1-f).
SPEED_FULL_DATA = 0


class Nav2Bridge(Node):
    def __init__(self):
        super().__init__("rulesync_nav2_bridge")
        self.declare_parameter("map_frame", "map")
        # Root info topics are what the costmap filters actually
        # subscribe (verified: both costmaps listen on
        # /keepout_costmap_filter_info with zero publishers otherwise).
        # Absolute mask topics (leading slash) avoid resolution ambiguity.
        self.declare_parameter("filter_channels", [
            "keepout_costmap_filter_info:/rulesync/keepout_mask:keepout",
            "speed_costmap_filter_info:/rulesync/speed_mask:speed"])
        self.declare_parameter("marker_topic", "/rulesync/rule_markers")
        self.declare_parameter("grid_width", 800)
        self.declare_parameter("grid_height", 800)
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_origin_x", -20.0)
        self.declare_parameter("grid_origin_y", -20.0)
        self.declare_parameter("max_speed_mps", 0.5)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        qos_latched = QoSProfile(depth=1,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        qos_vol = QoSProfile(depth=10,
                             durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.RELIABLE)
        qos = qos_latched
        self.mask_pubs = {}
        self.info_pubs = {}
        self._channels = []
        for ch in self.get_parameter("filter_channels").value:
            info_topic, mask_topic, kind = ch.split(":")
            info_topic = "/" + info_topic.lstrip("/")
            mask_topic = "/" + mask_topic.lstrip("/")
            self.mask_pubs[ch] = self.create_publisher(OccupancyGrid, mask_topic, qos)
            self.info_pubs[ch] = self.create_publisher(CostmapFilterInfo, info_topic, qos)
            self._channels.append((info_topic, mask_topic, kind))
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value), qos)
        self.declare_parameter("participant_name", "nav2_bridge")
        # Runtime crash-hook arm (ProtoCrash2D C2): ros2 param set to
        # "post-apply"; env RULESYNC_BRIDGE_CRASH_AT is the fallback.
        self.declare_parameter("crash_at", "")
        self.vote_pub = self.create_publisher(
            BridgeVoteMsg,
            "/rulesync/votes/" + str(self.get_parameter("participant_name").value),
            qos_vol)
        self.create_subscription(
            StagedSet, "/rulesync/staged_set", self.on_staged, qos_vol)
        # Synchronous vote service (same logic as topic path): the manager
        # collects votes via short-lived helper processes (see vote_once.py),
        # which succeed deterministically in this environment.
        self.create_service(BridgeVote, "/rulesync/nav2_bridge_vote",
                            self.on_vote_svc)
        # Applied-state query for manager startup reconciliation (crash
        # consistency): reports exactly what this bridge last applied,
        # so a restarted manager adopts newer applied state instead of
        # silently serving a stale snapshot/epoch 0.
        from rulesync_interfaces.srv import BridgeState as _BS
        self.create_service(_BS, "/rulesync/nav2_bridge_state",
                            self.on_bridge_state)
        self.active_rules: list = []
        self.active_epoch = 0
        self.staged_rules: list = []
        self.staged_epoch = 0
        self.staged_txn = ""
        self.staged_hash = ""
        self._robot_xy_cache = None
        self._last_goal = None
        # TF map<-odom for frame-correct feasibility (M2 fix: odom coords
        # are NOT map coords; without TF fall back to raw odom + warn).
        import tf2_ros as _tf2
        self._tf_buffer = _tf2.Buffer()
        _tf2.TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("goal_topic").value),
            self._on_goal, 10)
        # Post-commit automation (M2 blockers C/D/E).
        self.declare_parameter("auto_replan", True)
        self.declare_parameter("auto_rollback", True)
        self.declare_parameter("verify_retries", 2)
        from nav_msgs.msg import Path as _Path
        self._last_plan = None
        self.create_subscription(_Path, "/plan", self._on_plan, 10)
        from nav2_msgs.action import NavigateToPose as _N2P
        from nav2_msgs.action import ComputePathToPose as _CPP
        self._nav_action = _N2P
        self._cpp_action = _CPP
        from rclpy.action import ActionClient as _AC
        self._nav_client = _AC(self, _N2P, "navigate_to_pose")
        self._cpp_client = _AC(self, _CPP, "compute_path_to_pose")
        from rulesync_interfaces.srv import RemoveRule as _RR
        self._remove_client = self.create_client(_RR, "/rulesync/remove_rule")
        # Patch-aware rollback guard (M2): a verifier-accepted learned patch
        # for this epoch suppresses rollback even when Nav2's own replan
        # fails (the patch is tracked via FollowPath instead).
        from std_msgs.msg import String as _Str
        self._patch_ok_epoch = 0
        self.create_subscription(
            _Str, "/rulepatch/status", self._on_patch_status, qos_vol)
        # Mask geometry follows the live static map (/map, latched): the
        # keepout/speed filters index masks by world coordinates, so a
        # mismatched grid can invalidate the whole costmap. Nothing is
        # published until map metadata arrives (pre-bridge behavior preserved).
        self._map_info = None
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos)
        self.get_logger().info("nav2_bridge ready (masks await /map)")
        # Prewarm (latency optimization): discover planner actions once
        # at startup so first uses don't pay discovery. Bounded and
        # warn-only: headless stacks may lack navigate_to_pose.
        for _an, _ac in (("compute_path_to_pose", self._cpp_client),
                         ("navigate_to_pose", self._nav_client)):
            try:
                _ok = _ac.wait_for_server(timeout_sec=5.0)
            except Exception:
                _ok = False
            self.get_logger().info(
                f"prewarm {_an}: {'ready' if _ok else 'MISSING'}")

    def _on_odom(self, msg):
        self._robot_xy_cache = (float(msg.pose.pose.position.x),
                                float(msg.pose.pose.position.y))

    def _on_goal(self, msg):
        self._last_goal = (float(msg.pose.position.x),
                           float(msg.pose.position.y))

    def _vote(self, vote: int, reason: str, epoch: int, op: str) -> None:
        self.get_logger().info(f"voting {vote} ep={epoch} op={op}")
        msg = BridgeVoteMsg()
        msg.participant = str(self.get_parameter("participant_name").value)
        msg.vote = int(vote)
        msg.reason = str(reason)
        msg.epoch = int(epoch)
        msg.op = str(op)
        self.vote_pub.publish(msg)

    def _handle(self, rules: list, epoch: int, op: str,
                txn_id: str = "", target_hash: str = "") -> tuple[int, str]:
        if op == "prepare":
            self.get_logger().info("prepare: entering feasibility")
            ok, reason = self.feasibility(list(rules))
            self.get_logger().info(f"prepare: feasible={ok} reason={reason}")
            if not ok:
                self._vote(REJECT, reason, int(epoch), op)
                return REJECT, reason
            # Integrity: recompute hash over the RECEIVED target and
            # compare with the coordinator's target_hash (transport check:
            # geometry can change under identical IDs, so IDs alone never
            # suffice).
            try:
                seen = bridge_rule_hash(list(rules))
            except Exception as e:
                self._vote(REJECT, f"hash-failed: {e}", int(epoch), op)
                return REJECT, f"hash-failed: {e}"
            if str(target_hash) and seen != str(target_hash):
                self._vote(REJECT, "target-hash-mismatch", int(epoch),
                           op)
                return REJECT, "target-hash-mismatch"
            self.staged_rules = list(rules)
            self.staged_epoch = int(epoch)
            self.staged_txn = str(txn_id)
            self.staged_hash = str(target_hash or seen)
            self._vote(READY, "ready", int(epoch), op)
            return READY, "ready"
        elif op == "commit":
            # Certificate verification: exact prepared-identity match, or
            # the COMMIT is stale/mismatched and MUST be rejected.
            if not bridge_cert_ok(self.staged_txn, self.staged_epoch,
                                  self.staged_hash, txn_id, epoch,
                                  target_hash):
                self._vote(REJECT, "certificate-mismatch", int(epoch),
                           op)
                return REJECT, "certificate-mismatch"
            self.active_rules = list(self.staged_rules)
            self.active_epoch = int(self.staged_epoch)
            self.staged_rules, self.staged_epoch = [], 0
            self.staged_txn, self.staged_hash = "", ""
            self.publish_all()
            # Deterministic crash hook for ProtoCrash2D C2 (post-apply,
            # pre-ACK): trigger channels in priority order —
            #   1. /tmp/nav2_bridge_crash file ("post-apply" to fire;
            #      consumed on fire so a stale arm can never crash a
            #      later incarnation — file triggers don't depend on
            #      DDS discovery, which proved too flaky for arming),
            #   2. ROS param `crash_at` (runtime arming),
            #   3. env RULESYNC_BRIDGE_CRASH_AT (launch-time value).
            # Default OFF (all empty): zero behavior change.
            import os as _os
            _hook = ""
            try:
                with open("/tmp/nav2_bridge_crash") as _af:
                    _hook = str(_af.read() or "").strip()
            except Exception:
                _hook = ""
            if not _hook:
                try:
                    _hook = str(self.get_parameter(
                        "crash_at").value or "")
                except Exception:
                    _hook = ""
            if not _hook:
                _hook = str(_os.environ.get("RULESYNC_BRIDGE_CRASH_AT",
                                            "") or "")
            if _hook == "post-apply":
                self.get_logger().warning("crash-hook post-apply firing")
                try:
                    _os.remove("/tmp/nav2_bridge_crash")
                except Exception:
                    pass
                _os._exit(1)
            self._vote(READY, "applied", int(epoch), op)
            import threading as _th
            _th.Thread(target=self._post_commit_followup,
                       args=(list(self.active_rules), int(epoch)),
                       daemon=True).start()
            return READY, "applied"
        elif op == "abort":
            self.staged_rules, self.staged_epoch = [], 0
            self.staged_txn, self.staged_hash = "", ""
            self._vote(READY, "discarded", int(epoch), op)
            return READY, "discarded"
        return REJECT, f"unknown op {op}"

    def on_staged(self, msg) -> None:
        op = str(msg.op)
        self.get_logger().info(
            f"staged rx op={op} ep={int(msg.epoch)} n_rules={len(msg.rules)}")
        self._handle(list(msg.rules), int(msg.epoch), op,
                     txn_id=str(getattr(msg, "txn_id", "") or ""),
                     target_hash=str(getattr(msg, "target_hash", "") or ""))

    def on_vote_svc(self, req, resp):
        vote, reason = self._handle(
            list(req.staged_rules), int(req.epoch), str(req.op),
            txn_id=str(getattr(req, "txn_id", "") or ""),
            target_hash=str(getattr(req, "target_hash", "") or ""))
        resp.vote, resp.reason = vote, reason
        return resp

    def on_bridge_state(self, req, resp):
        resp.active_epoch = int(self.active_epoch)
        resp.active_rules = list(self.active_rules)
        try:
            resp.active_hash = bridge_rule_hash(self.active_rules)
        except Exception:
            pass
        return resp

    def _robot_in_map(self):
        """Robot XY in MAP frame (TF odom->map); fallback raw odom."""
        if self._robot_xy_cache is None:
            return None
        try:
            from rclpy.time import Time as _T  # noqa (documents time domain)
            from geometry_msgs.msg import PointStamped as _PS
            p = _PS()
            p.header.frame_id = "odom"
            p.point.x, p.point.y = (float(self._robot_xy_cache[0]),
                                    float(self._robot_xy_cache[1]))
            q = self._tf_buffer.transform(
                p, "map",
                timeout=rclpy.duration.Duration(seconds=1.0))
            return (float(q.point.x), float(q.point.y))
        except Exception as e:
            self.get_logger().warning(
                f"odom->map TF failed ({e}); using raw odom")
            return (float(self._robot_xy_cache[0]),
                    float(self._robot_xy_cache[1]))

    def feasibility(self, rules) -> tuple[bool, str]:
        # TEST HOOKS (ProtoCrash2D matrix only): fixed robot pose + goal
        # without sim/odom/Nav2. Env-gated, default OFF; logged when active.
        import os as _os
        _ov_xy, _ov_goal, _ov_src = None, None, ""
        _tp = str(_os.environ.get("RULESYNC_TEST_POSE2D", "") or "")
        _tg = str(_os.environ.get("RULESYNC_TEST_GOAL2D", "") or "")
        if _tp:
            try:
                _ov_xy = tuple(float(v) for v in _tp.split(",")[:2])
                _ov_src = "test-pose2d"
            except Exception:
                _ov_xy = None
        if _tg:
            try:
                _ov_goal = tuple(float(v) for v in _tg.split(",")[:2])
                _ov_src = (_ov_src + "+test-goal2d").lstrip("+")
            except Exception:
                _ov_goal = None
        if _ov_src:
            self.get_logger().info(
                f"feasibility override {_ov_src} pose={_ov_xy} goal={_ov_goal}")
        for r in rules:
            pts = [(p.x, p.y) for p in r.polygon.points]
            if (int(r.rule_type) == 0
                    and int(getattr(r, "severity", 1)) == 1):
                goal = _ov_goal if _ov_goal is not None else self._last_goal
                if goal is None:
                    return False, "NO_FEASIBLE_PATH: no known goal yet"
                robot_xy = _ov_xy if _ov_xy is not None \
                    else self._robot_in_map()
                for label, xy in (("robot", robot_xy),
                                  ("goal", goal)):
                    if xy is not None and self._point_in_polygon(xy, pts):
                        return False, f"NO_FEASIBLE_PATH: {label} inside polygon"
        return True, "ok"

    @staticmethod
    def _point_in_polygon(xy, pts) -> bool:
        inside, n = False, len(pts)
        x, y = float(xy[0]), float(xy[1])
        for i in range(n):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % n]
            if ((y0 > y) != (y1 > y)) and \
               (x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0):
                inside = not inside
        return inside

    def _on_map(self, msg) -> None:
        if self._map_info is None:
            self._map_info = (int(msg.info.width), int(msg.info.height),
                              float(msg.info.resolution),
                              float(msg.info.origin.position.x),
                              float(msg.info.origin.position.y))
            self.get_logger().info(
                f"map geometry {self._map_info[0]}x{self._map_info[1]} "
                f"@{self._map_info[2]} org=({self._map_info[3]},"
                f"{self._map_info[4]}); publishing default masks")
            self.publish_all()

    def _grid_geom(self):
        if self._map_info is not None:
            return self._map_info
        return (int(self.get_parameter("grid_width").value),
                int(self.get_parameter("grid_height").value),
                float(self.get_parameter("grid_resolution").value),
                float(self.get_parameter("grid_origin_x").value),
                float(self.get_parameter("grid_origin_y").value))

    def _blank_grid(self, fill: int) -> OccupancyGrid:
        g = OccupancyGrid()
        g.header.frame_id = str(self.get_parameter("map_frame").value)
        g.header.stamp = self.get_clock().now().to_msg()
        w, h, res, ox, oy = self._grid_geom()
        g.info.width, g.info.height, g.info.resolution = w, h, res
        g.info.origin.position.x = ox
        g.info.origin.position.y = oy
        g.data = [fill] * (w * h)
        return g

    def _world_to_grid(self, x: float, y: float):
        _, _, res, ox, oy = self._grid_geom()
        return int((x - ox) / res), int((y - oy) / res)

    def _rasterize(self, grid: OccupancyGrid, pts, value: int) -> None:
        w, h = grid.info.width, grid.info.height
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        gx0, gy0 = self._world_to_grid(min(xs), min(ys))
        gx1, gy1 = self._world_to_grid(max(xs), max(ys))
        res = float(grid.info.resolution)
        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)
        for gy in range(max(0, gy0), min(h, gy1 + 1)):
            for gx in range(max(0, gx0), min(w, gx1 + 1)):
                if self._point_in_polygon(((gx + 0.5) * res + ox,
                                           (gy + 0.5) * res + oy), pts):
                    grid.data[gy * w + gx] = value

    def _filter_info(self, mask_topic: str, ftype: int, base: float,
                       mult: float):
        msg = CostmapFilterInfo()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = int(ftype)
        msg.filter_mask_topic = str(mask_topic)
        msg.base = float(base)
        msg.multiplier = float(mult)
        return msg

    def _on_plan(self, msg) -> None:
        self._last_plan = msg
        self._last_plan_t = self.get_clock().now()

    def _plan_hits_zone(self, rule) -> bool:
        plan = self._last_plan
        if plan is None or not getattr(plan, "poses", None):
            return False
        pts = [(p.x, p.y) for p in rule.polygon.points]
        for pose in plan.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            if self._point_in_polygon((x, y), pts):
                return True
        return False

    def _on_patch_status(self, msg) -> None:
        try:
            import json as _json
            s = _json.loads(str(msg.data))
            if s.get("accepted"):
                note = str(s.get("note", ""))
                if note.startswith("epoch="):
                    self._patch_ok_epoch = max(
                        self._patch_ok_epoch, int(note.split("=")[1]))
        except Exception:
            pass

    def _post_commit_followup(self, rules, epoch: int) -> None:
        """C: auto-replan if current plan hits new keepout. D: verify with
        ComputePathToPose. E: auto-rollback (remove as new epoch) if the
        path stays invalid after verify_retries AND no accepted learned
        patch for this epoch. All timestamps logged."""
        import time as _time
        keepouts = [r for r in rules
                    if int(r.rule_type) == 0
                    and int(getattr(r, "severity", 1)) == 1]
        if bool(self.get_parameter("auto_replan").value) and keepouts \
                and self._last_goal is not None:
            hit = any(self._plan_hits_zone(r) for r in keepouts)
            if hit:
                from geometry_msgs.msg import PoseStamped as _PS
                g = _PS()
                g.header.frame_id = "map"
                gx, gy = self._last_goal
                g.pose.position.x, g.pose.position.y = float(gx), float(gy)
                g.pose.orientation.w = 1.0
                goal_msg = self._nav_action.Goal()
                goal_msg.pose = g
                t0 = _time.time()
                fut = self._nav_client.send_goal_async(goal_msg)
                self.get_logger().info(
                    f"auto-replan sent (t={t0:.1f}) after keepout commit")
                self._last_replan_t = t0
        if not bool(self.get_parameter("auto_rollback").value):
            return
        # First-contact DDS discovery can take 10-30s in this environment;
        # never validate against an undiscovered action server.
        if not self._cpp_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().warning(
                f"epoch={epoch}: cpp server unavailable, skip validation")
            return
        tries = int(self.get_parameter("verify_retries").value)
        ok = False
        for _ in range(max(tries, 1)):
            if self._check_path_valid():
                ok = True
                break
            if int(getattr(self, "_patch_ok_epoch", 0)) >= int(epoch):
                ok = True
                self.get_logger().info(
                    f"epoch={epoch}: accepted learned patch suppresses rollback")
                break
            _time.sleep(2.0)
        self.get_logger().info(f"post-commit plan valid={ok} epoch={epoch}")
        if not ok:
            rq = self._remove_client  # placeholder to keep linters calm
            self._auto_rollback(epoch)

    def _check_path_valid(self) -> bool:
        import time as _time
        if self._last_goal is None:
            return True
        fut = self._cpp_client.send_goal_async(self._make_cpp_goal())
        t0 = _time.time()
        while not fut.done() and _time.time() - t0 < 25.0:
            _time.sleep(0.1)
        if not fut.done():
            return False
        gh = fut.result()
        if gh is None or not getattr(gh, "accepted", False):
            return False
        res_fut = gh.get_result_async()
        t0 = _time.time()
        while not res_fut.done() and _time.time() - t0 < 25.0:
            _time.sleep(0.1)
        if not res_fut.done():
            return False
        try:
            return len(res_fut.result().result.path.poses) > 0
        except Exception:
            return False

    def _make_cpp_goal(self):
        gx, gy = self._last_goal
        g = self._cpp_action.Goal()
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x = float(gx)
        g.goal.pose.position.y = float(gy)
        g.goal.pose.orientation.w = 1.0
        return g

    def _auto_rollback(self, epoch: int) -> None:
        from rulesync_interfaces.srv import RemoveRule as _RR
        for r in list(self.active_rules):
            rq = _RR.Request()
            rq.rule_id = str(r.rule_id)
            rq.epoch = int(epoch) + 1
            fut = self._remove_client.call_async(rq)
            import time as _time
            t0 = _time.time()
            while not fut.done() and _time.time() - t0 < 40.0:
                _time.sleep(0.1)
            self.get_logger().info(
                f"auto-rollback {r.rule_id}: "
                f"{fut.result() if fut.done() else 'TIMEOUT'}")

    def publish_all(self) -> None:
        keep = self._blank_grid(KEEPOUT_FREE)
        speed = self._blank_grid(SPEED_FULL_DATA)
        vmax = max(0.05, float(self.get_parameter("max_speed_mps").value))
        for r in self.active_rules:
            if int(getattr(r, "severity", 1)) != 1:
                continue
            pts = [(p.x, p.y) for p in r.polygon.points]
            if int(r.rule_type) == 0:
                self._rasterize(keep, pts, KEEPOUT_ZONE)
            else:
                frac = min(1.0, max(0.01, float(r.speed_limit) / vmax))
                self._rasterize(speed, pts, int(round(100 * (1.0 - frac))))
        for (info_topic, mask_topic, kind), pub in zip(
                self._channels, self.mask_pubs.values()):
            pub.publish(keep if kind == "keepout" else speed)
        for (info_topic, mask_topic, kind), pub in zip(
                self._channels, self.info_pubs.values()):
            if kind == "keepout":
                pub.publish(self._filter_info(mask_topic, 0, 0.0, 1.0))
            else:
                pub.publish(self._filter_info(mask_topic, 1, 100.0, -1.0))
        self.marker_pub.publish(self._markers(self.active_rules))

    def _markers(self, rules) -> MarkerArray:
        from geometry_msgs.msg import Point
        arr = MarkerArray()
        for i, r in enumerate(rules):
            mk = Marker()
            mk.header.frame_id = str(self.get_parameter("map_frame").value)
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns, mk.id, mk.action = "rulesync", i, Marker.ADD
            mk.type = Marker.LINE_STRIP
            mk.scale.x = 0.08
            mk.color.a = 0.9
            if int(r.rule_type) == 0:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.2, 0.2
            else:
                mk.color.r, mk.color.g, mk.color.b = 0.2, 0.6, 1.0
            for p in list(r.polygon.points) + [r.polygon.points[0]]:
                mk.points.append(Point(x=float(p.x), y=float(p.y), z=0.05))
            arr.markers.append(mk)
        return arr


def main():
    rclpy.init()
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
