# UAV bridge participant for RuleSync-3D (mirrors nav2_bridge roles).
#
# - Votes on RuleVolume sets (feasibility = voxel-A* reachable from
#   current pose to last goal avoiding the proposed set + occupancy).
# - Applies commit rounds as the new active volume set (staged swap).
# - Serves applied state for manager handshake reconciliation.
# - Publishes the active set (latched) for gate/follower consumers.
# No auto-replan/rollback in the feasibility build (documented scope).
import time

import rclpy
from rclpy.node import Node

READY, REJECT, TIMEOUT = 0, 1, 2


class UavBridge(Node):
    def __init__(self):
        # Unique node name per process (see UavDriver: stale-GUID
        # poisoning on name reuse after kill -9).
        import os as _os
        super().__init__(f"rulesync_nav2_bridge_uav_{_os.getpid()}")
        self.declare_parameter("participant_name", "uav_bridge")
        self.declare_parameter("world_boxes", "")
        self.declare_parameter("occ_res", 0.2)
        self.declare_parameter("margin_m", 0.4)
        self.declare_parameter("spawn_xyz", [10.0, 10.0, 3.0])
        # Runtime crash-hook arm (ProtoCrash C2): ros2 param set to
        # "post-apply"; env RULESYNC_BRIDGE_CRASH_AT is the fallback.
        self.declare_parameter("crash_at", "")
        from rulesync_interfaces.srv import BridgeVoteVolume as _BVV
        from rulesync_interfaces.srv import BridgeVolumeState as _BVS
        self.create_service(_BVV, "/rulesync/uav_bridge_vote_volume",
                            self.on_vote)
        self.create_service(_BVS, "/rulesync/uav_bridge_state",
                            self.on_state)
        # 2D vote service: present for discovery/prewarm hygiene, always
        # REJECTs (this participant votes volumes only).
        from rulesync_interfaces.srv import BridgeVote as _BV2
        self.create_service(_BV2, "/rulesync/uav_bridge_vote",
                            self.on_vote_2d)
        from sensor_msgs.msg import JointState as _JS
        self.create_subscription(_JS, "/r2i_quad/joint_state",
                                 self._on_js, 10)
        from geometry_msgs.msg import PoseStamped as _PS
        self.create_subscription(_PS, "/uav_goal", self._on_goal, 10)
        self._xyz = None
        self._goal = None
        self._spawn = tuple(float(v) for v in
                            self.get_parameter("spawn_xyz").value)
        self.active_volumes = []
        self.active_epoch = 0
        self.staged_volumes = []
        self.staged_epoch = 0
        self.staged_txn = ""
        self.staged_hash = ""
        self._occ = None
        self.get_logger().info("uav_bridge ready")

    # ---------- state ----------
    def _on_js(self, msg):
        try:
            pos = {n: p for n, p in zip(msg.name, msg.position)}
            self._xyz = (
                self._spawn[0] + float(pos.get("jx", 0.0)),
                self._spawn[1] + float(pos.get("jy", 0.0)),
                self._spawn[2] + float(pos.get("jz", 0.0)))
        except Exception:
            pass

    def _on_goal(self, msg):
        self._goal = (float(msg.pose.position.x),
                      float(msg.pose.position.y),
                      float(msg.pose.position.z))

    def _ensure_occ(self):
        if self._occ is not None:
            return True
        import ast
        import numpy as _np
        try:
            boxes = ast.literal_eval(
                str(self.get_parameter("world_boxes").value))
        except Exception as e:
            self.get_logger().warning(f"bad world_boxes: {e}")
            return False
        from rulesync3d.rulesync3d.planners_3d import voxelize_bounds
        from rulesync3d.rulesync3d.worlds_3d import BOUNDS
        res = float(self.get_parameter("occ_res").value)
        margin = float(self.get_parameter("margin_m").value)
        # world_boxes entries are AABB (xmin,ymin,zmin,xmax,ymax,zmax) —
        # same file the case driver consumes directly. (A prior revision
        # misread them as center/size, exploding the y-wall to cover the
        # room and voting everything "in collision".)
        ab = [tuple(float(v) for v in b) for b in boxes]
        grid, org = voxelize_bounds(ab, res, BOUNDS, margin=margin)
        self._occ = (grid, org, res)
        return True

    def _vols_to_keepouts(self, vols):
        # RuleVolume -> AABB keepout for planning (AABB/SPHERE exact
        # inflate; CONVEX via vertex bbox, conservative).
        import numpy as _np
        from rulesync3d.rulesync3d.gate_3d import (
            GEOM_AABB, GEOM_SPHERE)
        boxes = []
        r = float(self.get_parameter("margin_m").value)
        for v in vols:
            g = int(v.geometry_type)
            if g == GEOM_AABB:
                c = (v.center.x, v.center.y, v.center.z)
                d = (v.dimensions.x, v.dimensions.y, v.dimensions.z)
                boxes.append((c[0] - d[0] / 2 - r, c[1] - d[1] / 2 - r,
                              c[2] - d[2] / 2 - r, c[0] + d[0] / 2 + r,
                              c[1] + d[1] / 2 + r, c[2] + d[2] / 2 + r))
            elif g == GEOM_SPHERE:
                c = (v.center.x, v.center.y, v.center.z)
                rr = float(v.dimensions.x) + r
                boxes.append((c[0] - rr, c[1] - rr, c[2] - rr,
                              c[0] + rr, c[1] + rr, c[2] + rr))
            else:
                xs = [p.x for p in v.vertices]
                ys = [p.y for p in v.vertices]
                zs = [p.z for p in v.vertices]
                boxes.append((min(xs) - r, min(ys) - r, min(zs) - r,
                              max(xs) + r, max(ys) + r, max(zs) + r))
        return boxes

    def _point_in_volumes(self, pt, vols):
        # 2D-analog admission check: reject iff an endpoint (robot pose)
        # lies inside a proposed volume (inflated by body margin).
        # Full-segment enforcement belongs to gate_3d, not admission —
        # same division as the 2D nav2_bridge (robot/goal-in-polygon).
        import numpy as _np
        from rulesync3d.rulesync3d.gate_3d import (
            GEOM_AABB, GEOM_SPHERE)
        r = float(self.get_parameter("margin_m").value)
        p = _np.asarray(pt, dtype=float)
        for v in vols:
            g = int(v.geometry_type)
            if g == GEOM_AABB:
                c = _np.array([v.center.x, v.center.y, v.center.z],
                              dtype=float)
                d = _np.array([v.dimensions.x, v.dimensions.y,
                               v.dimensions.z], dtype=float)
                if bool(((p >= c - d / 2 - r) & (p <= c + d / 2 + r))
                        .all()):
                    return True
            elif g == GEOM_SPHERE:
                c = _np.array([v.center.x, v.center.y, v.center.z],
                              dtype=float)
                if float(((p - c) ** 2).sum()) <= \
                        (float(v.dimensions.x) + r) ** 2:
                    return True
            else:
                xs = [q.x for q in v.vertices]
                ys = [q.y for q in v.vertices]
                zs = [q.z for q in v.vertices]
                if xs and (min(xs) - r <= p[0] <= max(xs) + r) and \
                        (min(ys) - r <= p[1] <= max(ys) + r) and \
                        (min(zs) - r <= p[2] <= max(zs) + r):
                    return True
        return False

    def _point_occupied(self, pt):
        if not self._ensure_occ():
            return None
        grid, org, res = self._occ
        ix, iy, iz = int((pt[0] - org[0]) / res), \
            int((pt[1] - org[1]) / res), int((pt[2] - org[2]) / res)
        H = (grid.shape[0], grid.shape[1], grid.shape[2])
        if 0 <= ix < H[2] and 0 <= iy < H[1] and 0 <= iz < H[0]:
            return bool(grid[iz, iy, ix])
        return True

    def feasibility(self, vols):
        # Admission control only (2D-analog): endpoints must be outside
        # proposed volumes and free space. Reachability is NOT voted on:
        # a committed volume with no remaining path is demonstrated via
        # planner failure + HOLD (fail-closed), not via commit rejection.
        xyz = self._xyz
        # TEST HOOK (ProtoCrash matrix only): fixed pose without sim.
        # Env-gated, default OFF; logged when active.
        import os as _os
        _tp = str(_os.environ.get("RULESYNC_TEST_POSE", "") or "")
        if _tp:
            try:
                xyz = tuple(float(v) for v in _tp.split(","))
                self.get_logger().info(f"test-pose override {xyz}")
            except Exception:
                pass
        if xyz is None:
            return False, "no pose yet"
        if not self._ensure_occ():
            return False, "no occupancy"
        if self._point_in_volumes(xyz, vols):
            return False, "NO_FEASIBLE_PATH: robot inside volume"
        occ = self._point_occupied(xyz)
        if occ is None:
            return False, "no occupancy"
        if occ:
            return False, "NO_FEASIBLE_PATH: robot in collision"
        return True, "ok"

    # ---------- 2PC participant (prepared identity, §5) ----------
    def _bridge_hash(self, vols):
        # Canonical snapshot hash over APPLIED/STAGED msgs (mirror of the
        # manager canonicalizer; parity-tested on shared vectors).
        from rulesync3d.rulesync3d.ruleguard import (
            snapshot_hash, volume_snapshot_items)
        return snapshot_hash(volume_snapshot_items(list(vols)))

    def _handle(self, vols, epoch, op, txn_id="", target_hash=""):
        if op == "prepare":
            ok, reason = self.feasibility(list(vols))
            self.get_logger().info(
                f"prepare: feasible={ok} reason={reason}")
            if not ok:
                return REJECT, reason
            # Integrity: recompute hash over the RECEIVED target and
            # compare with the coordinator's target_hash (transport check).
            try:
                seen = self._bridge_hash(list(vols))
            except Exception as e:
                return REJECT, f"hash-failed: {e}"
            if str(target_hash) and seen != str(target_hash):
                return REJECT, "target-hash-mismatch"
            self.staged_volumes = list(vols)
            self.staged_epoch = int(epoch)
            self.staged_txn = str(txn_id)
            self.staged_hash = str(target_hash or seen)
            return READY, "ready"
        if op == "commit":
            # Certificate verification: exact prepared-identity match, or
            # the COMMIT is stale/mismatched and MUST be rejected (§5).
            if str(txn_id) != str(getattr(self, "staged_txn", "")) or \
                    int(epoch) != int(self.staged_epoch) or \
                    str(target_hash) != str(getattr(self, "staged_hash",
                                                   "")):
                return REJECT, "certificate-mismatch"
            self.active_volumes = list(self.staged_volumes)
            self.active_epoch = int(self.staged_epoch)
            self.staged_volumes, self.staged_epoch = [], 0
            self.staged_txn, self.staged_hash = "", ""
            # Deterministic crash hook for ProtoCrash C2 (post-apply,
            # pre-ACK): trigger channels in priority order —
            #   1. /tmp/uav_bridge_crash file ("post-apply" to fire;
            #      consumed on fire so a stale arm can never crash a
            #      later incarnation — file triggers don't depend on
            #      DDS discovery, which proved too flaky for arming),
            #   2. ROS param `crash_at` (runtime arming),
            #   3. env RULESYNC_BRIDGE_CRASH_AT (launch-time value).
            # Default OFF (all empty): zero behavior change.
            import os as _os
            _hook = ""
            try:
                with open("/tmp/uav_bridge_crash") as _af:
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
                    import sys as _sys
                    _sys.stdout.flush()
                except Exception:
                    pass
                try:
                    _os.remove("/tmp/uav_bridge_crash")
                except Exception:
                    pass
                _os._exit(1)
            return READY, "applied"
        if op == "abort":
            self.staged_volumes, self.staged_epoch = [], 0
            self.staged_txn, self.staged_hash = "", ""
            return READY, "discarded"
        return REJECT, f"unknown op {op}"

    def on_vote_2d(self, req, resp):
        resp.vote, resp.reason = REJECT, "volume-participant"
        return resp

    def on_vote(self, req, resp):
        vote, reason = self._handle(
            list(req.volume_rules), int(req.epoch), str(req.op),
            txn_id=str(getattr(req, "txn_id", "") or ""),
            target_hash=str(getattr(req, "target_hash", "") or ""))
        resp.vote, resp.reason = vote, reason
        return resp

    def on_state(self, req, resp):
        resp.active_epoch = int(self.active_epoch)
        resp.active_volumes = list(self.active_volumes)
        try:
            resp.active_hash = self._bridge_hash(self.active_volumes)
        except Exception:
            resp.active_hash = ""
        return resp


def main():
    rclpy.init()
    node = UavBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
