"""rulesync_manager: propose/validate/stage/2-phase-commit RuleSpecs.

Services: /rulesync/propose_rule, /rulesync/commit_rule,
/rulesync/remove_rule, /rulesync/list_rules.
2PC over transient-local... volatile topics: PREPARE round (all READY?),
COMMIT round (bridges swap staged->active), else ABORT (discard). Old epoch
remains applied until APPLY/ACK; durable volume authority is visible sooner
for revocation, but is not a new-execution release permit. JSON-lines txn log.

Threading (hard-won): vote rounds run in a dedicated worker pool, NEVER on
executor callback threads. Blocking an executor thread starves subscription
dispatch in this environment (observed: queued votes arrive only after the
round, newest-first eviction hides prepare votes). Handlers only wait.
"""
import json
import time
from concurrent.futures import ThreadPoolExecutor
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from rulesync_interfaces.msg import RuleSpec, StagedSet, BridgeVoteMsg
from rulesync_interfaces.msg import RuleVolume
from rulesync_interfaces.srv import (ProposeRule, CommitRule, RemoveRule,
                                       ListRules, BridgeState)
from rulesync_interfaces.srv import (ProposeVolume, CommitVolume,
                                       RemoveVolume, ListVolumes)
from rulesync_interfaces.srv import BridgeVoteVolume, BridgeVolumeState

from .vote_coordinator import VoteCoordinator
from .volume_authority import VolumeAuthority

READY, REJECT, TIMEOUT = 0, 1, 2
_QOS = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE,
                  reliability=ReliabilityPolicy.RELIABLE)


# ---------- Canonical snapshot identity (correctness revision) ----------
# Pure helpers live in snapshot_identity.py (no ROS dependency, unit-tested
# by path without rclpy). The manager cannot import the checkout tree, so
# the volume canonicalizer is mirrored here via that module; parity with
# rulesync3d ruleguard is enforced by shared unit vectors (ruleguard tests
# locally + parity script on the machine). Schema v2 uses canonical JSON and
# 1e-12 numeric normalization.
from .snapshot_identity import (
    _cf, _cpt, _cts, _canon_vol_msg, _vol_snapshot_hash,
    _rule_snapshot_hash, _vol_measure, _vol_op_class,
    _rule_measure, _rule_op_class, cert_matches,
    select_durable_target, recovery_action,
)


class RuleManager(Node):
    def __init__(self):
        super().__init__("rulesync_manager")
        # Generous default: first-contact DDS discovery here can take seconds.
        self.declare_parameter("prepare_timeout_s", 15.0)
        self.declare_parameter("allowed_frames", ["map"])
        self.declare_parameter("participants", ["nav2_bridge"])
        self.declare_parameter("txn_log_path", "/tmp/rulesync_txn.log")
        self.declare_parameter("use_legacy_vote_helper", False)
        # Crash recovery (correctness revision): durable coordinator
        # decision (WAL, fsync) PRECEDES any authoritative participant
        # apply — that ordering IS the linearization point. Recovery is
        # driven by the durable decision, never by adopting bridge state.
        # Bridge state is evidence only. See _wal_write/_recover_state.
        self.declare_parameter("state_path", "/tmp/rulesync_state.json")
        self.declare_parameter("wal_path", "/tmp/rulesync_wal.jsonl")
        self.committed_epoch = 0
        self.active: dict[str, RuleSpec] = {}
        self.staged: dict[str, RuleSpec] = {}
        self.staged_epoch = 0
        # 3D volumes: parallel state sharing the single monotonic
        # committed_epoch space (one consistency domain across
        # geometries — the §10 hypothesis). Same machinery below.
        self.active_vol: dict[str, RuleVolume] = {}
        self.staged_vol: dict[str, RuleVolume] = {}
        self.staged_vol_epoch = 0
        # Volume consistency domain (I1): motion authorization requires
        # manager == bridge == guard on (epoch, hash). _vol_degraded
        # starts TRUE and clears ONLY on explicit convergence (never on
        # timeout/exception). Unknown state is NOT agreement.
        self._vol_degraded = True
        self._vol_txn_n = 0
        self._rule_txn_n = 0
        self._volume_authority = VolumeAuthority()
        self._authority_publish_lock = Lock()
        self._recover_state()
        # Seed only from retained coordinator state/WAL, never bridge state.
        self._restore_volume_authority()
        from rulesync_interfaces.msg import VolumeAuthority as AuthorityMsg
        from rulesync_interfaces.srv import GetVolumeAuthority
        self._authority_callbacks = ReentrantCallbackGroup()
        self._authority_pub = self.create_publisher(
            AuthorityMsg, "/rulesync/volume_authority",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_service(GetVolumeAuthority,
            "/rulesync/get_volume_authority", self.on_get_volume_authority,
            callback_group=self._authority_callbacks)
        self.create_timer(.2, self._publish_volume_authority,
                          callback_group=self._authority_callbacks)
        self._publish_volume_authority()
        self._votes: dict[str, dict] = {}
        self._vote_subs: dict[str, object] = {}
        self._staged_pub = self.create_publisher(
            StagedSet, "/rulesync/staged_set", _QOS)
        for _part in list(self.get_parameter("participants").value):
            self._ensure_vote_sub(_part)
        self._pool = ThreadPoolExecutor(max_workers=2,
                                        thread_name_prefix="round")
        self.create_service(ProposeRule, "/rulesync/propose_rule",
                            self.on_propose)
        self.create_service(CommitRule, "/rulesync/commit_rule",
                            self.on_commit)
        self.create_service(RemoveRule, "/rulesync/remove_rule",
                            self.on_remove)
        self.create_service(ListRules, "/rulesync/list_rules",
                            self.on_list)
        from rulesync_interfaces.srv import BridgeState as _BS0
        self._bstate_cli = self.create_client(
            _BS0, "/rulesync/nav2_bridge_state")
        # 3D volume services (additive; same 2PC machinery).
        self.create_service(ProposeVolume, "/rulesync/propose_volume",
                            self.on_propose_volume)
        self.create_service(CommitVolume, "/rulesync/commit_volume",
                            self.on_commit_volume)
        self.create_service(RemoveVolume, "/rulesync/remove_volume",
                            self.on_remove_volume)
        self.create_service(ListVolumes, "/rulesync/list_volumes",
                            self.on_list_volumes)
        # Helper is the historical default. Opt-in volume votes use a
        # separate continuously spun node, not nested manager spins.
        from rulesync_interfaces.srv import BridgeVolumeState as _BVS
        self._vol_state_cli = self.create_client(
            _BVS, "/rulesync/uav_bridge_state")
        # Persistent vote clients (latency path). Readiness barrier:
        # service discovery once here, before reporting SYSTEM_READY.
        self._coord = VoteCoordinator(
            list(self.get_parameter("participants").value),
            info=lambda m: self.get_logger().info(m))
        pre = self._coord.prewarm(timeout_each=10.0)
        self.get_logger().info(
            f"VoteCoordinator prewarm: {pre} "
            f"(remove_rule/list_rules served locally: ready)")
        import os
        mode = os.environ.get("RULESYNC_VOLUME_VOTE_MODE", "helper")
        if mode not in ("helper", "persistent"):
            raise ValueError("unknown RULESYNC_VOLUME_VOTE_MODE")
        self._volume_coord = None
        if mode == "persistent":
            from .volume_vote_coordinator import VolumeVoteCoordinator
            self._volume_coord = VolumeVoteCoordinator(
                info=lambda m: self.get_logger().info(m))
            self.get_logger().info(
                f"volume coordinator prewarm: {self._volume_coord.prewarm(timeout_each=10.)}")
        # Startup reconciliation (crash consistency): never serve or
        # transact while manager and bridge may disagree. Gate reads
        # via ListRules get the snapshot from t=0 (never blind).
        # Fully async: the compare result lands in _on_bridge_state;
        # the watchdog bounds a lost response. No nested spins.
        self._degraded = True
        self._reconciling = False
        self._ready_announced = False
        self._vol_reconciling = False
        self._reconcile_async()
        self._reconcile_volumes_async()
        # Watchdog (1 s) bounds lost responses; periodic (30 s) covers
        # bridges (re)starting after the manager.
        self.create_timer(1.0, self._reconcile_watchdog)
        self.create_timer(30.0, self._periodic_reconcile)
        if all(pre.values()) and pre:
            self.get_logger().info(
                "rulesync_manager SYSTEM_READY (persistent votes)")
        else:
            self.get_logger().warning(
                "rulesync_manager SYSTEM_DEGRADED (participants missing; "
                "rounds fail closed via vote timeouts)")
        self.get_logger().info("rulesync_manager ready "
                                 f"(epoch {self.committed_epoch})")

    def _periodic_reconcile(self):
        # Issue a fresh compare only when no reconcile is in flight.
        # Periodic attempts do NOT degrade upfront (damping: only a
        # confirmed mismatch degrades); startup does (unknown state).
        # (No nested spins anywhere here: the bridge-state call is
        # async with a done-callback; the watchdog below bounds it.
        # Nested rclpy spins inside manager callbacks wedged service
        # dispatch in testing — the codebase's documented threading
        # rule is that only pool threads block.)
        if getattr(self, "_reconciling", False):
            return
        try:
            self._reconcile_async(degrade_first=False)
        except Exception as e:
            self.get_logger().warning(f"periodic reconcile failed: {e}")
        try:
            self._reconcile_volumes_async()
        except Exception as e:
            self.get_logger().warning(
                f"periodic vol reconcile failed: {e}")

    def _reconcile_async(self, degrade_first=True):
        """Issue one async bridge-state compare. Results land in
        _on_bridge_state; the watchdog bounds a lost response."""
        import time as _t
        if getattr(self, "_reconciling", False):
            return
        self._reconciling = True
        self._reconcile_t0 = _t.monotonic()
        if degrade_first:
            self._degraded = True
        try:
            fut = self._bstate_cli.call_async(BridgeState.Request())
            fut.add_done_callback(self._on_bridge_state)
        except Exception as e:
            self._reconciling = False
            self.get_logger().warning(f"reconcile issue failed: {e}")

    def _recreate_bstate_client(self):
        # DELIBERATELY REMOVED (was: destroy + recreate after 3 fails).
        # Endpoint destroy/create churn is racy here and can poison a
        # working client; timeouts already fail closed. No-op.
        self._bstate_fails = 0
        self.get_logger().warning("reconcile: client recreate disabled; "
                                  "proceeding with snapshot")

    def _reconcile_watchdog(self):
        self._vol_reconcile_watchdog()
        import time as _t
        if getattr(self, "_reconciling", False) and \
                _t.monotonic() - float(
                    getattr(self, "_reconcile_t0", 0)) > 15.0:
            self._reconciling = False
            # Fail-closed: a lost bridge response proves NOTHING about
            # convergence (unknown state is NOT agreement). Stay DEGRADED;
            # the periodic timer retries. Never declare READY on timeout.
            self._degraded = True
            fails = int(getattr(self, "_bstate_fails", 0)) + 1
            self._bstate_fails = fails
            if fails >= 3:
                # Client likely never matched its server: recreate for
                # fresh discovery rather than flapping forever.
                self._recreate_bstate_client()
            self.get_logger().warning(
                "reconcile: bridge state response lost (>15 s); "
                f"DEGRADED until explicit convergence (fail #{fails})")
            self.log_txn(op="reconcile-skip",
                         epoch=int(self.committed_epoch),
                         reason="bridge-state-noresponse")

    def _on_bridge_state(self, fut):
        self._reconciling = False
        try:
            res = fut.result()
        except Exception as e:
            res = None
            self.get_logger().warning(
                f"reconcile: bridge state future failed: {e}")
        try:
            converged = self._reconcile_compare(res)
        except Exception as e:
            converged = False
            self.get_logger().warning(f"reconcile compare failed: {e}")
        # DEGRADED clears ONLY on explicit convergence evidence from the
        # compare (MATCH or successful durable-target replay). Timeout /
        # missing / exception paths stay DEGRADED (fail-closed).
        if converged:
            self._degraded = False
            if not getattr(self, "_ready_announced", False):
                self._ready_announced = True
                self.get_logger().info(
                    "rulesync_manager SYSTEM_READY (reconciled)")
        else:
            self._degraded = True

    def _reconcile_with_bridge(self, periodic=False):
        """Legacy entry (kept for compatibility): async path."""
        self._reconcile_async()

    def _durable_rule_target(self):
        """Authoritative RuleSpec state: (epoch, hash, {id: rule}) from the
        durable coordinator decision (WAL newest COMMIT wins over the
        snapshot file — covers crash between decision and checkpoint).
        Bridge state is evidence, never authority."""
        snap_dicts = [self._rule_to_dict(r)
                      for _, r in sorted(self.active.items())]
        epoch, dicts = select_durable_target(
            int(self.committed_epoch), snap_dicts, self._wal_read(),
            "rule-decision")
        rules = {}
        if epoch == int(self.committed_epoch):
            rules = {k: v for k, v in self.active.items()}
        else:
            for rd in dicts:
                try:
                    r = self._dict_to_rule(rd)
                    rules[str(r.rule_id)] = r
                except Exception:
                    continue
            if dicts and not rules:
                # Corrupt WAL payload must never empty authority: fall
                # back to the snapshot and stay DEGRADED for reconcile.
                self.get_logger().warning(
                    "durable target undecodable; keeping snapshot")
                return (int(self.committed_epoch),
                        _rule_snapshot_hash(list(self.active.values())),
                        {k: v for k, v in self.active.items()})
        return epoch, _rule_snapshot_hash(list(rules.values())), rules

    def _reconcile_compare(self, res):
        """Decision-driven convergence (I1/I3). The durable coordinator
        decision determines authority; the old adopt-bridge rule is
        REMOVED: uncommitted participant state is NEVER promoted (this is
        what makes uncommitted REMOVE/RELAX safe). Returns True only on
        explicit convergence (the sole path that may clear DEGRADED).

        - bridge silent (None) -> DEGRADED (unknown is NOT agreement).
        - bridge (epoch, hash, ids) == durable target -> converge + clear.
        - else -> DEGRADED + idempotent refresh replay of the durable
          target with its certificate; clear only if the replay is
          ACK-converged, else stay DEGRADED for the next retry.
        """
        if res is None:
            self._degraded = True
            self.log_txn(op="reconcile-skip",
                         epoch=int(self.committed_epoch),
                         reason="bridge-state-noresponse")
            return False
        self._bstate_fails = 0
        try:
            b_hash = str(res.active_hash)
        except Exception:
            b_hash = ""
        try:
            b_ids = sorted(str(r.rule_id) for r in res.active_rules)
        except Exception:
            b_ids = []
        b_epoch = int(res.active_epoch)
        d_epoch, d_hash, d_rules = self._durable_rule_target()
        d_ids = sorted(d_rules.keys())
        if recovery_action(d_epoch, d_hash, d_ids, True, b_epoch, b_hash,
                           b_ids) == "converge":
            # Explicit convergence: the ONLY path that clears DEGRADED.
            self.active = {k: v for k, v in d_rules.items()}
            self.committed_epoch = d_epoch
            self._degraded = False
            self.log_txn(op="reconcile-ok", epoch=d_epoch,
                         n_active=len(d_ids))
            self.get_logger().info(
                f"reconcile: converged at epoch {d_epoch} "
                f"({len(d_ids)} rules)")
            return True
        # Disagreement (stale bridge, uncommitted participant state, or
        # hash/geometry mismatch): DEGRADED + idempotent refresh replay of
        # the durable target with its certificate. Never adopt, never
        # clear on this path.
        self._degraded = True
        try:
            timeout = float(self.get_parameter(
                "prepare_timeout_s").value)
            rules = list(d_rules.values())
            txn_id = f"refresh-e{d_epoch}"
            votes = self._run_round(rules, d_epoch, "prepare", timeout,
                                    txn_id, d_hash)
            if votes and all(v == READY for v in votes.values()):
                c_votes = self._run_round(rules, d_epoch, "commit",
                                          timeout, txn_id, d_hash)
                if bool(c_votes) and all(v == READY
                                         for v in c_votes.values()):
                    self.active = {k: v for k, v in d_rules.items()}
                    self.committed_epoch = d_epoch
                    self._degraded = False
                    self.log_txn(op="state-refresh", epoch=d_epoch,
                                 n_active=len(rules), votes=c_votes)
                    self.get_logger().warning(
                        f"reconcile: REFRESHED bridge to durable epoch "
                        f"{d_epoch} ({len(rules)} rules)")
                    return True
            self.log_txn(op="state-refresh-abort", epoch=d_epoch,
                         votes=votes)
            self.get_logger().warning(
                "reconcile: bridge refresh ABORTED by votes; "
                "rounds fail closed")
        except Exception as e:
            # No unconditional finally-clear; stay DEGRADED.
            self.get_logger().warning(f"reconcile refresh failed: {e}")
            self.log_txn(op="state-refresh-error", epoch=d_epoch,
                         reason=str(e))
        return False

    def _reconcile_volumes_async(self):
        # Mirror of the 2D async reconcile for the uav_bridge volume
        # participant. Called from the same watchdog/periodic timers.
        import time as _t
        if getattr(self, "_vol_reconciling", False):
            return
        self._vol_reconciling = True
        self._vol_reconcile_t0 = _t.monotonic()
        try:
            from rulesync_interfaces.srv import BridgeVolumeState as _BVS
            fut = self._vol_state_cli.call_async(_BVS.Request())
            fut.add_done_callback(self._on_vol_bridge_state)
        except Exception as e:
            self._vol_reconciling = False
            self.get_logger().warning(
                f"vol reconcile issue failed: {e}")

    def _vol_reconcile_watchdog(self):
        import time as _t
        if not getattr(self, "_vol_reconciling", False):
            return
        if _t.monotonic() - float(
                getattr(self, "_vol_reconcile_t0", 0)) > 15.0:
            self._vol_reconciling = False
            fails = int(getattr(self, "_vol_state_fails", 0)) + 1
            self._vol_state_fails = fails
            # 6C: bridge silence is NOT agreement. Stay DEGRADED until
            # required (epoch, hash) agreement is explicitly confirmed.
            # (No client recreate here either: racy endpoint churn.)
            self._vol_degraded = True
            self._volume_authority_not_ready()
            self.get_logger().warning(
                "vol reconcile: bridge state lost (>15 s); DEGRADED "
                "until explicit convergence")
            self.log_txn(op="vol-reconcile-skip",
                         epoch=int(self.committed_epoch),
                         reason="bridge-state-noresponse")

    def _on_vol_bridge_state(self, fut):
        self._vol_reconciling = False
        try:
            res = fut.result()
        except Exception as e:
            res = None
            self.get_logger().warning(
                f"vol reconcile: future failed: {e}")
        try:
            self._reconcile_vol_compare(res)
        except Exception as e:
            self._vol_degraded = True
            self._volume_authority_not_ready()
            self.get_logger().warning(
                f"vol reconcile compare failed: {e}")

    def _durable_vol_target(self):
        """Authoritative volume state: (epoch, hash, {id: vol}) from the
        durable coordinator decision (WAL newest COMMIT wins over the
        snapshot file — covers crash between decision and checkpoint).
        Bridge state is evidence, never authority."""
        vols = {k: v for k, v in self.active_vol.items()}
        epoch = int(self.committed_epoch)
        best = None
        for rec in self._wal_read():
            if rec.get("kind") == "vol-decision" and \
                    rec.get("decision") == "COMMIT":
                if best is None or int(rec.get("epoch", 0)) > int(
                        best.get("epoch", 0)):
                    best = rec
        if best is not None and int(best.get("epoch", 0)) >= epoch:
            vols = {}
            for vd in best.get("target_volumes", []):
                v = self._dict_to_vol(vd)
                if str(v.rule_id) in vols:
                    raise ValueError("duplicate durable volume rule id")
                vols[str(v.rule_id)] = v
            if _vol_snapshot_hash(list(vols.values())) != best.get("target_hash"):
                raise ValueError("durable volume target hash mismatch")
            epoch = int(best.get("epoch", epoch))
        return epoch, _vol_snapshot_hash(list(vols.values())), vols

    def _reconcile_vol_compare(self, res):
        # Decision-driven convergence (I1/I3). No adopt-bridge rule:
        # uncommitted participant state is NEVER promoted.
        if res is None:
            # 6C: bridge silence is NOT agreement — stay DEGRADED until
            # required agreement is explicitly confirmed.
            self._vol_degraded = True
            self._volume_authority_not_ready()
            self.log_txn(op="vol-reconcile-skip",
                         epoch=int(self.committed_epoch),
                         reason="bridge-state-noresponse")
            return
        self._vol_state_fails = 0
        try:
            b_hash = str(res.active_hash)
        except Exception:
            b_hash = ""
        b_epoch = int(res.active_epoch)
        d_epoch, d_hash, d_vols = self._durable_vol_target()
        if b_epoch == d_epoch and b_hash and b_hash == d_hash:
            # Explicit convergence: the ONLY path that clears DEGRADED.
            self.active_vol = {k: v for k, v in d_vols.items()}
            self.committed_epoch = d_epoch
            self._vol_degraded = False
            self._volume_authority_ready(d_epoch, d_hash)
            self.log_txn(op="vol-reconcile-ok", epoch=d_epoch,
                         n_active=len(d_vols))
            return
        # Disagreement (stale bridge, uncommitted participant state, or
        # hash mismatch): DEGRADED + idempotent refresh replay of the
        # durable target with its certificate. Never adopt, never clear.
        self._vol_degraded = True
        self._volume_authority_not_ready()
        try:
            timeout = float(self.get_parameter(
                "prepare_timeout_s").value)
            vols = list(d_vols.values())
            votes = self._volume_round(vols, d_epoch, "prepare",
                                       timeout, f"refresh-e{d_epoch}",
                                       d_hash)
            if votes and all(v == READY for v in votes.values()):
                c_votes = self._volume_round(vols, d_epoch, "commit",
                                             timeout, f"refresh-e{d_epoch}",
                                             d_hash)
                if bool(c_votes) and all(v == READY
                                         for v in c_votes.values()):
                    self.active_vol = {k: v for k, v in d_vols.items()}
                    self.committed_epoch = d_epoch
                    self._vol_degraded = False
                    self._volume_authority_ready(d_epoch, d_hash)
                    self.log_txn(op="vol-state-refresh", epoch=d_epoch,
                                 n_active=len(vols), votes=c_votes)
                    return
            self.log_txn(op="vol-state-refresh-abort", epoch=d_epoch,
                         votes=votes)
        except Exception as e:
            # 6D: no unconditional finally-clear; stay DEGRADED.
            self.get_logger().warning(f"vol refresh failed: {e}")
            self.log_txn(op="vol-state-refresh-error", epoch=d_epoch,
                         reason=str(e))

    def log_txn(self, **rec):
        rec.setdefault("t_wall", time.time())
        rec.setdefault("t_mono", time.monotonic())
        rec.setdefault("node", "rulesync_manager")
        try:
            with open(str(self.get_parameter("txn_log_path").value), "a") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception as e:
            self.get_logger().warning(f"txn log write failed: {e}")

    def _rule_to_dict(self, r):
        return {
            "rule_id": str(r.rule_id), "epoch": int(r.epoch),
            "rule_type": int(r.rule_type), "severity": int(r.severity),
            "frame_id": str(r.frame_id),
            "polygon": [[float(p.x), float(p.y), float(p.z)]
                        for p in r.polygon.points],
            "speed_limit": float(getattr(r, "speed_limit", 0.0)),
            "valid_from": [int(r.valid_from.sec),
                           int(r.valid_from.nanosec)],
            "valid_until": [int(r.valid_until.sec),
                            int(r.valid_until.nanosec)],
        }

    def _vol_to_dict(self, v):
        return {
            "rule_id": str(v.rule_id), "epoch": int(v.epoch),
            "rule_type": int(v.rule_type),
            "severity": int(v.severity),
            "frame_id": str(v.frame_id),
            "geometry_type": int(v.geometry_type),
            "center": [float(v.center.x), float(v.center.y),
                       float(v.center.z)],
            "dimensions": [float(v.dimensions.x), float(v.dimensions.y),
                           float(v.dimensions.z)],
            "vertices": [[float(p.x), float(p.y), float(p.z)]
                         for p in v.vertices],
            "valid_from": [int(v.valid_from.sec),
                           int(v.valid_from.nanosec)],
            "valid_until": [int(v.valid_until.sec),
                            int(v.valid_until.nanosec)],
        }

    def _dict_to_vol(self, d):
        from geometry_msgs.msg import Point
        v = RuleVolume()
        v.rule_id = str(d["rule_id"])
        v.epoch = int(d["epoch"])
        v.rule_type = int(d["rule_type"])
        v.severity = int(d.get("severity", 1))
        v.frame_id = str(d["frame_id"])
        v.geometry_type = int(d["geometry_type"])
        v.center.x, v.center.y, v.center.z = (float(x) for x in d["center"])
        v.dimensions.x, v.dimensions.y, v.dimensions.z = (
            float(x) for x in d["dimensions"])
        v.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                      for p in d.get("vertices", [])]
        v.valid_from.sec, v.valid_from.nanosec = (
            int(d["valid_from"][0]), int(d["valid_from"][1]))
        v.valid_until.sec, v.valid_until.nanosec = (
            int(d["valid_until"][0]), int(d["valid_until"][1]))
        return v

    def _dict_to_rule(self, d):
        from geometry_msgs.msg import Point32
        r = RuleSpec()
        r.rule_id = str(d["rule_id"])
        r.epoch = int(d["epoch"])
        r.rule_type = int(d["rule_type"])
        r.severity = int(d["severity"])
        r.frame_id = str(d["frame_id"])
        r.polygon.points = [Point32(x=float(p[0]), y=float(p[1]),
                                    z=float(p[2])) for p in d["polygon"]]
        r.speed_limit = float(d.get("speed_limit", 0.0))
        r.valid_from.sec, r.valid_from.nanosec = (
            int(d["valid_from"][0]), int(d["valid_from"][1]))
        r.valid_until.sec, r.valid_until.nanosec = (
            int(d["valid_until"][0]), int(d["valid_until"][1]))
        return r

    def _wal_write(self, rec):
        """Append-only durable coordinator decision record: write + flush
        + fsync (single-coordinator WAL, no database). Callers must invoke
        BEFORE any authoritative participant apply (linearization point)."""
        import os as _os
        rec = dict(rec)
        rec.setdefault("t_wall", time.time())
        rec.setdefault("t_mono", time.monotonic())
        rec.setdefault("node", "rulesync_manager")
        path = str(self.get_parameter("wal_path").value)
        with open(path, "a") as f:
            f.write(json.dumps(rec, sort_keys=True) + "\n")
            f.flush()
            _os.fsync(f.fileno())
        self._on_durable_decision(rec)
        return rec

    def _on_durable_decision(self, rec):
        """Post-fsync only: expose revocation before blocking APPLY/ACK.

        This does NOT advance the checkpoint/applied snapshot or authorize
        a new execution. If decoding/publication fails, durable COMMIT still
        stands; consumers receive unknown authority / fail closed.
        """
        if rec.get("kind") != "vol-decision" or rec.get("decision") != "COMMIT":
            return
        self._vol_degraded = True
        try:
            rows = list(rec["target_volumes"])
            vols = [self._dict_to_vol(v) for v in rows]
            if len({v.rule_id for v in vols}) != len(vols):
                raise ValueError("duplicate durable rule id")
            digest = _vol_snapshot_hash(vols)
            if digest != rec["target_hash"]:
                raise ValueError("durable target hash mismatch")
            self._volume_authority.install_durable(rec["epoch"], digest, rows)
        except Exception as e:
            self._volume_authority.invalidate()
            self.get_logger().warning(f"durable volume authority unknown: {e}")
        self._publish_volume_authority()

    def _restore_volume_authority(self):
        try:
            epoch, digest, vols = self._durable_vol_target()
            rows = [self._vol_to_dict(v) for _, v in sorted(vols.items())]
            self._volume_authority.install_durable(epoch, digest, rows)
        except Exception as e:
            self._volume_authority.invalidate()
            self.get_logger().warning(f"volume authority recovery unknown: {e}")

    def _volume_authority_message(self):
        from rulesync_interfaces.msg import VolumeAuthority as AuthorityMsg
        view = self._volume_authority.read()
        return AuthorityMsg(available=view.available, epoch=view.epoch,
            snapshot_hash=view.snapshot_hash, ready=view.ready,
            volumes=[self._dict_to_vol(v) for v in view.volumes()])

    def _publish_volume_authority(self):
        try:
            # Serialize read+publish across the independent heartbeat and
            # transaction callbacks: an older heartbeat cannot be published
            # after a newer decision notification by this writer.
            with self._authority_publish_lock:
                self._authority_pub.publish(self._volume_authority_message())
        except Exception as e:
            # A failed notification never undoes a durable decision.
            # Independent RPC and freshness expiry are the fallback.
            self.get_logger().warning(f"volume authority publish failed: {e}")

    def on_get_volume_authority(self, req, resp):
        resp.authority = self._volume_authority_message()
        return resp

    def _volume_authority_not_ready(self):
        view = self._volume_authority.read()
        self._volume_authority.mark_converged(view.epoch, view.snapshot_hash, False)
        self._publish_volume_authority()

    def _volume_authority_ready(self, epoch, digest):
        if not self._volume_authority.mark_converged(epoch, digest, True):
            # This should be impossible in the serialized transaction path.
            # A stale certificate cannot quietly release a newer decision.
            self._vol_degraded = True
        self._publish_volume_authority()

    def _wal_read(self):
        import os as _os2
        out = []
        try:
            path = str(self.get_parameter("wal_path").value)
            if not _os2.path.exists(path):
                return out
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        try:
                            out.append(json.loads(line))
                        except Exception:
                            continue
        except Exception as e:
            self.get_logger().warning(f"WAL read failed: {e}")
        return out

    def _save_state(self):
        # Durable snapshot: tmp + flush + fsync + atomic rename + dir
        # fsync (torn-write AND OS-crash safe, unlike plain rename).
        import os as _os
        try:
            path = str(self.get_parameter("state_path").value)
            blob = json.dumps({
                "committed_epoch": int(self.committed_epoch),
                "active": [self._rule_to_dict(r)
                           for _, r in sorted(self.active.items())],
                "active_vol": [self._vol_to_dict(v)
                               for _, v in sorted(self.active_vol.items())],
            }, sort_keys=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                f.write(blob)
                f.flush()
                _os.fsync(f.fileno())
            _os.replace(tmp, path)
            dfd = _os.open(_os.path.dirname(_os.path.abspath(path))
                           or ".", _os.O_RDONLY)
            try:
                _os.fsync(dfd)
            finally:
                _os.close(dfd)
        except Exception as e:
            self.get_logger().warning(f"state snapshot failed: {e}")

    def _recover_state(self):
        import os as _os
        try:
            path = str(self.get_parameter("state_path").value)
            if not _os.path.exists(path):
                self.get_logger().info(
                    "rulesync_manager ready (epoch 0, no snapshot)")
            else:
                d = json.load(open(path))
                self.committed_epoch = int(d.get("committed_epoch", 0))
                for rd in d.get("active", []):
                    r = self._dict_to_rule(rd)
                    self.active[str(r.rule_id)] = r
                for vd in d.get("active_vol", []):
                    v = self._dict_to_vol(vd)
                    self.active_vol[str(v.rule_id)] = v
                self.get_logger().info(
                    f"rulesync_manager RECOVERED snapshot: epoch "
                    f"{self.committed_epoch}, {len(self.active)} active "
                    f"rule(s) + {len(self.active_vol)} volume(s) "
                    f"from {path}")
                self.log_txn(op="state-recovered",
                             epoch=int(self.committed_epoch),
                             n_active=len(self.active),
                             n_volumes=len(self.active_vol))
            # WAL replay (C3 case): a durable COMMIT newer than the
            # snapshot (crash between decision and checkpoint) is
            # authoritative — rebuild from the decision, never drop it.
            # Volumes first, then the 2D RuleSpec domain (same rule).
            newest = None
            for rec in self._wal_read():
                if rec.get("kind") == "vol-decision" and \
                        rec.get("decision") == "COMMIT":
                    if newest is None or int(rec.get("epoch", 0)) > int(
                            newest.get("epoch", 0)):
                        newest = rec
            if newest is not None and int(newest.get("epoch", 0)) > int(
                    self.committed_epoch):
                self.active_vol = {
                    str(vd["rule_id"]): self._dict_to_vol(vd)
                    for vd in newest.get("target_volumes", [])}
                self.committed_epoch = int(newest["epoch"])
                self._save_state()
                self.log_txn(op="wal-replay-adopt",
                             epoch=int(self.committed_epoch),
                             txn_id=str(newest.get("txn_id", "?")),
                             n_volumes=len(self.active_vol))
            snap_rule_dicts = [self._rule_to_dict(r)
                               for _, r in sorted(self.active.items())]
            r_epoch, r_dicts = select_durable_target(
                int(self.committed_epoch), snap_rule_dicts,
                self._wal_read(), "rule-decision")
            if r_epoch > int(self.committed_epoch):
                rebuilt = {}
                for rd in r_dicts:
                    try:
                        r = self._dict_to_rule(rd)
                        rebuilt[str(r.rule_id)] = r
                    except Exception:
                        continue
                if r_dicts and not rebuilt:
                    self.get_logger().warning(
                        "WAL replay undecodable; keeping snapshot")
                else:
                    self.active = rebuilt
                    self.committed_epoch = int(r_epoch)
                    self._save_state()
                    self.log_txn(op="wal-replay-adopt",
                                 epoch=int(self.committed_epoch),
                                 n_active=len(self.active))
        except Exception as e:
            self.get_logger().warning(
                f"state recovery failed ({e}); starting epoch 0")

    def on_propose(self, req, resp):
        if getattr(self, "_degraded", False):
            self.log_txn(op="propose", rule_id=getattr(req.rule, "rule_id", "?"),
                         accepted=False, reason="degraded: reconciling")
            resp.accepted, resp.reason, resp.staged_epoch = (
                False, "degraded: reconciling", 0)
            return resp
        from .validator import validate_rule
        ok, reason = validate_rule(
            req.rule, self.committed_epoch,
            tuple(self.get_parameter("allowed_frames").value))
        if not ok:
            self.log_txn(op="propose", rule_id=getattr(req.rule, "rule_id", "?"),
                         accepted=False, reason=reason)
            resp.accepted, resp.reason, resp.staged_epoch = False, reason, 0
            return resp
        # Merge semantics: proposals accumulate by rule_id (re-proposal
        # with a newer epoch = deterministic versioned update). A propose
        # must never silently evict previously staged rules.
        self.staged[req.rule.rule_id] = req.rule
        self.staged_epoch = max(int(self.staged_epoch), int(req.rule.epoch))
        self.log_txn(op="propose", rule_id=req.rule.rule_id, accepted=True,
                     staged_epoch=self.staged_epoch,
                     staged_ids=sorted(self.staged))
        resp.accepted, resp.reason, resp.staged_epoch = True, "staged", \
            self.staged_epoch
        return resp

    def _rule_begin_txn(self, target, epoch):
        self._rule_txn_n = int(getattr(self, "_rule_txn_n", 0)) + 1
        txn_id = f"ruletx-e{int(epoch)}-n{int(self._rule_txn_n)}"
        target_hash = _rule_snapshot_hash(list(target.values()))
        op_class = _rule_op_class(self.active, target)
        return txn_id, target_hash, op_class

    def on_remove(self, req, resp):
        if getattr(self, "_degraded", False):
            self.log_txn(op="remove-reject", rule_id=req.rule_id,
                         epoch=int(req.epoch), reason="degraded: reconciling")
            resp.ok, resp.reason = False, "degraded: reconciling"
            return resp
        if int(req.epoch) <= int(self.committed_epoch):
            self.log_txn(op="remove-reject", rule_id=req.rule_id,
                         epoch=int(req.epoch), reason="stale epoch")
            resp.ok, resp.reason = False, "stale epoch"
            return resp
        if req.rule_id not in self.active:
            self.log_txn(op="remove-reject", rule_id=req.rule_id,
                         epoch=int(req.epoch), reason="unknown rule_id")
            resp.ok, resp.reason = False, "unknown rule_id"
            return resp
        # COMPLETE target snapshot (the bridge APPLIES the commit-round
        # set as its new active set, so rounds carry the full remain-set).
        remain = {k: v for k, v in self.active.items()
                  if k != req.rule_id}
        timeout = float(self.get_parameter("prepare_timeout_s").value)
        txn_id, target_hash, op_class = self._rule_begin_txn(
            remain, int(req.epoch))
        votes = self._run_round(list(remain.values()), int(req.epoch),
                                "prepare", timeout, txn_id, target_hash)
        if not (votes and all(v == READY for v in votes.values())):
            self._run_round([], int(req.epoch), "abort", timeout, txn_id,
                            target_hash)
            self._wal_write({"kind": "rule-decision", "txn_id": txn_id,
                             "epoch": int(req.epoch),
                             "target_hash": target_hash,
                             "op_class": op_class, "decision": "ABORT",
                             "target_rules": [
                                 self._rule_to_dict(r) for _, r in
                                 sorted(remain.items())],
                             "votes": votes})
            self.log_txn(op="abort-remove", rule_id=req.rule_id,
                         epoch=int(req.epoch), votes=votes, committed=False,
                         txn_id=txn_id, target_hash=target_hash,
                         op_class=op_class)
            resp.ok, resp.reason = False, f"aborted: {votes}"
            return resp
        # DURABLE COMMIT DECISION — LINEARIZATION POINT. No participant
        # may apply a REMOVE/RELAX before this record exists.
        self._vol_crash_hook("pre-wal", txn_id, int(req.epoch))
        t_w0 = time.monotonic()
        self._wal_write({"kind": "rule-decision", "txn_id": txn_id,
                         "epoch": int(req.epoch),
                         "target_hash": target_hash, "op_class": op_class,
                         "decision": "COMMIT",
                         "target_rules": [self._rule_to_dict(r)
                                          for _, r in
                                          sorted(remain.items())],
                         "votes": votes})
        t_wal_ms = round((time.monotonic() - t_w0) * 1000.0, 2)
        self._vol_crash_hook("post-wal", txn_id, int(req.epoch))
        # COMMIT CERTIFICATE dispatch (txn_id/epoch/hash); bridge applies
        # only on exact prepared-identity match; ACKs converge it.
        c_votes = self._run_round(list(remain.values()), int(req.epoch),
                                  "commit", timeout, txn_id, target_hash)
        self._vol_crash_hook("post-dispatch", txn_id, int(req.epoch))
        # Checkpoint AFTER ACK processing (crash here loses only the
        # checkpoint; WAL replay rebuilds it — never the decision).
        self.active = {k: v for k, v in remain.items()}
        self.committed_epoch = int(req.epoch)
        self._save_state()
        if bool(c_votes) and all(v == READY
                                 for v in c_votes.values()):
            self._degraded = False
            self.log_txn(op="converge-explicit",
                         epoch=int(req.epoch), txn_id=txn_id,
                         target_hash=target_hash)
        else:
            # Decision stands (durable); participant uncertain -> DEGRADED
            # until reconcile replay converges. Never silently proceed.
            self._degraded = True
            self.log_txn(op="commit-uncertain", epoch=int(req.epoch),
                         txn_id=txn_id, target_hash=target_hash,
                         commit_votes=c_votes)
        self.log_txn(op="commit-remove", rule_id=req.rule_id,
                     epoch=int(req.epoch), votes=votes, committed=True,
                     txn_id=txn_id, target_hash=target_hash,
                     op_class=op_class, t_wal_ms=t_wal_ms)
        resp.ok, resp.reason = True, "removed"
        return resp

    def on_commit(self, req, resp):
        epoch = int(req.epoch)
        if getattr(self, "_degraded", False):
            resp.committed, resp.reason = False, "degraded: reconciling"
            return resp
        if epoch != int(self.staged_epoch) or not self.staged:
            resp.committed, resp.reason = False, "nothing staged for epoch"
            return resp
        timeout = float(self.get_parameter("prepare_timeout_s").value)
        # Full-set rounds: the bridge APPLIES the commit-round set as its
        # new active set, so rounds must carry active+staged (merged), never
        # a delta. (A delta previously blanked unrelated active rules on the
        # bridge while the manager stayed correct -- caught by mask audit.)
        merged = {**self.active, **self.staged}
        txn_id, target_hash, op_class = self._rule_begin_txn(merged,
                                                             epoch)
        votes = self._run_round(list(merged.values()), epoch,
                                "prepare", timeout, txn_id, target_hash)
        if not (votes and all(v == READY for v in votes.values())):
            self._run_round([], epoch, "abort", timeout, txn_id,
                            target_hash)
            self._wal_write({"kind": "rule-decision", "txn_id": txn_id,
                             "epoch": epoch, "target_hash": target_hash,
                             "op_class": op_class, "decision": "ABORT",
                             "target_rules": [
                                 self._rule_to_dict(r) for _, r in
                                 sorted(merged.items())],
                             "votes": votes})
            self.log_txn(op="abort", epoch=epoch, votes=votes,
                         committed=False, txn_id=txn_id,
                         target_hash=target_hash, op_class=op_class)
            resp.committed, resp.reason = False, f"aborted: {votes}"
            return resp
        self._vol_crash_hook("pre-wal", txn_id, epoch)  # C0 boundary
        # DURABLE COMMIT DECISION — LINEARIZATION POINT. No participant
        # may apply a relaxation/remove before this record exists.
        t_w0 = time.monotonic()
        self._wal_write({"kind": "rule-decision", "txn_id": txn_id,
                         "epoch": epoch, "target_hash": target_hash,
                         "op_class": op_class, "decision": "COMMIT",
                         "target_rules": [self._rule_to_dict(r)
                                          for _, r in
                                          sorted(merged.items())],
                         "votes": votes})
        t_wal_ms = round((time.monotonic() - t_w0) * 1000.0, 2)
        self._vol_crash_hook("post-wal", txn_id, epoch)  # C1 boundary
        # COMMIT CERTIFICATE dispatch (txn_id/epoch/hash); bridge applies
        # only on exact prepared-identity match; ACKs converge it.
        c_votes = self._run_round(list(merged.values()), epoch, "commit",
                                  timeout, txn_id, target_hash)
        self._vol_crash_hook("post-dispatch", txn_id, epoch)  # C3 boundary
        # TEST HOOK (fault injection F8 only): pause after the bridge
        # applied the commit round but before local bookkeeping, so a
        # crash in between is reproducible. Env-gated, default 0
        # (zero behavior change in production runs).
        import os as _os
        _pause = float(_os.environ.get(
            "RULESYNC_TEST_COMMIT_PAUSE_S", "0") or 0)
        if _pause > 0:
            time.sleep(_pause)
        # Checkpoint AFTER ACK processing (crash here loses only the
        # checkpoint; WAL replay rebuilds it — never the decision).
        # Merge semantics: commit unions staged into active
        # (staged ids win). Removals are explicit via remove_rule only.
        self.active = {**self.active, **self.staged}
        self.committed_epoch = epoch
        self.staged, self.staged_epoch = {}, 0
        self._save_state()
        if bool(c_votes) and all(v == READY
                                 for v in c_votes.values()):
            # Explicit convergence (ACKed exact certificate): clear.
            self._degraded = False
            self.log_txn(op="converge-explicit", epoch=epoch,
                         txn_id=txn_id, target_hash=target_hash)
        else:
            # Decision stands (durable); participant uncertain -> DEGRADED
            # until reconcile replay converges. Never silently proceed.
            self._degraded = True
            self.log_txn(op="commit-uncertain", epoch=epoch,
                         txn_id=txn_id, target_hash=target_hash,
                         commit_votes=c_votes)
        self.log_txn(op="commit", epoch=epoch, votes=votes, committed=True,
                     txn_id=txn_id, target_hash=target_hash,
                     op_class=op_class, t_wal_ms=t_wal_ms)
        resp.committed, resp.reason = True, "committed"
        return resp

    def on_list(self, req, resp):
        resp.active = list(self.active.values())
        resp.staged = list(self.staged.values())
        resp.epoch = int(self.committed_epoch)
        resp.degraded = bool(getattr(self, "_degraded", True))
        try:
            resp.active_hash = _rule_snapshot_hash(
                list(self.active.values()))
        except Exception:
            pass
        return resp

    # ---------- 3D RuleVolume paths (same 2PC machinery) ----------
    def _validate_volume(self, v):
        if str(getattr(v, "frame_id", "")) not in tuple(
                self.get_parameter("allowed_frames").value):
            return False, "bad frame"
        if int(v.epoch) <= int(self.committed_epoch):
            return False, (f"stale epoch {int(v.epoch)} "
                           f"(committed {int(self.committed_epoch)})")
        g = int(v.geometry_type)
        if g == 0:
            d = (float(v.dimensions.x), float(v.dimensions.y),
                 float(v.dimensions.z))
            if min(d) <= 0:
                return False, "degenerate AABB"
        elif g == 1:
            if float(v.dimensions.x) <= 0:
                return False, "degenerate sphere"
        elif g == 2:
            if len(v.vertices) < 4:
                return False, "degenerate convex"
        else:
            return False, "unknown geometry"
        return True, "ok"

    def _volume_round(self, vols, epoch, op, timeout, txn_id="",
                        target_hash=""):
        # Pool-thread helper round (same threading discipline as
        # _run_round: handler threads block on pool futures, never on
        # nested rclpy spins).
        fut = self._pool.submit(self._volume_round_body, vols, epoch,
                                op, timeout, txn_id, target_hash)
        try:
            return fut.result(timeout=timeout + 70.0)
        except Exception as e:
            self.get_logger().warning(f"volume round failed: {e}")
            return {}

    def _volume_round_body(self, vols, epoch, op, timeout, txn_id="",
                             target_hash=""):
        """Volume 2PC via helper (default) or isolated persistent client.
        PREPARE carries (txn_id, epoch,
        target_hash, complete target state); COMMIT/ABORT carry the
        certificate for prepared-identity verification. Fail-closed: any
        helper failure yields TIMEOUT votes -> abort.
        """
        import json as _json
        import subprocess as _sp
        started = time.monotonic()
        vd = [{"rule_id": str(v.rule_id), "epoch": int(v.epoch),
               "rule_type": int(v.rule_type),
               "severity": int(getattr(v, "severity", 1)),
               "frame_id": str(getattr(v, "frame_id", "map")),
               "geometry_type": int(v.geometry_type),
               "center": [float(v.center.x), float(v.center.y),
                          float(v.center.z)],
               "dims": [float(v.dimensions.x), float(v.dimensions.y),
                        float(v.dimensions.z)],
               "vertices": [[float(p.x), float(p.y), float(p.z)]
                            for p in v.vertices],
               "valid_from": [int(v.valid_from.sec),
                              int(v.valid_from.nanosec)],
               "valid_until": [int(v.valid_until.sec),
                               int(v.valid_until.nanosec)]}
              for v in vols]
        if getattr(self, "_volume_coord", None) is not None:
            replies = self._volume_coord.round(vd, epoch, op, timeout,
                                               txn_id, target_hash)
            votes = {"uav_bridge": replies.get("uav_bridge", (TIMEOUT, "missing", 0.))[0]}
            self.log_txn(op=f"vol-{op}", epoch=int(epoch), votes=votes,
                         path="persistent", elapsed_s=time.monotonic()-started,
                         replies=replies)
            return votes
        helper = "/home/yule/rulesync_ws/vote_volume_once.py"
        svc = "/rulesync/uav_bridge_vote_volume"
        cmd = (f"source /opt/ros/lyrical/setup.bash && "
               f"source /home/yule/rulesync_ws/install/setup.bash && "
               f"exec python3 {helper} {svc} {int(epoch)} {op} "
               f"'{_json.dumps(vd)}' {txn_id} {target_hash}")
        # Opt-in portable deployment: use the already sourced manager's
        # interpreter/environment, preserving its ROS domain and transport.
        # Historical deployments retain their original launch profile when
        # this override is absent. Use argv (not shell text) for payloads.
        import os as _os
        import sys as _sys
        portable = _os.environ.get("RULESYNC_VOLUME_VOTE_HELPER", "")
        if portable:
            command = [_sys.executable, portable, svc, str(int(epoch)), op,
                       _json.dumps(vd), str(txn_id), str(target_hash)]
            helper_env = _os.environ.copy()
        else:
            command = ["bash", "-c", cmd]
            helper_env = {"PATH": "/usr/bin:/bin", "HOME": "/home/yule",
                          "ROS_DISCOVERY_SERVER": "127.0.0.1:11811",
                          "ROS_DISABLE_ROS2_DAEMON": "1"}
        votes: dict[str, int] = {}
        try:
            cp = _sp.run(
                command, capture_output=True, text=True,
                timeout=min(timeout, 50.0),
                env=helper_env)
            vote, reason = 2, "helper-failed"
            for line in cp.stdout.splitlines() if cp.returncode == 0 else []:
                if line.startswith("TIMING="):
                    self.get_logger().info("volume helper " + line)
                if line.startswith("VOTE="):
                    chunks = line.split()
                    vote = int(chunks[0].split("=")[1])
                    if vote not in (0, 1, 2):
                        raise ValueError("invalid helper vote")
                    reason = chunks[1].split("=", 1)[1] \
                        if len(chunks) > 1 else ""
            votes["uav_bridge"] = vote
            self.get_logger().info(
                f"volume vote uav_bridge={vote} {reason} op={op} "
                f"ep={epoch}")
        except Exception as e:
            self.get_logger().warning(f"volume helper vote failed: {e}")
            votes["uav_bridge"] = 2
        self.log_txn(op=f"vol-{op}", epoch=int(epoch), votes=votes,
                     path="helper", elapsed_s=time.monotonic()-started)
        return votes

    def destroy_node(self):
        for name in ("_volume_coord", "_coord"):
            coordinator = getattr(self, name, None)
            if coordinator is not None:
                coordinator.shutdown()
        return super().destroy_node()

    def _vol_gates_open(self):
        # Volume consistency domain is gated ONLY on volume convergence
        # (_vol_degraded), not on the 2D domain flag: 2D reconcile flaps
        # (stale duplicate bridge-2 processes answering with phantom
        # epochs) must not wedge volume transactions, and vice versa.
        # The shared monotonic committed_epoch check stays (one space).
        return not bool(getattr(self, "_vol_degraded", False))

    def on_propose_volume(self, req, resp):
        if not self._vol_gates_open():
            resp.accepted, resp.reason, resp.staged_epoch = (
                False, "degraded: reconciling", 0)
            return resp
        ok, reason = self._validate_volume(req.volume)
        if not ok:
            self.log_txn(op="vol-propose",
                         rule_id=getattr(req.volume, "rule_id", "?"),
                         accepted=False, reason=reason)
            resp.accepted, resp.reason, resp.staged_epoch = (
                False, reason, 0)
            return resp
        self.staged_vol[req.volume.rule_id] = req.volume
        self.staged_vol_epoch = max(int(self.staged_vol_epoch),
                                    int(req.volume.epoch))
        self.log_txn(op="vol-propose", rule_id=req.volume.rule_id,
                     accepted=True, staged_epoch=self.staged_vol_epoch,
                     staged_ids=sorted(self.staged_vol))
        resp.accepted, resp.reason, resp.staged_epoch = (
            True, "staged", self.staged_vol_epoch)
        return resp

    def _vol_crash_hook(self, point, txn_id="", epoch=0):
        # Deterministic crash injection for the ProtoCrash matrices
        # (2D RuleSpec and 3D volume paths share this hook: the env-gated
        # points pre-wal/post-wal/post-dispatch are dimension-independent).
        # Env RULESYNC_MGR_CRASH_AT in {pre-wal, post-wal, post-dispatch}.
        # Default OFF (unset/empty): zero behavior change. os._exit skips
        # response and cleanup by design (it IS the crash under test).
        import os as _os
        at = str(_os.environ.get("RULESYNC_MGR_CRASH_AT", "") or "")
        if at and at == str(point):
            self.log_txn(op="crash-hook", point=str(point),
                         txn_id=str(txn_id), epoch=int(epoch))
            try:
                import sys as _sys
                _sys.stdout.flush()
            except Exception:
                pass
            _os._exit(1)

    def _vol_begin_txn(self, target, epoch):
        self._vol_txn_n = int(getattr(self, "_vol_txn_n", 0)) + 1
        txn_id = f"voltx-e{int(epoch)}-n{int(self._vol_txn_n)}"
        target_hash = _vol_snapshot_hash(list(target.values()))
        op_class = _vol_op_class(self.active_vol, target)
        return txn_id, target_hash, op_class

    def on_commit_volume(self, req, resp):
        from rulesync_interfaces.msg import RuleVolume as _RV
        _ = _RV  # keep linters calm about the additive import
        epoch = int(req.epoch)
        if not self._vol_gates_open():
            resp.committed, resp.reason = False, "degraded: reconciling"
            return resp
        if epoch != int(self.staged_vol_epoch) or not self.staged_vol:
            resp.committed, resp.reason = False, "nothing staged for epoch"
            return resp
        timeout = float(self.get_parameter("prepare_timeout_s").value)
        # COMPLETE target snapshot (I2 admits against this exact set).
        target = {**self.active_vol, **self.staged_vol}
        txn_id, target_hash, op_class = self._vol_begin_txn(target,
                                                            epoch)
        votes = self._volume_round(list(target.values()), epoch,
                                   "prepare", timeout, txn_id, target_hash)
        if not (votes and all(v == READY for v in votes.values())):
            self._volume_round([], epoch, "abort", timeout, txn_id,
                               target_hash)
            self._wal_write({"kind": "vol-decision", "txn_id": txn_id,
                             "epoch": epoch, "target_hash": target_hash,
                             "op_class": op_class, "decision": "ABORT",
                             "target_volumes": [
                                 self._vol_to_dict(v) for _, v in
                                 sorted(target.items())],
                             "votes": votes})
            self.log_txn(op="vol-abort", epoch=epoch, votes=votes,
                         committed=False, txn_id=txn_id,
                         target_hash=target_hash, op_class=op_class)
            resp.committed, resp.reason = False, f"aborted: {votes}"
            return resp
        self._vol_crash_hook("pre-wal", txn_id, epoch)  # C0 boundary
        # DURABLE COMMIT DECISION — LINEARIZATION POINT. No participant
        # may apply a relaxation/remove before this record exists.
        self._wal_write({"kind": "vol-decision", "txn_id": txn_id,
                         "epoch": epoch, "target_hash": target_hash,
                         "op_class": op_class, "decision": "COMMIT",
                         "target_volumes": [self._vol_to_dict(v)
                                            for _, v in
                                            sorted(target.items())],
                         "votes": votes})
        self._vol_crash_hook("post-wal", txn_id, epoch)  # C1 boundary
        # COMMIT CERTIFICATE dispatch (txn_id/epoch/hash); bridge applies
        # only on exact prepared-identity match; ACKs converge it.
        c_votes = self._volume_round(list(target.values()), epoch,
                                     "commit", timeout, txn_id,
                                     target_hash)
        self._vol_crash_hook("post-dispatch", txn_id, epoch)  # C3 boundary
        # Checkpoint AFTER ACK processing (C3 = crash here loses only the
        # checkpoint; WAL replay rebuilds it — never the decision).
        self.active_vol = {k: v for k, v in target.items()}
        self.committed_epoch = epoch
        self.staged_vol, self.staged_vol_epoch = {}, 0
        self._save_state()
        c_ok = bool(c_votes) and all(v == READY
                                     for v in c_votes.values())
        if c_ok:
            # Explicit convergence (ACKed exact certificate): clear.
            self._vol_degraded = False
            self._volume_authority_ready(epoch, target_hash)
            self.log_txn(op="vol-converge-explicit", epoch=epoch,
                         txn_id=txn_id, target_hash=target_hash)
        else:
            # Decision stands (durable); participant uncertain -> DEGRADED
            # until reconcile replay converges. Never silently proceed.
            self._vol_degraded = True
            self.log_txn(op="vol-commit-uncertain", epoch=epoch,
                         txn_id=txn_id, target_hash=target_hash,
                         commit_votes=c_votes)
        self.log_txn(op="vol-commit", epoch=epoch, votes=votes,
                     committed=True, txn_id=txn_id,
                     target_hash=target_hash, op_class=op_class)
        resp.committed, resp.reason = True, "committed"
        return resp

    def on_remove_volume(self, req, resp):
        if not self._vol_gates_open():
            resp.ok, resp.reason = False, "degraded: reconciling"
            return resp
        if int(req.epoch) <= int(self.committed_epoch):
            resp.ok, resp.reason = False, "stale epoch"
            return resp
        if req.rule_id not in self.active_vol:
            resp.ok, resp.reason = False, "unknown rule_id"
            return resp
        remain = {k: v for k, v in self.active_vol.items()
                  if k != req.rule_id}
        timeout = float(self.get_parameter("prepare_timeout_s").value)
        txn_id, target_hash, op_class = self._vol_begin_txn(
            remain, int(req.epoch))
        votes = self._volume_round(list(remain.values()), int(req.epoch),
                                   "prepare", timeout, txn_id, target_hash)
        if not (votes and all(v == READY for v in votes.values())):
            self._volume_round([], int(req.epoch), "abort", timeout,
                               txn_id, target_hash)
            self._wal_write({"kind": "vol-decision", "txn_id": txn_id,
                             "epoch": int(req.epoch),
                             "target_hash": target_hash,
                             "op_class": op_class, "decision": "ABORT",
                             "target_volumes": [
                                 self._vol_to_dict(v) for _, v in
                                 sorted(remain.items())],
                             "votes": votes})
            self.log_txn(op="vol-abort-remove", rule_id=req.rule_id,
                         epoch=int(req.epoch), votes=votes, committed=False,
                         txn_id=txn_id, target_hash=target_hash,
                         op_class=op_class)
            resp.ok, resp.reason = False, f"aborted: {votes}"
            return resp
        self._vol_crash_hook("pre-wal", txn_id, int(req.epoch))
        self._wal_write({"kind": "vol-decision", "txn_id": txn_id,
                         "epoch": int(req.epoch),
                         "target_hash": target_hash, "op_class": op_class,
                         "decision": "COMMIT",
                         "target_volumes": [self._vol_to_dict(v)
                                            for _, v in
                                            sorted(remain.items())],
                         "votes": votes})
        self._vol_crash_hook("post-wal", txn_id, int(req.epoch))
        c_votes = self._volume_round(list(remain.values()),
                                     int(req.epoch), "commit", timeout,
                                     txn_id, target_hash)
        self._vol_crash_hook("post-dispatch", txn_id, int(req.epoch))
        self.active_vol = {k: v for k, v in remain.items()}
        self.committed_epoch = int(req.epoch)
        self._save_state()
        if bool(c_votes) and all(v == READY
                                 for v in c_votes.values()):
            self._vol_degraded = False
            self._volume_authority_ready(int(req.epoch), target_hash)
            self.log_txn(op="vol-converge-explicit",
                         epoch=int(req.epoch), txn_id=txn_id,
                         target_hash=target_hash)
        else:
            self._vol_degraded = True
            self.log_txn(op="vol-commit-uncertain", epoch=int(req.epoch),
                         txn_id=txn_id, target_hash=target_hash,
                         commit_votes=c_votes)
        self.log_txn(op="vol-commit-remove", rule_id=req.rule_id,
                     epoch=int(req.epoch), votes=votes, committed=True,
                     txn_id=txn_id, target_hash=target_hash,
                     op_class=op_class)
        resp.ok, resp.reason = True, "removed"
        return resp

    def on_list_volumes(self, req, resp):
        resp.active = list(self.active_vol.values())
        resp.staged = list(self.staged_vol.values())
        resp.epoch = int(self.committed_epoch)
        resp.degraded = bool(getattr(self, "_vol_degraded", True))
        resp.active_hash = _vol_snapshot_hash(
            list(self.active_vol.values()))
        return resp

    # ---------- 2PC machinery ----------
    def _ensure_vote_sub(self, part: str) -> None:
        if part in self._vote_subs:
            return

        import threading as _th2
        self.get_logger().info(
            f"sub-created thread={_th2.get_ident()} active={_th2.active_count()}")
        def _cb(msg, part=part):
            self._votes[part] = {"vote": int(msg.vote),
                                 "reason": str(msg.reason),
                                 "epoch": int(msg.epoch),
                                 "op": str(msg.op),
                                 "t": time.time()}
            self.get_logger().info(
                f"cb thread={_th2.get_ident()} active={_th2.active_count()}")
            self.get_logger().info(
                f"vote rx {part}={msg.vote} ep={msg.epoch} op={msg.op}")

        self._vote_subs[part] = self.create_subscription(
            BridgeVoteMsg, f"/rulesync/votes/{part}", _cb, _QOS)

    def _run_round(self, rules: list, epoch: int, op: str,
                   timeout: float, txn_id: str = "",
                   target_hash: str = "") -> dict:
        fut = self._pool.submit(self._round_body, rules, epoch, op,
                                timeout, txn_id, target_hash)
        try:
            return fut.result(timeout=timeout + 70.0)
        except Exception as e:
            self.get_logger().warning(f"round worker failed: {e}")
            return {}

    def _round_body(self, rules: list, epoch: int, op: str,
                    timeout: float, txn_id: str = "",
                    target_hash: str = "") -> dict:
        """Dispatch: persistent coordinator (default) or legacy helper
        subprocesses (use_legacy_vote_helper=True, A/B only). The flag
        is read fresh every round so A/B needs no restart."""
        try:
            use_legacy = bool(self.get_parameter(
                "use_legacy_vote_helper").value)
        except Exception:
            use_legacy = False
        if use_legacy:
            return self._legacy_round_body(rules, epoch, op, timeout,
                                           txn_id, target_hash)
        rd = [self._rule_round_dict(r) for r in rules]
        res = self._coord.round(rd, int(epoch), str(op), float(timeout),
                                str(txn_id), str(target_hash))
        votes = {}
        lat = {}
        for part, (vote, reason, dt) in res.items():
            votes[part] = vote
            lat[part] = round(dt * 1000.0, 1)
            self.get_logger().info(
                f"persistent vote {part}={vote} {reason} op={op} "
                f"ep={epoch} dt={dt * 1000.0:.0f}ms")
        self.log_txn(op=op, epoch=int(epoch), votes=votes,
                     t_round_ms=round(
                         max(lat.values()) if lat else 0.0, 1),
                     per_participant_ms=lat, path="persistent")
        return votes

    def _rule_round_dict(self, r):
        # Complete RuleSpec transport dict: every authorization-relevant
        # field must round-trip (the snapshot hash covers polygon z and
        # the validity window — dropping them here would break
        # manager/bridge hash agreement, §8).
        try:
            vf, vu = r.valid_from, r.valid_until
            vff = [int(vf.sec), int(vf.nanosec)]
            vuu = [int(vu.sec), int(vu.nanosec)]
        except Exception:
            vff, vuu = [0, 0], [3600, 0]
        try:
            pts = [[float(p.x), float(p.y), float(p.z)]
                   for p in r.polygon.points]
        except Exception:
            pts = []
        return {"rule_id": r.rule_id, "epoch": int(r.epoch),
                "rule_type": int(r.rule_type),
                "severity": int(getattr(r, "severity", 1)),
                "frame_id": str(getattr(r, "frame_id", "map")),
                "points": pts,
                "speed_limit": float(getattr(r, "speed_limit", 0.0)),
                "valid_from": vff, "valid_until": vuu}

    def _legacy_round_body(self, rules: list, epoch: int, op: str,
                           timeout: float, txn_id: str = "",
                           target_hash: str = "") -> dict:
        """Vote collection via short-lived helper subprocesses.

        Rationale (documented): in-process vote collection (service clients
        and topic subscriptions alike) proved unreliable from this
        long-lived process in this environment, while fresh short-lived
        processes succeed deterministically. Each participant vote goes
        through vote_once.py (own process, own spin, exits after).
        """
        import json as _json
        import subprocess as _sp
        import sys as _sys
        parts = list(self.get_parameter("participants").value)
        rd = [self._rule_round_dict(r) for r in rules]
        helper = "/home/yule/rulesync_ws/vote_once.py"
        sitepkgs = "/home/yule/rulesync_ws/install/rulesync_interfaces" \
            "/lib/python3.14/site-packages"
        votes: dict[str, int] = {}
        for part in parts:
            svc = f"/rulesync/{part}_vote"
            cmd = (f"source /opt/ros/lyrical/setup.bash && "
                   f"source /home/yule/rulesync_ws/install/setup.bash && "
                   f"exec python3 {helper} {svc} {int(epoch)} {op} "
                   f"'{_json.dumps(rd)}' {txn_id} {target_hash}")
            try:
                cp = _sp.run(
                    ["bash", "-c", cmd], capture_output=True, text=True,
                    timeout=min(timeout, 50.0),
                    env={"PATH": "/usr/bin:/bin",
                         "HOME": "/home/yule",
                         "ROS_DISCOVERY_SERVER": "127.0.0.1:11811",
                         "ROS_DISABLE_ROS2_DAEMON": "1"})
                vote, reason = TIMEOUT, "helper-failed"
                for line in cp.stdout.splitlines():
                    if line.startswith("VOTE="):
                        chunks = line.split()
                        vote = int(chunks[0].split("=")[1])
                        reason = chunks[1].split("=", 1)[1]                             if len(chunks) > 1 else ""
                votes[part] = vote
                self.get_logger().info(
                    f"helper vote {part}={vote} {reason} op={op} ep={epoch}")
            except Exception as e:
                self.get_logger().warning(f"helper vote failed: {e}")
                votes[part] = TIMEOUT
        self.log_txn(op=op, epoch=int(epoch), votes=votes, path="legacy")
        return votes


def main():
    import rclpy.executors
    rclpy.init()
    node = RuleManager()
    ex = rclpy.executors.MultiThreadedExecutor()
    try:
        ex.add_node(node)
        ex.spin()
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
