"""Durable 3D volume-authority envelope for the B1 quadrotor DEV runner.

Scope and honest boundary
-------------------------
The 2D RuleSync execution stack commits polygon rules through a manager process
that (a) durably logs a PREPARE -> independent-vote -> COMMIT transaction with
fsync, (b) allocates a monotonically increasing *epoch* per committed snapshot,
and (c) publishes the committed (epoch, snapshot hash).  The frozen 2D runner
is read-only here and is NOT reused for 3D.

This module implements the same envelope for 3D AABB KEEP_OUT volumes inside
the DEV runner process:

  PREPARE  staged volume set + txn id (nothing executable yet)
  VOTE     one independent feasibility vote (exactly one voter in DEV, the
           "bridge" role); a failed vote makes COMMIT fail closed
  COMMIT   WAL record fsynced, then epoch += 1 and a canonical snapshot hash
           over every committed volume, then an atomically replaced state
           snapshot (also fsynced)

The identity/token predicates themselves are reused verbatim from the installed
``rulesync3d.authorization`` and snapshot canonicalisation from
``rulesync3d.ruleguard``; the launcher preflight sha256-pins that module build.
Crash recovery replays the WAL and compares it with the state snapshot, which is
what makes "durable" testable rather than a claim.

This is a DEV integration envelope, not the production manager: it is
single-process, has a single voter, and does not implement the ROS service
surface.  The 130-trial line must use the manager service path; see the report.
"""
import hashlib
import json
import math
import os
import time
from pathlib import Path

from rulesync3d import authorization as auth
from rulesync3d import ruleguard as rg

SCHEMA = "rulesync.quad3d.authority.v1"


class AuthorityError(RuntimeError):
    pass


class CommitRefused(AuthorityError):
    pass


def _canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False)


def _now_ns():
    return time.monotonic_ns()


def _iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def _fsync_write(path, payload):
    """Write payload atomically and fsync file + directory entry."""
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    tmp.replace(path)
    directory = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


class DurableVolumeAuthority:
    """PREPARE/VOTE/COMMIT volume authority with a replayable WAL.

    Volumes are plain dicts ``{"id", "geometry_type": 0, "bmin": [3],
    "bmax": [3], ...extra declared fields}``.  Only authorization-relevant
    fields participate in the canonical snapshot hash via
    ``rulesync3d.ruleguard.canon_volume``; provenance fields (source polygon,
    declared band label) do not, exactly like the 2D rule hash.
    """

    def __init__(self, root, volumes=()):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.wal_path = self.root / "wal.jsonl"
        self.state_path = self.root / "state.json"
        self.epoch = 0
        self.volumes = [dict(v) for v in volumes]
        self.snapshot_hash = self._hash(self.volumes)
        self._pending = None
        self._wal_count = 0
        self._wal_offset = 0

    # -- hashing ---------------------------------------------------------
    @staticmethod
    def _hash(volumes):
        return rg.snapshot_hash(rg.volume_snapshot_items(volumes))

    def gate_volumes(self, inflation):
        """Committed volumes as 3D gate dicts inflated by ``inflation`` m."""
        out = []
        for v in self.volumes:
            bmin = [float(x) - float(inflation) for x in v["bmin"]]
            bmax = [float(x) + float(inflation) for x in v["bmax"]]
            out.append({"geometry_type": 0, "bmin": bmin, "bmax": bmax})
        return out

    def snapshot(self):
        return {"epoch": self.epoch, "snapshot_hash": self.snapshot_hash,
                "volumes": json.loads(_canonical(self.volumes))}

    # -- WAL -------------------------------------------------------------
    def _append(self, record):
        record = dict(record)
        record.setdefault("seq", self._wal_count + 1)
        record.setdefault("receipt_ns", _now_ns())
        record.setdefault("wall_time", _iso())
        line = _canonical(record) + "\n"
        before = self.wal_path.stat().st_size if self.wal_path.exists() else 0
        with self.wal_path.open("a") as stream:
            stream.write(line)
            stream.flush()
            os.fsync(stream.fileno())
        self._wal_count = int(record["seq"])
        self._wal_offset = before + len(line.encode())
        record["wal_offset"] = before
        return record

    def _write_state(self):
        payload = {
            "schema": SCHEMA,
            "epoch": int(self.epoch),
            "snapshot_hash": self.snapshot_hash,
            "volumes": self.volumes,
            "wal_records": int(self._wal_count),
            "wal_offset": int(self._wal_offset),
            "updated_ns": _now_ns(),
            "updated_wall_time": _iso(),
        }
        _fsync_write(self.state_path, json.dumps(payload, indent=2,
                                                 sort_keys=True) + "\n")

    # -- transaction -----------------------------------------------------
    def propose(self, volumes, op="COMMIT", note=""):
        if self._pending is not None:
            raise AuthorityError("a transaction is already pending")
        volumes = [dict(v) for v in volumes]
        for volume in volumes:
            if int(volume.get("geometry_type", 0)) != 0:
                raise AuthorityError("DEV authority accepts AABB volumes only")
            if len(volume.get("bmin", [])) != 3 or len(volume.get("bmax", [])) != 3:
                raise AuthorityError("volume needs 3D bmin/bmax")
            if any(float(volume["bmax"][i]) <= float(volume["bmin"][i])
                   for i in range(3)):
                raise AuthorityError("volume has empty extent")
        epoch = self.epoch + 1
        digest = hashlib.sha256(
            _canonical({"epoch": epoch, "volumes": volumes}).encode()
        ).hexdigest()[:24]
        txn = {"txn_id": f"txn-{epoch:06d}-{digest}", "epoch": epoch,
               "op": op, "volumes": volumes, "note": note,
               "vote": None, "base_epoch": self.epoch,
               "base_snapshot_hash": self.snapshot_hash}
        record = self._append({"op": "PREPARE", "txn_id": txn["txn_id"],
                               "txn_epoch": epoch, "txn_op": op,
                               "base_epoch": self.epoch,
                               "base_snapshot_hash": self.snapshot_hash,
                               "staged_volumes": volumes})
        txn["prepare_record"] = record
        self._pending = txn
        return dict(txn)

    def vote(self, txn_id, voter, evidence=None):
        """One independent feasibility vote. ``voter(volumes, evidence)``
        returns ``(passed: bool, reason: str)``; exactly one voter in DEV."""
        txn = self._require(txn_id)
        if txn["vote"] is not None:
            raise AuthorityError("transaction already voted")
        passed, reason = voter(txn["volumes"], evidence)
        record = self._append({"op": "VOTE", "txn_id": txn["txn_id"],
                               "txn_epoch": txn["epoch"],
                               "voter": "quad3d_bridge_feasibility_v1",
                               "vote": "PASS" if passed else "FAIL",
                               "reason": str(reason)})
        txn["vote"] = {"passed": bool(passed), "reason": str(reason),
                       "record": record}
        return dict(txn["vote"])

    def commit(self, txn_id):
        txn = self._require(txn_id)
        if txn["vote"] is None or not txn["vote"]["passed"]:
            # fail closed: no commit without a passing independent vote
            self._append({"op": "ABORT", "txn_id": txn["txn_id"],
                          "txn_epoch": txn["epoch"],
                          "reason": "vote-not-passed"})
            self._pending = None
            raise CommitRefused("commit refused: independent vote did not pass")
        if txn["base_epoch"] != self.epoch:
            self._append({"op": "ABORT", "txn_id": txn["txn_id"],
                          "txn_epoch": txn["epoch"],
                          "reason": "epoch-advanced-during-txn"})
            self._pending = None
            raise CommitRefused("commit refused: epoch advanced during prepare")
        volumes = txn["volumes"]
        snapshot_hash = self._hash(volumes)
        record = self._append({"op": "COMMIT", "txn_id": txn["txn_id"],
                               "txn_epoch": txn["epoch"],
                               "epoch": txn["epoch"],
                               "snapshot_hash": snapshot_hash,
                               "volume_ids": [str(v.get("id", "?")) for v in volumes],
                               "txn_op": txn["op"]})
        self.epoch = int(txn["epoch"])
        self.volumes = volumes
        self.snapshot_hash = snapshot_hash
        self._write_state()
        txn["commit_record"] = record
        self._pending = None
        return {"txn_id": txn["txn_id"], "epoch": self.epoch,
                "snapshot_hash": self.snapshot_hash,
                "volumes": json.loads(_canonical(volumes)),
                "wal_offset": record["wal_offset"],
                "receipt_ns": record["receipt_ns"]}

    def abort(self, txn_id, reason="caller-abort"):
        txn = self._require(txn_id)
        record = self._append({"op": "ABORT", "txn_id": txn["txn_id"],
                               "txn_epoch": txn["epoch"], "reason": reason})
        self._pending = None
        return record

    def _require(self, txn_id):
        if self._pending is None or self._pending["txn_id"] != txn_id:
            raise AuthorityError(f"no pending transaction {txn_id!r}")
        return self._pending

    # -- identity / tokens ----------------------------------------------
    def issue_token(self, path_digest):
        return auth.token_id(self.epoch, self.snapshot_hash, path_digest)

    def verify_token(self, path_digest, token, epoch, snapshot_hash):
        return _verify_digest(path_digest, token, epoch, snapshot_hash)

    # -- recovery --------------------------------------------------------
    def recover(self):
        """Replay the WAL, compare it with the state snapshot, and continue.

        Returns a JSON-able record.  A WAL newer than the state snapshot is
        accepted (the state write is the last, non-atomic step of a commit);
        a state snapshot that is not reproduced by the WAL is an error.
        """
        result = {"schema": SCHEMA, "wal_path": str(self.wal_path),
                  "state_path": str(self.state_path),
                  "wal_present": self.wal_path.exists(),
                  "state_present": self.state_path.exists(),
                  "records": 0, "commits": [], "replayed_epoch": 0,
                  "replayed_snapshot_hash": self._hash([]),
                  "state_matches_replay": True, "genesis_written": False}
        if not self.wal_path.exists():
            record = self._append({"op": "GENESIS", "epoch": 0,
                                   "snapshot_hash": self._hash([]),
                                   "volumes": []})
            result["genesis_written"] = True
            result["records"] = 1
            self._write_state()
            result["state_matches_replay"] = True
            return result
        epoch, volumes, snapshot = 0, [], self._hash([])
        commits = []
        for line in self.wal_path.read_text().splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            result["records"] += 1
            self._wal_count = max(self._wal_count, int(record.get("seq", 0)))
            if record.get("op") == "COMMIT":
                epoch = int(record["epoch"])
                volumes = record.get("volumes", volumes)
                snapshot = str(record["snapshot_hash"])
                commits.append({"txn_id": record.get("txn_id"),
                                "epoch": epoch,
                                "snapshot_hash": snapshot,
                                "seq": record.get("seq"),
                                "wal_offset": record.get("wal_offset")})
        self._wal_offset = self.wal_path.stat().st_size
        self.epoch, self.volumes, self.snapshot_hash = epoch, volumes, snapshot
        result["commits"] = commits
        result["replayed_epoch"] = epoch
        result["replayed_snapshot_hash"] = snapshot
        if self.state_path.exists():
            state = json.loads(self.state_path.read_text())
            result["state_epoch"] = state.get("epoch")
            result["state_snapshot_hash"] = state.get("snapshot_hash")
            if (int(state.get("epoch", -1)) != epoch
                    or str(state.get("snapshot_hash")) != snapshot):
                result["state_matches_replay"] = False
        return result


def _verify_digest(path_digest, token, epoch, snapshot_hash):
    """Content-addressed check for an already-computed path digest."""
    expected = auth.token_id(epoch, snapshot_hash, path_digest)
    if str(expected) != str(token):
        return False, "authorization-token-mismatch", path_digest
    return True, "ok", path_digest


def sha256_file(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()
