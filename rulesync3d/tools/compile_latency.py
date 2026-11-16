# Compile protocol_latency.csv from archived per-cell txn logs
# (txn_all.jsonl / txn2d_all.jsonl — appended per cell by the runners;
# epochs are per-cell unique). Reports the durable-WAL overhead (§14):
# per durable decision the fsync cost (t_wal_ms), the PREPARE round time
# and the COMMIT-round time. One row per durable DECISION record
# (commit/abort with txn_id); round times joined by epoch+op.
# Usage: python3 compile_latency.py <artifacts_dir>
import csv
import json
import sys

out = sys.argv[1]
recs = []
for fn in (f"{out}/txn_all.jsonl", f"{out}/txn2d_all.jsonl"):
    try:
        for line in open(fn):
            line = line.strip()
            if line:
                try:
                    recs.append(json.loads(line))
                except Exception:
                    continue
    except Exception:
        pass

DECISIONS = {"commit", "commit-remove", "abort", "abort-remove",
             "vol-commit", "vol-commit-remove", "vol-abort",
             "vol-abort-remove"}


def is_decision(r):
    return str(r.get("op", "")) in DECISIONS and bool(r.get("txn_id"))


rows = []
for r in recs:
    if not is_decision(r):
        continue
    ep = int(r.get("epoch", -1))
    op = str(r.get("op", ""))
    # round times: 2D persistent path logs t_round_ms on prepare/commit
    # rounds; volume rounds don't carry it (recovery_ms covers e2e).
    base = "vol-" if op.startswith("vol-") else ""
    t_prep, t_commit = None, None
    for q in recs:
        if int(q.get("epoch", -2)) != ep:
            continue
        if q.get("t_round_ms") is None:
            continue
        qo = str(q.get("op", ""))
        if qo == base + "prepare" and t_prep is None:
            t_prep = q.get("t_round_ms")
        elif qo == base + "commit" and t_commit is None:
            # the COMMIT-round log doubles as the decision-adjacent
            # record; prefer a non-decision round entry when present
            if not str(q.get("txn_id", "")):
                t_commit = q.get("t_round_ms")
    if t_commit is None:
        for q in recs:
            if int(q.get("epoch", -2)) == ep and \
                    str(q.get("op", "")) == base + "commit" and \
                    q.get("t_round_ms") is not None:
                t_commit = q.get("t_round_ms")
                break
    rows.append({"epoch": ep, "txn_id": str(r.get("txn_id", "")),
                 "op": op, "op_class": str(r.get("op_class", "")),
                 "committed": bool(r.get("committed", False)),
                 "t_wal_ms": r.get("t_wal_ms"),
                 "t_prepare_ms": t_prep, "t_commit_round_ms": t_commit})

rows.sort(key=lambda d: (d["epoch"], d["txn_id"]))
with open(f"{out}/protocol_latency.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["epoch", "txn_id", "op",
                                      "op_class", "committed",
                                      "t_wal_ms", "t_prepare_ms",
                                      "t_commit_round_ms"])
    w.writeheader()
    for d in rows:
        w.writerow(d)

wals = [float(d["t_wal_ms"]) for d in rows
        if d["t_wal_ms"] is not None]
print(f"latency rows: {len(rows)} (decisions)")
if wals:
    print(f"t_wal_ms: n={len(wals)} mean={sum(wals)/len(wals):.2f} "
          f"max={max(wals):.2f}")
preps = [float(d["t_prepare_ms"]) for d in rows
         if d["t_prepare_ms"] is not None]
print(f"t_prepare_ms: n={len(preps)}" +
      (f" mean={sum(preps)/len(preps):.1f} max={max(preps):.1f}"
       if preps else ""))
commits = [float(d["t_commit_round_ms"]) for d in rows
           if d["t_commit_round_ms"] is not None]
print(f"t_commit_round_ms: n={len(commits)}" +
      (f" mean={sum(commits)/len(commits):.1f} max={max(commits):.1f}"
       if commits else ""))
