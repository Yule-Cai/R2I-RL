# ProtoCrash cell auditor: reads WAL/snapshot/services, adjudicates the
# four hard properties, appends the matrix row. No motion exists in this
# headless harness: motion_authorized = COMMIT granted for the test txn.
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rulesync_interfaces.srv import (ListVolumes, BridgeVolumeState,
                                      ProposeVolume)


def _await(n, fut, timeout=30.0):
    # Bounded service wait: spin_until_future_complete timeouts are
    # unreliable when a server died mid-call under the discovery-server
    # transport (verified driver hang). Wall-clock deadline bounds this.
    import time as _t
    t0 = _t.monotonic()
    while not fut.done() and _t.monotonic() - t0 < float(timeout):
        rclpy.spin_once(n, timeout_sec=1.0)
    return bool(fut.done())


def _res(fut):
    try:
        return fut.result() if fut.done() else None
    except Exception:
        return None


def _wait_svc(node, cli, name, attempts=5, timeout_each=30.0):
    # Retry an EXISTING client (created at node birth — see below).
    # Do NOT destroy/recreate per attempt: endpoint matching starts at
    # client creation and proceeds in the background; teardown resets it.
    # On final failure, snapshot this node's service graph into the
    # error (distinguishes client-blindness from unadvertised service).
    for _ in range(max(1, int(attempts))):
        try:
            if cli.wait_for_service(timeout_sec=float(timeout_each)):
                return cli
        except Exception:
            pass
    try:
        graph = sorted(nm for nm, _ in
                       node.get_service_names_and_types()
                       if "rulesync" in nm)
    except Exception as e:
        graph = [f"graph-failed:{e}"]
    raise RuntimeError(f"no-service:{name};graph={graph}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uid", required=True)
    ap.add_argument("--op", required=True)
    ap.add_argument("--crash", required=True)
    ap.add_argument("--restart", required=True)
    ap.add_argument("--epoch", type=int, required=True)
    ap.add_argument("--t0", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--expect", default="/tmp/proto_expect.json")
    a = ap.parse_args()
    try:
        _audit(a)
    except Exception as e:
        # Never fail the cell loop without a row: an INVALID row forces
        # the matrix gate to report NOT-CLOSED until the cell is rerun.
        import traceback
        traceback.print_exc()
        row = {"uid": a.uid, "op": a.op, "crash": a.crash,
               "restart": a.restart, "txn_id": "", "epoch": -1,
               "target_hash": "", "durable_present": False,
               "mgr_epoch": -1, "mgr_hash": "", "bridge_epoch": -1,
               "bridge_hash": "", "guard_view_consistent": False,
               "ready_state": "UNKNOWN", "motion_authorized": False,
               "recovery_ms": None, "unauthorized_motion": False,
               "silent_state_divergence": False,
               "committed_update_lost": False,
               "uncommitted_relaxation_promoted": False,
               "hook_verified": False,
               "hook_evidence": "INVALID",
               "note": f"audit-error:{e}"}
        with open(a.out, "a") as f:
            f.write(json.dumps(row) + "\n")
    finally:
        # Graceful shutdown on EVERY exit (including INVALID rows):
        # an unclean exit leaves a same-name ghost participant that
        # poisons the next cell's discovery (verified cascade).
        try:
            import rclpy as _rclpy
            if _rclpy.ok():
                _rclpy.shutdown()
        except Exception:
            pass


def _audit(a):
    exp = json.load(open(a.expect))
    wal = []
    try:
        for line in open("/tmp/rulesync_wal.jsonl"):
            line = line.strip()
            if line:
                wal.append(json.loads(line))
    except Exception:
        pass
    test_recs = [r for r in wal if r.get("kind") == "vol-decision"
                 and int(r.get("epoch", -1)) == int(
                     exp.get("test_epoch", -1)
                     if exp.get("test_epoch") is not None else -1)]
    commits = [r for r in test_recs if r.get("decision") == "COMMIT"]
    durable = commits[-1] if commits else None
    rclpy.init()
    # Unique node name per process: a constant name reused across cells
    # collides with ghost participants of previously killed audits and
    # poisons this audit's discovery (verified cascade). Precedent:
    # uav_bridge pid-suffixed names for the same reason.
    import os as _os
    n = Node(f"proto_audit_{_os.getpid()}")
    cl = n.create_client(ListVolumes, "/rulesync/list_volumes")
    cb = n.create_client(BridgeVolumeState, "/rulesync/uav_bridge_state")
    cp = n.create_client(ProposeVolume, "/rulesync/propose_volume")
    # NOTE: all clients are created UP FRONT (node age ~seconds).
    # Endpoint matching starts at creation; creating clients LATE
    # (minutes after node birth, e.g. after earlier round-trips) fails
    # systematically in a churned graph (verified: f1 works, later
    # created clients stay blind 150 s+). Never move creation later.
    cl = _wait_svc(n, cl, "/rulesync/list_volumes")
    cb = _wait_svc(n, cb, "/rulesync/uav_bridge_state")
    f1 = cl.call_async(ListVolumes.Request())
    _await(n, f1, 30.0)
    r1 = _res(f1)
    if r1 is None:
        raise RuntimeError("list-noresponse")
    m_ids = sorted(str(v.rule_id) for v in r1.active)
    m_ep = int(r1.epoch)
    try:
        m_hash = str(r1.active_hash)
    except Exception:
        m_hash = ""
    f2 = cb.call_async(BridgeVolumeState.Request())
    _await(n, f2, 30.0)
    r2 = _res(f2)
    if r2 is None:
        raise RuntimeError("bridge-state-noresponse")
    b_ids = sorted(str(v.rule_id) for v in r2.active_volumes)
    b_ep = int(r2.active_epoch)
    try:
        b_hash = str(r2.active_hash)
    except Exception:
        b_hash = ""
    # readiness probe: a fresh propose is REJECTED while DEGRADED.
    cp = _wait_svc(n, cp, "/rulesync/propose_volume")
    from rulesync_interfaces.msg import RuleVolume
    from builtin_interfaces.msg import Time
    pv = RuleVolume()
    pv.rule_id = "_probe"
    pv.epoch = int(exp.get("test_epoch", 0)
                   if exp.get("test_epoch") is not None else 0) + 900
    pv.frame_id = "map"
    pv.geometry_type = 0
    pv.center.x, pv.center.y, pv.center.z = 1.0, 10.0, 1.0
    pv.dimensions.x, pv.dimensions.y, pv.dimensions.z = 0.5, 0.5, 0.5
    t = int(time.time())
    pv.valid_from.sec, pv.valid_until.sec = t, t + 60
    f3 = cp.call_async(ProposeVolume.Request(volume=pv))
    _await(n, f3, 30.0)
    r3 = _res(f3)
    ready = bool(r3 is not None and r3.accepted)
    # guard view (offline evaluation of recorded states): all three agree?
    d_ids = sorted(str(vd["rule_id"]) for vd in
                   (durable.get("target_volumes", []) if durable else []))
    d_ep = int(durable.get("epoch", -1)) if durable else -1
    try:
        from rulesync3d.rulesync3d.ruleguard import snapshot_hash
        d_hash = durable.get("target_hash", "") if durable else ""
    except Exception:
        d_hash = ""
    # recovery_ms: first explicit convergence record after restart (WALL
    # clock from txn log), else None (never converged = stuck DEGRADED).
    g_consistent = (m_ep == b_ep and m_hash and
                    (m_hash == b_hash) and (m_ids == b_ids))
    if durable:
        g_consistent = bool(g_consistent) and (m_ep == d_ep) and \
            (m_hash == d_hash) and (m_ids == d_ids)
    rec_ms = None
    try:
        for line in open("/tmp/rulesync_txn.log"):
            try:
                r = json.loads(line)
            except Exception:
                continue
            if r.get("op") in ("vol-reconcile-ok",
                               "vol-converge-explicit",
                               "vol-state-refresh") and \
                    float(r.get("t_wall", 0)) >= float(a.t0):
                rec_ms = int((float(r["t_wall"]) - float(a.t0)) * 1000.0)
                break
    except Exception:
        pass
    motion_auth = exp.get("test_result", "").startswith("True")
    # unauthorized_motion: granted without all-READY votes + durable COMMIT.
    unauth = False
    if motion_auth:
        if not durable:
            unauth = True
        else:
            vs = durable.get("votes", {})
            if not (vs and all(int(x) == 0 for x in vs.values())):
                unauth = True
    # committed_update_lost: durable target not in manager active set.
    lost = bool(durable) and (sorted(m_ids) != sorted(d_ids))
    # uncommitted_relaxation_promoted: test target visible with NO durable.
    promoted = False
    if not durable:
        exp_ids = sorted(exp.get("target_ids", []))
        if sorted(m_ids) == exp_ids and exp_ids != sorted(
                exp.get("pre_ids", [])):
            promoted = True
        if sorted(b_ids) == exp_ids and exp_ids != sorted(
                exp.get("pre_ids", [])):
            promoted = True
    # silent divergence: states disagree but cell ended READY.
    silent = (not g_consistent) and ready
    # Hook verification: crash cells MUST show the hook fired (else the
    # cell is INVALID, not passing — a quiet run proves nothing).
    hook_ok, hook_ev = True, ""
    if a.crash in ("C0", "C1", "C3"):
        want = {"C0": "pre-wal", "C1": "post-wal",
                "C3": "post-dispatch"}[a.crash]
        try:
            wl = open("/tmp/rulesync_wal.jsonl").read()
        except Exception:
            wl = ""
        # The hook record is written via log_txn (txn log), not the WAL —
        # consult both (for C0 the WAL write never happens by design).
        try:
            tl = open("/tmp/rulesync_txn.log").read()
        except Exception:
            tl = ""
        hook_ok = (f'"point": "{want}"' in wl) or \
            (f'"point": "{want}"' in tl)
        hook_ev = f"mgr-{want}:{'fired' if hook_ok else 'MISSING'}"
    elif a.crash == "C2":
        # bridge logs nothing on _exit; C2 evidence = manager commit-round
        # TIMEOUT (uncertain) for the TEST epoch (epoch-matched: a stale
        # uncertain record from another epoch proves nothing).
        hook_ok = False
        try:
            _te = int(exp.get("test_epoch", -1)
                      if exp.get("test_epoch") is not None else -1)
            for line in open("/tmp/rulesync_txn.log"):
                try:
                    _r = json.loads(line)
                except Exception:
                    continue
                if _r.get("op") in ("vol-commit-uncertain",
                                    "commit-uncertain") and \
                        int(_r.get("epoch", -2)) == _te:
                    hook_ok = True
                    break
        except Exception:
            pass
        hook_ev = f"bridge-post-apply:{'inferred' if hook_ok else 'MISSING'}"
    row = {"uid": a.uid, "op": a.op, "crash": a.crash,
           "restart": a.restart,
           "txn_id": (durable.get("txn_id", "") if durable else ""),
           "epoch": int(exp.get("test_epoch", -1)),
           "target_hash": d_hash,
           "durable_present": bool(durable),
           "mgr_epoch": m_ep, "mgr_hash": m_hash,
           "bridge_epoch": b_ep, "bridge_hash": b_hash,
           "guard_view_consistent": bool(g_consistent),
           "ready_state": ("READY" if ready else "DEGRADED"),
           "motion_authorized": bool(motion_auth),
           "recovery_ms": rec_ms,
           "unauthorized_motion": bool(unauth),
           "silent_state_divergence": bool(silent),
           "committed_update_lost": bool(lost),
           "uncommitted_relaxation_promoted": bool(promoted),
           "hook_verified": bool(hook_ok),
           "hook_evidence": hook_ev,
           "note": f"test={exp.get('test_result')}"}
    print(f"[proto-audit] {a.uid} ready={row['ready_state']} "
          f"consistent={g_consistent} hard="
          f"{[k for k in ('unauthorized_motion', 'silent_state_divergence', 'committed_update_lost', 'uncommitted_relaxation_promoted') if row[k]] or 'clean'}",
          flush=True)
    with open(a.out, "a") as f:
        f.write(json.dumps(row) + "\n")
    n.destroy_node()
    rclpy.shutdown()


main()
