"""Small benchmark recovery policy; not a new planner or production token API.

Existing voxel A* proposes; the unchanged continuous gate independently admits.
The adapter must separately verify current authority/participant convergence,
fresh inputs and stopped motion before installing the admitted candidate.
"""
import hashlib
import json
import math
import sys

BOUNDS = (0., 8., .2, 8., 13., 6.)
GOAL_TOLERANCE = .10
WAYPOINT_TOLERANCE = .06


def backends():
    try:
        import rulesync3d.gate_3d as gate
        import rulesync3d.planners_3d as planners
        import rulesync3d
        sys.modules.setdefault("rulesync3d.rulesync3d", rulesync3d)
    except ImportError:
        from rulesync3d.rulesync3d import gate_3d as gate, planners_3d as planners
    return gate, planners


def path_digest(path):
    return hashlib.sha256(json.dumps(path, separators=(",", ":"),
        allow_nan=False).encode()).hexdigest()


def propose_detour(start, goal, volumes, rho):
    """Frozen proposer settings, with extra grid clearance for waypoint tracking.

    Extra .15 m is proposer conservatism, not a change to gate/monitor rho.
    Volume dictionaries supplied here contain the ORIGINAL AABB bounds.
    """
    gate, planners = backends()
    inflated = [dict(geometry_type=0, bmin=[x-rho for x in v["bmin"]],
        bmax=[x+rho for x in v["bmax"]]) for v in volumes]
    boxes = [tuple(v["bmin"]+v["bmax"]) for v in volumes]
    grid, origin = planners.voxelize_bounds(boxes, .20, BOUNDS, margin=rho+.15)
    raw = planners.voxel_astar(grid, origin, .20, start, goal, max_expand=100000)
    if raw is None:
        raise ValueError("existing voxel A* returned no candidate")
    # Smooth using the same extra proposer clearance; gate still uses rho only.
    conservative = [dict(geometry_type=0, bmin=[x-.15 for x in v["bmin"]],
        bmax=[x+.15 for x in v["bmax"]]) for v in inflated]
    path = planners.shortcut_smooth(raw, lambda x,y,z: False, conservative)
    allowed, reason = gate.gate_3d_path(path, inflated, lambda x,y,z: False, BOUNDS)
    if not allowed:
        raise ValueError("proposed detour denied by independent gate: "+reason)
    return path, reason


def release_checks(expected, authority, participant, ready, pose_age, snapshot_age,
                   path, position, goal, volumes, rho):
    """No pin change or HOLD clearing occurs here; failures are explicit."""
    if not ready or authority != expected or participant != expected:
        raise ValueError("release identity/convergence mismatch")
    if not 0 <= pose_age <= .25 or not 0 <= snapshot_age <= 1.:
        raise ValueError("release input freshness failed")
    if len(path) < 2 or math.dist(path[0], position) > .02 or math.dist(path[-1], goal) > 1e-9:
        raise ValueError("release candidate endpoints changed")
    gate, _ = backends()
    inflated = [dict(geometry_type=0, bmin=[x-rho for x in v["bmin"]],
        bmax=[x+rho for x in v["bmax"]]) for v in volumes]
    allowed, reason = gate.gate_3d_path(path, inflated, lambda x,y,z: False, BOUNDS)
    if not allowed:
        raise ValueError("release gate denied: "+reason)
    return reason


def audit_task(folder):
    """Reconstruct release/commands/arrival, never trust PASS as task success."""
    from pathlib import Path
    folder = Path(folder)
    events = sorted((json.loads(s) for p in folder.glob("*_events.jsonl")
        for s in p.read_text().splitlines() if s), key=lambda e:e["t_ns"])
    worlds = [json.loads(s) for s in (folder/"world_pose.jsonl").read_text().splitlines()]
    wal = [json.loads(s) for s in (folder/"wal.jsonl").read_text().splitlines()]
    commits = [e for e in events if e["event"]=="durable_commit" and e["epoch"]==2]
    releases = [e for e in events if e["event"]=="recovery_release"]
    goals = [e for e in events if e["event"]=="goal_reached"]
    holds = [e for e in events if e["event"]=="hold"]
    stopped = [e for e in events if e["event"]=="stopped_before_recovery"]
    denied = [e for e in events if e["event"]=="old_route_recheck"]
    if not (len(commits)==len(releases)==len(goals)==len(holds)==len(stopped)==len(denied)==1):
        raise ValueError("one commit/HOLD/stop/denial/release/arrival required")
    commit, release, goal, hold, stop, denial = commits[0], releases[0], goals[0], holds[0], stopped[0], denied[0]
    identity = [2, commit["snapshot_hash"]]
    decision = [w for w in wal if w.get("decision")=="COMMIT" and w.get("epoch")==2]
    if len(decision)!=1 or decision[0]["target_hash"]!=identity[1]:
        raise ValueError("release authority not grounded in durable WAL")
    if not commit["t_ns"] <= hold["t_ns"] < stop["t_ns"] < denial["t_ns"] < release["t_ns"] < goal["t_ns"]:
        raise ValueError("invalid task event ordering")
    if denial["allowed"] or any(release[k]!=identity for k in ("identity", "participant", "execution")):
        raise ValueError("unverified or mismatched recovery release")
    path = release["path"]
    if release["path_digest"]!=path_digest(path):
        raise ValueError("released path digest mismatch")
    volumes = [dict(geometry_type=0,
        bmin=[x-y/2 for x,y in zip(v["center"],v["dimensions"])],
        bmax=[x+y/2 for x,y in zip(v["center"],v["dimensions"])]) for v in decision[0]["target_volumes"]]
    release_checks(identity, release["identity"], release["participant"], release["ready"],
        release["pose_age_s"], release["snapshot_age_s"], path, release["xyz"],
        goal["goal"], volumes, release["rho"])
    commands = [e for e in events if e["event"]=="command" and max(abs(x) for x in e["velocity"])>0]
    resumed = [e for e in commands if e["t_ns"]>=release["t_ns"]]
    if any(hold["t_ns"] <= e["t_ns"] < release["t_ns"] for e in commands):
        raise ValueError("motion command before reauthorization")
    if not resumed or any(e.get("execution")!=identity or e.get("path_digest")!=release["path_digest"] for e in resumed):
        raise ValueError("resumed commands lost admitted path/identity binding")
    arrivals = [r for r in worlds if r["active"] and r["receipt_ns"]>=goal["t_ns"]]
    if len(arrivals)<5 or any(math.dist(r["xyz"],goal["goal"])>GOAL_TOLERANCE for r in arrivals):
        raise ValueError("arrival not confirmed by independent world poses")
    active = [r for r in worlds if r["active"]]
    return dict(task_completed=True, recovery_identity=identity, path_digest=release["path_digest"],
        waypoints=len(path), commit_to_release_ms=(release["t_ns"]-commit["t_ns"])/1e6,
        hold_to_release_ms=(release["t_ns"]-hold["t_ns"])/1e6,
        release_to_goal_ms=(goal["t_ns"]-release["t_ns"])/1e6,
        active_task_sim_s=(arrivals[-1]["sim_ns"]-active[0]["sim_ns"])/1e9,
        independent_final_goal_error_m=math.dist(arrivals[-1]["xyz"],goal["goal"]),
        resumed_nonzero_commands=len(resumed), arrival_samples=len(arrivals))
