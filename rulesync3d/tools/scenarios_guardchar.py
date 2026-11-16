# GuardChar-32 characterization scenarios (READ manifest first).
# Surface A (12): open float-graze geometry, speeds x delays x arms B/C.
# Sweeps B (12): one factor at a time, arm C.
# Mechanism C (8 NEW): nominal/graze/strong_lat/strong_vert/stale/missing
#   x arms where missing (A-arm + prior C rows reused as documented).
import json

EPOCH0 = 170000
STRIDE = 100
V0 = {"type": 0, "center": [10, 11.3, 1.2], "dims": [4, 1.2, 1.6]}
START, GOAL = [1, 10, 1], [19, 10, 1]

CASES = []
_ep = [EPOCH0]


def add(uid, arm, **kw):
    c = {"uid": uid, "map": "open_3d", "start": list(START),
         "goal": list(GOAL), "volumes": [dict(V0)],
         "volume": dict(V0), "planner": "astar", "epoch": _ep[0],
         "arm": arm}
    c.update(kw)
    CASES.append(c)
    _ep[0] += STRIDE


# A. surface: speed x delay x arm
for v in (0.75, 1.5):
    for d in (0, 100, 300):
        for arm in ("gate_geomon", "gate_guard"):
            tag = f"v{str(v).replace('.', 'p')}-d{d}"
            add(f"surf-{tag}-{arm[-7:]}", arm, vmax_xy=v, vmax_z=1.0,
                guard_delay_ms=d)
# B. sweeps around nominal (arm C)
for me in (1, 5, 20):
    add(f"sweep-mon{me}", "gate_guard", monitor_every=me)
for ns in (0.0, 0.1, 0.3):
    add(f"sweep-noise{str(ns).replace('.', 'p')}", "gate_guard",
        guard_noise=ns)
for wm in (0.3, 0.5, 0.8):
    add(f"sweep-warn{str(wm).replace('.', 'p')}", "gate_guard",
        warn_margin=wm)
for rh in (0.1, 0.2, 0.4):
    add(f"sweep-rel{str(rh).replace('.', 'p')}", "gate_guard",
        release_hyst=rh)
# C. mechanism deltas (A + prior C reused; these 9 are NEW)
add("mech-nominal-B", "gate_geomon")
add("mech-graze-B", "gate_geomon")
add("mech-stronglat-B", "gate_geomon", disturb="lat:0.8")
add("mech-strongvert-B", "gate_geomon", disturb="vert:0.8")
add("mech-missing-A", "gate")
add("mech-missing-B", "gate_geomon", guard_missing=True)
add("mech-missing-C", "gate_guard", guard_missing=True)
add("mech-stale-B", "gate_geomon", guard_stale=True)

print(f"guardchar: {len(CASES)} cases")
with open("/tmp/cases_guardchar.json", "w") as f:
    json.dump(CASES, f, indent=1)
