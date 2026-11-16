# RuleGuard deviation experiment: paired PATH_GATE_ONLY (arm=gate) vs
# PATH_GATE+RULEGUARD (arm=gate_guard) on identical cases.
# World: open_3d (flat, occupancy-safe for small deviations; isolates
# rule-entry from wall contact). Straight mission (1,10,1)->(19,10,1)
# with a floating KEEP_OUT volume north of the path; deviations push the
# TRACK toward/into it while the gate-approved PATH stays clean.
# Volume V0 (near): AABB(10,11.3,1.2, dims 4,1.2,1.6): inflated y-face
#   10.3 -> nominal clearance 0.3 (WARNING band, arrives clean).
# Volume V1 (far): AABB(10,12.0,1.2, dims 4,1.2,1.6): inflated y-face
#   11.0 -> nominal clearance 1.0 (CLEAR).
# Metrics per arm: actual rule entries, warnings, HOLDs, false HOLDs,
# min clearance, goal completion, recovery.
import json

EPOCH0 = 130000
STRIDE = 100


def AABB(cx, cy, cz, dx, dy, dz):
    return {"type": 0, "center": [cx, cy, cz], "dims": [dx, dy, dz]}


V0 = AABB(10, 11.3, 1.2, 4, 1.2, 1.6)
V1 = AABB(10, 12.0, 1.2, 4, 1.2, 1.6)
# Overhead volume for vertical-deviation cases: z 2.2-3.4 (inflated
# 1.8-3.8); path z=1.2 -> clearance 0.6 (CLEAR); vert:+0.8 pushes in.
V2 = AABB(10, 10, 2.8, 4, 4, 1.2)

CASES = []
_ep = [EPOCH0]


def add(uid, volumes, planner, arm, disturb="none", guard_stale=False):
    CASES.append({"uid": uid, "map": "open_3d",
                  "start": [1, 10, 1], "goal": [19, 10, 1],
                  "volumes": volumes,
                  "volume": (volumes[0] if volumes else None),
                  "planner": planner, "epoch": _ep[0],
                  "arm": arm, "disturb": disturb,
                  "guard_stale": guard_stale})
    _ep[0] += STRIDE


for arm in ("gate", "gate_guard"):
    tag = "A" if arm == "gate" else "B"
    add(f"guard-nominal-{tag}", [V1], "astar", arm)
    add(f"guard-graze-{tag}", [V0], "astar", arm)
    add(f"guard-lat-{tag}", [V0], "astar", arm, disturb="lat:0.8")
    add(f"guard-vert-{tag}", [V2], "astar", arm, disturb="vert:0.8")
    add(f"guard-noise-{tag}", [V0], "astar", arm, disturb="noise:0.4")
    add(f"guard-lag-{tag}", [V0], "astar", arm, disturb="lag:4")
    add(f"guard-stale-{tag}", [V1], "astar", arm, guard_stale=True)

print(f"ruleguard: {len(CASES)} cases")
with open("/tmp/cases_guard.json", "w") as f:
    json.dump(CASES, f, indent=1)
