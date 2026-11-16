# OMPL RRTConnect live-arrival smoke (Stage 1, 10 cases).
# Genuine libompl 2.0.2 (Python bindings) as UNTRUSTED proposer; every
# candidate gated (+guard where armed) before execution.
import json

EPOCH0 = 160000
STRIDE = 100


def AABB(cx, cy, cz, dx, dy, dz):
    return {"type": 0, "center": [cx, cy, cz], "dims": [dx, dy, dz]}


def SPHERE(cx, cy, cz, r):
    return {"type": 1, "center": [cx, cy, cz], "dims": [r, 0, 0]}


CASES = []
_ep = [EPOCH0]


def add(uid, map_, start, goal, volumes, arm="gate_guard"):
    CASES.append({"uid": uid, "map": map_, "start": list(start),
                  "goal": list(goal), "volumes": volumes,
                  "volume": (volumes[0] if volumes else None),
                  "planner": "ompl", "epoch": _ep[0], "arm": arm})
    _ep[0] += STRIDE


add("ompl-open-base", "open_3d", (1, 10, 1), (19, 10, 1), [])
add("ompl-open-float", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 1.5, 3, 3, 2)])
add("ompl-open-sphere", "open_3d", (1, 10, 1), (19, 10, 1),
    [SPHERE(10, 10, 1.2, 1.2)])
add("ompl-open-multi", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(7, 10, 1.5, 2, 4, 2), SPHERE(13, 10, 1.5, 1.2)])
add("ompl-barrier-base", "floating_barrier", (1, 10, 1), (19, 10, 1), [])
add("ompl-barrier-climb", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 4.5, 4, 4, 1.5)])
add("ompl-barrier-sphere", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [SPHERE(10, 10, 4.2, 1.0)])
add("ompl-shaft-base", "vertical_corridor", (10, 10, 1), (10, 1, 5), [])
add("ompl-levels-switch", "multi_level", (1, 10, 3), (19, 10, 1),
    [AABB(10, 10, 2.5, 3, 6, 2.0)])
add("ompl-levels-multi", "multi_level", (1, 10, 3), (19, 10, 1),
    [AABB(10, 10, 2.5, 3, 6, 2.0), SPHERE(5, 10, 3.5, 1.0),
     AABB(15, 10, 1.0, 3, 5, 1.5)])

print(f"ompl smoke: {len(CASES)} cases")
with open("/tmp/cases_ompl.json", "w") as f:
    json.dump(CASES, f, indent=1)
