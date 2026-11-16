# WAIT_READY three-arm baseline (paired with frozen geometry).
# Arms: native (trust stale) / wait (ready-ack + trust fresh) /
# rulesync (gate + guard). Fault 'bad_candidate' substitutes a violating
# straight candidate AFTER readiness: WAIT trusts it (violation expected),
# RULESYNC blocks it. Stale pre-commit candidates are the
# delayed-propagation analog (downstream state predates the commit).
# Epoch bases assigned by driver; run with --epoch-base 150000+.
import json

EPOCH0 = 150000
STRIDE = 100


def AABB(cx, cy, cz, dx, dy, dz):
    return {"type": 0, "center": [cx, cy, cz], "dims": [dx, dy, dz]}


OPEN_VOL = AABB(10, 10, 1.5, 3, 3, 2)      # floats across straight path
CLIMB_VOL = AABB(10, 10, 4.5, 4, 4, 1.5)   # barrier climb corridor

CASES = []
_ep = [EPOCH0]


def add(uid, map_, start, goal, volumes, planner, arm3, fault="none"):
    CASES.append({"uid": uid, "map": map_, "start": list(start),
                  "goal": list(goal), "volumes": volumes,
                  "volume": (volumes[0] if volumes else None),
                  "planner": planner, "epoch": _ep[0],
                  "arm": ("gate_guard" if arm3 == "rulesync" else "gate"),
                  "arm3": arm3, "fault": fault})
    _ep[0] += STRIDE


# healthy propagation: constraining volumes, all three arms
for arm3 in ("native", "wait", "rulesync"):
    add(f"open-3arm-{arm3}", "open_3d", (1, 10, 1), (19, 10, 1),
        [OPEN_VOL], "astar", arm3)
    add(f"climb-3arm-{arm3}", "floating_barrier", (1, 10, 1), (19, 10, 1),
        [CLIMB_VOL], "astar", arm3)
# invalid candidate after readiness: WAIT trusts (violation), RULESYNC blocks
for arm3 in ("wait", "rulesync"):
    add(f"open-fault-{arm3}", "open_3d", (1, 10, 1), (19, 10, 1),
        [OPEN_VOL], "astar", arm3, fault="bad_candidate")
    add(f"climb-fault-{arm3}", "floating_barrier", (1, 10, 1), (19, 10, 1),
        [CLIMB_VOL], "astar", arm3, fault="bad_candidate")

print(f"wait baseline: {len(CASES)} cases")
with open("/tmp/cases_wait.json", "w") as f:
    json.dump(CASES, f, indent=1)
