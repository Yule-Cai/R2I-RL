# Stage-A 3D scenarios (20 dev): families A-D across 4 worlds.
#
# NOTE: case epochs below are PLACEHOLDERS. The driver allocates epochs in
# run order (--epoch-base + idx*100); fixed file epochs + monotonic
# committed_epoch poisoned multi-world campaigns (stale rejections).
# Manifest freezes geometry/start/goal/planner; assigned epochs are logged.
#
# Volume dict: {"type": 0/1/2, "center": [x,y,z], "dims": [dx,dy,dz]
#   (AABB full extents; SPHERE radius in dims[0]), "vertices": [...]}.
# Epochs stride 100 from 50000. Planners alternate astar/rrt.
import json

EPOCH0 = 50000
STRIDE = 100


def AABB(cx, cy, cz, dx, dy, dz):
    return {"type": 0, "center": [cx, cy, cz], "dims": [dx, dy, dz]}


def SPHERE(cx, cy, cz, r):
    return {"type": 1, "center": [cx, cy, cz], "dims": [r, 0, 0]}


CASES = []
_ep = [EPOCH0]


def add(uid, map_, start, goal, volumes, planner):
    CASES.append({"uid": uid, "map": map_, "start": list(start),
                  "goal": list(goal), "volumes": volumes,
                  "volume": (volumes[0] if volumes else None), "planner": planner,
                  "epoch": _ep[0]})
    _ep[0] += STRIDE


# open_3d: baselines + floating rule across the straight path
add("open-baseline-astar", "open_3d", (1, 10, 1), (19, 10, 1), [], "astar")
add("open-baseline-rrt", "open_3d", (1, 10, 1), (19, 10, 1), [], "rrt")
add("open-float-aabb-astar", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 1.5, 3, 3, 2)], "astar")
add("open-float-aabb-rrt", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 1.5, 3, 3, 2)], "rrt")
add("open-float-sphere-astar", "open_3d", (1, 10, 1), (19, 10, 1),
    [SPHERE(10, 10, 1.2, 1.2)], "astar")

# floating_barrier: climb-over + floating sphere in the climb corridor
add("barrier-base-astar", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [], "astar")
add("barrier-climb-astar", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 4.5, 4, 4, 1.5)], "astar")
add("barrier-climb-rrt", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 4.5, 4, 4, 1.5)], "rrt")
add("barrier-sphere-astar", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [SPHERE(10, 10, 4.2, 1.0)], "astar")

# vertical_corridor: shaft transit + low-shaft closure (altitude switch).
# Closure volume keeps ≥1.0 m admission margin from the start pose
# (marginal 0.1 m geometry flapped between ADMIT and REJECT across runs).
add("shaft-base-astar", "vertical_corridor", (10, 10, 1), (10, 1, 5),
    [], "astar")
add("shaft-close-low-astar", "vertical_corridor", (10, 10, 1), (10, 1, 5),
    [AABB(10, 4.5, 1.5, 2.5, 8, 2.0)], "astar")
add("shaft-close-low-rrt", "vertical_corridor", (10, 10, 1), (10, 1, 5),
    [AABB(10, 4.5, 1.5, 2.5, 8, 2.0)], "rrt")

# multi_level: low->high switch + rule on the switch ramp
add("levels-base-astar", "multi_level", (1, 10, 3), (19, 10, 1), [],
    "astar")
add("levels-switch-astar", "multi_level", (1, 10, 3), (19, 10, 1),
    [AABB(10, 10, 2.5, 3, 6, 2.0)], "astar")
add("levels-switch-rrt", "multi_level", (1, 10, 3), (19, 10, 1),
    [AABB(10, 10, 2.5, 3, 6, 2.0)], "rrt")

# multi-volume: 2-3 simultaneous volumes, incl. an infeasible case
add("multi-2open-astar", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(7, 10, 1.5, 2, 4, 2), SPHERE(13, 10, 1.5, 1.2)], "astar")
add("multi-2barrier-rrt", "floating_barrier", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 4.5, 4, 4, 1.5), SPHERE(14, 10, 2.0, 1.0)], "rrt")
add("multi-3levels-astar", "multi_level", (1, 10, 3), (19, 10, 1),
    [AABB(10, 10, 2.5, 3, 6, 2.0), SPHERE(5, 10, 3.5, 1.0),
     AABB(15, 10, 1.0, 3, 5, 1.5)], "astar")
add("multi-sealed-astar", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 2.5, 20, 20, 5.0)], "astar")
add("multi-sealed-rrt", "open_3d", (1, 10, 1), (19, 10, 1),
    [AABB(10, 10, 2.5, 20, 20, 5.0)], "rrt")

print(f"stage A: {len(CASES)} cases")
with open("/tmp/stage_a_3d.json", "w") as f:
    json.dump(CASES, f, indent=1)
