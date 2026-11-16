# OMPL RRTConnect candidate generator (UNTRUSTED proposer only).
#
# Genuine OMPL (libompl 2.0.2, Python bindings): RealVectorStateSpace(3)
# over xyz with bounds, RRTConnect, StateValidityCheckerFn wrapping the
# same occupancy + RuleVolume information. Translation xyz is primary
# (attitude planner-managed/out-of-scope, documented). Deterministic
# given seed (ompl.util.RNG). Every result STILL passes gate_3d before
# any motion authorization — OMPL is a candidate source, never authority.
import math
import time


def ompl_rrt_connect(start, goal, occ_fn, bounds, vols=(),
                     max_time=8.0, range_step=0.5, seed=0):
    """OMPL RRTConnect in xyz. Returns world waypoints or None."""
    try:
        from ompl import base as _ob
        from ompl import geometric as _og
        from ompl import util as _ou
    except ImportError:
        return None
    _ou.RNG.setSeed(int(seed) & 0xFFFFFFFF)
    xmin, ymin, zmin, xmax, ymax, zmax = (float(v) for v in bounds)
    space = _ob.RealVectorStateSpace(3)
    rbounds = _ob.RealVectorBounds(3)
    rbounds.setLow(0, xmin)
    rbounds.setHigh(0, xmax)
    rbounds.setLow(1, ymin)
    rbounds.setHigh(1, ymax)
    rbounds.setLow(2, zmin)
    rbounds.setHigh(2, zmax)
    space.setBounds(rbounds)
    si = _ob.SpaceInformation(space)

    from rulesync3d.rulesync3d.gate_3d import (
        GEOM_AABB, GEOM_SPHERE, segment_intersects_volume)

    def pt_in_vols(x, y, z):
        for v in vols:
            g = int(v["geometry_type"])
            if g == GEOM_AABB:
                bmin, bmax = v["bmin"], v["bmax"]
                if (bmin[0] <= x <= bmax[0] and bmin[1] <= y <= bmax[1]
                        and bmin[2] <= z <= bmax[2]):
                    return True
            elif g == GEOM_SPHERE:
                c = v["center"]
                if (x - c[0]) ** 2 + (y - c[1]) ** 2 + \
                        (z - c[2]) ** 2 <= float(v["radius"]) ** 2:
                    return True
            else:
                xs = [q[0] for q in v.get("vertices", [])] or [x + 1]
                ys = [q[1] for q in v.get("vertices", [])] or [y + 1]
                zs = [q[2] for q in v.get("vertices", [])] or [z + 1]
                if min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys) \
                        and min(zs) <= z <= max(zs):
                    return True
        return False

    class _VC(_ob.StateValidityChecker):
        def isValid(self, state):
            x, y, z = float(state[0]), float(state[1]), float(state[2])
            if occ_fn(x, y, z):
                return False
            return not pt_in_vols(x, y, z)

    si.setStateValidityChecker(_VC(si))
    # Fine motion checking: OMPL interpolates edges at this fraction of
    # space extent; the gate re-verifies segments exactly regardless.
    si.setStateValidityCheckingResolution(0.005)
    setup = _og.SimpleSetup(si)
    planner = _og.RRTConnect(si)
    planner.setRange(float(range_step))
    setup.setPlanner(planner)
    s = space.allocState()
    s[0], s[1], s[2] = (float(v) for v in start)
    g = space.allocState()
    g[0], g[1], g[2] = (float(v) for v in goal)
    setup.setStartAndGoalStates(s, g)
    if setup.solve(float(max_time)) != _ob.PlannerStatus.EXACT_SOLUTION:
        return None
    path = setup.getSolutionPath()
    path.interpolate()
    pts = [(float(st[0]), float(st[1]), float(st[2]))
           for st in path.getStates()]
    if not pts:
        return None
    pts[0] = tuple(float(v) for v in start)
    pts[-1] = tuple(float(v) for v in goal)
    return pts
