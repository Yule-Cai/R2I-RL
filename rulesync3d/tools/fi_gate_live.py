# Live fault-injection: fetch COMMITTED volumes from the manager and gate
# hand-crafted intersecting trajectories against them. Asserts ALL blocked.
# Ties the gate to live committed state (not just unit fixtures).
import rclpy
from rclpy.node import Node
from rulesync_interfaces.srv import ListVolumes

rclpy.init()
n = Node("fi_gate_live")
c = n.create_client(ListVolumes, "/rulesync/list_volumes")
assert c.wait_for_service(timeout_sec=30.0)
f = c.call_async(ListVolumes.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
r = f.result()
print(f"epoch={r.epoch} active={[str(v.rule_id) for v in r.active]}",
      flush=True)
import numpy as _np
from rulesync3d.rulesync3d.gate_3d import (
    GEOM_AABB, GEOM_SPHERE, gate_3d_path)
vols = []
for v in r.active:
    g = int(v.geometry_type)
    if g == GEOM_AABB:
        cx, cy, cz = v.center.x, v.center.y, v.center.z
        dx, dy, dz = v.dimensions.x, v.dimensions.y, v.dimensions.z
        vols.append({"geometry_type": 0,
                     "bmin": _np.array([cx - dx / 2, cy - dy / 2,
                                        cz - dz / 2]),
                     "bmax": _np.array([cx + dx / 2, cy + dy / 2,
                                        cz + dz / 2])})
    elif g == GEOM_SPHERE:
        vols.append({"geometry_type": 1,
                     "center": _np.array([v.center.x, v.center.y,
                                          v.center.z]),
                     "radius": float(v.dimensions.x)})
FREE = lambda x, y, z: False  # noqa: E731
BOUNDS = (0.0, 0.0, 0.2, 20.0, 20.0, 6.0)
blocked = 0
tested = 0
for v in vols:
    if int(v["geometry_type"]) == GEOM_AABB:
        bmin, bmax = v["bmin"], v["bmax"]
        center = (bmin + bmax) / 2
        # 5 skewers through the box from different sides
        for k, off in enumerate([0.0, 0.3, -0.3, 0.6, -0.6]):
            a = (float(bmin[0]) - 2.0, float(center[1]) + off,
                 float(center[2]))
            b = (float(bmax[0]) + 2.0, float(center[1]) - off,
                 float(center[2]))
            ok, rs = gate_3d_path([a, center, b], vols, FREE, BOUNDS)
            tested += 1
            blocked += (not ok)
            print(f"skewer{k} ok={ok} reason={rs}", flush=True)
    else:
        ce = v["center"]
        a = (float(ce[0]) - 3.0, float(ce[1]), float(ce[2]))
        b = (float(ce[0]) + 3.0, float(ce[1]), float(ce[2]))
        ok, rs = gate_3d_path([a, tuple(float(x) for x in ce), b],
                              vols, FREE, BOUNDS)
        tested += 1
        blocked += (not ok)
        print(f"sphere-skewer ok={ok} reason={rs}", flush=True)
print(f"FI: {blocked}/{tested} known-intersecting blocked", flush=True)
assert tested > 0 and blocked == tested, "GATE LEAK!"
n.destroy_node()
rclpy.shutdown()
