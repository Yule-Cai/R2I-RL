# Gated PX4 mission demo (bounded standard-stack existence proof).
# MAVSDK flight (proven path) + RuleSync transaction (manager) + gate +
# RuleGuard on PX4 odometry. One floating volume, one detour mission.
# Frames: mission xyz (x=east, y=north, z=up); PX4 NED(x=north,y=east,
# z=down). xyz_to_ned = (y, x, -z).
import asyncio
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy

VOL = {"center": (10.0, 0.0, 5.0), "dims": (6.0, 6.0, 2.0)}
START = (0.0, 0.0, 5.0)
GOAL = (20.0, 0.0, 5.0)
EPOCH = 1000


def xyz_to_ned(p):
    return (float(p[1]), float(p[0]), float(-p[2]))


def px4_qos():
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    q.history = HistoryPolicy.KEEP_LAST
    return q


class RS(Node):
    def __init__(self):
        super().__init__("px4_rulesync_demo")
        from px4_msgs.msg import VehicleOdometry as _VO
        self.odom_xyz = None
        self.create_subscription(_VO, "/fmu/out/vehicle_odometry",
                                 self._on_odom, px4_qos())
        from rulesync_interfaces.srv import ProposeVolume as _PV
        from rulesync_interfaces.srv import CommitVolume as _CV
        self.cli_p = self.create_client(_PV, "/rulesync/propose_volume")
        self.cli_c = self.create_client(_CV, "/rulesync/commit_volume")

    def _on_odom(self, msg):
        try:
            self.odom_xyz = (float(msg.position[1]),
                             float(msg.position[0]),
                             float(-msg.position[2]))
        except Exception:
            pass

    def spin_bg(self):
        t = threading.Thread(target=rclpy.spin, args=(self,),
                             daemon=True)
        t.start()

    def commit_volume(self):
        from rulesync_interfaces.msg import RuleVolume
        from rulesync_interfaces.srv import ProposeVolume as _PV
        from rulesync_interfaces.srv import CommitVolume as _CV
        from builtin_interfaces.msg import Time
        assert self.cli_p.wait_for_service(timeout_sec=30.0)
        assert self.cli_c.wait_for_service(timeout_sec=30.0)
        v = RuleVolume()
        v.rule_id = "px4-demo-vol"
        v.epoch = EPOCH
        v.rule_type, v.severity, v.frame_id = 0, 1, "map"
        v.geometry_type = 0
        v.center.x, v.center.y, v.center.z = VOL["center"]
        v.dimensions.x, v.dimensions.y, v.dimensions.z = VOL["dims"]
        t = int(time.time())
        v.valid_from.sec, v.valid_until.sec = t, t + 3600
        f1 = self.cli_p.call_async(_PV.Request(volume=v))
        t0 = time.monotonic()
        while not f1.done() and time.monotonic() - t0 < 30.0:
            time.sleep(0.1)
        r1 = f1.result()
        print(f"propose accepted={r1.accepted} {r1.reason}", flush=True)
        if not r1.accepted:
            return False
        f2 = self.cli_c.call_async(_CV.Request(epoch=EPOCH))
        t0 = time.monotonic()
        while not f2.done() and time.monotonic() - t0 < 60.0:
            time.sleep(0.1)
        r2 = f2.result()
        print(f"commit committed={r2.committed} {r2.reason}", flush=True)
        return bool(r2.committed)


async def main():
    import sys
    sys.path.insert(0, "/home/yule/R2I-RL-checkout")
    rclpy.init()
    rs = RS()
    rs.spin_bg()
    t0 = time.time()
    while rs.odom_xyz is None and time.time() - t0 < 30.0:
        await asyncio.sleep(0.5)
    print(f"odom_xyz={rs.odom_xyz}", flush=True)
    from mavsdk_grpc import System
    from mavsdk_grpc.offboard import PositionNedYaw
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("mavlink connected", flush=True)
    await drone.action.arm()
    await drone.offboard.set_position_ned(PositionNedYaw(*xyz_to_ned(START),
                                                          0.0))
    await drone.offboard.start()
    print("offboard started, climbing to start", flush=True)
    await asyncio.sleep(15)
    # commit floating volume across the straight start->goal line
    ok = await asyncio.to_thread(rs.commit_volume)
    print(f"volume committed: {ok}", flush=True)
    # gate straight (expect BLOCK) and detour (expect PASS)
    import numpy as _np
    from rulesync3d.rulesync3d.gate_3d import gate_3d_path
    cx, cy, cz = VOL["center"]
    dx, dy, dz = VOL["dims"]
    vol = {"geometry_type": 0,
           "bmin": _np.array([cx - dx / 2, cy - dy / 2, cz - dz / 2]),
           "bmax": _np.array([cx + dx / 2, cy + dy / 2, cz + dz / 2])}
    B = (-30.0, -30.0, -5.0, 30.0, 30.0, 15.0)
    FREE = lambda x, y, z: False  # noqa: E731
    straight = [START, GOAL]
    detour = [START, (4.0, 0.0, 8.5), (16.0, 0.0, 8.5), GOAL]
    print("gate straight:", gate_3d_path(straight, [vol], FREE, B),
          flush=True)
    print("gate detour:", gate_3d_path(detour, [vol], FREE, B),
          flush=True)
    # fly detour with RuleGuard monitoring odometry
    from rulesync3d.rulesync3d.ruleguard import RuleGuard
    g = RuleGuard()
    g.update_snapshot(EPOCH, "px4-demo")
    g.bind_execution(EPOCH, "px4-demo")
    warns, holds, minclr = 0, 0, float("inf")
    for wp in detour[1:]:
        n, e, d = xyz_to_ned(wp)
        await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
        t1 = time.time()
        while time.time() - t1 < 25.0:
            await asyncio.sleep(0.2)
            if rs.odom_xyz is None:
                continue
            st, info = g.check_bound_3d(rs.odom_xyz, [vol], 0.5)
            clr = float(info.get("clearance", float("inf")))
            minclr = min(minclr, clr)
            if st == "WARNING":
                warns += 1
            elif st == "HOLD":
                holds += 1
                print(f"GUARD HOLD clr={clr:.2f} pos={rs.odom_xyz}",
                      flush=True)
                break
            import math
            if math.dist(rs.odom_xyz, wp) < 1.0:
                break
        else:
            pass
        if holds:
            break
    print(f"guard warns={warns} holds={holds} min_clr={minclr:.2f} "
          f"final={rs.odom_xyz}", flush=True)
    if holds:
        await drone.action.hold()
        print("HOLD sent", flush=True)
    await drone.action.land()
    print("LAND sent", flush=True)
    rs.destroy_node()
    rclpy.shutdown()


asyncio.run(main())
