# PX4 offboard via MAVSDK (ROS2 DDS-command path unproven; MAVLink proven).
import asyncio
from mavsdk_grpc import System
from mavsdk_grpc.offboard import PositionNedYaw, OffboardError


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected", flush=True)
            break
    async for h in drone.telemetry.health():
        print(f"gps={h.is_global_position_ok} "
              f"home={h.is_home_position_ok}", flush=True)
        break
    await drone.action.arm()
    print("ARMED", flush=True)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0,
                                                         0.0))
    try:
        await drone.offboard.start()
        print("OFFBOARD started", flush=True)
    except OffboardError as e:
        print(f"offboard failed: {e}", flush=True)
        return
    for _ in range(6):
        await asyncio.sleep(5)
        async for p in drone.telemetry.position():
            print(f"rel_alt={p.relative_altitude_m:.2f}", flush=True)
            break
    await drone.action.hold()
    print("HOLD sent", flush=True)
    await asyncio.sleep(3)
    await drone.action.land()
    print("LAND sent", flush=True)


asyncio.run(main())
