# PX4 arm/mission probe (mavsdk-grpc API).
import asyncio
from mavsdk_grpc import System


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("waiting for connection...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected", flush=True)
            break
    async for h in drone.telemetry.health():
        print(f"health gyro={h.is_gyrometer_calibration_ok} "
              f"accel={h.is_accelerometer_calibration_ok} "
              f"mag={h.is_magnetometer_calibration_ok} "
              f"gps={h.is_global_position_ok} "
              f"home={h.is_home_position_ok} "
              f"armable={h.is_armable}", flush=True)
        break
    try:
        await drone.action.arm()
        print("ARMED", flush=True)
    except Exception as e:
        print(f"arm failed: {e}", flush=True)
        return
    try:
        await drone.action.takeoff()
        print("TAKEOFF cmd sent", flush=True)
    except Exception as e:
        print(f"takeoff failed: {e}", flush=True)
        return
    await asyncio.sleep(12)
    async for p in drone.telemetry.position():
        print(f"pos rel_alt={p.relative_altitude_m:.2f}", flush=True)
        break
    try:
        await drone.action.hold()
        print("HOLD cmd sent", flush=True)
    except Exception as e:
        print(f"hold failed: {e}", flush=True)
    await asyncio.sleep(3)
    try:
        await drone.action.land()
        print("LAND cmd sent", flush=True)
    except Exception as e:
        print(f"land failed: {e}", flush=True)


asyncio.run(main())
