#!/usr/bin/env python3
"""
SIMPLE UP AND LAND TEST
First real flight test - climb 5 meters then land safely.
"""

import asyncio
import sys
from config import CONNECTION_STRING
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw


async def simple_up_and_land():
    print("=== SIMPLE UP AND LAND TEST ===")
    sys.stdout.flush()

    drone = System()
    await drone.connect(system_address=CONNECTION_STRING)

    print("Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to drone")
            break

    print("Waiting for global position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("✓ Global position OK")
            break

    print("Arming...")
    await drone.action.arm()
    print("✓ Armed")

    print("Taking off...")
    await drone.action.takeoff()
    print("✓ Takeoff command sent")

    await asyncio.sleep(3)

    # Enter OFFBOARD and climb to 5 meters
    print("Entering OFFBOARD mode...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
    )
    await drone.offboard.start()
    print("✓ OFFBOARD started")

    print("Climbing to 5 meters...")
    while True:
        position = await drone.telemetry.position().__anext__()
        alt = position.relative_altitude_m
        print(f"Altitude: {alt:.2f}m", end="\r")

        if alt >= 5.0:
            print(f"\n✓ Reached 5 meters")
            break
        await asyncio.sleep(0.2)

    # Stop velocity and switch to position hold (matching your other scripts)
    print("Holding position...")
    ned = await drone.telemetry.position_velocity_ned().__anext__()
    att = await drone.telemetry.attitude_euler().__anext__()

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            north_m=ned.position.north_m,
            east_m=ned.position.east_m,
            down_m=ned.position.down_m,
            yaw_deg=att.yaw_deg
        )
    )
    await asyncio.sleep(3)  # Hold for 3 seconds

    # Land
    print("Landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Land command sent")

    # Wait for landing
    print("Waiting for landing to complete...")
    for _ in range(30):
        in_air = await drone.telemetry.in_air().__anext__()
        if not in_air:
            print("✓ Landed safely")
            break
        await asyncio.sleep(1.0)

    print("=== Test Complete ===")


if __name__ == "__main__":
    try:
        asyncio.run(simple_up_and_land())
    except KeyboardInterrupt:
        print("\n\nExiting.")