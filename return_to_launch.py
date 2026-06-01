#!/usr/bin/env python3
"""
EMERGENCY RTL SCRIPT (Fast Version)
Run this immediately if the drone is doing something weird or unsafe.
"""

import asyncio
import sys
from mavsdk import System


async def emergency_rtl():
    print("=== ⚠️  EMERGENCY RTL ACTIVATED ⚠️  ===")
    sys.stdout.flush()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected")
            break

    # Set RTL altitude
    await drone.param.set_param_float("RTL_RETURN_ALT", 15.0)

    # Immediately send RTL (don't stop OFFBOARD first - PX4 handles it)
    print("Sending EMERGENCY RETURN TO LAUNCH...")
    await drone.action.return_to_launch()
    print("✓ RTL command sent — Drone returning home immediately")

    # Wait for landing confirmation
    print("Waiting for drone to land...")
    for _ in range(120):
        try:
            in_air = await drone.telemetry.in_air().__anext__()
            if not in_air:
                print("✓ Drone has landed safely")
                print("=== Emergency RTL Complete ===")
                return
        except Exception:
            pass
        await asyncio.sleep(1.0)

    print("⚠ Timeout waiting for landing")
    print("=== Emergency RTL Complete ===")


if __name__ == "__main__":
    try:
        asyncio.run(emergency_rtl())
    except KeyboardInterrupt:
        print("\n\nExiting.")