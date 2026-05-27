#!/usr/bin/env python3
"""
3-Drone Simultaneous Land for Pegasus (Isaac Sim + PX4)

Uses udpin://0.0.0.0:14540/41/42 — MAVSDK listens, PX4 pushes telemetry.
Connects sequentially to avoid overloading the machine during server startup,
then lands all three in parallel.
"""

import asyncio
from mavsdk import System

DRONES = [
    ("udpin://0.0.0.0:14540", "Drone 1 (Leader)", 50051),
    ("udpin://0.0.0.0:14541", "Drone 2 (Left)",   50052),
    ("udpin://0.0.0.0:14542", "Drone 3 (Right)",  50053),
]


async def connect_simple(drone: System, address: str, name: str):
    print(f"[{name}] Connecting...")
    try:
        await asyncio.wait_for(drone.connect(system_address=address), timeout=30.0)
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"[{name}] ✓ Connected")
                return drone
    except Exception as e:
        print(f"[{name}] ✗ Failed: {e}")
    return None


async def land_and_wait(drone: System, name: str, timeout: float = 50.0):
    """Send land command and wait for touchdown."""
    try:
        try:
            await drone.offboard.stop()
        except Exception:
            pass

        print(f"[{name}] Sending LAND command...")
        await drone.action.land()

        start = asyncio.get_event_loop().time()
        last_print = 0.0

        while (asyncio.get_event_loop().time() - start) < timeout:
            try:
                in_air = await drone.telemetry.in_air().__anext__()
                pos = await drone.telemetry.position().__anext__()
                alt = pos.relative_altitude_m
                armed = await drone.telemetry.armed().__anext__()
            except Exception:
                await asyncio.sleep(0.25)
                continue

            now = asyncio.get_event_loop().time()
            if now - last_print > 0.6:
                print(f"[{name}] ... alt={alt:5.2f}m  in_air={in_air}  armed={armed}", end="\r")
                last_print = now

            if not in_air and alt < 1.0:
                print(f"\n[{name}] ✓ LANDED (alt={alt:.2f}m)")
                await asyncio.sleep(2.0)
                return True

            await asyncio.sleep(0.2)

        print(f"\n[{name}] ⚠ Timeout — landing not confirmed")
        return False

    except Exception as e:
        print(f"\n[{name}] ✗ Land error: {e}")
        return False


async def main():
    print("=== 3-Drone Simultaneous Land ===\n")
    print("Connecting sequentially...\n")

    ready = []
    for address, name, grpc_port in DRONES:
        drone = System(port=grpc_port)
        result = await connect_simple(drone, address, name)
        if result:
            ready.append((result, name))
        else:
            print(f"  Skipping {name}\n")

    if not ready:
        print("\n✗ Nothing connected.")
        return

    print(f"\n=== Landing {len(ready)} drone(s) together ===\n")

    results = await asyncio.gather(*[land_and_wait(d, n) for d, n in ready])

    ok = sum(1 for r in results if r)
    print(f"\n=== Done: {ok}/{len(ready)} drones confirmed on the ground ===")


if __name__ == "__main__":
    asyncio.run(main())
