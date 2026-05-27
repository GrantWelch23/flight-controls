#!/usr/bin/env python3
"""
3-Drone Simultaneous Takeoff for Pegasus (Isaac Sim + PX4)

PX4 MAVLink port layout (from px4-rc.mavlink):
  - 18570/71/72  GCS link  (QGC uses these)
  - 14580/81/82  API link server ports  (PX4 listens)
  - 14540/41/42  API link remote ports  (PX4 pushes telemetry here)

We use udpin://0.0.0.0:14540/41/42 — MAVSDK listens and PX4 pushes to us.
This mirrors the working single-drone scripts and avoids the handshake
race where udpout:// times out under heavy Isaac Sim CPU load.

Connections are established sequentially (not in parallel) to avoid
spawning three mavsdk-server processes at once on a loaded machine.
After all three are connected, arm and takeoff run in parallel.
"""

import asyncio
from mavsdk import System

DRONES = [
    ("udpin://0.0.0.0:14540", "Drone 1 (Leader)", 50051),
    ("udpin://0.0.0.0:14541", "Drone 2 (Left)",   50052),
    ("udpin://0.0.0.0:14542", "Drone 3 (Right)",  50053),
]


async def connect_ready(drone: System, address: str, name: str, timeout: float = 40.0):
    """Connect + wait for EKF/global position. Returns the drone or None."""
    print(f"[{name}] Connecting via {address} ...")

    try:
        await asyncio.wait_for(drone.connect(system_address=address), timeout=30.0)

        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"[{name}] ✓ Connected")
                break

        print(f"[{name}] Waiting for global position (EKF ready)...")
        start = asyncio.get_event_loop().time()
        while (asyncio.get_event_loop().time() - start) < timeout:
            try:
                health = await drone.telemetry.health().__anext__()
                if health.is_global_position_ok:
                    print(f"[{name}] ✓ EKF ready")
                    return drone
            except Exception:
                pass
            await asyncio.sleep(0.2)

        print(f"[{name}] ✗ Timeout waiting for EKF")
        return None

    except asyncio.TimeoutError:
        print(f"[{name}] ✗ Connection timeout")
        return None
    except Exception as e:
        print(f"[{name}] ✗ Error: {e}")
        return None


async def arm_takeoff_wait(drone: System, name: str, max_wait: float = 30.0):
    """Arm + takeoff, then wait until the drone is verifiably airborne."""
    try:
        print(f"[{name}] Arming...")
        await drone.action.arm()
        print(f"[{name}] ✓ Armed")
        await asyncio.sleep(0.4)

        print(f"[{name}] Taking off...")
        await drone.action.takeoff()

        start = asyncio.get_event_loop().time()
        last_print = 0.0

        while (asyncio.get_event_loop().time() - start) < max_wait:
            try:
                in_air = await drone.telemetry.in_air().__anext__()
                pos = await drone.telemetry.position().__anext__()
                alt = pos.relative_altitude_m
            except Exception:
                await asyncio.sleep(0.2)
                continue

            now = asyncio.get_event_loop().time()
            if now - last_print > 0.5:
                print(f"[{name}] ... alt={alt:5.2f}m  in_air={in_air}", end="\r")
                last_print = now

            if in_air and alt >= 1.8:
                print(f"\n[{name}] ✓ TAKEOFF COMPLETE ({alt:.2f}m)")
                return True

            await asyncio.sleep(0.15)

        print(f"\n[{name}] ⚠ Timeout — never confirmed liftoff")
        return False

    except Exception as e:
        print(f"\n[{name}] ✗ Arm/Takeoff error: {e}")
        return False


async def main():
    print("=== 3-Drone Simultaneous Takeoff ===\n")
    print("Connecting sequentially (reduces mavsdk-server startup pressure)...\n")

    ready = []
    for address, name, grpc_port in DRONES:
        drone = System(port=grpc_port)
        result = await connect_ready(drone, address, name)
        if result:
            ready.append((result, name))
        else:
            print(f"  Skipping {name} — could not connect.\n")

    if not ready:
        print("\n✗ No drones connected.")
        return

    print(f"\n=== {len(ready)}/3 drone(s) ready — arming and taking off together ===\n")

    results = await asyncio.gather(*[arm_takeoff_wait(d, n) for d, n in ready])

    ok = sum(1 for r in results if r)
    print(f"\n=== Done: {ok}/{len(ready)} drones airborne ===")
    if ok > 0:
        print("Run swarm_land.py when ready to land.")


if __name__ == "__main__":
    asyncio.run(main())
