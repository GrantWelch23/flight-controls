#!/usr/bin/env python3
"""
3-Drone Pegasus Swarm Diagnostics

Each test creates a System() which auto-starts a mavsdk-server process on the
given gRPC port. Running them sequentially with the same port (50051) caused the
3rd test to crash with "Connection refused". Fixed by using a different port per test.

Note on "frozen" telemetry: a disarmed drone sitting on the ground shows
static DIS/GND/~0.01m/HOLD data. That is correct, not a ghost link.
The test below checks connectivity and reports what it sees — judge it yourself.
"""

import asyncio
from mavsdk import System

# (address, label, grpc_port)
# Sequential tests need different gRPC ports so each mavsdk-server starts cleanly.
ATTEMPTS = [
    ("udpout://127.0.0.1:14580", "Drone 1 — udpout 14580", 50051),
    ("udpout://127.0.0.1:14581", "Drone 2 — udpout 14581", 50052),
    ("udpout://127.0.0.1:14582", "Drone 3 — udpout 14582", 50053),
    ("udpin://0.0.0.0:14540",    "Drone 1 — udpin  14540", 50054),
    ("udpin://0.0.0.0:14541",    "Drone 2 — udpin  14541", 50055),
    ("udpin://0.0.0.0:14542",    "Drone 3 — udpin  14542", 50056),
]


async def test_one(address: str, label: str, grpc_port: int, seconds: float = 14.0):
    drone = System(port=grpc_port)
    print(f"\n[{label}] Trying {address}")

    try:
        await asyncio.wait_for(drone.connect(system_address=address), timeout=7.0)
    except asyncio.TimeoutError:
        print(f"[{label}] ✗ Timed out")
        return
    except Exception as e:
        msg = str(e).lower()
        if "bind" in msg or "address in use" in msg:
            print(f"[{label}] ✗ BIND ERROR — something (PX4) is already listening on this port.")
            print(f"           You cannot use udpin here. Use udpout:// instead.")
        else:
            print(f"[{label}] ✗ Connection error: {e}")
        return

    # Transport level connected
    try:
        async for st in drone.core.connection_state():
            if st.is_connected:
                print(f"[{label}] ✓ Transport connected")
                break
    except Exception as e:
        print(f"[{label}] ✗ State error: {e}")
        return

    # The only thing that matters: do we get changing, useful telemetry?
    print(f"[{label}] Sampling live data for ~{int(seconds)}s (look for changing altitude/mode)...")

    start = asyncio.get_event_loop().time()
    last_alt = None
    last_mode = None
    changes = 0
    samples = 0

    while (asyncio.get_event_loop().time() - start) < seconds:
        try:
            pos = await drone.telemetry.position().__anext__()
            mode = await drone.telemetry.flight_mode().__anext__()
            in_air = await drone.telemetry.in_air().__anext__()
            armed = await drone.telemetry.armed().__anext__()

            alt = round(pos.relative_altitude_m, 2)
            mode_s = str(mode)
            samples += 1

            if last_alt is None or abs(alt - (last_alt or 0)) > 0.2 or mode_s != last_mode:
                changes += 1
                print(f"[{label}] { 'ARM' if armed else 'DIS' } / {'AIR' if in_air else 'GND' }  alt={alt:5.2f}m  mode={mode_s}")

            last_alt = alt
            last_mode = mode_s

        except Exception:
            pass

        await asyncio.sleep(0.35)

    if samples == 0:
        print(f"[{label}] ⚠ Connected but NO telemetry received. Link cannot control the drone.")
    else:
        print(f"[{label}] ✓ Got {samples} samples, {changes} state changes.")
        print(f"         NOTE: a stationary grounded drone (DIS/GND/HOLD) is healthy — static data is correct.")
        print(f"         Arm + takeoff to confirm the link is truly live.")


async def main():
    print("=== 3-Drone Pegasus Swarm Diagnostics ===\n")
    print("Goal: find which addresses give you REAL, changing telemetry on drones 2 and 3.\n")

    for address, label, grpc_port in ATTEMPTS:
        await test_one(address, label, grpc_port, seconds=14.0)

    print("\n=== Finished ===")
    print("Any address that received telemetry samples is a candidate.")
    print("Static DIS/GND/HOLD data from a grounded drone is NORMAL, not a ghost link.")
    print("Use swarm_takeoff.py with the udpout://14580/81/82 addresses.")


if __name__ == "__main__":
    asyncio.run(main())
