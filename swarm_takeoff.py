#!/usr/bin/env python3
"""
3-Drone Arm/Takeoff first → then Offboard (separated phases)

This version was rewritten after observing the real failure mode:
- Arming drones 2 and 3 with `COMMAND_DENIED` happens specifically *after*
  drone 1 has already entered offboard.
- Therefore we must do **all arming + takeoff first** (while no drones
  are in offboard), then transition them into offboard one by one.

Goal: Get all three drones reliably into OFFBOARD mode holding position
so you can run swarm behaviors.

Structure:
  Phase 1: Connect + Arm + Takeoff all three (light stagger between drones)
  Phase 2: Once all are airborne, put them into offboard one-by-one
           (climb to 5m, then switch to stable PositionNedYaw hold)

At the end the script idles so all three offboard streams + holds stay active.

Run:
    python3 flight_controls/swarm_takeoffv4.py
"""

import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
from mavsdk.telemetry import FlightMode


DRONES = [
    ("udpin://0.0.0.0:14540", "Drone 1 (Leader)", 50051),
    ("udpin://0.0.0.0:14541", "Drone 2 (Left)",   50052),
    ("udpin://0.0.0.0:14542", "Drone 3 (Right)",  50053),
]


async def connect_drone(address: str, name: str, grpc_port: int):
    """Connect and wait for basic health. Returns the System or None."""
    drone = System(port=grpc_port)
    print(f"[{name}] Connecting via {address} ...")
    try:
        await asyncio.wait_for(drone.connect(system_address=address), timeout=30)
    except Exception as e:
        print(f"[{name}] ✗ Connection failed: {e}")
        return None

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{name}] ✓ Connected")
            break

    print(f"[{name}] Waiting for global position...")
    for _ in range(150):  # ~30s max
        try:
            health = await drone.telemetry.health().__anext__()
            if health.is_global_position_ok:
                print(f"[{name}] ✓ Global position OK")
                return drone
        except Exception:
            pass
        await asyncio.sleep(0.2)

    print(f"[{name}] ✗ Timed out waiting for global position")
    return None


async def arm_and_takeoff(drone: System, name: str):
    """Arm + takeoff. Returns True on success."""
    try:
        print(f"[{name}] Arming...")
        await drone.action.arm()
        print(f"[{name}] ✓ Armed")
        await asyncio.sleep(0.5)

        print(f"[{name}] Taking off...")
        await drone.action.takeoff()
        print(f"[{name}] ✓ Takeoff command sent")
        return True
    except Exception as e:
        print(f"[{name}] ✗ Arm/Takeoff error: {e}")
        return False


async def wait_airborne(drone: System, name: str, timeout: float = 25.0):
    """Wait until the drone is clearly in the air."""
    start = asyncio.get_running_loop().time()
    while (asyncio.get_running_loop().time() - start) < timeout:
        try:
            in_air = await drone.telemetry.in_air().__anext__()
            pos = await drone.telemetry.position().__anext__()
            if in_air and pos.relative_altitude_m > 1.5:
                print(f"[{name}] ✓ Airborne (alt={pos.relative_altitude_m:.1f}m)")
                return True
        except Exception:
            pass
        await asyncio.sleep(0.3)
    print(f"[{name}] ⚠ Did not confirm airborne within timeout")
    return False


async def enter_offboard_and_climb(drone: System, name: str, target_alt: float = 5.0):
    """Put one drone into offboard, climb, then switch to position hold."""
    print(f"[{name}] Entering OFFBOARD + climbing to {target_alt}m...")

    try:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
        )
        await drone.offboard.start()
        print(f"[{name}] ✓ OFFBOARD started")
    except Exception as e:
        print(f"[{name}] ✗ Offboard start failed: {e}")
        return False

    # Climb loop
    for _ in range(300):
        try:
            pos = await drone.telemetry.position().__anext__()
            alt = pos.relative_altitude_m
            print(f"[{name}] alt={alt:.2f}m", end="\r")

            if alt >= target_alt:
                print(f"\n[{name}] ✓ Reached {alt:.2f}m")

                # Switch to stable hold
                try:
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
                    print(f"[{name}] ✓ Now holding position in OFFBOARD")
                    await asyncio.sleep(2.0)  # let it settle
                    return True
                except Exception as hold_err:
                    print(f"[{name}] ⚠ Hold setpoint error: {hold_err}")
                    return False
        except Exception:
            pass
        await asyncio.sleep(0.1)

    print(f"\n[{name}] ⚠ Climb timeout")
    return False


async def main():
    print("=== 3-Drone OFFBOARD (Arm all first → then Offboard) ===\n")
    sys.stdout.flush()

    # ===================== PHASE 1: ARM + TAKEOFF ALL FIRST =====================
    print("=== PHASE 1: Arm + Takeoff all drones (no offboard yet) ===\n")

    drones = []   # will hold (drone, name) for the ones that make it through arm/takeoff

    for i, (address, name, grpc_port) in enumerate(DRONES):
        drone = await connect_drone(address, name, grpc_port)
        if not drone:
            continue

        ok = await arm_and_takeoff(drone, name)
        if not ok:
            continue

        # Small stagger between starting each arm/takeoff sequence
        if i < len(DRONES) - 1:
            await asyncio.sleep(1.2)

        drones.append((drone, name))

    if not drones:
        print("\n✗ No drones armed/taken off. Exiting.")
        return

    print(f"\n=== Waiting for all {len(drones)} drone(s) to become airborne ===\n")
    airborne = []
    for drone, name in drones:
        if await wait_airborne(drone, name):
            airborne.append((drone, name))

    if not airborne:
        print("\n✗ No drones confirmed airborne. Exiting.")
        return

    print(f"\n=== {len(airborne)}/{len(DRONES)} drones airborne — moving to offboard phase ===\n")

    # ===================== PHASE 2: OFFBOARD ONE BY ONE =====================
    print("=== PHASE 2: Enter OFFBOARD one-by-one ===\n")

    successful = []
    for i, (drone, name) in enumerate(airborne):
        ok = await enter_offboard_and_climb(drone, name)
        if ok:
            successful.append(name)

        # Small stagger between offboard entries (not for CPU — to reduce chance of command conflict)
        if i < len(airborne) - 1:
            await asyncio.sleep(1.0)

    # ===================== FINAL STATE =====================
    print("\n=== FINAL RESULT ===")
    print(f"Successfully in OFFBOARD hold: {len(successful)}/3 → {successful}")

    if len(successful) == 3:
        print("\n✓ All three drones are now in OFFBOARD holding position at ~5 m.")
        print("  The MAVSDK offboard streams are active.")
        print("  Ready for swarm behavior logic.")

        # ===================== LIVE VERIFICATION =====================
        print("\n=== LIVE OFFBOARD VERIFICATION ===")
        print("Monitoring actual flight_mode telemetry for the next 15 seconds...")
        print("You should see 'OFFBOARD' for all three. This is the real proof.\n")

        verify_start = asyncio.get_running_loop().time()
        while (asyncio.get_running_loop().time() - verify_start) < 15:
            modes = []
            for drone, name in airborne:
                try:
                    mode = await drone.telemetry.flight_mode().__anext__()
                    mode_str = str(mode).split('.')[-1] if '.' in str(mode) else str(mode)
                    modes.append(f"{name}: {mode_str}")
                except Exception:
                    modes.append(f"{name}: ?")

            print(" | ".join(modes))
            await asyncio.sleep(2.0)

        print("\n=== VERIFICATION COMPLETE ===")
        print("If you saw OFFBOARD for all three above, they are genuinely in offboard simultaneously.")
        print("The altitude climb was done using velocity setpoints sent *after* offboard.start() succeeded.")
        print("PX4 will not obey offboard velocity/position setpoints unless the vehicle is actually in OFFBOARD mode.")

        print("\nIdling with live mode monitoring... (Ctrl-C to exit)")
        while True:
            modes = []
            for drone, name in airborne:
                try:
                    mode = await drone.telemetry.flight_mode().__anext__()
                    mode_str = str(mode).split('.')[-1] if '.' in str(mode) else str(mode)
                    modes.append(f"{name}: {mode_str}")
                except Exception:
                    modes.append(f"{name}: ?")
            print(" | ".join(modes))
            await asyncio.sleep(4)
    else:
        print("\nSome drones did not reach offboard hold.")
        print("The ones that did are still holding (their streams are active).")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nExiting — offboard streams will stop.")
