#!/usr/bin/env python3

import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
from mavsdk.telemetry import FlightMode


# ====================== Drone Configuration ======================

DRONES = [
    ("udpin://0.0.0.0:14540", "Drone 1 (Leader)", 50051),
    ("udpin://0.0.0.0:14541", "Drone 2 (Left)",   50052),
    ("udpin://0.0.0.0:14542", "Drone 3 (Right)",  50053),
]


# ====================== Connect Drone ======================

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


# ====================== Arm and Takeoff ======================

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


# ====================== Wait Airborne ======================

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


# ====================== Enter Offboard and Climb ======================

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

                # Switch to stable hold using current NED position + yaw
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

# ====================== Fly Forward ======================

async def fly_forward(drone: System, name: str, distance_m: float = 10.0, speed_mps: float = 2.0):
    """Fly forward a set distance while maintaining altitude and heading."""
    print(f"[{name}] Flying forward {distance_m}m at {speed_mps}m/s...")

    try:
        # Get current position and yaw
        ned = await drone.telemetry.position_velocity_ned().__anext__()
        att = await drone.telemetry.attitude_euler().__anext__()

        current_north = ned.position.north_m
        current_east = ned.position.east_m
        current_down = ned.position.down_m
        current_yaw = att.yaw_deg

        # Calculate target position
        import math
        yaw_rad = math.radians(current_yaw)
        target_north = current_north + distance_m * math.cos(yaw_rad)
        target_east = current_east + distance_m * math.sin(yaw_rad)

        # Move to target position
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=target_north,
                east_m=target_east,
                down_m=current_down,
                yaw_deg=current_yaw
            )
        )

        # Wait until we reach the target
        for _ in range(200):  # ~20 seconds max
            current_ned = await drone.telemetry.position_velocity_ned().__anext__()
            dn = current_ned.position.north_m - current_north
            de = current_ned.position.east_m - current_east
            distance_traveled = math.sqrt(dn**2 + de**2)

            print(f"[{name}] Distance traveled: {distance_traveled:.2f}m / {distance_m:.2f}m", end="\r")

            if distance_traveled >= distance_m - 0.5:
                print(f"\n[{name}] ✓ Reached target distance ({distance_traveled:.2f}m)")
                return True

            await asyncio.sleep(0.1)

        print(f"\n[{name}] ⚠ Forward flight timeout")
        return False

    except Exception as e:
        print(f"\n[{name}] ✗ Forward flight error: {e}")
        return False
    
# ====================== CLEANUP / LAND ======================

async def land_and_cleanup(drone: System, name: str):
    """Land a drone and clean up OFFBOARD mode."""
    try:
        print(f"[{name}] Stopping OFFBOARD and landing...")
        await drone.offboard.stop()
        await drone.action.land()
        print(f"[{name}] ✓ Land command sent")

        # Wait for landing to complete
        for _ in range(100):  # ~10 seconds max
            try:
                in_air = await drone.telemetry.in_air().__anext__()
                pos = await drone.telemetry.position().__anext__()
                if not in_air and pos.relative_altitude_m < 0.5:
                    print(f"[{name}] ✓ Landed successfully")
                    return True
            except Exception:
                pass
            await asyncio.sleep(0.1)

        print(f"[{name}] ⚠ Landing timeout (may still be in progress)")
        return False

    except Exception as e:
        print(f"[{name}] ✗ Land error: {e}")
        return False


# ====================== Mission Start ======================

async def main():
    print("=== 3-Drone Takeoff + Forward Flight ===\n")
    sys.stdout.flush()

    # ===================== PHASE 1: ARM + TAKEOFF ALL FIRST =====================
    print("=== PHASE 1: Arm + Takeoff all drones ===\n")

    drones = []
    for i, (address, name, grpc_port) in enumerate(DRONES):
        drone = await connect_drone(address, name, grpc_port)
        if not drone:
            continue

        ok = await arm_and_takeoff(drone, name)
        if not ok:
            continue

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

        if i < len(airborne) - 1:
            await asyncio.sleep(1.0)

    if len(successful) < 3:
        print(f"\n✗ Only {len(successful)}/3 drones reached OFFBOARD. Exiting.")
        return

    print("\n=== All 3 drones in OFFBOARD — starting forward flight ===\n")

    # ===================== PHASE 3: FORWARD FLIGHT =====================
    print("=== PHASE 3: Fly forward together ===\n")

    results = await asyncio.gather(*[fly_forward(d, n, distance_m=10.0, speed_mps=2.0) for d, n in airborne])

    print("\n=== Done ===\n")

    # ===================== PHASE 4: LAND ALL DRONES (PARALLEL) =====================
    print("=== PHASE 4: Land all drones together ===\n")

    await asyncio.gather(*[land_and_cleanup(d, n) for d, n in airborne])

    print("\n=== Script complete — all drones landed ===\n")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nExiting — offboard streams will stop.")