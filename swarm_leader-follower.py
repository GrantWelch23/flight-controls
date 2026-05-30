#!/usr/bin/env python3
"""
3-Drone Leader-Follower Formation Flight (Fixed)
"""

import asyncio
import sys
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw


# ====================== Drone Configuration ======================

DRONES = [
    ("udpin://0.0.0.0:14540", "Drone 1 (Leader)", 50051),
    ("udpin://0.0.0.0:14541", "Drone 2 (Left)",   50052),
    ("udpin://0.0.0.0:14542", "Drone 3 (Right)",  50053),
]


# ====================== Connect Drone ======================

async def connect_drone(address: str, name: str, grpc_port: int):
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
    for _ in range(150):
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

    for _ in range(300):
        try:
            pos = await drone.telemetry.position().__anext__()
            alt = pos.relative_altitude_m
            print(f"[{name}] alt={alt:.2f}m", end="\r")

            if alt >= target_alt:
                print(f"\n[{name}] ✓ Reached {alt:.2f}m")

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
                    await asyncio.sleep(2.0)
                    return True
                except Exception as hold_err:
                    print(f"[{name}] ⚠ Hold setpoint error: {hold_err}")
                    return False
        except Exception:
            pass
        await asyncio.sleep(0.1)

    print(f"\n[{name}] ⚠ Climb timeout")
    return False


# ====================== Align Yaw ======================

async def align_yaw(drone: System, name: str, target_yaw: float, tolerance: float = 5.0):
    """
    Rotate drone to face a specific yaw angle, then re-establish a stable
    PositionNedYaw hold at the target yaw. This prevents leaving the drone
    under velocity control when moving to the next phase.
    """
    print(f"[{name}] Aligning yaw to {target_yaw:.1f}°...")

    final_pos = None
    aligned = False

    for _ in range(100):
        try:
            att = await drone.telemetry.attitude_euler().__anext__()
            current_yaw = att.yaw_deg

            yaw_error = target_yaw - current_yaw
            if yaw_error > 180:
                yaw_error -= 360
            elif yaw_error < -180:
                yaw_error += 360

            if abs(yaw_error) < tolerance:
                print(f"[{name}] ✓ Yaw aligned ({current_yaw:.1f}°)")
                aligned = True
                break

            # Rotate toward target
            rotation_speed = 20.0 if yaw_error > 0 else -20.0
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, rotation_speed)
            )

            await asyncio.sleep(0.2)

            # Continuously refresh current position for the final hold
            try:
                ned = await drone.telemetry.position_velocity_ned().__anext__()
                final_pos = ned.position
            except Exception:
                pass

        except Exception:
            pass

    # Always try to end in a clean position hold with the desired yaw
    try:
        if final_pos is None:
            ned = await drone.telemetry.position_velocity_ned().__anext__()
            final_pos = ned.position

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=final_pos.north_m,
                east_m=final_pos.east_m,
                down_m=final_pos.down_m,
                yaw_deg=target_yaw
            )
        )
        await asyncio.sleep(1.0)  # let the hold take effect
        print(f"[{name}] ✓ Position hold re-established at yaw={target_yaw:.1f}°")
    except Exception as hold_err:
        print(f"[{name}] ⚠ Could not re-establish position hold after yaw: {hold_err}")

    if not aligned:
        print(f"[{name}] ⚠ Yaw alignment timeout (still holding best effort at target yaw)")

    return aligned


# ====================== Formation Geometry Helpers ======================

def body_offset_to_ned(forward_m: float, right_m: float, yaw_deg: float):
    """
    Convert a body-frame offset (forward, right) into NED delta given a yaw heading.
    Positive forward = along the heading.
    Positive right = starboard (right when facing heading).
    Returns (dnorth, deast).
    """
    yaw_rad = math.radians(yaw_deg)
    dn = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
    de = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
    return dn, de


# ====================== Move to Formation Position ======================

async def move_to_formation_position(drone: System, name: str, target_north: float, target_east: float,
                                       desired_yaw: float, altitude: float = 7.0, timeout: float = 60.0):
    """
    Move a drone to an absolute NED position and hold a specific yaw on arrival.
    Used for precise isosceles triangle formation setup.
    """
    print(f"[{name}] Moving to formation position (yaw={desired_yaw:.1f}°)...")

    try:
        target_down = -altitude

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=target_north,
                east_m=target_east,
                down_m=target_down,
                yaw_deg=desired_yaw
            )
        )

        iterations = int(timeout / 0.3)
        for _ in range(iterations):
            current = await drone.telemetry.position_velocity_ned().__anext__()
            dn = current.position.north_m - target_north
            de = current.position.east_m - target_east
            distance = math.sqrt(dn**2 + de**2)

            print(f"[{name}] Distance to target: {distance:.2f}m", end="\r")

            if distance < 1.0:
                print(f"\n[{name}] ✓ In formation position")
                # Re-assert clean hold with correct yaw to settle
                await drone.offboard.set_position_ned(
                    PositionNedYaw(
                        north_m=target_north,
                        east_m=target_east,
                        down_m=target_down,
                        yaw_deg=desired_yaw
                    )
                )
                await asyncio.sleep(0.5)
                return True

            await asyncio.sleep(0.3)

        print(f"\n[{name}] ⚠ Formation position timeout")
        return False

    except Exception as e:
        print(f"\n[{name}] ✗ Formation movement error: {e}")
        return False


# ====================== Leader-Follower Flight ======================

async def leader_follower_flight(leader, wingmen, mission_yaw: float, body_offsets,
                                 speed_mps: float = 2.0, duration_s: float = 20.0,
                                 formation_alt: float = 7.0, update_hz: float = 15.0):
    """
    True dynamic leader-follower formation flight.

    - Leader advances along the captured mission_yaw at constant speed (using position targets for smoothness).
    - Wingmen continuously recompute their NED targets from the *live* leader position + rotated body offsets.
    - All aircraft hold the same mission_yaw (they fly the direction they were facing at start).
    - Isosceles triangle geometry is preserved throughout the flight.
    """
    print("=== Starting TRUE DYNAMIC Leader-Follower Formation Flight ===")
    print(f"    Mission yaw (formation heading): {mission_yaw:.1f}°")
    print(f"    Body offsets (forward, right): {body_offsets}")
    print(f"    Speed: {speed_mps} m/s  |  Duration: {duration_s}s  |  Update: {update_hz} Hz")

    dt = 1.0 / update_hz
    end_time = asyncio.get_running_loop().time() + duration_s

    # Starting leader position (for leader's marching target)
    try:
        leader_start = await leader.telemetry.position_velocity_ned().__anext__()
        leader_n = leader_start.position.north_m
        leader_e = leader_start.position.east_m
        leader_d = leader_start.position.down_m
    except Exception as e:
        print(f"✗ Failed to read initial leader position: {e}")
        return False

    # Precompute unit vector for leader advancement
    yaw_rad = math.radians(mission_yaw)
    fwd_n = math.cos(yaw_rad)
    fwd_e = math.sin(yaw_rad)

    print("\n[Formation] Entering dynamic control loop...")

    loop_count = 0
    last_log = 0.0

    try:
        while asyncio.get_running_loop().time() < end_time:
            now = asyncio.get_running_loop().time()

            # === Read live leader state ===
            try:
                lp = await leader.telemetry.position_velocity_ned().__anext__()
                live_leader_n = lp.position.north_m
                live_leader_e = lp.position.east_m
            except Exception:
                # If we can't read, keep marching from last known
                pass

            # === Advance leader along mission heading (smooth position control) ===
            # March forward from the original start line so total distance is predictable
            elapsed = now - (end_time - duration_s)
            distance_along = max(0.0, min(speed_mps * elapsed, speed_mps * duration_s))
            leader_target_n = leader_n + fwd_n * distance_along
            leader_target_e = leader_e + fwd_e * distance_along

            await leader.offboard.set_position_ned(
                PositionNedYaw(
                    north_m=leader_target_n,
                    east_m=leader_target_e,
                    down_m=leader_d,
                    yaw_deg=mission_yaw
                )
            )

            # === Dynamic follower targets (true leader-relative) ===
            for i, (drone, name) in enumerate(wingmen):
                fwd, rgt = body_offsets[i]
                dn, de = body_offset_to_ned(fwd, rgt, mission_yaw)

                target_n = live_leader_n + dn
                target_e = live_leader_e + de
                target_d = -formation_alt

                await drone.offboard.set_position_ned(
                    PositionNedYaw(
                        north_m=target_n,
                        east_m=target_e,
                        down_m=target_d,
                        yaw_deg=mission_yaw
                    )
                )

            # Flight logging
            loop_count += 1
            if now - last_log >= 1.0:
                # Show how far the formation has traveled
                print(f"[Formation] t={elapsed:5.1f}s  leader dist={distance_along:6.2f}m  yaw={mission_yaw:.1f}°")
                last_log = now

            await asyncio.sleep(dt)

        # Final stop: hold current positions
        print("\n[Formation] Flight duration complete — holding final positions...")
        for _ in range(3):
            try:
                lp = await leader.telemetry.position_velocity_ned().__anext__()
                await leader.offboard.set_position_ned(
                    PositionNedYaw(
                        north_m=lp.position.north_m,
                        east_m=lp.position.east_m,
                        down_m=lp.position.down_m,
                        yaw_deg=mission_yaw
                    )
                )
                for drone, name in wingmen:
                    wp = await drone.telemetry.position_velocity_ned().__anext__()
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(
                            north_m=wp.position.north_m,
                            east_m=wp.position.east_m,
                            down_m=wp.position.down_m,
                            yaw_deg=mission_yaw
                        )
                    )
            except Exception:
                pass
            await asyncio.sleep(0.3)

        print("=== Dynamic Leader-Follower Flight Complete ===")
        return True

    except Exception as e:
        print(f"✗ Dynamic formation flight error: {e}")
        # Emergency stop all velocity
        try:
            await leader.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            for drone, _ in wingmen:
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        except Exception:
            pass
        return False


# ====================== Land and Cleanup ======================

async def land_and_cleanup(drone: System, name: str):
    try:
        print(f"[{name}] Landing...")
        await drone.offboard.stop()
        await drone.action.land()

        for _ in range(100):
            try:
                in_air = await drone.telemetry.in_air().__anext__()
                pos = await drone.telemetry.position().__anext__()
                if not in_air and pos.relative_altitude_m < 0.5:
                    print(f"[{name}] ✓ Landed")
                    return True
            except Exception:
                pass
            await asyncio.sleep(0.1)

        return False
    except Exception as e:
        print(f"[{name}] Land error: {e}")
        return False


# ====================== Flight Start ======================

async def main():
    print("=== 3-Drone Leader-Follower Formation ===\n")
    sys.stdout.flush()

    # ===================== PHASE 1: TAKEOFF =====================
    print("=== PHASE 1: Arm + Takeoff ===\n")

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
        print("\n✗ No drones armed. Exiting.")
        return

    airborne = []
    for drone, name in drones:
        if await wait_airborne(drone, name):
            airborne.append((drone, name))

    if len(airborne) < 3:
        print("\n✗ Not all drones airborne. Exiting.")
        return

    print("\n=== All drones airborne — entering OFFBOARD ===\n")

    # ===================== PHASE 2: OFFBOARD =====================
    successful = []
    for i, (drone, name) in enumerate(airborne):
        ok = await enter_offboard_and_climb(drone, name, target_alt=5.0)
        if ok:
            successful.append((drone, name))

        if i < len(airborne) - 1:
            await asyncio.sleep(1.0)

    if len(successful) < 3:
        print("\n✗ Not all drones reached OFFBOARD. Exiting.")
        return

    leader = successful[0][0]
    wingmen = successful[1:]

    # ===================== CAPTURE MISSION HEADING =====================
    # This is the direction they were facing when the script first got them into stable offboard.
    # Per requirements: the formation will fly this heading the entire time (no forced 0°).
    leader_att = await leader.telemetry.attitude_euler().__anext__()
    mission_yaw = leader_att.yaw_deg

    print(f"\n=== Mission heading captured: {mission_yaw:.1f}° (all aircraft will hold this yaw) ===\n")

    # ===================== PHASE 2.5: YAW ALIGNMENT =====================
    print("=== PHASE 2.5: Align all drones to mission heading ===\n")

    await asyncio.gather(
        align_yaw(leader, "Drone 1 (Leader)", mission_yaw),
        align_yaw(wingmen[0][0], wingmen[0][1], mission_yaw),
        align_yaw(wingmen[1][0], wingmen[1][1], mission_yaw)
    )

    print("\n=== Yaw alignment complete — moving to isosceles triangle formation ===\n")

    # ===================== PHASE 3: FORMATION (ISOSCELES TRIANGLE) =====================
    print("=== PHASE 3: Move to Isosceles Triangle Formation ===\n")

    # Isosceles triangle: leader at tip, wingmen behind-left and behind-right.
    # Offsets are in body frame (forward, right) and will be rotated by mission_yaw.
    # Tighter formation (user request)
    FORMATION_BEHIND_M = 5.0
    FORMATION_LATERAL_M = 4.0
    FORMATION_ALT_M = 7.0

    body_offsets = [
        (-FORMATION_BEHIND_M, -FORMATION_LATERAL_M),   # Drone 2 - Left wing
        (-FORMATION_BEHIND_M,  FORMATION_LATERAL_M),   # Drone 3 - Right wing
    ]

    leader_pos = await leader.telemetry.position_velocity_ned().__anext__()

    # Compute absolute NED targets for each wingman by rotating body offsets
    formation_targets = []
    for i, (fwd, rgt) in enumerate(body_offsets):
        dn, de = body_offset_to_ned(fwd, rgt, mission_yaw)
        target_n = leader_pos.position.north_m + dn
        target_e = leader_pos.position.east_m + de
        formation_targets.append((target_n, target_e))

    # Move wingmen in parallel to their computed positions, arriving with correct yaw
    await asyncio.gather(
        move_to_formation_position(
            wingmen[0][0], wingmen[0][1],
            formation_targets[0][0], formation_targets[0][1],
            desired_yaw=mission_yaw,
            altitude=FORMATION_ALT_M,
            timeout=60.0
        ),
        move_to_formation_position(
            wingmen[1][0], wingmen[1][1],
            formation_targets[1][0], formation_targets[1][1],
            desired_yaw=mission_yaw,
            altitude=FORMATION_ALT_M,
            timeout=60.0
        )
    )

    # Give the formation time to fully settle before starting dynamic flight
    print("\n[Formation] Settling in position for 3.5 seconds...")
    await asyncio.sleep(3.5)

    print("\n=== Formation stable — starting true dynamic leader-follower flight ===\n")

    # ===================== PHASE 4: DYNAMIC LEADER-FOLLOWER FLIGHT =====================
    await leader_follower_flight(
        leader,
        wingmen,
        mission_yaw=mission_yaw,
        body_offsets=body_offsets,
        speed_mps=2.0,
        duration_s=20.0,
        formation_alt=FORMATION_ALT_M,
        update_hz=15.0
    )

    print("\n=== Landing all drones in parallel ===\n")

    # ===================== PHASE 5: LAND (PARALLEL) =====================
    # All three drones land simultaneously for faster mission completion.
    await asyncio.gather(*[land_and_cleanup(drone, name) for drone, name in successful])

    print("\n=== Mission Complete ===\n")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nExiting.")