import asyncio
import sys
import math
import time
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

# ====================== Rotate to Yaw Function =================

async def rotate_to_yaw(drone, target_yaw, rotation_speed=25.0, tolerance=2.0, relative=False):
    """Rotate to a specific yaw angle (absolute or relative)."""
    
    if relative:
        attitude = await drone.telemetry.attitude_euler().__anext__()
        current_yaw = attitude.yaw_deg
        target_yaw = current_yaw + target_yaw   # treat input as relative offset

    print(f"Rotating to target yaw: {target_yaw:.1f}°...")

    while True:
        attitude = await drone.telemetry.attitude_euler().__anext__()
        current_yaw = attitude.yaw_deg

        yaw_error = target_yaw - current_yaw
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        print(f"Current yaw: {current_yaw:.1f}° | Error: {yaw_error:.1f}°", end="\r")

        if abs(yaw_error) < tolerance:
            print(f"\n✓ Reached target yaw: {current_yaw:.1f}°")
            break

        direction = 1 if yaw_error > 0 else -1
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=0.0,
                yawspeed_deg_s=rotation_speed * direction
            )
        )

        await asyncio.sleep(0.1)

    # Stop rotation
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(0.3)


# ====================== Fly In A Circle Function =================

async def fly_circle(drone, radius_m=10.0, speed_mps=3.0, duration_s=30.0):
    """Fly a smooth circle using constant forward speed + yaw rate."""
    print(f"\n=== Starting Circle ===")
    print(f"Radius: {radius_m}m | Speed: {speed_mps}m/s | Duration: {duration_s}s")

    # Calculate required yaw rate (deg/s)
    yaw_rate_dps = (speed_mps / radius_m) * (180 / math.pi)
    print(f"Calculated yaw rate: {yaw_rate_dps:.1f}°/s")

    # Rotate LEFT 90° (relative) so we start facing tangent to the rightward circle
    await rotate_to_yaw(drone, -90, relative=True)   

    # Start circle flight
    print("Flying circle...")

    start_time = time.perf_counter()

    while True:
        #Command the drone to fly forward while constantly yawing
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=speed_mps,
                right_m_s=0.0,
                down_m_s=0.0,
                yawspeed_deg_s=yaw_rate_dps 
            )
        )

        #Track how long we've been flying the circle
        elapsed = time.perf_counter() - start_time
        print(f"Circle time: {elapsed:.1f}s / {duration_s}s", end="\r")

        #Stop after 30s
        if elapsed >= duration_s:
            print(f"\n✓ Circle complete ({duration_s}s)")
            break

        await asyncio.sleep(0.1)

    # Stop the drone
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    print("✓ Circle stopped\n")


# ====================== Mission Start ======================

async def run():
    print("=== Flight Started ===")
    sys.stdout.flush()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Drone connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("✓ Global position estimate OK")
            break

    print("Arming...")
    await drone.action.arm()
    print("✓ Drone is armed!")

    print("Taking off...")
    await drone.action.takeoff()
    print("✓ Takeoff command sent")

    await asyncio.sleep(3)

    print("Entering OFFBOARD mode")

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=-1.0,
            yawspeed_deg_s=0.0
        )
    )

    await drone.offboard.start()
    print("✓ OFFBOARD mode started")

    # Climb to safe altitude
    print("Climbing to safe altitude...")
    while True:
        position = await drone.telemetry.position().__anext__()
        current_alt = position.relative_altitude_m
        print(f"Current altitude: {current_alt:.2f}m", end="\r")

        if current_alt >= 5.0:
            print(f"\n✓ Reached target altitude: {current_alt:.2f}m")
            break

        await asyncio.sleep(0.1)

    # ====================== FLY CIRCLE ======================
    await fly_circle(drone, radius_m=10.0, speed_mps=3.0, duration_s=30.0)

    # ====================== Land =========================
    print("Stopping and landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Landing command sent")

    print("=== Script complete ===")


if __name__ == "__main__":
    asyncio.run(run())