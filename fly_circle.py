import asyncio
import sys
import math
import time
from config import CONNECTION_STRING
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw


# ====================== Safety Monitor ======================

async def battery_monitor(drone, name, low_battery_threshold=25.0):
    """Background task: RTL if battery drops below threshold."""
    while True:
        try:
            battery = await drone.telemetry.battery().__anext__()
            percent = battery.remaining_percent * 100

            if percent < low_battery_threshold:
                print(f"\n[{name}] ⚠ LOW BATTERY ({percent:.1f}%) — Returning to Launch!")
                await drone.action.return_to_launch()
                return
        except Exception:
            pass
        await asyncio.sleep(5.0)


# ====================== Rotate to Yaw Function =================

async def rotate_to_yaw(drone, target_yaw, rotation_speed=25.0, tolerance=2.0, relative=False):
    """Rotate to a specific yaw angle (absolute or relative)."""
    
    if relative:
        attitude = await drone.telemetry.attitude_euler().__anext__()
        current_yaw = attitude.yaw_deg
        target_yaw = current_yaw + target_yaw

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
            VelocityBodyYawspeed(0.0, 0.0, 0.0, rotation_speed * direction)
        )
        await asyncio.sleep(0.1)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.3)


# ====================== Fly In A Circle Function =================

async def fly_circle(drone, radius_m=10.0, speed_mps=3.0, duration_s=30.0):
    print(f"\n=== Starting Circle ===")
    print(f"Radius: {radius_m}m | Speed: {speed_mps}m/s | Duration: {duration_s}s")

    yaw_rate_dps = (speed_mps / radius_m) * (180 / math.pi)
    print(f"Calculated yaw rate: {yaw_rate_dps:.1f}°/s")

    await rotate_to_yaw(drone, -90, relative=True)

    print("Flying circle...")
    start_time = time.perf_counter()

    while True:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(speed_mps, 0.0, 0.0, yaw_rate_dps)
        )

        elapsed = time.perf_counter() - start_time
        print(f"Circle time: {elapsed:.1f}s / {duration_s}s", end="\r")

        if elapsed >= duration_s:
            print(f"\n✓ Circle complete ({duration_s}s)")
            break

        await asyncio.sleep(0.1)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    print("✓ Circle stopped\n")


# ====================== Mission Start ======================

async def run():
    print("=== Flight Started ===")
    sys.stdout.flush()

    drone = System()
    await drone.connect(system_address=CONNECTION_STRING)

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

    # Start battery safety monitor
    battery_task = asyncio.create_task(battery_monitor(drone, "Drone 1"))

    print("Entering OFFBOARD mode")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
    )
    await drone.offboard.start()
    print("✓ OFFBOARD mode started")

    # Climb to safe altitude
    print("Climbing to safe altitude...")
    while True:
        position = await drone.telemetry.position().__anext__()
        current_alt = position.relative_altitude_m
        print(f"Current altitude: {current_alt:.2f}m", end="\r")

        if current_alt >= 10.0:
            print(f"\n✓ Reached target altitude: {current_alt:.2f}m")
            break
        await asyncio.sleep(0.1)

    # ====================== FLY CIRCLE ======================
    await fly_circle(drone, radius_m=10.0, speed_mps=3.0, duration_s=30.0)

    # ====================== RETURN TO LAUNCH ======================
    print("Stopping and returning to launch...")
    await drone.offboard.stop()
    await drone.param.set_param_float("RTL_RETURN_ALT", 10.0)
    await drone.action.return_to_launch()
    print("✓ Return to Launch command sent")

    # Wait for RTL to complete (drone will land automatically at home)
    await asyncio.sleep(30)
    print("=== Mission Complete ===")

    battery_task.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n\nExiting.")