import asyncio
import sys
import math
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


# ====================== Fly Forward Function (Smooth) ====================

async def fly_forward(drone, distance_m=10.0, max_speed=2.0):
    """Fly forward smoothly with gradual acceleration (no jerky movements)."""
    print(f"Flying forward {distance_m} meters (smooth acceleration)...")

    # Get current position and yaw
    ned = await drone.telemetry.position_velocity_ned().__anext__()
    current_north = ned.position.north_m
    current_east = ned.position.east_m
    current_down = ned.position.down_m

    attitude = await drone.telemetry.attitude_euler().__anext__()
    current_yaw = attitude.yaw_deg

    # Calculate target position
    yaw_rad = math.radians(current_yaw)
    target_north = current_north + distance_m * math.cos(yaw_rad)
    target_east = current_east + distance_m * math.sin(yaw_rad)

    # Use velocity control with gradual acceleration for smoother flight
    start_time = asyncio.get_running_loop().time()
    acceleration_time = 2.0  # seconds to reach full speed

    while True:
        elapsed = asyncio.get_running_loop().time() - start_time

        # Gradual acceleration (0 → max_speed over 2 seconds)
        if elapsed < acceleration_time:
            current_speed = (elapsed / acceleration_time) * max_speed
        else:
            current_speed = max_speed

        # Fly forward at current speed
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(current_speed, 0.0, 0.0, 0.0)
        )

        # Check distance traveled
        current_ned = await drone.telemetry.position_velocity_ned().__anext__()
        dn = current_ned.position.north_m - current_north
        de = current_ned.position.east_m - current_east
        distance_traveled = math.sqrt(dn**2 + de**2)

        print(f"Distance traveled: {distance_traveled:.2f}m / {distance_m:.2f}m", end="\r")

        if distance_traveled >= distance_m - 0.5:
            print(f"\n✓ Reached target distance: {distance_traveled:.2f}m")
            break

        await asyncio.sleep(0.1)

    # Stop the drone
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.5)


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

    # Start battery safety monitor
    battery_task = asyncio.create_task(battery_monitor(drone, "Drone 1"))

    print("Entering OFFBOARD mode")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
    )
    await drone.offboard.start()
    print("✓ OFFBOARD mode started")

    # Climb to safe altitude (now 10m)
    print("Climbing to safe altitude...")
    while True:
        position = await drone.telemetry.position().__anext__()
        current_alt = position.relative_altitude_m
        print(f"Current altitude: {current_alt:.2f}m", end="\r")

        if current_alt >= 10.0:
            print(f"\n✓ Reached target altitude: {current_alt:.2f}m")
            break
        await asyncio.sleep(0.1)

    # ====================== FLY FORWARD (SMOOTH) ==================
    await fly_forward(drone, distance_m=10.0, max_speed=2.0)

    # ====================== RETURN TO LAUNCH ======================
    print("Stopping and returning to launch...")
    await drone.offboard.stop()
    await drone.param.set_param_float("RTL_RETURN_ALT", 10.0)
    await drone.action.return_to_launch()
    print("✓ Return to Launch command sent")

    await asyncio.sleep(25)
    print("=== Mission Complete ===")

    battery_task.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n\nExiting.")