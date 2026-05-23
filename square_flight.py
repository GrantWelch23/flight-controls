import asyncio
import sys
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

# ====================== Fly Forward Function (10m) ====================

async def fly_forward(drone, distance_m=10.0):
    """Fly forward in the direction the drone is currently facing"""
    print(f"Flying forward {distance_m} meters...")

    # Get current position
    ned = await drone.telemetry.position_velocity_ned().__anext__()
    current_north = ned.position.north_m
    current_east = ned.position.east_m
    current_down = ned.position.down_m

    # Get current yaw (direction the drone is facing)
    attitude = await drone.telemetry.attitude_euler().__anext__()
    current_yaw = attitude.yaw_deg

    # Calculate target position based on current yaw
    yaw_rad = math.radians(current_yaw)
    target_north = current_north + distance_m * math.cos(yaw_rad)
    target_east = current_east + distance_m * math.sin(yaw_rad)

    # Move to the new target
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            north_m=target_north,
            east_m=target_east,
            down_m=current_down,
            yaw_deg=current_yaw
        )
    )

    # Live updating distance
    for _ in range(150):  # Max ~15 seconds
        current_ned = await drone.telemetry.position_velocity_ned().__anext__()
        dn = current_ned.position.north_m - current_north
        de = current_ned.position.east_m - current_east
        distance_traveled = math.sqrt(dn**2 + de**2)

        print(f"Distance traveled: {distance_traveled:.2f}m / {distance_m:.2f}m", end="\r")

        if distance_traveled >= distance_m - 0.1:
            print(f"\n✓ Reached target distance: {distance_traveled:.2f}m")
            break

        await asyncio.sleep(0.1)

# ====================== Rotate Function (90 degrees) ======================

async def rotate_90(drone):
    """Rotate 90 degrees clockwise with precision"""
    print("Rotating 90 degrees...")

    # Get current yaw
    attitude = await drone.telemetry.attitude_euler().__anext__()
    current_yaw = attitude.yaw_deg

    # Calculate target yaw (+90 degrees)
    target_yaw = current_yaw + 90
    if target_yaw > 180:
        target_yaw -= 360
    elif target_yaw < -180:
        target_yaw += 360

    print(f"Current yaw: {current_yaw:.1f}° → Target yaw: {target_yaw:.1f}°")

    # Rotate while monitoring actual yaw
    while True:
        attitude = await drone.telemetry.attitude_euler().__anext__()
        current_yaw = attitude.yaw_deg

        yaw_error = target_yaw - current_yaw
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        print(f"Current yaw: {current_yaw:.1f}° | Error: {yaw_error:.1f}°", end="\r")

        if abs(yaw_error) < .8:
            print(f"\n✓ Reached target yaw: {current_yaw:.1f}°")
            break

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=0.0,
                yawspeed_deg_s=25.0
            )
        )

        await asyncio.sleep(0.1)

    # Stop rotating
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    print("✓ Rotation complete")

# ====================== Flight Plan Start ======================

async def run():
    print("=== Flight Started ==")
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

    # Wait 3 seconds so takeoff can begin
    await asyncio.sleep(3)

    print("Entering OFFBOARD mode")

    # Set an initial upward velocity setpoint 
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=-1.0,        # Negative = climb upward
            yawspeed_deg_s=0.0
        )
    )

    # OFFBOARD mode 
    await drone.offboard.start()
    print("✓ OFFBOARD mode started")
    print("Take off initiating...")

    # Climb until we reach ~5 meters
    print("Climbing to safe altitude...")
    while True:
        position = await drone.telemetry.position().__anext__()
        current_alt = position.relative_altitude_m
        print(f"Current altitude: {current_alt:.2f}m", end="\r")

        if current_alt >= 5.0:
            print(f"\n✓ Reached target altitude: {current_alt:.2f}m")
            break

        await asyncio.sleep(0.1)

    # ====================== SQUARE PATTERN ======================
    print("\n=== Starting Square Pattern ===")

    for i in range(4):
        print(f"\n--- Side {i+1}/4 ---")
        await fly_forward(drone, distance_m=10.0)
        await rotate_90(drone)

    print("\n=== Square Pattern Complete ===")
    # ========================================================

    print("Stopping and landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Landing command sent")

    print("=== Script complete ===")


if __name__ == "__main__":
    asyncio.run(run())