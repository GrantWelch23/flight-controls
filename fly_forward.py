import asyncio
import sys
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

# ====================== Fly Forward Function ====================

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

        if distance_traveled >= distance_m - 0.3:
            print(f"\n✓ Reached target distance: {distance_traveled:.2f}m")
            break

        await asyncio.sleep(0.1)

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

    # ====================== FLY FORWARD ==================
    await fly_forward(drone, distance_m=10.0)
    
    # ====================== Land =========================

    print("Stopping and landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Landing command sent")

    print("=== Script complete ===")


if __name__ == "__main__":
    asyncio.run(run())