import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

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

    print("Flying forward 10 meters...")

    # Stop any previous movement
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1.0)

    # Get current position
    ned = await drone.telemetry.position_velocity_ned().__anext__()
    start_north = ned.position.north_m
    current_east = ned.position.east_m
    current_down = ned.position.down_m

    # Set target 10 meters forward
    target_north = start_north + 10.0

    # Switch to position control
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            north_m=target_north,
            east_m=current_east,
            down_m=current_down,
            yaw_deg=0.0
        )
    )

    # Live updating distance
    for _ in range(150):  # Max 15 seconds
        current_ned = await drone.telemetry.position_velocity_ned().__anext__()
        start_north = current_ned.position.north_m
        distance = start_north - start_north

        print(f"Distance traveled: {distance:.2f}m / 10.00m", end="\r")

        if distance >= 9.5:
            print(f"\n✓ Reached target distance: {distance:.2f}m")
            break

        await asyncio.sleep(0.1)

    # Give it time to reach the target
    await asyncio.sleep(5)

    print("✓ Reached forward position")

    # Stop moving
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1)

    print("Stopping and landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Landing command sent")

    print("=== Script complete ===")


if __name__ == "__main__":
    asyncio.run(run())