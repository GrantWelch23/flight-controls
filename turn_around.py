import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw


# ====================== Rotate Function ======================
async def rotate_180(drone):
    print("Rotating 180 degrees precisely...")

    # Get current yaw
    attitude = await drone.telemetry.attitude_euler().__anext__()
    current_yaw = attitude.yaw_deg

    # Calculate target yaw (+180 degrees)
    target_yaw = current_yaw + 180
    if target_yaw > 180:
        target_yaw -= 360
    elif target_yaw < -180:
        target_yaw += 360

    print(f"Current yaw: {current_yaw:.1f}° → Target yaw: {target_yaw:.1f}°")

    # Rotate while monitoring actual yaw
    while True:
        attitude = await drone.telemetry.attitude_euler().__anext__()
        current_yaw = attitude.yaw_deg

        # Calculate how far we are from target
        yaw_error = target_yaw - current_yaw
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        # Live update
        print(f"Current yaw: {current_yaw:.1f}° | Error: {yaw_error:.1f}°", end="\r")

        # Stop when we're within ±8 degrees of target
        if abs(yaw_error) < 8:
            print(f"\n✓ Reached target yaw: {current_yaw:.1f}°")
            break

        # Continue rotating (adjust speed as needed)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=0.0,
                yawspeed_deg_s=25.0   # Moderate speed for precision
            )
        )

        await asyncio.sleep(0.1)

    # Stop rotating
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    print("✓ Rotation complete")

# =========================Script Start=============================

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

    # Call the rotate function
    await rotate_180(drone)

    print("Stopping and landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Landing command sent")

    print("=== Script complete ===")


if __name__ == "__main__":
    asyncio.run(run())