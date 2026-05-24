import asyncio
import sys
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw

# ====================== Fly In A Circle Function =================

async def fly_circle(drone, radius_m=10.0, speed_mps=3.0, duration_s=30.0):
    """Fly a smooth circle using constant forward speed + yaw rate"""
    print(f"\n=== Starting Circle ===")
    print(f"Radius: {radius_m}m | Speed: {speed_mps}m/s | Duration: {duration_s}s")

    # Calculate required yaw rate (degrees per second)
    yaw_rate_dps = (speed_mps / radius_m) * (180 / math.pi)
    print(f"Calculated yaw rate: {yaw_rate_dps:.1f}°/s")

    # Start flying the circle
    print("Flying circle...")

    start_time = asyncio.get_event_loop().time()

    while True:
        # Send velocity command (forward + yaw rate)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=speed_mps,
                right_m_s=0.0,
                down_m_s=0.0,
                yawspeed_deg_s=yaw_rate_dps
            )
        )

        # Live status
        elapsed = asyncio.get_event_loop().time() - start_time
        print(f"Circle time: {elapsed:.1f}s / {duration_s}s", end="\r")

        # Check if we've flown long enough
        if elapsed >= duration_s:
            print(f"\n✓ Circle complete ({duration_s}s)")
            break

        await asyncio.sleep(0.1)

    # Stop the drone
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    print("✓ Circle stopped\n")

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

    # ====================== Circle Pattern ======================

    print("\n=== Starting Circle Pattern ===")

    # Fly the circle
    await fly_circle(
        drone,
        radius_m=10.0,      # Radius of the circle (meters)
        speed_mps=3.0,      # Forward speed (meters per second)
        duration_s=30.0     # How long to fly the circle (seconds)
    )

    print("\n=== Circle Pattern Complete ===")

    # ========================= Land =============================

    print("Stopping and landing...")
    await drone.offboard.stop()
    await drone.action.land()
    print("✓ Landing command sent")

    print("=== Script complete ===")


if __name__ == "__main__":
    asyncio.run(run())