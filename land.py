import asyncio
from config import CONNECTION_STRING
from mavsdk import System

async def run():
    print("=== Emergency Land Script ===")
    
    drone = System()
    await drone.connect(system_address=CONNECTION_STRING)

    print("Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to drone")
            break

    print("Sending LAND command...")
    await drone.action.land()
    print("✓ Land command sent")

    # Wait for landing to complete
    print("Waiting for landing to complete...")
    for _ in range(60):
        try:
            in_air = await drone.telemetry.in_air().__anext__()
            if not in_air:
                print("✓ Landed safely")
                break
        except Exception:
            pass
        await asyncio.sleep(1.0)

    # Disarm
    print("Disarming...")
    await drone.action.disarm()
    print("✓ Disarmed")

    # Give PX4 time to fully reset its state (important!)
    print("Waiting for system to reset...")
    await asyncio.sleep(5)

    print("=== Land script finished ===")


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n\nExiting.")