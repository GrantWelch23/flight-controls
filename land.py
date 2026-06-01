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

    # Stop OFFBOARD mode first (important safety step)
    try:
        await drone.offboard.stop()
        print("✓ Stopped OFFBOARD mode")
    except:
        pass  # It's okay if OFFBOARD wasn't active

    print("Sending LAND command...")
    await drone.action.land()
    print("✓ Land command sent")

    # Give it a few seconds to descend
    await asyncio.sleep(8)
    print("=== Land script finished ===")


if __name__ == "__main__":
    asyncio.run(run())