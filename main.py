from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import Interdrone
import asyncio
import argparse


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()
    print("Hello from iarc-10!")
    drone: Drone = Drone()
    flight_settings: FlightSettings = FlightSettings()
    interdrone: Interdrone = Interdrone(flight_settings=flight_settings, drone=drone)
    interdroneTask = asyncio.create_task(interdrone.start_interdrone())
    try:
        # Keep the networking loop alive
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Networking shutting down...")
        interdroneTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
