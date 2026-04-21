from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import Interdrone
import asyncio
import argparse


async def main():
    # Get drone ID arg
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

    drone: Drone = Drone()
    flight_settings: FlightSettings = FlightSettings.from_mission_config(
        self_id=args.id
    )
    interdrone: Interdrone = Interdrone(flight_settings=flight_settings, drone=drone)
    interdroneTask = asyncio.create_task(interdrone.start_interdrone())
    # Call ping here and get response

    while True:
        ping_worked: bool = await interdrone.ping_drones()
        if ping_worked:
            print("Ping succeeded. Exiting loop")
            break
        else:
            print("Ping failed. Trying again")
        await asyncio.sleep(0.1)
    try:
        # Keep the networking loop alive
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Networking shutting down...")
        interdroneTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
