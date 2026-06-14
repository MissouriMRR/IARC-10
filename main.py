from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import CMD_MSG, Interdrone
from state_machine.drone_state import DroneState
import asyncio
import argparse


async def main():
    # Get drone ID arg
    parser = argparse.ArgumentParser()  # TODO MOVE ARG PARSER TO FLIGHT SETTINGS
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

    print("Hello from iarc-10!")
    drone: Drone = Drone()
    flight_settings: FlightSettings = FlightSettings.from_mission_config(self_id=args.id)
    # Create drone_state to access state of other drones in the test
    drone_states: list[DroneState] = (
        []
    )  # TODO TALK TO HARPER. MAY NOT NEED DRONE STATE AT STATE MACHINE LEVEL. IF SO MOVE TO INTERDRONE
    for id in flight_settings.other_drones_in_mission:
        drone_states.append(
            DroneState(
                drone_id=id,
                drone_ip=next(d["IP"] for d in flight_settings.drone_info if d["id"] == id),
            )
        )
    interdrone: Interdrone = Interdrone(flight_settings=flight_settings, drone=drone)
    interdroneTask = asyncio.create_task(interdrone.start_interdrone())

    try:
        # Keep the networking loop alive
        while True:
            ping_worked: bool = await interdrone.ping_drones()
            if ping_worked:
                print("Ping succeeded. Exiting loop")
                break
            else:
                print("Ping failed. Trying again")
            await asyncio.sleep(0.1)
        # Try to arm once ping works
        current_cmd_msg = CMD_MSG.NONE
        while True:
            # Check for change in cmd msg and print it
            if interdrone.get_cmd_msg() != current_cmd_msg:
                current_cmd_msg = interdrone.get_cmd_msg()
                print(current_cmd_msg)
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Networking shutting down...")
        interdroneTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
