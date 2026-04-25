from flight.waypoint import Waypoint
from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import Interdrone
from state_machine.drone_state import DroneState
import asyncio
import argparse


async def main():
    # Get drone ID arg
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

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
    interdrone: Interdrone = Interdrone(
        flight_settings=flight_settings, drone=drone, drone_states=drone_states
    )
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
    # Try to arm once ping works
    if flight_settings.current_drone_ID == 1:
        print("trying to send arm")
        await interdrone.send_ARM(
            dronesToSendData=tuple(
                interdrone.flight_settings.other_drones_in_mission,
            )
        )
        while not await interdrone.all_armed():
            await asyncio.sleep(0.1)
        print("All drones are armed!")

        print("Trying to takeoff")
        await interdrone.send_takeoff(
            dronesToSendData=tuple(
                interdrone.flight_settings.other_drones_in_mission,
            )
        )
        while not await interdrone.all_takeoff():
            await asyncio.sleep(0.1)
        # TODO test start mission
        # TODO test start demo
        # Test waypoints
        waypoints: list[Waypoint] = [
            Waypoint(1, flight_settings.current_drone_ID, 1.0, 1.0),
            Waypoint(2, flight_settings.current_drone_ID, 1.0, 1.0),
            Waypoint(3, flight_settings.current_drone_ID, 1.0, 1.0),
        ]
        await interdrone.add_waypoints(
            tuple(
                flight_settings.other_drones_in_mission,
            ),
            waypoints=waypoints,
        )
    try:
        # Keep the networking loop alive
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Networking shutting down...")
        interdroneTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
