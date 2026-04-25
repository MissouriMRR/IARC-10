import asyncio
import logging

from flight.circlePath import circle_waypoints
from state_machine.state_tracker import (
    update_state,
    update_drone,
    update_flight_settings,
)
from state_machine.states.land import Land
from state_machine.states.poif import POIF


async def run(self: POIF) -> None:
    """
    Implements the run method for the POIF state.

    This method handles the logic for the POIF state and transitions to the appropriate next state.

    Returns
    -------
    Start : State
        The next state after the drone has successfully landed.

    Notes
    -----
    This method is responsible for initiating the landing process of the drone and transitioning
    it back to the Start state, preparing for a new flight.

    """
    try:
        update_state("POIF")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("POIF state running")

        # Instruct the drone to land
        self.drone.vehicle.airspeed = 20
        location = (
            self.drone.vehicle.location.global_relative_frame.lat,
            self.drone.vehicle.location.global_relative_frame.lon,
        )

        circleWaypoints = circle_waypoints(
            *location, 10,drone_id=self.drone.id
        )
        self.drone.updateWaypoints(circleWaypoints[:5])
        circleWaypoints = circleWaypoints[5:]
        for state in self.interdrone.drone_states:
            self.drone.checkForCollision(state.list_of_waypoints)

        while True:
            curWaypoint = await self.drone.gotoWaypoint()
            await self.interdrone.reached_waypoint(curWaypoint)
            self.drone.updateWaypoints([circleWaypoints.pop(0)])
            for drone in self.interdrone.drone_states:
                send_new_waypoints = drone.list_of_waypoints[drone.next_waypoint_index :]
                self.drone.checkForCollision(drone.list_of_waypoints)

            if len(circleWaypoints) == 0:
                break

        return Land(self.drone, self.flight_settings, self.interdrone)

    except asyncio.CancelledError as ex:
        logging.error("Land state canceled")
        raise ex


# Setting the run_callable attribute of the Land class to the run function
POIF.run_callable = run
