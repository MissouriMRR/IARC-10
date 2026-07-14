import asyncio
import logging

from flight.circlePath import circle_waypoints
from flight.lidar import lidar_approach_is_safe
from state_machine.state_tracker import (
    update_state,
    update_drone,
    update_flight_settings,
)
from state_machine.states.land import Land
from state_machine.states.lidar_map import LidarMap
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

        self.drone.vehicle.airspeed = 20

        # Only generate a fresh path when there is no backlog: after a
        # LidarMap diversion this state is re-entered with the remaining
        # waypoints still queued on the drone
        if not self.drone.waypoints:
            location = (
                self.drone.vehicle.location.global_relative_frame.lat,
                self.drone.vehicle.location.global_relative_frame.lon,
            )
            circleWaypoints = []
            for i in range(10):
                circleWaypoints.extend(circle_waypoints(*location, 10, drone_id=self.drone.id))
            self.drone.updateWaypoints(circleWaypoints)

            await self.interdrone.send_new_waypoints(
                tuple(self.flight_settings.other_drones_in_mission), circleWaypoints[:5]
            )
            for state in self.interdrone.drone_states:
                self.drone.checkForCollision(state.list_of_waypoints)

        while self.drone.waypoints:
            curWaypoint = await self.drone.gotoWaypoint()
            await self.interdrone.reached_waypoint(
                tuple(self.flight_settings.other_drones_in_mission), curWaypoint
            )
            for drone in self.interdrone.drone_states:
                await self.interdrone.send_new_waypoints(
                    tuple(self.flight_settings.other_drones_in_mission), [curWaypoint]
                )

                self.drone.checkForCollision(drone.list_of_waypoints)

            # Divert to map a LIDAR-detected object once the current leg is
            # done, but only if the approach to it is clear
            if self.drone.lidar is not None and self.drone.lidar.scan_pending:
                if lidar_approach_is_safe(self.drone.lidar, self.drone.vehicle):
                    return LidarMap(
                        self.drone,
                        self.flight_settings,
                        self.interdrone,
                        resume_state=POIF(self.drone, self.flight_settings, self.interdrone),
                    )
                self.drone.lidar.clear_pending()

        return Land(self.drone, self.flight_settings, self.interdrone)

    except asyncio.CancelledError as ex:
        logging.error("Land state canceled")
        raise ex


# Setting the run_callable attribute of the Land class to the run function
POIF.run_callable = run
