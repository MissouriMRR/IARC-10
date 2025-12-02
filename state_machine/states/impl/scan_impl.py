"""Implements the behavior of the Takeoff state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_state,
    update_drone,
    update_flight_settings,
)
from state_machine.states.state import State
from state_machine.states.scan import Scan
from state_machine.states.drone_share import DroneShare


async def run(self: Scan) -> State:
    """
    Implements the run method for the Scan state.

    This method initiates the drone scan process and transitions to the DroneShare state.

    Returns
    -------
    DroneShare : State
        The next state after a successful scan.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the Scan state is canceled.

    Notes
    -----
    This method is responsible for taking off the drone and transitioning it to the
    DroneShare state, which represents the state where the drone shares information with other drones.

    """
    try:
        update_state("Scan")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("Scan state running")

        # Set takeoff altitude to the minimum allowed altitude, plus one meter
        takeoff_altitude: float = (
            extract_gps(self.flight_settings.mission_data_path).altitude_limits.min_altitude + 1.0
        )
        await self.drone.takeoff(takeoff_altitude)

        return DroneShare(self.drone, self.flight_settings)
    except asyncio.CancelledError as ex:
        logging.error("Takeoff state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Takeoff class to the run function
Scan.run_callable = run
