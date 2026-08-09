"""Implements the behavior of the Scan state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.drone_share import DroneShare
from state_machine.states.scan import Scan
from state_machine.states.state import State


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

        self.drone.completeTasks()
        # Add scan code here

        return DroneShare(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Takeoff state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Scan class to the run function
Scan.run_callable = run
