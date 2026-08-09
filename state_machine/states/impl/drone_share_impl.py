"""Implements the behavior of the DroneShare state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.drone_share import DroneShare
from state_machine.states.state import State


async def run(self: DroneShare) -> State:
    """
    Implements the run method for the DroneShare state.

    This method initiates the drone to drone data sharing process and transitions to the CalcScanPath state.

    Returns
    -------
    CalcScanPath : State
        The next state after a successful drone communication.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the DroneShare state is canceled.

    Notes
    -----
    This method is responsible for commmunicating with other drones and transitioning it to the
    CalcScanPath state, which represents the state where the next scan path is calculated and a valid path is checked for.

    """
    try:
        update_state("DroneShare")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("DroneShare state running")

        for i in range(1, 5):
            if self.drone.id == i:
                # Broadcast new minedata and new sight data
                broadcast()
            else:
                # Recieve new minedata and new sight data
                recieve()
        # Add drone share code here

        return CalcScanPath(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("DroneShare state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the DroneShare class to the run function
DroneShare.run_callable = run
