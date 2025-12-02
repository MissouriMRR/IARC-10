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
from state_machine.states.app_share import AppShare
from state_machine.states.scan import Scan


async def run(self: AppShare) -> State:
    """
    Implements the run method for the AppShare state.

    This method initiates the app share process and transitions to the Scan state.

    Returns
    -------
    Scan : State
        The next state after a successful App share.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the AppShare state is canceled.

    Notes
    -----
    This method is responsible for sharing data with the app and transitioning it to the
    scan state, which represents the phase to scan for mines.

    """
    try:
        update_state("AppShare")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("AppShare state running")

        # Set takeoff altitude to the minimum allowed altitude, plus one meter
        takeoff_altitude: float = (
            extract_gps(self.flight_settings.mission_data_path).altitude_limits.min_altitude + 1.0
        )
        await self.drone.takeoff(takeoff_altitude)

        return Scan(self.drone, self.flight_settings)
    except asyncio.CancelledError as ex:
        logging.error("AppShare state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Takeoff class to the run function
AppShare.run_callable = run
