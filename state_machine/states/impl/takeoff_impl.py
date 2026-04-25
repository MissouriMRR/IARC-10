"""Implements the behavior of the Takeoff state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.land import Land
from state_machine.states.state import State
from state_machine.states.takeoff import Takeoff
from state_machine.interdrone import CMD_MSG, get_input


async def run(self: Takeoff) -> State:
    """
    Implements the run method for the Takeoff state.

    This method initiates the drone takeoff process and transitions to the Land state.

    Returns
    -------
    Land : State
        The next state after a successful takeoff.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the Takeoff state is canceled.

    Notes
    -----
    This method is responsible for taking off the drone and transitioning it to the
    land state, which represents the navigation phase to land the drone.

    """
    try:
        update_state("Takeoff")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("Takeoff state running")

        while True:
            if (
                self.interdrone.CMD_MSG == CMD_MSG.MISSION
                or self.interdrone.CMD_MSG == CMD_MSG.DEMO
                or self.interdrone.cmd_msg == CMD_MSG.TAKEOFF
            ):
                logging.info("Mission command received. Initiating takeoff.")
                break

        await self.drone.takeoff(takeoff_altitude)

        return InitialCalcScanPath(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Takeoff state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Takeoff class to the run function
Takeoff.run_callable = run
