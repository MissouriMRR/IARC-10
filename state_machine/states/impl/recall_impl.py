"""Implements the behavior of the Recall state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.land import Land
from state_machine.states.recall import Recall
from state_machine.states.state import State


async def run(self: Recall) -> State:
    """
    Implements the run method for the Recall state.

    This method initiates the recall process and transitions to the Land state.

    Returns
    -------
    Land : State
        The next state after a successful recall.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the Recall state is canceled.

    Notes
    -----
    This method is responsible for navigating the drone to the landing point and transitioning it to the
    land state, which represents the phase to land.

    """
    try:
        update_state("Recall")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("Recall state running")

        self.drone.recall()

        return Land(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Recall state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Recall class to the run function
Recall.run_callable = run
