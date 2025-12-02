"""Implements the behavior of the AppShare state."""

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
from state_machine.states.recall import Recall


async def run(self: AppShare) -> State:
    """
    Implements the run method for the AppShare state.

    This method initiates the app share process and transitions to the Scan or Recall state.

    Returns
    -------
    Scan/Recall : State
        The next state after a successful App share.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the AppShare state is canceled.

    Notes
    -----
    This method is responsible for sharing data with the app and transitioning it to the
    scan state or the recall state, which represent the phase to scan for mines and the state to move the drone to the landing coordinates respectively.

    """
    try:
        update_state("AppShare")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("AppShare state running")

        # App share code here


        # needs to be conditional, go to scan or recall
        # return Scan(self.drone, self.flight_settings)
    

    except asyncio.CancelledError as ex:
        logging.error("AppShare state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the AppShare class to the run function
AppShare.run_callable = run
