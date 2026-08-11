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

    This method initiates the app share process and transitions to the Recall state.

    Returns
    -------
    Recall : State
        The next state after a successful App share.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the AppShare state is canceled.

    Notes
    -----
    Only ever reached from EndRun now (mission genuinely over -- either
    the scan finished and ExpandNodes hardened its route, or the flight
    time budget ran out), so unlike this state's own older docstring,
    there's no longer a "go back to Scan" branch to be conditional
    about -- Recall (fly to the landing coordinates) is the only next
    state.
    """
    try:
        update_state("AppShare")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("AppShare state running")

        # PLACEHOLDER: share this mission's final results (discovered
        # mines, flown coverage) with the app here -- see
        # MessageType.SEND_PATHS_TO_APP/SEND_GROUND_TRUTH_COORDS in
        # interdrone_communication/message_types.py for the closest
        # existing message shapes. Left deliberately unimplemented for
        # now, same as the interdrone relay work elsewhere in this file.

        return Recall(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("AppShare state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the AppShare class to the run function
AppShare.run_callable = run
