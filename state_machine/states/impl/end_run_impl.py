"""Implements the behavior of the EndRun state."""

import asyncio
import logging

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.app_share import AppShare
from state_machine.states.end_run import EndRun
from state_machine.states.state import State


async def run(self: EndRun) -> State:
    """
    Implements the run method for the EndRun state.

    The mission is over -- either the scan finished and ExpandNodes
    hardened its route, or CalcScanPath/Scan cut the scan short because
    FlightSettings.max_flight_time was exceeded (see Drone.
    time_exceeded) -- either way, there's nothing more for this drone to
    search for. Logs whatever this drone's own Pathfinder ended up with
    (an ASSISTANT has none, see configureField) and transitions to
    AppShare to report it before heading home.

    Returns
    -------
    AppShare : State
        Always -- the mission's final results still need to reach the
        app before Recall/Land actually bring the drone down.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the EndRun state is canceled.
    """
    try:
        update_state("EndRun")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("EndRun state running")

        pf = Pathfinder.instance
        flight_log.event(
            "end_run",
            confirmed_path_len=len(pf.maze_confirmed_path) if pf is not None else None,
            time_exceeded=self.drone.time_exceeded(self.flight_settings.max_flight_time),
        )

        return AppShare(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("EndRun state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the EndRun class to the run function
EndRun.run_callable = run
