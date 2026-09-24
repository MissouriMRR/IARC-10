"""Implements the behavior of the AppShare state."""

import asyncio
import logging

import flight.flight_log as flight_log
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

        # The mission's own final route -- EndRun already resynced
        # self.drone.mission_path with this Pathfinder's now-final,
        # post-ExpandNodes route (see its own docstring on why that's a
        # wholesale replace, not another append) before transitioning
        # here. A no-op for anything but drone 1 (push_mission_path's own
        # gate) and for an ASSISTANT (mission_path stays empty its whole
        # life -- see configureField).
        #
        # PLACEHOLDER still: sharing discovered-mine data specifically
        # with the app isn't implemented -- see SEND_GROUND_TRUTH_COORDS
        # in interdrone_communication/message_types.py for the closest
        # existing message shape, if that's the intended vehicle.
        await self.interdrone.push_mission_path()
        _write_path_file(self.drone)

        return Recall(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("AppShare state canceled")
        raise ex
    finally:
        pass


def _write_path_file(drone) -> None:
    """Writes this drone's own final path to
    Logs/<run_id>/drone_<id>_path.txt -- a persistent, on-disk record of
    what this drone actually computed, independent of whether
    SEND_PATHS_TO_APP's interdrone message ever reaches the app (or
    there's even an app connected at all, e.g. a bench test run).

    Unlike push_mission_path, NOT restricted to drone 1 -- every
    GAMBLER/SOLOGAMBLER gets its own file (an ASSISTANT's mission_path
    stays empty its whole life and it never sets no_valid_path_found
    either -- see configureField/_check_no_valid_path -- so the guard
    below already skips it with no extra role check needed). A local
    debug record has no reason to inherit "only drone 1 talks to the
    app"'s reporting convention.
    """
    if not drone.mission_path and not drone.no_valid_path_found:
        return
    try:
        path_file = flight_log.run_dir() / f"drone_{drone.id}_path.txt"
    except RuntimeError:
        # flight_log.configure() was never called (e.g. a test harness
        # that never set FLIGHT_LOG_RUN/drives states directly) -- no run
        # directory exists to write into.
        logging.warning("could not write path file: flight_log not configured")
        return
    if drone.no_valid_path_found:
        path_file.write_text(
            "NO VALID PATH FOUND -- no route existed from the field entry to "
            "any ending node (field may be fully mine-blocked).\n"
        )
    else:
        path_file.write_text(drone.get_mission_iarc_path() + "\n")
    logging.info("wrote path file: %s", path_file)


# Setting the run_callable attribute of the AppShare class to the run function
AppShare.run_callable = run
