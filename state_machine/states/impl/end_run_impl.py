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
        if pf is not None:
            # Re-syncs mission_path with this Pathfinder's own final route
            # -- a wholesale REPLACE, not another append. CalcScanPath's
            # incremental extend_mission_path calls are a best-effort
            # progress snapshot, taken as each stretch was confirmed; they
            # are not still accurate by the time ExpandNodes has run.
            # ExpandNodes' own increase_radius (the final competition-
            # safety-margin growth) can shift EXISTING node positions
            # outward even without changing the route's node count or
            # structure -- confirmed directly: a 4-node confirmed+approach
            # route came out of a real simulated run with the same 4-node
            # shape before and after ExpandNodes, but different lat/lon,
            # since the nodes themselves moved. get_maze_path() here is
            # this Pathfinder's own, now-final, post-repair route -- the
            # authoritative answer mission_path needs to hold once the
            # mission concludes.
            #
            # This replaces mission_path with ONLY this Pathfinder's own
            # contribution -- correct today (nothing else populates it),
            # but will need to become a "replace my own portion, keep
            # anything relayed from elsewhere" merge once cross-pair relay
            # (two-pair missions) actually feeds mission_path from a
            # SECOND Pathfinder that this drone has no live object for.
            self.drone.mission_path = [
                pf.coord_converter.local_to_latlon(n.x, n.y) for n in pf.get_maze_path()
            ]
        flight_log.event(
            "end_run",
            confirmed_path_len=len(pf.maze_confirmed_path) if pf is not None else None,
            mission_path_len=len(self.drone.mission_path),
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
