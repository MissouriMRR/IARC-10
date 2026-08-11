"""Implements the behavior of the CalcScanPath state."""

import asyncio
import logging
import time

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from flight.waypoint import Waypoint
from state_machine.flight_settings import Role
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.end_run import EndRun
from state_machine.states.expand_nodes import ExpandNodes
from state_machine.states.scan import Scan
from state_machine.states.state import State

# Camera footprint for one photo/check, in feet -- matches the shape_size_ft
# every droneWorkflowTest.py simulation this session was verified against
# (200-seed sweeps, both single-pair and two-pair). TODO: derive from real
# camera FOV/altitude once vision is wired in (see DroneShare's own
# placeholder) the same way Pathfinder.matSize already does for the "no
# override" case.
SHAPE_SIZE_FT = (6.0, 4.0)
OVERLAP = 0.1

# How long an ASSISTANT's waypoint queue may sit empty before this drone
# gives up waiting and moves on. A real deployment needs an actual
# "mission complete" signal from the paired GAMBLER instead (see the
# multi-drone mission flow diagram) -- nothing sends that signal yet, so
# this is a stopgap that keeps an unfed ASSISTANT from cycling through
# CalcScanPath/Scan forever rather than the real termination condition.
ASSISTANT_IDLE_TIMEOUT_S = 30.0


def _maze_started(pf: Pathfinder) -> bool:
    """Whether start_maze_navigation() has already run for this mission --
    calling it again would silently wipe out real progress (maze_
    confirmed_path, the chained cross-pair retarget state, etc). True
    once any of the three A/B/C segments holds anything, which can only
    happen after the first call."""
    return bool(pf.maze_confirmed_path or pf.maze_a_path or pf.maze_b_path)


async def run(self: CalcScanPath) -> State:
    """
    Implements the run method for the CalcScanPath state.

    Computes this drone's next batch of places to check (GAMBLER/
    SOLOGAMBLER only -- an ASSISTANT has no Pathfinder of its own, see
    configureField) and saves them onto the drone's own waypoint queue
    (self.drone.waypoints) for Scan to fly. This is also where a replan
    actually happens: Scan/DroneShare transition back here whenever
    self.drone.replan_needed is set (remote mine/image data relayed in
    over interdrone -- see that flag's own docstring on Drone) or when
    DroneShare finds a mine itself, and this is what clears the flag once
    the queue has been recomputed to account for it.

    Returns
    -------
    EndRun : State
        If FlightSettings.max_flight_time has been exceeded (see Drone.
        time_exceeded) -- skips ExpandNodes entirely, prioritizing
        actually getting home over hardening a route there's no time
        left to fly anyway.
    Scan : State
        Once there are places queued to check (or, for an ASSISTANT, to
        give Scan a chance to drain whatever it's been sent).
    ExpandNodes : State
        Once there's genuinely nothing left to check anywhere on this
        drone's Pathfinder (or, for an ASSISTANT, once its queue has sat
        empty for ASSISTANT_IDLE_TIMEOUT_S -- see that constant's own
        docstring on why this is a stopgap, not the real signal).

    Raises
    ------
    asyncio.CancelledError
        If the execution of the CalcScanPath state is canceled.
    """
    try:
        update_state("CalcScanPath")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("CalcScanPath state running")

        if self.drone.time_exceeded(self.flight_settings.max_flight_time):
            flight_log.event("calc_scan_path_time_exceeded")
            return EndRun(self.drone, self.flight_settings, self.interdrone)

        # Whatever triggered this entry -- a local find or a relayed one
        # -- has now been accounted for by the recompute below (or, for
        # an ASSISTANT, doesn't apply). Clear it before anything else can
        # re-set it.
        self.drone.replan_needed = None

        if self.flight_settings.role == Role.ASSISTANT:
            # No Pathfinder of its own -- nothing to compute here. Scan
            # drains whatever's already queued (populated by the paired
            # GAMBLER, once that relay is wired up -- see Scan's and
            # DroneShare's own docstrings); this state just decides
            # whether it's worth another round or time to give up waiting.
            if self.drone.waypoints:
                self.drone.assistant_idle_since = None
                return Scan(self.drone, self.flight_settings, self.interdrone)
            if self.drone.assistant_idle_since is None:
                self.drone.assistant_idle_since = time.monotonic()
            elif time.monotonic() - self.drone.assistant_idle_since > ASSISTANT_IDLE_TIMEOUT_S:
                flight_log.event("calc_scan_path_complete", role="assistant")
                return ExpandNodes(self.drone, self.flight_settings, self.interdrone)
            return Scan(self.drone, self.flight_settings, self.interdrone)

        pf = Pathfinder.instance
        if not _maze_started(pf):
            pf.start_maze_navigation()

        places = pf.get_places_to_check_maze(overlap=OVERLAP, shape_size_ft=SHAPE_SIZE_FT)
        a_places, b_places = places["a"], places["b"]
        if not a_places and not b_places:
            pf.confirm_b_into_c()  # no-op if b is already empty -- safety net
            flight_log.event("calc_scan_path_complete", role="gambler_or_solo")
            return ExpandNodes(self.drone, self.flight_settings, self.interdrone)

        # A paired GAMBLER would hand b_places to its ASSISTANT here
        # instead of queuing them for itself -- not wired up yet (see
        # Scan/DroneShare's own docstrings), so this drone flies both
        # segments itself, same as a SOLOGAMBLER or a single-drone
        # mission already has to. The "scan_A_"/"scan_B_" name prefix is
        # how DroneShare later tells which Rule (1 or 2) applies to a
        # given waypoint without needing any extra state passed along.
        waypoints = [
            Waypoint(self.drone.id, lat, lon, name=f"scan_A_{i}")
            for i, (lat, lon) in enumerate(a_places)
        ] + [
            Waypoint(self.drone.id, lat, lon, name=f"scan_B_{i}")
            for i, (lat, lon) in enumerate(b_places)
        ]
        self.drone.resetWaypoints(waypoints)

        return Scan(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("CalcScanPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the CalcScanPath class to the run function
CalcScanPath.run_callable = run
