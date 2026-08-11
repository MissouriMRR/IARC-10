"""Implements the behavior of the DroneShare state."""

import asyncio
import logging

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from state_machine.flight_settings import Role
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.drone_share import DroneShare
from state_machine.states.scan import Scan
from state_machine.states.state import State

async def _check_photo() -> tuple[float, float] | None:
    """"The actual check": reports whether a new mine was found in the
    photo Scan already captured (self.drone.last_captured_image).

    Footprint marking is no longer this function's job -- scan_impl.py's
    own capture/coverage-climb loop already marked pf.seen_tracker with the
    photo's TRUE ground footprint (from the camera's real FOV/mount
    geometry) before this state ever runs, which is also why it no longer
    assumes a fixed-size rectangle here the way this function used to.

    PLACEHOLDER: mine detection (RPICamera.capture_and_detect_mines() +
    BaseCamera.get_pixel_coordinate() to turn a pixel detection into a
    lat/lon) isn't wired up yet -- always reports no mine found, which
    keeps the loop moving correctly but means it never actually finds
    anything against a real camera.
    """
    return None


def _apply_local_mine(pf: Pathfinder, waypoint, mine_xy: tuple[float, float]) -> None:
    """Integrates a mine found at mine_xy into pf's own graph, applying
    Rule 1 (segment A -- helper-node detour, else full recompute) or
    Rule 2 (segment B -- local reroute), matching exactly what
    droneWorkflowTest.py's simulations do per discovery. Which rule
    applies is read off the "scan_A_"/"scan_B_" prefix
    calc_scan_path_impl.py names each queued waypoint with."""
    mine_lat, mine_lon = pf.coord_converter.local_to_latlon(*mine_xy)
    obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
    if rewound:
        return
    if (waypoint.name or "").startswith("scan_A_"):
        if not pf.start_helper_node_detour(obstacle):
            pf.on_forward_mine_discovered()
    else:
        pf.reroute_b_segment()


async def run(self: DroneShare) -> State:
    """
    Implements the run method for the DroneShare state.

    "The actual check": processes the photo just taken at the waypoint
    Scan flew to (self.drone.last_reached_waypoint) -- mine detection,
    then integrating any find into this drone's own Pathfinder -- and
    would share the footprint/mine with the rest of the swarm (see the
    PLACEHOLDER below). An ASSISTANT has no Pathfinder to check against,
    so it only shares (or would) and returns straight to Scan.

    PLACEHOLDER: sharing the footprint/mine with other drones over
    interdrone comms (extending SHARE_PHOTOS, or a dedicated cross-pair
    relay message) isn't implemented -- see interdrone.py's own
    placeholder receive cases and the multi-drone mission flow diagram
    for the full message list this needs. This is also the mechanism
    that's supposed to set OTHER drones' self.drone.replan_needed on
    THEIR end when it eventually exists -- this drone's own local finds
    don't need that flag at all, since this state can just decide the
    transition directly (see Returns below).

    Returns
    -------
    CalcScanPath : State
        If a mine was found here -- this drone's own Pathfinder needs to
        replan around it before Scan can safely fly anything else queued.
    Scan : State
        Otherwise -- there may be more already-queued waypoints to fly
        before the next replan.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the DroneShare state is canceled.
    """
    try:
        update_state("DroneShare")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("DroneShare state running")

        waypoint = self.drone.last_reached_waypoint
        mine_found = False

        if self.flight_settings.role != Role.ASSISTANT and waypoint is not None:
            pf = Pathfinder.instance
            mine_xy = await _check_photo()
            if mine_xy is not None:
                _apply_local_mine(pf, waypoint, mine_xy)
                mine_found = True
                flight_log.event(
                    "mine_found",
                    waypoint=flight_log.waypoint_brief(waypoint),
                    label="A" if (waypoint.name or "").startswith("scan_A_") else "B",
                )

        # PLACEHOLDER: share waypoint's footprint (and the mine, if any)
        # with the rest of the swarm here.

        if mine_found:
            return CalcScanPath(self.drone, self.flight_settings, self.interdrone)
        return Scan(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("DroneShare state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the DroneShare class to the run function
DroneShare.run_callable = run
