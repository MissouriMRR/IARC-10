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
from state_machine.states.app_share import AppShare
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.state import State

# Camera footprint for one photo/check, in feet -- matches the shape_size_ft
# every droneWorkflowTest.py simulation this session was verified against
# (200-seed sweeps, both single-pair and two-pair). TODO: derive from real
# camera FOV/altitude once vision is wired in (see _fly_and_check's own
# placeholder below) the same way Pathfinder.matSize already does for the
# "no override" case.
SHAPE_SIZE_FT = (6.0, 4.0)
OVERLAP = 0.1

# How long an ASSISTANT (no Pathfinder of its own -- see configureField)
# waits for its next waypoint before giving up and moving on. A real
# deployment needs an actual "mission complete" signal from the paired
# GAMBLER instead of a timeout (see _run_assistant's own docstring) --
# nothing sends that signal yet, so this is a stopgap that keeps an
# unfed ASSISTANT from hanging the state machine forever rather than the
# real termination condition.
ASSISTANT_IDLE_TIMEOUT_S = 30.0
ASSISTANT_POLL_INTERVAL_S = 0.5


async def _fly_and_check(self: CalcScanPath, lat: float, lon: float, label: str) -> tuple[float, float] | None:
    """Flies this drone to (lat, lon) and reports whether a new mine was
    found there. `label` is "A" or "B" -- which segment this waypoint
    belongs to, for logging only (Pathfinder itself doesn't need to know;
    it's implicit in which places-to-check list the waypoint came from).

    Returns the newly discovered mine's local (x, y) if one was found
    under the photo footprint the Pathfinder doesn't already know about,
    else None.

    PLACEHOLDER: photo-taking/mine-detection (vision.camera.takeImage +
    vision.mine_detection.detect_mines) isn't wired to state_machine.drone.
    Drone yet -- this always marks the footprint seen and reports no mine,
    which keeps the mission loop moving correctly but means it never
    actually finds anything against a real camera. See
    flight/pathfinding/main.py's Drone.takePhoto/processPhoto for the
    vision-side pieces a real implementation needs to call here.
    """
    pf = Pathfinder.instance
    waypoint = Waypoint(self.drone.id, lat, lon, name=f"scan_{label}")
    self.drone.updateWaypoints([waypoint])
    # drone_states is read-only peer state this drone already maintains
    # for collision avoidance (see conflictsForLeg's own docstring on why
    # it applies to non-formation, maze-mode waypoints too) -- not a new
    # interdrone send.
    await self.drone.gotoWaypoint(self.interdrone.drone_states)

    # Real image-footprint corners, computed from where we actually are --
    # this part IS real geometry, not a placeholder (matches exactly what
    # every droneWorkflowTest.py simulation does per photo).
    along_ft, across_ft = SHAPE_SIZE_FT
    x, y = pf.coord_converter.latlon_to_local(lat, lon)
    llx, lly = x - across_ft / 2.0, y - along_ft / 2.0
    corners_local = [
        (llx, lly), (llx + across_ft, lly),
        (llx + across_ft, lly + along_ft), (llx, lly + along_ft),
    ]
    corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
    pf.accept_image_corner_coord(corners_latlon)

    # PLACEHOLDER: run real mine detection on the photo just taken here,
    # and return its local (x, y) if one was found. Always "nothing found"
    # for now.
    return None


async def _run_gambler_or_solo(self: CalcScanPath) -> None:
    """GAMBLER/SOLOGAMBLER: this drone owns the real Pathfinder (see
    configureField) and drives the whole discover-as-you-fly loop -- the
    same A/B/C incremental replanning simulate_one_drone_maze exercises
    in simulation, just against real flight instead of a simulated
    minefield.

    A paired GAMBLER still checks its OWN segment B here rather than
    handing it to its ASSISTANT (the leader/follower split
    simulate_leader_follower_pair proves out in simulation) -- that split
    needs the paired ASSISTANT to actually receive and report back on
    real waypoints over interdrone comms, which isn't wired up yet (see
    _run_assistant and interdrone.py's own NEW_WAYPOINTS/SHARE_PHOTOS
    placeholders). Flying both segments itself is the correct, safe
    fallback in the meantime: identical to how a SOLOGAMBLER (no partner
    at all) or a single-drone mission already has to work.
    """
    pf = Pathfinder.instance
    pf.start_maze_navigation()

    while True:
        places = pf.get_places_to_check_maze(overlap=OVERLAP, shape_size_ft=SHAPE_SIZE_FT)
        a_places, b_places = places["a"], places["b"]
        if not a_places and not b_places:
            pf.confirm_b_into_c()  # no-op if b is already empty -- safety net
            break

        found_a = None
        for lat, lon in a_places:
            mine_xy = await _fly_and_check(self, lat, lon, "A")
            if mine_xy is not None:
                found_a = mine_xy
                break

        if found_a is not None:
            mine_lat, mine_lon = pf.coord_converter.local_to_latlon(*found_a)
            obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
            if not rewound:
                if not pf.start_helper_node_detour(obstacle):
                    pf.on_forward_mine_discovered()
            flight_log.event("mine_found", label="A", lat=mine_lat, lon=mine_lon)

        current_b_places = b_places if found_a is None else pf.get_places_to_check_maze(
            overlap=OVERLAP, shape_size_ft=SHAPE_SIZE_FT
        )["b"]
        found_b = None
        for lat, lon in current_b_places:
            mine_xy = await _fly_and_check(self, lat, lon, "B")
            if mine_xy is not None:
                found_b = mine_xy
                break

        if not current_b_places:
            pf.confirm_b_into_c()
        elif found_b is not None:
            mine_lat, mine_lon = pf.coord_converter.local_to_latlon(*found_b)
            obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
            if not rewound:
                pf.reroute_b_segment()
            flight_log.event("mine_found", label="B", lat=mine_lat, lon=mine_lon)
        else:
            pf.advance_b_prefix_into_c()

    flight_log.event("calc_scan_path_complete", role="gambler_or_solo")


async def _run_assistant(self: CalcScanPath) -> None:
    """ASSISTANT: no Pathfinder of its own (see configureField) -- flies
    whatever's in its own waypoint queue (self.drone.waypoints) as it
    arrives, one at a time, same as _fly_and_check's leg-flying but
    without ever calling into a Pathfinder (an ASSISTANT never plans,
    only flies what it's told).

    PLACEHOLDER: nothing populates that queue yet. The intended source
    (see the coordinating-4-drones plan and
    simulate_leader_follower_pair's own LEADER<-FOLLOWER boundary
    functions) is the paired GAMBLER relaying its segment-B places over
    interdrone comms -- not implemented yet, see this state's own
    _run_gambler_or_solo (which still flies segment B itself for exactly
    this reason) and interdrone.py's NEW_WAYPOINTS/SHARE_PHOTOS
    placeholders. This loop's own logic is real and ready to consume
    real waypoints the moment something sends them; ASSISTANT_IDLE_TIMEOUT_S
    below only exists so an unfed ASSISTANT doesn't hang the state machine
    forever in the meantime -- replace it with a real "mission complete"
    signal from the paired GAMBLER once that exists.
    """
    last_activity = time.monotonic()
    while time.monotonic() - last_activity < ASSISTANT_IDLE_TIMEOUT_S:
        if not self.drone.waypoints:
            await asyncio.sleep(ASSISTANT_POLL_INTERVAL_S)
            continue
        last_activity = time.monotonic()
        waypoint = self.drone.waypoints[0]
        await self.drone.gotoWaypoint(self.interdrone.drone_states)
        # PLACEHOLDER: take a photo here and report it (footprint + any
        # mine found) back to the paired GAMBLER -- see
        # _leader_apply_follower_report in droneWorkflowTest.py for the
        # exact data shape (plain lat/lon photo corners + mine
        # coordinates) this needs to send, and SHARE_PHOTOS's own
        # placeholder receive case in interdrone.py for where the
        # GAMBLER side of it would land.
        flight_log.event("assistant_waypoint_flown", waypoint=flight_log.waypoint_brief(waypoint))

    flight_log.event("calc_scan_path_complete", role="assistant")


async def run(self: CalcScanPath) -> State:
    """
    Implements the run method for the CalcScanPath state.

    Drives this drone's own role-specific piece of the minefield search --
    a GAMBLER or SOLOGAMBLER runs the real discover-as-you-fly Pathfinder
    loop, an ASSISTANT flies whatever waypoint queue it's given -- until
    that role's work is done, then transitions to AppShare.

    Returns
    -------
    AppShare : State
        The next state once this drone's scan work is complete.

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

        if self.flight_settings.role == Role.ASSISTANT:
            await _run_assistant(self)
        else:
            await _run_gambler_or_solo(self)

        return AppShare(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("CalcScanPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the CalcScanPath class to the run function
CalcScanPath.run_callable = run
