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


def _build_waypoints(drone_id: int, a_places, b_places) -> list[Waypoint]:
    """Builds the Waypoint batch for a round's places-to-check. The
    "scan_A_"/"scan_B_" name prefix is how DroneShare later tells which
    Rule (1 or 2) applies to a given waypoint without needing any extra
    state passed along."""
    return [
        Waypoint(drone_id, lat, lon, name=f"scan_A_{i}") for i, (lat, lon) in enumerate(a_places)
    ] + [
        Waypoint(drone_id, lat, lon, name=f"scan_B_{i}") for i, (lat, lon) in enumerate(b_places)
    ]


def _run_assistant(self: CalcScanPath) -> State:
    """ASSISTANT: no Pathfinder of its own (see configureField) -- nothing
    to compute here. Scan drains whatever's already queued (populated by
    the paired GAMBLER, once that relay is wired up -- see Scan's and
    DroneShare's own docstrings); this just decides whether it's worth
    another round or time to give up waiting."""
    if self.drone.waypoints:
        self.drone.assistant_idle_since = None
        return Scan(self.drone, self.flight_settings, self.interdrone)
    if self.drone.assistant_idle_since is None:
        self.drone.assistant_idle_since = time.monotonic()
    elif time.monotonic() - self.drone.assistant_idle_since > ASSISTANT_IDLE_TIMEOUT_S:
        flight_log.event("calc_scan_path_complete", role="assistant")
        return ExpandNodes(self.drone, self.flight_settings, self.interdrone)
    return Scan(self.drone, self.flight_settings, self.interdrone)


async def _confirm_b_into_mission_path(self: CalcScanPath, pf: Pathfinder) -> None:
    """Appends pf.maze_b_path's own points onto self.drone.mission_path
    (see that attribute's own docstring) BEFORE pf.confirm_b_into_c()
    clears maze_b_path -- this is the exact moment a stretch stops being
    reroutable and becomes settled history, which is what mission_path is
    meant to track. No-op (matching confirm_b_into_c's own safety-net
    behavior) if b is already empty -- including the app push below,
    since nothing about mission_path actually changed. Pushes the
    updated path to the app (Interdrone.push_mission_path, itself a
    no-op for anything but drone 1) right after, so the app's own view
    updates live as each stretch settles instead of only once at the
    very end (AppShare's own final push)."""
    if pf.maze_b_path:
        self.drone.extend_mission_path(
            [pf.coord_converter.local_to_latlon(n.x, n.y) for n in pf.maze_b_path]
        )
        await self.interdrone.push_mission_path()
    pf.confirm_b_into_c()


async def _drain_cross_pair_mines(self: CalcScanPath, pf: Pathfinder) -> None:
    """Applies every staged CROSS_PAIR_MINE_RELAY report (see Drone.
    pending_cross_pair_mines' own docstring) to this Pathfinder via
    add_discovered_mine(..., prefer_local_patch=True) -- the bounded
    local-splice repair a RELAYED (not self-found) mine needs, so this
    pair's own already-confirmed progress isn't needlessly discarded just
    because the OTHER pair found something (see that method's own
    prefer_local_patch docstring). Replies with CROSS_PAIR_PATCHED_SPAN
    for each one that actually produced a local patch
    (pf.last_patched_span is not None) -- nothing to reply for the ones
    check_path_envelopment's own unconditional full recompute handled
    instead (patch_confirmed_span's own docstring covers why only a real
    patch needs the discovering pair to go verify anything)."""
    reports = self.drone.pending_cross_pair_mines
    self.drone.pending_cross_pair_mines = []
    for report in reports:
        pf.add_discovered_mine(report.lat, report.lon, prefer_local_patch=True)
        if pf.last_patched_span is not None:
            patched_span_latlon = [
                pf.coord_converter.local_to_latlon(n.x, n.y) for n in pf.last_patched_span
            ]
            await self.interdrone.send_cross_pair_patched_span(
                report.obstacle_hash, patched_span_latlon, (report.discovering_drone_id,)
            )


async def _drain_photo_reports(self: CalcScanPath, pf: Pathfinder) -> None:
    """Applies every staged SHARE_PHOTOS report (see Drone.
    pending_photo_reports's own docstring) to this Pathfinder: marks
    coverage for the photo's footprint (pf.accept_image_corner_coord),
    then folds in any mine physically under it the same way DroneShare's
    own _apply_local_mine does for a segment-B find (pf.add_discovered_
    mine + pf.reroute_b_segment, skipped if check_merge_rewind already
    fully re-routed things) -- every SHARE_PHOTOS waypoint is necessarily
    segment B, since that's the whole reason the paired ASSISTANT was
    flying it instead of this drone."""
    reports = self.drone.pending_photo_reports
    self.drone.pending_photo_reports = []
    for report in reports:
        pf.accept_image_corner_coord(report.corner_coordinates)
        for mine_lat, mine_lon in report.mines:
            _obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
            if not rewound:
                pf.reroute_b_segment()


def _apply_point_a_sync(self: CalcScanPath, pf: Pathfinder) -> None:
    """Applies a staged CROSS_PAIR_POINT_A_SYNC (see Drone.
    pending_point_a_sync's own docstring), retargeting this pair's
    segment-A search onto the other pair's current point_A instead of
    the field's fixed far edge -- see Pathfinder.retarget_approach_
    target's own docstring for the mechanism, and
    flight/pathfinding/tests/droneWorkflowTest.py's _sync_approach_target
    for the pathfinder-only reference version of this same idea."""
    sync = self.drone.pending_point_a_sync
    self.drone.pending_point_a_sync = None
    if sync is None:
        return
    lat, lon = sync
    x, y = pf.coord_converter.latlon_to_local(lat, lon)
    pf.retarget_approach_target(x, y)


async def _send_point_a_sync_if_changed(self: CalcScanPath, pf: Pathfinder) -> None:
    """Sends this pair's own current point_A (maze_a_path[0]) to the
    other pair/solo drone via CROSS_PAIR_POINT_A_SYNC, but only if it
    actually moved since the last send -- point_A only changes as a side
    effect of a mine-driven reroute (including a retarget_approach_target
    of our own), so most rounds are a no-op here, same as
    _sync_approach_target's own epsilon guard."""
    partner_id = self.flight_settings.cross_pair_partner_id
    if partner_id is None or not pf.maze_a_path:
        return
    a0 = pf.maze_a_path[0]
    pos = pf.coord_converter.local_to_latlon(a0.x, a0.y)
    last = self.drone.last_synced_point_a
    if last is not None and abs(last[0] - pos[0]) < 1e-9 and abs(last[1] - pos[1]) < 1e-9:
        return
    await self.interdrone.send_cross_pair_point_a_sync(pos[0], pos[1], (partner_id,))
    self.drone.last_synced_point_a = pos


def _drain_verification_waypoints(self: CalcScanPath) -> list[Waypoint]:
    """Turns every staged CROSS_PAIR_PATCHED_SPAN point (see Drone.
    pending_verification_waypoints' own docstring) into a Waypoint --
    named "verify_", not "scan_A_"/"scan_B_", since these aren't part of
    either maze segment. DroneShare's own Rule 1/2 dispatch (checking for
    the "scan_A_" prefix) falls through to Rule 2 (a local reroute) for
    anything else, which is the right call here too -- a NEW mine turning
    up in re-verified territory is exactly a segment-B-shaped surprise,
    not the far-end "approach" Rule 1 is for."""
    points = self.drone.pending_verification_waypoints
    self.drone.pending_verification_waypoints = []
    return [
        Waypoint(self.drone.id, lat, lon, name=f"verify_{i}") for i, (lat, lon) in enumerate(points)
    ]


async def _run_sologambler(self: CalcScanPath) -> State:
    """SOLOGAMBLER: unpaired -- plans its own route and flies every
    segment (A and B) itself, since there's no ASSISTANT to hand segment B
    to."""
    pf = Pathfinder.instance
    if not _maze_started(pf):
        pf.start_maze_navigation()

    await _drain_cross_pair_mines(self, pf)
    _apply_point_a_sync(self, pf)
    verification_waypoints = _drain_verification_waypoints(self)

    places = pf.get_places_to_check_maze(overlap=OVERLAP, shape_size_ft=SHAPE_SIZE_FT)
    a_places, b_places = places["a"], places["b"]
    if not a_places and not b_places and not verification_waypoints:
        await _confirm_b_into_mission_path(self, pf)
        flight_log.event("calc_scan_path_complete", role="sologambler")
        partner_id = self.flight_settings.cross_pair_partner_id
        if partner_id is not None:
            # Verify agreement now that this side has nothing left to
            # check -- see Field.mineHash's own docstring for why this is
            # order/id-independent and safe to compare directly.
            await self.interdrone.send_checksum(pf.nodeField.mineHash(), (partner_id,))
        return ExpandNodes(self.drone, self.flight_settings, self.interdrone)

    self.drone.resetWaypoints(_build_waypoints(self.drone.id, a_places, b_places) + verification_waypoints)
    await _send_point_a_sync_if_changed(self, pf)
    return Scan(self.drone, self.flight_settings, self.interdrone)


async def _run_gambler(self: CalcScanPath) -> State:
    """GAMBLER: paired with an ASSISTANT. Plans the scored route the same
    way SOLOGAMBLER does, but only queues a_places onto its own waypoints
    -- b_places is handed to the paired ASSISTANT instead, over
    SEND_SEGMENT_B_WAYPOINTS (see interdrone.py's own send_segment_b_
    waypoints and its matching receive case), mirroring the already-
    verified split in flight/pathfinding/tests/droneWorkflowTest.py's
    simulate_leader_follower_pair. Kept as its own function (rather than
    folded into _run_sologambler) since the two roles diverge here."""
    pf = Pathfinder.instance
    if not _maze_started(pf):
        pf.start_maze_navigation()

    await _drain_cross_pair_mines(self, pf)
    await _drain_photo_reports(self, pf)
    _apply_point_a_sync(self, pf)
    verification_waypoints = _drain_verification_waypoints(self)

    places = pf.get_places_to_check_maze(overlap=OVERLAP, shape_size_ft=SHAPE_SIZE_FT)
    a_places, b_places = places["a"], places["b"]
    if not a_places and not b_places and not verification_waypoints:
        await _confirm_b_into_mission_path(self, pf)
        flight_log.event("calc_scan_path_complete", role="gambler")
        partner_id = self.flight_settings.cross_pair_partner_id
        if partner_id is not None:
            await self.interdrone.send_checksum(pf.nodeField.mineHash(), (partner_id,))
        return ExpandNodes(self.drone, self.flight_settings, self.interdrone)

    # The paired ASSISTANT flies segment B -- see SEND_SEGMENT_B_WAYPOINTS's
    # own receive case in interdrone.py, which queues these directly onto
    # the ASSISTANT's own self.drone.waypoints. Only a_places (this
    # drone's own segment A) goes onto its local queue below.
    paired_id = self.flight_settings.paired_drone
    if paired_id is not None and b_places:
        # b_places stays the SAME set every pass until the ASSISTANT's own
        # SHARE_PHOTOS reports get drained back in and shrink it (see
        # _drain_photo_reports) -- resend only on an actual change, or
        # every CalcScanPath pass would resetWaypoints the ASSISTANT's
        # queue out from under whatever it's mid-flight on (see Drone.
        # last_sent_segment_b's own docstring).
        b_places_key = tuple(b_places)
        if self.drone.last_sent_segment_b != b_places_key:
            await self.interdrone.send_segment_b_waypoints((paired_id,), b_places)
            self.drone.last_sent_segment_b = b_places_key
        b_places = []

    self.drone.resetWaypoints(_build_waypoints(self.drone.id, a_places, b_places) + verification_waypoints)
    await _send_point_a_sync_if_changed(self, pf)
    return Scan(self.drone, self.flight_settings, self.interdrone)


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

    Dispatches on FlightSettings.role to one of three separate functions
    (_run_assistant/_run_gambler/_run_sologambler) rather than branching
    only on "is this an ASSISTANT" -- GAMBLER hands segment B to its
    paired ASSISTANT (see _run_gambler's own docstring) while SOLOGAMBLER
    flies both segments itself, so the two roles diverge here.

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

        match self.flight_settings.role:
            case Role.ASSISTANT:
                return _run_assistant(self)
            case Role.GAMBLER:
                return await _run_gambler(self)
            case Role.SOLOGAMBLER:
                return await _run_sologambler(self)
    except asyncio.CancelledError as ex:
        logging.error("CalcScanPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the CalcScanPath class to the run function
CalcScanPath.run_callable = run
