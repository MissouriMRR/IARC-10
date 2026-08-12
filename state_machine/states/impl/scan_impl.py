"""Implements the behavior of the Scan state."""

import asyncio
import logging
import math

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from flight.pathfinding.utils.goto import move_to
from state_machine.drone import LEG_GROUNDSPEED_M_S, LEG_TOLERANCE_M, MISSION_ALTITUDE_M
from state_machine.flight_settings import Role
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.drone_share import DroneShare
from state_machine.states.end_run import EndRun
from state_machine.states.scan import Scan
from state_machine.states.state import State
from vision.common.drone_coordinates import DronePose

# How far to climb between capture attempts when the camera's TRUE footprint
# (from its real FOV/mount geometry, not the fixed-size rectangle
# calc_scan_path_impl.SHAPE_SIZE_FT assumed when queuing this waypoint)
# doesn't fully cover the cell this waypoint was sent to check -- a higher
# camera sees a wider footprint on the ground.
CAMERA_CLIMB_STEP_M: float = 0.5

# Safety valve on the climb-retry loop below, independent of
# FlightSettings.max_flight_height (also enforced, per-attempt): stops a
# runaway climb if the coverage check can never be satisfied (e.g. a bug, or
# a waypoint right at the field's edge where no achievable altitude's
# footprint fully encloses the cell).
MAX_CAMERA_CLIMB_ATTEMPTS: int = 8

# How far off the commanded climb altitude counts as "arrived" -- deliberately
# tight (move_to's own default, ALTITUDE_TOLERANCE_M, is 1.5m and would call a
# 0.5m climb "arrived" before it had actually climbed).
_CLIMB_ALTITUDE_TOLERANCE_M: float = 0.15


async def _capture_for_assistant(self: Scan, waypoint) -> None:
    """ASSISTANT: captures one photo at this waypoint and stores its TRUE
    ground footprint corners (self.drone.last_photo_corners_latlon) for
    DroneShare to report back to the paired GAMBLER via SHARE_PHOTOS.

    No seen_tracker/climb-retry loop here, unlike _capture_and_mark_seen
    -- an ASSISTANT has no Pathfinder (see configureField) to check
    coverage against; the GAMBLER applies coverage to ITS OWN
    seen_tracker once the report arrives (see interdrone.py's own
    SHARE_PHOTOS receive case)."""
    camera = self.drone.camera
    if camera is None:
        logging.warning("drone %d has no camera configured -- skipping photo capture", self.drone.id)
        return

    image = camera.capture_image(only_metadata=False)
    if image is None or image.image is None:
        logging.warning("drone %d: camera capture failed -- skipping photo", self.drone.id)
        return
    self.drone.last_captured_image = image

    vehicle = self.drone.vehicle
    location = vehicle.location.global_relative_frame
    attitude = vehicle.attitude
    altitude_agl_m = self.drone.rangefinder_altitude_agl_m
    if altitude_agl_m is None:
        logging.warning(
            "drone %d: no fresh rangefinder3 reading -- falling back to %.1fm for"
            " this photo's footprint math",
            self.drone.id,
            MISSION_ALTITUDE_M,
        )
        altitude_agl_m = MISSION_ALTITUDE_M

    drone_pose = DronePose(
        lat=location.lat,
        lon=location.lon,
        altitude=altitude_agl_m,
        # dronekit's Attitude is in radians; DronePose expects degrees.
        yaw=math.degrees(attitude.yaw),
        pitch=math.degrees(attitude.pitch),
        roll=math.degrees(attitude.roll),
    )

    width, height = image.image.size
    top_left, top_right, bottom_left, bottom_right = camera.get_image_corner_coordinates(
        width, height, drone_pose
    )
    if None in (top_left, top_right, bottom_left, bottom_right):
        logging.warning(
            "drone %d: a corner ray never reached the ground -- skipping photo report",
            self.drone.id,
        )
        self.drone.last_photo_corners_latlon = None
        return

    # Reordered to a perimeter walk (top_left, top_right, bottom_right,
    # bottom_left) -- get_image_corner_coordinates returns a diagonal-
    # crossing order, same reason _capture_and_mark_seen reorders before
    # calling accept_image_corner_coord. The GAMBLER does that reordered
    # call itself once this report arrives; ASSISTANT has no Pathfinder to
    # call it on.
    self.drone.last_photo_corners_latlon = [top_left, top_right, bottom_right, bottom_left]


async def _capture_and_mark_seen(self: Scan, pf: Pathfinder, waypoint) -> None:
    """Captures a photo at the drone's current position, computes its TRUE
    ground footprint from the camera's real FOV/mount geometry (BaseCamera.
    get_image_corner_coordinates), and marks pf.seen_tracker with it --
    instead of the fixed-size-rectangle assumption calc_scan_path_impl used
    to space out waypoints in the first place (SHAPE_SIZE_FT), which is only
    ever a planning estimate, not what the camera actually saw.

    If that TRUE footprint doesn't fully cover the cell this waypoint was
    actually sent to check, climbs CAMERA_CLIMB_STEP_M (a higher camera sees
    a wider footprint) and retries, until it does, or until
    FlightSettings.max_flight_height or MAX_CAMERA_CLIMB_ATTEMPTS is hit.

    Altitude for the footprint math comes exclusively from
    self.drone.rangefinder_altitude_agl_m -- the only real AGL source
    available (GPS/relative-frame altitude is not AGL: it drifts with
    terrain and takeoff-point error) -- falling back to MISSION_ALTITUDE_M, with
    a logged warning, if no rangefinder3 reading has arrived yet. That
    fallback altitude is ONLY ever used for this footprint math, never for
    flight control.
    """
    camera = self.drone.camera
    if camera is None:
        logging.warning("drone %d has no camera configured -- skipping photo capture", self.drone.id)
        return

    vehicle = self.drone.vehicle
    required_x, required_y = pf.coord_converter.latlon_to_local(waypoint.lat, waypoint.long)
    required_col, required_row = pf.seen_tracker.real_to_cell(required_x, required_y)
    cell_in_bounds = (
        0 <= required_col < pf.seen_tracker.width and 0 <= required_row < pf.seen_tracker.height
    )

    attempts = 0
    while True:
        image = camera.capture_image(only_metadata=False)
        if image is None or image.image is None:
            logging.warning("drone %d: camera capture failed -- skipping photo", self.drone.id)
            return
        self.drone.last_captured_image = image

        location = vehicle.location.global_relative_frame
        attitude = vehicle.attitude
        altitude_agl_m = self.drone.rangefinder_altitude_agl_m
        if altitude_agl_m is None:
            logging.warning(
                "drone %d: no fresh rangefinder3 reading -- falling back to %.1fm for"
                " this photo's footprint math",
                self.drone.id,
                MISSION_ALTITUDE_M,
            )
            altitude_agl_m = MISSION_ALTITUDE_M

        drone_pose = DronePose(
            lat=location.lat,
            lon=location.lon,
            altitude=altitude_agl_m,
            # dronekit's Attitude is in radians; DronePose expects degrees.
            yaw=math.degrees(attitude.yaw),
            pitch=math.degrees(attitude.pitch),
            roll=math.degrees(attitude.roll),
        )

        width, height = image.image.size
        top_left, top_right, bottom_left, bottom_right = camera.get_image_corner_coordinates(
            width, height, drone_pose
        )
        if None in (top_left, top_right, bottom_left, bottom_right):
            logging.warning(
                "drone %d: a corner ray never reached the ground -- skipping footprint marking",
                self.drone.id,
            )
            return

        # get_image_corner_coordinates returns (top_left, top_right,
        # bottom_left, bottom_right) -- a diagonal-crossing order, not a
        # perimeter walk. accept_image_corner_coord's fill is a simple-
        # polygon point-in-test, so the corners must be reordered to trace
        # the rectangle's actual edges (top_left, top_right, bottom_right,
        # bottom_left) or the "polygon" self-intersects and the coverage
        # test is wrong.
        pf.accept_image_corner_coord([top_left, top_right, bottom_right, bottom_left])

        if not cell_in_bounds or pf.seen_tracker.get(required_col, required_row):
            return

        attempts += 1
        next_altitude_agl_m = altitude_agl_m + CAMERA_CLIMB_STEP_M
        if attempts >= MAX_CAMERA_CLIMB_ATTEMPTS or next_altitude_agl_m > self.flight_settings.max_flight_height:
            flight_log.event(
                "photo_coverage_incomplete",
                waypoint=flight_log.waypoint_brief(waypoint),
                attempts=attempts,
                altitude_agl_m=altitude_agl_m,
            )
            logging.warning(
                "drone %d: waypoint %d's target cell still not fully covered after"
                " %d climb(s) -- giving up",
                self.drone.id,
                waypoint.waypoint_id,
                attempts,
            )
            return

        current = vehicle.location.global_relative_frame
        target_relative_alt = current.alt + CAMERA_CLIMB_STEP_M
        flight_log.event(
            "photo_coverage_climb",
            waypoint=flight_log.waypoint_brief(waypoint),
            attempt=attempts,
            to_relative_alt=target_relative_alt,
        )
        await move_to(
            vehicle,
            current.lat,
            current.lon,
            target_relative_alt,
            groundspeed=LEG_GROUNDSPEED_M_S,
            tolerance=LEG_TOLERANCE_M,
            altitude_tolerance=_CLIMB_ALTITUDE_TOLERANCE_M,
        )


async def run(self: Scan) -> State:
    """
    Implements the run method for the Scan state.

    Flies the next waypoint off self.drone.waypoints (queued by
    CalcScanPath) and hands off to DroneShare to process/share whatever
    was photographed there. This is "the goto loop": CalcScanPath -> Scan
    -> DroneShare -> Scan -> DroneShare -> ... is the actual loop,
    implemented as a cycle through the state machine (its own
    `while self.current_state:` already supports this) rather than a
    Python while-loop inside any one state -- so DroneShare's "process
    this photo, share it" work is a real, separately-runnable state
    instead of a step buried inside this one.

    Breaks out of that cycle -- back to CalcScanPath instead of onward to
    DroneShare -- in two cases, both read from self.drone.replan_needed
    (see that flag's own docstring on Drone):
      - it's already set when this state is entered (remote mine/image
        data arrived while some OTHER waypoint was being flown --
        strictly between two Scan entries, not asynchronously mid-leg;
        this state doesn't poll for it during the leg itself);
      - the queue CalcScanPath left is empty (nothing left to fly this
        round).
    DroneShare is the other place this cycle can break to CalcScanPath:
    if IT finds a mine in the photo it just processed.

    Immediately after reaching the waypoint (GAMBLER/SOLOGAMBLER only --
    see _capture_and_mark_seen), captures a photo and marks pf.seen_tracker
    with its TRUE ground footprint, climbing in 0.5m steps and recapturing
    until that footprint fully covers the cell this waypoint was sent to
    check. DroneShare, which runs next, only has photo *processing* left to
    do (mine detection) -- see its own docstring.

    Also checked at the top of every entry, ahead of even replan_needed:
    FlightSettings.max_flight_time (see Drone.time_exceeded) -- there's no
    point starting (or continuing to plan around) another leg once the
    mission's flight time budget is gone. Breaks straight to EndRun, not
    CalcScanPath, since there's nothing left to replan for.

    Returns
    -------
    EndRun : State
        If the flight time budget has been exceeded.
    DroneShare : State
        After flying one waypoint, to process/share what was seen there.
    CalcScanPath : State
        If a replan is already needed, or nothing is queued.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the Scan state is canceled.
    """
    try:
        update_state("Scan")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("Scan state running")

        if self.drone.time_exceeded(self.flight_settings.max_flight_time):
            flight_log.event("scan_time_exceeded")
            return EndRun(self.drone, self.flight_settings, self.interdrone)

        if self.drone.replan_needed is not None or not self.drone.waypoints:
            # An empty queue with nothing to replan around yet (an
            # ASSISTANT waiting on its paired GAMBLER's next SEND_SEGMENT_
            # B_WAYPOINTS, or a GAMBLER that just handed segment B off and
            # has nothing local left of its own) bounces straight back to
            # CalcScanPath with no other await in between -- across two
            # concurrently-running drones sharing one event loop, that is
            # a tight loop with no suspension point, which starves every
            # OTHER drone's task (including the very interdrone_loop that
            # would deliver the message this drone is waiting on). A
            # single real drone process never notices this (nothing else
            # is competing for its event loop); asyncio.sleep(0) costs it
            # nothing and fixes the concurrent-drones-in-one-process case.
            await asyncio.sleep(0)
            return CalcScanPath(self.drone, self.flight_settings, self.interdrone)

        # drone_states is read-only peer state this drone already
        # maintains for collision avoidance (see conflictsForLeg's own
        # docstring on why it applies to non-formation, maze-mode
        # waypoints too) -- not a new interdrone send.
        # MISSION_ALTITUDE_M, not gotoWaypoint's own POIF-oriented default
        # (LEG_ALTITUDE_M) -- this is the mapping-mission scan loop.
        reached = await self.drone.gotoWaypoint(
            self.interdrone.drone_states, altitude_m=MISSION_ALTITUDE_M
        )
        self.drone.last_reached_waypoint = reached
        flight_log.event("scan_waypoint_reached", waypoint=flight_log.waypoint_brief(reached))

        # An ASSISTANT has no Pathfinder (see configureField), and therefore
        # no seen_tracker to check photo coverage against -- it still takes
        # the photo (_capture_for_assistant), just without the climb-retry
        # coverage loop; DroneShare reports the result back to its paired
        # GAMBLER via SHARE_PHOTOS instead of applying it locally.
        if self.flight_settings.role != Role.ASSISTANT:
            await _capture_and_mark_seen(self, Pathfinder.instance, reached)
        else:
            await _capture_for_assistant(self, reached)

        return DroneShare(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Scan state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Scan class to the run function
Scan.run_callable = run
