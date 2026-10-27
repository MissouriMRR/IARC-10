"""Implements the behavior of the Scan state."""

import asyncio
import logging
import math

import flight.flight_log as flight_log
from flight.pathfinding.utils.goto import FlightInterrupted, LegStalled, move_to

# state_machine.drone patches the collections aliases dronekit needs on
# import, so it must come before dronekit.
from state_machine.drone import (
    LEG_GROUNDSPEED_M_S,
    LEG_TOLERANCE_M,
    MISSION_ALTITUDE_M,
    FormationLost,
)
import dronekit
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.drone_share import DroneShare
from state_machine.states.end_run import EndRun
from state_machine.states.land import Land
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


async def _capture_and_mark_seen(self: Scan, planner, waypoint) -> None:
    """Captures a photo at the drone's current position, computes its TRUE
    ground footprint from the camera's real FOV/mount geometry (BaseCamera.
    get_image_corner_coordinates), and records it as coverage on the planner --
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

    Leaves the last footprint on Drone.last_photo_corners_latlon (None if none) --
    DroneShare relays it to the other side (CROSS_PAIR_COVERAGE) and the side-mate.
    """
    self.drone.last_photo_corners_latlon = None
    camera = self.drone.camera
    if camera is None:
        logging.warning("drone %d has no camera configured -- skipping photo capture", self.drone.id)
        return

    vehicle = self.drone.vehicle

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
        # perimeter walk. The coverage fill is a simple-polygon test, so the
        # corners must be reordered to trace the rectangle's actual edges
        # (top_left, top_right, bottom_right, bottom_left) or the "polygon"
        # self-intersects and the coverage test is wrong.
        corners = [top_left, top_right, bottom_right, bottom_left]
        planner.own_footprint(corners)
        self.drone.last_photo_corners_latlon = corners

        # None: the waypoint's cell is off the field, so no altitude can cover it.
        if planner.cell_seen_at(waypoint.lat, waypoint.long) is not False:
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


async def run(self: Scan) -> State | None:
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

    Immediately after reaching the waypoint (see _capture_and_mark_seen), captures a
    photo and records its TRUE
    ground footprint on the planner, climbing in 0.5m steps and recapturing
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
    Land : State
        If separation was lost or a leg stalled (drone still under our control).
    None
        If the autopilot took over (FlightInterrupted): the drone is put in LAND
        (unless already LAND/RTL), then this waits for disarm and ends the machine.

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
            # An empty queue with nothing to replan around yet (waiting on the
            # other side's point A or the side-mate's photos) bounces straight back to
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
        try:
            reached = await self.drone.gotoWaypoint(
                self.interdrone.drone_states, altitude_m=MISSION_ALTITUDE_M
            )
        except FormationLost as ex:
            # Measured separation from a peer fell below the floor (or a hold never
            # cleared): the avoidance guarantee is gone. Land, as POIF does, rather
            # than let the exception kill the state machine with the drone in GUIDED.
            flight_log.event("scan_separation_lost", reason=str(ex))
            logging.critical("drone %d: scan aborted -- %s", self.drone.id, ex)
            return Land(self.drone, self.flight_settings, self.interdrone)
        self.drone.last_reached_waypoint = reached
        flight_log.event("scan_waypoint_reached", waypoint=flight_log.waypoint_brief(reached))

        # Records the photo's real footprint on this drone's planner, climbing to retry
        # while it misses the waypoint's square; DroneShare then applies the photo.
        await _capture_and_mark_seen(self, self.drone.planner, reached)

        return DroneShare(self.drone, self.flight_settings, self.interdrone)
    except LegStalled as ex:
        # Covers gotoWaypoint and the climb-retry move_to in _capture_and_mark_seen.
        # Still GUIDED but not tracking: land rather than end the machine airborne.
        flight_log.event("scan_leg_stalled", reason=str(ex))
        logging.error("drone %d: scan aborted, not reaching its waypoints: %s", self.drone.id, ex)
        return Land(self.drone, self.flight_settings, self.interdrone)
    except FlightInterrupted as ex:
        flight_log.event("scan_failsafe", reason=str(ex))
        logging.critical("drone %d: scan aborted, autopilot took over: %s", self.drone.id, ex)
        # The autopilot owns the vehicle: don't command it further (Land's RTL would
        # wait on GUIDED movement). Make sure it's coming down, wait for disarm.
        vehicle = self.drone.vehicle
        if vehicle.armed and vehicle.mode.name not in ("LAND", "RTL"):
            vehicle.mode = dronekit.VehicleMode("LAND")
        while vehicle.armed:
            await asyncio.sleep(0.5)
        flight_log.event("scan_failsafe_landed")
        logging.info("Failsafe landing complete, motors disarmed")
        return None
    except asyncio.CancelledError as ex:
        logging.error("Scan state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Scan class to the run function
Scan.run_callable = run
