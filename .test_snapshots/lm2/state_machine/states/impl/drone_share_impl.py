"""Implements the behavior of the DroneShare state."""

import asyncio
import logging
import math

import flight.flight_log as flight_log
from state_machine.drone import MISSION_ALTITUDE_M
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.drone_share import DroneShare
from state_machine.states.state import State
from vision.common.drone_coordinates import DronePose


async def _check_photo_latlon(drone) -> tuple[float, float] | None:
    """"The actual check": runs mine detection and reports the highest-
    confidence find's (lat, lon), or None if nothing was found. Returns
    lat/lon, which is what the planner interface takes.

    Footprint marking is NOT this function's job -- scan_impl.py's own
    capture/coverage-climb loop already recorded the photo's TRUE ground
    footprint (from the camera's real FOV/mount
    geometry) before this state ever runs.

    The detection itself (RPICamera.capture_and_detect_mines(), backed by
    the IMX500's on-camera inference) already exists and does the actual
    finding -- this just wires its pixel-space output into world
    coordinates via BaseCamera.get_pixel_coordinate(), the same way
    scan_impl.py's own capture loop builds a DronePose. A fresh capture is
    taken here rather than reusing self.drone.last_captured_image, since
    capture_and_detect_mines() drives its own capture internally (tied to
    the IMX500's own metadata/tensor output for that specific frame) --
    the drone hasn't moved since Scan's own capture, so this is still the
    same hover position, just a separate frame.

    BYPASSES multi-frame voting for now (config.json's useVoting/
    scanFrames/minFrameHits/minAverageConfidence) -- that machinery has no
    orchestrator anywhere reachable (vision.BIGVISIONCLASS.Vision would be
    it, but has its own separate, unrelated bugs -- see scan_impl.py's own
    docstring for why it's bypassed too) -- so this is single-shot
    detection only, taking whatever capture_and_detect_mines() returns
    from ONE frame.
    """
    camera = drone.camera
    if camera is None:
        logging.warning("drone %d has no camera configured -- skipping mine detection", drone.id)
        return None

    detections = camera.capture_and_detect_mines()
    if not detections:
        return None
    best = max(detections, key=lambda d: d.score)
    px, py, _w, _h = best.box
    image_width, image_height = best.imageSize

    vehicle = drone.vehicle
    location = vehicle.location.global_relative_frame
    attitude = vehicle.attitude
    altitude_agl_m = drone.rangefinder_altitude_agl_m
    if altitude_agl_m is None:
        logging.warning(
            "drone %d: no fresh rangefinder3 reading -- falling back to %.1fm for"
            " mine-detection geolocation",
            drone.id,
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

    ground = camera.get_pixel_coordinate(px, py, image_width, image_height, drone_pose)
    if ground is None:
        logging.warning("drone %d: mine detection's pixel ray never reached the ground", drone.id)
        return None
    mine_lat, mine_lon = ground
    return mine_lat, mine_lon


async def run(self: DroneShare) -> State:
    """
    Implements the run method for the DroneShare state.

    "The actual check", the same on every drone: mine detection on the photo just taken
    at the waypoint Scan flew to (self.drone.last_reached_waypoint), then the side agent
    applies the photo to this drone's copy of the side's planner, relays new mines and the
    footprint to the other side, and sends the photo to the side-mate (SHARE_PHOTOS). A
    photo with no footprint and no mine counts as a miss (side_agent.failed_photo).

    Returns
    -------
    CalcScanPath : State
        After every photo: the forward gap aims at the other side's point A, which moves,
        and a mine found here needs a replan before anything queued is safe to fly.

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
        if waypoint is not None:
            corners = self.drone.last_photo_corners_latlon
            mine_latlon = await _check_photo_latlon(self.drone) if corners is not None else None
            mines = [mine_latlon] if mine_latlon is not None else []
            if mines:
                flight_log.event("mine_found", waypoint=flight_log.waypoint_brief(waypoint))
            if corners is None and not mines:
                # No footprint at all: a miss for the squares it was aimed at, here and in
                # the side-mate's copy, so an untakeable photo isn't handed out forever.
                await self.drone.side.failed_photo(waypoint.lat, waypoint.long)
            else:
                # Applies the photo (mines first, then path nodes), relays new mines and the
                # footprint to the other side, and sends the photo to the side-mate.
                await self.drone.side.own_photo(corners, mines)
        return CalcScanPath(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("DroneShare state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the DroneShare class to the run function
DroneShare.run_callable = run
