"""Implements the behavior of the DroneShare state."""

import asyncio
import logging
import math

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from state_machine.drone import MISSION_ALTITUDE_M
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
from vision.common.drone_coordinates import DronePose


async def _check_photo_latlon(drone) -> tuple[float, float] | None:
    """"The actual check": runs mine detection and reports the highest-
    confidence find's (lat, lon), or None if nothing was found. Returns
    lat/lon, not local (x, y): an ASSISTANT has no Pathfinder/coord_
    converter to convert with (see configureField), and GAMBLER/
    SOLOGAMBLER's own caller does that one conversion itself, right after
    calling this.

    Footprint marking is NOT this function's job -- scan_impl.py's own
    capture/coverage-climb loop already marked pf.seen_tracker with the
    photo's TRUE ground footprint (from the camera's real FOV/mount
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


def _apply_local_mine(pf: Pathfinder, waypoint, mine_xy: tuple[float, float]):
    """Integrates a mine found at mine_xy into pf's own graph, applying
    Rule 1 (segment A -- helper-node detour, else full recompute) or
    Rule 2 (segment B -- local reroute), matching exactly what
    droneWorkflowTest.py's simulations do per discovery. Which rule
    applies is read off the "scan_A_"/"scan_B_" prefix
    calc_scan_path_impl.py names each queued waypoint with.

    Returns (obstacle, mine_lat, mine_lon) so the caller can relay this
    find to the other pair (CROSS_PAIR_MINE_RELAY needs the mine's own
    obstacle_hash, which only exists on the live obstacle -- see
    PolygonObstacle.obstacle_hash) -- or None if check_merge_rewind
    already fully re-routed things (rewound=True), which the ordinary
    intra-pair discovery path already treats as "nothing more to do
    here", so there is nothing new worth relaying either."""
    mine_lat, mine_lon = pf.coord_converter.local_to_latlon(*mine_xy)
    obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
    if rewound:
        return None
    if (waypoint.name or "").startswith("scan_A_"):
        if not pf.start_helper_node_detour(obstacle):
            pf.on_forward_mine_discovered()
    else:
        pf.reroute_b_segment()
    return obstacle, mine_lat, mine_lon


async def run(self: DroneShare) -> State:
    """
    Implements the run method for the DroneShare state.

    "The actual check": processes the photo just taken at the waypoint
    Scan flew to (self.drone.last_reached_waypoint). GAMBLER/SOLOGAMBLER:
    mine detection, then integrating any find into this drone's own
    Pathfinder, plus a CROSS_PAIR_MINE_RELAY to the other pair/solo drone
    if this is a two-pair mission. ASSISTANT: no Pathfinder to check
    against (see configureField) -- instead reports the photo's footprint
    (from Scan's own _capture_for_assistant) and any mine found in it back
    to its own paired GAMBLER via SHARE_PHOTOS, which applies it on
    arrival (see interdrone.py's own SHARE_PHOTOS receive case and Drone.
    pending_photo_reports).

    Returns
    -------
    CalcScanPath : State
        If a mine was found here (GAMBLER/SOLOGAMBLER only) -- this
        drone's own Pathfinder needs to replan around it before Scan can
        safely fly anything else queued.
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
            mine_latlon = await _check_photo_latlon(self.drone)
            if mine_latlon is not None:
                mine_xy = pf.coord_converter.latlon_to_local(*mine_latlon)
                applied = _apply_local_mine(pf, waypoint, mine_xy)
                mine_found = True
                flight_log.event(
                    "mine_found",
                    waypoint=flight_log.waypoint_brief(waypoint),
                    label="A" if (waypoint.name or "").startswith("scan_A_") else "B",
                )
                # CROSS-PAIR: tell the other pair/solo drone (if this is a
                # two-pair mission) about this LOCAL find -- see
                # FlightSettings.cross_pair_partner_id and Pathfinder.
                # add_discovered_mine's own prefer_local_patch docstring
                # for why the receiving side must NOT just call
                # add_discovered_mine(lat, lon) the same way this drone
                # just did. `applied` is None when check_merge_rewind
                # already fully handled things above (rewound=True) --
                # nothing new to relay in that case either.
                partner_id = self.flight_settings.cross_pair_partner_id
                if applied is not None and partner_id is not None:
                    obstacle, mine_lat, mine_lon = applied
                    await self.interdrone.send_cross_pair_mine_relay(
                        mine_lat, mine_lon, obstacle.obstacle_hash, (partner_id,)
                    )

        elif self.flight_settings.role == Role.ASSISTANT and waypoint is not None:
            # Report this waypoint's photo back to the paired GAMBLER --
            # footprint corners from Scan's own _capture_for_assistant,
            # plus any mine physically under it. Not this drone's own
            # cross-pair relay above (that's intra-pair GAMBLER-to-
            # GAMBLER, gated out for an ASSISTANT by the branch above);
            # this is GAMBLER-to-its-own-ASSISTANT's report going back.
            corners = self.drone.last_photo_corners_latlon
            if corners is not None:
                mine_latlon = await _check_photo_latlon(self.drone)
                mines = [mine_latlon] if mine_latlon is not None else []
                paired_id = self.flight_settings.paired_drone
                if paired_id is not None:
                    await self.interdrone.share_photos(
                        [{"cornerCoordinates": corners, "mines": mines}], (paired_id,)
                    )

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
