"""Implements the behavior of the DroneShare state."""

import asyncio
import logging
import math

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from state_machine.drone import LEG_ALTITUDE_M
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


async def _check_photo(drone, pf: Pathfinder) -> tuple[float, float] | None:
    """"The actual check": runs mine detection and reports the highest-
    confidence find's LOCAL (x, y), or None if nothing was found.

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
            LEG_ALTITUDE_M,
        )
        altitude_agl_m = LEG_ALTITUDE_M
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
    return pf.coord_converter.latlon_to_local(mine_lat, mine_lon)


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
            mine_xy = await _check_photo(self.drone, pf)
            if mine_xy is not None:
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

        # PLACEHOLDER: share waypoint's footprint with the ASSISTANT's own
        # paired GAMBLER (SHARE_PHOTOS) -- being implemented separately;
        # not this drone's own cross-pair relay above, which is intra-
        # pair GAMBLER-to-GAMBLER, not GAMBLER-to-its-own-ASSISTANT.

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
