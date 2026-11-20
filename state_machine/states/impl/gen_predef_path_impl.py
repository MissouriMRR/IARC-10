"""Implements the behavior of the CalcScanPath state."""

import asyncio
import json
import logging
import math

import flight.flight_log as flight_log
from flight.waypoint import Waypoint
from state_machine.drone import MISSION_ALTITUDE_M
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.gen_predef_path import GenPreDefPath
from state_machine.states.end_run import EndRun
from state_machine.states.expand_nodes import ExpandNodes
from state_machine.states.scan import Scan
from state_machine.states.state import State

# Same file Takeoff.run() loads the real camera from (RPICamera's own
# config, not the AirSim one -- SHAPE_SIZE_FT is a waypoint-SPACING
# planning estimate, and AirSim missions still fly the same real-world
# camera/lens the field deployment does).
VISION_CONFIG_PATH = "./vision/config.json"
_METERS_TO_FT = 3.28084
_FALLBACK_SHAPE_SIZE_FT = (6.0, 4.0)


def _camera_footprint_ft(altitude_m: float, config_path: str) -> tuple[float, float]:
    """Ground footprint (along_ft, across_ft) of one photo at altitude_m,
    derived from vision/config.json's own hFovDeg/vFovDeg -- straight-
    down flat-ground approximation (2 * altitude * tan(fov / 2)), the
    same formula flight/pathfinding/path_subdivision.py's own (dead,
    unreachable) ground_covered_image uses. This is only ever used to
    decide how far apart to SPACE planned waypoints -- Scan's own
    _capture_and_mark_seen computes the REAL per-photo footprint later,
    via full ray-casting (BaseCamera.get_image_corner_coordinates), for
    the actual coverage marking.

    along/across assignment (h_fov -> along, v_fov -> across) matches
    this constant's own previous hardcoded placeholder, (6.0, 4.0) --
    along was the larger of the two there, and hFovDeg (66.0 in the
    current config) is likewise larger than vFovDeg (52.3) -- but is
    otherwise an assumption about the camera's mount orientation
    (cameraMountRotationDeg) that hasn't been confirmed against real
    hardware; swap the two if the real mount turns out to run the other
    way.

    Falls back to the previous hardcoded placeholder if config.json is
    missing/unreadable or either FOV is left at its own 0.0 default,
    rather than degenerating to a zero-size (or crashing) footprint.
    """
    try:
        with open(config_path) as f:
            vision_config = json.load(f)
    except (OSError, json.JSONDecodeError):
        return _FALLBACK_SHAPE_SIZE_FT
    h_fov_deg = vision_config.get("hFovDeg", 0.0)
    v_fov_deg = vision_config.get("vFovDeg", 0.0)
    if h_fov_deg <= 0.0 or v_fov_deg <= 0.0:
        return _FALLBACK_SHAPE_SIZE_FT
    altitude_ft = altitude_m * _METERS_TO_FT
    along_ft = 2 * altitude_ft * math.tan(math.radians(h_fov_deg) / 2)
    across_ft = 2 * altitude_ft * math.tan(math.radians(v_fov_deg) / 2)
    return (along_ft, across_ft)


# Camera footprint for one photo/check, in feet -- see _camera_footprint_ft's
# own docstring. Computed against MISSION_ALTITUDE_M=2.0m and the current
# vision/config.json (hFovDeg=66.0, vFovDeg=52.3): (8.52, 6.44) ft --
# larger than the old (6.0, 4.0) hardcoded placeholder, not smaller: that
# placeholder was never actually derived from any altitude, real or
# assumed, so there's no size relationship to read into the change.
SHAPE_SIZE_FT = _camera_footprint_ft(MISSION_ALTITUDE_M, VISION_CONFIG_PATH)
OVERLAP = 0.1

# How long an idle drone waits before polling again.
IDLE_POLL_S = 0.1


def _build_waypoints(drone_id: int, points) -> list[Waypoint]:
    places = []
    for i in points:
        places.append([i["lat"], i["lon"]])
    """Waypoints for (lat, lon) places, in flying order."""
    return [Waypoint(drone_id, lat, lon, name=f"scan_{i}") for i, (lat, lon) in enumerate(places)]


async def run(self: GenPreDefPath) -> State:
    """
    Implements the run method for the GenPreDefPath state.

    The planning step, the same on every drone: through its side agent
    (flight/pathfinding/side_agent.py) it applies everything relayed in, works out the
    side's claims from where both drones are, announces its own job, and queues its own
    share for Scan. It finishes when its copy of the side's planner has nothing left.

    This is also where a replan happens: Scan/DroneShare come back here after every photo
    and whenever self.drone.replan_needed is set (relayed data), and this clears the flag.

    Returns
    -------
    EndRun : State
        If FlightSettings.max_flight_time has been exceeded (Drone.time_exceeded) -- skips
        ExpandNodes, prioritizing getting home.
    Scan : State
        To fly the queue (or poll again if it is empty).
    ExpandNodes : State
        Once there's nothing left to check on this side, or no route exists at all.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the GenPreDefPath state is canceled.
    """
    try:
        update_state("GenPreDefPath")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("GenPreDefPath state running")

        if self.drone.time_exceeded(self.flight_settings.max_flight_time):
            flight_log.event("gen_predef_path_exceeded")
            return EndRun(self.drone, self.flight_settings, self.interdrone)

        # Whatever triggered this entry is accounted for by the recompute below. Clear it
        # before anything else can re-set it.
        self.drone.replan_needed = None

        self.drone.resetWaypoints(_build_waypoints(self.drone.id, self.flight_settings.flight_path[self.drone.id]))
        
        return Scan(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("GenPreDefPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the CalcScanPath class to the run function
GenPreDefPath.run_callable = run