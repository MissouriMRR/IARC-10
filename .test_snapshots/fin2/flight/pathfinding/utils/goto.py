"""
File containing the move_to function responsible
for moving the drone to a certain waypoint and stopping there for 15 secs
"""

import asyncio
import logging
import math
import time
from typing import Callable

import dronekit

from flight.pathfinding.utils.calculate_distance import horizontal_distance

# Waypoint tolerance in meters: 6 meters = 19.685 feet
WAYPOINT_TOLERANCE: int = 6

# How far off the commanded altitude the drone may be and still be called
# arrived. Altitude is held by the autopilot's own loop, converges on its own
# schedule, and is the same for every leg of a level path — so a leg's
# horizontal tolerance should not have to absorb it. Anything tighter than the
# copter's steady-state altitude error makes arrival unreachable and sends
# every leg through the stall branch.
ALTITUDE_TOLERANCE_M: float = 1.5

# How much closer the drone has to get before it counts as still converging.
# Below this, the change is position noise rather than progress.
STALL_EPSILON_M: float = 0.05

# How long the drone may fail to beat its own closest approach before we
# conclude the position controller has settled as near as it is going to get.
# The real vehicle parks with a steady-state offset and then station-keeps with
# a slow drift around it; waiting for it to converge further never terminates.
STALL_TIMEOUT_S: float = 5.0

# Absolute ceiling on one leg, regardless of what the distance is doing. A leg
# of the POIF circle is ~1.3 m at 2 m/s; anything past this is not a leg that is
# going to finish.
LEG_TIMEOUT_S: float = 60.0


class FlightInterrupted(RuntimeError):
    """The autopilot is no longer under our control.

    Raised when the vehicle leaves GUIDED mode (EKF/battery failsafe, pilot
    takeover) or disarms while we are commanding it. Callers must stop the
    mission loop: continuing to command waypoints and broadcast progress from
    a drone that is actually landing feeds every peer a phantom position.
    """


class LegStalled(RuntimeError):
    """The drone stopped closing on the waypoint and never got acceptably near.

    Distinct from FlightInterrupted: the autopilot is still ours and still in
    GUIDED, it just isn't getting there. Raised only after the goto has been
    re-commanded once, so this means the vehicle is genuinely not tracking, not
    that a single MAVLink message was dropped.
    """


# duplicate code disabled since we may want different functionality
# for waypoints/odlcs search points
# pylint: disable=duplicate-code
async def move_to(
    drone: dronekit.Vehicle,
    latitude: float,
    longitude: float,
    altitude: float,
    groundspeed: float | None = None,
    tolerance: float | None = None,
    max_tolerance: float | None = None,
    altitude_tolerance: float | None = None,
    abort_check: Callable[[], None] | None = None,
) -> None:
    """
    This function takes in a latitude, longitude and altitude and autonomously
    moves the drone to that waypoint.

    `tolerance` and `max_tolerance` are horizontal distances only; altitude is
    judged separately against `altitude_tolerance`. Folding altitude error into
    the same number couples a tight lateral tolerance to a loop we do not
    control: a copter parked half a meter above the commanded altitude is on
    station laterally, but a 3-D distance says it is 0.5 m out before it has
    moved at all, and no leg can ever report arrival.

    Arrival is declared as soon as the drone is within `tolerance`, so a vehicle
    that can fly accurately still does. A vehicle that cannot is the reason
    `max_tolerance` exists: ArduCopter settles at a steady-state offset from the
    commanded point and then station-keeps around it with a slow drift, so
    "distance < tolerance" may simply never become true. Waiting on it forever
    hangs the leg — and in a lockstep formation, one hung drone parks the whole
    swarm at the barrier. Once the drone demonstrably stops closing in, being
    inside `max_tolerance` counts as arrived.

    Parameters
    ----------
    drone : dronekit.Vehicle
        The drone to move.
    latitude : float
        The requested latitude to move to, in degrees.
    longitude : float
        The requested longitude to move to, in degrees.
    altitude : float
        The requested altitude to go to, in meters.
    groundspeed : float, default None
        The requested groundspeed in meters per second, or None to let the
        autopilot decide. Groundspeed rather than airspeed because this is a
        multirotor: DroneKit will send either, but a copter's navigation is
        commanded in groundspeed and its handling of an airspeed request is not
        something to depend on.
    tolerance : float, default None
        The horizontal tolerance in meters, or None for 2 meters.
    max_tolerance : float, default None
        How far from the waypoint, horizontally, the drone may settle and still
        be called arrived, once it has stopped getting any closer. Defaults to
        `tolerance`, i.e. no allowance. This is the number peers must assume
        when they model where this drone actually is.
    altitude_tolerance : float, default None
        How far off `altitude` the drone may be and still count as arrived, or
        None for `ALTITUDE_TOLERANCE_M`. Kept separate from `tolerance` because
        altitude converges on the autopilot's schedule, not ours.
    abort_check : Callable[[], None], default None
        Called once per poll; raise from it to abandon the leg. Legs are short
        but not instant, and a swarm-level reason to stop flying — a peer
        drifting into us — should not have to wait for this one to finish.

    Raises
    ------
    FlightInterrupted
        If the autopilot left GUIDED or disarmed mid-leg.
    LegStalled
        If the drone stopped converging while still outside `max_tolerance`,
        and re-commanding the goto did not restart it.
    Exception
        Whatever `abort_check` raises, unchanged.
    """
    if tolerance is None:
        tolerance = 2
    if max_tolerance is None or max_tolerance < tolerance:
        max_tolerance = tolerance
    if altitude_tolerance is None:
        altitude_tolerance = ALTITUDE_TOLERANCE_M

    target = dronekit.LocationGlobalRelative(latitude, longitude, altitude)
    drone.simple_goto(target, groundspeed=groundspeed)
    # First determine if we need to move fast through waypoints or need to slow down at each one
    # Then loops until the waypoint is reached
    logging.info("Going to waypoint")

    started: float = time.monotonic()

    # Closest we have ever been on this leg, and when we last beat it. Measuring
    # against the best rather than the previous sample means the drift that
    # follows a settle doesn't keep looking like fresh progress.
    best_distance: float = math.inf
    best_altitude_error: float = math.inf
    last_progress: float = started
    recommanded: bool = False

    while True:
        if drone.mode.name != "GUIDED" or not drone.armed:
            raise FlightInterrupted(
                f"autopilot left our control mid-leg "
                f"(mode={drone.mode.name}, armed={drone.armed})"
            )
        if abort_check is not None:
            abort_check()
        position: dronekit.LocationGlobalRelative = drone.location.global_relative_frame

        # continuously checks current latitude, longitude and altitude of the drone
        drone_lat: float = position.lat
        drone_long: float = position.lon
        drone_alt: float = position.alt

        total_distance: float = horizontal_distance(
            drone_lat,
            drone_long,
            latitude,
            longitude,
        )
        altitude_error: float = abs(drone_alt - altitude)
        at_altitude: bool = altitude_error <= altitude_tolerance

        if total_distance < tolerance and at_altitude:
            logging.info(
                "Arrived %.2fm away from waypoint (%.2fm off altitude)",
                total_distance,
                altitude_error,
            )
            return

        now: float = time.monotonic()
        # Closing horizontally or still descending onto the commanded altitude
        # both count as progress: a drone holding station while it bleeds off
        # half a meter of altitude is working, not stalled.
        if (
            total_distance < best_distance - STALL_EPSILON_M
            or altitude_error < best_altitude_error - STALL_EPSILON_M
        ):
            best_distance = min(best_distance, total_distance)
            best_altitude_error = min(best_altitude_error, altitude_error)
            last_progress = now
        elif now - last_progress >= STALL_TIMEOUT_S:
            if total_distance <= max_tolerance and at_altitude:
                logging.warning(
                    "Settled %.2fm from waypoint (closest %.2fm, %.2fm off altitude) and"
                    " stopped closing after %.0fs; within the %.2fm allowance, calling it"
                    " arrived",
                    total_distance,
                    best_distance,
                    altitude_error,
                    now - last_progress,
                    max_tolerance,
                )
                return
            if not recommanded:
                # A dropped or superseded SET_POSITION_TARGET looks exactly like
                # a vehicle that won't track. Re-send once before giving up.
                recommanded = True
                logging.warning(
                    "Stalled %.2fm from waypoint, %.2fm off altitude (allowances"
                    " %.2fm / %.2fm); re-commanding goto",
                    total_distance,
                    altitude_error,
                    max_tolerance,
                    altitude_tolerance,
                )
                drone.simple_goto(target, groundspeed=groundspeed)
                best_distance = math.inf
                best_altitude_error = math.inf
                last_progress = now
            else:
                raise LegStalled(
                    f"drone stopped {total_distance:.2f}m from the waypoint and"
                    f" {altitude_error:.2f}m off its altitude (closest approach"
                    f" {best_distance:.2f}m, allowances {max_tolerance:.2f}m /"
                    f" {altitude_tolerance:.2f}m) and did not resume after re-commanding"
                )

        if now - started >= LEG_TIMEOUT_S:
            raise LegStalled(
                f"leg exceeded {LEG_TIMEOUT_S:.0f}s, still {total_distance:.2f}m"
                f" from the waypoint and {altitude_error:.2f}m off its altitude"
                f" (closest approach {best_distance:.2f}m)"
            )

        # Poll fast enough that a sub-meter arrival tolerance isn't blown past
        # between checks, while still not hammering the link.
        await asyncio.sleep(0.25)
