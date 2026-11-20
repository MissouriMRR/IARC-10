"""Defines the Drone class for the state machine."""

import asyncio
import collections
import collections.abc
import json
import logging
import os
import sys
import time
from typing import Any, NamedTuple

# dronekit uses removed collections aliases (Python 3.10+)
collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]
collections.Mapping = collections.abc.Mapping  # type: ignore[attr-defined]
collections.MutableSet = collections.abc.MutableSet  # type: ignore[attr-defined]
collections.Callable = collections.abc.Callable  # type: ignore[attr-defined]

import dronekit
from pymavlink import mavutil
from pymavlink.dialects.v20.all import MAVLink_command_long_message

from flight.utils.calculate_distance import calculate_distance
from state_machine.flight_settings import SimMode
from flight.waypoint import Waypoint, segment_distance
import flight.flight_log as flight_log
from flight.utils.goto import move_to

# Off by default so a real-mode run never blocks on a console that isn't there.
WAIT_FOR_USER_INPUT: bool = os.environ.get("WAIT_FOR_USER_INPUT", "0") not in (
    "0",
    "false",
    "False",
    "",
)

# POIF requirement: 20 ft. POIF-demo legs only (see gotoWaypoint's own
# altitude_m default, and Takeoff's DEMO branch) -- NOT the mapping-
# mission altitude, see MISSION_ALTITUDE_M below for that.
LEG_ALTITUDE_M: float = 6.0

# Mapping-mission flight altitude: 2m, chosen together with vision/
# config.json's own hFovDeg/vFovDeg (RPICamera/AirSimCamera read FOV
# from that file, never a hardcoded value here -- this constant only
# fixes the ALTITUDE half of the footprint, camera-coverage math still
# depends on whatever FOV config.json currently holds). Used for the
# MISSION takeoff itself (see takeoff_impl.py), every leg Scan flies
# during CalcScanPath's own scan loop (gotoWaypoint's altitude_m), and
# as the geolocation fallback (scan_impl.py/drone_share_impl.py) when no
# fresh rangefinder reading is available -- that fallback has to match
# the real commanded altitude, or a stale rangefinder reading makes the
# footprint math think the drone is higher (or lower) than it actually
# commanded, silently skewing every mine/coverage position it computes.
MISSION_ALTITUDE_M: float = 2.0

# Speed bounds how far two drones can drift apart in waypoint index between
# lockstep checks, so this is a formation parameter as much as a pace one.
# Groundspeed, not airspeed: a copter navigates in groundspeed, and DroneKit will
# happily send either.
LEG_GROUNDSPEED_M_S: float = 2.0

# The vehicle flies to LEG_TOLERANCE_M whenever it can
LEG_TOLERANCE_M: float = 0.35
LEG_MAX_TOLERANCE_M: float = 0.6

# ArduPilot's RNGFNDn parameter blocks are 1-indexed by name (RNGFND1..RNGFND10)
# but MAVLink's DISTANCE_SENSOR.id field is 0-indexed on the wire, so
# "rangefinder3" (RNGFND3) is expected to arrive as id == 2. This mapping is a
# naming convention, not a MAVLink guarantee -- confirm against the real
# RNGFND3_* parameters on actual hardware before trusting it.
RANGEFINDER3_MAVLINK_ID: int = 2

# DISTANCE_SENSOR.current_distance (and min/max_distance) are centimeters on
# the wire; the rest of this codebase works in meters (LEG_ALTITUDE_M,
# vision.common.drone_coordinates.DronePose.altitude, etc.).
_RANGEFINDER_CM_PER_M: float = 100.0

# A rangefinder reading older than this is treated as unusable by
# rangefinder_altitude_agl_m -- stale enough that trusting it for an image's
# footprint math would be worse than falling back to a known altitude.
RANGEFINDER_STALE_S: float = 1.0


class CrossPairMineReport(NamedTuple):
    """One mine from the partner side (CROSS_PAIR_MINE_RELAY or _SYNC), staged on
    Drone.pending_cross_pair_mines by interdrone's receive handler for CalcScanPath to drain
    later, synchronously -- the planner is never touched from the interdrone loop's own
    (concurrent) task."""

    lat: float
    lon: float


class SharedPhotoReport(NamedTuple):
    """One SHARE_PHOTOS entry from the side-mate,
    staged on Drone.pending_photo_reports by interdrone's receive handler
    for CalcScanPath to drain later, synchronously -- same reason
    CrossPairMineReport is staged rather than applied directly: the planner
    is never touched from the interdrone loop's own (concurrent) task."""

    corner_coordinates: list[tuple[float, float]]
    mines: list[tuple[float, float]]
    # A photo that produced no footprint (camera failure): where it was planned. It counts
    # as a miss (planner.failed_photo); corner_coordinates is empty.
    failed_at: tuple[float, float] | None = None


class Drone:
    """
    A drone for the state machine to control.
    This class is a wrapper around the dronekit Vehicle class,
    and will be passed around to each state.
    Data can be stored in this class to be shared between states.

    Attributes
    ----------
    address : str
        The address used to connect to the drone.
    baud : int | None
        The baud rate, or None to use the default.
    is_connected
    sim_mode
    _sim_mode : SimMode | None
        The simulation mode of this drone, or None if one has not been specified.
    vehicle
    _vehicle : dronekit.Vehicle | None
        The Dronekit Vehicle object that controls the drone, or None if a connection
        hasn't been made yet.

    Methods
    -------
    __init__(connection_string: str) -> None
        Initialize a new Drone object, but do not connect to a drone.
    arm(self) -> Awaitable[None]
        Arm the drone.
    close(self) -> Awaitable[None]
        Close the owned DroneKit Vehicle object.
    close_servo(self, servo_num: int) -> Awaitable[None]:
        Close the servo with the given number.
    connect_drone(self) -> Awaitable[None]
        Connect to a drone.
    is_connected(self) -> bool
        Checks if a drone has been connected to.
    open_servo(self, servo_num: int) -> Awaitable[None]:
        Open the servo with the given number.
    remove_arming_check(self) -> None
        For use with airsim.
    return_to_launch(self) -> Awaitable[none]
        Method to move vehicle above home location, then descend vertically.
    takeoff(self, takeoff_alt: float, margin: float = 1.5) -> Awaitable[None]
        Takeoff vertically to the passed altitude.
    use_settings(self, sim_mode: SimMode) -> None
        Modify the connection settings based on the given simulation mode.
    vehicle(self) -> dronekit.Vehicle
        Get the Dronekit Vehicle object owned by this Drone object.
    """

    def __init__(
        self, address: str = "", baud: int | None = None, id: int = 0
    ) -> None:
        """
        Initialize a new Drone object, but do not connect to a drone.

        Parameters
        ----------
        address : str, default ""
            The address of the drone to connect to when the `connect_drone()`
            method is called.
        baud : int, default None
            The baud rate, or None to use the default.
        """
        self._sim_mode: SimMode | None = None
        self._vehicle: dronekit.Vehicle | None = None
        self.address: str = address
        self.baud: int | None = baud
        self.tasks: tuple = []
        self.id = id
        self.waypoints = []
        # Set (to a reason string) by interdrone's own message handling when
        # remote mine/image data arrives that could invalidate the currently
        # queued waypoints -- "async event sets a flag, the flight loop
        # notices it". Checked by Scan's
        # goto loop between legs; cleared once CalcScanPath has actually
        # recomputed the queue in response, so a discovery that arrives
        # mid-leg is never silently dropped.
        self.replan_needed: str | None = None
        # The waypoint Scan most recently flew to, for DroneShare to
        # process/share -- gotoWaypoint() pops it off self.waypoints and
        # returns it, but each state's run() is a fresh call with no
        # shared closure, so it has to live here to cross that boundary.
        self.last_reached_waypoint: Waypoint | None = None
        self.startTime: float | None = None
        # Last speed sent to the autopilot, so commandPoint can skip re-sending it.
        self._commanded_groundspeed: float | None = None
        # Most recent rangefinder3 (DISTANCE_SENSOR id=RANGEFINDER3_MAVLINK_ID)
        # reading, in meters AGL, and when it arrived -- written by
        # _on_distance_sensor, read through the rangefinder_altitude_agl_m
        # property below. GPS/relative-frame altitude is not AGL (it drifts
        # with terrain and takeoff-point error), so this is the only real AGL
        # source available, and it is used exclusively for image-footprint
        # math (see scan_impl.py) -- nothing about flight control depends on it.
        self._rangefinder_altitude_agl_m: float | None = None
        self._rangefinder_updated_at: float | None = None
        # The RPICamera this drone was configured with at Takeoff (MISSION
        # branch only -- see takeoff_impl.py). None until then, or if this
        # Drone was never taken through Takeoff (e.g. a test harness).
        self.camera: Any = None
        # The last Image captured during Scan's photo/coverage-climb loop,
        # for DroneShare's mine-detection placeholder to process without
        # recapturing.
        self.last_captured_image: Any = None
        # The TRUE ground footprint corners (lat, lon) of the photo Scan just
        # took -- None if that capture failed. DroneShare sends it to the
        # side-mate (SHARE_PHOTOS) and the other side (CROSS_PAIR_COVERAGE).
        self.last_photo_corners_latlon: list[tuple[float, float]] | None = None
        # The mission's reported route: plain (lat, lon) points, in flight
        # order -- readable by something outside this drone's own process
        # (the app, a status file) without a live planner. Set by
        # CalcScanPath when this side finishes and replaced by EndRun: in a
        # two-sided mission drone 1's is the full-field route, joined with
        # the other side's (see end_run_impl._join_with_partner). Only drone
        # 1's (the mission's app link -- see SEND_SWARM_STATUS) is read
        # externally.
        self.mission_path: list[tuple[float, float]] = []
        # This drone's copy of its side's planner (flight/pathfinding/mission_planner.py) and
        # the agent that drives it and does its messaging (flight/pathfinding/side_agent.py).
        # Both built by Takeoff's configureField, on every drone.
        self.planner = None
        self.side = None
        # Staged by interdrone's receive handler and applied by the side agent on the next
        # planning pass -- the planner is never touched from the interdrone task:
        # - the other side's mines (CROSS_PAIR_MINE_RELAY / _SYNC);
        self.pending_cross_pair_mines: list[CrossPairMineReport] = []
        # - the side-mate's photos (SHARE_PHOTOS);
        self.pending_photo_reports: list[SharedPhotoReport] = []
        # - the other side's latest point A (latest wins) and its photo footprints;
        self.pending_partner_point_a: tuple[float, float] | None = None
        self.pending_partner_coverage: list[list[tuple[float, float]]] = []
        # - the job each other drone last announced (SIDE_JOB; latest wins), by sender id:
        #   the side-mate's is held from it (claims.py), the other side's steers clear of it.
        self.peer_jobs: dict[int, Any] = {}
        # Set by calc_scan_path_impl.py's _flag_no_valid_path when the
        # planner finds no route at all -- which otherwise looks exactly
        # like a finished scan. AppShare's own path file/push checks this
        # so a fully mine-blocked field is reported as "no path found,"
        # not silently as an empty-but-successful one.
        self.no_valid_path_found: bool = False
        # The join (EndRun, two-sided missions):
        # - drone 1: the other side's route (CROSS_PAIR_CHAIN), for the joined report;
        self.partner_route: list[tuple[float, float]] | None = None
        self.partner_route_complete: bool = False
        self.partner_route_mine_hash: str | None = None
        # - drone 1: the join is done, and the mine hash it was made with. Later
        #   CROSS_PAIR_CHAIN resends are acked straight from the receive handler, so a lost
        #   ack can't strand the other side.
        self.route_joined: bool = False
        self.joined_mine_hash: str | None = None
        # - the top side's speaker: the mine hash drone 1 joined its route with
        #   (CROSS_PAIR_CHAIN_ACK). It may leave only while that matches its own.
        self.partner_route_ack_hash: str | None = None
        # The mapping mission's POSITION_REPORT stream (started by Takeoff, cancelled on
        # landing): peers need it for collision avoidance, the side-mate for its claims.
        self.position_stream = None
        # Drone 1, two-sided mission: the reported route is only this side's half
        # because the partner's route never arrived or didn't connect.
        self.reported_half_route: bool = False
        # TODO: add reference to mine and path data classes

    @property
    def is_connected(self) -> bool:
        """Checks if a drone has been connected to.

        Returns
        -------
        bool
            Whether this Drone object has connected to a drone.
        """
        return self._vehicle is not None

    @property
    def sim_mode(self) -> SimMode | None:
        """Get the simulation mode of this drone.

        Returns
        -------
        SimMode | None
            The simulation mode of this drone, or None if one has not been specified.
        """
        return self._sim_mode

    @property
    def vehicle(self) -> dronekit.Vehicle:
        """Get the DroneKit Vehicle object owned by this Drone object.

        Returns
        -------
        dronekit.Vehicle
            The Vehicle object owned by this Drone object.

        Raises
        ------
        AttributeError
            If a connection hasn't been made yet.
        """
        vehicle: dronekit.Vehicle | None = self._vehicle
        if vehicle is None:
            raise RuntimeError("we haven't connected to the drone yet")
        return vehicle

    async def connect_drone(self) -> None:
        """Connect to a drone. This operation is idempotent.

        Raises
        ------
        RuntimeError
            If no connection address has been set.
        """
        if self.is_connected:
            return

        if len(self.address) == 0:
            raise RuntimeError("no connection address specified")

        logging.info("Waiting for drone to connect...")
        self._vehicle = (
            dronekit.connect(self.address, wait_ready=True, timeout=90)
            if self.baud is None
            else dronekit.connect(self.address, wait_ready=True, baud=self.baud, timeout=90)
        )
        logging.info("Drone discovered!")
        self._vehicle.add_message_listener("DISTANCE_SENSOR", self._on_distance_sensor)

        if self._sim_mode is not SimMode.REAL or not WAIT_FOR_USER_INPUT:
            return

        # Opt-in only: the flight code normally starts as a systemd service with
        # no console attached, so a blocking prompt would hang the boot. Set
        # WAIT_FOR_USER_INPUT=1 for a hand-launched run where you want the
        # pre-arm messages on screen before the drone does anything.
        if not sys.stdin.isatty():
            logging.warning("WAIT_FOR_USER_INPUT is set but stdin is not a terminal; continuing.")
            return

        message_1: str = "Waiting for user input to continue... "
        message_2: str = "(press enter when ready) "
        input(f"\x1b[38;2;255;255;0m{message_1}" f"\x1b[3m{message_2}" "\x1b[0m")

    def _on_distance_sensor(self, _vehicle: dronekit.Vehicle, _name: str, message) -> None:
        """dronekit message-listener callback for DISTANCE_SENSOR.

        The (vehicle, name, message) signature is fixed by dronekit's
        add_message_listener -- it is never bound to a caller-chosen object,
        which is why this is a bound method (self comes from the normal
        method-binding, not a closure) rather than a free function defined
        inside connect_drone. Only rangefinder3 (RANGEFINDER3_MAVLINK_ID) is
        used for anything here -- other DISTANCE_SENSOR instances (e.g. an
        obstacle-avoidance rangefinder facing a different direction) are
        ignored.
        """
        if message is None or message.id != RANGEFINDER3_MAVLINK_ID:
            return
        self._rangefinder_altitude_agl_m = message.current_distance / _RANGEFINDER_CM_PER_M
        self._rangefinder_updated_at = time.time()

    @property
    def rangefinder_altitude_agl_m(self) -> float | None:
        """Most recent rangefinder3 altitude reading, in meters AGL.

        None if no reading has arrived yet, or the most recent one is older
        than RANGEFINDER_STALE_S -- callers need a fresh number, not a stale
        extrapolation, since this is the only AGL altitude source available
        and it feeds directly into image-footprint math.
        """
        if self._rangefinder_altitude_agl_m is None or self._rangefinder_updated_at is None:
            return None
        if time.time() - self._rangefinder_updated_at > RANGEFINDER_STALE_S:
            return None
        return self._rangefinder_altitude_agl_m

    def remove_arming_check(self) -> None:
        """
        For use with airsim.
        """
        self.vehicle.parameters["ARMING_CHECK"] = 0

    async def arm(self) -> None:
        """
        Arm the drone.
        """

        logging.info("Waiting for vehicle to intialize...")
        while not self.vehicle.is_armable:
            # Vehicle is not ready to accept code
            await asyncio.sleep(0.5)

        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle is properly armed
        logging.info("Waiting for arming...")
        while not self.vehicle.armed or self.vehicle.mode.name != "GUIDED":
            await asyncio.sleep(0.5)

    async def takeoff(self, takeoff_alt: float, margin: float = 1.5) -> None:
        """
        Takeoff vertically to the passed altitude.

        Parameters
        ----------
        takeoff_alt: float
            Altitude to reach in meters
        margin: float, default 1.5
            Extra altitude, in meters, to command above `takeoff_alt` so the
            climb overshoots rather than undershoots. Pass 0 when the drone must
            end up at a specific altitude — a mission whose legs command
            `takeoff_alt` spends the first leg descending back out of the margin.
        """
        logging.info("Using takeoff altitude of %f m (+%f m margin)", takeoff_alt, margin)
        self.vehicle.simple_takeoff(takeoff_alt + margin)

        # Verify vehicle reaches target altitude
        while self.vehicle.location.global_relative_frame.alt < takeoff_alt:
            await asyncio.sleep(0.5)  # ALTITUDE IS IN METERS
        logging.info("Reached target altitude (%f m).", takeoff_alt)

    # See the recall function at the bottom, we might remove this function as its specific to SAUS
    async def return_to_launch(self) -> None:
        """
        Method to move vehicle above home location, then descend vertically.
        """
        home_loc = dronekit.LocationGlobalRelative(
            self.vehicle.home_location.lat, self.vehicle.home_location.lon, 23
        )  # Min alt should be in constants file
        self.vehicle.simple_goto(home_loc)
        logging.info("Moving to home lat/lon...")
        while (
            calculate_distance(
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.alt,
                home_loc.lat,
                home_loc.lon,
                home_loc.alt,
            )
            > 1
        ):  # Get within 1 meter above home location
            await asyncio.sleep(0.5)
        self.vehicle.mode = dronekit.VehicleMode("RTL")
        logging.info("Descending...")
        while (
            self.vehicle.location.global_relative_frame.alt > 0.2  # ALTITUDE IS IN METERS
        ):  # Ensure drone gets within 8in above ground
            await asyncio.sleep(0.5)
        logging.info("Reached ground.")

    async def close(self) -> None:
        """Close the owned DroneKit Vehicle object."""
        self.vehicle.close()

    def use_settings(self, sim_mode: SimMode) -> None:
        """Set the simulation mode of this drone and update the connection settings
        according to the simulation mode.

        Parameters
        ----------
        sim_mode : SimMode
            The simulation mode.

        Raises
        ------
        ValueError
            If `sim_mode` is not a valid SimMode.
        """
        self._sim_mode = sim_mode
        # sim_vehicle.py hands SITL's serial0 (tcp:5760) to MAVProxy as its master, and the
        # SITL only accepts one client per port, so 5760 is unavailable to us. serial1
        # (tcp:5762) is the port left free for external clients like dronekit. Override via
        # SITL_MAVLINK_PORT when running the SITL without MAVProxy (--no-mavproxy), where
        # 5760 is free and is the right choice.
        port = int(os.environ.get("SITL_MAVLINK_PORT", "5762"))
        # In AIRSIM mode the SITL is not necessarily on this machine. When the flight code
        # runs on a Pi and the sim runs on a desktop, this is the sim host's LAN address.
        host = os.environ.get("SITL_MAVLINK_HOST", "127.0.0.1")

        match sim_mode:
            case SimMode.REAL:
                # Talk to the FC directly. MAVProxy is not installed on every Pi, and when it
                # is missing this is the only address that works -- a udpin default fails as a
                # 30s heartbeat timeout with nothing in the log to say why, because binding a
                # socket nobody dials looks identical to a healthy one.
                # On a Pi, /dev/serial0 is the symlink to whichever UART is wired to the FC.
                # Where MAVProxy *is* running it owns the port exclusively, so it must be
                # stopped -- or set REAL_MAVLINK_ADDRESS=udpin:127.0.0.1:14551 to take its
                # loopback --out and let it keep fanning the stream out to Mission Planner.
                # udpin, not udp: MAVProxy's --out is the sender, so we bind and it dials us.
                self.address = os.environ.get("REAL_MAVLINK_ADDRESS", "/dev/serial0")
                self.baud = 115200 if self.address.startswith("/dev/") else None
            case SimMode.SIM:
                logging.info(f"Using SIM mode settings with port {port}")
                self.address = "tcp:127.0.0.1:" + str(port)
                self.baud = None
            case SimMode.AIRSIM:

                port += (
                    self.id - 1
                ) * 10  # IDs are assigned sequentially starting at 5762 and increasing by 10 for each drone.
                logging.info(f"Using AIRSIM mode settings with host {host} port {port}")
                self.address = f"tcp:{host}:{port}"
                self.baud = None
            case _:
                raise ValueError("invalid sim mode")

    def updateWaypoints(self, waypoints):
        for waypoint in waypoints:
            self.waypoints.append(waypoint)

    def resetWaypoints(self, waypoints):
        self.waypoints = waypoints

    def getWaypoints(self):
        return self.waypoints

    def getWaypointChecksum(self):
        return Waypoint.getChecksum(self.waypoints)

    def get_mission_iarc_path(self, buffer_width: int = 0) -> str:
        """Returns self.mission_path in the IARC text-reporting format (see
        flight.pathfinding.iarc_report.format_iarc_path for the format itself).

        Needs self.planner for coordinate geometry (the local frame format_iarc_path
        works in). Every drone has one once Takeoff has configured the field.

        Raises ValueError if mission_path is empty, or if this drone has no planner."""
        if self.planner is None:
            raise ValueError(
                "get_mission_iarc_path needs a planner for coordinate geometry, and this"
                " drone has none"
            )
        # The planner reports with strict=False: a discontinuity is logged, not raised --
        # raising out of EndRun/AppShare would leave the drone hovering.
        return self.planner.iarc_path(self.mission_path, buffer_width)

    def currentPosition(self) -> tuple[float, float]:
        """This drone's live position as a (latitude, longitude) pair."""
        position = self.vehicle.location.global_relative_frame
        return (position.lat, position.lon)

    def time_exceeded(self, max_flight_time: float) -> bool:
        """Whether more than max_flight_time seconds have passed since
        self.startTime (set once, in Takeoff's mission branch). False if
        startTime is still None -- too early in the mission for "exceeded"
        to mean anything -- rather than raising, so callers (CalcScanPath/
        Scan's own time-exceeded check, ahead of every replan and every
        flight leg) don't need their own None-guard. time.monotonic(), as
        startTime is set: a wall-clock step (NTP, GPS time sync) mid-flight
        must not end the mission early or never."""
        if self.startTime is None:
            return False
        return time.monotonic() - self.startTime > max_flight_time

    def commandPoint(
        self,
        lat: float,
        long: float,
        altitude: float,
        groundspeed: float | None = None,
        force: bool = False,
    ) -> None:
        """Point the vehicle at a location and return immediately.

        `move_to` commands a target and then blocks until the drone arrives,
        which is the right shape for a leg flown one waypoint at a time and the
        wrong one for a target that moves every tick. This is the non-blocking
        half: the caller owns the loop, decides when the target has been passed,
        and re-commands as often as it likes.

        The speed is only re-sent when it changes. DroneKit turns the keyword
        into a separate DO_CHANGE_SPEED message, and repeating that at the
        controller's tick rate is a message per tick spent restating something
        the autopilot already knows. Pass `force` to re-send it anyway, for the
        case where the vehicle appears to have missed the command entirely.
        """
        if force:
            self._commanded_groundspeed = None
        if groundspeed is not None and groundspeed != self._commanded_groundspeed:
            self._commanded_groundspeed = groundspeed
            self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(lat, long, altitude),
                groundspeed=groundspeed,
            )
            return
        self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat, long, altitude))

    async def gotoWaypoint(self, altitude_m: float = LEG_ALTITUDE_M):
        """Fly to the next waypoint.

        altitude_m defaults to LEG_ALTITUDE_M -- Scan's own call, during the
        mapping mission, passes MISSION_ALTITUDE_M explicitly instead.
        """
        curWaypoint = self.waypoints[0]
        target = (curWaypoint.lat, curWaypoint.long)
        leg_from = self.currentPosition()

        flight_log.event(
            "leg_start",
            target=flight_log.waypoint_brief(curWaypoint),
            frm=leg_from,
            queued=len(self.waypoints),
        )

        move_started = time.monotonic()
        await move_to(
            self.vehicle,
            curWaypoint.lat,
            curWaypoint.long,
            altitude_m,
            groundspeed=LEG_GROUNDSPEED_M_S,
            tolerance=LEG_TOLERANCE_M,
            max_tolerance=LEG_MAX_TOLERANCE_M,
        )

        curWaypoint.has_visited = True
        arrived_at = self.currentPosition()
        flight_log.event(
            "wp_reached",
            target=flight_log.waypoint_brief(curWaypoint),
            at=arrived_at,
            # How far from the waypoint move_to() called it arrived — a lumpy
            # circle is often just this tolerance stacking up, not avoidance.
            error_m=segment_distance(arrived_at, arrived_at, target, target),
            leg_seconds=time.monotonic() - move_started,
        )
        logging.info(f"Reached waypoint {curWaypoint.waypoint_id}")

        # The queue can be replaced mid-leg (a replan while flying): return the waypoint
        # this leg flew, and drop it only if it is still queued -- never pop whatever is at
        # the head now (another waypoint, or nothing at all).
        for i, queued in enumerate(self.waypoints):
            if queued is curWaypoint:
                del self.waypoints[i]
                break
        return curWaypoint

    # Smart landing sequence, Should be usable in final product!!
    async def recall(self) -> None:
        """Flies to the nearest of the four field edges (in the planner's local frame) at
        MISSION_ALTITUDE_M -- exits the field along the shortest path
        before Land's own return_to_launch takes over for the rest of the
        way home, rather than cutting straight back across ground that
        might not be fully swept. Only ever reached from the mapping
        mission's own EndRun -> AppShare -> Recall tail (see recall_impl.py),
        never POIF, hence MISSION_ALTITUDE_M and not LEG_ALTITUDE_M here.
        A no-op if this drone has no planner (a test harness that
        skipped Takeoff)."""
        planner = self.planner
        if planner is None:
            return
        lat, lon = self.currentPosition()
        x, y = planner.latlon_to_local(lat, lon)
        w, h = planner.field_w, planner.field_h
        # (distance to the edge, the point on it) for left, right, bottom, top. The old
        # version only compared the right and top edges, so a drone near the left or
        # bottom edge flew across the field to leave it.
        _dist, (target_x, target_y) = min(
            (x, (0.0, y)), (w - x, (w, y)), (y, (x, 0.0)), (h - y, (x, h)),
            key=lambda edge: edge[0],
        )
        target_lat, target_lon = planner.local_to_latlon(target_x, target_y)
        self.commandPoint(target_lat, target_lon, MISSION_ALTITUDE_M, groundspeed=LEG_GROUNDSPEED_M_S)
        target = (target_lat, target_lon)
        while segment_distance(self.currentPosition(), self.currentPosition(), target, target) > LEG_TOLERANCE_M:
            await asyncio.sleep(0.5)
