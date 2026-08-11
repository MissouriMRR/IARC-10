"""Defines the Drone class for the state machine."""

import asyncio
import collections
import collections.abc
import json
import logging
import os
import time
from typing import Any, Awaitable, Callable, Iterable, NamedTuple

# dronekit uses removed collections aliases (Python 3.10+)
collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]
collections.Mapping = collections.abc.Mapping  # type: ignore[attr-defined]
collections.MutableSet = collections.abc.MutableSet  # type: ignore[attr-defined]
collections.Callable = collections.abc.Callable  # type: ignore[attr-defined]

import dronekit
from pymavlink import mavutil
from pymavlink.dialects.v20.all import MAVLink_command_long_message

from flight.pathfinding.utils.calculate_distance import calculate_distance
import flight.pathfinding.utils.seen_by_drone as seen_by_drone
import flight.pathfinding.nodeField as nodeGen
from state_machine.flight_settings import SimMode
from flight.waypoint import (
    COLLISION_RADIUS,
    FORMATION_TOLERANCE_M,
    SEPARATION_ABORT_M,
    Waypoint,
    segment_distance,
)
import flight.collisionAvoidance
import flight.flight_log as flight_log
from flight.pathfinding.utils.goto import FlightInterrupted, move_to

# Separate logger so collision avoidance can be tuned on its own:
#   logging.getLogger("collision").setLevel(logging.DEBUG)
collision_log = logging.getLogger("collision")

HOLD_POLL_INTERVAL_S: float = 0.1  # how often a held drone re-checks the leg
HOLD_HEARTBEAT_S: float = 5.0  # how often a continuing hold is logged

# A peer silent this long stops being avoided or waited for: longer than a
# message hiccup or a hold heartbeat, short enough that a dead drone can't
# freeze the swarm on its last-known position.
PEER_STALE_S: float = 12.0

# How long a drone may hold for a peer before it gives up and lands.
HOLD_ABORT_S: float = 15.0

# POIF requirement: 20 ft.
LEG_ALTITUDE_M: float = 6.0

# Speed bounds how far two drones can drift apart in waypoint index between
# lockstep checks, so this is a formation parameter as much as a pace one.
# Groundspeed, not airspeed: a copter navigates in groundspeed, and DroneKit will
# happily send either.
LEG_GROUNDSPEED_M_S: float = 2.0

# The vehicle flies to LEG_TOLERANCE_M whenever it can
LEG_TOLERANCE_M: float = 0.35
LEG_MAX_TOLERANCE_M: float = 0.6


class FormationLost(RuntimeError):
    """The swarm is no longer in the formation the avoidance strategy assumes.

    Raised when a peer stops flying its assigned path, a hold outlasts
    HOLD_ABORT_S, or measured separation drops below SEPARATION_ABORT_M. Unlike
    FlightInterrupted and LegStalled, this vehicle is fine — the swarm isn't.
    Callers should land.
    """


class LegConflict(NamedTuple):
    """A reason this drone is not currently willing to fly its next leg."""

    peer_id: int
    clearance_m: float
    peer_from: tuple[float, float]
    peer_to: tuple[float, float]
    # "swept_path": peer outside our lockstep window, its whole advertised path
    # counts as occupied and our leg comes too close.
    # "off_formation": peer inside the window but too far from the leg it claims
    # to be flying for the barrier's guarantee to hold.
    reason: str = "swept_path"


def _describe_conflict(conflict: LegConflict) -> str:
    if conflict.reason == "off_formation":
        return (
            f"drone {conflict.peer_id} is {conflict.clearance_m:.2f}m off the leg it"
            f" reports flying (limit {FORMATION_TOLERANCE_M:.2f}m)"
        )
    if conflict.peer_from == conflict.peer_to:
        where = f"at ({conflict.peer_from[0]:.7f}, {conflict.peer_from[1]:.7f})"
    else:
        where = (
            f"between ({conflict.peer_from[0]:.7f}, {conflict.peer_from[1]:.7f})"
            f" and ({conflict.peer_to[0]:.7f}, {conflict.peer_to[1]:.7f})"
        )
    return f"drone {conflict.peer_id} {where}, clearance {conflict.clearance_m:.2f}m"


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
        self, address: str = "", baud: int | None = None, mine_radius: int = 36, id: int = 0
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
        self.field_size: tuple[int, int] = (3600, 960)
        self.mine_radius = mine_radius
        self.tasks: tuple = []
        self.seen_tracker = seen_by_drone.SightTracker(self.field_size)
        self.field: "nodeGen.Field" = None
        self.id = id
        self.start_node = None
        self.end_nodes = []
        self.waypoints = []
        self.formation_abort: str | None = None
        # Last speed sent to the autopilot, so commandPoint can skip re-sending it.
        self._commanded_groundspeed: float | None = None
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

        if self._sim_mode is not SimMode.REAL:
            return

        message_1: str = "Waiting for user input to continue... "
        message_2: str = "(press enter when ready) "
        input(f"\x1b[38;2;255;255;0m{message_1}" f"\x1b[3m{message_2}" "\x1b[0m")

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
                # MAVProxy (mavproxy-forward.service) owns /dev/serial0 exclusively so it can
                # fan the FC stream out to both Mission Planner and us. We take its loopback
                # --out. Set REAL_MAVLINK_ADDRESS=/dev/serial0 to bypass MAVProxy and talk to
                # the FC directly, which requires mavproxy-forward to be stopped.
                # udpin, not udp: MAVProxy's --out is the sender, so we bind and it dials us.
                # Bare "udp:" would work (pymavlink defaults input=True) but is direction-
                # ambiguous and depends on a dronekit default we don't control.
                self.address = os.environ.get("REAL_MAVLINK_ADDRESS", "udpin:127.0.0.1:14551")
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

    def checkForCollision(self, other_waypoints: list[Waypoint]) -> int:
        """Report where this drone's *planned* route crosses another drone's.

        Advisory only — overlapping circles cross almost everywhere. The real
        go/no-go is made per leg against live positions in `gotoWaypoint`.
        """
        collisions = Waypoint.check_for_collision(self.waypoints, other_waypoints)
        other_id = other_waypoints[0].drone_id if other_waypoints else -1
        flight_log.event(
            "route_check",
            peer=other_id,
            own_queue=flight_log.waypoints_brief(self.waypoints),
            peer_queue=flight_log.waypoints_brief(other_waypoints),
            crossings=[
                {
                    "own": [
                        c.drone1_waypoints[0].waypoint_id,
                        c.drone1_waypoints[1].waypoint_id,
                    ],
                    "peer": [
                        c.drone2_waypoints[0].waypoint_id,
                        c.drone2_waypoints[1].waypoint_id,
                    ],
                }
                for c in collisions
            ],
        )
        if not collisions:
            return 0

        collision_log.debug(
            "drone %d: planned route crosses drone %d's route at %d segment pair(s): %s",
            self.id,
            other_id,
            len(collisions),
            ", ".join(
                f"{c.drone1_waypoints[0].waypoint_id}->{c.drone1_waypoints[1].waypoint_id}"
                f" x {c.drone2_waypoints[0].waypoint_id}->{c.drone2_waypoints[1].waypoint_id}"
                for c in collisions
            ),
        )
        return len(collisions)

    def currentPosition(self) -> tuple[float, float]:
        """This drone's live position as a (latitude, longitude) pair."""
        position = self.vehicle.location.global_relative_frame
        return (position.lat, position.lon)

    def conflictsForLeg(
        self,
        leg_start: tuple[float, float],
        leg_end: tuple[float, float],
        peer_states: Iterable[Any],
        log_reason: str | None = None,
        points_per_lap: int | None = None,
        my_progress: int | None = None,
    ) -> list[LegConflict]:
        """Reasons not to fly the leg about to be flown, one per offending peer.

        The question asked depends on whether the peer is inside the lockstep
        barrier's window (it has reached `my_progress` or `my_progress + 1`).

        **Inside** — the barrier already pins the peer to the same leg we're
        flying, on its own circle, so geometry bounds the separation and a
        swept-path test is unsatisfiable here (congruent circles 3 m apart must
        intersect; the test parked the whole swarm at index 3). What's worth
        checking is the premise: `formation_drift` against the peer's *measured*
        position catches a drone blown off its circle while its waypoint
        messages still look healthy.

        **Outside** — nothing constrains where the peer is going, so its whole
        advertised path counts as occupied at COLLISION_RADIUS. A peer ahead of
        the window is provably parked by its own barrier, so it is treated as a
        point instead.

        Every live peer is avoided regardless of ID. Peers silent longer than
        PEER_STALE_S are measured and logged but not acted on — holding on a
        frozen belief is its own deadlock.
        """
        conflicts: list[LegConflict] = []
        # Stale peers are measured too, so the logs show which near misses were
        # deliberately not acted on.
        considered: list[dict[str, Any]] = []
        for state in peer_states:
            occupied = state.believed_occupancy(PEER_STALE_S)
            stale = state.is_stale(PEER_STALE_S)
            peer_progress = (
                state.last_reached_global_index(points_per_lap)
                if points_per_lap is not None
                else None
            )
            in_formation = (
                my_progress is not None
                and peer_progress is not None
                and my_progress <= peer_progress <= my_progress + 1
            )
            entry: dict[str, Any] = {
                "peer": state.drone_id,
                "stale": stale,
                "progress": peer_progress,
                "in_formation": in_formation,
            }

            if occupied is None:
                entry["clearance"] = None
                considered.append(entry)
                continue

            if in_formation:
                # The barrier is holding this peer to the same leg we are on, so
                # geometry covers the separation. Verify the premise instead.
                drift = state.formation_drift()
                entry["clearance"] = None
                entry["drift"] = drift
                considered.append(entry)
                if drift is None:
                    # Nothing to check against. Not a veto — holding on a
                    # missing measurement would deadlock the swarm — but the
                    # formation is unverified until a report arrives.
                    collision_log.debug(
                        "drone %d: no position report from drone %d, formation"
                        " integrity unverified",
                        self.id,
                        state.drone_id,
                    )
                elif drift > FORMATION_TOLERANCE_M and not stale:
                    conflicts.append(
                        LegConflict(
                            peer_id=state.drone_id,
                            clearance_m=drift,
                            peer_from=occupied[0],
                            peer_to=occupied[1],
                            reason="off_formation",
                        )
                    )
                continue

            if (
                peer_progress is not None
                and my_progress is not None
                and peer_progress > my_progress
                and state.position_is_stale(PEER_STALE_S)
            ):
                # Ahead of the window and silent: its own barrier holds it at
                # the waypoint it last reached until we catch up, so treat it as
                # parked there. Skipped when a live position exists.
                parked = state.last_reached_waypoint
                occupied = ((parked.lat, parked.long), (parked.lat, parked.long))

            clearance = segment_distance(leg_start, leg_end, occupied[0], occupied[1])
            entry["clearance"] = clearance
            entry["occupied"] = occupied
            considered.append(entry)

            if stale:
                if clearance < COLLISION_RADIUS:
                    collision_log.warning(
                        "drone %d: ignoring conflict with silent drone %d "
                        "(last belief %.2fm from leg) — no message in over %.0fs",
                        self.id,
                        state.drone_id,
                        clearance,
                        PEER_STALE_S,
                    )
                continue

            if clearance < COLLISION_RADIUS:
                conflicts.append(
                    LegConflict(
                        peer_id=state.drone_id,
                        clearance_m=clearance,
                        peer_from=occupied[0],
                        peer_to=occupied[1],
                        reason="swept_path",
                    )
                )

        # Worst first. The reasons run opposite ways (low clearance is bad, high
        # drift is bad), so rank both by margin left, negative once exceeded.
        conflicts.sort(
            key=lambda c: (
                FORMATION_TOLERANCE_M - c.clearance_m
                if c.reason == "off_formation"
                else c.clearance_m
            )
        )
        # The hold loop re-checks at 10 Hz, so callers pass a reason only for
        # the checks worth keeping in the log.
        if log_reason is not None:
            flight_log.event(
                "leg_check",
                reason=log_reason,
                leg=[leg_start, leg_end],
                radius=COLLISION_RADIUS,
                peers=considered,
                blocking=[c.peer_id for c in conflicts],
            )
        return conflicts

    def _check_still_ours(self) -> None:
        """Raise if we should stop flying: autopilot took over, or formation lost.

        Without the first check, a drone bumped out of GUIDED keeps "flying" its
        waypoint list and broadcasting progress it isn't making. The second
        surfaces a latched separation breach on the mission loop's own thread,
        so the abort happens wherever the loop is rather than between legs.
        """
        vehicle = self.vehicle
        if vehicle.mode.name != "GUIDED" or not vehicle.armed:
            raise FlightInterrupted(
                f"autopilot left our control (mode={vehicle.mode.name}," f" armed={vehicle.armed})"
            )
        if self.formation_abort is not None:
            raise FormationLost(self.formation_abort)

    def checkSeparation(self, peer_states: Iterable[Any]) -> float | None:
        """Measure separation against every peer, latching an abort if too close.

        Last line of defence, and the only check measured on both ends rather
        than reasoning about where drones are *supposed* to be. Peers whose
        position reports have dried up are skipped — an absent measurement is
        not a small distance; PEER_STALE_S covers total silence.

        Returns the closest measured separation, or None if nothing was
        measurable.
        """
        me = self.currentPosition()
        closest: float | None = None
        breaches: list[dict[str, Any]] = []
        for state in peer_states:
            peer = state.live_position
            if peer is None or state.position_is_stale(PEER_STALE_S):
                continue
            distance = segment_distance(me, me, peer, peer)
            if closest is None or distance < closest:
                closest = distance
            if distance < SEPARATION_ABORT_M:
                breaches.append({"peer": state.drone_id, "separation": distance, "at": peer})

        if breaches and self.formation_abort is None:
            nearest = min(breaches, key=lambda b: b["separation"])
            self.formation_abort = (
                f"measured separation from drone {nearest['peer']} fell to"
                f" {nearest['separation']:.2f}m (floor {SEPARATION_ABORT_M:.2f}m)"
            )
            flight_log.event(
                "separation_breach",
                floor=SEPARATION_ABORT_M,
                breaches=breaches,
                at=me,
            )
            collision_log.critical("drone %d ABORTING: %s", self.id, self.formation_abort)

        return closest

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

    async def gotoWaypoint(
        self,
        peer_states: Iterable[Any] = (),
        heartbeat: Callable[[], Awaitable[None]] | None = None,
        points_per_lap: int | None = None,
    ):
        """Fly to the next waypoint, holding position first if the leg is not clear.

        The hold is re-evaluated against where the other drones actually are and
        releases the moment the leg is clear. (Waiting for a peer to *reach* a
        named waypoint released the hold while it sat on the conflict point.)

        While holding, `heartbeat` is awaited every few seconds so peers don't
        drop us as stale after PEER_STALE_S.

        A hold outlasting HOLD_ABORT_S raises FormationLost: two in-formation
        drones can't block each other, so a hold that long means a peer has
        stopped flying its circle, which waiting won't fix.
        """
        peer_states = list(peer_states)
        curWaypoint = self.waypoints[0]
        target = (curWaypoint.lat, curWaypoint.long)
        leg_from = self.currentPosition()

        # The index we're departing from, for the parked-peer inference in
        # conflictsForLeg.
        my_progress: int | None = None
        if (
            points_per_lap is not None
            and curWaypoint.lap is not None
            and curWaypoint.index is not None
        ):
            my_progress = curWaypoint.lap * points_per_lap + curWaypoint.index - 1

        flight_log.event(
            "leg_start",
            target=flight_log.waypoint_brief(curWaypoint),
            frm=leg_from,
            queued=len(self.waypoints),
        )

        conflicts = self.conflictsForLeg(
            leg_from,
            target,
            peer_states,
            log_reason="pre_leg",
            points_per_lap=points_per_lap,
            my_progress=my_progress,
        )
        if conflicts:
            hold_started = time.monotonic()
            flight_log.event(
                "hold_start",
                target=flight_log.waypoint_brief(curWaypoint),
                conflicts=[c._asdict() for c in conflicts],
            )
            collision_log.info(
                "drone %d HOLDING short of waypoint %d: %s (need %.2fm)",
                self.id,
                curWaypoint.waypoint_id,
                "; ".join(_describe_conflict(c) for c in conflicts),
                COLLISION_RADIUS,
            )

            last_heartbeat = hold_started
            blocking = {c.peer_id for c in conflicts}
            while conflicts:
                await asyncio.sleep(HOLD_POLL_INTERVAL_S)
                self._check_still_ours()
                self.checkSeparation(peer_states)
                now = time.monotonic()
                heartbeat_due = now - last_heartbeat >= HOLD_HEARTBEAT_S
                conflicts = self.conflictsForLeg(
                    self.currentPosition(),
                    target,
                    peer_states,
                    log_reason="hold_heartbeat" if heartbeat_due else None,
                    points_per_lap=points_per_lap,
                    my_progress=my_progress,
                )

                if conflicts and now - hold_started >= HOLD_ABORT_S:
                    flight_log.event(
                        "hold_abort",
                        held_for=now - hold_started,
                        target=flight_log.waypoint_brief(curWaypoint),
                        conflicts=[c._asdict() for c in conflicts],
                    )
                    raise FormationLost(
                        f"held {now - hold_started:.0f}s short of waypoint"
                        f" {curWaypoint.waypoint_id}: "
                        + "; ".join(_describe_conflict(c) for c in conflicts)
                    )

                # Log blocker changes: a hold passing between peers and one peer
                # never moving look identical in the duration alone.
                still_blocking = {c.peer_id for c in conflicts}
                if still_blocking != blocking:
                    flight_log.event(
                        "hold_blockers_changed",
                        was=sorted(blocking),
                        now=sorted(still_blocking),
                        held_for=now - hold_started,
                    )
                    blocking = still_blocking

                if conflicts and heartbeat_due:
                    last_heartbeat = now
                    # Stay visibly alive while parked, or peers drop us as stale
                    # and fly through our position.
                    if heartbeat is not None:
                        await heartbeat()
                    flight_log.event(
                        "hold_tick",
                        held_for=now - hold_started,
                        conflicts=[c._asdict() for c in conflicts],
                    )
                    collision_log.info(
                        "drone %d still holding after %.1fs: %s",
                        self.id,
                        now - hold_started,
                        _describe_conflict(conflicts[0]),
                    )

            flight_log.event(
                "hold_end",
                held_for=time.monotonic() - hold_started,
                target=flight_log.waypoint_brief(curWaypoint),
            )
            collision_log.info(
                "drone %d CLEAR after %.1fs, proceeding to waypoint %d",
                self.id,
                time.monotonic() - hold_started,
                curWaypoint.waypoint_id,
            )

        def leg_guard() -> None:
            """Keep watching the swarm while the leg is in the air."""
            self.checkSeparation(peer_states)
            if self.formation_abort is not None:
                raise FormationLost(self.formation_abort)

        move_started = time.monotonic()
        await move_to(
            self.vehicle,
            curWaypoint.lat,
            curWaypoint.long,
            LEG_ALTITUDE_M,
            groundspeed=LEG_GROUNDSPEED_M_S,
            tolerance=LEG_TOLERANCE_M,
            max_tolerance=LEG_MAX_TOLERANCE_M,
            abort_check=leg_guard,
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

        return self.waypoints.pop(0)

    def setFieldSize(self, xMin, xMax, yMin, yMax):
        self.field = nodeGen.Field(xMin, xMax, yMin, yMax)

    # Smart landing sequence, Should be usable in final product!!
    async def recall(self):
        if self.field_size[0] - self.x < self.field_size[1] - self.y:
            self.convert_goto(
                self, [self.field_size[0] * round(self.x / self.field_size[0]), self.y], 23
            )
        else:
            self.convert_goto(
                self, [self.x, self.field_size[1] * round(self.y / self.field_size[1])], 23
            )
