import time
from dataclasses import dataclass

from flight.waypoint import Waypoint, segment_distance


@dataclass
class DroneState:
    """
    Represents the status and information about a specific drone.
    Tracks hardware, network address, and specific flags.
    """

    def __init__(self, drone_id: int, drone_ip: str):

        # --- Physical Identity variables
        self._drone_id: int = drone_id
        self._drone_ip: str = drone_ip

        # --- Safety/Power flags
        self._armed: bool = False  # True if motors are on
        self._takeoff: bool = False  # True if drone is off the ground

        # --- Connectivity Monitoring ---
        # Stores the latest ping response status. None = No response yet, False = PING_NACK, True = PING_ACK
        self._ping_response: bool = False

        # --- Mission Control Flags ---
        self._demo_start: bool = False  # Flag for pre-programmed demonstration mode
        self._mission_start: bool = False  # Flag for autonomous mission execution
        self._list_of_waypoints: list[Waypoint] = []  # Collection of GPS/coordinate targets
        self._waypoint_up_to_date: bool = False
        # Last waypoint this drone reported reaching. Together with the head of
        # list_of_waypoints it bounds where the drone can physically be right now,
        # which is what collision avoidance needs. None until its first report.
        self._last_reached_waypoint: Waypoint | None = None
        # Last position this drone actually reported measuring, as (lat, lon), and when it arrived (monotonic clock).
        self._live_position: tuple[float, float] | None = None
        self._live_position_at: float | None = None
        # Altitude that came with that position, for logging and for telling a
        # genuine horizontal conflict from two drones at different heights.
        self._live_altitude: float | None = None
        # When we last heard anything from this drone (monotonic clock). None
        # until the first message. A drone that goes silent — crashed, landed on
        # a failsafe, lost comms — must eventually stop blocking the swarm: its
        # believed position freezes wherever it last reported, and holding on a
        # frozen belief forever is the deadlock we saw when drone 3's EKF
        # failsafe put it in LAND mid-demo.
        self._last_heard_monotonic: float | None = None

    # --- Property Accessors ---
    # These provide a controlled interface for reading/writing internal attributes

    # Drone ID: Unique identifier for the drone
    @property
    def drone_id(self) -> int:
        return self._drone_id

    @drone_id.setter
    def drone_id(self, value) -> None:
        self._drone_id = value

    # Drone IP: The network address used for communication
    @property
    def drone_ip(self) -> str:
        return self._drone_ip

    @drone_ip.setter
    def drone_ip(self, value) -> None:
        self._drone_ip = value

    # Armed Status: Controls the drone motors to spend
    @property
    def armed(self) -> bool:
        return self._armed

    @armed.setter
    def armed(self, value) -> None:
        self._armed = value

    # Ping Response: Used to monitor link latency
    @property
    def ping_response(self) -> bool:
        return self._ping_response

    @ping_response.setter
    def ping_response(self, value: bool) -> None:
        self._ping_response = value

    # Takeoff: Tracks if the drone has transitioned to flight
    @property
    def takeoff(self) -> bool:
        return self._takeoff

    @takeoff.setter
    def takeoff(self, value) -> None:
        self._takeoff = value

    # Demo Mode: Used for testing
    @property
    def demo_start(self) -> bool:
        return self._demo_start

    @demo_start.setter
    def demo_start(self, value) -> None:
        self._demo_start = value

    # Mission Status: Indicates if the drone is executing its primary objective
    @property
    def mission_start(self) -> bool:
        return self._mission_start

    @mission_start.setter
    def mission_start(self, value) -> None:
        self._mission_start = value

    # Waypoints: A list of coordinates the drone must visit
    @property
    def list_of_waypoints(self) -> list[Waypoint]:
        return self._list_of_waypoints

    @list_of_waypoints.setter
    def list_of_waypoints(self, value: list[Waypoint]) -> None:
        self._list_of_waypoints = value

    # Last Reached Waypoint: best available estimate of where this drone is now
    @property
    def last_reached_waypoint(self) -> Waypoint | None:
        return self._last_reached_waypoint

    @last_reached_waypoint.setter
    def last_reached_waypoint(self, value: Waypoint | None) -> None:
        self._last_reached_waypoint = value

    def touch(self) -> None:
        """Record that a message from this drone just arrived."""
        self._last_heard_monotonic = time.monotonic()

    def is_stale(self, max_silence_s: float) -> bool:
        """True when this drone has been silent longer than max_silence_s.

        A drone we have never heard from at all is also stale: we have no
        position for it, so there is nothing to avoid or wait for.
        """
        if self._last_heard_monotonic is None:
            return True
        return time.monotonic() - self._last_heard_monotonic > max_silence_s

    def last_reached_global_index(self, points_per_lap: int) -> int | None:
        """Progress of this drone around its circle as a single global index.

        lap * points_per_lap + index for the last waypoint it reported
        reaching, or None if it has not reached any lapped waypoint yet.
        """
        wp = self._last_reached_waypoint
        if wp is None or wp.lap is None or wp.index is None:
            return None
        return wp.lap * points_per_lap + wp.index

    @property
    def live_position(self) -> tuple[float, float] | None:
        """Last position this drone reported measuring, or None if never."""
        return self._live_position

    @property
    def live_altitude(self) -> float | None:
        """Altitude that came with the last reported position."""
        return self._live_altitude

    def set_live_position(self, lat: float, lon: float, alt: float | None = None) -> None:
        """Record a position report from this drone."""
        self._live_position = (lat, lon)
        self._live_altitude = alt
        self._live_position_at = time.monotonic()

    def position_is_stale(self, max_age_s: float) -> bool:
        """True when no position report has arrived recently enough to trust.

        A drone we have never had a position from is stale too — an absent
        measurement is not evidence that the drone is where its waypoints say.
        """
        if self._live_position_at is None:
            return True
        return time.monotonic() - self._live_position_at > max_age_s

    def formation_drift(self) -> float | None:
        """How far this drone's reported position is from the path it claims.

        The drone says it reached W and is heading for X, so it should be
        somewhere on the segment W->X. This is the distance from its actual
        reported position to that segment: zero while it flies the plan, growing
        as it stops flying the plan. None when either half is unknown.

        This is what makes the lockstep guarantee checkable. The barrier keeps
        the formation safe on the assumption that every drone is on its assigned
        leg; without an independent measurement, avoidance can only restate that
        assumption back to itself.
        """
        occupied = self.occupied_segment()
        if occupied is None or self._live_position is None:
            return None
        return segment_distance(self._live_position, self._live_position, occupied[0], occupied[1])

    def occupied_segment(self) -> tuple[tuple[float, float], tuple[float, float]] | None:
        """The stretch of air this drone's *plan* says it may occupy.

        A drone that reported reaching W and is now heading for X should be
        somewhere on W->X. Returns a degenerate segment (both points equal) when
        only one end is known, and None when nothing is known about the drone
        yet.

        This is the planned occupancy, which is what `formation_drift` measures
        the reported position against. For deciding whether it is safe to fly,
        use `believed_occupancy` — it prefers the measurement.
        """
        here = self._last_reached_waypoint
        target = self._list_of_waypoints[0] if self._list_of_waypoints else None

        if here is None and target is None:
            return None
        if here is None:
            return ((target.lat, target.long), (target.lat, target.long))
        if target is None:
            return ((here.lat, here.long), (here.lat, here.long))
        return ((here.lat, here.long), (target.lat, target.long))

    def believed_occupancy(
        self, max_age_s: float
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        """Best available answer to "where is this drone right now?".

        A fresh position report is a measurement of where the drone is, good to
        however far it can travel between reports — far better than the whole
        waypoint-to-waypoint segment its plan permits, which at the start of the
        demo is five metres long. Using the plan instead is what let two drones
        reach 0.12 m in the 2026-08-07 5 m run: the drone that was still parked
        on its hover point was modelled at the waypoint it had been *sent to*,
        five metres away, so its neighbour flew straight onto it.

        Falls back to the planned segment when no recent report exists, since a
        conservative guess beats no guess.
        """
        if self._live_position is not None and not self.position_is_stale(max_age_s):
            return (self._live_position, self._live_position)
        return self.occupied_segment()

    @property
    def waypoint_up_to_date(self) -> bool:
        return self._waypoint_up_to_date

    @waypoint_up_to_date.setter
    def waypoint_up_to_date(self, value) -> None:
        self._waypoint_up_to_date = value

    # --- Logic Methods ---

    def update_state(self):
        """
        Placeholder for logic that synchronizes local state
        with incoming data from drone.
        """
        pass

    def clear_waypoints(self):
        """
        Resets the mission path by emptying the waypoint list.
        """
        self.list_of_waypoints = []
