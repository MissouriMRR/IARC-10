from dataclasses import dataclass

from flight.waypoint import Waypoint


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
