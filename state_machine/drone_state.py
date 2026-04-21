from dataclasses import dataclass

from state_machine.flight_settings import FlightSettings


@dataclass
class DroneState:
    """
    Represents the status and information about a specific drone.
    Tracks hardware, network address, and specific flags.
    """

    def __init__(self, flight_settings: FlightSettings):
        # --- Flight Settings Values
        self.flight_settings: FlightSettings = flight_settings

        # --- Physical Identity variables
        self._drone_id: int = self.flight_settings.current_drone_ID
        self._drone_ip: str = self.flight_settings.drone_info[self.drone_id - 1]["IP"]

        # --- Safety/Power flags
        self._armed: bool = False  # True if motors are on
        self._takeoff: bool = False  # True if drone is off the ground

        # --- Connectivity Monitoring ---
        self._ping_response: dict[
            int, bool
        ] = {}  # Stores the latest ping response status

        # --- Mission Control Flags ---
        self._demo_start: bool = False  # Flag for pre-programmed demonstration mode
        self._mission_start: bool = False  # Flag for autonomous mission execution
        self._list_of_waypoints = []  # Collection of GPS/coordinate targets TODO GIVE ME A TYPE

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
    def drone_ip(self):
        return self._drone_ip

    @drone_ip.setter
    def drone_ip(self, value):
        self._drone_ip = value

    # Armed Status: Controls the drone motors to spend
    @property
    def armed(self):
        return self._armed

    @armed.setter
    def armed(self, value):
        self._armed = value

    # Ping Response: Used to monitor link latency
    @property
    def ping_response(self):
        return self._ping_response

    @ping_response.setter
    def ping_response(self, value):
        self._ping_response = value

    # Takeoff: Tracks if the drone has transitioned to flight
    @property
    def takeoff(self):
        return self._takeoff

    @takeoff.setter
    def takeoff(self, value):
        self._takeoff = value

    # Demo Mode: Used for testing
    @property
    def demo_start(self):
        return self._demo_start

    @demo_start.setter
    def demo_start(self, value):
        self._demo_start = value

    # Mission Status: Indicates if the drone is executing its primary objective
    @property
    def mission_start(self):
        return self._mission_start

    @mission_start.setter
    def mission_start(self, value):
        self._mission_start = value

    # Waypoints: A list of coordinates the drone must visit
    @property
    def list_of_waypoints(self):
        return self._list_of_waypoints

    @list_of_waypoints.setter
    def list_of_waypoints(self, value):
        self._list_of_waypoints = value

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
