import asyncio
from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class PersistentConnection:
    reader: asyncio.StreamReader
    writer: asyncio.StreamWriter
    lastUsed: float


@dataclass
class DroneState:

    def __init__(self, drone_id=None, drone_ip=None):
        # Member Variables / Flags
        
        
        self.drone_id = drone_id
        self.drone_ip = drone_ip
        self.armed = False
        self.ping_response = None
        self.takeoff = False
        self.demo_start = False
        self.mission_start = False
        self.list_of_waypoints = []

    # --- Functions ---

# --- Drone ID ---
    @property
    def drone_id(self):
        return self._drone_id

    @drone_id.setter
    def drone_id(self, value):
        self._drone_id = value

    # --- Drone IP ---
    @property
    def drone_ip(self):
        return self._drone_ip

    @drone_ip.setter
    def drone_ip(self, value):
        self._drone_ip = value

    # --- Armed ---
    @property
    def armed(self):
        return self._armed

    @armed.setter
    def armed(self, value):
        self._armed = value

    # --- Ping Response ---
    @property
    def ping_response(self):
        return self._ping_response

    @ping_response.setter
    def ping_response(self, value):
        self._ping_response = value

    # --- Takeoff ---
    @property
    def takeoff(self):
        return self._takeoff

    @takeoff.setter
    def takeoff(self, value):
        self._takeoff = value

    # --- Demo Start ---
    @property
    def demo_start(self):
        return self._demo_start

    @demo_start.setter
    def demo_start(self, value):
        self._demo_start = value

    # --- Mission Start ---
    @property
    def mission_start(self):
        return self._mission_start

    @mission_start.setter
    def mission_start(self, value):
        self._mission_start = value

    # --- Waypoints ---
    @property
    def list_of_waypoints(self):
        return self._list_of_waypoints

    @list_of_waypoints.setter
    def list_of_waypoints(self, value):
        self._list_of_waypoints = value
    def update_state(self):
        pass

    def clear_waypoints(self):

        self.list_of_waypoints = []
