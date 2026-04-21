from typing import TypedDict
from state_machine.flight_settings import FlightSettings


class DroneInfo(TypedDict):
    id: int
    IP: str
    port: int


class AppInfo(TypedDict):
    ip: str
    port: str | int


class NetworkConfig:
    def __init__(self, flight_settings: FlightSettings = FlightSettings()) -> None:
        self._self_id: int = flight_settings.current_drone_ID
        self._drone_info: list[DroneInfo] = list(flight_settings.drone_info)
        self._app_ip: str = flight_settings.app_IP
        self._app_port: int = flight_settings.app_port
        self._speed_test_data_size: int = 16
        self._range_test_toggle: bool = False
        self._other_drones: list[DroneInfo] = []
        for drone in self._drone_info:
            if drone["id"] != self._self_id:
                self._other_drones.append(drone)

    def _find_drone(self, droneId: int) -> DroneInfo:
        for drone in self._drone_info:
            if drone["id"] == droneId:
                return drone
        raise ValueError(f"Drone ID {droneId} not found in drone_info")

    def get_drone_ip(self, droneId: int) -> str:
        return str(self._find_drone(droneId)["IP"])

    def get_drone_port(self, droneId: int) -> int:
        return int(self._find_drone(droneId)["port"])

    def get_other_drones(self) -> list[DroneInfo]:
        return self._other_drones

    def get_app_ip(self) -> str:
        return self._app_ip

    def set_app_ip(self, newIP: str) -> None:
        self._app_ip = newIP

    def get_app_port(self) -> int:
        return self._app_port

    def set_app_port(self, newPort: int) -> None:
        self._app_port = newPort

    def get_self_id(self) -> int:
        return self._self_id

    def set_self_id(self, newId: int) -> None:
        if self._self_id == newId:
            return
        self._find_drone(newId)  # raises ValueError if not found
        self._self_id = newId

    def get_speed_test_data_size(self) -> int:
        return self._speed_test_data_size

    def get_range_test_toggle(self) -> bool:
        return self._range_test_toggle

    def set_range_test_toggle(self, rangeTestToggle: bool) -> None:
        self._range_test_toggle = rangeTestToggle

    def get_number_of_drones(self) -> int:
        return len(self._drone_info)
