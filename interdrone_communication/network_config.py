import json
from typing import TypedDict
from pathlib import Path


class DroneInfo(TypedDict):
    id: int
    IP: str
    port: int


class AppInfo(TypedDict):
    ip: str
    port: str | int


class MissionConfigData(TypedDict):
    current_drone_info: DroneInfo
    number_of_total_drones: int
    other_drone_info: list[DroneInfo]
    app_info: AppInfo


class NetworkConfig:
    def __init__(self) -> None:
        self.config: MissionConfigData
        config_path = Path(__file__).resolve().parent.parent / "mission_config.json"
        with config_path.open("r", encoding="utf-8") as file:
            self.config = json.load(file)

    def _find_drone(self, droneId: int) -> DroneInfo:
        if self.config["current_drone_info"]["id"] == droneId:
            return self.config["current_drone_info"]
        for drone in self.config["other_drone_info"]:
            if drone["id"] == droneId:
                return drone
        raise ValueError(f"Drone ID {droneId} not found in mission_config.json")

    def get_drone_ip(self, droneId: int) -> str:
        return str(self._find_drone(droneId)["IP"])

    def get_drone_port(self, droneId: int) -> int:
        return int(self._find_drone(droneId)["port"])

    def get_app_ip(self) -> str:
        return str(self.config["app_info"]["ip"])

    def set_app_ip(self, newIP: str) -> None:
        self.config["app_info"]["ip"] = newIP

    def get_app_port(self) -> int:
        return int(self.config["app_info"]["port"])

    def set_app_port(self, newPort: int) -> None:
        self.config["app_info"]["port"] = newPort

    def get_self_id(self) -> int:
        return int(self.config["current_drone_info"]["id"])

    def set_self_id(self, newId: int) -> None:
        old_self = self.config["current_drone_info"]
        if old_self["id"] == newId:
            return
        for i, drone in enumerate(self.config["other_drone_info"]):
            if drone["id"] == newId:
                self.config["other_drone_info"][i] = old_self
                self.config["current_drone_info"] = drone
                return
        raise ValueError(f"Drone ID {newId} not found in mission_config.json")

    def get_speed_test_data_size(self) -> int:
        return int(self.config.get("speed_test_kb_data_size", 16))  # type: ignore[call-overload]

    def get_range_test_toggle(self) -> bool:
        return bool(self.config.get("range_test_toggle", False))  # type: ignore[call-overload]

    def set_range_test_toggle(self, rangeTestToggle: bool) -> None:
        self.config["range_test_toggle"] = rangeTestToggle  # type: ignore[typeddict-unknown-key]

    def get_number_of_drones(self) -> int:
        return int(self.config["number_of_total_drones"])
