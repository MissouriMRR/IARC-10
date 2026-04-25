"""Gets the mission configuration."""

import json
from typing import TextIO, TypedDict


class SimModeConfig(TypedDict):
    """
    A configuration containing settings specific to each sim mode.

    Attributes
    ----------
    mission_data_path : str
        The path to the JSON file containing the boundary data.
    """

    mission_data_path: str


class DroneInfo(TypedDict):
    id: int
    IP: str
    port: int


class AppInfo(TypedDict):
    ip: str
    port: str | int


class MissionConfig(TypedDict):
    """
    A configuration for a flight mission.

    Attributes
    ----------
    run_title : str
        The name for the current flight operation.
    run_description : str
        A small description for the current flight.
    real_mode_config : SimModeConfig
        Settings to use when running in real mode.
    sim_mode_config : SimModeConfig
        Settings to use when running in sim mode.
    airsim_mode_config : SimModeConfig
        Settings to use when running in airsim mode.
    simple_takeoff : bool
        Sets if flight will use a simple vertical takeoff.
    app_opperable : bool
        Whether the app is operational.
    self_id : int
        ID of this drone (can be overridden with -i flag).
    drone_info : list[DroneInfo]
        ID, IP, and port for all drones in the mission.
    app_info : AppInfo
        IP and port for the ground control app.
    speed_test_kb_data_size : int
        Payload size in KB used by network speed tests.
    range_test_toggle : bool
        Whether range test timeout logging is enabled.
    mission_field_corners : list[dict[str, float]]
        GPS coordinates (lat/lon) of the four field corners.
    start_coord : dict[str, float]
        Starting GPS coordinate (lat/lon).
    max_flight_height : float
        Maximum flight altitude in metres.
    """

    run_title: str
    run_description: str
    real_mode_config: SimModeConfig
    sim_mode_config: SimModeConfig
    airsim_mode_config: SimModeConfig
    simple_takeoff: bool
    app_opperable: bool
    self_id: int
    drones_in_mission: list[int]
    drone_info: list[DroneInfo]
    app_info: AppInfo
    speed_test_kb_data_size: int
    range_test_toggle: bool
    mission_field_corners: list[dict[str, float]]
    start_coord: dict[str, float]
    max_flight_height: float


def get_mission_config(config_path: str) -> MissionConfig:
    """
    Get the mission configuration from mission_config.json

    Returns
    -------
    MissionConfig
        The mission configuration.
    """
    config_file: TextIO
    with open(config_path, "r", encoding="utf-8") as config_file:
        return json.load(config_file)
