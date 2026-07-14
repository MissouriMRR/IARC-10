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
    latitude: float
    longitude: float


class LidarConfig(TypedDict):
    """
    Settings for the LIDAR obstacle detection and mapping mode.

    Attributes
    ----------
    enabled : bool
        Whether the background proximity monitor runs at all.
    proximity_threshold_m : float
        Range in meters below which an object queues a mapping scan.
    standoff_radius_m : float
        Radius in meters of the circle flown around a detected object.
    circle_num_points : int
        Number of waypoints (sampling stops) on the scan circle.
    dedupe_radius_ft : float
        Objects whose centers are within this many feet of an already
        scanned object are considered the same object and skipped.
    max_object_radius_ft : float
        Scan returns farther than this from the object center are treated
        as belonging to a different obstacle and discarded.
    """

    enabled: bool
    proximity_threshold_m: float
    standoff_radius_m: float
    circle_num_points: int
    dedupe_radius_ft: float
    max_object_radius_ft: float


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
    lidar_config : LidarConfig
        Settings for the LIDAR obstacle detection and mapping mode.
        Optional; LIDAR mode is disabled when absent.
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
    mission_type: str
    max_flight_height: float
    lidar_config: LidarConfig


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
