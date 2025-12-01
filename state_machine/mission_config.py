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
        Settings to use when running in real mode.
    airsim_mode_config : SimModeConfig
        Settings to use when running in real mode.
    simple_takeoff : bool
        Sets if flight will use a simple vertical takeoff.
    """

    run_title: str
    run_description: str
    real_mode_config: SimModeConfig
    sim_mode_config: SimModeConfig
    airsim_mode_config: SimModeConfig
    simple_takeoff: bool


def get_mission_config() -> MissionConfig:
    """
    Get the mission configuration from mission_config.json

    Returns
    -------
    MissionConfig
        The mission configuration.
    """
    config_file: TextIO
    with open("mission_config.json", "r", encoding="utf-8") as config_file:
        return json.load(config_file)
