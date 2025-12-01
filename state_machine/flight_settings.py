"""Class to contain setters, getters & parameters for current flight"""

from asyncio import Event
from enum import Enum
import logging
import sys
from typing import Final

from state_machine import mission_config
from state_machine.mission_config import MissionConfig, SimModeConfig

DEFAULT_RUN_TITLE: Final[str] = "IARC Test Flight"
DEFAULT_RUN_DESCRIPTION: Final[str] = "Test flight for IARC 2026"


class SimMode(Enum):
    """
    Distinguishes whether a drone is real, running in the sim, or running in airsim.
    """

    REAL = "real"
    SIM = "sim"
    AIRSIM = "airsim"


# pylint: disable=too-many-instance-attributes
class FlightSettings:
    """
    Class to contain basic information for a flight, as well as some flight parameters

    Attributes
    ----------
    _read_sim_mode: bool
        Whether the sim mode has been read. Used to determine when to show the message
        about the sim mode.
    __simple_takeoff: bool
        Sets if the drone will ascend vertically or at an angle
    __run_title: str
        The name for the current flight operation
    __run_description: str
        A small description for the current flight
    __sim_mode: SimMode
        Whether the drone is real, running in the ardupilot sim, or running in airsim
    __mission_data_path: str
        The path to the JSON file containing the boundary data.
    __yolo_status: Event
        An asyncio Event tracking whether the YOLO model has
        finished processing images.

    Methods
    -------
    from_mission_config() -> FlightSettings
        Creates a new FlightSettings object from the mission config
    simple_takeoff() -> bool
        Returns the status of the takeoff type for the flight
    simple_takeoff(simple_takeoff: bool) -> None
        Sets the parameter for a simple or diagonal takeoff
    run_title() -> str
        `Returns the flight title
    run_title(new_title: str) -> None
        Sets a new title for the current flight
    run_description() -> str
        Returns the small description for the current flight
    run_description(new_description: str) -> None
        Sets a new description for the new flight
    sim_mode() -> SimMode
        Returns the simulation mode
    sim_mode(sim_mode: SimMode) -> None
        Sets the simulation mode
    mission_data_path() -> str
        Return the path to the JSON file containing the boundary data.
    mission_data_path(mission_data_path: str) -> None
        Set the path to the JSON file containing the boundary data.
    """

    _read_sim_mode: bool = False

    # pylint: disable=too-many-arguments
    def __init__(
        self,
        simple_takeoff: bool = False,
        title: str = DEFAULT_RUN_TITLE,
        description: str = DEFAULT_RUN_DESCRIPTION,
        sim_mode: SimMode = SimMode.REAL,
        mission_data_path: str = "flight/data/golf_data.json",
    ) -> None:
        """
        Default Constructor for flight settings

        Parameters
        ----------
        simple_takeoff : bool, default False
            Sets if flight will use a simple vertical takeoff.
        title : str
            The name for the flight execution.
        description : str
            Sets a descriptive explanation for the current flight execution.
        sim_mode : SimMode, default SimMode.REAL
            Whether the drone is real, running in the ardupilot sim, or running in airsim.
        mission_data_path : str, default "flight/data/golf_data.json"
            The path to the JSON file containing the boundary data.
        """
        self.__simple_takeoff: bool = simple_takeoff
        self.__run_title: str = title
        self.__run_description: str = description
        self.__sim_mode: SimMode = sim_mode
        self.__mission_data_path: str = mission_data_path
        self.__yolo_status: Event = Event()

    @staticmethod
    def from_mission_config() -> "FlightSettings":
        """
        Creates a new FlightSettings object from the mission config file and command line
        arguments

        Returns
        -------
        FlightSettings
            A FlightSettings object with settings from mission_config.json.
        """
        sim_flag: bool = "-s" in sys.argv or "--sim" in sys.argv
        airsim_flag: bool = "-a" in sys.argv or "--airsim" in sys.argv
        sim_mode: SimMode = (
            SimMode.AIRSIM if airsim_flag else SimMode.SIM if sim_flag else SimMode.REAL
        )
        if not FlightSettings._read_sim_mode:
            FlightSettings._read_sim_mode = True
            logging.info(
                "Running in %s mode."
                " Pass -s or --sim to run in sim mode."
                " Pass -a or --airsim to run in airsim mode.",
                sim_mode.name,
            )

        config: MissionConfig = mission_config.get_mission_config()
        sim_mode_config: SimModeConfig = (
            config["airsim_mode_config"]
            if airsim_flag
            else (config["sim_mode_config"] if sim_flag else config["real_mode_config"])
        )
        config_settings: FlightSettings = FlightSettings(
            config["simple_takeoff"],
            config["run_title"],
            config["run_description"],
            sim_mode,
            sim_mode_config["mission_data_path"],
        )
        return config_settings

    # ----- Takeoff Settings ----- #
    @property
    def simple_takeoff(self) -> bool:
        """
        Gets simple_takeoff as a private member variable

        Returns
        -------
        simple_takeoff : bool
            Flag for vertical takeoff implementation or other
        """
        return self.__simple_takeoff

    @simple_takeoff.setter
    def simple_takeoff(self, simple_takeoff: bool) -> None:
        """
        Sets the flag for vertical takeoff

        Parameters
        ----------
        simple_takeoff : bool
            Flag for vertical takeoff
        """
        self.__simple_takeoff = simple_takeoff

    # ----- Flight Initialization Settings ----- #
    @property
    def run_title(self) -> str:
        """
        Return the title for the current flight execution

        Returns
        -------
        run_title : str
            Current title for the flight
        """
        return self.__run_title

    @run_title.setter
    def run_title(self, new_title: str) -> None:
        """
        Sets a new title for the current flight

        Parameters
        ----------
        new_title : str
            New title for the flight execution
        """
        self.__run_title = new_title

    @property
    def run_description(self) -> str:
        """
        Returns the description for the current flight execution

        Returns
        -------
        run_description : str
            Detailed description for the current flight plan
        """
        return self.__run_description

    @run_description.setter
    def run_description(self, new_description: str) -> None:
        """
        Sets a new description for the flight

        Parameters
        ----------
        new_description : str
            New description for the current flight
        """
        self.__run_description = new_description

    @property
    def sim_mode(self) -> SimMode:
        """
        Returns the simulation mode

        Returns
        -------
        sim_mode : SimMode
            The simulation mode
        """
        return self.__sim_mode

    @sim_mode.setter
    def sim_mode(self, sim_mode: SimMode) -> None:
        """
        Sets the simulation mode

        Parameters
        ----------
        sim_mode : SimMode
            The simulation mode
        """
        self.__sim_mode = sim_mode

    @property
    def mission_data_path(self) -> str:
        """
        Return the path to the JSON file containing the boundary data.

        Returns
        -------
        mission_data_path : str
            The path to the JSON file containing the boundary data.
        """
        return self.__mission_data_path

    @mission_data_path.setter
    def mission_data_path(self, mission_data_path: str) -> None:
        """
        Set the path to the JSON file containing the boundary data.

        Parameters
        ----------
        mission_data_path : str
            The path to the JSON file containing the boundary data.
        """

    @property
    def yolo_status(self) -> Event:
        """
        Return the asyncio Event that tracks for completion of image processing.

        Returns
        -------
        yolo_status : Event
            The Event with status tracking the YOLO model.
        """
        return self.__yolo_status
