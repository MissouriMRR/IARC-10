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
        app_opperable: bool = False,
        title: str = DEFAULT_RUN_TITLE,
        description: str = DEFAULT_RUN_DESCRIPTION,
        drone_ID: int = 1,
        drones_in_mission: list[int] = [1, 2, 3, 4],
        drone_info: list[mission_config.DroneInfo] = [],
        app_IP: str = "",
        app_port: int = 0,
        mission_corners: list[dict[str, float]] | None = None,
        max_height: float = 10,
        start_coord: dict = {},
        mission_type: str = "",
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
        self.__app_opperable: bool = app_opperable
        self.__app_IP: str = app_IP
        self.__app_port: int = app_port
        self.__current_drone_ID = drone_ID
        self.__drones_in_mission: list[int] = list(drones_in_mission)
        self.__drone_info: list[mission_config.DroneInfo] = list(drone_info)
        self.__mission_field_corners: list[dict[str, float]] = mission_corners or []
        self.__max_flight_height: float = max_height
        self.__start_coord: dict = start_coord
        self.__sim_mode: SimMode = sim_mode
        self.__mission_data_path: str = mission_data_path
        self.__mission_type: str = mission_type
        self.__yolo_status: Event = Event()

    @staticmethod
    def from_mission_config(self_id: int | None = None) -> "FlightSettings":
        """
        Creates a new FlightSettings object from the mission config file and command line
        arguments

        Parameters
        ----------
        self_id : int | None
            Override for which drone ID is "self". Defaults to self_id in the config file.

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
        config_path: str
        if "--config" in sys.argv:
            config_index: int = sys.argv.index("--config") + 1
            if config_index < len(sys.argv):
                config_path = sys.argv[config_index]
                logging.info("Using mission config file at %s", config_path)
            else:
                config_path = "mission_config.json"

        else:
            logging.warning("--config flag passed without a path. Using default mission config.")
            config_path = "mission_config.json"

        config: MissionConfig = mission_config.get_mission_config(config_path)
        resolved_id: int = self_id if self_id is not None else config["self_id"]

        sim_mode_config: SimModeConfig = (
            config["airsim_mode_config"]
            if airsim_flag
            else (config["sim_mode_config"] if sim_flag else config["real_mode_config"])
        )

        all_drones: list[mission_config.DroneInfo] = config["drone_info"]
        if not any(d["id"] == resolved_id for d in all_drones):
            raise ValueError(f"Drone ID {resolved_id} not found in drone_info")

        config_settings: FlightSettings = FlightSettings(
            simple_takeoff=config["simple_takeoff"],
            app_opperable=config["app_opperable"],
            title=config["run_title"],
            description=config["run_description"],
            drone_ID=resolved_id,
            drones_in_mission=list(config["drones_in_mission"]),
            drone_info=all_drones,
            app_IP=config["app_info"]["ip"],
            app_port=int(config["app_info"]["port"]),
            mission_corners=config["mission_field_corners"],
            max_height=config["max_flight_height"],
            start_coord=config["start_coord"],
            mission_type=config["mission_type"],
            sim_mode=sim_mode,
            mission_data_path=sim_mode_config["mission_data_path"],
        )
        return config_settings

    @property
    def mission_type(self) -> str:
        """
        Returns the mission type

        Returns
        -------
        mission_type : str
            The mission type, either "Prompted" or "Automatic"
        """
        return self.__mission_type

    @mission_type.setter
    def mission_type(self, mission_type: str) -> None:
        """
        Sets the mission type

        Parameters
        ----------
        mission_type : str
            The mission type, either "Prompted" or "Automatic"
        """
        if mission_type not in ["Prompted", "Automatic"]:
            raise ValueError("mission_type must be either 'Prompted' or 'Automatic'")
        self.__mission_type = mission_type

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

    @property
    def app_opperable(self) -> bool:
        """
        Gets app_opperable as a private member variable

        Returns
        -------
        app_opperable : bool
            Flag for if app is active
        """
        return self.__app_opperable

    @app_opperable.setter
    def app_opperable(self, app_opperable: bool) -> None:
        """
        Sets the flag for if app is opperational

        Parameters
        ----------
        app_opperable : bool
            Flag for if app is active
        """
        self.__app_opperable = app_opperable

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
    def current_drone_ID(self) -> int:
        """
        Returns this drone's ID integer

        Returns
        -------
        current_drone_ID : int
            Integer ID for the current drone
        """
        return int(self.__current_drone_ID)

    @current_drone_ID.setter
    def current_drone_ID(self, drone_ID: str) -> None:
        """
        Sets a new ID for this drone

        Parameters
        ----------
        current_drone_ID : int
            New ID for the current drone
        """
        self.__current_drone_ID = drone_ID

    # Other drones in mission_is_used to get other drones ids
    @property
    def other_drones_in_mission(self) -> list[int]:

        return [
            drone_id for drone_id in self.__drones_in_mission if drone_id != self.__current_drone_ID
        ]

    @property
    def drones_in_mission(self) -> list[int]:
        return self.__drones_in_mission

    @drones_in_mission.setter
    def drones_in_mission(self, drones_in_mission: list[int]) -> None:
        self.__drones_in_mission = list(drones_in_mission)

    @property
    def drone_info(self) -> list[mission_config.DroneInfo]:
        return self.__drone_info

    @drone_info.setter
    def drone_info(self, info: list[mission_config.DroneInfo]) -> None:
        self.__drone_info = list(info)

    @property
    def app_IP(self) -> str:
        """
        Returns the app's IP address

        Returns
        -------
        app_IP : str
            IP for the app that the drone tries to connect to
        """
        return self.__app_IP

    @app_IP.setter
    def app_IP(self, app_IP: str) -> None:
        """
        Sets a new IP address the app

        Parameters
        ----------
        app_IP : str
            New IP for the app that the drone tries to connect to
        """
        self.__app_IP = app_IP

    @property
    def app_port(self) -> int:
        return self.__app_port

    @app_port.setter
    def app_port(self, port: int) -> None:
        self.__app_port = port

    @property
    def number_of_total_drones(self) -> int:
        return len(self.__drone_info)

    @property
    def other_drone_info(self) -> list[mission_config.DroneInfo]:
        return [d for d in self.__drone_info if d["id"] != self.__current_drone_ID]

    @property
    def mission_field_corners(self) -> list[dict[str, float]]:
        """
        Returns

        Returns
        -------
         :

        """
        return self.__mission_field_corners

    @mission_field_corners.setter
    def mission_field_corners(self, field_corners: list[dict[str, float]]) -> None:
        """
        Sets a new

        Parameters
        ----------
         :
            New
        """
        self.__mission_field_corners = field_corners

    @property
    def start_coord(self) -> dict:
        """
        Returns

        Returns
        -------
         :

        """
        return self.__start_coord

    @start_coord.setter
    def start_coord(self, start_coord: dict) -> None:
        """
        Sets a new

        Parameters
        ----------
         :
            New
        """
        self.__start_coord = start_coord

    @property
    def max_flight_height(self) -> float:
        """
        Returns

        Returns
        -------
         :

        """
        return self.__max_flight_height

    @max_flight_height.setter
    def max_flight_height(self, max_height: float) -> None:
        """
        Sets a new

        Parameters
        ----------
         :
            New
        """
        self.__max_flight_height = max_height

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

    def get_drone_by_id(self, drone_id: int) -> mission_config.DroneInfo:
        for drone in self.__drone_info:
            if drone["id"] == drone_id:
                return drone
        raise ValueError(f"Drone ID {drone_id} not found in drone_info")

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
