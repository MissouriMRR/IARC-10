"""Defines the Drone class for the state machine."""

import asyncio
import collections
import collections.abc
import json
import logging

# dronekit uses removed collections aliases (Python 3.10+)
collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]
collections.Mapping = collections.abc.Mapping  # type: ignore[attr-defined]
collections.MutableSet = collections.abc.MutableSet  # type: ignore[attr-defined]
collections.Callable = collections.abc.Callable  # type: ignore[attr-defined]

import dronekit
from pymavlink import mavutil
from pymavlink.dialects.v20.all import MAVLink_command_long_message

from flight.pathfinding.utils.calculate_distance import calculate_distance
import flight.pathfinding.utils.seen_by_drone as seen_by_drone
import flight.pathfinding.node_generation as nodeGen
from state_machine.flight_settings import SimMode
from flight.waypoint import Waypoint
import flight.collisionAvoidance
from flight.pathfinding.utils.goto import move_to


class Drone:
    """
    A drone for the state machine to control.
    This class is a wrapper around the dronekit Vehicle class,
    and will be passed around to each state.
    Data can be stored in this class to be shared between states.

    Attributes
    ----------
    address : str
        The address used to connect to the drone.
    baud : int | None
        The baud rate, or None to use the default.
    is_connected
    sim_mode
    _sim_mode : SimMode | None
        The simulation mode of this drone, or None if one has not been specified.
    vehicle
    _vehicle : dronekit.Vehicle | None
        The Dronekit Vehicle object that controls the drone, or None if a connection
        hasn't been made yet.

    Methods
    -------
    __init__(connection_string: str) -> None
        Initialize a new Drone object, but do not connect to a drone.
    arm(self) -> Awaitable[None]
        Arm the drone.
    close(self) -> Awaitable[None]
        Close the owned DroneKit Vehicle object.
    close_servo(self, servo_num: int) -> Awaitable[None]:
        Close the servo with the given number.
    connect_drone(self) -> Awaitable[None]
        Connect to a drone.
    is_connected(self) -> bool
        Checks if a drone has been connected to.
    open_servo(self, servo_num: int) -> Awaitable[None]:
        Open the servo with the given number.
    remove_arming_check(self) -> None
        For use with airsim.
    return_to_launch(self) -> Awaitable[none]
        Method to move vehicle above home location, then descend vertically.
    takeoff(self, takeoff_alt: float) -> Awaitable[None]
        Takeoff vertically to the passed altitude.
    use_settings(self, sim_mode: SimMode) -> None
        Modify the connection settings based on the given simulation mode.
    vehicle(self) -> dronekit.Vehicle
        Get the Dronekit Vehicle object owned by this Drone object.
    """

    def __init__(
        self, address: str = "", baud: int | None = None, mine_radius: int = 36, id: int = 0
    ) -> None:
        """
        Initialize a new Drone object, but do not connect to a drone.

        Parameters
        ----------
        address : str, default ""
            The address of the drone to connect to when the `connect_drone()`
            method is called.
        baud : int, default None
            The baud rate, or None to use the default.
        """
        self._sim_mode: SimMode | None = None
        self._vehicle: dronekit.Vehicle | None = None
        self.address: str = address
        self.baud: int | None = baud
        self.field_size: tuple[int, int] = [3600, 960]
        self.mine_radius = mine_radius
        self.tasks: tuple = []
        self.seen_tracker = seen_by_drone.SightTracker(self.field_size)
        self.field: "nodeGen.Field" = None
        self.id = id
        self.start_node = None
        self.end_nodes = []
        self.waypoints = []
        # TODO: add reference to mine and path data classes

    @property
    def is_connected(self) -> bool:
        """Checks if a drone has been connected to.

        Returns
        -------
        bool
            Whether this Drone object has connected to a drone.
        """
        return self._vehicle is not None

    @property
    def sim_mode(self) -> SimMode | None:
        """Get the simulation mode of this drone.

        Returns
        -------
        SimMode | None
            The simulation mode of this drone, or None if one has not been specified.
        """
        return self._sim_mode

    @property
    def vehicle(self) -> dronekit.Vehicle:
        """Get the DroneKit Vehicle object owned by this Drone object.

        Returns
        -------
        dronekit.Vehicle
            The Vehicle object owned by this Drone object.

        Raises
        ------
        AttributeError
            If a connection hasn't been made yet.
        """
        vehicle: dronekit.Vehicle | None = self._vehicle
        if vehicle is None:
            raise RuntimeError("we haven't connected to the drone yet")
        return vehicle

    async def connect_drone(self) -> None:
        """Connect to a drone. This operation is idempotent.

        Raises
        ------
        RuntimeError
            If no connection address has been set.
        """
        if self.is_connected:
            return

        if len(self.address) == 0:
            raise RuntimeError("no connection address specified")

        logging.info("Waiting for drone to connect...")
        self._vehicle = (
            dronekit.connect(self.address, wait_ready=True, timeout=90)
            if self.baud is None
            else dronekit.connect(self.address, wait_ready=True, baud=self.baud, timeout=90)
        )
        logging.info("Drone discovered!")

        if self._sim_mode is not SimMode.REAL:
            return

        message_1: str = "Waiting for user input to continue... "
        message_2: str = "(press enter when ready) "
        input(f"\x1b[38;2;255;255;0m{message_1}" f"\x1b[3m{message_2}" "\x1b[0m")

    def remove_arming_check(self) -> None:
        """
        For use with airsim.
        """
        self.vehicle.parameters["ARMING_CHECK"] = 0

    async def arm(self) -> None:
        """
        Arm the drone.
        """

        logging.info("Waiting for vehicle to intialize...")
        while not self.vehicle.is_armable:
            # Vehicle is not ready to accept code
            await asyncio.sleep(0.5)

        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle is properly armed
        logging.info("Waiting for arming...")
        while not self.vehicle.armed or self.vehicle.mode.name != "GUIDED":
            await asyncio.sleep(0.5)

    async def takeoff(self, takeoff_alt: float) -> None:
        """
        Takeoff vertically to the passed altitude.

        Parameters
        ----------
        takeoff_alt: float
            Altitude to reach in meters
        """
        logging.info("Using takeoff altitude of %f m", takeoff_alt)
        self.vehicle.simple_takeoff(
            takeoff_alt + 1.5
        )  # Add 5ft for margin of error (Alt is measured in meters by drone kit tho?)

        # Verify vehicle reaches target altitude
        while self.vehicle.location.global_relative_frame.alt < takeoff_alt:
            await asyncio.sleep(0.5)  # ALTITUDE IS IN METERS
        logging.info("Reached target altitude (%f m).", takeoff_alt)

    # See the recall function at the bottom, we might remove this function as its specific to SAUS
    async def return_to_launch(self) -> None:
        """
        Method to move vehicle above home location, then descend vertically.
        """
        home_loc = dronekit.LocationGlobalRelative(
            self.vehicle.home_location.lat, self.vehicle.home_location.lon, 23
        )  # Min alt should be in constants file
        self.vehicle.simple_goto(home_loc)
        logging.info("Moving to home lat/lon...")
        while (
            calculate_distance(
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.alt,
                home_loc.lat,
                home_loc.lon,
                home_loc.alt,
            )
            > 1
        ):  # Get within 1 meter above home location
            await asyncio.sleep(0.5)
        self.vehicle.mode = dronekit.VehicleMode("RTL")
        logging.info("Descending...")
        while (
            self.vehicle.location.global_relative_frame.alt > 0.2  # ALTITUDE IS IN METERS
        ):  # Ensure drone gets within 8in above ground
            await asyncio.sleep(0.5)
        logging.info("Reached ground.")

    async def close(self) -> None:
        """Close the owned DroneKit Vehicle object."""
        self.vehicle.close()

    def use_settings(self, sim_mode: SimMode) -> None:
        """Set the simulation mode of this drone and update the connection settings
        according to the simulation mode.

        Parameters
        ----------
        sim_mode : SimMode
            The simulation mode.

        Raises
        ------
        ValueError
            If `sim_mode` is not a valid SimMode.
        """
        self._sim_mode = sim_mode
        port = 5762

        match sim_mode:
            case SimMode.REAL:
                self.address = "/dev/ttyUSB0"
                self.baud = 57600
            case SimMode.SIM:
                logging.info(f"Using SIM mode settings with port {port}")
                self.address = "tcp:127.0.0.1:" + str(port)
                self.baud = None
            case SimMode.AIRSIM:
                
                port += (
                    self.id - 1
                ) * 10  # IDs are assigned sequentially starting at 5762 and increasing by 10 for each drone.
                logging.info(f"Using AIRSIM mode settings with port {port}")
                self.address = "tcp:127.0.0.1:" + str(port)
                self.baud = None
            case _:
                raise ValueError("invalid sim mode")

    def updateWaypoints(self, waypoints):
        for waypoint in waypoints:
            self.waypoints.append(waypoint)

    def resetWaypoints(self, waypoints):
        self.waypoints = waypoints

    def getWaypoints(self):
        return self.waypoints

    def getWaypointChecksum(self):
        return Waypoint.getChecksum(self.waypoints)

    def checkForCollision(self, other_waypoints):
        collisions = Waypoint.check_for_collision(self.waypoints, other_waypoints)
        for collision in collisions:
            logging.info(
                f"Collision detected between waypoints {collision[0].waypoint_id} and {collision[1].waypoint_id}"
            )
            collision.drone1_waypoints[0].has_to_wait = True
            if (
                collision.drone2_waypoints[1]
                not in collision.drone1_waypoints[0].waypoints_to_reach
            ):
                if self.id < collision.drone2_waypoints[1].drone_id:
                    collision.drone1_waypoints[0].waypoints_to_reach.append(
                        collision.drone2_waypoints[1]
                    )

    async def gotoWaypoint(self):

        curWaypoint = self.waypoints[0]
        if curWaypoint.has_to_wait:
            logging.info(
                f"Waiting for waypoints {curWaypoint.waypoints_to_reach} to be reached before proceeding to next waypoint..."
            )
            while True:
                await asyncio.sleep(0.1)

                curWaypoint.check_wait()
        await move_to(
            self.vehicle, curWaypoint.latitude, curWaypoint.longitude, curWaypoint.altitude
        )

        curWaypoint.has_visited = True
        logging.info(f"Reached waypoint {curWaypoint.waypoint_id}")

        return self.waypoints.pop(0)

    def setFieldSize(self, xMin, xMax, yMin, yMax):
        self.field = nodeGen.Field(xMin, xMax, yMin, yMax)

    # Smart landing sequence, Should be usable in final product!!
    async def recall(self):
        if self.field_size[0] - self.x < self.field_size[1] - self.y:
            self.convert_goto(
                self, [self.field_size[0] * round(self.x / self.field_size[0]), self.y], 23
            )
        else:
            self.convert_goto(
                self, [self.x, self.field_size[1] * round(self.y / self.field_size[1])], 23
            )
