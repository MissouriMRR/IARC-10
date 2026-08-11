"""
Contains the Interdrone class, which handles interdrone communication messages
and allows for the cancellation and starting of states based on message data.
"""

# Outside Imports
import asyncio
from asyncio import Task
from collections.abc import Awaitable, Callable
import logging
from typing import TYPE_CHECKING
from typing import Any
from flight.waypoint import Waypoint
import flight.flight_log as flight_log
import queue
import sys
import threading
import time

# IARC Imports
from interdrone_communication.networking_interface import NetworkingInterface
from state_machine.flight_settings import FlightSettings

# state_machine.drone patches the collections aliases dronekit needs on
# import, so it must come before dronekit.
from state_machine.drone import Drone
import dronekit
from interdrone_communication.message_types import Message, MessageType
from state_machine.drone_state import DroneState
from enum import Enum

if TYPE_CHECKING:
    # If this import is left outside of the TYPE_CHECKING check,
    # it causes a circular import.
    from state_machine.states.state import State


# Console input is read by one daemon thread per process rather than by
# run_in_executor(input): a blocking input() call cannot be abandoned, so a state
# that also wants to react to a command from the app would be stuck in the
# executor until somebody typed a line. The thread parks on stdin forever and the
# queue below is polled, which makes "typed a command" and "the app sent one"
# two conditions that can be waited on together.
_stdin_lines: "queue.Queue[str]" = queue.Queue()
_stdin_thread: threading.Thread | None = None
_stdin_thread_lock = threading.Lock()


def _pump_stdin() -> None:
    """Feed every line typed on the console into _stdin_lines until EOF."""
    while True:
        try:
            line = sys.stdin.readline()
        except (ValueError, OSError):
            # stdin closed underneath us (common when run as a service).
            return
        if not line:
            return
        _stdin_lines.put(line.strip())


def _start_stdin_reader() -> None:
    global _stdin_thread
    with _stdin_thread_lock:
        if _stdin_thread is None:
            _stdin_thread = threading.Thread(target=_pump_stdin, daemon=True)
            _stdin_thread.start()


async def get_input_or(
    prompt: str,
    condition: Callable[[], bool],
    poll_s: float = 0.05,
) -> str | None:
    """
    Wait for a line typed on the console *or* for `condition()` to become true.

    Returns the typed line, or None if the condition fired first. This is what
    lets a prompted run be started either from the console or from the app: the
    app's command only ever lands in `Interdrone.cmd_msg`, so anything that waits
    on the console alone silently ignores it.
    """
    _start_stdin_reader()
    print(prompt, end="", flush=True)
    while True:
        if condition():
            # Newline so the abandoned prompt doesn't run into later output.
            print()
            return None
        try:
            return _stdin_lines.get_nowait()
        except queue.Empty:
            await asyncio.sleep(poll_s)


async def get_input(prompt: str) -> str:
    """Wait for a line typed on the console."""
    return await get_input_or(prompt, lambda: False) or ""


# How long an interrupt waits for the state it asked for to actually start
# before deciding the state machine isn't going to get there and commanding the
# vehicle itself. Longer than StateMachine.RESTART_WAIT_S would make the
# fallback unreachable; shorter leaves a healthy restart no room.
RESTART_CONFIRM_S: float = 6.0


def _log_restart_failure(task: Task) -> None:
    """Surface an exception from a fire-and-forget state machine restart."""
    if task.cancelled():
        return
    exception = task.exception()
    if exception is not None:
        logging.critical("State machine restart failed", exc_info=exception)


def _log_task_failure(task: Task) -> None:
    """
    Surface an exception from any fire-and-forget task.

    A task nobody awaits swallows its exception until the interpreter exits, which
    on a long-running drone means never. Attach this to every `ensure_future` whose
    result is dropped.
    """
    if task.cancelled():
        return
    exception = task.exception()
    if exception is not None:
        logging.error("Background task %s failed", task.get_name(), exc_info=exception)


class CMD_MSG(Enum):
    NONE = 0
    ARM = 1
    TAKEOFF = 2
    DEMO = 3
    MISSION = 4
    DEMO_DONE = 5
    MISSION_DONE = 6
    LAND = 7
    EMERGENCY_LAND = 8
    DISARM = 9
    SURVEY_START = 10


class Interdrone:
    """
    Handles interdrone communication messages and allows
    for the cancellation and starting of states based on message data.

    Attributes
    ----------
    _current_task : Task | None
        The current task being executed.
    _current_state : State | None
        The current state being executed in the state machine.
    _restart_callback : Callable[[State | None], Awaitable[None]] | None
        The callback function to be called when the state machine needs to be restarted.
    """

    def __init__(
        self,
        flight_settings: FlightSettings,
        drone: Drone,
    ):
        from interdrone_communication.networking_thread import NetworkingThread

        self._current_task: Task | None = None
        self._current_state: "State | None" = None
        self._restart_callback: Callable[["State | None"], Awaitable[None]] | None = None
        self.flight_settings: FlightSettings = flight_settings
        self.drone: Drone = drone
        # Create drone_states to access state of other drones in the test
        drone_states: list[DroneState] = []
        for id in flight_settings.other_drones_in_mission:
            drone_states.append(
                DroneState(
                    drone_id=id,
                    drone_ip=next(d["IP"] for d in flight_settings.drone_info if d["id"] == id),
                )
            )

        self.drone_states: list[DroneState] = drone_states
        self.cmd_msg: CMD_MSG = CMD_MSG.NONE

        # Most of the interdrone loop's message handling has no await in it, so the
        # loop only yields when it stops draining. This bounds how long it can hold
        # the event loop before the sleep at the bottom of the loop runs.
        self.max_messages_per_tick: int = 25
        # Set while a gather_swarm_status() task is in flight. Two of them share one
        # SEND_DRONE_STATUS queue, so each would eat responses meant for the other and
        # report drones as unavailable that had in fact answered.
        self._swarm_status_in_progress: bool = False

        # Store messages that each function may need
        self.interdrone_messages = {
            MessageType.PING_ACK: queue.Queue(),
            MessageType.PING_NACK: queue.Queue(),
            MessageType.SEND_GPS_OFFSET_ACK: queue.Queue(),
            MessageType.SEND_DRONE_STATUS: queue.Queue(),
            # NOTE ADD OTHER MESSAGES HERE AS NEEDED
        }

        # Create interdrone resources
        # Create instance of NetworkingThread class and setup resources_readyVariable to pass in
        networking_thread_obj: NetworkingThread = NetworkingThread()
        resources_ready: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)

        # Start networking thread
        networking_thread = threading.Thread(
            target=networking_thread_obj.run_networking_thread,
            args=(resources_ready, self.flight_settings),
            daemon=True,
        )
        networking_thread.start()

        # Wait for networking to be ready
        self.networking: NetworkingInterface = (
            resources_ready.get()
        )  # Used to interface with networking thread

    def register_state_machine(self, callback: Callable[["State | None"], Awaitable[None]]) -> None:
        """
        Registers a state machine with the run() method. This allows for the
        state machine to be restarted from the Interdrone object with any state.

        Parameters
        ----------
        callback : Callable[[State | None], Awaitable[None]]
            The callback function to be called when the state machine needs to be restarted.
        """
        self._restart_callback = callback

    def update_task(self, task: Task) -> None:
        """
        Updates the current asyncio task being executed.
        This should be a Task of a state being executed in the state machine.

        Parameters
        ----------
        task : Task
            The task to update the current task to.
        """
        self._current_task = task

    def update_state(self, state: "State") -> None:
        """
        Updates the current state being executed in the state machine.

        Parameters
        ----------
        state : State
            The state to update the current state to.
        """
        self._current_state = state

    def get_drone_state_from_id(self, drone_id: int) -> DroneState | None:
        return next((s for s in self.drone_states if s.drone_id == drone_id), None)

    async def ping_drones(self, timeout_sec: float = 2.0) -> bool:
        """
        All drones run this function.
        Sends PING and waits (with a timeout) until every other drone has ACK/NACK.
        Returns True only if all responded with ACK; drones that don't answer within
        timeout_sec are treated as failures rather than blocking forever.
        """
        # If only drone in test, return true for ping_drones
        if self.flight_settings.other_drones_in_mission == []:
            return True

        # Drop any responses left over from a previous ping so they aren't
        # mistaken for answers to this one.
        for stale_msg_type in (MessageType.PING_ACK, MessageType.PING_NACK):
            stale_queue: queue.Queue[Message] = self.interdrone_messages[stale_msg_type]
            while True:
                try:
                    stale_queue.get_nowait()
                except queue.Empty:
                    break

        # Track responses by drone id: None=not received yet, True=ACK, False=NACK
        ping_by_id: dict[int, bool | None] = {state.drone_id: None for state in self.drone_states}

        ping_message: Message = Message.create(
            id=MessageType.PING,
            drones_to_send_data=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )
        self.send(ping_message)

        logging.debug("Pings have been sent. Current ping response state is %s", ping_by_id)
        # TODO may move this functionality down to the interdrone loop and just wait up here. TBD. Keep this ok solution for now
        deadline = time.time() + timeout_sec
        while time.time() < deadline and None in ping_by_id.values():
            updated = False

            try:
                ack: Message = self.interdrone_messages[MessageType.PING_ACK].get_nowait()
                if ack.sender_id in ping_by_id:
                    ping_by_id[ack.sender_id] = True
                    updated = True
            except queue.Empty:
                pass

            try:
                nack: Message = self.interdrone_messages[MessageType.PING_NACK].get_nowait()
                if nack.sender_id in ping_by_id:
                    ping_by_id[nack.sender_id] = False
                    updated = True
            except queue.Empty:
                pass

            if not updated:
                await asyncio.sleep(0.05)

        # Anything still unanswered at the deadline is a failure, not a wait
        unanswered: list[int] = [
            drone_id for drone_id, result in ping_by_id.items() if result is None
        ]
        if unanswered:
            logging.warning("No ping response from drones %s after %.1fs", unanswered, timeout_sec)
        for drone_id in unanswered:
            ping_by_id[drone_id] = False

        # Copy results back into DroneState objects
        for state in self.drone_states:
            state.ping_response = ping_by_id[state.drone_id]

        all_ack: bool = all(result is True for result in ping_by_id.values())

        logging.debug("Return %s from ping_drones. Ping status is: %s", all_ack, ping_by_id)
        return all_ack

    async def send_ARM(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Message ID = 520
        Sends an arm message to all other drones in mission.
        Only used by drone 1.
        """
        arm_message: Message = Message.create(
            id=MessageType.ARM,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(arm_message)

        return

    async def send_arm_ack(self) -> None:
        """
        Sends arm_ack message to drone 1.
        Not used by drone 1, only recieved.
        """
        arm_ack_message: Message = Message.create(
            id=MessageType.ARM_ACK,
            drones_to_send_data=(1,),  # Only need to send to drone 1
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )
        self.send(arm_ack_message)

        return

    async def send_arm_nack(self) -> None:
        """
        Sends arm_nack message to drone 1.
        Not used by drone 1, only recieved.
        """
        arm_nack_message: Message = Message.create(
            id=MessageType.ARM_NACK,
            drones_to_send_data=(1,),  # Only need to send to drone 1
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )
        self.send(arm_nack_message)

        return

    async def all_armed(self) -> bool:
        """
        Used by drone one.
        Loop through all droneState objects to see if they are armed.
        Return true they are, false otherwise.
        """
        # Treat empty list as "not ready"

        if not self.drone_states:
            return True

        return all(state.armed is True for state in self.drone_states) or self.drone.id != 1

    async def send_takeoff(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Send a takeoff message to the drones passed as a parameter
        """
        takeoff_message: Message = Message.create(
            id=MessageType.START_TAKEOFF,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(takeoff_message)

        return

    async def send_takeoff_ack(self) -> None:
        """
        Sends takeoff_ack message to drone 1
        Not used by drone 1, only recieved.
        """
        takeoff_ack_message: Message = Message.create(
            id=MessageType.START_TAKEOFF_ACK,
            drones_to_send_data=(1,),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(takeoff_ack_message)

        return

    async def all_takeoff(self) -> bool:
        """
        Used by drone one
        Loop through all droneState objects to see if they have taken off.
        """
        if not self.drone_states:
            return False

        return all(state.takeoff is True for state in self.drone_states) or self.drone.id != 1

    async def send_start_demo(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Send a start demo message to the drone id passed as a parameter
        """
        demo_message: Message = Message.create(
            id=MessageType.START_DEMO,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(demo_message)

        return

    async def send_demo_ack(self) -> None:
        """
        Sends demo_ack message.
        Not used by drone 1, only recieved.
        """
        demo_ack_message: Message = Message.create(
            id=MessageType.START_DEMO_ACK,
            drones_to_send_data=(1,),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(demo_ack_message)

        return

    async def all_demo_start(self) -> bool:
        """
        Used by drone one
        Loop through all droneState objects to see if they have started demo
        """
        if not self.drone_states:
            return True
        print(f"Checking if all drones have started demo. Drone states: {self.drone_states}")
        return all(state.demo_start is True for state in self.drone_states) or self.drone.id != 1

    async def send_start_mission(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Send a start mission message to the drone id passed as a parameter.
        """
        mission_message: Message = Message.create(
            id=MessageType.START_MISSION,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(mission_message)

        return

    async def send_mission_ack(self) -> None:
        """
        Sends mission_ack message.
        Not used by drone 1, only recieved.
        """
        mission_ack_message: Message = Message.create(
            id=MessageType.START_MISSION_ACK,
            drones_to_send_data=(1,),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(mission_ack_message)

        return

    async def all_mission_start(self) -> bool:
        """
        Used by drone one
        Loop through all droneState objects to see if they have started mission
        """
        if not self.drone_states:
            return False

        return all(state.mission_start is True for state in self.drone_states)

    async def send_new_waypoints(
        self,
        drones_to_send_data: tuple[int, ...],
        waypoints: list[Waypoint],
    ) -> None:
        """
        Message ID = 545
        Send new_waypoints message to all drones
        new_waypoints contains:
        the new waypoints added to the current drone
        a checksum of waypoints for the other drone
        When received, if checksum of other drone doesn't match, a reconfirm_waypoints message is sent to make sure waypoints are synced
        """
        # The receiver compares against its full accumulated copy of our list, so the
        # checksum has to cover everything we hold, not just the waypoints being added.
        checksum = Waypoint.getChecksum(self.drone.getWaypoints())
        for target_drone in drones_to_send_data:
            state = self.get_drone_state_from_id(target_drone)
            if state is not None:
                new_waypoints_message: Message = Message.create(
                    id=MessageType.NEW_WAYPOINTS,
                    drones_to_send_data=(target_drone,),
                    sender_id=self.flight_settings.current_drone_ID,
                    data={
                        "newWaypoints": waypoints,
                        "sender_drone_waypoints_checksum": checksum,
                    },
                )
                self.send(new_waypoints_message)
                state.waypoint_up_to_date = False

        return

    async def reached_waypoint(
        self, drones_to_send_data: tuple[int, ...], waypoint: Waypoint
    ) -> None:
        """
        Message ID = 550
        Send reached_waypoint message to all drones
        """
        reached_waypoint_message: Message = Message.create(
            id=MessageType.REACHED_WAYPOINT,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "reached_waypoint_id": waypoint.waypoint_id,
            },
        )

        self.send(reached_waypoint_message)

        for drone_id in drones_to_send_data:
            state = self.get_drone_state_from_id(drone_id)
            if state is not None:
                state.waypoint_up_to_date = False

        return

    async def report_position(
        self, drones_to_send_data: tuple[int, ...], lat: float, lon: float, alt: float
    ) -> None:
        """
        Message ID = 552
        Broadcast where this drone actually is to every other drone.

        Every other position the swarm holds about a peer is inferred from that
        peer's waypoint messages, which means avoidance can only ever confirm the
        plan it already believes. This is the one input that can contradict it —
        the drone that has drifted off its circle or been taken over by a
        failsafe is precisely the drone whose waypoint list has stopped
        describing reality.

        Not acked, and not resent on failure: it is a continuous stream, a
        dropped report is superseded within a few hundred milliseconds, and
        receivers already treat a peer whose reports have dried up as unknown
        rather than as safely predicted.
        """
        position_message: Message = Message.create(
            id=MessageType.POSITION_REPORT,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={"lat": lat, "lon": lon, "alt": alt},
        )
        self.send(position_message)
        return

    async def send_survey_start(self) -> None:
        """
        Message ID = 565
        Set the CMD_MSG in interdrone to SURVEY_START
        Send Survey_Start message to all other drones
        """
        survey_message: Message = Message.create(
            id=MessageType.SURVEY_START,
            drones_to_send_data=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_message)

        return

    async def send_survey_ack(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Message ID = 566
        Sends survey_start_ack message to the drone that sent the original message
        drones_to_send_data should just be one drone
        """
        survey_ack_message: Message = Message.create(
            id=MessageType.SURVEY_START_ACK,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_ack_message)

        return

    async def send_survey_end(self) -> None:
        """
        Message ID = 570
        Sends survey_end message to all other drones
        """
        survey_end_message: Message = Message.create(
            id=MessageType.SURVEY_END,
            drones_to_send_data=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_end_message)

        return

    async def send_survey_end_ack(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Message ID = 571
        Sends survey_end_ack message to the drone that sent the original message
        drones_to_send_data should just be one drone
        """
        survey_end_ack_message: Message = Message.create(
            id=MessageType.SURVEY_END_ACK,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_end_ack_message)

        return

    async def share_photos(self, photos: list[dict[str, Any]]) -> None:
        """
        Message ID = 575
        Sends list of photos to all other drones
        """
        photos_message: Message = Message.create(
            id=MessageType.SHARE_PHOTOS,
            drones_to_send_data=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "photos": photos,
            },
        )

        self.send(photos_message)

        return

    async def send_checksum(self, checksum: int):
        """
        Message ID = 580
        Radius representing the state of the shared mat and all mines.
        Sends to all other drones
        """
        # TODO: Calculate checksum elsewhere
        checksum_message: Message = Message.create(
            id=MessageType.FIELD_CHECKSUM,
            drones_to_send_data=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "checksum": checksum,
            },
        )

        self.send(checksum_message)

        return

    async def send_mission_end(self) -> None:
        """
        Message ID = 585
        Sends mission_end message to all other drones
        """
        mission_end_message: Message = Message.create(
            id=MessageType.MISSION_END,
            drones_to_send_data=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(mission_end_message)

        return

    async def send_mission_end_ack(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Message ID = 586
        Sends mission_end_ack message to the drone that sent the original message
        drones_to_send_data should just be one drone
        """
        mission_end_ack_message: Message = Message.create(
            id=MessageType.MISSION_END_ACK,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(mission_end_ack_message)

        return

    async def send_ground_truth_ack(self) -> None:
        """
        Message ID = 441
        Sends SEND_GROUND_TRUTH_COORDS_ACK back to the app.
        If this drone is drone 1, it can address the app directly.
        Otherwise the ack must be routed through drone 1 first, since the app
        only holds a connection to drone 1.
        calculating_drone_id is carried explicitly in the payload because drone 1's
        relay (see SEND_GROUND_TRUTH_COORDS_ACK case below) rewrites sender_id to its own id.
        """
        ground_truth_ack_message: Message = Message.create(
            id=MessageType.SEND_GROUND_TRUTH_COORDS_ACK,
            drones_to_send_data=(0,) if self.flight_settings.current_drone_ID == 1 else (1,),
            sender_id=self.flight_settings.current_drone_ID,
            data={"calculating_drone_id": self.flight_settings.current_drone_ID},
        )

        self.send(ground_truth_ack_message)

        return

    async def distribute_gps_offset(self) -> None:
        """
        Message IDs = 444, 445, 443
        Used only by the drone commanded (via COMMAND_OFFSET_DISTRIBUTION) to distribute
        its calculated GPS offset.
        Sends SEND_GPS_OFFSET to every other drone in the mission, waits (with a timeout)
        for SEND_GPS_OFFSET_ACK from each, then reports the results back to the app via
        COMMAND_OFFSET_DISTRIBUTION_ACK (routed through drone 1 if this isn't drone 1).
        Run as a background task (see COMMAND_OFFSET_DISTRIBUTION case) since it can block
        for up to gps_offset_ack_timeout_sec waiting on acks.
        """
        gps_offset_ack_timeout_sec = 5.0

        other_drone_ids: tuple[int, ...] = tuple(self.flight_settings.other_drones_in_mission)
        ack_by_id: dict[int, bool] = {drone_id: False for drone_id in other_drone_ids}

        # Send GPS offset to all other drones
        offset_message: Message = Message.create(
            id=MessageType.SEND_GPS_OFFSET,
            drones_to_send_data=other_drone_ids,
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "lat_offset": self.flight_settings.gps_lat_offset,
                "lon_offset": self.flight_settings.gps_lon_offset,
            },
        )
        self.send(offset_message)

        # Wait up to 5s to get acks back from all other drones that they've updated their GPS offset values
        ack_queue: queue.Queue[Message] = self.interdrone_messages.setdefault(
            MessageType.SEND_GPS_OFFSET_ACK, queue.Queue()
        )
        deadline = time.time() + gps_offset_ack_timeout_sec
        while time.time() < deadline and not all(ack_by_id.values()):
            try:
                ack: Message = ack_queue.get_nowait()
                if ack.sender_id in ack_by_id:
                    ack_by_id[ack.sender_id] = True
            except queue.Empty:
                await asyncio.sleep(0.05)

        drone_ack_list: list[dict[str, Any]] = [
            {"drone_id": drone_id, "acked": acked} for drone_id, acked in ack_by_id.items()
        ]

        # Send COMMAND_OFFSET_DISTRIBUTION_ACK back to app with list of drones who set their offset
        distribution_ack_message: Message = Message.create(
            id=MessageType.COMMAND_OFFSET_DISTRIBUTION_ACK,
            drones_to_send_data=(0,) if self.flight_settings.current_drone_ID == 1 else (1,),
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "num_drones": len(other_drone_ids),
                "drone_ack_list": drone_ack_list,
            },
        )
        self.send(distribution_ack_message)

        return

    async def send_drone_status(self, drones_to_send_data: tuple[int, ...]) -> None:
        """
        Message ID = 517
        Sends this drone's current cmd message back to the drone that asked for it
        (drone 1). drone_id isn't included since it's carried in sender_id, and
        availability is inferred by drone 1 from whether this response arrives at all.
        drones_to_send_data should just be one drone.
        """
        drone_status_message: Message = Message.create(
            id=MessageType.SEND_DRONE_STATUS,
            drones_to_send_data=drones_to_send_data,
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "drone_cmd_msg": self.cmd_msg.value,
            },
        )

        self.send(drone_status_message)

        return

    async def gather_swarm_status(self) -> None:
        """
        Message IDs = 516, 517, 426
        Used only by drone 1 in response to REQUEST_SWARM_STATUS from the app.
        Sends REQUEST_DRONE_STATUS to every other drone in the mission, waits (with a
        timeout) for SEND_DRONE_STATUS from each, then reports the whole swarm's status
        back to the app via SEND_SWARM_STATUS. Drones that don't answer in time are
        reported with drone_available = False.
        Run as a background task (see REQUEST_SWARM_STATUS case) since it can block for
        up to drone_status_timeout_sec waiting on responses.

        The app has no other way to learn this failed: it shows a spinner from the moment
        it sends the request until a SEND_SWARM_STATUS arrives. So every path out of here
        sends one, including the error path -- a status saying every drone is unavailable
        is far more useful than a spinner that never stops.
        """
        if self._swarm_status_in_progress:
            logging.warning("Swarm status already being gathered; ignoring duplicate request")
            return

        self._swarm_status_in_progress = True
        try:
            await self._gather_swarm_status()
        except Exception:  # pylint: disable=broad-except
            # Nothing awaits this task, so without this the app just waits forever and
            # the only trace is an "exception was never retrieved" at interpreter exit.
            logging.exception("Failed to gather swarm status; reporting the swarm as unavailable")
            self._send_swarm_status(
                [
                    {
                        "drone_id": drone_id,
                        "drone_available": False,
                        "drone_cmd_msg": CMD_MSG.NONE.value,
                    }
                    for drone_id in self.flight_settings.drones_in_mission
                ]
            )
        finally:
            self._swarm_status_in_progress = False

    async def _gather_swarm_status(self) -> None:
        """Body of gather_swarm_status(); see there for what this does and why."""
        drone_status_timeout_sec = 2.0

        other_drone_ids: tuple[int, ...] = tuple(self.flight_settings.other_drones_in_mission)
        # None = no response yet, otherwise the drone's reported CMD_MSG value
        cmd_msg_by_id: dict[int, int | None] = {drone_id: None for drone_id in other_drone_ids}

        status_queue: queue.Queue[Message] = self.interdrone_messages.setdefault(
            MessageType.SEND_DRONE_STATUS, queue.Queue()
        )

        # Drop any responses left over from a previous request so they aren't
        # mistaken for answers to this one.
        while True:
            try:
                status_queue.get_nowait()
            except queue.Empty:
                break

        if other_drone_ids:
            request_message: Message = Message.create(
                id=MessageType.REQUEST_DRONE_STATUS,
                drones_to_send_data=other_drone_ids,
                sender_id=self.flight_settings.current_drone_ID,
                data={},
            )
            self.send(request_message)

            deadline = time.time() + drone_status_timeout_sec
            while time.time() < deadline and None in cmd_msg_by_id.values():
                try:
                    response: Message = status_queue.get_nowait()
                    if response.sender_id in cmd_msg_by_id:
                        # A response that failed schema validation arrives with data
                        # emptied. Treating that as "answered, cmd message unknown" keeps
                        # the drone marked available (it clearly is -- it replied) instead
                        # of raising KeyError and killing this whole poll.
                        cmd_msg_by_id[response.sender_id] = response.data.get(
                            "drone_cmd_msg", CMD_MSG.NONE.value
                        )
                except queue.Empty:
                    await asyncio.sleep(0.05)

        # Drone 1 reports itself first, then every other drone in the mission.
        drone_status: list[dict[str, Any]] = [
            {
                "drone_id": self.flight_settings.current_drone_ID,
                "drone_available": True,
                "drone_cmd_msg": self.cmd_msg.value,
            }
        ]
        for drone_id, cmd_msg_value in cmd_msg_by_id.items():
            drone_status.append(
                {
                    "drone_id": drone_id,
                    "drone_available": cmd_msg_value is not None,
                    # Unreachable drones have no known cmd message, so report NONE
                    "drone_cmd_msg": (
                        cmd_msg_value if cmd_msg_value is not None else CMD_MSG.NONE.value
                    ),
                }
            )

        logging.info("Swarm status: %s", drone_status)
        self._send_swarm_status(drone_status)

        return

    def _send_swarm_status(self, drone_status: list[dict[str, Any]]) -> None:
        """
        Message ID = 426
        Report the swarm's status to the app. Split out from gather_swarm_status() so
        its error path can answer the app too rather than leaving it waiting.
        """
        swarm_status_message: Message = Message.create(
            id=MessageType.SEND_SWARM_STATUS,
            drones_to_send_data=(0,),  # 0 addresses the app
            sender_id=self.flight_settings.current_drone_ID,
            data={
                "num_drones": len(drone_status),
                "drone_status": drone_status,
            },
        )
        self.send(swarm_status_message)

    async def cancel_state(self) -> None:
        """
        Cancels the current state being executed.
        If the current state is in an atomic context,
        waits for the lock to release before cancelling.

        Raises
        ------
        RuntimeError
            If there is no running state/task to cancel.
        """
        if not self._current_state or not self._current_task:
            raise RuntimeError("No running state/task to cancel")

        # Check if the state is in an atomic context before cancelling
        # If so, wait for the lock to release then cancel task
        task_to_cancel = self._current_task
        async with self._current_state.atomic:
            task_to_cancel.cancel()
            self._current_state = None
            self._current_task = None

        # We need to wait for the task to actually be cancelled before
        # doing anything useful, which we can do by awaiting the task after
        # it has already been scheduled for cancellation
        # This will always raise the CancelledError
        try:
            await task_to_cancel
        except asyncio.CancelledError:
            pass

    async def restart_state_machine(self, state: "State | None") -> None:
        """
        Restarts the state machine with the given state, or if none is
        provided, the last state that was running

        Parameters
        ----------
        state : State | None
            The state to restart the state machine with.

        Raises
        ------
        RuntimeError
            If there is a running task or the state machine has not been registered.
        """
        if self._current_state or self._current_task:
            raise RuntimeError("Cannot restart state while a task is running")

        if not self._restart_callback:
            raise RuntimeError("Cannot restart state machine without a registered callback")

        # Start the restart callback as a separate task but do not wait for it
        restart_task: Task = asyncio.ensure_future(self._restart_callback(state))
        # Nothing awaits that task, so without this a failure to restart is
        # visible only as the state machine mysteriously not running.
        restart_task.add_done_callback(_log_restart_failure)

    async def interrupt_into(
        self,
        state_factory: Callable[[], "State"],
        label: str,
        fallback_mode: str,
    ) -> None:
        """
        Cancel whatever state is running and restart the state machine into a new one.

        This is how a command that has to take effect *now* — landing, above all
        — reaches a state machine that is busy flying something else. Cancelling
        goes through `cancel_state`, so a state inside its `atomic` section
        finishes that section first.

        Must not be awaited from the interdrone loop: `cancel_state` blocks on
        the running state's atomic lock, and the loop it would block is the one
        feeding position reports to the rest of the swarm. Launch it with
        `asyncio.ensure_future`.

        Parameters
        ----------
        state_factory : Callable[[], State]
            Builds the state to restart into. Called only once the running state
            has actually been cancelled.
        label : str
            What this interrupt is, for the logs.
        fallback_mode : str
            Flight mode to command directly if the state machine can't be
            interrupted. The command still has to reach the vehicle even when
            there is no state machine left to route it through.
        """
        try:
            if not self._current_state or not self._current_task:
                # No state machine to interrupt: the command arrived before the
                # machine started, or after it finished. Neither is a reason to
                # drop a land command on the floor.
                logging.critical(
                    "%s commanded with no state running -- commanding %s directly",
                    label,
                    fallback_mode,
                )
                self.command_vehicle_mode(fallback_mode)
                return

            logging.critical("%s commanded -- cancelling %s", label, self._current_state.name)
            await self.cancel_state()

            new_state: "State" = state_factory()
            await self.restart_state_machine(new_state)

            # restart_state_machine only schedules the restart, so confirm the
            # new state actually started rather than assuming it. Everything
            # between here and the vehicle is asynchronous, and a land command
            # that quietly failed to take is the worst outcome available.
            deadline: float = time.monotonic() + RESTART_CONFIRM_S
            while self._current_state is not new_state and time.monotonic() < deadline:
                await asyncio.sleep(0.05)

            if self._current_state is not new_state:
                logging.critical(
                    "%s state did not start within %.1fs -- commanding %s directly",
                    label,
                    RESTART_CONFIRM_S,
                    fallback_mode,
                )
                self.command_vehicle_mode(fallback_mode)
        except Exception:  # pylint: disable=broad-except
            # An exception escaping here would be raised inside the interdrone
            # loop's task and take the whole comms layer down with it, on a
            # drone that is both airborne and no longer being flown.
            logging.exception(
                "Failed to interrupt into %s -- falling back to %s", label, fallback_mode
            )
            self.command_vehicle_mode(fallback_mode)

    def command_vehicle_mode(self, mode_name: str) -> None:
        """
        Put the vehicle into a flight mode directly, bypassing the state machine.

        Last resort for the interrupt paths above. Does nothing if the vehicle
        isn't connected or is already disarmed — there is nothing to command,
        and forcing a mode onto a parked vehicle only confuses the next start.
        """
        try:
            vehicle = self.drone.vehicle
        except RuntimeError:
            logging.error("Cannot command %s: no vehicle connection", mode_name)
            return

        if not vehicle.armed:
            logging.info("Not commanding %s: vehicle is already disarmed", mode_name)
            return

        if vehicle.mode.name != mode_name:
            logging.critical("Commanding %s directly (was %s)", mode_name, vehicle.mode.name)
            vehicle.mode = dronekit.VehicleMode(mode_name)

    def start_emergency_land(self) -> Task:
        """
        Take the drone out of whatever it is doing and land it where it stands.

        Returns the task doing the interrupting so callers (and tests) can wait
        on it. The interdrone loop must not: see `interrupt_into`.
        """
        # Imported here rather than at module scope: the state implementations
        # import this module, so a top-level import is circular.
        from state_machine.states.impl import EmergencyLand

        return asyncio.ensure_future(
            self.interrupt_into(
                lambda: EmergencyLand(self.drone, self.flight_settings, self),
                label="EMERGENCY_LAND",
                fallback_mode="LAND",
            )
        )

    def start_land(self) -> Task:
        """
        Take the drone out of whatever it is doing and return it to launch.

        The orderly counterpart to `start_emergency_land`: the Land state flies
        home before descending, so it needs the drone to still be flyable.
        """
        from state_machine.states.impl import Land

        return asyncio.ensure_future(
            self.interrupt_into(
                lambda: Land(self.drone, self.flight_settings, self),
                label="LAND",
                fallback_mode="RTL",
            )
        )

    def read(self) -> Message | None:
        """
        Try to read a message.
        """
        return self.networking.try_get_server_message(timeout=0.02)

    def send(self, message: Message) -> None:
        """
        Send a message.
        """
        self.networking.queue_client_message(message)

    def get_cmd_msg(self) -> CMD_MSG:
        # NOTE May want to set to None after returning it
        return self.cmd_msg

    def set_cmd_msg(self, cmd_msg: CMD_MSG) -> None:
        self.cmd_msg = cmd_msg

    # Start client code and call the client_loop()
    async def start_interdrone(self):
        await self.interdrone_loop()

    # Check for new messages to send and create tasks to send them
    async def interdrone_loop(self) -> None:
        startTime = time.time()
        try:
            while True:
                # Check for server messages, draining what is already queued this tick.
                # Handling only one per iteration caps throughput at ~10 msg/sec (one per
                # sleep below), which lets bursts of traffic build a backlog. Draining
                # without a cap is the opposite failure: under a sustained stream -- three
                # peers broadcasting POSITION_REPORT during POIF is enough -- this loop
                # never reaches the sleep, and most cases below contain no await at all, so
                # nothing else on the event loop gets to run. That includes the background
                # tasks started from inside this very loop, so a REQUEST_SWARM_STATUS could
                # be handled here and gather_swarm_status() still never execute, leaving the
                # app waiting on a response that was never going to be sent.
                messages_this_tick: int = 0
                while messages_this_tick < self.max_messages_per_tick and (
                    message := self.networking.try_get_server_message(timeout=0.02)
                ) is not None:
                    messages_this_tick += 1
                    # Adds the new message to its respective message queue
                    self.interdrone_messages.setdefault(message.id, queue.Queue()).put(
                        message
                    )  # TODO GET RID OF THIS SOON
                    match message.id:
                        # NOTE could move this functionality to a receive function. That would be a cleanup item though
                        case MessageType.ARM:
                            # Drone is able to arm if vehicle is armable and ping response to other drones is true
                            # if self.drone.vehicle.is_armable and all(
                            #     state.ping_response is True
                            #     for state in self.drone_states
                            # ):
                            if all(
                                state.ping_response is True for state in self.drone_states
                            ):  # Extra if used for local testing
                                # Set cmd_msg to arm (signals to state machine to arm the drone)
                                self.cmd_msg = CMD_MSG.ARM
                                if self.flight_settings.current_drone_ID == 1:
                                    # Drone 1 distributes message to other drones in the mission
                                    await self.send_ARM(
                                        drones_to_send_data=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                # Other drones send ACK
                                else:
                                    await self.send_arm_ack()
                            # Send ARM_NACK if drone can't arm
                            else:
                                # Drone 1 is the drone that *collects* NACKs, so it has
                                # nobody to send one to. send_arm_nack() addresses drone 1
                                # unconditionally, which the client's send-to-self path
                                # loops straight back into this branch -- an unthrottled
                                # ARM/ARM_NACK storm. Log and drop instead.
                                if self.flight_settings.current_drone_ID == 1:
                                    logging.warning(
                                        "Drone 1 cannot arm: no ping response yet from %s. Ignoring ARM.",
                                        [
                                            s.drone_id
                                            for s in self.drone_states
                                            if s.ping_response is not True
                                        ],
                                    )
                                else:
                                    await self.send_arm_nack()

                        case MessageType.ARM_ACK:
                            # When drone 1 receives an ACK, set others drone arm state to true
                            state = next(
                                (s for s in self.drone_states if s.drone_id == message.sender_id),
                                None,
                            )

                            if state is not None:
                                state.armed = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.sender_id}. Something is ary!"
                                )
                        case MessageType.ARM_NACK:
                            # A NACK from ourselves can only have come from the client's
                            # send-to-self path; resending ARM to ourselves would spin.
                            if message.sender_id == self.flight_settings.current_drone_ID:
                                logging.warning(
                                    "Ignoring ARM_NACK from self (drone %s)", message.sender_id
                                )
                            else:
                                # Try and resend ARM to drone that sent NACK
                                print(
                                    f"Drone {message.sender_id} failed to arm. Resending message."
                                )
                                await self.send_ARM(drones_to_send_data=(message.sender_id,))
                        case MessageType.DISARM:
                            self.cmd_msg = CMD_MSG.DISARM
                            if self.flight_settings.current_drone_ID == 1:
                                # Drone 1 distributes message to other drones in the mission
                                disarm_message: Message = Message.create(
                                    id=MessageType.DISARM,
                                    drones_to_send_data=(message.sender_id,),
                                    sender_id=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(disarm_message)
                            self.drone.vehicle.armed = False
                        case MessageType.START_TAKEOFF:
                            # TODO MAKE SURE THIS IS THE RIGHT WAY TO CHECK FOR ARM SET (we could unset arm cmd msg. want to double check we dont)
                            # TODO MAKE SURE WE DON'T NEED A START_TAKEOFF_NACK
                            if self.cmd_msg == CMD_MSG.ARM:
                                self.cmd_msg = CMD_MSG.TAKEOFF
                                if self.flight_settings.current_drone_ID == 1:
                                    # Drone 1 distributes message to other drones in the mission
                                    await self.send_takeoff(
                                        drones_to_send_data=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                # Other drones send ACK
                                else:
                                    await self.send_takeoff_ack()
                        case MessageType.START_TAKEOFF_ACK:
                            state = self.get_drone_state_from_id(message.sender_id)
                            if state is not None:
                                state.takeoff = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.sender_id}! Something is ary!"
                                )
                        case MessageType.START_DEMO:
                            print("GOT START DEMO MESSAGE")
                            if self.cmd_msg == CMD_MSG.ARM:  # TODO VERIFY THIS IS CORRECT
                                self.cmd_msg = CMD_MSG.DEMO
                                if self.flight_settings.current_drone_ID == 1:
                                    await self.send_start_demo(
                                        drones_to_send_data=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                else:
                                    await self.send_demo_ack()
                        case MessageType.START_DEMO_ACK:
                            state = self.get_drone_state_from_id(message.sender_id)

                            if state is not None:
                                state.demo_start = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.sender_id}! Something is ary!"
                                )
                        case MessageType.DEMO_DONE:
                            self.cmd_msg = CMD_MSG.DEMO_DONE
                            if self.flight_settings.current_drone_ID == 1:
                                demo_done_message: Message = Message.create(
                                    id=MessageType.DEMO_DONE,
                                    drones_to_send_data=tuple(
                                        self.flight_settings.other_drones_in_mission,
                                    ),
                                    sender_id=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(demo_done_message)
                        case MessageType.START_MISSION:
                            print("start mission recieved")
                            if self.cmd_msg == CMD_MSG.ARM:
                                self.cmd_msg = CMD_MSG.MISSION
                                if self.flight_settings.current_drone_ID == 1:
                                    await self.send_start_mission(
                                        drones_to_send_data=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                else:
                                    await self.send_mission_ack()
                        case MessageType.START_MISSION_ACK:
                            state = self.get_drone_state_from_id(message.sender_id)

                            if state is not None:
                                state.mission_start = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.sender_id}! Something is ary!"
                                )
                        case MessageType.NEW_WAYPOINTS:
                            # Get state and set list_of_waypoints
                            state = self.get_drone_state_from_id(message.sender_id)
                            # TODO CHECK THE CHECKSUM TO DETERMINE WHETHER TO SEND RECONFIRM WAYPOINTS
                            if state is not None:
                                state.touch()
                                # Add received waypoints to list_of_waypoints
                                state.list_of_waypoints += message.data["newWaypoints"]

                                flight_log.event(
                                    "waypoints_recv",
                                    peer=message.sender_id,
                                    added=flight_log.waypoints_brief(message.data["newWaypoints"]),
                                    queued_after=len(state.list_of_waypoints),
                                    checksum=Waypoint.getChecksum(state.list_of_waypoints),
                                    sender_checksum=message.data["sender_drone_waypoints_checksum"],
                                )

                                # TODO IMPLEMENT CHECKSUM HERE
                                # Get checksum of self.drone.waypoint_checksum and compare to message.data[""]
                                checksum = Waypoint.getChecksum(state.list_of_waypoints)
                                sender_checksum = message.data["sender_drone_waypoints_checksum"]
                                # Only worth reading when they disagree; logged below if so.
                                logging.debug(
                                    "drone %d waypoints: %d held, checksum %d (sender says %d)",
                                    message.sender_id,
                                    len(state.list_of_waypoints),
                                    checksum,
                                    sender_checksum,
                                )
                                # Check if stored list of waypoints matches what the other drone has
                                # If so, send NEW_WAYPOINTS_ACK
                                if checksum == sender_checksum:
                                    waypoints_ack_message: Message = Message.create(
                                        id=MessageType.NEW_WAYPOINTS_ACK,
                                        drones_to_send_data=(message.sender_id,),
                                        sender_id=self.flight_settings.current_drone_ID,
                                        data={},
                                    )
                                    self.send(waypoints_ack_message)
                                # If checksums don't match, send reconfirm waypoints message
                                else:
                                    logging.warning(
                                        "waypoint list out of sync with drone %d "
                                        "(ours %d vs theirs %d), resyncing",
                                        message.sender_id,
                                        checksum,
                                        sender_checksum,
                                    )
                                    state.waypoint_up_to_date = False
                                    waypoints = self.drone.getWaypoints()
                                    reconfirm_waypoints_message: Message = Message.create(
                                        id=MessageType.RECONFIRM_WAYPOINTS,
                                        drones_to_send_data=(message.sender_id,),
                                        sender_id=self.flight_settings.current_drone_ID,
                                        data={
                                            "all_waypoints": waypoints,
                                            "need_response": True,
                                        },
                                    )
                                    self.send(reconfirm_waypoints_message)
                                self.drone.checkForCollision(state.list_of_waypoints)
                            # PLACEHOLDER -- not implemented yet (see the
                            # coordinating-4-drones plan's leader/follower
                            # design, flight/pathfinding/tests/droneWorkflowTest.py's
                            # simulate_leader_follower_pair): for an ASSISTANT
                            # receiving its paired GAMBLER's segment-B waypoints,
                            # this is where they'd need to actually reach flight
                            # logic -- e.g. handing state.list_of_waypoints to
                            # this drone's own goto/waypoint-queue state (see
                            # Drone.gotoWaypoint) so the assistant starts flying
                            # them, not just holding them in DroneState. Left
                            # deliberately unimplemented for now.

                        case MessageType.NEW_WAYPOINTS_ACK:
                            state = self.get_drone_state_from_id(message.sender_id)
                            if state is not None:
                                state.waypoint_up_to_date = True
                        case MessageType.REACHED_WAYPOINT:
                            state = self.get_drone_state_from_id(message.sender_id)

                            if state is not None:
                                state.touch()
                                reached_id = message.data["reached_waypoint_id"]
                                reached_waypoint: Waypoint | None = None
                                for waypoint in state.list_of_waypoints:
                                    if waypoint.waypoint_id == reached_id:
                                        reached_waypoint = waypoint
                                if reached_waypoint is not None:
                                    state.list_of_waypoints.remove(reached_waypoint)
                                    reached_waypoint.has_visited = True
                                    # This is the freshest position fix we have for
                                    # that drone; collision avoidance holds on it.
                                    state.last_reached_waypoint = reached_waypoint

                                # A report for a waypoint we were never told about
                                # leaves last_reached_waypoint stale, so avoidance
                                # keeps reasoning about where that drone used to be.
                                flight_log.event(
                                    "reached_recv",
                                    peer=message.sender_id,
                                    reached_id=reached_id,
                                    matched=reached_waypoint is not None,
                                    queued_after=len(state.list_of_waypoints),
                                    last_reached=flight_log.waypoint_brief(
                                        state.last_reached_waypoint
                                    ),
                                )

                                # print(f"State of waypoint list after: {state.list_of_waypoints} ")
                                reached_waypoint_ack_message: Message = Message.create(
                                    id=MessageType.REACHED_WAYPOINT_ACK,
                                    drones_to_send_data=(message.sender_id,),
                                    sender_id=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(reached_waypoint_ack_message)
                            # PLACEHOLDER -- not implemented yet: this only
                            # updates the SENDER's bookkeeping (state.
                            # list_of_waypoints/last_reached_waypoint, used
                            # for collision avoidance) -- it doesn't yet
                            # trigger anything on THIS drone's own flight
                            # logic in response (e.g. a GAMBLER reacting to
                            # its paired ASSISTANT finishing a leg). Left
                            # deliberately unimplemented for now.
                        case MessageType.POSITION_REPORT:
                            state = self.get_drone_state_from_id(message.sender_id)
                            if state is not None:
                                state.touch()
                                state.set_live_position(
                                    message.data["lat"],
                                    message.data["lon"],
                                    message.data["alt"],
                                )
                        case MessageType.REACHED_WAYPOINT_ACK:

                            state = self.get_drone_state_from_id(message.sender_id)
                            if state is not None:
                                state.waypoint_up_to_date = True
                        case MessageType.RECONFIRM_WAYPOINTS:
                            state = self.get_drone_state_from_id(message.sender_id)

                            if state is not None:
                                state.touch()
                                # Update list of waypoints with the correct ones
                                state.list_of_waypoints = message.data["all_waypoints"]
                                state.waypoint_up_to_date = True
                                # Message needs responses, send back message with current drones waypoints
                                if message.data["need_response"]:
                                    waypoints = self.drone.getWaypoints()
                                    reconfirm_waypoints_message_response: Message = Message.create(
                                        id=MessageType.RECONFIRM_WAYPOINTS,
                                        drones_to_send_data=(message.sender_id,),
                                        sender_id=self.flight_settings.current_drone_ID,
                                        data={
                                            "all_waypoints": waypoints,
                                            "need_response": False,
                                        },
                                    )
                                    self.send(reconfirm_waypoints_message_response)
                        case MessageType.SHARE_PHOTOS:
                            # PLACEHOLDER -- send-only today (see share_photos
                            # above); this receive-side case doesn't exist
                            # yet. Per the leader/follower design (see the
                            # coordinating-4-drones plan and
                            # simulate_leader_follower_pair's
                            # _leader_apply_follower_report), this is where an
                            # ASSISTANT's photo reports would need to reach
                            # its paired GAMBLER's own Pathfinder: for each
                            # photo in message.data["photos"], call
                            # accept_image_corner_coord for the image's corner
                            # coords (coverage), then add_discovered_mine for
                            # any mine coordinates reported in it. Left
                            # deliberately unimplemented for now.
                            pass
                        case MessageType.FIELD_CHECKSUM:
                            # PLACEHOLDER -- send_checksum (see above) has no
                            # caller anywhere yet either, so this case has
                            # nothing to receive today. Intended purpose (see
                            # the coordinating-4-drones plan): verify every
                            # drone agrees on mission_field_corners before
                            # relying on lat/lon as a shared coordinate frame
                            # -- reusing NEW_WAYPOINTS' own checksum +
                            # RECONFIRM_WAYPOINTS resync pattern above is the
                            # identified but not-yet-built approach. Left
                            # deliberately unimplemented for now.
                            pass
                        case MessageType.EMERGENCY_LAND:
                            # Duplicates are expected, not exceptional: drone 1
                            # fans the command out, the client loops
                            # sends-to-self back in, and EMERGENCY_LAND is in
                            # the resend-on-failure set. Acting on every copy
                            # would cancel the EmergencyLand state mid-descent
                            # and re-broadcast on every arrival.
                            if self.cmd_msg == CMD_MSG.EMERGENCY_LAND:
                                logging.debug(
                                    "Ignoring duplicate EMERGENCY_LAND from drone %s",
                                    message.sender_id,
                                )
                            else:
                                self.cmd_msg = CMD_MSG.EMERGENCY_LAND
                                if self.flight_settings.current_drone_ID == 1:
                                    emergency_land_message: Message = Message.create(
                                        id=MessageType.EMERGENCY_LAND,
                                        drones_to_send_data=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        ),
                                        sender_id=self.flight_settings.current_drone_ID,
                                        data={},
                                    )
                                    self.send(emergency_land_message)
                                self.start_emergency_land()
                        case MessageType.LAND:
                            if self.cmd_msg == CMD_MSG.EMERGENCY_LAND:
                                # An emergency land outranks an orderly one.
                                # Pulling a descending drone back up into a
                                # return-to-launch transit is the opposite of
                                # what was asked for.
                                logging.warning(
                                    "Ignoring LAND from drone %s: emergency land in progress",
                                    message.sender_id,
                                )
                            elif self.cmd_msg == CMD_MSG.LAND:
                                logging.debug(
                                    "Ignoring duplicate LAND from drone %s", message.sender_id
                                )
                            else:
                                self.cmd_msg = CMD_MSG.LAND
                                if self.flight_settings.current_drone_ID == 1:
                                    land_message: Message = Message.create(
                                        id=MessageType.LAND,
                                        drones_to_send_data=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        ),
                                        sender_id=self.flight_settings.current_drone_ID,
                                        data={},
                                    )
                                    self.send(land_message)
                                self.start_land()
                        case MessageType.SEND_APP_COORDS:
                            # TODO droneState stuff?
                            if self.flight_settings.current_drone_ID == 1:
                                send_app_coords_message: Message = Message.create(
                                    id=MessageType.SEND_APP_COORDS,
                                    drones_to_send_data=tuple(
                                        self.flight_settings.other_drones_in_mission,
                                    ),
                                    sender_id=self.flight_settings.current_drone_ID,
                                    data={
                                        "app_latitude": self.flight_settings.app_latitude,
                                        "app_longitude": self.flight_settings.app_longitude,
                                    },
                                )
                                self.send(send_app_coords_message)
                        case MessageType.SEND_APP_COORDS_ACK:
                            # TODO Harper implement this or something
                            pass
                        case MessageType.SEND_GROUND_TRUTH_COORDS:
                            # If this message isn't addressed to us, only drone 1 (the app's
                            # only connection) should be relaying it on to the real target.
                            if message.drones_to_send_data != (
                                self.flight_settings.current_drone_ID,
                            ):
                                if self.flight_settings.current_drone_ID == 1:
                                    self.send(message)
                            # Else: message is at the target
                            # Calculate offset
                            else:
                                # TODO: calculate this drone's actual GPS offset from
                                # message.data["lat"]/["lon"] (ground truth) vs its onboard GPS
                                # TODO Set ardupilot param as well
                                self.flight_settings.gps_lat_offset = 0.0
                                self.flight_settings.gps_lon_offset = 0.0
                                await self.send_ground_truth_ack()
                        case MessageType.SEND_GROUND_TRUTH_COORDS_ACK:
                            # Relay to the app if we're drone 1 and this ack arrived over the
                            # network from another drone (rather than being generated locally).
                            if (
                                self.flight_settings.current_drone_ID == 1
                                and message.sender_id != 1
                            ):
                                relay_message: Message = Message.create(
                                    id=MessageType.SEND_GROUND_TRUTH_COORDS_ACK,
                                    drones_to_send_data=(0,),
                                    sender_id=self.flight_settings.current_drone_ID,
                                    data={
                                        "calculating_drone_id": message.data["calculating_drone_id"]
                                    },
                                )
                                self.send(relay_message)
                        case MessageType.COMMAND_OFFSET_DISTRIBUTION:
                            # Same drone-1-relay pattern as SEND_GROUND_TRUTH_COORDS above.
                            # (sends message to target drone since app always sends to 1)
                            if message.drones_to_send_data != (
                                self.flight_settings.current_drone_ID,
                            ):
                                if self.flight_settings.current_drone_ID == 1:
                                    self.send(message)
                            else:
                                # Runs as a background task since it waits on SEND_GPS_OFFSET_ACK
                                # responses and shouldn't block the interdrone loop.
                                asyncio.ensure_future(self.distribute_gps_offset())
                        case MessageType.SEND_GPS_OFFSET:
                            # Update GPS offset values based on message values
                            self.flight_settings.gps_lat_offset = message.data["lat_offset"]
                            self.flight_settings.gps_lon_offset = message.data["lon_offset"]
                            gps_offset_ack_message: Message = Message.create(
                                id=MessageType.SEND_GPS_OFFSET_ACK,
                                drones_to_send_data=(message.sender_id,),
                                sender_id=self.flight_settings.current_drone_ID,
                                data={},
                            )
                            self.send(gps_offset_ack_message)
                        case MessageType.SEND_GPS_OFFSET_ACK:
                            # Consumed directly out of self.interdrone_messages by
                            # distribute_gps_offset(); nothing to do here.
                            pass
                        case MessageType.COMMAND_OFFSET_DISTRIBUTION_ACK:
                            # Relay to the app if we're drone 1 and this ack arrived over the
                            # network from the drone that ran the distribution.
                            if (
                                self.flight_settings.current_drone_ID == 1
                                and message.sender_id != 1
                            ):
                                relay_distribution_ack: Message = Message.create(
                                    id=MessageType.COMMAND_OFFSET_DISTRIBUTION_ACK,
                                    drones_to_send_data=(0,),
                                    sender_id=self.flight_settings.current_drone_ID,
                                    data={
                                        "num_drones": message.data["num_drones"],
                                        "drone_ack_list": message.data["drone_ack_list"],
                                    },
                                )
                                self.send(relay_distribution_ack)

                            # Check if drone 1 received and then distribute
                        case MessageType.REQUEST_SWARM_STATUS:
                            # Only drone 1 holds the app connection and polls the swarm.
                            if self.flight_settings.current_drone_ID == 1:
                                # Runs as a background task since it waits on
                                # SEND_DRONE_STATUS responses and shouldn't block this loop.
                                swarm_status_task: Task = asyncio.ensure_future(
                                    self.gather_swarm_status()
                                )
                                # Nothing awaits this, so an exception escaping it would
                                # otherwise be invisible -- the app would just spin.
                                swarm_status_task.add_done_callback(_log_task_failure)
                        case MessageType.REQUEST_DRONE_STATUS:
                            # Respond to whichever drone asked (drone 1) with our cmd message
                            await self.send_drone_status(drones_to_send_data=(message.sender_id,))
                        case MessageType.SEND_DRONE_STATUS:
                            # Consumed directly out of self.interdrone_messages by
                            # gather_swarm_status(); nothing to do here.
                            pass
                    # Catch different messages here and add them to interdrone message queue so other functions can use them
                    # msgNum += 1
                    # print(f"Server Data: {msgNum}")

                # Send heartbeat if queue is empty
                # if self.networking.is_client_in_empty():
                #     heartbeatMessage.data["payload"] = str(msgNum)
                #     self.networking.queue_client_message(heartbeatMessage)

                await asyncio.sleep(0.1)

        except KeyboardInterrupt:
            print(f"Program ran for {time.time() - startTime}")
            print("Shutting down...")
