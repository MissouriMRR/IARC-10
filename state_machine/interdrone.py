"""
Contains the Interdrone class, which handles interdrone communication messages
and allows for the cancellation and starting of states based on message data.
"""

# Outside Imports
import asyncio
from asyncio import Task
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING
from flight.waypoint import Waypoint
import queue
import threading
import time

# IARC Imports
from interdrone_communication.networking_interface import NetworkingInterface
from state_machine.flight_settings import FlightSettings
from state_machine.drone import Drone
from interdrone_communication.message_types import Message, MessageType
from state_machine.drone_state import DroneState
from enum import Enum

if TYPE_CHECKING:
    # If this import is left outside of the TYPE_CHECKING check,
    # it causes a circular import.
    from state_machine.states.state import State


async def get_input(prompt: str) -> str:
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, input, prompt)


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

        # Store messages that each function may need
        self.interdrone_messages = {
            MessageType.PING_ACK: queue.Queue(),
            MessageType.PING_NACK: queue.Queue(),
            # NOTE ADD OTHER MESSAGES HERE AS NEEDED
        }

        # Create interdrone resources
        # Create instance of NetworkingThread class and setup resourcesReadyVariable to pass in
        networkingThreadClassInstance: NetworkingThread = NetworkingThread()
        resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)

        # Start networking thread
        networkingThread = threading.Thread(
            target=networkingThreadClassInstance.run_networking_thread,
            args=(resourcesReady, self.flight_settings),
            daemon=True,
        )
        networkingThread.start()

        # Wait for networking to be ready
        self.networking: NetworkingInterface = (
            resourcesReady.get()
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

    async def ping_drones(self) -> bool:
        """
        All drones run this function.
        Sends PING and waits until every other drone has ACK/NACK.
        Returns True only if all responded with ACK.
        """
        # If only drone in test, return true for ping_drones
        if self.flight_settings.other_drones_in_mission == []:
            return True

        # Track responses by drone id: None=not received yet, True=ACK, False=NACK
        ping_by_id: dict[int, bool | None] = {state.drone_id: None for state in self.drone_states}

        ping_message: Message = Message.create(
            id=MessageType.PING,
            dronesToSendData=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )
        self.send(ping_message)

        print(f"Pings have been sent. Current ping response state is {ping_by_id}")
        # TODO may move this functionality down to the interdrone loop and just wait up here. TBD. Keep this ok solution for now
        while None in ping_by_id.values():
            updated = False

            try:
                ack: Message = self.interdrone_messages[MessageType.PING_ACK].get_nowait()
                if ack.senderId in ping_by_id:
                    ping_by_id[ack.senderId] = True
                    updated = True
            except queue.Empty:
                pass

            try:
                nack: Message = self.interdrone_messages[MessageType.PING_NACK].get_nowait()
                if nack.senderId in ping_by_id:
                    ping_by_id[nack.senderId] = False
                    updated = True
            except queue.Empty:
                pass

            if not updated:
                await asyncio.sleep(0.05)

        # Copy results back into DroneState objects
        for state in self.drone_states:
            result = ping_by_id.get(state.drone_id)
            if result is not None:
                state.ping_response = result

        all_ack: bool = all(result is True for result in ping_by_id.values())

        print(f"Return {all_ack} from ping_drones. Ping status is: {ping_by_id}")
        return all_ack

    async def send_ARM(self) -> None:
        """
        Message ID = 520
        Sends an arm message to all other drones in mission.
        Only used by drone 1.
        """
        arm_message: Message = Message.create(
            id=MessageType.ARM,
            dronesToSendData=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            senderId=self.flight_settings.current_drone_ID,
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
            dronesToSendData=(1,),  # Only need to send to drone 1
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )
        self.send(arm_ack_message)

        return

    async def send_arm_nack(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Sends arm_nack message to drone 1.
        Not used by drone 1, only recieved.
        """
        arm_ack_message: Message = Message.create(
            id=MessageType.ARM_NACK,
            dronesToSendData=dronesToSendData,  # Only need to send to drone 1
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )
        self.send(arm_ack_message)

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

    async def send_takeoff(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Send a takeoff message to the drones passed as a parameter
        """
        takeoff_message: Message = Message.create(
            id=MessageType.START_TAKEOFF,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
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
            dronesToSendData=(1,),
            senderId=self.flight_settings.current_drone_ID,
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

    async def send_start_demo(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Send a start demo message to the drone id passed as a parameter
        """
        demo_message: Message = Message.create(
            id=MessageType.START_DEMO,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
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
            dronesToSendData=(1,),
            senderId=self.flight_settings.current_drone_ID,
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

    async def send_start_mission(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Send a start mission message to the drone id passed as a parameter.
        """
        mission_message: Message = Message.create(
            id=MessageType.START_MISSION,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
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
            dronesToSendData=(1,),
            senderId=self.flight_settings.current_drone_ID,
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
        dronesToSendData: tuple[int, ...],
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
        checksum = Waypoint.getChecksum(waypoints)
        for target_drone in dronesToSendData:
            state = self.get_drone_state_from_id(target_drone)
            if state is not None:
                new_waypoints_message: Message = Message.create(
                    id=MessageType.NEW_WAYPOINTS,
                    dronesToSendData=(target_drone,),
                    senderId=self.flight_settings.current_drone_ID,
                    data={
                        "newWaypoints": waypoints,
                        "senderDroneWaypointsChecksum": checksum,
                    },
                )
                self.send(new_waypoints_message)
                state.waypoint_up_to_date = False

        return

    async def reached_waypoint(self, dronesToSendData: tuple[int, ...], waypoint: Waypoint) -> None:
        """
        Message ID = 550
        Send reached_waypoint message to all drones
        """
        reached_waypoint_message: Message = Message.create(
            id=MessageType.REACHED_WAYPOINT,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
            data={
                "reachedWaypointId": waypoint.waypoint_id,
            },
        )

        self.send(reached_waypoint_message)

        for drone_id in dronesToSendData:
            state = self.get_drone_state_from_id(drone_id)
            if state is not None:
                state.waypoint_up_to_date = False

        return

    async def send_survey_start(self) -> None:
        """
        Message ID = 565
        Set the CMD_MSG in interdrone to SURVEY_START
        Send Survey_Start message to all other drones
        """
        survey_message: Message = Message.create(
            id=MessageType.SURVEY_START,
            dronesToSendData=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_message)

        return

    async def send_survey_ack(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Message ID = 566
        Sends survey_start_ack message to the drone that sent the original message
        dronesToSendData should just be one drone
        """
        survey_ack_message: Message = Message.create(
            id=MessageType.SURVEY_START_ACK,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
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
            dronesToSendData=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_end_message)

        return

    async def send_survey_end_ack(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Message ID = 571
        Sends survey_end_ack message to the drone that sent the original message
        dronesToSendData should just be one drone
        """
        survey_end_ack_message: Message = Message.create(
            id=MessageType.SURVEY_END_ACK,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(survey_end_ack_message)

        return

    async def send_mission_end(self) -> None:
        """
        Message ID = 585
        Sends mission_end message to all other drones
        """
        mission_end_message: Message = Message.create(
            id=MessageType.MISSION_END,
            dronesToSendData=tuple(
                self.flight_settings.other_drones_in_mission,
            ),
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(mission_end_message)

        return

    async def send_mission_end_ack(self, dronesToSendData: tuple[int, ...]) -> None:
        """
        Message ID = 586
        Sends mission_end_ack message to the drone that sent the original message
        dronesToSendData should just be one drone
        """
        mission_end_ack_message: Message = Message.create(
            id=MessageType.MISSION_END_ACK,
            dronesToSendData=dronesToSendData,
            senderId=self.flight_settings.current_drone_ID,
            data={},
        )

        self.send(mission_end_ack_message)

        return

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
        asyncio.ensure_future(self._restart_callback(state))

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
                # Check for server messages
                message = self.networking.try_get_server_message(timeout=0.02)
                if message is not None:
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
                                        dronesToSendData=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                # Other drones send ACK
                                else:
                                    await self.send_arm_ack()
                            # Send ARM_NACK if drone can't arm
                            else:
                                await self.send_arm_nack(dronesToSendData=(message.senderId,))

                        case MessageType.ARM_ACK:
                            # When drone 1 receives an ACK, set others drone arm state to true
                            state = next(
                                (s for s in self.drone_states if s.drone_id == message.senderId),
                                None,
                            )

                            if state is not None:
                                state.armed = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.senderId}. Something is ary!"
                                )
                        case MessageType.ARM_NACK:
                            # Try and resend ARM to drone that sent NACK
                            print(f"Drone {message.senderId} failed to arm. Resending message.")
                            await self.send_ARM(dronesToSendData=(message.senderId,))
                        case MessageType.DISARM:
                            self.cmd_msg = CMD_MSG.DISARM
                            if self.flight_settings.current_drone_ID == 1:
                                # Drone 1 distributes message to other drones in the mission
                                disarm_message: Message = Message.create(
                                    id=MessageType.DISARM,
                                    dronesToSendData=(message.senderId,),
                                    senderId=self.flight_settings.current_drone_ID,
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
                                        dronesToSendData=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                # Other drones send ACK
                                else:
                                    await self.send_takeoff_ack()
                        case MessageType.START_TAKEOFF_ACK:
                            state = self.get_drone_state_from_id(message.senderId)
                            if state is not None:
                                state.takeoff = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.senderId}! Something is ary!"
                                )
                        case MessageType.START_DEMO:
                            print("GOT START DEMO MESSAGE")
                            if self.cmd_msg == CMD_MSG.ARM:  # TODO VERIFY THIS IS CORRECT
                                self.cmd_msg = CMD_MSG.DEMO
                                if self.flight_settings.current_drone_ID == 1:
                                    await self.send_start_demo(
                                        dronesToSendData=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                else:
                                    await self.send_demo_ack()
                        case MessageType.START_DEMO_ACK:
                            state = self.get_drone_state_from_id(message.senderId)

                            if state is not None:
                                state.demo_start = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.senderId}! Something is ary!"
                                )
                        case MessageType.DEMO_DONE:
                            self.cmd_msg = CMD_MSG.DEMO_DONE
                            if self.flight_settings.current_drone_ID == 1:
                                demo_done_message: Message = Message.create(
                                    id=MessageType.DEMO_DONE,
                                    dronesToSendData=tuple(
                                        self.flight_settings.other_drones_in_mission,
                                    ),
                                    senderId=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(demo_done_message)
                        case MessageType.START_MISSION:
                            print("start mission recieved")
                            if self.cmd_msg == CMD_MSG.ARM:
                                self.cmd_msg = CMD_MSG.MISSION
                                if self.flight_settings.current_drone_ID == 1:
                                    await self.send_start_mission(
                                        dronesToSendData=tuple(
                                            self.flight_settings.other_drones_in_mission,
                                        )
                                    )
                                else:
                                    await self.send_mission_ack()
                        case MessageType.START_MISSION_ACK:
                            state = self.get_drone_state_from_id(message.senderId)

                            if state is not None:
                                state.mission_start = True
                            else:
                                print(
                                    f"No DroneState found for drone_id={message.senderId}! Something is ary!"
                                )
                        case MessageType.NEW_WAYPOINTS:
                            # Get state and set list_of_waypoints
                            state = self.get_drone_state_from_id(message.senderId)
                            # TODO CHECK THE CHECKSUM TO DETERMINE WHETHER TO SEND RECONFIRM WAYPOINTS
                            if state is not None:
                                # Add received waypoints to list_of_waypoints
                                state.list_of_waypoints += message.data["newWaypoints"]
                                print(state.list_of_waypoints)

                                # TODO IMPLEMENT CHECKSUM HERE
                                # Get checksum of self.drone.waypoint_checksum and compare to message.data[""]
                                checksum = Waypoint.getChecksum(state.list_of_waypoints)
                                print(
                                    f"Comparing checksum. message.checksum = {message.data['senderDroneWaypointsChecksum']} and checksum(state.list_of_waypoints) = {checksum}"
                                )
                                # Check if stored list of waypoints matches what the other drone has
                                # If so, send NEW_WAYPOINTS_ACK
                                if checksum == message.data["senderDroneWaypointsChecksum"]:
                                    waypoints_ack_message: Message = Message.create(
                                        id=MessageType.NEW_WAYPOINTS_ACK,
                                        dronesToSendData=(message.senderId,),
                                        senderId=self.flight_settings.current_drone_ID,
                                        data={},
                                    )
                                    self.send(waypoints_ack_message)
                                # If checksums don't match, send reconfirm waypoints message
                                else:
                                    state.waypoint_up_to_date = False
                                    waypoints = self.drone.getWaypoints()
                                    reconfirm_waypoints_message: Message = Message.create(
                                        id=MessageType.RECONFIRM_WAYPOINTS,
                                        dronesToSendData=(message.senderId,),
                                        senderId=self.flight_settings.current_drone_ID,
                                        data={
                                            "allWaypoints": waypoints,
                                            "needResponse": True,
                                        },
                                    )
                                    self.send(reconfirm_waypoints_message)
                                self.drone.checkForCollision(state.list_of_waypoints)
                            # TODO HARPER CALL STATE MACHINE WAYPOINT STUFF?

                        case MessageType.NEW_WAYPOINTS_ACK:
                            state = self.get_drone_state_from_id(message.senderId)
                            if state is not None:
                                state.waypoint_up_to_date = True
                        case MessageType.REACHED_WAYPOINT:
                            state = self.get_drone_state_from_id(message.senderId)

                            if state is not None:
                                """
                                print(
                                    f"Got reached waypoint. State of waypoint list before: {state.list_of_waypoints} "
                                )
                                """
                                print("_______________________________________________")
                                reached_id = message.data["reachedWaypointId"]
                                reached_waypoint: Waypoint | None = None
                                for waypoint in state.list_of_waypoints:
                                    if waypoint.waypoint_id == reached_id:
                                        reached_waypoint = waypoint
                                if reached_waypoint is not None:
                                    state.list_of_waypoints.remove(reached_waypoint)
                                    reached_waypoint.has_visited = True

                                # print(f"State of waypoint list after: {state.list_of_waypoints} ")
                                reached_waypoint_ack_message: Message = Message.create(
                                    id=MessageType.REACHED_WAYPOINT_ACK,
                                    dronesToSendData=(message.senderId,),
                                    senderId=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(reached_waypoint_ack_message)
                            # TODO HARPER CALL STATE MACHINE WAYPOINT STUFF
                        case MessageType.REACHED_WAYPOINT_ACK:

                            state = self.get_drone_state_from_id(message.senderId)
                            if state is not None:
                                state.waypoint_up_to_date = True
                        case MessageType.RECONFIRM_WAYPOINTS:
                            state = self.get_drone_state_from_id(message.senderId)

                            if state is not None:
                                # Update list of waypoints with the correct ones
                                state.list_of_waypoints = message.data["allWaypoints"]
                                state.waypoint_up_to_date = True
                                # Message needs responses, send back message with current drones waypoints
                                if message.data["needResponse"]:
                                    waypoints = self.drone.getWaypoints()
                                    reconfirm_waypoints_message_response: Message = Message.create(
                                        id=MessageType.RECONFIRM_WAYPOINTS,
                                        dronesToSendData=(message.senderId,),
                                        senderId=self.flight_settings.current_drone_ID,
                                        data={
                                            "allWaypoints": waypoints,
                                            "needResponse": False,
                                        },
                                    )
                                    self.send(reconfirm_waypoints_message_response)
                        case MessageType.EMERGENCY_LAND:
                            self.cmd_msg = CMD_MSG.EMERGENCY_LAND
                            if self.flight_settings.current_drone_ID == 1:
                                emergency_land_message: Message = Message.create(
                                    id=MessageType.EMERGENCY_LAND,
                                    dronesToSendData=tuple(
                                        self.flight_settings.other_drones_in_mission,
                                    ),
                                    senderId=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(emergency_land_message)
                        case MessageType.LAND:
                            self.cmd_msg = CMD_MSG.LAND
                            if self.flight_settings.current_drone_ID == 1:
                                land_message: Message = Message.create(
                                    id=MessageType.LAND,
                                    dronesToSendData=tuple(
                                        self.flight_settings.other_drones_in_mission,
                                    ),
                                    senderId=self.flight_settings.current_drone_ID,
                                    data={},
                                )
                                self.send(land_message)

                            # Check if drone 1 received and then distribute
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
