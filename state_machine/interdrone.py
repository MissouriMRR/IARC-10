"""
Contains the Interdrone class, which handles interdrone communication messages
and allows for the cancellation and starting of states based on message data.
"""

# Outside Imports
import asyncio
from asyncio import Task
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING
import queue
import threading
import time

# IARC Imports
from interdrone_communication.networking_interface import NetworkingInterface
from state_machine.flight_settings import FlightSettings
from state_machine.drone import Drone
from interdrone_communication.message_types import Message, MessageType
from state_machine.mission_config import DroneInfo
from enum import Enum

if TYPE_CHECKING:
    # If this import is left outside of the TYPE_CHECKING check,
    # it causes a circular import.
    from state_machine.states.state import State


class CMD_MSG(Enum):
    NONE = 0
    ARM = 1
    TAKEOFF = 2
    DEMO = 3
    MISSION = 4


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

    def __init__(self, flight_settings: FlightSettings, drone: Drone):
        from interdrone_communication.network_config import NetworkConfig
        from interdrone_communication.networking_thread import NetworkingThread

        self._current_task: Task | None = None
        self._current_state: "State | None" = None
        self._restart_callback: Callable[["State | None"], Awaitable[None]] | None = (
            None
        )
        self.flight_settings = flight_settings
        self.drone = drone
        self.network_config = NetworkConfig(flight_settings=flight_settings)
        self.cmd_msg: CMD_MSG = CMD_MSG.NONE

        # Create interdrone resources
        # Create instance of NetworkingThread class and setup resourcesReadyVariable to pass in
        networkingThreadClassInstance: NetworkingThread = NetworkingThread()
        resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)

        # Start networking thread
        networkingThread = threading.Thread(
            target=networkingThreadClassInstance.run_networking_thread,
            args=(resourcesReady, NetworkConfig(self.flight_settings)),
            daemon=True,
        )
        networkingThread.start()

        # Wait for networking to be ready
        self.networking: NetworkingInterface = (
            resourcesReady.get()
        )  # Used to interface with networking thread

    def register_state_machine(
        self, callback: Callable[["State | None"], Awaitable[None]]
    ) -> None:
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

    async def ping_drones(self) -> bool:
        # NOTE TO MATT: YOU NEED TO FINISH IMPLEMENTING THIS

        """
        All drones run this function.
        Sets ping_response flag in droneState to false for all drones.
        Loops through all drones and pings them.
        Whenever we recieve a ping_ack we set the ping_response flag to true.
        Wait x seconds and check if all of the ping_response flags are set.
        If they are, return true, otherwise return false.
        """
        # TODO: Get all the drones
        drone_list: list[DroneInfo] = self.network_config.get_other_drones()
        drone_response: dict[int, bool] = {drone["id"]: False for drone in drone_list}

        # TODO NEXT STEP: SCAFFOLD ALL MESSAGES AND THEN BEGIN IMPLEMENTING.

        # TODO: Ping all drones
        for drone in drone_list:
            pass

        # TODO: Wait x seconds and check if all drones responded
        all_respond = True  # check if all actually responded

        return all_respond

    async def send_ARM(self, drone_id: int) -> None:
        """
        Message ID = 430
        Send an arm message to the drone id passed as a parameter.
        """
        # TODO: Send message ID 430 (arm drones) to drone_id

        return

    async def send_arm_ack(self) -> None:
        """
        Sends arm_ack message.
        Not used by drone 1, only recieved.
        """
        # TODO: Send arm_ack message

        return

    # Function name might change
    async def all_armed(self) -> bool:
        """
        Used by drone one.
        Loop through all droneState objects to see if they are armed.
        Return true they are, false otherwise.
        """
        # TODO: Check if all droneState objects are armed
        armed = True  # actually check if they are

        return armed

    async def all_takeoff(self) -> bool:
        """
        Used by drone one
        Loop through all droneState objects to see if they have taken off.
        """
        # TODO: Check if all droneState objects have taken off
        taken_off = True  # actually check if they are

        return taken_off

    async def all_mission_start(self) -> bool:
        """
        Used by drone one
        Loop through all droneState objects to see if they have started mission
        """
        # TODO: Check if all droneState objects have started mission
        started_mission = True  # actually check if they are

        return started_mission

    async def all_demo_start(self) -> bool:
        """
        Used by drone one
        Loop through all droneState objects to see if they have started demo
        """
        # TODO: Check if all droneState objects have started demo
        started_demo = True  # actually check if they are

        return started_demo

    # I'm lowkey guessing on descriptions from here until the async cancel_state
    async def send_takeoff(self, drone_id: int) -> None:
        """
        Send a takeoff message to the drone id passed as a parameter
        """
        # TODO: Send message id xxx (takeoff) to drone_id
        # TODO: actually make a message id for takeoff

        return

    async def send_takeoff_ack(self) -> None:
        """
        Sends takeoff_ack message.
        Not used by drone 1, only recieved.
        """
        # TODO: Send takeoff_ack message

        return

    async def send_start_demo(self, drone_id: int) -> None:
        """
        Send a start demo message to the drone id passed as a parameter
        """
        # TODO: Send message id xxx (start demo) to drone_id
        # TODO: actually make a message id for start demo

        return

    async def send_demo_ack(self) -> None:
        """
        Sends demo_ack message.
        Not used by drone 1, only recieved.
        """
        # TODO: Send demo_ack message

        return

    async def send_start_mission(self, drone_id: int) -> None:
        """
        Send a start mission message to the drone id passed as a parameter.
        """
        # TODO: Send message id xxx (start mission) to drone_id
        # TODO: actually make a message id for start mission

        return

    async def send_mission_ack(self) -> None:
        """
        Sends mission_ack message.
        Not used by drone 1, only recieved.
        """
        # TODO: Send mission_ack message

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
            raise RuntimeError(
                "Cannot restart state machine without a registered callback"
            )

        # Start the restart callback as a separate task but do not wait for it
        asyncio.ensure_future(self._restart_callback(state))

    def read(self) -> None:
        """
        Read an message.
        """
        raise NotImplementedError

    def send(self) -> None:
        """
        Send a message.
        """
        # ADDS A NEW MESSAGE TO QUEUE
        raise NotImplementedError

    def get_cmd_msg(self) -> CMD_MSG:
        return self.cmd_msg

    # Start client code and call the client_loop()
    async def start_interdrone(self):
        await self.interdrone_loop()

    # Check for new messages to send and create tasks to send them
    async def interdrone_loop(self) -> None:
        # TODO maybe setup a fancy message storage class

        heartbeatMessage: Message = Message.create(
            id=MessageType.HEARTBEAT,
            dronesToSendData=(),
            senderId=self.flight_settings.current_drone_ID,
            data={
                "payload": "Hello server!",
            },
        )

        # Main loop
        msgNum = 0  # Used for testing
        startTime = time.time()
        try:
            while True:  # TODO COMMENT THIS STUFF OUT ONCE STATE MACHINE IS GOING
                # Check for server messages
                serverMsg = self.networking.try_get_server_message(timeout=0.02)
                if serverMsg is not None:
                    msgNum += 1
                    print(f"Server Data: {msgNum}")

                # Send heartbeat if queue is empty
                if self.networking.is_client_in_empty():
                    heartbeatMessage.data["payload"] = str(msgNum)
                    self.networking.queue_client_message(heartbeatMessage)

                await asyncio.sleep(0.1)  # TODO CHANGE TO 0 ONCE STATE MACHINE IS LIVE

        except KeyboardInterrupt:
            print(f"Program ran for {time.time() - startTime}")
            print("Shutting down...")
