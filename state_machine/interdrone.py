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
from interdrone_communication.networking_thread import NetworkingThread
from state_machine.flight_settings import FlightSettings
from interdrone_communication.message_types import Message, MessageType

if TYPE_CHECKING:
    # If this import is left outside of the TYPE_CHECKING check,
    # it causes a circular import.
    from state_machine.states.state import State


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
        self._current_task: Task | None = None
        self._current_state: "State | None" = None
        self._restart_callback: Callable[["State | None"], Awaitable[None]] | None = None
        self.flight_settings = flight_settings
        self.drone = drone

        # Create interdrone resources
        # Create instance of NetworkingThread class and setup resourcesReadyVariable to pass in
        networkingThreadClassInstance: NetworkingThread = NetworkingThread()
        resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)

        # Start networking thread
        networkingThread = threading.Thread(
            target=networkingThreadClassInstance.run_networking_thread,
            args=(resourcesReady, networkConfigData),
            daemon=True,
        )  # TODO FIX ONCE JSON CONFIG STUFF IS MERGED
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

    # Start client code and call the client_loop()
    async def start_interdrone(self):
        await self.interdrone_loop()

    # Check for new messages to send and create tasks to send them
    async def interdrone_loop(self) -> None:
        # TODO maybe setup a fancy message storage class

        heartbeatMessage: Message = Message.create(
            id=MessageType.HEARTBEAT,
            dronesToSendData=(),
            data={
                "senderId": 1,  # TODO UPDATE ONCE CONFIG READER ACTUALLY WORKS :)
                "payload": "Hello server!",
            },
        )

        # Main loop
        msgNum = 0  # Used for testing
        startTime = time.time()
        try:
            while True:
                # Check for server messages
                serverMsg = self.networking.try_get_server_message(timeout=0.02)
                if serverMsg is not None:
                    msgNum += 1
                    # print(f"Server Data: {serverMsg}")
                    print(f"Client Data: {msgNum}")

                # Check for client responses
                clientMsg = self.networking.try_get_client_response(timeout=0.02)
                if clientMsg is not None:
                    # print(f"Client Data: {msgNum}")
                    pass
                # Send heartbeat if queue is empty
                if self.networking.is_client_in_empty():
                    heartbeatMessage.data["payload"] = str(msgNum)
                    self.networking.queue_client_message(heartbeatMessage)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print(f"Program ran for {time.time() - startTime}")
            print("Shutting down...")
