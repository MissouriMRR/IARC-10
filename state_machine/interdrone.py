"""
Contains the Interdrone class, which handles interdrone communication messages
and allows for the cancellation and starting of states based on message data.
"""

import asyncio
from asyncio import Task
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING
from state_machine.flight_settings import FlightSettings
from state_machine.drone import Drone

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
        self.drone=drone

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
        raise NotImplementedError
