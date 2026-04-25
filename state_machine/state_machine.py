"""Defines the StateMachine class."""

import asyncio
import logging
from asyncio import Task

from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import Interdrone
from state_machine.states import State


class StateMachine:
    """
    A state machine controlling a drone.

    Attributes
    ----------
    current_state : State
        The state this state machine is currently running.
    drone : Drone
        The drone this state machine controls.
    flight_settings : FlightSettings
        The flight settings this flight uses.
    interdrone : Interdrone
        The interdrone object this state machine uses.
    run_task : Task[None] | None
        The task that runs through the states in a loop. If the state machine
        is not running, this should be None.

    Methods
    -------
    __init__(
        initial_state: State,
        drone: Drone,
        flight_settings: FlightSettings,
        interdrone: Interdrone,
    )
        Initialize a new state machine object.
    run(initial_state: State | None) -> Awaitable[None]
        Run the flight code specific to each state until completion.
    """

    def __init__(
        self,
        initial_state: State,
        drone: Drone,
        flight_settings: FlightSettings,
        interdrone: Interdrone,
    ):
        """
        Initialize a new state machine object.

        Parameters
        ----------
        initial_state : State
            The first state that runs when the state machine is started by the
            `run()` method.
        drone : Drone
            The drone this state machine will control.
        flight_settings : FlightSettings
            The flight settings to use.
        interdrone : Interdrone
            The interdrone communication object to use to coordinate.
        """
        self.current_state: State | None = initial_state
        self.drone: Drone = drone
        self.flight_settings: FlightSettings = flight_settings
        self.interdrone: Interdrone = interdrone
        self.run_task: Task[None] | None = None

        self.interdrone.register_state_machine(self.run)

    async def run(self, initial_state: State | None = None) -> None:
        """Run the flight code specific to each state until completion.

        Parameters
        ----------
        initial_state : State | None
            If provided, sets the state machine's current state. This must
            share the same drone and flight settings as this state machine.
        """
        if self.run_task is not None:
            return

        if initial_state is not None:
            self.current_state = initial_state

        if self.current_state is None:
            raise ValueError("No state to run")

        run_task: Task[None] = asyncio.ensure_future(self._run())
        self.interdrone.update_task(run_task)
        self.run_task = run_task
        logging.info("State Machine started")

        try:
            await run_task
        except asyncio.CancelledError:
            logging.info("State Machine cancelled")
            
            

        if self.run_task is not None:
            self.run_task = None

            logging.info("State Machine complete")

    async def _run(self) -> None:
        """Runs the flight code specific to each state until completion."""
        while self.current_state:
            self.interdrone.update_state(self.current_state)
            self.current_state = await self.current_state.run()
