"""
This module contains a test for the cancel and restart functionality of the state machine.
"""

import asyncio
import logging
from typing import Awaitable

from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import Interdrone
from state_machine.state_machine import StateMachine
from state_machine.states import State


class TestState1(State):
    """
    A test state that sleeps for 5 seconds and then transitions to TestState2.
    """

    def run(self) -> Awaitable[State]:
        return self.run_callable()

    async def run_callable(self) -> State:
        """
        Sleeps for 5 seconds and then transitions to TestState2.
        """
        await asyncio.sleep(5)
        return TestState2(self._drone, self._flight_settings, self._interdrone)


class TestState2(State):
    """
    A test state that sleeps for 10 seconds inside of an atomic section,
    sleeps for 10 seconds outside of the atomic section.
    """

    def run(self) -> Awaitable[None]:
        return self.run_callable()

    async def run_callable(self) -> None:
        """
        Runs the state logic: enters an atomic section, sleeps for 10 seconds,
        exits the atomic section, and then sleeps for 10 seconds before returning.
        """
        # Enter an atomic section using async with self.atomic
        async with self.atomic:
            logging.info("State2: Entering atomic section")
            await asyncio.sleep(10)

        logging.info("State2: Exiting atomic section")
        await asyncio.sleep(10)
        return


async def cancel_and_restart_task(
    drone: Drone, flight_settings: FlightSettings, interdrone: Interdrone
) -> None:
    """
    This task seeks to cover every case of where the state machine would be cancelled and restarted.

    First, the state machine is cancelled while TestState2 is inside of its atomic section.
    The state machine is then restarted with a start state of TestState1.
    Then, the state machine is cancelled again, outside of the atomic section.
    The state machine is restarted again, but this time is not given a start state.
    This means that it will restart with the last running state, which will be TestState2.
    The state machine is then allowed to run to completion.
    """
    logging.info("Cancel task: Running cancel and restart task")
    await asyncio.sleep(11)
    logging.info("Cancel task: Cancelling state")
    await interdrone.cancel_state()
    logging.info("Cancel task: Restarting state machine and starting State1 again")
    await interdrone.restart_state_machine(TestState1(drone, flight_settings, interdrone))
    logging.info(
        "Cancel task: State machine restarted and State1 started again, "
        "waiting before cancelling again"
    )
    await asyncio.sleep(18)
    logging.info("Cancel task: Cancelling state again")
    await interdrone.cancel_state()
    logging.info("Cancel task: Restarting state machine without given state")
    await interdrone.restart_state_machine(None)
    logging.info(
        "Cancel task: State machine restarted without given state. "
        "Awaiting state machine completion..."
    )
    while interdrone._current_task is None:  # pylint: disable=protected-access
        # Wait for task to be set
        await asyncio.sleep(0.1)
    if interdrone._current_task is not None:  # pylint: disable=protected-access
        # Wait for state machine task to complete
        await interdrone._current_task  # pylint: disable=protected-access
    logging.info("Cancel task: Done.")


async def start_test_task(
    drone: Drone, flight_settings: FlightSettings, interdrone: Interdrone
) -> None:
    """
    This task starts the state machine with a start state of TestState1,
    then returns when the state machine is first cancelled.
    """
    start_state = TestState1(drone, flight_settings, interdrone)
    state_machine = StateMachine(start_state, drone, flight_settings, interdrone)

    try:
        await state_machine.run()
    except asyncio.CancelledError:
        logging.info("Test: State machine cancelled")

    logging.info("Test: Done.")


async def start_test() -> None:
    """
    Initializes the test by creating a drone, flight settings, and interdrone object,
    then starts the test task and the cancel-and-restart task concurrently with gather.
    """
    drone: Drone = Drone()
    flight_settings: FlightSettings = FlightSettings()
    interdrone: Interdrone = Interdrone(flight_settings=flight_settings, drone=drone)

    drone.use_settings(flight_settings.sim_mode)

    await asyncio.gather(
        start_test_task(drone, flight_settings, interdrone),
        cancel_and_restart_task(drone, flight_settings, interdrone),
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(start_test())
