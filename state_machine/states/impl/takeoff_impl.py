"""Implements the behavior of the Takeoff state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.land import Land
from state_machine.states.state import State
from state_machine.states.takeoff import Takeoff
from state_machine.states.poif import POIF
from state_machine.interdrone import CMD_MSG, get_input
from state_machine.states.initial_calc_scan_path import InitialCalcScanPath


async def run(self: Takeoff) -> State:
    """
    Implements the run method for the Takeoff state.

    This method initiates the drone takeoff process and transitions to the Land state.

    Returns
    -------
    Land : State
        The next state after a successful takeoff.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the Takeoff state is canceled.

    Notes
    -----
    This method is responsible for taking off the drone and transitioning it to the
    land state, which represents the navigation phase to land the drone.

    """
    try:
        update_state("Takeoff")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("Takeoff state running")

        if self.flight_settings.mission_type == "Prompted":
            action_type = await get_input("Enter action command (takeoff, demo, or mission): ")
            await self.drone.takeoff(10)
            await asyncio.sleep(5)  # Wait for the drone to stabilize after takeoff
            while action_type.lower() not in ["takeoff", "demo", "mission"]:
                action_type = await get_input(
                    "Invalid input. Enter action command (takeoff, demo, or mission): "
                )

            if action_type == "takeoff":
                return Land(self.drone, self.flight_settings, self.interdrone)
            elif action_type == "demo":
                return POIF(self.drone, self.flight_settings, self.interdrone)
            elif action_type == "mission":
                return InitialCalcScanPath(self.drone, self.flight_settings, self.interdrone)

        while True:
            if self.interdrone.CMD_MSG == CMD_MSG.DEMO:
                if self.drone.id == 1:
                    for id in self.flight_settings.drones_in_mission:
                        if id != self.drone.id:
                            self.interdrone.send_start_demo(id)

                    while not self.interdrone.all_demo_start():
                        logging.info("Waiting for all drones to start the demo...")
                        await asyncio.sleep(0.1)
                self.drone.takeoff(5)  # Fix altitude later lol
                return POIF(self.drone, self.flight_settings, self.interdrone)

            if self.interdrone.CMD_MSG == CMD_MSG.MISSION:
                if self.drone.id == 1:
                    for id in self.flight_settings.drones_in_mission:
                        if self.drone.id == 1 and id != self.drone.id:
                            self.interdrone.send_start_mission(id)

                    while not self.interdrone.all_mission_start():

                        logging.info("Waiting for all drones to start the mission...")
                        await asyncio.sleep(0.1)
                    break
                await self.drone.takeoff(5)  # Fix altitude later lol
                return InitialCalcScanPath(self.drone, self.flight_settings, self.interdrone)
            if self.interdrone.CMD_MSG == CMD_MSG.TAKEOFF:
                if self.drone.id == 1:
                    for id in self.flight_settings.drones_in_mission:
                        if id != self.drone.id:
                            await self.interdrone.send_takeoff(id)

                    while not self.interdrone.all_takeoff():
                        logging.info("Waiting for all drones to takeoff...")
                        await asyncio.sleep(0.1)
                    break
                await self.drone.takeoff(5)  # Fix altitude later lol
                return Land(self.drone, self.flight_settings, self.interdrone)

        return Land(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Takeoff state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Takeoff class to the run function
Takeoff.run_callable = run
