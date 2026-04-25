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
        action_type=""
        while True:
            print("waiting")
            print(f"cmd msg: {self.interdrone.get_cmd_msg()}")
            if(self.flight_settings.mission_type=="Prompted"):
                action_type = await get_input("Enter action command (takeoff, demo, or mission): ")

            if self.interdrone.get_cmd_msg() == CMD_MSG.DEMO or action_type.lower() == "demo":
                if self.drone.id == 1:
                    
                    await self.interdrone.send_start_demo(tuple(self.flight_settings.other_drones_in_mission))

                    while not await self.interdrone.all_demo_start():
                        logging.info("Waiting for all drones to start the demo...")
                        
                        await asyncio.sleep(0.1)
                else:
                    await self.interdrone.send_start_demo(tuple([1]))
                await self.drone.takeoff(5)  # Fix altitude later lol
                await asyncio.sleep(5)
                
                return POIF(self.drone, self.flight_settings, self.interdrone)

            if self.interdrone.get_cmd_msg() == CMD_MSG.MISSION or action_type.lower() == "mission":
                if self.drone.id == 1:

                    await self.interdrone.send_start_mission(tuple(self.flight_settings.other_drones_in_mission))

                    while not await self.interdrone.all_mission_start():

                        logging.info("Waiting for all drones to start the mission...")
                        await asyncio.sleep(0.1)
                    break
                else:
                    await self.interdrone.send_start_mission(tuple([1]))
                await self.drone.takeoff(5)  # Fix altitude later lol
                await asyncio.sleep(5)
                
                return InitialCalcScanPath(self.drone, self.flight_settings, self.interdrone)
            if self.interdrone.get_cmd_msg() == CMD_MSG.TAKEOFF or action_type.lower() == "takeoff":
                if self.drone.id == 1:

                    await self.interdrone.send_takeoff(tuple(self.flight_settings.other_drones_in_mission))

                    while not await self.interdrone.all_takeoff():
                        logging.info("Waiting for all drones to takeoff...")
                        await asyncio.sleep(0.1)
                    
                else:
                    await self.interdrone.send_start_takeoff(tuple([1]))
                await self.drone.takeoff(5)  # Fix altitude later lol
                await asyncio.sleep(5)
            
                return Land(self.drone, self.flight_settings, self.interdrone)
            await asyncio.sleep(0.5)
        return Land(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Takeoff state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Takeoff class to the run function
Takeoff.run_callable = run
