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
from state_machine.drone import LEG_ALTITUDE_M
from state_machine.states.takeoff import Takeoff
from state_machine.states.poif import POIF
from state_machine.interdrone import CMD_MSG, get_input_or
from state_machine.states.initial_calc_scan_path import InitialCalcScanPath
from state_machine.flight_settings import Role, Side
from flight.pathfinder import Pathfinder


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
        action_type = ""
        # Commands from the app arrive as cmd_msg, commands from the operator as
        # typed lines. In prompted mode both are live at once, so the prompt has
        # to be abandonable -- see get_input_or.
        app_commands: tuple[CMD_MSG, ...] = (CMD_MSG.TAKEOFF, CMD_MSG.DEMO, CMD_MSG.MISSION)

        def commanded_by_app() -> bool:
            return self.interdrone.get_cmd_msg() in app_commands

        while True:
            logging.debug("Takeoff waiting -- cmd msg: %s", self.interdrone.get_cmd_msg())
            if self.flight_settings.mission_type == "Prompted":
                typed: str | None = await get_input_or(
                    "Enter action command (takeoff, demo, or mission): ", commanded_by_app
                )
                action_type = typed.strip() if typed is not None else ""
                # Mirror a console command into cmd_msg so the rest of the swarm
                # logic and the app's status view agree on what this drone is doing.
                match action_type.lower():
                    case "demo":
                        self.interdrone.set_cmd_msg(CMD_MSG.DEMO)
                    case "mission":
                        self.interdrone.set_cmd_msg(CMD_MSG.MISSION)
                    case "takeoff":
                        self.interdrone.set_cmd_msg(CMD_MSG.TAKEOFF)

            if self.interdrone.get_cmd_msg() == CMD_MSG.DEMO or action_type.lower() == "demo":
                if self.drone.id == 1:

                    await self.interdrone.send_start_demo(
                        tuple(self.flight_settings.other_drones_in_mission)
                    )

                    while not await self.interdrone.all_demo_start():
                        logging.info("Waiting for all drones to start the demo...")

                        await asyncio.sleep(0.1)
                else:
                    await self.interdrone.send_start_demo(tuple([1]))
                # Take off to exactly the altitude the POIF legs command, with no
                # overshoot margin: every leg holds LEG_ALTITUDE_M, so climbing
                # past it just means descending back down on the first leg.
                await self.drone.takeoff(LEG_ALTITUDE_M, margin=0.0)
                await asyncio.sleep(5)

                return POIF(self.drone, self.flight_settings, self.interdrone)

            if self.interdrone.get_cmd_msg() == CMD_MSG.MISSION or action_type.lower() == "mission":
                configureField(self)
                if self.drone.id == 1:

                    await self.interdrone.send_start_mission(
                        tuple(self.flight_settings.other_drones_in_mission)
                    )

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

                    await self.interdrone.send_takeoff(
                        tuple(self.flight_settings.other_drones_in_mission)
                    )

                    while not await self.interdrone.all_takeoff():
                        logging.info("Waiting for all drones to takeoff...")
                        await asyncio.sleep(0.1)

                else:
                    await self.interdrone.send_takeoff_ack()
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


def configureField(state: Takeoff) -> None:
    """
    Builds this drone's Pathfinder and node field from the mission config --
    GAMBLER/SOLOGAMBLER only. An ASSISTANT never plans its own route (see
    the leader/follower design in flight/pathfinding/tests/droneWorkflowTest.py's
    simulate_leader_follower_pair): it flies whatever segment-B waypoints its
    paired GAMBLER hands it over interdrone comms, so building a second,
    unused Pathfinder for it would be dead weight at best and a second
    source of truth to keep in sync at worst.

    Parameters
    ----------
    state : Takeoff
        The running Takeoff state, whose flight_settings carry the mission
        field corners, altitude, and this drone's ID/pairing/role info.
    """
    if state.flight_settings.role == Role.ASSISTANT:
        return

    missionFieldCorners = []
    for corner in state.flight_settings.mission_field_corners:
        missionFieldCorners.append((corner["lat"], corner["lon"]))

    startEdge = "bottom" if state.flight_settings.start_side == Side.START else "top"

    pathfinder = Pathfinder(
        missionFieldCorners,
        state.flight_settings.max_flight_height,
        90,
        state.flight_settings.current_drone_ID,
    )
    # Pathfinder.__init__ already sets Pathfinder.instance to this object.
    pathfinder.buildNodeField(
        (state.flight_settings.start_coord["lat"], state.flight_settings.start_coord["lon"]),
        startEdge=startEdge,
    )
