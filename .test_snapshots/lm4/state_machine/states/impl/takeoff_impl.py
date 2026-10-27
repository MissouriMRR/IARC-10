"""Implements the behavior of the Takeoff state."""

import asyncio
import json
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.land import Land
from state_machine.states.state import State
from state_machine.drone import LEG_ALTITUDE_M, MISSION_ALTITUDE_M
from state_machine.states.takeoff import Takeoff
from state_machine.states.poif import POIF
from state_machine.interdrone import CMD_MSG, get_input_or
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.flight_settings import SimMode
from flight.pathfinding.mission_planner import make_planner
from flight.pathfinding.side_agent import SideAgent
from time import monotonic

# RPICamera and AirSimCamera are imported inside the MISSION branch below, not
# here: importing RPICamera at module level pulls in picamera2, which only
# exists on a Pi with the camera stack installed. That would make run.py
# unimportable on a dev laptop or on any sim node without a camera, even for
# flights that never open one.

# vision/config.json's own keys (hFovDeg, cameraOffsetM, modelPath, ...) --
# see RPICamera.__init__/initialize_camera for what each is read for. Paths in
# it are relative to the repo root, since that is the service WorkingDirectory.
VISION_CONFIG_PATH = "./vision/config.json"
AIRSIM_VISION_CONFIG_PATH = "./vision/airsim_config.json"

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
                self.drone.startTime = monotonic()
                await asyncio.sleep(5)

                return POIF(self.drone, self.flight_settings, self.interdrone)

            if self.interdrone.get_cmd_msg() == CMD_MSG.MISSION or action_type.lower() == "mission":
                configureField(self)
                # RPICamera.__init__ reads its config as a dict, not a path --
                # parse the file first. initialize_camera() actually opens the
                # camera/model (RPICamera.piCam is a class-level singleton, but
                # scan_impl.py reads it off self.drone.camera, set here, so the
                # camera survives even if something else touches the singleton).
                if self.flight_settings.sim_mode is SimMode.AIRSIM:
                    from vision.Cameras.AirSimCamera.AirSimCamera import AirSimCamera

                    with open(AIRSIM_VISION_CONFIG_PATH) as vision_config_file:
                        vision_config = json.load(vision_config_file)
                    camera = AirSimCamera(vision_config)
                    camera.initialize_camera()
                else:
                    from vision.Cameras.RPICamera.RPICamera import RPICamera

                    with open(VISION_CONFIG_PATH) as vision_config_file:
                        vision_config = json.load(vision_config_file)
                    camera = RPICamera(vision_config)
                    camera.initialize_camera()
                self.drone.camera = camera
                if self.drone.id == 1:

                    await self.interdrone.send_start_mission(
                        tuple(self.flight_settings.other_drones_in_mission)
                    )

                    while not await self.interdrone.all_mission_start():

                        logging.info("Waiting for all drones to start the mission...")
                        await asyncio.sleep(0.1)
                else:
                    await self.interdrone.send_start_mission(tuple([1]))
                await self.drone.takeoff(MISSION_ALTITUDE_M)
                self.drone.startTime = monotonic()
                # Peers avoid us, and our side-mate claims work, from where we actually are.
                peers = tuple(self.flight_settings.other_drones_in_mission)
                if peers and self.drone.position_stream is None:
                    self.drone.position_stream = asyncio.create_task(
                        self.interdrone.stream_positions(peers)
                    )
                await asyncio.sleep(5)

                # CalcScanPath starts the planner the first time it runs
                # (planner.start_if_needed) -- no separate "initial" state.
                return CalcScanPath(self.drone, self.flight_settings, self.interdrone)
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
                # CMD_MSG.TAKEOFF is a standalone "just take off and land"
                # command -- not POIF (LEG_ALTITUDE_M) or the mapping
                # mission (also MISSION_ALTITUDE_M, but for a different
                # reason) -- MISSION_ALTITUDE_M chosen here too since
                # there's no competition rule tying this command to a
                # specific altitude.
                await self.drone.takeoff(MISSION_ALTITUDE_M)
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
    Builds this drone's copy of its side's planner (Drone.planner) and the agent that drives
    it (Drone.side, flight/pathfinding/side_agent.py) from the mission config. Every drone
    gets both: a side's drones are equals.

    Raises ValueError here, on the ground, if the field corners don't trace a simple
    counter-clockwise quadrilateral (see flight/pathfinding/frame.py).
    """
    # Local import: calc_scan_path_impl's footprint constants are the planner's too.
    from state_machine.states.impl.calc_scan_path_impl import OVERLAP, SHAPE_SIZE_FT

    state.drone.planner = make_planner(state.flight_settings, SHAPE_SIZE_FT, OVERLAP)
    state.drone.side = SideAgent(state.drone.planner, state.drone, state.flight_settings,
                                 state.interdrone)
