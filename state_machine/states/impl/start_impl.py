"""Implements the behavior of the Start state."""

from typing import Any


import asyncio
import logging
import time

from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.start import Start
from state_machine.states.state import State
from state_machine.states.takeoff import Takeoff
from state_machine.interdrone import CMD_MSG, get_input_or
import dronekit


async def run(self: Start) -> State:
    """
    Implements the run method for the Start state.

    This method establishes a connection with the drone, waits for the drone to be
    discovered, ensures the drone has a global position estimate, arms the drone,
    and transitions to the Takeoff state.

    Returns
    -------
    Takeoff : State
        The next state after the Start state has successfully run.

    Notes
    -----
    This method is responsible for initializing the drone and transitioning it to the
    Takeoff state, which is the next step in the state machine.

    """
    try:
        update_state("Start")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("Start state running")

        await self.drone.connect_drone()
        self.drone.vehicle.mode = dronekit.VehicleMode("GUIDED")

        # Wait until this drone can arm and every other drone answers a ping.
        # is_armable is checked first since it's a local property, while pinging
        # costs a network round trip -- during the (much longer) wait for EKF/GPS
        # convergence there's no point putting ping traffic on the network at all.
        ping_ok: bool = False
        ever_armable: bool = False
        prev_armable: bool = False
        wait_started: float = time.monotonic()
        while True:
            armable: bool = self.drone.vehicle.is_armable
            if armable and not ever_armable:
                ever_armable = True
                logging.info(
                    "Drone %s became armable after %.1fs -- starting pings",
                    self.drone.id,
                    time.monotonic() - wait_started,
                )
            elif prev_armable and not armable:
                # EKF/GPS can fall back out of a converged state. Without this the
                # regression is indistinguishable from a ping failure in the log.
                logging.warning("Drone %s is no longer armable", self.drone.id)
            prev_armable = armable
            if armable:
                ping_ok = await self.interdrone.ping_drones()
                if ping_ok:
                    break
            logging.info("Waiting to start -- armable: %s, ping: %s", armable, ping_ok)
            await asyncio.sleep(1.0)
        logging.info("All drones connected and armable")

        # Wait until the drone has a global position estimate
        # ^Check if itself is ready to arm
        if self.flight_settings.mission_type == "Prompted":
            # A prompted run can be started from the console *or* from the app.
            # The app's ARM only ever lands in interdrone.cmd_msg, so waiting on
            # the console alone leaves the drone parked at this prompt with the
            # command already delivered and nothing looking at it.
            def armed_by_app() -> bool:
                return self.interdrone.get_cmd_msg() == CMD_MSG.ARM

            msg: str | None = await get_input_or(
                "Type 'arm' to arm the drone and start the mission: ", armed_by_app
            )
            while msg is not None and msg.strip().lower() != "arm":
                msg = await get_input_or(
                    "Invalid input. Type 'arm' to arm the drone and start the mission: ",
                    armed_by_app,
                )
            if msg is not None:
                # Armed from the console. Publish it the same way the app's ARM
                # would have, since later states and the app's swarm status both
                # read cmd_msg to decide what this drone is doing.
                self.interdrone.set_cmd_msg(CMD_MSG.ARM)
        else:
            while self.interdrone.get_cmd_msg() != CMD_MSG.ARM:
                await asyncio.sleep(0.1)

        await self.drone.arm()

        if self.drone.id == 1:
            for drone_id in self.flight_settings.drones_in_mission:
                if drone_id != self.drone.id:
                    await self.interdrone.send_ARM((drone_id,))
        while not await self.interdrone.all_armed():
            logging.info("Waiting for all drones to be armed...")
            await asyncio.sleep(0.1)

        return Takeoff(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("Start state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the Start class to the run function
Start.run_callable = run
