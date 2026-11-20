"""Implements the behavior of the EmergencyLand state."""

import asyncio
import logging
import time

import flight.flight_log as flight_log

# state_machine.drone patches the collections aliases dronekit needs on
# import, so it must come before dronekit.
import state_machine.drone  # noqa: F401  (imported for its dronekit patch)
import dronekit
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.emergency_land import EmergencyLand

# Modes that are already bringing the vehicle down. RTL counts: if the autopilot
# has taken the vehicle for a failsafe return there is nothing to gain from
# fighting it back into LAND mid-descent.
DESCENDING_MODES: frozenset[str] = frozenset({"LAND", "RTL"})

# How long to wait for a commanded mode change to show up in telemetry before
# sending it again. DroneKit mode sets are fire-and-forget over MAVLink, and
# this is the one command in the mission that must not be silently dropped.
MODE_CONFIRM_S: float = 1.0

# How often the descent is checked. Fast enough that a dropped mode set is
# re-sent promptly, slow enough not to spin.
POLL_INTERVAL_S: float = 0.25


async def run(self: EmergencyLand) -> None:
    """
    Implements the run method for the EmergencyLand state.

    Puts the vehicle into LAND mode where it currently is and waits for the
    motors to disarm. There is no transit and no successor state: this is where
    a flight ends once an emergency land has been commanded.

    Notes
    -----
    Descending in place is deliberate. The drones fly the demo on hover points a
    fixed distance apart, so coming straight down keeps that separation, while
    anything that routes them through shared airspace would need the collision
    avoidance that dies with the state this one replaced.

    This state takes no `atomic` lock. Every other state uses one to avoid being
    cancelled mid-manoeuvre; this state is the manoeuvre that must not be
    deferred, and nothing should be waiting to interrupt it.
    """
    update_state("EmergencyLand")
    update_drone(self.drone)
    update_flight_settings(self.flight_settings)

    vehicle = self.drone.vehicle
    logging.critical(
        "EmergencyLand state running -- descending in place (mode=%s, armed=%s)",
        vehicle.mode.name,
        vehicle.armed,
    )
    flight_log.event(
        "emergency_land_commanded",
        mode=vehicle.mode.name,
        armed=vehicle.armed,
    )

    if not vehicle.armed:
        # Already down, or never left the ground. Commanding LAND at a disarmed
        # vehicle achieves nothing and leaves it in a mode nobody chose.
        logging.info("Vehicle is already disarmed, nothing to land")
        flight_log.event("emergency_landed", commanded=False)
        return

    commands_sent = 0
    last_command_at = 0.0

    while vehicle.armed:
        if vehicle.mode.name not in DESCENDING_MODES:
            now = time.monotonic()
            if commands_sent == 0 or now - last_command_at >= MODE_CONFIRM_S:
                if commands_sent:
                    logging.warning(
                        "LAND mode has not taken after %.1fs (mode=%s), re-sending",
                        now - last_command_at,
                        vehicle.mode.name,
                    )
                    flight_log.event(
                        "emergency_land_mode_resent",
                        attempt=commands_sent + 1,
                        mode=vehicle.mode.name,
                    )
                vehicle.mode = dronekit.VehicleMode("LAND")
                commands_sent += 1
                last_command_at = now

        await asyncio.sleep(POLL_INTERVAL_S)

    logging.info("Emergency landing complete, motors disarmed")
    flight_log.event("emergency_landed", commanded=True, mode_commands=commands_sent)
    return


# Setting the run_callable attribute of the EmergencyLand class to the run function
EmergencyLand.run_callable = run
