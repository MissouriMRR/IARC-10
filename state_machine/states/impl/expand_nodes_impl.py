"""Implements the behavior of the ExpandNodes state."""

import asyncio
import logging

import flight.flight_log as flight_log
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.end_run import EndRun
from state_machine.states.expand_nodes import ExpandNodes
from state_machine.states.state import State

# How much extra safety margin (in feet) to grow every known mine's danger
# radius by before ending the run: a further, deliberately conservative pad
# for the final, committed route. TODO: pick a real value from competition
# rules/hardware GPS error budget -- this is a placeholder guess.
MINE_RADIUS_EXPANSION_FT = 2.0


async def run(self: ExpandNodes) -> State:
    """
    Implements the run method for the ExpandNodes state.

    Once the scan is fully complete (CalcScanPath found nothing left to
    check), grows every known mine's danger zone by MINE_RADIUS_EXPANSION_FT
    as a final safety margin (MissionPlanner.expand_margin).

    Currently a no-op: how the path-node planner applies the margin is
    still to be designed (the expanding-mines phase).

    Returns
    -------
    EndRun : State
        Always -- there's nothing else for this state to hand off to.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the ExpandNodes state is canceled.
    """
    try:
        update_state("ExpandNodes")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("ExpandNodes state running")

        planner = self.drone.planner
        if planner is not None:
            expanded = planner.expand_margin(MINE_RADIUS_EXPANSION_FT)
            if expanded is not None:
                flight_log.event("nodes_expanded", **expanded)

        return EndRun(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("ExpandNodes state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the ExpandNodes class to the run function
ExpandNodes.run_callable = run
