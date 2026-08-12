"""Implements the behavior of the ExpandNodes state."""

import asyncio
import logging

import flight.flight_log as flight_log
from flight.pathfinder import Pathfinder
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.end_run import EndRun
from state_machine.states.expand_nodes import ExpandNodes
from state_machine.states.state import State

# How much extra safety margin (in feet) to grow every known mine's
# danger radius by before actually ending the run. The scan itself
# already flies with Pathfinder.mine_saftey_radius's own margin (see
# add_discovered_mine) -- this is a further, deliberately conservative
# pad specifically for the final, committed route, not the exploratory
# one. TODO: pick a real value from competition rules/hardware GPS
# error budget -- this is a placeholder guess.
MINE_RADIUS_EXPANSION_FT = 2.0


async def run(self: ExpandNodes) -> State:
    """
    Implements the run method for the ExpandNodes state.

    Once the scan is fully complete (CalcScanPath found nothing left to
    check), grows every known mine's danger polygon by
    MINE_RADIUS_EXPANSION_FT (Field.expandField, via Pathfinder.
    increase_radius) as a final safety margin, then "restarts the gambler/
    assistant loop" against the now-expanded mine set (Pathfinder.
    repair_path_after_expansion) -- growing every mine at once can break
    edges the planned route relies on the same way a newly-discovered mine
    can mid-scan, and repair_path_after_expansion is the general form of
    the same repair check_path_envelopment already does for one fresh
    discovery, replayed to a fixed point instead of once (see its own
    docstring for why more than one pass can genuinely be needed here).

    An ASSISTANT has no Pathfinder of its own (see configureField), so
    this is a no-op for it.

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

        pf = Pathfinder.instance
        if pf is not None:
            pf.increase_radius(MINE_RADIUS_EXPANSION_FT)
            repair_passes = pf.repair_path_after_expansion()
            flight_log.event(
                "nodes_expanded",
                expansion_ft=MINE_RADIUS_EXPANSION_FT,
                mines=len(pf.nodeField.mines),
                unions=len(pf.nodeField.unionObstacles),
                repair_passes=repair_passes,
            )

        return EndRun(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("ExpandNodes state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the ExpandNodes class to the run function
ExpandNodes.run_callable = run
