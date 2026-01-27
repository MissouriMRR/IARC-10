"""Implements the behavior of the InitialCalcScanPath state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_state,
    update_drone,
    update_flight_settings,
)
from state_machine.states.state import State
from state_machine.states.initial_calc_scan_path import InitialCalcScanPath
from state_machine.states.scan import Scan

import flight.pathfinding.genCoordsFromPath as gotoDiv


async def run(self: InitialCalcScanPath) -> State:
    """
    Implements the run method for the InitialCalcScanPath state.

    This method initiates the initial scan path calculation and transitions to the Scan state.

    Returns
    -------
    Scan : State
        The next state after a successful initial scan path calculation.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the CalcScanPath state is canceled.

    Notes
    -----
    This method is responsible for calculating the initial scan path and transitioning it to the
    Scan state, which represents scanning the minefield.

    """
    try:
        update_state("InitialCalcScanPath")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("InitialCalcScanPath state running")

        gotoCoords = gotoDiv.gotoPath(currentPath)
        diviedGoto = []
        for y in range(len(gotoCoords)/4):
            diviedGoto.append(gotoCoords[id*6+y])
        self.drone.updateTasks(diviedGoto)
        # Add InitialCalcScanPath code here

        return Scan(self.drone, self.flight_settings)
    except asyncio.CancelledError as ex:
        logging.error("InitialCalcScanPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the InitialCalcScanPath class to the run function
InitialCalcScanPath.run_callable = run
