"""Implements the behavior of the CalcScanPath state."""

import asyncio
import logging

from flight.extract_gps import extract_gps
from state_machine.state_tracker import (
    update_state,
    update_drone,
    update_flight_settings,
)
from state_machine.states.state import State
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.app_share import AppShare

import flight.pathfinding.pathSubdivision as gotoDiv


async def run(self: CalcScanPath) -> State:
    """
    Implements the run method for the CalcScanPath state.

    This method initiates the scan path calculation and valid path checking and transitions to the AppShare state.

    Returns
    -------
    AppShare : State
        The next state after a successful scan path calculation.

    Raises
    ------
    asyncio.CancelledError
        If the execution of the CalcScanPath state is canceled.

    Notes
    -----
    This method is responsible for calculating the scan path and checking for valid paths and transitioning it to the
    AppShare state, which represents sharing data with the app.

    """
    try:
        update_state("CalcScanPath")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("CalcScanPath state running")

        gotoCoords = gotoDiv.gotoPath(currentPath)
        diviedGoto = []
        for y in range(len(gotoCoords)/4):
            diviedGoto.append(gotoCoords[id*6+y])
        self.drone.updateTasks(diviedGoto)
        # Add CalcScanPath code here

        return AppShare(self.drone, self.flight_settings)
    except asyncio.CancelledError as ex:
        logging.error("CalcScanPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the CalcScanPath class to the run function
CalcScanPath.run_callable = run
