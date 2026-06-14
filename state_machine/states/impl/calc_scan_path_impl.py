"""Implements the behavior of the CalcScanPath state."""

import asyncio
import logging

import flight.pathfinding.path_subdivision as gotoDiv
from flight.extract_gps import extract_gps
from flight.pathfinding.node_generation import Connection, Field, Mine, Node
from flight.pathfinding.path_calculation import Graph
from flight.pathfinding.utils import seen_by_drone
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.app_share import AppShare
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.state import State


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
        remove_start_end_nodes()
        update_nodes()

        self.drone.start_node, self.drone.end_nodes = place_start_end_nodes()
        newGraph = Graph(self.drone.field.nodeGraph)
        nodeList = newGraph.shortest_path(start, end)

        goto_coords, seg_path = gotoDiv.generate_goto_points(nodeList)

        # This relies on the ID's for the drones being 0-3 and not 1-4 CHANGE IF THAT ISN'T THE CASE
        divied_goto = goto_coords[id * len(divied_goto) / 4 : (id + 1) * len(divied_goto) / 4]
        divied_seg = seg_path[id * len(seg_path) / 4 : (id + 1) * len(seg_path) / 4]

        pruned_goto = seen_by_drone.remove_extra_coords(
            self.drone.seen_tracker, divied_goto, divied_seg, self.drone.get_cam_size()
        )
        self.drone.updateTasks(pruned_goto)
        # Add CalcScanPath code here

        return AppShare(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("CalcScanPath state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the CalcScanPath class to the run function
CalcScanPath.run_callable = run
