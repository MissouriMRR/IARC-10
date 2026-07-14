"""Defines the state classes."""

from state_machine.states.impl import (
    Land,
    Start,
    Takeoff,
    AppShare,
    CalcScanPath,
    DroneShare,
    InitialCalcScanPath,
    LidarMap,
    Recall,
    Scan,
)
from state_machine.states.state import State
