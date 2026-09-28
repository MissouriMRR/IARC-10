"""Shared constants and small helpers for the field / protoMine geometry layer."""

import math
from enum import Enum

CELL_FT = 2.0                # side of a mine-field square
SAFETY_RADIUS_FT = 3.0       # a mine makes every square its circle touches unsafe
VERTEX_INFLATE_FT = 0.1      # graph corners are the outline pushed out by this (mitre)
VISIBILITY_MARGIN_FT = 0.05  # obstacles are grown by this for every clearance test


class nodeDirection(Enum):
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4


DIRECTION_VECTOR = {
    nodeDirection.UP: (0.0, 1.0),
    nodeDirection.DOWN: (0.0, -1.0),
    nodeDirection.LEFT: (-1.0, 0.0),
    nodeDirection.RIGHT: (1.0, 0.0),
}


def snap_to_cell_center(x, y):
    """Centre of the CELL_FT square containing (x, y)."""
    return (math.floor(x / CELL_FT) + 0.5) * CELL_FT, (math.floor(y / CELL_FT) + 0.5) * CELL_FT
