from __future__ import annotations

from typing import List, Tuple
import numpy as np


class BlockyObstacle:
    """Simple blocky obstacle wrapper used by the visualization utilities.

    The original implementation was not present in the working tree, so this
    lightweight version provides a compatible constructor and a polygon-like
    vertex list for the plotting code.
    """

    def __init__(
        self,
        blockMatrix: List[List[int]],
        bottomLeftCorner: Tuple[float, float],
        simSquareWidth: int,
        simSquareHeight: int,
        minX: float = 0,
        minY: float = 0,
    ):
        self.blockMatrix = blockMatrix
        self.bottomLeftCorner = bottomLeftCorner
        self.simSquareWidth = simSquareWidth
        self.simSquareHeight = simSquareHeight
        self.vertices = self._build_vertices()


