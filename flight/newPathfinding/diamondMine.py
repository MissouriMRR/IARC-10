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

    def _build_vertices(self) -> List[Tuple[float, float]]:
        rows = len(self.blockMatrix)
        cols = len(self.blockMatrix[0]) if rows else 0
        occupied: List[Tuple[int, int]] = []

        for y in range(rows):
            for x in range(cols):
                if self.blockMatrix[y][x] != 0:
                    occupied.append((x, y))

        if not occupied:
            return []

        min_x = min(x for x, _ in occupied)
        max_x = max(x for x, _ in occupied)
        min_y = min(y for _, y in occupied)
        max_y = max(y for _, y in occupied)

        ox, oy = self.bottomLeftCorner
        return [
            (ox + min_x, oy + min_y),
            (ox + max_x + 1, oy + min_y),
            (ox + max_x + 1, oy + max_y + 1),
            (ox + min_x, oy + max_y + 1),
        ]

    def generate_vertices_from_block_matrix(
        self, blockMatrix: List[List[int]], bottomLeftCorner: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        self.blockMatrix = blockMatrix
        self.bottomLeftCorner = bottomLeftCorner
        self.vertices = self._build_vertices()
        return self.vertices
