"""Shared field constants and cell helpers. No engine lives here (see engines/).

Takes / gives:
- Constants: FIELD_W, FIELD_H, CELL (2 ft grid), SAFETY_RADIUS_FT, plus the visibility-graph
  tolerances VERTEX_INFLATE_FT and VISIBILITY_MARGIN_FT that engines share.
- snap_to_cell_center(x, y): centre of the 2 ft cell holding the point.
- unsafe_cells(x, y, radius): (col, row) of every cell the mine's safety circle touches.
- mine_zone(x, y, radius): convex hull of those cells, as a shapely polygon.

A mine is snapped to its cell centre; its zone is whole unsafe squares, never a partial one.
"""

import math

from shapely.geometry import box
from shapely.ops import unary_union

FIELD_W = 80.0
FIELD_H = 300.0
CELL = 2.0
SAFETY_RADIUS_FT = 3.0
VERTEX_INFLATE_FT = 0.1
VISIBILITY_MARGIN_FT = 0.05


def snap_to_cell_center(x, y):
    """Centre of the 2ft cell containing (x, y)."""
    return (math.floor(x / CELL) + 0.5) * CELL, (math.floor(y / CELL) + 0.5) * CELL


def _circle_touches_square(cx, cy, r, sx, sy):
    """Whether a circle of radius r at (cx, cy) touches the CELLxCELL square at (sx, sy)."""
    px = min(max(cx, sx), sx + CELL)
    py = min(max(cy, sy), sy + CELL)
    return math.hypot(cx - px, cy - py) <= r


def unsafe_cells(x, y, radius=SAFETY_RADIUS_FT):
    """(col, row) of every cell whose square the mine's safety circle touches."""
    cx, cy = snap_to_cell_center(x, y)
    col0, row0 = int(cx // CELL), int(cy // CELL)
    n = math.ceil(radius / CELL)
    out = []
    for dc in range(-n, n + 1):
        for dr in range(-n, n + 1):
            col, row = col0 + dc, row0 + dr
            if _circle_touches_square(cx, cy, radius, col * CELL, row * CELL):
                out.append((col, row))
    return out


def mine_zone(x, y, radius=SAFETY_RADIUS_FT):
    """Convex hull of the mine's unsafe squares."""
    squares = [box(c * CELL, r * CELL, (c + 1) * CELL, (r + 1) * CELL)
               for c, r in unsafe_cells(x, y, radius)]
    return unary_union(squares).convex_hull
