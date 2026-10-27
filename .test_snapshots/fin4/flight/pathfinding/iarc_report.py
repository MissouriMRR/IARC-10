"""The IARC text path report.

Standalone so either planner can produce the report without importing the other. Moved
here unchanged from flight/pathfinder.py, which re-exports it; the field size is now a
parameter instead of that module's constants.
"""

import logging

from flight.pathfinding.cellField.cellField import CellField

CELL_FT = 2.0

# (col, row) step -> IARC direction letter. Same mapping as cellField._path's private
# _STEP_TO_DIRECTION (row increases north = "U").
_IARC_STEP_TO_DIRECTION: dict[tuple[int, int], str] = {
    (0, 1): "U",
    (0, -1): "D",
    (-1, 0): "L",
    (1, 0): "R",
}


def format_iarc_path(
    points_local_xy: list[tuple[float, float]],
    buffer_width: int = 0,
    strict: bool = True,
    field_w: float = 80.0,
    field_h: float = 300.0,
) -> str:
    """
    Convert ordered LOCAL (x, y) points (feet, not lat/lon) to the IARC text format:
        S,<start_col>,<buffer_width>
        U,9
        L,1
        ...

    Takes:
    - points_local_xy: local (x, y) points, typically many feet apart.
    - buffer_width: safety-buffer width in squares. Only written to the S line, not enforced.
    - strict: raise on a discontinuous path (sims, tests); False logs instead, for a live
      mission, where raising out of EndRun/AppShare would leave the drone hovering.
    - field_w, field_h: field size in feet; the grid is 2ft squares.

    Does:
    - Rasterizes the points into grid cells (CellField.path_cells), then run-length
      compresses steps into U/D/L/R commands.
    - Prepends a virtual entry cell just OUTSIDE the field, on whichever edge the path
      starts from (the nearer one), so the first command enters the field:
        - bottom entry -> virtual row -1, first command "U"
        - top entry    -> virtual row n_rows, first command "D"

    Notes:
    - Takes local xy, NOT lat/lon. A lat/lon round trip isn't lossless and can push a
      point on a cell boundary into the wrong column. Convert once, one-way.

    Raises ValueError if points_local_xy is empty or entirely outside the field.
    """
    cols = round(field_w / CELL_FT)
    n_rows = round(field_h / CELL_FT)
    grid = CellField(cols, n_rows, max_corner=(cols * CELL_FT, n_rows * CELL_FT))
    cells = grid.path_cells(points_local_xy)
    if not cells:
        raise ValueError("no current path to report -- points_local_xy is empty")
    start_col, start_row = cells[0]
    entered_from_top = (n_rows - 1 - start_row) < start_row
    cells = [(start_col, n_rows if entered_from_top else -1)] + cells

    commands: list[tuple[str, int]] = []
    for i in range(1, len(cells)):
        step = (cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1])
        direction = _IARC_STEP_TO_DIRECTION.get(step)
        if direction is None:
            if step == (0, 0):
                continue  # same cell twice; nothing to emit
            msg = (
                f"path is discontinuous at cell {i}: {cells[i - 1]} -> {cells[i]} "
                f"is not a unit step"
            )
            if strict:
                raise ValueError(msg)
            logging.error("format_iarc_path: %s -- reported path skips this gap", msg)
            continue
        if commands and commands[-1][0] == direction:
            commands[-1] = (direction, commands[-1][1] + 1)
        else:
            commands.append((direction, 1))

    lines = [f"S,{start_col},{buffer_width}"]
    lines.extend(f"{direction},{count}" for direction, count in commands)
    return "\n".join(lines)
