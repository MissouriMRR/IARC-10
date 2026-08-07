"""
Path rasterization for CellField: marking every cell a real-world polyline
passes through, and converting a path into an ordered cell sequence or a
run-length-compressed list of cardinal (U/D/L/R) block moves.
"""
import math
from typing import Sequence

_STEP_TO_DIRECTION = {(0, 1): "U", (0, -1): "D", (-1, 0): "L", (1, 0): "R"}


def _clip_segment_to_bounds(
    x0: float, y0: float, x1: float, y1: float, width: int, height: int
) -> tuple[float, float, float, float] | None:
    """
    Liang-Barsky clip of segment (x0,y0)-(x1,y1) (cell-space floats) to the
    rectangle [0,width] x [0,height]. Returns the clipped endpoints, or None
    if the segment never touches the rectangle at all. A path waypoint can
    be arbitrarily far outside the field -- clipping first bounds any later
    cell-by-cell walk to at most O(width+height) steps regardless of how far
    outside the original endpoints were, instead of walking every cell
    between two possibly-enormous out-of-bounds coordinates.
    """
    dx, dy = x1 - x0, y1 - y0
    t_enter, t_exit = 0.0, 1.0
    for p, q in ((-dx, x0), (dx, width - x0), (-dy, y0), (dy, height - y0)):
        if p == 0:
            if q < 0:
                return None
            continue
        t = q / p
        if p < 0:
            if t > t_exit:
                return None
            t_enter = max(t_enter, t)
        else:
            if t < t_enter:
                return None
            t_exit = min(t_exit, t)
    if t_enter > t_exit:
        return None
    return (
        x0 + t_enter * dx,
        y0 + t_enter * dy,
        x0 + t_exit * dx,
        y0 + t_exit * dy,
    )


class _PathMixin:
    def mark_path(self, path: Sequence[tuple[float, float]], value: bool = True) -> None:
        """
        Sets every cell that the polyline through `path` (real-world
        coordinates) geometrically passes through, in-place. Each segment is
        walked cell-by-cell (Amanatides & Woo fast voxel traversal), so a
        diagonal segment can't skip a cell it clips at a grid-line crossing.
        """
        if not path:
            return
        if len(path) == 1:
            col, row = self.real_to_cell(*path[0])
            if 0 <= col < self._width and 0 <= row < self._height:
                self.set(col, row, value)
            return
        for i in range(len(path) - 1):
            self._mark_segment(path[i], path[i + 1], value)

    def _mark_segment(
        self, p0: tuple[float, float], p1: tuple[float, float], value: bool
    ) -> None:
        cs_x, cs_y = self._cell_size
        min_x, min_y = self._min_corner
        x0 = (p0[0] - min_x) / cs_x
        y0 = (p0[1] - min_y) / cs_y
        x1 = (p1[0] - min_x) / cs_x
        y1 = (p1[1] - min_y) / cs_y

        # Clip to the field's own rectangle BEFORE walking cell-by-cell --
        # without this, a waypoint far outside the field forces the
        # traversal below to step through every cell between two possibly
        # enormous out-of-bounds coordinates before it can even reach (or
        # miss) the field, which is a real hang risk, not just a slowdown.
        clipped = _clip_segment_to_bounds(x0, y0, x1, y1, self._width, self._height)
        if clipped is None:
            return  # segment never touches the field at all
        x0, y0, x1, y1 = clipped

        # A clipped endpoint can land exactly on the field's outer edge
        # (e.g. x1 == width), whose floor is one past the last valid column
        # -- clamp into range rather than let it slip past bounds checks.
        col = min(max(math.floor(x0), 0), self._width - 1)
        row = min(max(math.floor(y0), 0), self._height - 1)
        end_col = min(max(math.floor(x1), 0), self._width - 1)
        end_row = min(max(math.floor(y1), 0), self._height - 1)

        def _mark(c: int, r: int) -> None:
            if 0 <= c < self._width and 0 <= r < self._height:
                self.set(c, r, value)

        _mark(col, row)
        if col == end_col and row == end_row:
            return

        dx, dy = x1 - x0, y1 - y0
        step_col = 1 if dx > 0 else (-1 if dx < 0 else 0)
        step_row = 1 if dy > 0 else (-1 if dy < 0 else 0)
        INF = float("inf")

        if dx != 0:
            next_boundary_x = col + (1 if step_col > 0 else 0)
            t_max_x = (next_boundary_x - x0) / dx
            t_delta_x = abs(1.0 / dx)
        else:
            t_max_x = t_delta_x = INF

        if dy != 0:
            next_boundary_y = row + (1 if step_row > 0 else 0)
            t_max_y = (next_boundary_y - y0) / dy
            t_delta_y = abs(1.0 / dy)
        else:
            t_max_y = t_delta_y = INF

        # t_max_x/t_max_y accumulate via repeated += of t_delta_x/t_delta_y,
        # which can drift a hair past 1.0 by floating-point error before the
        # traversal actually reaches the end cell -- EPS keeps that drift
        # from triggering an early break that misses the last step or two.
        EPS = 1e-9
        while True:
            if abs(t_max_x - t_max_y) < EPS:
                # Segment passes exactly through a grid corner -- touch both
                # cells adjacent to that corner before stepping diagonally,
                # so the traversal never leaves a gap at the crossing.
                if t_max_x > 1.0 + EPS:
                    break
                _mark(col + step_col, row)
                _mark(col, row + step_row)
                col += step_col
                row += step_row
                t_max_x += t_delta_x
                t_max_y += t_delta_y
            elif t_max_x < t_max_y:
                if t_max_x > 1.0 + EPS:
                    break
                col += step_col
                t_max_x += t_delta_x
            else:
                if t_max_y > 1.0 + EPS:
                    break
                row += step_row
                t_max_y += t_delta_y
            _mark(col, row)
            if col == end_col and row == end_row:
                break

        # Belt-and-suspenders: guarantee the segment's endpoint is always
        # marked regardless of any floating-point edge case in the loop
        # above -- an unconditional O(1) safety net, not a substitute for
        # getting the traversal itself right.
        _mark(end_col, end_row)

    @classmethod
    def from_path(
        cls,
        path: Sequence[tuple[float, float]],
        min_corner: tuple[float, float],
        max_corner: tuple[float, float],
        width: int,
        height: int,
        buffer: int = 1,
    ):
        """Builds a new field of the given shape/bounds with every cell the path passes through set."""
        field = cls(width, height, buffer, min_corner=min_corner, max_corner=max_corner)
        field.mark_path(path)
        return field

    def _segment_cells(
        self, p0: tuple[float, float], p1: tuple[float, float]
    ) -> list[tuple[int, int]]:
        # Like _mark_segment's traversal, but returns the ORDERED list of
        # cells visited instead of setting bits, and resolves an exact
        # grid-corner tie as two explicit single-axis steps rather than a
        # diagonal jump -- every consecutive pair in the result always
        # differs by exactly one cell in exactly one axis, which is what
        # block_commands needs to describe the walk as U/D/L/R moves.
        cs_x, cs_y = self._cell_size
        min_x, min_y = self._min_corner
        x0 = (p0[0] - min_x) / cs_x
        y0 = (p0[1] - min_y) / cs_y
        x1 = (p1[0] - min_x) / cs_x
        y1 = (p1[1] - min_y) / cs_y

        clipped = _clip_segment_to_bounds(x0, y0, x1, y1, self._width, self._height)
        if clipped is None:
            return []
        x0, y0, x1, y1 = clipped

        col = min(max(math.floor(x0), 0), self._width - 1)
        row = min(max(math.floor(y0), 0), self._height - 1)
        end_col = min(max(math.floor(x1), 0), self._width - 1)
        end_row = min(max(math.floor(y1), 0), self._height - 1)

        cells = [(col, row)]
        if col == end_col and row == end_row:
            return cells

        dx, dy = x1 - x0, y1 - y0
        step_col = 1 if dx > 0 else (-1 if dx < 0 else 0)
        step_row = 1 if dy > 0 else (-1 if dy < 0 else 0)
        INF = float("inf")

        if dx != 0:
            next_boundary_x = col + (1 if step_col > 0 else 0)
            t_max_x = (next_boundary_x - x0) / dx
            t_delta_x = abs(1.0 / dx)
        else:
            t_max_x = t_delta_x = INF

        if dy != 0:
            next_boundary_y = row + (1 if step_row > 0 else 0)
            t_max_y = (next_boundary_y - y0) / dy
            t_delta_y = abs(1.0 / dy)
        else:
            t_max_y = t_delta_y = INF

        EPS = 1e-9
        while True:
            if abs(t_max_x - t_max_y) < EPS:
                if t_max_x > 1.0 + EPS:
                    break
                col += step_col
                cells.append((col, row))
                row += step_row
                cells.append((col, row))
                t_max_x += t_delta_x
                t_max_y += t_delta_y
            elif t_max_x < t_max_y:
                if t_max_x > 1.0 + EPS:
                    break
                col += step_col
                cells.append((col, row))
                t_max_x += t_delta_x
            else:
                if t_max_y > 1.0 + EPS:
                    break
                row += step_row
                cells.append((col, row))
                t_max_y += t_delta_y
            if col == end_col and row == end_row:
                break

        if cells[-1] != (end_col, end_row):
            cells.append((end_col, end_row))

        # Post-filter as a final safety net -- clipping should already keep
        # every cell in range, but this guarantees the output never carries
        # an out-of-bounds cell even if some rare float edge case slips one
        # past the clamps above.
        return [(c, r) for c, r in cells if 0 <= c < self._width and 0 <= r < self._height]

    def path_cells(self, path: Sequence[tuple[float, float]]) -> list[tuple[int, int]]:
        """
        Converts real-world `path` waypoints into an ordered, cardinal-step
        (never diagonal) list of (col, row) cells, clipped to this field's
        bounds. If the path leaves and re-enters the field, the returned
        list simply stops covering the out-of-bounds portion and picks back
        up on re-entry -- consecutive cells across that gap won't be a
        single step apart; block_commands() skips such a transition rather
        than raising.
        """
        if not path:
            return []
        if len(path) == 1:
            col, row = self.real_to_cell(*path[0])
            return [(col, row)] if (0 <= col < self._width and 0 <= row < self._height) else []

        cells: list[tuple[int, int]] = []
        for i in range(len(path) - 1):
            seg_cells = self._segment_cells(path[i], path[i + 1])
            if not seg_cells:
                continue
            if cells and cells[-1] == seg_cells[0]:
                cells.extend(seg_cells[1:])
            else:
                cells.extend(seg_cells)
        return cells

    def block_commands(self, path: Sequence[tuple[float, float]]) -> list[tuple[str, int]]:
        """
        Returns `path` as a run-length-compressed list of cardinal block
        moves, e.g. [("U", 10), ("L", 2), ("D", 4), ("R", 6)] -- matching
        the command style BlockField.to_iarc_path uses. Any transition that
        isn't a single cardinal step (only possible where the path leaves
        and re-enters the field's bounds) is skipped rather than raised, so
        an out-of-bounds excursion never breaks command generation.
        """
        cells = self.path_cells(path)
        commands: list[tuple[str, int]] = []
        for i in range(1, len(cells)):
            c0, r0 = cells[i - 1]
            c1, r1 = cells[i]
            direction = _STEP_TO_DIRECTION.get((c1 - c0, r1 - r0))
            if direction is None:
                continue
            if commands and commands[-1][0] == direction:
                commands[-1] = (direction, commands[-1][1] + 1)
            else:
                commands.append((direction, 1))
        return commands
