"""
Sub-field extraction and shape-placement operations for CellField:
cropping out a vertical band, and finding a set of shape placements that
cover every "on" cell.
"""


class _RegionMixin:
    def _row_slice(self, start_row: int, end_row: int):
        # Rows are stored as fixed-size, contiguous `stride`-bit chunks, so
        # a row-aligned range is just a byte-slice of self._bytes plus a
        # small (<8-bit) realignment shift -- cost is proportional to the
        # SLICE's size, not this field's whole size, since only the bytes
        # covering [start_row,end_row) are ever sliced or converted to an
        # int; the untouched rows outside that range are never touched.
        if not (0 <= start_row < end_row <= self._height):
            raise ValueError(f"row range [{start_row},{end_row}) invalid for height={self._height}")

        lo = start_row * self._stride
        hi = end_row * self._stride
        byte_lo = lo // 8
        byte_hi = (hi + 7) // 8
        bit_offset = lo - byte_lo * 8

        chunk_int = int.from_bytes(self._bytes[byte_lo:byte_hi], "little")
        new_height = end_row - start_row
        extracted = (chunk_int >> bit_offset) & ((1 << (new_height * self._stride)) - 1)

        new_min = (self._min_corner[0], self._min_corner[1] + start_row * self._cell_size[1])
        new_max = (self._min_corner[0] + self._width * self._cell_size[0],
                   self._min_corner[1] + end_row * self._cell_size[1])
        # type(self), not the literal class name, so this mixin never needs
        # to import the composed CellField class itself.
        new_field = type(self)(self._width, new_height, self._buffer, new_min, new_max)
        new_field._load_int(extracted)
        return new_field

    def vertical_slice(self, start_frac: float, end_frac: float):
        """
        Returns a new, smaller field containing only the rows covering the
        vertical band from start_frac to end_frac of this field's height
        (e.g. 0.0-0.25 for the bottom quarter, 0.75-1.0 for the top
        quarter). Width/buffer are unchanged; the returned field's corners
        are adjusted so its own (0,0) lines up with the start of the slice
        in this field's coordinate frame. Cheap even for a small slice of a
        huge field -- see _row_slice.
        """
        if not (0.0 <= start_frac <= end_frac <= 1.0):
            raise ValueError(
                f"start_frac/end_frac must satisfy 0<=start_frac<=end_frac<=1, "
                f"got ({start_frac},{end_frac})"
            )
        start_row = round(start_frac * self._height)
        end_row = round(end_frac * self._height)
        return self._row_slice(start_row, end_row)

    def vertical_slice_index(self, n: int, m: int):
        """Convenience for the n-th (0-indexed) of m equal vertical slices, e.g. vertical_slice_index(0, 4) for the bottom quarter."""
        if not (0 <= n < m):
            raise ValueError(f"n must satisfy 0<=n<m, got n={n}, m={m}")
        return self.vertical_slice(n / m, (n + 1) / m)

    def cover_with_shape(
        self, shape, shape_center: tuple[float, float] = None
    ) -> list[tuple[float, float]]:
        """
        Finds placements of `shape` (a small CellField mask, e.g. a
        fill_disk footprint) whose union covers every "on" cell in this
        field, and returns each placement's center in this field's
        real-world coordinates (e.g. where to fly to for that placement).

        Geometric set cover is NP-hard in general, so this is a heuristic,
        not an exact solver: standard GREEDY set cover, which repeatedly
        picks whichever placement covers the most still-uncovered cells,
        restricted to placements that land at least one of the shape's own
        "on" cells exactly on a currently-uncovered cell (the only
        placements that can possibly help, so nothing relevant is missed).
        This is provably within a log factor of the optimal placement
        count, not optimal itself. Cost is roughly O(remaining^2 * shape
        area) in the worst case -- fine for hundreds of cells to cover, not
        designed for huge target sets.

        `shape_center` (in the shape's own cell coordinates, default: its
        bounding-box center) is the point within the shape each returned
        center refers to.
        """
        shape_cells = list(shape.on_cells())
        if not shape_cells:
            raise ValueError("shape has no on cells -- it can never cover anything")
        if shape_center is None:
            shape_center = (shape.width / 2.0, shape.height / 2.0)

        remaining = set(self.on_cells())
        placements = []
        while remaining:
            best_offset = None
            best_covered = None
            tried = set()
            for rx, ry in remaining:
                for sx, sy in shape_cells:
                    offset = (rx - sx, ry - sy)
                    if offset in tried:
                        continue
                    tried.add(offset)
                    covered = {(sx2 + offset[0], sy2 + offset[1]) for sx2, sy2 in shape_cells} & remaining
                    if best_covered is None or len(covered) > len(best_covered):
                        best_offset = offset
                        best_covered = covered
            placements.append(best_offset)
            remaining -= best_covered

        centers = []
        for ox, oy in placements:
            cx = self._min_corner[0] + (ox + shape_center[0]) * self._cell_size[0]
            cy = self._min_corner[1] + (oy + shape_center[1]) * self._cell_size[1]
            centers.append((cx, cy))
        return centers
