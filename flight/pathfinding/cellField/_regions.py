"""
Sub-field extraction and shape-placement operations for CellField:
masking out a vertical band, and finding a set of shape placements that
cover every "on" cell.
"""


class _RegionMixin:
    def _row_band(self, start_row: int, end_row: int):
        # Same width/height/buffer/corners as self -- everything outside
        # [start_row,end_row) is cleared, everything inside is copied as-is.
        # Because the result is always the SAME SIZE as self, this can't be
        # cheaper than O(field size) (the output itself is that big), but it
        # avoids doing MORE than that: copy the whole byte array once, then
        # clear the two flanking regions with _fill_bit_range (a bulk
        # byte-range op, not a per-cell loop) rather than converting
        # anything to a giant int.
        if not (0 <= start_row < end_row <= self._height):
            raise ValueError(f"row range [{start_row},{end_row}) invalid for height={self._height}")

        new = self._new_like()
        new._bytes = bytearray(self._bytes)
        lo = start_row * self._stride
        hi = end_row * self._stride
        new._fill_bit_range(0, lo, False)
        new._fill_bit_range(hi, new._total_bits, False)
        return new

    def vertical_slice(self, start_frac: float, end_frac: float):
        """
        Returns a new field the SAME SIZE as this one (same width, height,
        buffer, and corners), with every cell outside the real-world
        vertical band from start_frac to end_frac of this field's height
        cleared (e.g. 0.0-0.25 for the bottom quarter, 0.75-1.0 for the top
        quarter) and every cell inside it copied as-is. Because the result
        matches this field's shape exactly, it can be combined directly
        with this field or any other same-shaped field (AND/OR/XOR/
        apply_mask, etc.) with no coordinate translation needed.
        """
        if not (0.0 <= start_frac <= end_frac <= 1.0):
            raise ValueError(
                f"start_frac/end_frac must satisfy 0<=start_frac<=end_frac<=1, "
                f"got ({start_frac},{end_frac})"
            )
        start_row = round(start_frac * self._height)
        end_row = round(end_frac * self._height)
        return self._row_band(start_row, end_row)

    def vertical_slice_index(self, n: int, m: int):
        """Convenience for the n-th (0-indexed) of m equal vertical slices, e.g. vertical_slice_index(0, 4) for the bottom quarter. Same-size output as vertical_slice."""
        if not (0 <= n < m):
            raise ValueError(f"n must satisfy 0<=n<m, got n={n}, m={m}")
        return self.vertical_slice(n / m, (n + 1) / m)

    def cover_with_shape(
        self, shape, shape_center: tuple[float, float] = None
    ) -> list[tuple[float, float]]:
        """
        Finds placements of `shape` whose union covers every "on" cell in
        this field, and returns each placement's center in this field's
        real-world coordinates (e.g. where to fly to for that placement).
        `shape` is either a small CellField mask (e.g. a fill_disk
        footprint, for a non-rectangular shape) or a plain (width, height)
        pair -- e.g. (2, 3) -- for a solid rectangular block of cells,
        which is the common case and doesn't need a CellField built for it.

        Geometric set cover is NP-hard in general, so this is a heuristic,
        not an exact solver: GREEDY set cover, which repeatedly picks
        whichever placement covers the most still-uncovered cells,
        restricted to placements that land at least one of the shape's own
        "on" cells exactly on a currently-uncovered cell (the only
        placements that can possibly help, so nothing relevant is missed).
        This is provably within a log factor of the optimal placement
        count, not optimal itself. Ties on new-coverage count are broken,
        in order: (1) prefer whichever candidate overlaps ALREADY-PLACED
        shapes the least, so equally good candidates don't stack on each
        other for no reason, which plain greedy has no incentive to avoid
        on its own; (2) prefer the lowest y then lowest x, so that with
        nothing placed yet (or every remaining tie still at zero overlap)
        the choice is a consistent, corner-anchored sweep rather than
        arbitrary iteration order -- this is what actually reconstructs a
        clean non-overlapping grid tiling when the target region divides
        evenly into shape-sized tiles. At each individual step the primary
        criterion (maximize new coverage) is never sacrificed for less
        overlap, but because this is greedy, a different tie-break can
        still occasionally lead to a slightly different total placement
        count later on, not just different overlap -- in practice this
        trades at most a placement or two for a large overlap reduction,
        not the other way around. Cost is roughly O(remaining^2 * shape area) in the worst case
        -- fine for hundreds of cells to cover, not designed for huge
        target sets.

        `shape_center` (in the shape's own cell coordinates, default: its
        bounding-box center) is the point within the shape each returned
        center refers to. A placement near the field's edge is allowed to
        hang off the edge (only at least one of its own cells needs to land
        on an actual target cell) -- if that puts shape_center itself
        outside the field, the returned point falls back to the centroid of
        just the on-field portion of that placement instead, so every
        returned point is always somewhere actually on the field.
        """
        if isinstance(shape, (tuple, list)):
            if len(shape) != 2:
                raise ValueError(f"shape as (width, height) needs exactly 2 values, got {shape}")
            shape_width, shape_height = shape
            if shape_width <= 0 or shape_height <= 0:
                raise ValueError(f"shape (width, height) must both be positive, got {shape}")
            shape_cells = [(x, y) for x in range(shape_width) for y in range(shape_height)]
        else:
            shape_cells = list(shape.on_cells())
            shape_width, shape_height = shape.width, shape.height

        if not shape_cells:
            raise ValueError("shape has no on cells -- it can never cover anything")
        if shape_center is None:
            shape_center = (shape_width / 2.0, shape_height / 2.0)

        remaining = set(self.on_cells())
        placed_footprint = set()  # union of every cell any placement so far has covered, including overlap
        placements = []
        footprints = []  # this placement's own footprint, kept per-placement (not just merged into placed_footprint) for the off-field center fallback below
        while remaining:
            best_offset = None
            best_score = None
            tried = set()
            for rx, ry in remaining:
                for sx, sy in shape_cells:
                    offset = (rx - sx, ry - sy)
                    if offset in tried:
                        continue
                    tried.add(offset)
                    ox, oy = offset
                    # Only counts here, not a materialized footprint set --
                    # this loop runs once per (remaining cell, shape cell)
                    # pair, and most candidates lose, so building a full
                    # up-to-shape-area set just to intersect and discard it
                    # was pure waste. The winning offset gets its actual
                    # footprint/covered sets rebuilt once, below, after the
                    # loop -- same result, just not paid for on every loser.
                    covered_count = 0
                    overlap_count = 0
                    for sx2, sy2 in shape_cells:
                        cell = (sx2 + ox, sy2 + oy)
                        if cell in remaining:
                            covered_count += 1
                        if cell in placed_footprint:
                            overlap_count += 1
                    # primary: cover as many new cells as possible (keeps the
                    # placement count from getting worse); then: among
                    # equally good candidates, prefer the one that overlaps
                    # already-placed shapes the least; final tiebreak:
                    # lowest y then lowest x -- with nothing placed yet (or
                    # every remaining tie still at zero overlap), the first
                    # two criteria don't distinguish candidates at all, and
                    # an arbitrary pick there (Python set iteration order)
                    # can misalign the whole tiling and force overlap later
                    # that a consistent, corner-anchored sweep would have
                    # avoided -- this is what actually reconstructs a clean
                    # grid tiling when one exists, not the overlap term.
                    score = (covered_count, -overlap_count, -oy, -ox)
                    if best_score is None or score > best_score:
                        best_score = score
                        best_offset = offset
            ox, oy = best_offset
            best_footprint = {(sx2 + ox, sy2 + oy) for sx2, sy2 in shape_cells}
            best_covered = best_footprint & remaining
            placements.append(best_offset)
            footprints.append(best_footprint)
            remaining -= best_covered
            placed_footprint |= best_footprint

        centers = []
        for (ox, oy), footprint in zip(placements, footprints):
            cx = self._min_corner[0] + (ox + shape_center[0]) * self._cell_size[0]
            cy = self._min_corner[1] + (oy + shape_center[1]) * self._cell_size[1]
            if not (
                self._min_corner[0] <= cx <= self._max_corner[0]
                and self._min_corner[1] <= cy <= self._max_corner[1]
            ):
                # The shape hangs far enough off the field's edge that its
                # own (possibly off-center) shape_center lands outside the
                # field entirely -- not just physically wrong (nowhere to
                # fly to), but exactly backwards from the point of this
                # placement, which is to cover the part of the shape that
                # IS on the field. Fall back to the centroid of just the
                # on-field cells (every placement is guaranteed to have at
                # least one, since it was only chosen to cover an on-field
                # target cell in the first place).
                on_field = [
                    (fx, fy) for fx, fy in footprint if 0 <= fx < self._width and 0 <= fy < self._height
                ]
                avg_x = sum(fx + 0.5 for fx, fy in on_field) / len(on_field)
                avg_y = sum(fy + 0.5 for fx, fy in on_field) / len(on_field)
                cx = self._min_corner[0] + avg_x * self._cell_size[0]
                cy = self._min_corner[1] + avg_y * self._cell_size[1]
            centers.append((cx, cy))
        return centers
