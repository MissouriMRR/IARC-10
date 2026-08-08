"""
Region-filling and mask-stamping operations for CellField: rectangles,
disks, and combining a smaller CellField ("mask") into a larger one.
Everything here is built on _fill_bit_range/_combine_bit_range, which write
a contiguous run of bits directly on the byte array -- cost proportional to
the run's size, not the whole field's.
"""

import math


def _point_on_segment(px, py, x1, y1, x2, y2, eps=1e-9) -> bool:
    # Is (px, py) on the CLOSED segment (x1,y1)-(x2,y2), within a small
    # tolerance? Used so a polygon-covered test counts its own boundary as
    # "inside" (a cell whose corner exactly touches the polygon's edge --
    # the common case for a cell that fully fits a rectangular image
    # footprint right up against its edge -- should still count as covered).
    scale = max(1.0, abs(x2 - x1), abs(y2 - y1), abs(px - x1), abs(py - y1))
    cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
    if abs(cross) > eps * scale:
        return False
    dot = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)
    if dot < -eps * scale:
        return False
    seg_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    return dot <= seg_len_sq + eps * scale


def _point_in_polygon(point, poly) -> bool:
    # Closed-region containment: on the boundary counts as inside (checked
    # explicitly first, since plain ray-casting is asymmetric -- it treats
    # only two of a rectangle's four edges as "inside", excluding points
    # exactly on the other two), then the standard ray-casting / even-odd
    # test for the interior (counts how many polygon edges a horizontal ray
    # from `point` toward +x crosses; odd = inside). `poly` is a list of
    # (x, y) vertices in the same (already cell-space) coordinates as `point`.
    x, y = point
    x1, y1 = poly[-1]
    for x2, y2 in poly:
        if _point_on_segment(x, y, x1, y1, x2, y2):
            return True
        x1, y1 = x2, y2

    inside = False
    x1, y1 = poly[-1]
    for x2, y2 in poly:
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
        x1, y1 = x2, y2
    return inside


def _polygon_area(poly) -> float:
    if len(poly) < 3:
        return 0.0
    area = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


def _clip_polygon_to_rect(poly, rx0, ry0, rx1, ry1):
    # Sutherland-Hodgman: clip `poly` (any simple polygon, convex or not)
    # against the rectangle one edge at a time -- correct for a concave
    # SUBJECT polygon (e.g. a merged unionObstacle) since only the CLIP
    # shape needs to be convex, which a rectangle always is.
    def clip(vertices, keep, intersect):
        if not vertices:
            return []
        result = []
        n = len(vertices)
        for i in range(n):
            cur, prev = vertices[i], vertices[i - 1]
            cur_in, prev_in = keep(cur), keep(prev)
            if cur_in:
                if not prev_in:
                    result.append(intersect(prev, cur))
                result.append(cur)
            elif prev_in:
                result.append(intersect(prev, cur))
        return result

    def inter_x(p, q, xb):
        t = (xb - p[0]) / (q[0] - p[0])
        return (xb, p[1] + t * (q[1] - p[1]))

    def inter_y(p, q, yb):
        t = (yb - p[1]) / (q[1] - p[1])
        return (p[0] + t * (q[0] - p[0]), yb)

    verts = list(poly)
    verts = clip(verts, lambda p: p[0] >= rx0, lambda p, q: inter_x(p, q, rx0))
    verts = clip(verts, lambda p: p[0] <= rx1, lambda p, q: inter_x(p, q, rx1))
    verts = clip(verts, lambda p: p[1] >= ry0, lambda p, q: inter_y(p, q, ry0))
    verts = clip(verts, lambda p: p[1] <= ry1, lambda p, q: inter_y(p, q, ry1))
    return verts


def _rect_intersects_polygon(rx0, ry0, rx1, ry1, poly, min_area=1e-9) -> bool:
    # Genuine (positive-area) overlap between the rectangle and `poly` --
    # deliberately NOT satisfied by merely sharing a boundary edge or a
    # single corner point (zero-area contact), which is exactly the case a
    # naive "does any corner/vertex land on the other shape's closed
    # boundary" test gets wrong: for a grid-aligned polygon, every cell
    # diagonally or edge-adjacent to the true footprint shares at least a
    # point or a zero-width edge with it, and a closed/inclusive test would
    # wrongly mark the whole ring of neighbors around the real footprint
    # too (verified empirically: a 4x4 true footprint came back as 6x6
    # before switching to this clip-and-measure-area approach).
    return _polygon_area(_clip_polygon_to_rect(poly, rx0, ry0, rx1, ry1)) > min_area


class _FillMixin:
    def _fill_bit_range(self, lo: int, hi: int, value: bool) -> None:
        # Sets bits [lo, hi) (absolute indices into self._bytes, hi exclusive)
        # directly on the byte array -- cost is proportional to the number of
        # BYTES touched (hi-lo)/8, not the whole field, and the interior
        # whole-byte run is written with one slice assignment (a single fast
        # C-level bulk memory op) rather than a per-cell Python loop. This is
        # the primitive fill_rect/fill_disk are built on.
        if lo >= hi:
            return
        data = self._bytes
        byte_lo, bit_lo = divmod(lo, 8)
        byte_hi, bit_hi = divmod(hi, 8)

        if byte_lo == byte_hi:
            mask = ((1 << (hi - lo)) - 1) << bit_lo
            data[byte_lo] = (data[byte_lo] | mask) if value else (data[byte_lo] & ~mask & 0xFF)
            return

        first_mask = (0xFF << bit_lo) & 0xFF
        data[byte_lo] = (
            (data[byte_lo] | first_mask) if value else (data[byte_lo] & ~first_mask & 0xFF)
        )

        if byte_hi > byte_lo + 1:
            fill_byte = 0xFF if value else 0x00
            data[byte_lo + 1 : byte_hi] = bytes((fill_byte,)) * (byte_hi - byte_lo - 1)

        if bit_hi > 0:
            last_mask = (1 << bit_hi) - 1
            data[byte_hi] = (
                (data[byte_hi] | last_mask) if value else (data[byte_hi] & ~last_mask & 0xFF)
            )

    def fill_rect(self, x0: int, y0: int, x1: int, y1: int, value: bool = True) -> None:
        """
        Sets every cell in [x0,x1) x [y0,y1) (half-open, clipped to the
        field) to `value` in-place. O(rows + area/8) via _fill_bit_range,
        not O(area) individual set() calls.
        """
        x0c, x1c = max(0, x0), min(self._width, x1)
        y0c, y1c = max(0, y0), min(self._height, y1)
        if x0c >= x1c or y0c >= y1c:
            return
        for y in range(y0c, y1c):
            row_base = y * self._stride + self._buffer
            self._fill_bit_range(row_base + x0c, row_base + x1c, value)

    def fill_aligned_rect_covered(
        self, x0: float, y0: float, x1: float, y1: float, value: bool = True
    ) -> None:
        """
        Real-world-coordinate counterpart to fill_rect, for the common case
        of an AXIS-ALIGNED (not rotated) rectangle -- e.g. a camera
        footprint that was never rotated to the drone's heading. Sets every
        cell whose ENTIRE area is enclosed by [x0,x1] x [y0,y1] (same
        "whole cell" semantics as fill_polygon_covered), but computes the
        exact covered cell-index range directly from the box's own edges
        instead of testing each candidate cell's corners against a general
        polygon -- O(rows) via fill_rect, not O(candidate cells) of
        per-cell point-in-polygon tests. Give fill_polygon_covered a real
        (rotated or otherwise non-rectangular) polygon instead; this is
        only correct for a box aligned to this field's own x/y axes.
        """
        col_lo, row_lo, col_hi, row_hi = self._aligned_rect_covered_range(x0, y0, x1, y1)
        self.fill_rect(col_lo, row_lo, col_hi, row_hi, value)

    def fill_aligned_rect_touched(
        self, x0: float, y0: float, x1: float, y1: float, value: bool = True
    ) -> None:
        """
        Loose counterpart to fill_aligned_rect_covered: sets every cell
        [x0,x1] x [y0,y1] overlaps with POSITIVE area (same semantics as
        fill_polygon_touched -- boundary-only/zero-area contact does not
        count), computed directly from the box's edges rather than
        candidate-cell-vs-polygon tests. Only correct for a box aligned to
        this field's own x/y axes -- see fill_aligned_rect_covered.
        """
        col_lo, row_lo, col_hi, row_hi = self._aligned_rect_touched_range(x0, y0, x1, y1)
        self.fill_rect(col_lo, row_lo, col_hi, row_hi, value)

    def _aligned_rect_covered_range(self, x0: float, y0: float, x1: float, y1: float):
        # Exact half-open cell-index range [col_lo,col_hi) x [row_lo,row_hi)
        # of cells FULLY enclosed by the box: a cell [c,c+1) is fully inside
        # iff c >= x0_cell and c+1 <= x1_cell, whose smallest/largest
        # integer solutions are ceil(x0_cell) and floor(x1_cell) (this holds
        # whether or not x0_cell/x1_cell themselves land exactly on an
        # integer cell boundary). EPS nudges each bound back onto its true
        # side before rounding, so ordinary float noise from the /cell_size
        # division can't shift an exact-integer boundary to the wrong cell.
        if x1 < x0:
            x0, x1 = x1, x0
        if y1 < y0:
            y0, y1 = y1, y0
        cs_x, cs_y = self._cell_size
        min_x, min_y = self._min_corner
        EPS = 1e-9
        col_lo = math.ceil((x0 - min_x) / cs_x - EPS)
        col_hi = math.floor((x1 - min_x) / cs_x + EPS)
        row_lo = math.ceil((y0 - min_y) / cs_y - EPS)
        row_hi = math.floor((y1 - min_y) / cs_y + EPS)
        return col_lo, row_lo, col_hi, row_hi

    def _aligned_rect_touched_range(self, x0: float, y0: float, x1: float, y1: float):
        # Exact half-open cell-index range of cells the box overlaps with
        # POSITIVE area: a cell [c,c+1) touches iff c+1 > x0_cell and
        # c < x1_cell (strict, so a cell only sharing a zero-width edge
        # with the box's boundary is excluded) -- whose smallest/largest
        # integer solutions are floor(x0_cell) and ceil(x1_cell). Same EPS
        # nudge as the covered range, in the opposite direction (guarding
        # against a boundary cell being wrongly INCLUDED by float noise
        # instead of wrongly excluded), since here the failure mode of
        # over-inclusion vs. the covered range's under-inclusion are
        # swapped between which bound moves which way.
        if x1 < x0:
            x0, x1 = x1, x0
        if y1 < y0:
            y0, y1 = y1, y0
        cs_x, cs_y = self._cell_size
        min_x, min_y = self._min_corner
        EPS = 1e-9
        col_lo = math.floor((x0 - min_x) / cs_x + EPS)
        col_hi = math.ceil((x1 - min_x) / cs_x - EPS)
        row_lo = math.floor((y0 - min_y) / cs_y + EPS)
        row_hi = math.ceil((y1 - min_y) / cs_y - EPS)
        return col_lo, row_lo, col_hi, row_hi

    def fill_disk(self, cx: int, cy: int, radius: int, value: bool = True) -> None:
        """
        Sets every cell within `radius` of (cx, cy) (inclusive, Euclidean) to
        `value` in-place. Each row's horizontal span is computed from circle
        geometry and written in one _fill_bit_range call -- O(radius rows +
        area/8), not O(area) individual set() calls.
        """
        if radius < 0:
            return
        r2 = radius * radius
        y_lo = max(0, cy - radius)
        y_hi = min(self._height - 1, cy + radius)
        for y in range(y_lo, y_hi + 1):
            dy = y - cy
            dx2 = r2 - dy * dy
            if dx2 < 0:
                continue
            dx = int(dx2**0.5)
            while (dx + 1) * (dx + 1) <= dx2:  # guard against float sqrt rounding down
                dx += 1
            while dx * dx > dx2:
                dx -= 1
            x0 = max(0, cx - dx)
            x1 = min(self._width, cx + dx + 1)
            if x0 < x1:
                row_base = y * self._stride + self._buffer
                self._fill_bit_range(row_base + x0, row_base + x1, value)

    def fill_polygon_covered(self, vertices, value: bool = True) -> None:
        """
        Sets every cell whose ENTIRE area is enclosed by the polygon defined
        by `vertices` (real-world (x, y) points, in order) -- not cells the
        polygon merely touches or partially overlaps (see apply_mask/
        mark_path for that). Meant for "this exact region was fully
        captured/verified" bookkeeping -- e.g. a camera image's corner
        coordinates marking a "seen" field, where a cell should only count
        as seen once the whole cell was actually inside the photographed
        footprint, not just clipped by its edge.

        A cell counts as fully covered when all four of its corners are
        inside the polygon. That's exact for any convex polygon (a camera
        image's rectangular footprint, at any rotation, is always convex)
        and correct for a general simple polygon too: a cell's own
        rectangle is itself convex, so if the polygon's boundary never
        crosses the cell's interior, the four corners alone determine which
        side of the boundary the whole cell is on.
        """
        if len(vertices) < 3:
            raise ValueError(f"a polygon needs at least 3 vertices, got {len(vertices)}")

        cs_x, cs_y = self._cell_size
        min_x, min_y = self._min_corner
        poly = [((vx - min_x) / cs_x, (vy - min_y) / cs_y) for vx, vy in vertices]

        poly_min_x = min(p[0] for p in poly)
        poly_max_x = max(p[0] for p in poly)
        poly_min_y = min(p[1] for p in poly)
        poly_max_y = max(p[1] for p in poly)

        col_lo = max(0, math.floor(poly_min_x))
        col_hi = min(self._width - 1, math.ceil(poly_max_x) - 1)
        row_lo = max(0, math.floor(poly_min_y))
        row_hi = min(self._height - 1, math.ceil(poly_max_y) - 1)

        for row in range(row_lo, row_hi + 1):
            for col in range(col_lo, col_hi + 1):
                corners = ((col, row), (col + 1, row), (col, row + 1), (col + 1, row + 1))
                if all(_point_in_polygon(corner, poly) for corner in corners):
                    self.set(col, row, value)

    def fill_polygon_touched(self, vertices, value: bool = True) -> None:
        """
        Sets every cell the polygon defined by `vertices` (real-world
        (x, y) points, in order) overlaps AT ALL, even just clipping an
        edge or a single corner -- the loose counterpart to
        fill_polygon_covered's strict full-encompass test. Meant for
        rasterizing a small shape's actual footprint onto a CellField for
        visualization/combination the same way a path already is (e.g. a
        mine's safety-zone polygon, typically only a handful of cells
        across) -- fill_polygon_covered's "must be fully inside" test would
        under-represent a shape that small, often to zero cells.

        Correct for concave polygons too (e.g. a merged unionObstacle),
        not just the convex case a standalone mine's own polygon always is:
        each candidate cell is tested for genuine rectangle-vs-polygon
        overlap (vertex-in-rect, corner-in-polygon, or an edge crossing),
        not just "is some single point inside".
        """
        if len(vertices) < 3:
            raise ValueError(f"a polygon needs at least 3 vertices, got {len(vertices)}")

        cs_x, cs_y = self._cell_size
        min_x, min_y = self._min_corner
        poly = [((vx - min_x) / cs_x, (vy - min_y) / cs_y) for vx, vy in vertices]

        poly_min_x = min(p[0] for p in poly)
        poly_max_x = max(p[0] for p in poly)
        poly_min_y = min(p[1] for p in poly)
        poly_max_y = max(p[1] for p in poly)

        # A generous +/-1 cell margin on the candidate range, unlike
        # fill_polygon_covered's tight bound -- correctness here doesn't
        # depend on a tight bound (every candidate still gets an exact
        # test), and boundary-touching cases are easy to get an off-by-one
        # away from a cell whose right/top edge sits exactly at the
        # polygon's min extent.
        col_lo = max(0, math.floor(poly_min_x) - 1)
        col_hi = min(self._width - 1, math.floor(poly_max_x) + 1)
        row_lo = max(0, math.floor(poly_min_y) - 1)
        row_hi = min(self._height - 1, math.floor(poly_max_y) + 1)

        for row in range(row_lo, row_hi + 1):
            for col in range(col_lo, col_hi + 1):
                if _rect_intersects_polygon(col, row, col + 1, row + 1, poly):
                    self.set(col, row, value)

    def _read_bit_range(self, lo: int, hi: int) -> int:
        # Returns bits [lo, hi) as an integer, right-normalized so bit 0 of
        # the result corresponds to bit `lo` of the field.
        if lo >= hi:
            return 0
        byte_lo = lo // 8
        byte_hi = (hi + 7) // 8
        value = int.from_bytes(self._bytes[byte_lo:byte_hi], "little")
        return (value >> (lo - byte_lo * 8)) & ((1 << (hi - lo)) - 1)

    def _combine_bit_range(self, lo: int, hi: int, value: int, op: str) -> None:
        # Combines a (hi-lo)-bit-wide `value` (bit 0 = position lo) into
        # self._bytes at [lo, hi) using `op`, leaving every bit outside
        # [lo, hi) -- including other bits sharing the same boundary bytes
        # -- untouched.
        if lo >= hi:
            return
        byte_lo = lo // 8
        byte_hi = (hi + 7) // 8
        span = byte_hi - byte_lo
        shift = lo - byte_lo * 8
        width = hi - lo
        existing = int.from_bytes(self._bytes[byte_lo:byte_hi], "little")
        shifted_value = (value & ((1 << width) - 1)) << shift

        if op == "or":
            combined = existing | shifted_value
        elif op == "xor":
            combined = existing ^ shifted_value
        elif op == "and":
            full_mask = ((1 << width) - 1) << shift
            combined = (existing & ~full_mask) | (existing & shifted_value)
        elif op == "set":
            full_mask = ((1 << width) - 1) << shift
            combined = (existing & ~full_mask) | shifted_value
        else:
            raise ValueError(f"unknown op {op!r}; expected 'or', 'and', 'xor', or 'set'")

        self._bytes[byte_lo:byte_hi] = combined.to_bytes(span, "little")

    def apply_mask(self, mask, ox: int, oy: int, op: str = "or") -> None:
        """
        Combines `mask` into this field with its own (0,0) placed at
        (ox, oy), in-place. `op` is 'or' (default -- stamp the mask's on
        cells in, e.g. adding a mine's precomputed safety-zone shape),
        'and' (keep only cells on in both), 'xor', or 'set' (overwrite this
        field's cells exactly with the mask's, within the mask's footprint).

        The mask may be any size/shape and is clipped at every edge of this
        field; parts of it that land outside are silently dropped. Cost is
        O(mask area / 8), not O(this field's area) -- each of the mask's
        rows is read and combined as a single bit-range operation, so
        stamping a small precomputed mask onto a huge field is cheap
        regardless of the field's size.
        """
        if op not in ("or", "and", "xor", "set"):
            raise ValueError(f"unknown op {op!r}; expected 'or', 'and', 'xor', or 'set'")

        for my in range(mask._height):
            ty = oy + my
            if not (0 <= ty < self._height):
                continue

            mask_row_lo = my * mask._stride + mask._buffer
            row_bits = mask._read_bit_range(mask_row_lo, mask_row_lo + mask._width)

            src_x0, src_x1 = 0, mask._width
            dst_x0, dst_x1 = ox, ox + mask._width
            if dst_x0 < 0:
                src_x0 += -dst_x0
                dst_x0 = 0
            if dst_x1 > self._width:
                src_x1 -= dst_x1 - self._width
                dst_x1 = self._width
            if src_x0 >= src_x1 or dst_x0 >= dst_x1:
                continue

            trimmed = (row_bits >> src_x0) & ((1 << (src_x1 - src_x0)) - 1)
            target_lo = ty * self._stride + self._buffer + dst_x0
            target_hi = target_lo + (dst_x1 - dst_x0)
            self._combine_bit_range(target_lo, target_hi, trimmed, op)
