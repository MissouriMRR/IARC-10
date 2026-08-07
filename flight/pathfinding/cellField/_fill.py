"""
Region-filling and mask-stamping operations for CellField: rectangles,
disks, and combining a smaller CellField ("mask") into a larger one.
Everything here is built on _fill_bit_range/_combine_bit_range, which write
a contiguous run of bits directly on the byte array -- cost proportional to
the run's size, not the whole field's.
"""


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
        data[byte_lo] = (data[byte_lo] | first_mask) if value else (data[byte_lo] & ~first_mask & 0xFF)

        if byte_hi > byte_lo + 1:
            fill_byte = 0xFF if value else 0x00
            data[byte_lo + 1:byte_hi] = bytes((fill_byte,)) * (byte_hi - byte_lo - 1)

        if bit_hi > 0:
            last_mask = (1 << bit_hi) - 1
            data[byte_hi] = (data[byte_hi] | last_mask) if value else (data[byte_hi] & ~last_mask & 0xFF)

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
            dx = int(dx2 ** 0.5)
            while (dx + 1) * (dx + 1) <= dx2:  # guard against float sqrt rounding down
                dx += 1
            while dx * dx > dx2:
                dx -= 1
            x0 = max(0, cx - dx)
            x1 = min(self._width, cx + dx + 1)
            if x0 < x1:
                row_base = y * self._stride + self._buffer
                self._fill_bit_range(row_base + x0, row_base + x1, value)

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
