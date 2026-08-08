"""
CellField's core identity, raw storage, and serialization: construction,
coordinate conversion, per-cell get/set, and the int<->bytes plumbing every
other mixin (bitwise ops, fill, path, regions) builds on.
"""
import math


class _CoreMixin:
    def __init__(
        self,
        width: int,
        height: int,
        buffer: int = 1,
        min_corner: tuple[float, float] = (0.0, 0.0),
        max_corner: tuple[float, float] | None = None,
    ):
        if width <= 0:
            raise ValueError(f"width must be positive, got {width}")
        if height <= 0:
            raise ValueError(f"height must be positive, got {height}")
        if buffer < 0:
            raise ValueError(f"buffer must be non-negative, got {buffer}")

        self._width = width
        self._height = height
        self._buffer = buffer
        self._stride = buffer + width
        self._total_bits = self._stride * height
        self._nbytes = (self._total_bits + 7) // 8
        self._row_mask = self._build_row_mask(width, height, buffer, self._stride)
        self._bytes = bytearray(self._nbytes)

        # Real-world bounds this field covers, assumed consistent with the
        # field's own corners (not re-validated here) -- lets callers work
        # in real-world coordinates (real_to_cell/cell_to_real/mark_path)
        # instead of raw cell indices. Defaults to an identity mapping (one
        # unit per cell, origin at (0,0)) so existing width/height/buffer-only
        # construction is unaffected.
        self._min_corner = (float(min_corner[0]), float(min_corner[1]))
        if max_corner is None:
            max_corner = (min_corner[0] + width, min_corner[1] + height)
        self._max_corner = (float(max_corner[0]), float(max_corner[1]))
        if self._max_corner[0] <= self._min_corner[0] or self._max_corner[1] <= self._min_corner[1]:
            raise ValueError(f"max_corner {self._max_corner} must exceed min_corner {self._min_corner}")
        self._cell_size = (
            (self._max_corner[0] - self._min_corner[0]) / width,
            (self._max_corner[1] - self._min_corner[1]) / height,
        )

    @staticmethod
    def _build_row_mask(width: int, height: int, buffer: int, stride: int) -> int:
        # Doubling construction: each of the O(log height) steps costs
        # roughly proportional to the mask's CURRENT size, and that size
        # doubles each step, so the total cost is O(final size) -- i.e.
        # O(height*stride) overall. A naive row-by-row OR loop looks like
        # O(height) iterations but each iteration's cost scales with the
        # mask's accumulated size, making it O(height^2) in practice --
        # far too slow once height reaches into the thousands.
        row_pattern = ((1 << width) - 1) << buffer
        mask = row_pattern
        covered = 1
        while covered < height:
            step = min(covered, height - covered)
            low_part = mask & ((1 << (step * stride)) - 1)
            mask |= low_part << (covered * stride)
            covered += step
        return mask

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def buffer(self) -> int:
        return self._buffer

    @property
    def min_corner(self) -> tuple[float, float]:
        return self._min_corner

    @property
    def max_corner(self) -> tuple[float, float]:
        return self._max_corner

    @property
    def cell_size(self) -> tuple[float, float]:
        return self._cell_size

    def real_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Real-world (x, y) -> the (col, row) cell it falls in (not bounds-checked)."""
        col = math.floor((x - self._min_corner[0]) / self._cell_size[0])
        row = math.floor((y - self._min_corner[1]) / self._cell_size[1])
        return col, row

    def cell_to_real(self, col: int, row: int) -> tuple[float, float]:
        """(col, row) -> the real-world coordinate of that cell's center."""
        x = self._min_corner[0] + (col + 0.5) * self._cell_size[0]
        y = self._min_corner[1] + (row + 0.5) * self._cell_size[1]
        return x, y

    def _same_shape(self, other: "_CoreMixin") -> bool:
        return (
            self._width == other._width
            and self._height == other._height
            and self._buffer == other._buffer
            and self._min_corner == other._min_corner
            and self._max_corner == other._max_corner
        )

    def _require_same_shape(self, other: "_CoreMixin") -> None:
        if not isinstance(other, type(self)) or not self._same_shape(other):
            raise ValueError(
                "CellField shape mismatch: "
                f"({self._width},{self._height},buffer={self._buffer},"
                f"corners={self._min_corner}-{self._max_corner}) vs "
                f"({getattr(other, '_width', None)},{getattr(other, '_height', None)},"
                f"buffer={getattr(other, '_buffer', None)},"
                f"corners={getattr(other, '_min_corner', None)}-{getattr(other, '_max_corner', None)})"
            )

    def _index(self, x: int, y: int) -> int:
        if not (0 <= x < self._width):
            raise IndexError(f"x={x} out of range [0,{self._width})")
        if not (0 <= y < self._height):
            raise IndexError(f"y={y} out of range [0,{self._height})")
        return y * self._stride + self._buffer + x

    # -- true O(1) per-cell access, backed directly by the bytearray -------

    def get(self, x: int, y: int) -> bool:
        idx = self._index(x, y)
        byte_i, bit_i = divmod(idx, 8)
        return bool((self._bytes[byte_i] >> bit_i) & 1)

    def set(self, x: int, y: int, value: bool = True) -> None:
        idx = self._index(x, y)
        byte_i, bit_i = divmod(idx, 8)
        if value:
            self._bytes[byte_i] |= 1 << bit_i
        else:
            self._bytes[byte_i] &= ~(1 << bit_i) & 0xFF

    def clear(self, x: int, y: int) -> None:
        self.set(x, y, False)

    def clear_all(self) -> None:
        """Sets every cell off, in-place. O(field size in bytes) -- one bulk
        zero-fill, not a per-cell loop."""
        self._bytes = bytearray(self._nbytes)

    def _new_like(self):
        # A fresh, same-shaped field carrying the same real-world bounds --
        # used for every internal result of a whole-field op (copy, shift,
        # AND/OR/XOR, invert) so a derived field still knows its own extent.
        # type(self), not the literal class name, so this mixin never needs
        # to import the composed CellField class itself.
        return type(self)(self._width, self._height, self._buffer, self._min_corner, self._max_corner)

    def copy(self):
        new = self._new_like()
        new._bytes = bytearray(self._bytes)
        return new

    # -- int<->bytes plumbing shared by bitwise ops, fill, and regions -----

    def _as_int(self) -> int:
        return int.from_bytes(self._bytes, "little")

    def _load_int(self, value: int) -> None:
        self._bytes = bytearray((value & self._row_mask).to_bytes(self._nbytes, "little"))

    # -- serialization: the bytearray already IS the packed representation -

    def to_bytes(self) -> bytes:
        return bytes(self._bytes)

    @classmethod
    def from_bytes(
        cls,
        data: bytes,
        width: int,
        height: int,
        buffer: int = 1,
        min_corner: tuple[float, float] = (0.0, 0.0),
        max_corner: tuple[float, float] | None = None,
    ):
        field = cls(width, height, buffer, min_corner=min_corner, max_corner=max_corner)
        if len(data) != field._nbytes:
            raise ValueError(
                f"expected {field._nbytes} bytes for a {width}x{height} field "
                f"(buffer={buffer}), got {len(data)}"
            )
        field._bytes = bytearray(data)
        return field

    def __repr__(self) -> str:
        return (
            f"CellField(width={self._width}, height={self._height}, "
            f"buffer={self._buffer}, count={self.count()})"
        )
