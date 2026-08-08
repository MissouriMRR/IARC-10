"""
Whole-field operations for CellField: AND/OR/XOR/invert/shift, popcount, and
enumerating "on" cells. Everything here is O(field size) by nature (a
whole-field op has to touch the whole field) -- the goal is just to pay that
cost once per call via native bulk operations, not per cell.
"""

from typing import Iterator

_BYTE_BITS = tuple(tuple(i for i in range(8) if (b >> i) & 1) for b in range(256))
_ON_CELLS_CHUNK_SIZE = 65536
_ZERO_CHUNK = bytes(_ON_CELLS_CHUNK_SIZE)


class _BitwiseOpsMixin:
    def _shifted_bits(self, dx: int, dy: int) -> int:
        if abs(dx) > self._buffer:
            raise ValueError(
                f"dx={dx} exceeds configured buffer={self._buffer}; "
                "a larger horizontal shift would corrupt adjacent rows"
            )
        net = dy * self._stride + dx
        bits = self._as_int()
        if net >= 0:
            shifted = bits << net
        else:
            shifted = bits >> (-net)
        return shifted & self._row_mask

    def shift(self, dx: int, dy: int):
        new = self._new_like()
        new._load_int(self._shifted_bits(dx, dy))
        return new

    def shift_inplace(self, dx: int, dy: int) -> None:
        self._load_int(self._shifted_bits(dx, dy))

    def _expanded_bits(self) -> int:
        if self._buffer < 1:
            raise ValueError(
                f"expand needs buffer>=1 (to shift left/right by one column "
                f"without corrupting the adjacent row -- see shift/_shifted_bits), "
                f"got buffer={self._buffer}"
            )
        bits = self._as_int()
        stride = self._stride
        # OR the field with itself shifted +-1 column and +-1 row (the four
        # von Neumann neighbors) directly on the whole backing int -- four
        # native shift+OR passes, not a per-cell loop, so this costs the
        # same O(field size) as a single shift()/AND/OR regardless of how
        # many cells are set. Each shifted term can spill into a row's
        # buffer zone or past the field's own top/bottom edge; masking once
        # at the end (inside _load_int, same as shift()) clears all of that
        # in one pass instead of after every individual term -- valid
        # because ANDing a single mask over an OR of terms is the same as
        # ANDing each term first, then OR-ing.
        return bits | (bits << 1) | (bits >> 1) | (bits << stride) | (bits >> stride)

    def expand(self):
        """
        Returns a new CellField where every currently-on cell's four
        von Neumann (up/down/left/right, NOT diagonal) neighbors are also
        on, in addition to the cell itself -- a single-cell dilation. See
        _expanded_bits: built from native bulk shift+OR on the field's
        whole backing int, the same cost model as shift()/AND/OR/XOR, not
        a per-cell Python loop.
        """
        new = self._new_like()
        new._load_int(self._expanded_bits())
        return new

    def expand_inplace(self) -> None:
        """In-place version of expand() -- see its docstring."""
        self._load_int(self._expanded_bits())

    def __and__(self, other):
        self._require_same_shape(other)
        new = self._new_like()
        new._load_int(self._as_int() & other._as_int())
        return new

    def __or__(self, other):
        self._require_same_shape(other)
        new = self._new_like()
        new._load_int(self._as_int() | other._as_int())
        return new

    def __xor__(self, other):
        self._require_same_shape(other)
        new = self._new_like()
        new._load_int(self._as_int() ^ other._as_int())
        return new

    def __iand__(self, other):
        self._require_same_shape(other)
        self._load_int(self._as_int() & other._as_int())
        return self

    def __ior__(self, other):
        self._require_same_shape(other)
        self._load_int(self._as_int() | other._as_int())
        return self

    def __ixor__(self, other):
        self._require_same_shape(other)
        self._load_int(self._as_int() ^ other._as_int())
        return self

    def __invert__(self):
        new = self._new_like()
        new._load_int(~self._as_int())
        return new

    def on_cells(self) -> Iterator[tuple[int, int]]:
        # A lowest-set-bit extraction loop on the whole field as one big int
        # (`bits & -bits`, clear, repeat) looks like O(popcount), but each
        # iteration's cost is actually proportional to the CURRENT bit-length
        # of `bits` -- which stays close to the full field size until the
        # highest set bit gets cleared. For a huge, even moderately populated
        # field that's O(popcount * field size), not O(popcount). Scanning
        # the backing bytearray in large chunks instead, skipping any
        # all-zero chunk via a fast C-level bytes comparison, keeps the cost
        # to O(field size in bytes) for the scan plus O(popcount) for the
        # actual per-bit extraction inside non-zero bytes.
        data = self._bytes
        stride = self._stride
        buffer = self._buffer
        height = self._height
        n = len(data)
        for start in range(0, n, _ON_CELLS_CHUNK_SIZE):
            chunk = data[start : start + _ON_CELLS_CHUNK_SIZE]
            zero = _ZERO_CHUNK if len(chunk) == _ON_CELLS_CHUNK_SIZE else bytes(len(chunk))
            if chunk == zero:
                continue
            for offset, byte_val in enumerate(chunk):
                if byte_val == 0:
                    continue
                base = (start + offset) * 8
                for bit_i in _BYTE_BITS[byte_val]:
                    idx = base + bit_i
                    y, rem = divmod(idx, stride)
                    if y >= height or rem < buffer:
                        continue
                    yield rem - buffer, y

    def count(self) -> int:
        return self._as_int().bit_count()
