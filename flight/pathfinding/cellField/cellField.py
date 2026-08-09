"""
CellField: a width x height grid of single-bit cells, packed row-major into
one contiguous bytearray (exportable as one bytes object).

The class itself is assembled from mixins, one per concern, kept in
separate files so no single file has to hold the whole picture at once:

    _core.py          construction, coordinate conversion, get/set,
                       int<->bytes plumbing, copy, serialization
    _bitwise_ops.py    AND/OR/XOR/invert/shift, count, on_cells
    _fill.py           fill_rect/fill_disk/apply_mask
    _path.py           mark_path/path_cells/block_commands
    _regions.py        vertical_slice/cover_with_shape

Every mixin operates purely through `self` (using `type(self)`/`cls` rather
than importing CellField by name for any "make another one of me" calls),
so there's no import cycle between this file and the mixins, and no
mixin needs to know about any other -- they're combined here into one
class with the exact same public API a single-file version would have.
"""

from flight.pathfinding.cellField._core import _CoreMixin
from flight.pathfinding.cellField._bitwise_ops import _BitwiseOpsMixin
from flight.pathfinding.cellField._fill import _FillMixin
from flight.pathfinding.cellField._path import _PathMixin
from flight.pathfinding.cellField._regions import _RegionMixin


class CellField(_CoreMixin, _BitwiseOpsMixin, _FillMixin, _PathMixin, _RegionMixin):
    """
    Each row reserves `buffer` low-order bits as padding before its `width`
    real cells:

        row layout (low -> high): [buffer bits][x=0][x=1]...[x=width-1]

    This lets a raw shift by dx (bounded by |dx| <= buffer) move every row's
    content without corrupting the neighboring row -- any overflow lands in
    a buffer zone, which the post-shift mask then clears.

    Storage is a mutable bytearray so individual get/set are true O(1)
    (touch one byte) rather than O(field size) -- a single Python int would
    be immutable, so every set() would reallocate and copy the entire field.
    Bulk operations (AND/OR/XOR/invert/shift/count/on_cells) instead convert
    to a Python int just-in-time (native, C-level bitwise ops), operate, and
    write the result back -- an O(field size) cost that's unavoidable for a
    whole-field operation regardless of backing representation, now paid
    only when a bulk op actually runs rather than on every mutation.
    """
