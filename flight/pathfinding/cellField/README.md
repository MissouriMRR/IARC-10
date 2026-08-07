# CellField

A `width x height` grid of single-bit cells, packed into one contiguous
`bytearray` (exportable as a single `bytes` object). Built for cases where a
field is disposable and cheap: you build one, use it for a specific purpose
(an obstacle mask, a coverage/"seen" tracker, a path footprint, a mine
safety zone), and throw it away — never one canonical, long-lived field.

Optionally maps onto real-world coordinates (`min_corner`/`max_corner`), so
callers can work in actual field units instead of raw cell indices.

## File layout

The class is assembled from mixins, one per concern, so no single file has
to hold the whole picture:

| File | Responsibility |
|---|---|
| `cellField.py` | Combines the mixins below into the public `CellField` class. Import from here. |
| `_core.py` | Construction, coordinate conversion, per-cell `get`/`set`, int&#8596;bytes plumbing, `copy`, serialization. |
| `_bitwise_ops.py` | Whole-field algebra: AND/OR/XOR/invert/shift, `count`, `on_cells`. |
| `_fill.py` | Region filling and mask stamping: `fill_rect`, `fill_disk`, `apply_mask`. |
| `_path.py` | Rasterizing a real-world path into cells: `mark_path`, `path_cells`, `block_commands`. |
| `_regions.py` | Sub-field extraction and shape-cover placement: `vertical_slice`, `cover_with_shape`. |
| `visual.py` | `render_field` — renders a field to a PNG (black/white, gridlines). Kept separate so using `CellField` never requires importing matplotlib/numpy. |

Every mixin talks to the others only through `self` (never importing
`CellField` by name — `type(self)`/`cls` instead), so there's no import
cycle and no mixin needs to know about any other.

## Design notes (why it's built this way)

- **Storage is a mutable `bytearray`, not a Python `int`.** A single `int`
  would be immutable, so every `set()` would reallocate and copy the whole
  field — fine for occasional edits, ruinous if you're placing many
  individual cells (e.g. rasterizing shapes). A `bytearray` makes `get`/`set`
  true O(1) (touch one byte). Bulk operations (AND/OR/XOR/shift/count/etc.)
  convert to a Python `int` just-in-time, operate with native C-level
  bitwise ops, and write the result back — an O(field size) cost that's
  unavoidable for a whole-field operation regardless of backing
  representation, now paid only when a bulk op actually runs.
- **Each row reserves `buffer` low-order bits as padding**
  (`[buffer bits][x=0][x=1]...[x=width-1]`), which lets a raw horizontal
  shift by `dx` (bounded by `|dx| <= buffer`) move a row's content without
  corrupting the row above/below — any overflow lands in the buffer zone,
  which gets masked off after the shift. `buffer` defaults to `1`; set it
  higher if you'll ever shift by more than 1 cell in one call.
- **Row-major layout** means a *vertical* slice (a contiguous range of rows)
  is just a byte-slice plus a small sub-byte realignment — see
  `vertical_slice`.

## Quick start

```python
from flight.pathfinding.cellField.cellField import CellField

field = CellField(width=100, height=80, buffer=1,
                   min_corner=(0.0, 0.0), max_corner=(200.0, 160.0))

field.set(10, 5)
field.fill_disk(cx=50, cy=40, radius=8)          # a mine's safety zone
field.mark_path([(0.0, 0.0), (120.0, 90.0)])     # a flight path

for x, y in field.on_cells():
    ...

field.clear_all()
```

## API reference

### Construction

- **`CellField(width, height, buffer=1, min_corner=(0.0, 0.0), max_corner=None)`**
  `width`/`height` are cell counts. `buffer` is the per-row padding (see
  above). `min_corner`/`max_corner` establish the real-world coordinate
  mapping; if `max_corner` is omitted it defaults to one unit per cell.
  Raises `ValueError` on non-positive width/height, negative buffer, or
  `max_corner` not exceeding `min_corner`.
- **`CellField.from_bytes(data, width, height, buffer=1, min_corner=(0.0, 0.0), max_corner=None)`**
  Rebuilds a field from bytes produced by `to_bytes()`. Raises `ValueError`
  if `data`'s length doesn't match the given shape.
- **`CellField.from_path(path, min_corner, max_corner, width, height, buffer=1)`**
  Builds a new field of the given shape/bounds with every cell `path`
  passes through set (see `mark_path` below).

### Properties

`width`, `height`, `buffer`, `min_corner`, `max_corner`, `cell_size` (a
`(cell_size_x, cell_size_y)` tuple derived from the corners) — all read-only.

### Coordinates

- **`real_to_cell(x, y) -> (col, row)`** — real-world point to the cell it
  falls in. Not bounds-checked.
- **`cell_to_real(col, row) -> (x, y)`** — a cell's real-world center.

### Per-cell access — O(1)

- **`get(x, y) -> bool`**
- **`set(x, y, value=True)`**
- **`clear(x, y)`** — shorthand for `set(x, y, False)`.
- **`clear_all()`** — turns every cell off. One bulk zero-fill, not a loop.
- **`copy() -> CellField`** — an independent field with the same content and bounds.

### Whole-field bitwise ops — O(field size), each call touches the whole field once

- **`shift(dx, dy) -> CellField`** / **`shift_inplace(dx, dy)`** — translate
  the on/off pattern by `(dx, dy)` cells; bits pushed off an edge are
  dropped, the far side zero-fills. Raises `ValueError` if `abs(dx) >
  buffer` (see Design notes) — `dy` is unrestricted.
- **`field & other`, `field | other`, `field ^ other`, `~field`** — new
  `CellField`. `&=`, `|=`, `^=` mutate in place. All require `other` to have
  the same width/height/buffer/corners as `self` (raises `ValueError`
  otherwise) — combining fields with different real-world bounds is almost
  always a mistake, not something to combine silently.
- **`count() -> int`** — population count (`int.bit_count()`, native).
- **`on_cells() -> Iterator[(x, y)]`** — every on cell. Skips empty regions
  in large chunks rather than testing bit-by-bit, so it stays fast even for
  a huge, sparsely-populated field.

### Filling regions and stamping masks

- **`fill_rect(x0, y0, x1, y1, value=True)`** — sets every cell in the
  half-open `[x0,x1) x [y0,y1)`, clipped to the field.
- **`fill_disk(cx, cy, radius, value=True)`** — sets every cell within
  `radius` (Euclidean, inclusive) of `(cx, cy)`.
- **`apply_mask(mask, ox, oy, op="or")`** — combines another `CellField`
  ("mask", e.g. a small precomputed shape) into this one with the mask's
  `(0,0)` placed at `(ox, oy)`. `op` is `"or"` (stamp in — the common case),
  `"and"` (keep only cells on in both, within the mask's footprint), `"xor"`,
  or `"set"` (overwrite exactly, within the footprint). The mask is clipped
  silently at every field edge. Cost is proportional to the *mask's* size,
  not the field's — build a shape once (`mask.fill_disk(...)`), then stamp
  it at many locations cheaply (e.g. once per detected mine).

Both `fill_rect`/`fill_disk` and `apply_mask` write whole runs of bits
directly (byte-slice assignment for interior spans), not one `set()` call
per cell — meaningfully faster for large filled regions.

### Paths

- **`mark_path(path, value=True)`** — sets every cell the polyline through
  `path` (real-world `(x, y)` waypoints) geometrically passes through.
  Walked with Amanatides & Woo fast voxel traversal (not basic Bresenham),
  so a diagonal segment can't skip a cell it clips at a grid-line crossing.
  Waypoints may lie anywhere, including far outside the field — each
  segment is clipped to the field's rectangle before being walked, so an
  out-of-bounds excursion costs no more than the in-bounds portion (not
  "however far outside the point was").
- **`path_cells(path) -> [(col, row), ...]`** — `path` as an ordered,
  strictly-cardinal-step list of cells (no diagonal jumps — a grid-corner
  crossing becomes two single-axis steps). If the path leaves and re-enters
  the field, the list just stops covering the outside portion and resumes
  on re-entry.
- **`block_commands(path) -> [(direction, count), ...]`** — `path` as
  run-length-compressed cardinal moves, e.g.
  `[("U", 10), ("L", 2), ("D", 4), ("R", 6)]` (matches the command style
  `BlockField.to_iarc_path` uses elsewhere in this repo). A transition that
  isn't a single cardinal step (only possible where the path left and
  re-entered the field) is skipped, not raised.

### Sub-fields and shape coverage

- **`vertical_slice(start_frac, end_frac) -> CellField`** — a new, smaller
  field containing only the rows in the real-world band from `start_frac`
  to `end_frac` of this field's height (e.g. `(0.0, 0.25)` for the bottom
  quarter, `(0.75, 1.0)` for the top quarter). Width/buffer are unchanged;
  corners are recomputed so the result's own `(0,0)` lines up correctly.
  Cheap even for a tiny slice of a huge field — extracting rows is a
  byte-slice of the source, not a conversion of the whole thing.
- **`vertical_slice_index(n, m) -> CellField`** — convenience for "the
  n-th (0-indexed) of m equal vertical slices", e.g. `vertical_slice_index(0, 4)`
  for the bottom quarter.
- **`cover_with_shape(shape, shape_center=None) -> [(x, y), ...]`** — finds
  placements of `shape` (a small `CellField` mask) whose union covers every
  on cell in this field, returning each placement's center in real-world
  coordinates. Geometric set cover is NP-hard in general, so this is the
  standard **greedy** heuristic (repeatedly place wherever it covers the
  most still-uncovered cells) — provably within a log factor of optimal,
  not optimal itself. `shape_center` (default: the shape's bounding-box
  center) is the point within the shape each returned coordinate refers to.
  Cost is roughly O(remaining&sup2; &times; shape area) — fine for hundreds
  of cells to cover, not built for huge target sets.

### Serialization

- **`to_bytes() -> bytes`** — the packed field as one `bytes` object.
- **`CellField.from_bytes(...)`** — see Construction above.

## Visualization

```python
from flight.pathfinding.cellField.visual import render_field
render_field(field, save_path="field.png", cell_pixels=16, title="...")
```

Off cells are black, on cells are white, with a thin gridline dividing every
cell. Omit `save_path` to show it interactively instead.

## Tests

`flight/pathfinding/tests/cellFieldTest.py` — run directly with
`python flight/pathfinding/tests/cellFieldTest.py`. Covers correctness for
every method above plus the performance properties called out here (e.g.
`apply_mask`/`vertical_slice` cost staying independent of the field's
overall size, `fill_disk` beating a per-cell `set()` loop).
