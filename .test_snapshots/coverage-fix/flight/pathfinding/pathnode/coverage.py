"""What has been photographed, what a stretch of route needs photographed, and where to put
photos to cover it.

Squares are CELL (2 ft) on a side, (col, row) from the field's bottom-left.
- `seen`: one bit per square, set when a photo fully covers it (record).
- need(poly): the squares that must be photographed for a stretch to count as checked --
  the ones it touches (corridor), plus every square a mine could endanger it from (band).
- Misses: a needed square a photo was aimed at but didn't cover (the footprint turned with
  the drone's heading, came out smaller than planned, or the camera failed). After
  MISS_CENTRE misses the square gets a photo centred on it; after MISS_BLOCK the planner
  blocks it (an obstacle, like a tiny mine zone), so the same photo is never handed out
  forever.
  - Aimed at = the squares the placement PLANNED that photo point for (remembered per point
    by places_along / tile), plus any square whose centre is within aim_radius of the photo.
    The plan is the source of truth: a planned square can sit farther from the point than
    aim_radius (a path-cover photo), and a photo that missed it must still be counted.
  - One attempt per waypoint: Scan may climb and re-photograph a waypoint several times;
    all of those together count as ONE attempt (only the waypoint's final footprint counts
    -- see Session.own_footprint / own_photo). Retries are not misses.
- Placement: photos along a path (places_along), or tiles over scattered squares (tile).
  A tile keeps its target square inside the footprint's inscribed circle, so it covers the
  square at any heading.
"""

import math

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.path_cover import path_cover_unseen
from flight.pathfinding.pathnode.config import (CACHE_CAP, DANGER_OFFSETS, FOOTPRINT_SLACK_FT,
                                                MISS_BLOCK, MISS_CENTRE, TILE_MARGIN_FT,
                                                CoverageMode)
from flight.pathfinding.pathnode.geometry import CELL
from flight.pathfinding.pathnode.polyline import arc_position, as_key


def cell_centre(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


class Coverage:
    def __init__(self, field_w, field_h, config):
        self.config = config
        cols = math.ceil(field_w / CELL - 1e-9)
        rows = math.ceil(field_h / CELL - 1e-9)
        self.seen = CellField(cols, rows, max_corner=(cols * CELL, rows * CELL))
        self._scratch = CellField(cols, rows, max_corner=(cols * CELL, rows * CELL))
        self.version = 0            # bumped by every photo (and every miss)
        self.misses = {}            # square -> photos aimed at it that didn't cover it
        self._touched = {}          # polyline -> frozenset of the squares it touches
        self._photos = {}           # (polyline, its unphotographed squares) -> photos needed
        self._aims = {}             # bucket -> {photo point: squares it was planned for}
        self._n_aims = 0

    # -- queries ---------------------------------------------------------------------------
    def in_field(self, c, r):
        return 0 <= c < self.seen.width and 0 <= r < self.seen.height

    def touched(self, poly):
        """Every square the polyline touches, traced in BOTH directions: CellField.mark_path
        breaks ties differently by direction for a segment exactly on a grid line, and joining
        two halves walks the partner's half backwards."""
        key = as_key(poly)
        cells = self._touched.get(key)
        if cells is None:
            if len(self._touched) > CACHE_CAP:
                self._touched.clear()
            self._scratch.clear_all()
            self._scratch.mark_path(poly)
            self._scratch.mark_path(poly[::-1])
            cells = frozenset(self._scratch.on_cells())
            self._touched[key] = cells
        return cells

    def need(self, poly):
        """Squares that must be photographed before this stretch counts as checked."""
        cells = self.touched(poly)
        if self.config.coverage == CoverageMode.CORRIDOR:
            return cells
        out = set()
        for c, r in cells:
            for dc, dr in DANGER_OFFSETS:
                if self.in_field(c + dc, r + dr):
                    out.add((c + dc, r + dr))
        return out

    def unseen(self, cells):
        return {c for c in cells if not self.seen.get(*c)}

    def all_seen(self, poly):
        return all(self.seen.get(c, r) for c, r in self.need(poly))

    def footprint_cells(self, x, y):
        """Squares fully inside the planning footprint centred at (x, y), axis-aligned."""
        along, across = self.config.shape_size_ft
        x0, x1, y0, y1 = x - across / 2, x + across / 2, y - along / 2, y + along / 2
        eps = 1e-9
        cols = range(math.ceil((x0 - eps) / CELL), math.floor((x1 + eps) / CELL))
        rows = range(math.ceil((y0 - eps) / CELL), math.floor((y1 + eps) / CELL))
        return {(c, r) for c in cols for r in rows}

    def cell_seen_at(self, x, y):
        """Whether the square containing (x, y) is photographed; None if it's off the field."""
        c, r = self.seen.real_to_cell(x, y)
        if not self.in_field(c, r):
            return None
        return self.seen.get(c, r)

    # -- photos in -------------------------------------------------------------------------
    def record(self, corners):
        """Mark every square fully inside a photo's footprint (any quadrilateral, corners in
        order). Grown by FOOTPRINT_SLACK_FT first: a lat/lon round trip can move an edge that
        sits exactly on a grid line by ~1e-4 ft and un-cover the square. Returns the
        footprint's centre."""
        pts = list(corners)
        cx = sum(x for x, _y in pts) / len(pts)
        cy = sum(y for _x, y in pts) / len(pts)
        grown = []
        for x, y in pts:
            d = math.hypot(x - cx, y - cy) or 1.0
            grown.append((x + (x - cx) / d * FOOTPRINT_SLACK_FT,
                          y + (y - cy) / d * FOOTPRINT_SLACK_FT))
        self.seen.fill_polygon_covered(grown)
        self.version += 1
        return cx, cy

    def _bucket(self, x, y):
        return (math.floor(x / CELL), math.floor(y / CELL))

    def _aim(self, p, targets):
        """Remember that the photo point p was planned for `targets` (latest plan wins)."""
        if self._n_aims > CACHE_CAP:
            self._aims.clear()
            self._n_aims = 0
        bucket = self._aims.setdefault(self._bucket(*p), {})
        self._n_aims += p not in bucket
        bucket[p] = frozenset(targets)

    def _aims_for(self, p):
        """The squares the photo point p was planned for (empty if it wasn't planned)."""
        return self._aims.get(self._bucket(*p), {}).get(p, frozenset())

    def aimed_at(self, x, y):
        """Squares a photo centred at (x, y) was aimed at: those the placement planned the
        nearby photo point (within aim_radius) for, plus every square whose centre is within
        aim_radius."""
        reach = self.aim_radius()
        b0, b1 = self._bucket(x, y)
        n = math.ceil(reach / CELL) + 1
        out = set()
        for i in range(b0 - n, b0 + n + 1):
            for j in range(b1 - n, b1 + n + 1):
                for p, targets in self._aims.get((i, j), {}).items():
                    if math.dist(p, (x, y)) <= reach + 1e-9:
                        out |= targets
        c0, r0 = self.seen.real_to_cell(x, y)
        for c in range(c0 - n, c0 + n + 1):
            for r in range(r0 - n, r0 + n + 1):
                if math.dist(cell_centre(c, r), (x, y)) <= reach + 1e-9:
                    out.add((c, r))
        return out

    def count_misses(self, x, y, needed):
        """A photo centred at (x, y) was taken (or failed). Count a miss for every square in
        `needed` it was aimed at (aimed_at) that is still unphotographed. Call once per
        waypoint, with its final photo. Returns the squares that have now reached MISS_BLOCK."""
        self.version += 1
        hopeless = []
        for c, r in sorted(self.aimed_at(x, y)):
            if (c, r) not in needed or self.seen.get(c, r):
                continue
            self.misses[(c, r)] = self.misses.get((c, r), 0) + 1
            if self.misses[(c, r)] >= MISS_BLOCK:
                hopeless.append((c, r))
        return hopeless

    def missed(self, cell):
        return self.misses.get(cell, 0) >= MISS_CENTRE

    # -- photos out ------------------------------------------------------------------------
    def tile_offset(self):
        """How far a tile may sit from its target square's centre and still enclose the square
        at any heading: the square's far corner stays TILE_MARGIN_FT inside the footprint's
        inscribed circle."""
        return max(min(self.config.shape_size_ft) / 2 - CELL * math.sqrt(2) / 2 - TILE_MARGIN_FT,
                   0.0)

    def aim_radius(self):
        """A photo centred within this of a square's centre was aimed at that square: a tile's
        largest offset, plus half a square of position error."""
        return self.tile_offset() + CELL / 2

    def places_along(self, poly, plan=True):
        """Photo centres covering every unphotographed needed square of `poly`, in walking order
        from its start. Any square no placement fully encloses (or that has been missed) gets a
        photo centred on it, so a batch always completes the stretch. plan=False: only asking
        (no photo point is remembered as planned)."""
        need = self.unseen(self.need(poly))
        if not need:
            return []
        key = lambda p: arc_position(poly, p)            # noqa: E731
        if self.config.coverage == CoverageMode.BAND:
            return self.tile(need, key, plan)
        placed, covered = [], set()
        missed = {c for c in need if self.missed(c)}
        for p in path_cover_unseen(poly, self.seen, self.config.shape_size_ft,
                                   overlap=self.config.overlap):
            got = (self.footprint_cells(*p) & need) - missed
            if got - covered:
                placed.append(p)
                if plan:
                    self._aim(p, got - covered)
                covered |= got
        for c, r in sorted(need - covered, key=lambda cr: key(cell_centre(*cr))):
            if (c, r) in covered:
                continue
            p = cell_centre(c, r)
            placed.append(p)
            if plan:
                self._aim(p, {(c, r)})
            covered |= self.footprint_cells(*p)
        return sorted(placed, key=key)

    def tile(self, need, key, plan=True):
        """Greedy tiling of scattered squares: the next needed square in `key` order, covered by
        the candidate footprint enclosing the most still-needed squares. Candidates keep the
        target inside the footprint's inscribed circle (tile_offset); a missed square gets
        only the centred one."""
        h = self.tile_offset()
        d = h / math.sqrt(2)
        offsets = [(0.0, 0.0), (h, 0.0), (-h, 0.0), (0.0, h), (0.0, -h),
                   (d, d), (d, -d), (-d, d), (-d, -d)]
        need = set(need)
        placed = []
        while need:
            c, r = min(need, key=lambda cr: key(cell_centre(*cr)))
            cx, cy = cell_centre(c, r)
            if self.missed((c, r)):
                cands = [(cx, cy)]
            else:
                cands = [(cx + dx, cy + dy) for dx, dy in offsets]
            best = max(cands, key=lambda p: (len(self.footprint_cells(*p) & need), key(p)))
            placed.append(best)
            if plan:
                self._aim(best, {(c, r)})
            need -= self.footprint_cells(*best) | {(c, r)}
        return placed

    def photos_needed(self, poly):
        """How many photos checking poly would take. Cached on the polyline and its
        still-unphotographed squares -- the placement depends on nothing else."""
        unseen = frozenset(self.unseen(self.need(poly)))
        if not unseen:
            return 0
        key = (as_key(poly), unseen)
        count = self._photos.get(key)
        if count is None:
            if len(self._photos) > CACHE_CAP:
                self._photos.clear()
            count = len(self.places_along(poly, plan=False))
            self._photos[key] = count
        return count
