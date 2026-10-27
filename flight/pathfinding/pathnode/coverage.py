"""What has been photographed. What a stretch needs and where photos go is the placement's job
(placement.py); this class holds the state it reads and answers through it.

Squares are CELL (2 ft) on a side, (col, row) from the field's bottom-left.
- `seen`: one bit per square, set when a photo fully covers it (record).
- `placement`: the PhotoPlacement for config.coverage. need / places_along / all_seen /
  photos_needed here are the stretch queries, answered by it against `seen`.
- Misses: a needed square a photo was aimed at but didn't cover (the footprint turned with
  the drone's heading, came out smaller than planned, or the camera failed). After
  MISS_CENTRE misses the square gets a photo centred on it; after MISS_BLOCK the planner
  blocks it (an obstacle, like a tiny mine zone), so the same photo is never handed out
  forever.
  - Aimed at = the squares the placement PLANNED that photo point for (remembered per point
    by the placement via plan_aim), plus any square whose centre is within aim_radius of the photo.
    The plan is the source of truth: a planned square can sit farther from the point than
    aim_radius (a path-cover photo), and a photo that missed it must still be counted.
  - One attempt per waypoint: Scan may climb and re-photograph a waypoint several times;
    all of those together count as ONE attempt (only the waypoint's final footprint counts
    -- see Session.own_footprint / own_photo). Retries are not misses.
- A tile (placement.py) keeps its target square inside the footprint's inscribed circle, so it
  covers the square at any heading; aim_radius is how far that lets a photo sit from its square.
"""

import math

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.pathnode.config import (CACHE_CAP, FOOTPRINT_SLACK_FT, MISS_BLOCK,
                                                MISS_CENTRE)
from flight.pathfinding.pathnode.geometry import CELL
from flight.pathfinding.pathnode.placement import cell_centre, placement_for
from flight.pathfinding.pathnode.polyline import as_key

AIM_CAP = 5000              # remembered photo points before the plan memory is dropped


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
        self.placement = placement_for(config.coverage, self)

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
        return self.placement.need(poly)

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

    def plan_aim(self, p, targets):
        """Remember that the photo point p was planned for `targets` (latest plan wins).
        Called by the placement."""
        if self._n_aims > AIM_CAP:
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
    def aim_radius(self):
        """A photo centred within this of a square's centre was aimed at that square: a tile's
        largest offset, plus half a square of position error."""
        return self.placement.tile_offset() + CELL / 2

    def places_along(self, poly, plan=True):
        """Photo centres in walking order covering the stretch (PhotoPlacement.places_along)."""
        return self.placement.places_along(poly, plan)

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
