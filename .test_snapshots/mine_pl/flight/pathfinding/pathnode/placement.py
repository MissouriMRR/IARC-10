"""Photo placement: which squares a stretch of route needs photographed, and where to put the
photos that cover them.

  PhotoPlacement       the contract (abstract) plus the tiling of scattered squares
  CorridorPlacement    need = the squares the stretch touches; photos laid along it
  BandPlacement        need = every square a mine could endanger the stretch from; tiled
  placement_for(...)   the one place CoverageMode is turned into a class

Coverage (coverage.py) owns what HAS been photographed and asks its placement for the rest.
A placement reads Coverage (seen grid, footprints, misses) and writes only one thing back: the
aim memory (Coverage.plan_aim), so a photo that misses can be charged to the squares it was
planned for.

A new placement (say, a cell-based waypoint generator) is one subclass of PhotoPlacement plus
one line in PLACEMENTS. It must guarantee:
- need(poly) is a set of squares in the field, a function of the polyline alone;
- places_along(poly) returns photo centres in walking order such that taking the whole batch
  leaves no square of need(poly) unphotographed -- a square that has been missed
  (Coverage.missed) gets a photo centred on it, so the batch always completes the stretch;
- it is deterministic and reads only Coverage state, so both drones' copies of a side's planner
  (same photos in) give the same photos out -- claims.py relies on that;
- plan=False asks without touching the aim memory.
"""

import math
from abc import ABC, abstractmethod

from flight.pathfinding.path_cover import path_cover_unseen
from flight.pathfinding.pathnode.config import DANGER_OFFSETS, TILE_MARGIN_FT, CoverageMode
from flight.pathfinding.pathnode.geometry import CELL
from flight.pathfinding.pathnode.polyline import arc_position


def cell_centre(c, r):
    return ((c + 0.5) * CELL, (r + 0.5) * CELL)


class PhotoPlacement(ABC):
    """Decides what a stretch needs and where its photos go. Subclasses fill in `need` and
    `_place`; the rest is shared."""

    def __init__(self, coverage):
        self.cov = coverage         # the photographed state this placement reads

    # -- the contract ----------------------------------------------------------------------
    @abstractmethod
    def need(self, poly):
        """Takes a polyline. Returns the set of squares that must be photographed before the
        stretch counts as checked (seen or not)."""

    @abstractmethod
    def _place(self, poly, need, plan):
        """Takes a polyline, its still-unphotographed squares (non-empty) and whether to
        remember the aim. Returns photo centres in walking order that cover all of `need`."""

    def places_along(self, poly, plan=True):
        """Takes a polyline. Returns photo centres in walking order covering every
        unphotographed needed square (see the module doc for what a batch must guarantee).
        plan=False: only asking, so no photo point is remembered as planned."""
        need = self.cov.unseen(self.need(poly))
        if not need:
            return []
        return self._place(poly, need, plan)

    # -- shared: tiling scattered squares --------------------------------------------------
    def tile_offset(self):
        """How far a tile may sit from its target square's centre and still enclose the square
        at any heading: the square's far corner stays TILE_MARGIN_FT inside the footprint's
        inscribed circle."""
        return max(min(self.cov.config.shape_size_ft) / 2 - CELL * math.sqrt(2) / 2
                   - TILE_MARGIN_FT, 0.0)

    def tile(self, need, key, plan=True):
        """Greedy tiling of scattered squares: the next needed square in `key` order, covered by
        the candidate footprint enclosing the most still-needed squares. Candidates keep the
        target inside the footprint's inscribed circle (tile_offset); a missed square gets
        only the centred one."""
        cov = self.cov
        h = self.tile_offset()
        d = h / math.sqrt(2)
        offsets = [(0.0, 0.0), (h, 0.0), (-h, 0.0), (0.0, h), (0.0, -h),
                   (d, d), (d, -d), (-d, d), (-d, -d)]
        need = set(need)
        placed = []
        while need:
            c, r = min(need, key=lambda cr: key(cell_centre(*cr)))
            cx, cy = cell_centre(c, r)
            if cov.missed((c, r)):
                cands = [(cx, cy)]
            else:
                cands = [(cx + dx, cy + dy) for dx, dy in offsets]
            best = max(cands, key=lambda p: (len(cov.footprint_cells(*p) & need), key(p)))
            placed.append(best)
            if plan:
                cov.plan_aim(best, {(c, r)})
            need -= cov.footprint_cells(*best) | {(c, r)}
        return placed


class CorridorPlacement(PhotoPlacement):
    """Check only the squares the stretch touches, with photos laid along it."""

    def need(self, poly):
        return self.cov.touched(poly)

    def _place(self, poly, need, plan):
        """path_cover_unseen lays photos along the polyline; any square no placement fully
        encloses (or that has been missed) gets a photo centred on it."""
        cov = self.cov
        key = lambda p: arc_position(poly, p)            # noqa: E731
        placed, covered = [], set()
        missed = {c for c in need if cov.missed(c)}
        for p in path_cover_unseen(poly, cov.seen, cov.config.shape_size_ft,
                                   overlap=cov.config.overlap):
            got = (cov.footprint_cells(*p) & need) - missed
            if got - covered:
                placed.append(p)
                if plan:
                    cov.plan_aim(p, got - covered)
                covered |= got
        for c, r in sorted(need - covered, key=lambda cr: key(cell_centre(*cr))):
            if (c, r) in covered:
                continue
            p = cell_centre(c, r)
            placed.append(p)
            if plan:
                cov.plan_aim(p, {(c, r)})
            covered |= cov.footprint_cells(*p)
        return sorted(placed, key=key)


class BandPlacement(PhotoPlacement):
    """Check every square a mine could endanger the stretch from (the squares it touches plus
    DANGER_OFFSETS around each), tiled in walking order."""

    def need(self, poly):
        cov = self.cov
        out = set()
        for c, r in cov.touched(poly):
            for dc, dr in DANGER_OFFSETS:
                if cov.in_field(c + dc, r + dr):
                    out.add((c + dc, r + dr))
        return out

    def _place(self, poly, need, plan):
        return self.tile(need, lambda p: arc_position(poly, p), plan)


PLACEMENTS = {CoverageMode.CORRIDOR: CorridorPlacement, CoverageMode.BAND: BandPlacement}


def placement_for(mode, coverage):
    """Takes a CoverageMode and the Coverage it will read. Returns that mode's placement."""
    return PLACEMENTS[mode](coverage)
