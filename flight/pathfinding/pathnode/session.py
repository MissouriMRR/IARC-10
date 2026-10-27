"""PathNodeBackend: the path-node planner as a SearchBackend (the default backend).

One drone's copy of its side's planning: what it does with each thing that happens. Every
drone of a side keeps its own copy of the side's PathNodePlanner and applies the same events
-- its own photos and its side-mate's -- so each can work out the side's claims itself. The
simulator (sim.py) and the flight code (mission_planner.py -> the states) both drive a side
through the SearchBackend contract (flight/pathfinding/search_backend.py), so the rules live
in one place. This file is a thin layer: the work is in planner.py.

Per photo, in order: record the footprint, apply the mines, then advance() (lay path nodes
over the checked ground).

Beyond the contract:
- `planner`: the PathNodePlanner inside. Pathnode only -- tests, plots and sim metrics
  (nodes, links, claims, coverage) read it; a flight caller must not.
- `from_planner(planner)`: wrap a hand-built planner (tests and walkthrough demos).
- the level API (level, level_state, try_advance_level, report_route, ...): forwards to
  planner.levels (level_control.py).
"""

from flight.pathfinding.pathnode.config import Edge, PlannerConfig, Source
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.search_backend import Outbox, SearchBackend

__all__ = ["PathNodeBackend", "Outbox"]


class PathNodeBackend(SearchBackend):
    def __init__(self, field_w, field_h, entry_x, edge, side_drones, has_partner, **options):
        """
        Takes: the make_backend arguments; options are PlannerConfig fields (shape_size_ft,
        overlap, coverage, engine, ...). A partner side targets this side's entry x until the
        first point A arrives.
        """
        self.planner = PathNodePlanner(
            field_w, field_h, entry_x, Edge(edge),
            partner_entry_x=entry_x if has_partner else None, drones=tuple(side_drones),
            config=PlannerConfig(**options))

    @classmethod
    def from_planner(cls, planner):
        """A backend around an existing PathNodePlanner (tests, demos)."""
        self = cls.__new__(cls)
        self.planner = planner
        return self

    # -- who ------------------------------------------------------------------------------
    @property
    def drones(self):
        return self.planner.drones

    @property
    def has_partner(self):
        return self.planner.target.has_partner

    def _outbox(self, coverage, mines):
        out = Outbox(mines=mines, coverage=coverage)
        if self.has_partner:
            out.point_a = self.planner.point_a()
        return out

    def _add_pair_mines(self, mines):
        return [m for m in mines if self.planner.add_mine(*m, Source.PAIR)]

    # -- events: this side's photos -------------------------------------------------------
    def own_footprint(self, corners):
        self.planner.record_footprint(corners)

    def own_photo(self, corners, mines):
        """This drone's photo is in (own_footprint already recorded its footprint; corners is
        the last one, or None if there was none) and these mines were found in it."""
        new = self._add_pair_mines(mines)
        self.planner.advance()          # path nodes only after the photo's mines are in
        return self._outbox(corners if self.has_partner else None, new)

    def mate_photo(self, corners, mines):
        """A photo report from the side's other drone. Its mines were relayed to the partner
        side by the drone that found them; the Outbox is for completeness (both copies of the
        side's planner learn a mine, and the partner dedupes by square)."""
        self.planner.record_footprint(corners)
        new = self._add_pair_mines(mines)
        self.planner.advance()
        return self._outbox(None, new)

    def failed_photo(self, x, y):
        self.planner.record_failed_photo(x, y)

    # -- events: the partner side ---------------------------------------------------------
    def partner_mine(self, x, y):
        self.planner.add_mine(x, y, Source.PARTNER)
        self.planner.advance()

    def partner_coverage(self, corners):
        self.planner.record_footprint(corners)
        self.planner.advance()

    def partner_point_a(self, x, y):
        self.planner.set_partner_point_a(x, y)

    def partner_route(self, route):
        """The partner's latest route (it is met). Returns the joined route or None; the
        bottom side re-opens its gap if the two don't connect (planner.check_join)."""
        joined = self.planner.check_join(route)
        return None if joined is None or self.crosses(joined) else joined

    # -- work -----------------------------------------------------------------------------
    def assignments(self, positions, held=None, me=None, others=None, lost=()):
        """drone id -> [(x, y, group), ...]: see PathNodePlanner.plan."""
        return self.planner.plan(positions, held, me, others, lost)

    def current_job(self, drone):
        """(group, key, (x, y)) of the job `drone` is working on per the last assignments."""
        return self.planner.claims.heads.get(drone)

    # -- results --------------------------------------------------------------------------
    @property
    def failure(self):
        return self.planner.failure

    @property
    def met(self):
        return self.planner.met

    def done(self):
        return self.planner.done()

    def route_verified(self):
        return self.planner.route_verified()

    def point_a(self):
        return self.planner.point_a()

    def route(self):
        return self.planner.route()

    def join(self, partner_route):
        joined = self.planner.join(partner_route)
        return None if joined is None or self.crosses(joined) else joined

    def crosses(self, route):
        return self.planner.engine.crosses(route)

    def mines(self):
        return list(self.planner.discovered)

    def mine_hash(self):
        return self.planner.level_hash()

    def cell_seen_at(self, x, y):
        return self.planner.cell_seen_at(x, y)

    # -- expanding mines (the LevelController does the work: level_control.py) ---------------
    @property
    def level(self):
        return self.planner.level

    def level_state(self, me):
        return self.planner.levels.state(me)

    def hear_level_state(self, state):
        self.planner.levels.hear(state)

    def try_advance_level(self, me, now=0.0, lost=(), max_level=None):
        return self.planner.levels.try_advance(me, now, lost, max_level)

    def growth_fits(self, remaining):
        return self.planner.levels.growth_fits(remaining)

    def valid_snapshots(self):
        return self.planner.levels.valid_snapshots()

    def report_route(self, level=None):
        return self.planner.levels.report_route(level)

    def report_valid(self):
        return self.planner.levels.report_valid()

    def join_at(self, level, partner_route, partner_hash=None):
        return self.planner.levels.join_at(level, partner_route, partner_hash)

    def buffer_width(self):
        return self.planner.levels.buffer_width()

    def partner_lost(self):
        if self.has_partner:
            self.planner.partner_lost()

    def stats(self):
        return dict(self.planner.stats)
