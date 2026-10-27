"""One drone's copy of its side's planning: what it does with each thing that happens, and
what it must tell the partner side. Every drone of a side keeps its own copy of the side's
planner and applies the same events -- its own photos and its side-mate's -- so each can work
out the side's claims itself. The simulator (sim.py) and the flight code
(mission_planner.py -> the states) both drive a side through this, so the rules live in one
place.

Events in (local feet), in the order they happen:
- own_footprint(corners)     a photo this drone just took (may repeat while it climbs)
- own_photo(corners, mines)  ...then the mines found in it       -> Outbox
- mate_photo(corners, mines)  a photo report from the side's other drone -> Outbox
- failed_photo(x, y)         a planned photo that produced no footprint
- partner_mine(x, y), partner_coverage(corners), partner_point_a(x, y), partner_route(route)
Work out:
- assignments(positions)     who flies what (claims.py), keeping clear of the other side

An Outbox is what to relay to the partner side (if there is one): new mines, this photo's
footprint, and this side's point A. Relaying is the caller's job -- in flight, messages; in
the simulator, its bus.
"""

from dataclasses import dataclass, field

from flight.pathfinding.pathnode.config import Source


@dataclass
class Outbox:
    mines: list = field(default_factory=list)   # mines this side just learned, (x, y)
    coverage: list | None = None                # the photo's footprint corners
    point_a: tuple | None = None                # this side's point A, when it has a partner


class SideSession:
    def __init__(self, planner):
        self.planner = planner

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

    # -- this side's photos ----------------------------------------------------------------
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

    # -- the partner side ------------------------------------------------------------------
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
        return self.planner.check_join(route)

    # -- work out --------------------------------------------------------------------------
    def assignments(self, positions, held=None, me=None, others=None, lost=()):
        """drone id -> [(x, y, group), ...]: see PathNodePlanner.plan."""
        return self.planner.plan(positions, held, me, others, lost)

    def current_job(self, drone):
        """The job `drone` is working on per the last assignments, to announce to the
        side-mate: (group, key, (x, y)) or None."""
        return self.planner.claims.heads.get(drone)
