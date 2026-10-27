"""The work still to do: photos to take, in priority groups (config.Group).

  PATH          everything needed for one valid path: the forward gap and every pending span
  BIG_SHORTCUT  the big shortcut being tried -- ahead of ADJACENT, so squares beside a
                section about to be replaced aren't photographed for nothing
  ADJACENT      unphotographed squares within adjacent_ring squares of the checked route
  SHORTCUT      the ordinary shortcut being tried
  OUTER         the next layer out: beyond adjacent_ring, within outer_ring

Ring squares whose centres are inside a known obstacle are skipped. Each group is a list of
(key, sequence): the photos along one path in flying order, keyed by the path, or one
free-standing photo (key None). claims.py decides which drone takes which.

Each group is filled by a WorkSource (work_sources.py), which also documents the keys. Work is
the ordered list of sources plus the ring-band geometry and the caching; it adds no work of its
own.

Everything here is cached on Work.state_key -- the coverage, the obstacles, the chain and the
shortcut trials -- since the drones ask after every photo and most photos change little.
"""

from flight.pathfinding.pathnode.config import Group, Tier
from flight.pathfinding.pathnode.work_sources import PathSource, RingSource, ShortcutSource


def _copy(groups):
    return [[(key, list(seq)) for key, seq in g] for g in groups]


class Work:
    def __init__(self, chain, routes, coverage, shortcuts, config):
        self.chain, self.routes, self.coverage, self.shortcuts, self.config = (
            chain, routes, coverage, shortcuts, config)
        self._groups = None         # (state key, groups)
        self._bands = {}            # (inner, outer) -> (state key, squares)
        # Register every kind of work here; groups() lists the sources' jobs by Group. Sources
        # run in this order, and a later plan for the same photo point overwrites an earlier one
        # (Coverage.plan_aim), so the order is behaviour.
        self.sources = (
            [PathSource(chain, coverage),
             RingSource(Group.ADJACENT, self.adjacent, chain, coverage),
             RingSource(Group.OUTER, self.outer, chain, coverage)]
            + [ShortcutSource(tier, shortcuts, coverage) for tier in Tier])

    def state_key(self):
        """Everything the work depends on. Equal keys, equal work."""
        return (self.coverage.version, self.routes.version, self.chain.signature(),
                self.shortcuts.signature())

    # -- rings -----------------------------------------------------------------------------
    def route_cells(self):
        """Every square the checked route touches."""
        cells = set()
        for k in self.chain.links:
            if k.is_checked:
                cells.update(self.coverage.touched(k.poly))
        return cells

    def band(self, inner, outer):
        """Unphotographed squares further than `inner` but within `outer` squares (Chebyshev
        distance) of the checked route's squares, whose centres are outside every obstacle."""
        if outer <= inner:
            return set()
        key = self.state_key()
        cached = self._bands.get((inner, outer))
        if cached is not None and cached[0] == key:
            return set(cached[1])
        route = self.route_cells()

        def within(n):
            return {(c + dc, r + dr) for c, r in route
                    for dc in range(-n, n + 1) for dr in range(-n, n + 1)}

        band = within(outer) - within(inner) if inner > 0 else within(outer)
        cov = self.coverage
        out = {(c, r) for c, r in band
               if cov.in_field(c, r) and not cov.seen.get(c, r) and not self.routes.in_zone(c, r)}
        self._bands[(inner, outer)] = (key, frozenset(out))
        return out

    def adjacent(self):
        return self.band(0, max(self.config.adjacent_ring, 0))

    def outer(self):
        return self.band(max(self.config.adjacent_ring, 0), self.config.outer_ring or 0)

    # -- what's wanted ---------------------------------------------------------------------
    def needed_squares(self):
        """Every unphotographed square some source still wants."""
        need = set()
        for source in self.sources:
            need |= source.needs()
        return self.coverage.unseen(need)

    def groups(self):
        """The photos still to take, one list per Group (see the module doc)."""
        key = self.state_key()
        if self._groups is not None and self._groups[0] == key:
            return _copy(self._groups[1])
        out = {g: [] for g in Group}
        for source in self.sources:
            out[source.group].extend(source.entries())
        groups = [out[g] for g in Group]
        self._groups = (key, _copy(groups))
        return groups
