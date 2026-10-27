"""Producers of work: one WorkSource per kind of job, each filling one priority Group.

  WorkSource      the contract (abstract)
  PathSource      the path itself: the forward gap and every pending span     -> Group.PATH
  ShortcutSource  the trial of one shortcut Tier                              -> TIER_GROUP[tier]
  RingSource      free-standing photos in a band of squares beside the route  -> ADJACENT / OUTER

work.py holds the ordered list of sources and the caching. A new kind of work (say, snapshot
squares for expanding mines) is one WorkSource subclass here plus one line in Work.__init__.

Keys name a path by its geometry, not by an object id, so every drone's copy of the side's
planner gives the same job the same key (claims.py relies on it).
"""

from abc import ABC, abstractmethod

from flight.pathfinding.pathnode.config import TIER_GROUP, Group, LinkKind


class WorkSource(ABC):
    """One kind of work, filling one Group. Reads planner state, never changes it."""

    group: Group

    @abstractmethod
    def entries(self):
        """Returns this source's jobs as a list of (key, sequence): the photo centres in flying
        order, and the key naming the job (None for a free-standing photo). Order matters:
        claims.py breaks ties by position."""

    @abstractmethod
    def needs(self):
        """Returns the squares this source still wants photographed (may include squares
        already photographed; Work filters those)."""


class PathSource(WorkSource):
    """Everything needed for one valid path: the forward gap and every pending span, each
    photographed from its start."""
    group = Group.PATH

    def __init__(self, chain, coverage):
        self.chain, self.coverage = chain, coverage

    def entries(self):
        """Keys: "forward" for the forward gap, ("span", end) for a pending span by its far
        end node (its start moves as it is checked; its end doesn't)."""
        chain = self.chain
        return [
            ("forward" if k.kind == LinkKind.FORWARD else ("span", chain.nodes[i + 1].xy),
             self.coverage.places_along(k.poly))
            for i, k in enumerate(chain.links) if k.kind in (LinkKind.PENDING, LinkKind.FORWARD)]

    def needs(self):
        need = set()
        for k in self.chain.unchecked():
            need |= self.coverage.need(k.poly)
        return need


class ShortcutSource(WorkSource):
    """The shortcut being tried at one Tier (none: no work)."""

    def __init__(self, tier, shortcuts, coverage):
        self.tier, self.shortcuts, self.coverage = tier, shortcuts, coverage
        self.group = TIER_GROUP[tier]

    def entries(self):
        """Key: ("shortcut", a, b) by the trial's two anchors."""
        tr = self.shortcuts.trials[self.tier]
        if tr is None:
            return []
        return [(("shortcut", tr.a.xy, tr.b.xy), self.coverage.places_along(tr.poly))]

    def needs(self):
        tr = self.shortcuts.trials[self.tier]
        return set() if tr is None else set(self.coverage.need(tr.poly))


class RingSource(WorkSource):
    """Free-standing photos over a band of squares beside the checked route, one job per photo
    (key None), nearest the entry edge first."""

    def __init__(self, group, band, chain, coverage):
        """band: zero-argument callable returning the band's unphotographed squares."""
        self.group, self.band, self.chain, self.coverage = group, band, chain, coverage

    def entries(self):
        need = self.band()
        if not need:
            return []
        y0 = self.chain.nodes[0].y
        tiles = self.coverage.placement.tile(need, lambda p: (abs(p[1] - y0), p[0]))
        return [(None, [p]) for p in tiles]

    def needs(self):
        return self.band()
