"""Shortcuts: replacing a checked section of the chain with a shorter path around the known
obstacles.

- Anchors: the entry, every jump node (earlier point As), every path node on an obstacle
  corner (where every detour bends) and, once met, the target (the final connection node).
- For a pair of anchors -- never the entry and the target together, which would be the whole
  path -- whose chain between them is fully checked: the shortest path between them. It is
  worth trying if it is
    - big: shorter than big_shortcut_ratio x the section and saving >= big_shortcut_saving_ft;
    - ordinary: shorter than shortcut_ratio x the section and saving >= SHORTCUT_MIN_SAVING_FT;
  and the photos it needs (only unphotographed squares count) are within its allowance:
  shortcut_max_waypoints, or one per shortcut_ft_per_photo it saves if that is more.
- One that needs no photos at all is spliced in at once, biggest saving first.
- Otherwise one of each tier is tried at a time (Trial), the biggest saving in its tier. The
  chain is untouched while it is tried; the section is replaced only once all of the
  shortcut is photographed and clear of every known obstacle.
- Greedy: after every change the trial is recomputed. A big one that no longer qualifies as big
  drops back to the ordinary tier; once it no longer qualifies as ordinary, or needs more
  photos than its allowance, that pair is given up for good.
"""

import itertools
import math
from dataclasses import dataclass

from flight.pathfinding.pathnode.config import ROUTE_MATCH_FT, Placer, Tier
from flight.pathfinding.pathnode.polyline import poly_len


@dataclass(eq=False)
class Trial:
    """A shortcut being tried: replace the chain between anchor nodes a and b with poly."""
    a: object
    b: object
    poly: list
    section: float      # the length of the chain it would replace
    uid: int


class Shortcuts:
    def __init__(self, chain, routes, coverage, config, log):
        self.chain, self.routes, self.coverage, self.config, self.log = (
            chain, routes, coverage, config, log)
        self.trials = {tier: None for tier in Tier}
        self.given_up = set()               # (anchor node, anchor node) never to try again
        self._cache = {}                    # tier -> (key, candidates)
        self._uid = itertools.count()

    @property
    def enabled(self):
        return bool(self.config.shortcut_ratio)

    def signature(self):
        return tuple(None if t is None else (t.uid, tuple(map(tuple, t.poly)))
                     for t in self.trials.values())

    # -- candidates ------------------------------------------------------------------------
    def anchors(self, met):
        """Indices of the anchor nodes (see the module doc)."""
        nodes = self.chain.nodes
        last = len(nodes) - 1
        corners = {(round(x, 6), round(y, 6)) for x, y in self.routes.corners()}
        return [i for i, n in enumerate(nodes)
                if n.placed_by in (Placer.ENTRY, Placer.JUMP) or (i == last and met)
                or (round(n.x, 6), round(n.y, 6)) in corners]

    def route(self, a, b, section, tier):
        """The shortest path a -> b if it is short enough to be a shortcut of this tier.
        Doesn't check the photo allowance."""
        ratio = self.config.tier_ratio(tier)
        if ratio is None:
            return None
        if math.dist(a.xy, b.xy) <= ROUTE_MATCH_FT:
            poly = [a.xy, b.xy]          # the section is a loop back to where it started
        else:
            found = self.routes.shortest(a.xy, b.xy)
            if found is None:
                return None
            poly = [a.xy] + list(found[0][1:-1]) + [b.xy]
        length = poly_len(poly)
        if length >= ratio * section or section - length < self.config.tier_min_saving(tier):
            return None
        return poly

    def allowance(self, section, poly):
        """How many photos a shortcut may take (see the module doc)."""
        saving = section - poly_len(poly)
        return max(self.config.shortcut_max_waypoints, saving / self.config.shortcut_ft_per_photo)

    def candidates(self, tier, met):
        """Every anchor pair short enough to be a shortcut of this tier, biggest saving first,
        as [(saving, a, b, poly, section)]. Cached until the chain or the obstacles change."""
        ratio = self.config.tier_ratio(tier)
        if not self.enabled or not ratio:
            return []
        chain = self.chain
        anchors = self.anchors(met)
        sections = tuple(chain.section(i, j) for i, j in zip(anchors, anchors[1:]))
        # The anchor nodes themselves, not id(node): the key keeps them alive, so a deleted
        # node's id can't be reused by a new one and make a stale entry match.
        key = (self.routes.version, tuple(chain.nodes[i] for i in anchors),
               tuple(None if x is None else round(x, 6) for x in sections), len(self.given_up))
        cached = self._cache.get(tier)
        if cached is not None and cached[0] == key:
            return cached[1]
        found = []
        last = len(chain.nodes) - 1
        for x, i in enumerate(anchors):
            for j in anchors[x + 1:]:
                if i == 0 and j == last:
                    continue            # never the whole path
                a, b = chain.nodes[i], chain.nodes[j]
                section = chain.section(i, j)
                if section is None or (a, b) in self.given_up:
                    continue
                # No path is shorter than the straight line: skip pairs that can't qualify.
                if math.dist(a.xy, b.xy) >= ratio * section:
                    continue
                poly = self.route(a, b, section, tier)
                if poly is not None:
                    found.append((section - poly_len(poly), a, b, poly, section))
        found.sort(key=lambda c: -c[0])
        self._cache[tier] = (key, found)
        return found

    def best(self, tier, met):
        """This tier's pair with the biggest saving within its allowance, as
        (a, b, poly, section), or None. Skips a pair the other tier is trying."""
        busy = {(t.a, t.b) for t in self.trials.values() if t is not None}
        for _saving, a, b, poly, section in self.candidates(tier, met):
            if (a, b) not in busy and (self.coverage.photos_needed(poly)
                                       <= self.allowance(section, poly)):
                return a, b, poly, section
        return None

    def left_to_try(self, met):
        return any(self.trials[t] is not None or self.best(t, met) is not None for t in Tier)

    # -- changes ---------------------------------------------------------------------------
    def _splice(self, a, b, poly, section, tier):
        chain = self.chain
        chain.replace(chain.index(a), chain.index(b), poly, Placer.SHORTCUT)
        self.log.count("shortcuts")
        if tier == Tier.BIG:
            self.log.count("big_shortcuts")
        self.log.event("shortcut", tier.value, a.xy, b.xy, section, poly_len(poly))

    def _apply_free(self, met):
        """Splice in, biggest saving first, every shortcut that needs no photos at all."""
        while True:
            free = next(((a, b, poly, section) for _s, a, b, poly, section
                         in self.candidates(Tier.ORDINARY, met)
                         if self.coverage.all_seen(poly)), None)
            if free is None:
                return
            a, b = free[0], free[1]
            # Finishing the one a tier was trying (its last photo just came in) is that
            # tier's; anything else needed no photos at all.
            tier = next((t for t, tr in self.trials.items()
                         if tr is not None and tr.a is a and tr.b is b), None)
            if tier is not None:
                self.trials[tier] = None
            else:
                self.log.count("instant_shortcuts")
            self._splice(*free, tier or Tier.ORDINARY)

    def update(self, met):
        """Apply free shortcuts; carry each tier's trial forward -- drop it if its anchors or
        section changed, recompute it around the known obstacles (see the module doc), splice
        it in once fully checked; then start the next trial in each tier."""
        if not self.enabled:
            return
        self._apply_free(met)
        chain = self.chain
        for tier in Tier:
            tr = self.trials[tier]
            if tr is None:
                continue
            i, j = chain.index(tr.a), chain.index(tr.b)
            current = chain.section(i, j) if i is not None and j is not None and j > i else None
            if current is None or abs(current - tr.section) > 1e-6:
                self.trials[tier] = None              # the chain changed under it
                continue
            poly = self.route(tr.a, tr.b, tr.section, tier)
            if poly is not None and (self.coverage.photos_needed(poly)
                                     > self.allowance(tr.section, poly)):
                self._give_up(tier, tr.a, tr.b)
            elif poly is None:
                if tier == Tier.BIG and self.route(tr.a, tr.b, tr.section, Tier.ORDINARY):
                    self.trials[tier] = None
                    self.log.event("shortcut-demoted", tr.a.xy, tr.b.xy)
                else:
                    self._give_up(tier, tr.a, tr.b)
            elif self.coverage.all_seen(poly):
                self._splice(tr.a, tr.b, poly, tr.section, tier)
                self.trials[tier] = None
            else:
                tr.poly = poly
        for tier in Tier:
            if self.trials[tier] is None:
                found = self.best(tier, met)
                if found is not None:
                    a, b, poly, section = found
                    self.trials[tier] = Trial(a, b, poly, section, next(self._uid))

    def _give_up(self, tier, a, b):
        self.trials[tier] = None
        self.given_up.add((a, b))
        self.log.count("shortcuts_given_up")
        self.log.event("shortcut-given-up", a.xy, b.xy)
