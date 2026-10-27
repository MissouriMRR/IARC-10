"""The mine rule: repairing the chain when a new obstacle (a mine's zone, or a blocked square)
appears.

  0. If the entry is now inside it, the entry shifts along its edge to the nearest clear spot.
  1. Path nodes inside it are deleted (never the target); checked links crossing it are cut.
  2. Pending and forward links crossing it are replanned between their endpoints.
  3. Every fragment no longer connected to a root -- the entry or the target -- is an orphan
     and is deleted, whether or not it is inside the obstacle.
  4. The two root fragments are re-bridged by a shortest path: a pending span, or the forward
     gap if the far side is just the target. If no route leaves the entry's fragment at all
     (walled in), the entry moves along its edge to the nearest spot that has one.
  5. Jump: when the frontier was affected and this side's own drones found the mine
     (Source.PAIR), point A jumps to the mine's far corner and the detour behind it becomes a
     pending span. Mines relayed by the partner side, and blocked squares, only re-bridge.

Works on a PathNodePlanner (planner.py): its chain, routes, target and log.

Entry point: apply(pl, gid, source). It runs a MineRule, whose methods are the steps above in
order (_clear_entry, _delete_nodes_inside, _repair_links, _split_into_fragments, then either
_jump_past_mine or _rebridge). The per-link cases are _cut_checked, _replan_forward and
_replan_pending.
"""

from flight.pathfinding.pathnode.chain import Link, PathNode
from flight.pathfinding.pathnode.config import LinkKind, Placer, Source
from flight.pathfinding.pathnode.geometry import CELL


def apply(pl, gid, source):
    """Apply the rule for obstacle group gid, just added to pl.routes.

    Takes: pl (PathNodePlanner), gid (the new group), source (Source.PAIR or PARTNER).
    Does: repairs pl.chain in place; sets pl.failure when no route can be kept.
    """
    MineRule(pl, gid, source).run()


class MineRule:
    """One application of the rule: the repair of pl.chain for the new obstacle gid.

    State between steps:
    - broken: indices of links that are cut (checked links crossing it, unroutable spans,
      the link after a shifted entry).
    - dead: ids of nodes deleted because they are inside the obstacle.
    - fwd_route: the replanned forward route (points, gids), or None if the forward link
      was not touched.
    """

    def __init__(self, pl, gid, source):
        self.pl, self.gid, self.source = pl, gid, source
        self.chain, self.routes, self.log = pl.chain, pl.routes, pl.log
        self.broken = set()
        self.dead = set()
        self.fwd_route = None

    def run(self):
        """The steps in order (see the module docstring)."""
        if not self._clear_entry():
            return
        self._delete_nodes_inside()
        self._repair_links()
        f1, f2, frags = self._split_into_fragments()
        if f1 is f2:
            self._jump_past_mine()
        else:
            self._rebridge(f1, f2, frags)

    # -- step 0 ----------------------------------------------------------------------------
    def _clear_entry(self):
        """Shift the entry out of the obstacle if it is inside. False when it cannot be moved."""
        if self.routes.contains(*self.chain.nodes[0].xy):
            if not _shift_entry(self.pl):
                return False
            self.broken.add(0)
        return True

    # -- step 1 ----------------------------------------------------------------------------
    def _delete_nodes_inside(self):
        """Mark every path node inside the obstacle dead (never the target)."""
        chain = self.chain
        last = len(chain.nodes) - 1
        self.dead = {id(n) for i, n in enumerate(chain.nodes)
                     if i != last and self.routes.contains(n.x, n.y)}
        self.log.count("nodes_in_mine", len(self.dead))

    # -- steps 1-2: one named case per link kind -------------------------------------------
    def _repair_links(self):
        """Visit every surviving link that crosses the obstacle and repair it by kind."""
        chain = self.chain
        for i, link in enumerate(chain.links):
            a, b = chain.nodes[i], chain.nodes[i + 1]
            if i in self.broken or id(a) in self.dead or id(b) in self.dead:
                continue
            if not self.routes.crosses(link.poly):
                continue
            if link.is_checked:
                self._cut_checked(i)
            elif link.is_forward:
                self._replan_forward(i, link, a)
            else:
                self._replan_pending(i, link, a, b)

    def _cut_checked(self, i):
        """A photographed link crossing the obstacle is cut."""
        self.broken.add(i)

    def _replan_forward(self, i, link, a):
        """Replan the forward link from its start to the target; the target moves to the route's end."""
        route = self.pl.target.route_from(a.xy)
        if route[0] is None:
            self.broken.add(i)
            return
        link.poly = route[0]
        self.chain.nodes[i + 1] = PathNode(*route[0][-1], Placer.TARGET)
        self.fwd_route = route
        self.log.count("replans")

    def _replan_pending(self, i, link, a, b):
        """Replan a pending span between its two ends."""
        route = self.routes.route(a.xy, b.xy)
        if route[0] is None:
            self.broken.add(i)
            return
        link.poly = route[0]
        self.log.count("replans")

    # -- step 3 ----------------------------------------------------------------------------
    def _split_into_fragments(self):
        """Split the node indices at dead nodes and cut links.

        Returns: (f1, f2, frags): the fragment holding the entry, the one holding the target
        (the same list when the chain is still connected), and all fragments. The rest are
        orphans.
        """
        chain = self.chain
        frags, cur = [], []
        for i, n in enumerate(chain.nodes):
            if id(n) in self.dead:
                if cur:
                    frags.append(cur)
                    cur = []
                continue
            if cur and (i - 1) in self.broken:
                frags.append(cur)
                cur = []
            cur.append(i)
        if cur:
            frags.append(cur)
        last = len(chain.nodes) - 1
        f1 = next((f for f in frags if f[0] == 0), None)
        f2 = next((f for f in frags if f[-1] == last), None)
        return f1, f2, frags

    # -- step 5 (chain still connected) ----------------------------------------------------
    def _jump_past_mine(self):
        """Still one piece: only if the forward link was replanned around a mine this side
        found does point A jump past it."""
        chain = self.chain
        if self.fwd_route is not None and self.source == Source.PAIR:
            f = chain.forward_index()
            nodes, links = _frontier_links(self.pl, self.fwd_route, self.source, self.gid)
            chain.splice(f, nodes, links)
            chain.set_target(chain.links[-1].poly[-1])

    # -- step 4 (chain split) --------------------------------------------------------------
    def _rebridge(self, f1, f2, frags):
        """Delete orphans and join the entry's fragment to the target's by a shortest path."""
        pl, chain, routes, log = self.pl, self.chain, self.routes, self.log
        last = len(chain.nodes) - 1
        log.count("orphans", sum(len(f) for f in frags if f is not f1 and f is not f2))
        u = chain.nodes[f1[-1]]
        # If nothing but the target survives on the far side, the gap reaches the target: that
        # is the frontier -- the forward link was lost, or (after meeting) the checked join into
        # the target was cut, which re-opens the gap.
        frontier = f2 == [last]
        if frontier:
            route_from = pl.target.route_from
            where = "the target"
        else:
            v = chain.nodes[f2[0]]
            route_from = lambda p: routes.route(p, v.xy)       # noqa: E731
            where = f"({v.x:.1f},{v.y:.1f})"
        route = route_from(u.xy)
        if route[0] is None:
            # Walled in: start again from wherever along the entry edge has a way through.
            route = _reroute_from_entry(pl, route_from)
            if route[0] is None:
                pl.failure = f"no route from ({u.x:.1f},{u.y:.1f}) to {where}"
                return
            log.count("orphans", len(f1) - 1)
            f1 = [0]
        if frontier:
            mid_nodes, mid_links = _frontier_links(pl, route, self.source, self.gid)
            tail_nodes, tail_links = [PathNode(*route[0][-1], Placer.TARGET)], []
        else:
            mid_nodes, mid_links = [], [Link(LinkKind.PENDING, route[0])]
            tail_nodes = [chain.nodes[i] for i in f2]
            tail_links = [chain.links[i] for i in f2[:-1]]
            log.count("bridges")
        chain.nodes = [chain.nodes[i] for i in f1] + mid_nodes + tail_nodes
        chain.links = [chain.links[i] for i in f1[:-1]] + mid_links + tail_links


def _frontier_links(pl, route, source, gid):
    """The new frontier from route (points, gids). On a mine this side found, jump past it:
    point A moves to the last corner of that mine's group the route bends around."""
    pts, gids = route
    if source == Source.PAIR:
        ks = [k for k in range(1, len(pts) - 1) if gids[k] == gid]
        if ks:
            k = ks[-1]
            pl.log.count("jumps")
            pl.log.event("jump", pts[k])
            return ([PathNode(*pts[k], Placer.JUMP)],
                    [Link(LinkKind.PENDING, pts[:k + 1]), Link(LinkKind.FORWARD, pts[k:])])
    return [], [Link(LinkKind.FORWARD, pts)]


def _shift_entry(pl):
    """The entry is inside an obstacle: move it to the nearest clear spot along its edge."""
    old = pl.chain.nodes[0]
    xs = [(k + 0.5) * CELL for k in range(int(pl.field_w // CELL))]
    xs = [x for x in xs if not pl.routes.contains(x, old.y)]
    if not xs:
        pl.failure = "entry fully blocked"
        return False
    x = min(xs, key=lambda v: abs(v - old.x))
    pl.chain.nodes[0] = PathNode(x, old.y, Placer.ENTRY)
    pl.log.count("entry_shifts")
    pl.log.event("entry-shift", old.x, x)
    return True


def _reroute_from_entry(pl, route_from):
    """Move the entry to the nearest spot along its edge with a route onward, and return that
    route -- (None, None) if there is none. route_from: (x, y) -> (points, gids)."""
    old = pl.chain.nodes[0]
    xs = sorted(((k + 0.5) * CELL for k in range(int(pl.field_w // CELL))),
                key=lambda v: abs(v - old.x))
    for x in xs:
        if x == old.x or pl.routes.contains(x, old.y):
            continue
        route = route_from((x, old.y))
        if route[0] is not None:
            pl.chain.nodes[0] = PathNode(x, old.y, Placer.ENTRY)
            pl.log.count("entry_shifts")
            pl.log.event("entry-shift", old.x, x)
            return route
    return None, None
