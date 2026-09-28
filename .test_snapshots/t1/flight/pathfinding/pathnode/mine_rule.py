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
"""

from flight.pathfinding.pathnode.chain import Link, PathNode
from flight.pathfinding.pathnode.config import LinkKind, Placer, Source
from flight.pathfinding.pathnode.geometry import CELL


def apply(pl, gid, source):
    """Apply the rule for obstacle group gid, just added to pl.routes."""
    chain, routes, log = pl.chain, pl.routes, pl.log
    broken = set()
    if routes.contains(*chain.nodes[0].xy):
        if not _shift_entry(pl):
            return
        broken.add(0)

    last = len(chain.nodes) - 1
    dead = {id(n) for i, n in enumerate(chain.nodes)
            if i != last and routes.contains(n.x, n.y)}
    log.count("nodes_in_mine", len(dead))

    fwd_route = None
    for i, link in enumerate(chain.links):
        a, b = chain.nodes[i], chain.nodes[i + 1]
        if i in broken or id(a) in dead or id(b) in dead:
            continue
        if not routes.crosses(link.poly):
            continue
        if link.kind == LinkKind.CHECKED:
            broken.add(i)
            continue
        route = pl.target.route_from(a.xy) if link.kind == LinkKind.FORWARD else routes.route(a.xy, b.xy)
        if route[0] is None:
            broken.add(i)
            continue
        link.poly = route[0]
        if link.kind == LinkKind.FORWARD:
            chain.nodes[i + 1] = PathNode(*route[0][-1], Placer.TARGET)
            fwd_route = route
        log.count("replans")

    frags, cur = [], []
    for i, n in enumerate(chain.nodes):
        if id(n) in dead:
            if cur:
                frags.append(cur)
                cur = []
            continue
        if cur and (i - 1) in broken:
            frags.append(cur)
            cur = []
        cur.append(i)
    if cur:
        frags.append(cur)
    last = len(chain.nodes) - 1
    f1 = next((f for f in frags if f[0] == 0), None)
    f2 = next((f for f in frags if f[-1] == last), None)

    if f1 is f2:
        if fwd_route is not None and source == Source.PAIR:
            f = chain.forward_index()
            nodes, links = _frontier_links(pl, fwd_route, source, gid)
            chain.splice(f, nodes, links)
            chain.set_target(chain.links[-1].poly[-1])
        return

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
        mid_nodes, mid_links = _frontier_links(pl, route, source, gid)
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
