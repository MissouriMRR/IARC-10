"""Path-node planner simulation: two gamblers and two assistants on one field.

Structure
- self.nodes: ordered path nodes from the bottom entry to the top entry.
- self.links[i] joins nodes[i] -> nodes[i+1]; its poly runs between them, inclusive.
    "checked"  photographed ground (segment C)
    "pending"  a planned, unchecked span owned by one assistant (segment B)
    "forward"  the unchecked gap between the two gamblers' point As (one, until they meet)
  A path node is only ever joined to the node before and after it. Where the route bends
  around a mine, that bend vertex sits just outside the mine.

Rules
1. Gambler 1's point A is the forward link's start, gambler 2's its end. Each photographs
   along the forward link from its own end and lays path nodes over what it has checked,
   which advances its point A. The forward link is always the shortest route between the
   two point As, so the gamblers aim straight at each other.
2. On a mine discovery:
   a. path nodes inside the mine are deleted and checked links crossing it are cut;
   b. pending/forward links crossing it are replanned between their surviving endpoints;
   c. every fragment of the chain no longer connected to an entry -- the roots of valid
      segment C -- is an orphan and is deleted, whether or not it is inside the mine;
   d. the two root fragments are re-bridged by a shortest path. If the bridge replaces the
      frontier and a gambler made the discovery, that gambler jumps to a new point A at the
      mine's far corner and the detour becomes a pending span for its own assistant.
3. Assistants check only their own pending spans, lay path nodes the same way, and apply
   the same rule to their own discoveries -- a span they re-bridge stays theirs.
4. Done when the gamblers have met (no forward link) and no pending span is left.
"""

import math
from dataclasses import dataclass

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.path_cover import path_cover_unseen
from flight.pathfinding.pathnode.geometry import (CELL, FIELD_H, FIELD_W, ZoneMap, mine_zone,
                                                  unsafe_cells)

SHAPE_FT = (6.0, 4.0)   # (along = y, across = x) -- the same footprint as the old sim
OVERLAP = 0.1
SAMPLE_FT = 0.5         # resolution for deciding how far along a link is checked
ROLES = ("g1", "g2", "a1", "a2")
# Squares from which a mine could make square (0, 0) unsafe. The touch rule is symmetric, so
# this is the same cross pattern as one mine's own unsafe squares.
DANGER_OFFSETS = unsafe_cells(CELL / 2, CELL / 2)


@dataclass(eq=False)
class PathNode:
    x: float
    y: float
    placed_by: str      # "entry1"/"entry2", "g1"/"g2"/"a1"/"a2", or "A1"/"A2" for a jump
    side: int

    @property
    def xy(self):
        return (self.x, self.y)


@dataclass(eq=False)
class Link:
    kind: str           # "checked" | "pending" | "forward"
    poly: list
    owner: str = None   # "a1"/"a2" for pending
    walk_from_end: bool = False


def _dedupe(poly):
    out = [poly[0]]
    for p in poly[1:]:
        if math.dist(p, out[-1]) > 1e-9:
            out.append(p)
    return out


def _poly_len(poly):
    return sum(math.dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def _arc_position(poly, p):
    """Arc length along poly of the point on it nearest p."""
    best_d, best_t, acc = math.inf, 0.0, 0.0
    for i in range(len(poly) - 1):
        a, b = poly[i], poly[i + 1]
        seg = math.dist(a, b)
        if seg == 0:
            continue
        t = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / seg ** 2
        t = min(max(t, 0.0), 1.0)
        d = math.dist((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), p)
        if d < best_d:
            best_d, best_t = d, acc + t * seg
        acc += seg
    return best_t


def _samples(poly):
    """Points every SAMPLE_FT along poly, each tagged with the segment index it lies on.
    Every vertex is included (as the end of its segment)."""
    out = [(poly[0], 0)]
    for i in range(len(poly) - 1):
        a, b = poly[i], poly[i + 1]
        k = max(1, math.ceil(math.dist(a, b) / SAMPLE_FT))
        for j in range(1, k + 1):
            t = j / k
            out.append(((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), i))
    return out


class PathNodeMission:
    def __init__(self, true_mines, entry_x=(FIELD_W / 2, FIELD_W / 2), coverage="corridor",
                 discovery="square", record_frames=False, max_ticks=4000):
        """
        coverage:
        - "corridor" (default): photograph the squares the route passes through. The route
          may cross an unsafe square of a mine nobody has seen -- accepted for now.
        - "band": also photograph every square a mine could endanger the route from, so a
          checked route is provably clear. Costs ~2.5x makespan.
        discovery:
        - "square" (default): a mine is found only when its own square is fully
          photographed, so a mine is known iff its square has been seen.
        - "point": the old simulator's rule -- found when its position falls anywhere in
          the photo. Only for like-for-like comparison with the old planner.
        """
        if coverage not in ("corridor", "band"):
            raise ValueError(f"coverage must be 'corridor' or 'band', got {coverage!r}")
        if discovery not in ("square", "point"):
            raise ValueError(f"discovery must be 'square' or 'point', got {discovery!r}")
        self.coverage = coverage
        self.discovery = discovery
        self.true_mines = list(true_mines)
        self.zones = ZoneMap()
        w, h = int(FIELD_W // CELL), int(FIELD_H // CELL)
        self.seen = CellField(w, h, max_corner=(FIELD_W, FIELD_H))
        self._scratch = CellField(w, h, max_corner=(FIELD_W, FIELD_H))
        e1 = PathNode(entry_x[0], 0.0, "entry1", 1)
        e2 = PathNode(entry_x[1], FIELD_H, "entry2", 2)
        self.nodes = [e1, e2]
        self.links = [Link("forward", [e1.xy, e2.xy])]
        self.discovered = {}
        self.pos = {"g1": e1.xy, "a1": e1.xy, "g2": e2.xy, "a2": e2.xy}
        self.photos = {r: [] for r in ROLES}
        self.current = {"a1": None, "a2": None}
        self.tick = 0
        self.met_tick = None
        self.failure = None
        self.events = []
        self.stats = dict(jumps=0, nodes_in_mine=0, orphans=0, bridges=0, replans=0,
                          entry_shifts=0)
        self.record_frames = record_frames
        self.frames = []
        self.max_ticks = max_ticks

    # -- cells ---------------------------------------------------------------------
    def _touched(self, poly):
        """Cells a polyline touches (conservative -- every cell the line grazes)."""
        self._scratch.clear_all()
        self._scratch.mark_path(poly)
        return list(self._scratch.on_cells())

    def _need(self, poly):
        """Squares that must be photographed before this stretch counts as checked."""
        cells = self._touched(poly)
        if self.coverage == "corridor":
            return set(cells)
        w, h = self.seen.width, self.seen.height
        out = set()
        for c, r in cells:
            for dc, dr in DANGER_OFFSETS:
                cc, rr = c + dc, r + dr
                if 0 <= cc < w and 0 <= rr < h:
                    out.add((cc, rr))
        return out

    def _all_seen(self, poly):
        return all(self.seen.get(c, r) for c, r in self._need(poly))

    @staticmethod
    def _footprint_cells(x, y):
        """Cells fully inside the footprint centred at (x, y) -- what a photo marks seen."""
        along, across = SHAPE_FT
        x0, x1, y0, y1 = x - across / 2, x + across / 2, y - along / 2, y + along / 2
        eps = 1e-9
        cols = range(math.ceil((x0 - eps) / CELL), math.floor((x1 + eps) / CELL))
        rows = range(math.ceil((y0 - eps) / CELL), math.floor((y1 + eps) / CELL))
        return {(c, r) for c in cols for r in rows}

    # -- structure helpers -----------------------------------------------------------
    def _index(self, link):
        return next((i for i, x in enumerate(self.links) if x is link), None)

    def _forward_index(self):
        return next((i for i, x in enumerate(self.links) if x.kind == "forward"), None)

    def done(self):
        return self._forward_index() is None and not any(
            x.kind == "pending" for x in self.links)

    def route(self):
        pts = [self.nodes[0].xy]
        for x in self.links:
            pts.extend(x.poly[1:])
        return pts

    def _splice(self, i, new_nodes, new_links):
        """Replace links[i] (nodes[i] -> nodes[i+1]) with new_nodes and new_links."""
        assert len(new_links) == len(new_nodes) + 1
        self.nodes[i + 1:i + 1] = new_nodes
        self.links[i:i + 1] = new_links

    def _convert_checked(self, i, placer):
        """Whole link photographed -> segment C. Its bend vertices become path nodes."""
        poly = self.links[i].poly
        side = int(placer[1])
        self._splice(i, [PathNode(x, y, placer, side) for x, y in poly[1:-1]],
                     [Link("checked", [poly[k], poly[k + 1]]) for k in range(len(poly) - 1)])

    def _advance(self, i, from_end, placer):
        """Lay path nodes over the photographed part of links[i], from the chosen end.

        The same Link object stays as the unchecked remainder, so anyone working it keeps
        their reference. Returns True once the whole link is checked.
        """
        link = self.links[i]
        poly = link.poly[::-1] if from_end else link.poly
        samples = _samples(poly)
        good = 0
        for k in range(1, len(samples)):
            if not self._all_seen([samples[k - 1][0], samples[k][0]]):
                break
            good = k
        if good == 0:
            return False
        if good == len(samples) - 1:
            self._convert_checked(i, placer)
            return True
        end_pt, seg = samples[good]
        head = _dedupe(poly[:seg + 1] + [end_pt])
        tail = _dedupe([end_pt] + poly[seg + 1:])
        if len(head) < 2 or len(tail) < 2:
            return False
        side = int(placer[1])
        if not from_end:
            new_nodes = [PathNode(x, y, placer, side) for x, y in head[1:]]
            checked = [Link("checked", [head[k], head[k + 1]]) for k in range(len(head) - 1)]
            link.poly = tail
            self._splice(i, new_nodes, checked + [link])
        else:
            head, tail = head[::-1], tail[::-1]        # back to original orientation
            new_nodes = [PathNode(x, y, placer, side) for x, y in head[:-1]]
            checked = [Link("checked", [head[k], head[k + 1]]) for k in range(len(head) - 1)]
            link.poly = tail
            self._splice(i, new_nodes, [link] + checked)
        return False

    # -- photos ----------------------------------------------------------------------
    def _choose_photo(self, poly, from_end):
        need = {c for c in self._need(poly) if not self.seen.get(*c)}
        if not need:
            return None
        key = lambda p: _arc_position(poly, p)            # noqa: E731
        pick = max if from_end else min
        if self.coverage == "band":
            return self._tile_photo(need, key, pick)
        useful = [p for p in path_cover_unseen(poly, self.seen, SHAPE_FT, overlap=OVERLAP)
                  if self._footprint_cells(*p) & need]
        if useful:
            return pick(useful, key=key)
        # A placement can graze cells without fully enclosing any. A footprint centred on
        # a cell always encloses it, so fall back to the first unseen cell in walking order.
        centers = [((c + 0.5) * CELL, (r + 0.5) * CELL) for c, r in need]
        return pick(centers, key=key)

    def _tile_photo(self, need, key, pick):
        """Greedy band tiling: take the next unseen needed square in walking order and
        choose, among the grid-aligned footprints that fully enclose it, the one that
        covers the most needed squares (ties broken toward the walking direction)."""
        c, r = pick(need, key=lambda cr: key(((cr[0] + 0.5) * CELL, (cr[1] + 0.5) * CELL)))
        x0, y0 = c * CELL, r * CELL
        cands = [(x, y) for x in (x0, x0 + CELL) for y in (y0 - 1.0, y0 + 1.0, y0 + 3.0)]
        sign = 1 if pick is max else -1
        return max(cands, key=lambda p: (len(self._footprint_cells(*p) & need), sign * key(p)))

    def _photo(self, role, x, y):
        self.photos[role].append((x, y))
        self.pos[role] = (x, y)
        along, across = SHAPE_FT
        hx, hy = across / 2, along / 2
        self.seen.fill_polygon_covered([(x - hx, y - hy), (x + hx, y - hy),
                                        (x + hx, y + hy), (x - hx, y + hy)])
        # A mine is found only when the photo fully covers the mine's OWN square -- never
        # from an unsafe square around it, and never from a partial view of its square.
        # So a mine is known iff its square has been seen.
        covered = self._footprint_cells(x, y)
        if self.discovery == "square":
            found = [m for m in self.true_mines if m not in self.discovered
                     and (math.floor(m[0] / CELL), math.floor(m[1] / CELL)) in covered]
        else:
            found = [m for m in self.true_mines if m not in self.discovered
                     and abs(m[0] - x) <= hx and abs(m[1] - y) <= hy]
        for m in found:
            self.discovered[m] = self.tick
            self._on_mine(m, role)
            if self.failure:
                return

    # -- the mine rule ---------------------------------------------------------------
    def _shift_entry(self, end):
        """A mine over an entry: move that entry along its edge to the nearest clear cell."""
        old = self.nodes[end]
        xs = [(k + 0.5) * CELL for k in range(int(FIELD_W // CELL))]
        xs = [x for x in xs if not self.zones.contains(x, old.y)]
        if not xs:
            self.failure = f"entry {old.placed_by} fully blocked"
            return False
        x = min(xs, key=lambda v: abs(v - old.x))
        self.nodes[end] = PathNode(x, old.y, old.placed_by, old.side)
        self.stats["entry_shifts"] += 1
        self.events.append((self.tick, f"{old.placed_by} shifted {old.x:.0f}->{x:.0f}"))
        return True

    def _frontier_links(self, u, v, route, role, gid):
        """Bridge that replaces the frontier. The discovering gambler jumps past the mine."""
        pts, gids = route
        if role == "g1":
            ks = [k for k in range(1, len(pts) - 1) if gids[k] == gid]
            if ks:
                k = ks[-1]
                self.stats["jumps"] += 1
                self.events.append((self.tick, f"g1 jumps to ({pts[k][0]:.1f},{pts[k][1]:.1f})"))
                return ([PathNode(*pts[k], "A1", 1)],
                        [Link("pending", pts[:k + 1], "a1"), Link("forward", pts[k:])])
        elif role == "g2":
            ks = [k for k in range(1, len(pts) - 1) if gids[k] == gid]
            if ks:
                k = ks[0]
                self.stats["jumps"] += 1
                self.events.append((self.tick, f"g2 jumps to ({pts[k][0]:.1f},{pts[k][1]:.1f})"))
                return ([PathNode(*pts[k], "A2", 2)],
                        [Link("forward", pts[:k + 1]), Link("pending", pts[k:], "a2")])
        return [], [Link("forward", pts)]

    def _on_mine(self, mine, role):
        gid = self.zones.add_mine(*mine)
        broken = set()
        for end, link_i in ((0, 0), (len(self.nodes) - 1, len(self.links) - 1)):
            if self.zones.contains(*self.nodes[end].xy):
                if not self._shift_entry(end):
                    return
                broken.add(link_i)

        # a. nodes inside the mine
        dead = {id(n) for n in self.nodes if self.zones.contains(n.x, n.y)}
        self.stats["nodes_in_mine"] += len(dead)

        # a./b. links crossing it: cut if checked, replan if not
        fwd_before = self._forward_index()
        fwd_route = None
        for i, link in enumerate(self.links):
            a, b = self.nodes[i], self.nodes[i + 1]
            if i in broken or id(a) in dead or id(b) in dead:
                continue
            if not self.zones.crosses(link.poly):
                continue
            if link.kind == "checked":
                broken.add(i)
                continue
            route = self.zones.shortest_path(a.xy, b.xy)
            if route is None:
                broken.add(i)
                continue
            link.poly = route[0]
            self.stats["replans"] += 1
            if link.kind == "forward":
                fwd_route = route

        # c. fragments; only those rooted at an entry survive
        frags, cur = [], []
        for i, n in enumerate(self.nodes):
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
        last = len(self.nodes) - 1
        f1 = next((f for f in frags if f[0] == 0), None)
        f2 = next((f for f in frags if f[-1] == last), None)
        if f1 is None or f2 is None:
            self.failure = "an entry root was lost"
            return

        if f1 is f2:
            # Still one connected chain. If the forward link had to bend round this mine
            # and a gambler found it, that gambler jumps past it.
            if fwd_route is not None and role in ("g1", "g2"):
                f = self._forward_index()
                nodes, links = self._frontier_links(self.nodes[f], self.nodes[f + 1],
                                                    fwd_route, role, gid)
                self._splice(f, nodes, links)
            return

        orphans = sum(len(f) for f in frags if f is not f1 and f is not f2)
        self.stats["orphans"] += orphans
        u, v = self.nodes[f1[-1]], self.nodes[f2[0]]
        route = self.zones.shortest_path(u.xy, v.xy)
        if route is None:
            self.failure = f"no route between ({u.x:.1f},{u.y:.1f}) and ({v.x:.1f},{v.y:.1f})"
            return
        frontier = (fwd_before is not None and not fwd_before + 1 <= f1[-1]
                    and not fwd_before >= f2[0])
        if frontier:
            mid_nodes, mid_links = self._frontier_links(u, v, route, role, gid)
        else:
            owner = role if role in ("a1", "a2") else f"a{u.side}"
            mid_nodes, mid_links = [], [Link("pending", route[0], owner)]
            self.stats["bridges"] += 1
        self.events.append((self.tick, f"{role} mine at ({mine[0]:.0f},{mine[1]:.0f}): "
                                        f"{len(dead)} in mine, {orphans} orphaned, "
                                        f"{'frontier' if frontier else 'bridge'}"))
        self.nodes = [self.nodes[i] for i in f1] + mid_nodes + [self.nodes[i] for i in f2]
        self.links = ([self.links[i] for i in f1[:-1]] + mid_links
                      + [self.links[i] for i in f2[:-1]])

    # -- roles -----------------------------------------------------------------------
    def _gambler_step(self, role):
        f = self._forward_index()
        if f is None:
            return False
        from_end = role == "g2"
        if self._all_seen(self.links[f].poly):
            self._convert_checked(f, role)
            self.met_tick = self.met_tick or self.tick
            return False
        p = self._choose_photo(self.links[f].poly, from_end)
        self._photo(role, *p)
        if self.failure:
            return True
        f = self._forward_index()
        if f is not None and self._advance(f, from_end, role):
            self.met_tick = self.met_tick or self.tick
        return True

    def _pick_pending(self, role):
        mine = [x for x in self.links if x.kind == "pending" and x.owner == role]
        if not mine:
            return None
        here = self.pos[role]
        best = min(mine, key=lambda x: min(math.dist(here, x.poly[0]), math.dist(here, x.poly[-1])))
        best.walk_from_end = math.dist(here, best.poly[-1]) < math.dist(here, best.poly[0])
        return best

    def _assistant_step(self, role):
        while True:
            link = self.current[role]
            if link is None or self._index(link) is None or link.kind != "pending" \
                    or link.owner != role:
                link = self.current[role] = self._pick_pending(role)
                if link is None:
                    return False
            i = self._index(link)
            if self._all_seen(link.poly):
                self._convert_checked(i, role)
                self.current[role] = None
                continue
            break
        p = self._choose_photo(link.poly, link.walk_from_end)
        self._photo(role, *p)
        if self.failure:
            return True
        i = self._index(link)
        if i is not None and link.kind == "pending" and self._advance(i, link.walk_from_end, role):
            self.current[role] = None
        return True

    # -- run -------------------------------------------------------------------------
    def _snapshot(self):
        zones = []
        for g in self.zones.groups.values():
            for part in (g.geoms if hasattr(g, "geoms") else [g]):
                zones.append(list(part.exterior.coords))
        return {"tick": self.tick, "seen": self.seen.copy(), "zones": zones,
                "links": [(x.kind, x.owner, list(x.poly)) for x in self.links],
                "nodes": [(n.x, n.y, n.placed_by) for n in self.nodes],
                "photos": {r: list(v) for r, v in self.photos.items()},
                "discovered": set(self.discovered)}

    def run(self):
        if self.record_frames:
            self.frames.append(self._snapshot())
        while self.tick < self.max_ticks and not self.failure and not self.done():
            self.tick += 1
            acted = False
            for role in ROLES:
                if self.failure:
                    break
                step = self._gambler_step if role[0] == "g" else self._assistant_step
                acted = step(role) or acted
            if self.record_frames:
                self.frames.append(self._snapshot())
            if not acted and not self.done():
                self.failure = "stalled"
        return self

    # -- results ---------------------------------------------------------------------
    def distances(self):
        return {r: _poly_len(p) if len(p) > 1 else 0.0 for r, p in self.photos.items()}

    def summary(self):
        d = self.distances()
        return {"makespan": max(d.values()), "photos": sum(len(p) for p in self.photos.values()),
                "ticks": self.tick, "met_tick": self.met_tick, "failure": self.failure,
                "done": self.done(), "per_role": {r: (len(self.photos[r]), round(d[r]))
                                                 for r in ROLES}, **self.stats}


def evaluate(mission):
    """Independent checks on the finished route, for safety and completeness."""
    import shapely
    from shapely.ops import unary_union

    route = mission.route()
    ok_links = all(
        math.dist(link.poly[0], mission.nodes[i].xy) < 1e-6
        and math.dist(link.poly[-1], mission.nodes[i + 1].xy) < 1e-6
        for i, link in enumerate(mission.links))
    spans = abs(route[0][1]) < 1e-6 and abs(route[-1][1] - FIELD_H) < 1e-6
    segs = [shapely.LineString([route[k], route[k + 1]]) for k in range(len(route) - 1)
            if math.dist(route[k], route[k + 1]) > 1e-9]

    def crossings(mines):
        if not mines:
            return 0
        zone = unary_union([mine_zone(*m) for m in mines]).buffer(-1e-6)
        return int(sum(shapely.intersects(s, zone) for s in segs))

    hidden = [m for m in mission.true_mines if m not in mission.discovered]
    unseen = sum(1 for c, r in mission._touched(route) if not mission.seen.get(c, r))
    # The ground truth question: does any route square lie in ANY true mine's unsafe
    # squares -- found or not?
    route_cells = set(mission._touched(route))
    unsafe = set()
    for m in mission.true_mines:
        unsafe.update(unsafe_cells(*m))
    in_danger = len(route_cells & unsafe)
    return {"complete": mission.done() and mission.failure is None, "links_consistent": ok_links,
            "spans_field": spans, "bad_edges_known": crossings(list(mission.discovered)),
            "bad_edges_hidden": crossings(hidden), "unseen_cells": unseen,
            "route_cells_in_danger": in_danger,
            "pending_left": sum(1 for x in mission.links if x.kind != "checked")}
