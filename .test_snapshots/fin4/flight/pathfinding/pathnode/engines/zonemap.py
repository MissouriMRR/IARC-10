"""ZoneMap: the default PathEngine. Mine zones as convex hulls plus a visibility graph.

Zone model:
- A discovered mine is snapped to the centre of the 2ft cell it falls in.
- Its danger zone is the convex hull of every cell whose square the safety circle touches
  (geometry.mine_zone) -- whole unsafe squares, never a partial one.
- Overlapping zones merge into one group. Each group has an id so a route can tell which
  mine it is wrapping around.

Shortest paths run on a visibility graph: nodes are the zone-group corners pushed
VERTEX_INFLATE_FT outward, edges are straight lines that stay clear of every zone by
VISIBILITY_MARGIN_FT. The margin is smaller than the inflation, so a route can hug a
corner, but two zones that only touch at a point still block the gap between them.

Growth (set_level): a mine's zone at level k is its hull Minkowski-summed with a diamond of
radius 2k ft (city-block growth; blocked squares do not grow). It grows the original octagonal
hull, whose axis tips sit at 5 ft from the square centre (Field's diamond tips sit at 6.05 ft).
"""

import heapq
import math

import numpy as np
import shapely
from shapely.geometry import MultiPoint, Point, box
from shapely.ops import unary_union

from flight.pathfinding.pathnode.engines.base import STEP_FT, PathEngine
from flight.pathfinding.pathnode.geometry import (
    CELL, FIELD_H, FIELD_W, VERTEX_INFLATE_FT, VISIBILITY_MARGIN_FT, mine_zone)


class ZoneMap(PathEngine):
    """Known (discovered) mine zones, merged into groups, plus the visibility graph.

    The original PathEngine (engines/base.py): zones are convex hulls, merged by shapely union.
    """

    def __init__(self, field_w=FIELD_W, field_h=FIELD_H):
        self.field_w, self.field_h = field_w, field_h
        self.groups = {}          # gid -> shapely geometry (Polygon or MultiPolygon)
        self._next_gid = 0
        self.blocker = None       # union of zones grown by the visibility margin
        self.vertices = np.zeros((0, 2))
        self.vertex_gid = []
        self._adj = []            # per vertex: list of (neighbour index, length)
        self._merged_into = {}    # gid -> the gid of the group it was merged into
        self._level = 0
        self._items = []          # (level-0 zone, grows) in the order added: set_level replays it
        self._members = {}        # gid -> item numbers in the group
        self._grown = {}          # gid -> group geometry grown by the visibility margin
        self._last_added = None

    # -- zones ---------------------------------------------------------------------
    def add_mine(self, x, y):
        """Add a mine, merging it with any group it overlaps or touches. Returns its gid."""
        return self.add_zone(mine_zone(x, y), grows=True)

    def add_square(self, c, r):
        """Block square (c, r). Returns its gid."""
        return self.add_zone(box(c * CELL, r * CELL, (c + 1) * CELL, (r + 1) * CELL))

    def add_zone(self, z, grows=False):
        """Add any zone polygon (z is its level-0 shape; convex when grows), merging it with any
        group it overlaps or touches. Returns its gid. self.last_added (set here) is the new zone
        grown by the margin: what just became off limits."""
        self._items.append((z, grows))
        gid = self._merge(self._zone_at(z, grows, self._level), len(self._items) - 1)
        self._rebuild()
        return gid

    def _merge(self, zone, idx):
        """Put zone (item number idx) into self.groups, merging touching groups. Returns its gid."""
        grown = zone.buffer(VISIBILITY_MARGIN_FT)
        merged = [gid for gid in self.groups if self._grown[gid].intersects(grown)]
        geom = unary_union([zone] + [self.groups.pop(g) for g in merged])
        gid = self._next_gid
        self._next_gid += 1
        members = [idx]
        for g in merged:
            self._merged_into[g] = gid
            members += self._members.pop(g)
            del self._grown[g]
        self.groups[gid] = geom
        self._members[gid] = members
        self._grown[gid] = geom.buffer(VISIBILITY_MARGIN_FT)
        self._last_added = grown
        return gid

    @staticmethod
    def _zone_at(z, grows, k):
        """z grown k steps: hull of its corners shifted STEP_FT * k along each axis (Minkowski
        sum with a diamond, exact for a convex z). A zone that does not grow is returned as is."""
        if not grows or k == 0:
            return z
        r = k * STEP_FT
        pts = [q for x, y in z.exterior.coords
               for q in ((x + r, y), (x - r, y), (x, y + r), (x, y - r))]
        return MultiPoint(pts).convex_hull

    @property
    def level(self):
        return self._level

    def set_level(self, k):
        """Rebuild every zone at growth level k from its level-0 shape (see base.py).

        - one visibility-graph rebuild, however many zones
        - a pre-change gid resolves to the new group holding its first item
        - last_added: the area that became off limits (empty when zones shrank)
        """
        if k < 0 or k != int(k):
            raise ValueError(f"level must be a non-negative integer, got {k!r}")
        k = int(k)
        if k == self._level:
            return
        self._level = k
        if not self._items:
            return
        old_blocker = self.blocker
        stale = {gid: min(m) for gid, m in self._members.items()}
        self.groups, self._members, self._grown = {}, {}, {}
        for idx, (z, grows) in enumerate(self._items):
            self._merge(self._zone_at(z, grows, k), idx)
        owner = {i: gid for gid, m in self._members.items() for i in m}
        for gid, first in stale.items():
            self._merged_into[gid] = owner[first]
        self._rebuild()
        self._last_added = self.blocker.difference(old_blocker)

    @property
    def last_added(self):
        """The newest zone grown by the margin: what just became off limits."""
        return self._last_added

    def obstacle_shapes(self):
        """{gid: geometry} of every live group."""
        return dict(self.groups)

    def corners(self):
        """Every zone corner the paths bend around, as (x, y)."""
        return [(float(x), float(y)) for x, y in self.vertices]

    def resolve(self, gid):
        """The group a (possibly since merged) gid belongs to now."""
        while gid in self._merged_into:
            gid = self._merged_into[gid]
        return gid

    def contains(self, x, y):
        """Inside some zone (grown by the margin) -- i.e. not a valid place for a node."""
        return self.blocker is not None and self.blocker.contains(Point(x, y))

    def crosses(self, poly):
        """Whether a polyline enters any zone (grown by the margin)."""
        if self.blocker is None or len(poly) < 2:
            return False
        return bool(shapely.intersects(shapely.linestrings(poly), self.blocker))

    # -- visibility graph ------------------------------------------------------------
    def _in_field(self, x, y):
        return -1e-9 <= x <= self.field_w + 1e-9 and -1e-9 <= y <= self.field_h + 1e-9

    def _rebuild(self):
        self.blocker = unary_union([g.buffer(VISIBILITY_MARGIN_FT) for g in self.groups.values()])
        shapely.prepare(self.blocker)
        verts, vgid = [], []
        for gid, g in self.groups.items():
            grown = g.buffer(VERTEX_INFLATE_FT, join_style="mitre")
            parts = grown.geoms if hasattr(grown, "geoms") else [grown]
            for part in parts:
                rings = [part.exterior] + list(part.interiors)
                for ring in rings:
                    for vx, vy in list(ring.coords)[:-1]:
                        if self._in_field(vx, vy) and not self.blocker.contains(Point(vx, vy)):
                            verts.append((vx, vy))
                            vgid.append(gid)
        self.vertices = np.array(verts, dtype=float).reshape(-1, 2)
        self.vertex_gid = vgid
        n = len(verts)
        self._adj = [[] for _ in range(n)]
        if n < 2:
            return
        ii, jj = np.triu_indices(n, 1)
        segs = np.stack([self.vertices[ii], self.vertices[jj]], axis=1)
        blocked = shapely.intersects(shapely.linestrings(segs), self.blocker)
        lengths = np.hypot(segs[:, 1, 0] - segs[:, 0, 0], segs[:, 1, 1] - segs[:, 0, 1])
        for a, b, blk, d in zip(ii, jj, blocked, lengths):
            if not blk:
                self._adj[a].append((int(b), float(d)))
                self._adj[b].append((int(a), float(d)))

    def _visible_from(self, p):
        """Mask of graph vertices with a clear straight line from p."""
        if len(self.vertices) == 0:
            return np.zeros(0, dtype=bool)
        if self.blocker is None:
            return np.ones(len(self.vertices), dtype=bool)
        segs = np.stack([np.broadcast_to(p, self.vertices.shape), self.vertices], axis=1)
        return ~shapely.intersects(shapely.linestrings(segs), self.blocker)

    def clear_line(self, p, q):
        return not self.crosses([p, q])

    def shortest_path(self, p, q):
        """
        Shortest obstacle-free polyline from p to q.

        Returns: (points, gids) -- gids[i] is the zone group whose corner points[i] is,
        None for the two endpoints. None if no route exists (or an endpoint is inside a
        zone).
        """
        p, q = (float(p[0]), float(p[1])), (float(q[0]), float(q[1]))
        if self.contains(*p) or self.contains(*q):
            return None
        if self.clear_line(p, q):
            return [p, q], [None, None]
        n = len(self.vertices)
        if n == 0:
            return None
        src, dst = n, n + 1
        vis_p, vis_q = self._visible_from(p), self._visible_from(q)
        coords = [(float(vx), float(vy)) for vx, vy in self.vertices] + [p, q]
        dist = {src: 0.0}
        prev = {}
        heap = [(0.0, src)]
        done = set()
        while heap:
            d, u = heapq.heappop(heap)
            if u in done:
                continue
            done.add(u)
            if u == dst:
                break
            if u == src:
                nbrs = [(i, math.dist(p, coords[i])) for i in np.flatnonzero(vis_p)]
            else:
                nbrs = list(self._adj[u])
                if vis_q[u]:
                    nbrs.append((dst, math.dist(coords[u], q)))
            for v, w in nbrs:
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        if dst not in dist:
            return None
        order = [dst]
        while order[-1] != src:
            order.append(prev[order[-1]])
        order.reverse()
        pts = [coords[i] for i in order]
        gids = [None] + [self.vertex_gid[i] for i in order[1:-1]] + [None]
        return pts, gids
