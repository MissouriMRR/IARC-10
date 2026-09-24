"""Mine danger zones and obstacle-aware shortest paths.

Written from scratch for the path-node planner; no Field, Pathfinder or protoMine.

Zone model:
- A discovered mine is snapped to the centre of the 2ft cell it falls in (the same
  snapping the old planner did).
- Its danger zone is the convex hull of every cell whose square the safety circle
  (radius SAFETY_RADIUS_FT) touches -- i.e. whole unsafe squares, never a partial one.
- Overlapping zones merge into one group. Each group has an id so a route can tell which
  mine it is wrapping around.

Shortest paths run on a visibility graph: nodes are the zone-group corners pushed
VERTEX_INFLATE_FT outward, edges are straight lines that stay clear of every zone by
VISIBILITY_MARGIN_FT. The margin is smaller than the inflation, so a route can hug a
corner, but two zones that only touch at a point still block the gap between them.
"""

import heapq
import math

import numpy as np
import shapely
from shapely.geometry import Point, box
from shapely.ops import unary_union

FIELD_W = 80.0
FIELD_H = 300.0
CELL = 2.0
SAFETY_RADIUS_FT = 3.0
VERTEX_INFLATE_FT = 0.1
VISIBILITY_MARGIN_FT = 0.05


def snap_to_cell_center(x, y):
    """Centre of the 2ft cell containing (x, y)."""
    return (math.floor(x / CELL) + 0.5) * CELL, (math.floor(y / CELL) + 0.5) * CELL


def _circle_touches_square(cx, cy, r, sx, sy):
    """Whether a circle of radius r at (cx, cy) touches the CELLxCELL square at (sx, sy)."""
    px = min(max(cx, sx), sx + CELL)
    py = min(max(cy, sy), sy + CELL)
    return math.hypot(cx - px, cy - py) <= r


def unsafe_cells(x, y, radius=SAFETY_RADIUS_FT):
    """(col, row) of every cell whose square the mine's safety circle touches."""
    cx, cy = snap_to_cell_center(x, y)
    col0, row0 = int(cx // CELL), int(cy // CELL)
    n = math.ceil(radius / CELL)
    out = []
    for dc in range(-n, n + 1):
        for dr in range(-n, n + 1):
            col, row = col0 + dc, row0 + dr
            if _circle_touches_square(cx, cy, radius, col * CELL, row * CELL):
                out.append((col, row))
    return out


def mine_zone(x, y, radius=SAFETY_RADIUS_FT):
    """Convex hull of the mine's unsafe squares."""
    squares = [box(c * CELL, r * CELL, (c + 1) * CELL, (r + 1) * CELL)
               for c, r in unsafe_cells(x, y, radius)]
    return unary_union(squares).convex_hull


class ZoneMap:
    """Known (discovered) mine zones, merged into groups, plus the visibility graph."""

    def __init__(self, field_w=FIELD_W, field_h=FIELD_H):
        self.field_w, self.field_h = field_w, field_h
        self.groups = {}          # gid -> shapely geometry (Polygon or MultiPolygon)
        self._next_gid = 0
        self.blocker = None       # union of zones grown by the visibility margin
        self.vertices = np.zeros((0, 2))
        self.vertex_gid = []
        self._adj = []            # per vertex: list of (neighbour index, length)

    # -- zones ---------------------------------------------------------------------
    def add_mine(self, x, y):
        """Add a mine, merging it with any group it overlaps or touches. Returns its gid."""
        z = mine_zone(x, y)
        grown = z.buffer(VISIBILITY_MARGIN_FT)
        merged = [gid for gid, g in self.groups.items()
                  if g.buffer(VISIBILITY_MARGIN_FT).intersects(grown)]
        geom = unary_union([z] + [self.groups.pop(gid) for gid in merged])
        gid = self._next_gid
        self._next_gid += 1
        self.groups[gid] = geom
        self._rebuild()
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
