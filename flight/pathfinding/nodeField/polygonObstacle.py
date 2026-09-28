"""PolygonObstacle: one obstacle's outline, its grown clearance shape, and its tangents.

Outline rings are counter-clockwise MineNode lists (a union pinched at a point has several).
The shortest-path graph is built on the rings pushed out VERTEX_INFLATE_FT (graphRings).

Tangents follow Abrahamsen and Walczak, "Common Tangents of Two Disjoint Polygons in Linear
Time and Constant Workspace", arXiv:1601.01816 (Algorithm 2). They work on non-convex simple
polygons, but return only the (up to) 4 GLOBAL tangents of a pair. See FieldConnections for
the extra local bitangents a non-convex outline needs.
"""

from shapely.geometry import MultiPolygon, Polygon
from shapely.geometry import Point

from flight.pathfinding.common import VERTEX_INFLATE_FT, VISIBILITY_MARGIN_FT
from flight.pathfinding.nodeField.node import outlineNodes


# -- Abrahamsen-Walczak tangent search ---------------------------------------------------
def _det(a, b):
    return a[0] * b[1] - b[0] * a[1]


def _det_star3(a, b, c):
    return _det(a, b) + _det(b, c) + _det(c, a)


def _orientation_sign(verts):
    if len(verts) < 3:
        return 1  # a single query point has no orientation; any sign works
    total = sum(_det(verts[i], verts[(i + 1) % len(verts)]) for i in range(len(verts)))
    return (total > 0) - (total < 0)


def _in_open_triangle(z, a, b, c):
    d1, d2, d3 = _det_star3(z, a, b), _det_star3(z, b, c), _det_star3(z, c, a)
    return (d1 > 0 and d2 > 0 and d3 > 0) or (d1 < 0 and d2 < 0 and d3 < 0)


def _find_common_tangent(verts0, verts1, alpha0, alpha1):
    """Indices (i0, i1) whose line is the tangent with verts0 on side alpha0 and verts1 on
    side alpha1 (+1 right, -1 left; equal = outer tangent, opposite = separating).
    None if there is none."""
    n = (len(verts0), len(verts1))
    if 0 in n:
        return None
    sign = (_orientation_sign(verts0), _orientation_sign(verts1))
    if 0 in sign:
        return None
    beta = (alpha1 * sign[0], -alpha0 * sign[1])
    verts = (verts0, verts1)
    alpha = (alpha0, alpha1)

    def real(k, i):
        return (beta[k] * i) % n[k]

    def point(k, i):
        return verts[k][real(k, i)]

    s, v, blocked, u = [0, 0], [0, 0], [False, False], 0
    cap = 6 * (n[0] + n[1]) + 10  # float-safety net; the paper proves this bound
    steps = 0
    while s[0] < 2 * n[0] and s[1] < 2 * n[1] and (v[0] < s[0] + n[0] or v[1] < s[1] + n[1]):
        steps += 1
        if steps > cap:
            return None
        v[u] += 1
        c = point(u, v[u])
        if alpha[u] * _det_star3(point(0, s[0]), point(1, s[1]), c) > 0 and not blocked[u]:
            other = 1 - u
            if _in_open_triangle(point(other, s[other]), point(u, s[u]), point(u, v[u] - 1), c):
                blocked[u] = True
            else:
                s[u] = v[u]
                v[other] = s[other]
                blocked[other] = False
        u = 1 - u
    if s[0] >= 2 * n[0] or s[1] >= 2 * n[1] or blocked[0] or blocked[1]:
        return None
    return real(0, s[0]), real(1, s[1])


class PolygonObstacle:
    """An obstacle with a hole-free outline.

    - rings: counter-clockwise MineNode lists
    - shape: shapely Polygon / MultiPolygon of the outline
    - grown: shape grown by VISIBILITY_MARGIN_FT (what clearance tests use)
    - id: set by Field; mergedInto: the obstacle this one was merged into (forward link)
    - grows: whether Field.expandField grows it (False: a blocked square stays as it is)
    """

    grows = False

    def __init__(self, rings, origin=None, shape=None):
        self.id = None
        self.mergedInto = None
        if shape is not None:
            self._setShape(shape)
        else:
            self._setRings(rings)
        self.origin = origin if origin is not None else (self.shape.centroid.x, self.shape.centroid.y)

    @classmethod
    def fromShape(cls, shape, origin=None):
        """Obstacle for a shapely Polygon / MultiPolygon (holes are ignored)."""
        return cls([], origin, shape)

    # -- geometry ------------------------------------------------------------------------
    def _setRings(self, rings):
        self.rings = rings
        parts = [Polygon([(n.x, n.y) for n in ring]) for ring in rings]
        for ring in rings:
            for node in ring:
                node.parentMine = self
        if parts:
            self.shape = parts[0] if len(parts) == 1 else MultiPolygon(parts)
            self._refresh()

    def _setShape(self, shape):
        parts = list(shape.geoms) if hasattr(shape, "geoms") else [shape]
        self._setRings([outlineNodes(p.exterior.coords, self) for p in parts if not p.is_empty])

    def _refresh(self):
        self.grown = self.shape.buffer(VISIBILITY_MARGIN_FT, join_style="mitre")
        self._graphRings = None
        self._pushed = None

    @property
    def pushed(self):
        """Outline pushed VERTEX_INFLATE_FT out: the region a query point may sit in that is
        inside the graph's corner ring yet still clear of the grown obstacle."""
        if self._pushed is None:
            self._pushed = self.shape.buffer(VERTEX_INFLATE_FT, join_style="mitre")
        return self._pushed

    @property
    def nodes(self):
        return [n for ring in self.rings for n in ring]

    @property
    def vertices(self):
        return [(n.x, n.y) for n in self.nodes]

    @property
    def polygon(self):
        return self.shape

    @property
    def graphRings(self):
        """Outline rings pushed VERTEX_INFLATE_FT outward (mitre): the graph's corners."""
        if self._graphRings is None:
            self._graphRings = []
            parts = list(self.shape.geoms) if hasattr(self.shape, "geoms") else [self.shape]
            for part in parts:
                pushed = part.buffer(VERTEX_INFLATE_FT, join_style="mitre")
                pushed = max(getattr(pushed, "geoms", [pushed]), key=lambda g: g.area)
                self._graphRings.append(outlineNodes(pushed.exterior.coords, self))
        return self._graphRings

    def expand(self, distance):
        """Grow the outline by distance (mitre offset). Field.expandField skips obstacles with
        grows False."""
        self._setShape(self.shape.buffer(distance, join_style="mitre"))

    def contains_point(self, point):
        return self.shape.contains(Point(point))

    def obstacleOverlap(self, other):
        """Whether this and other are within the margin of each other (they must merge)."""
        return self.grown.intersects(other.grown)

    # -- tangents (on the pushed rings) --------------------------------------------------
    def commonTangents(self, other):
        """(self node, other node) pairs of the common tangents of the two obstacles."""
        out = []
        for ring0 in self.graphRings:
            v0 = [(n.x, n.y) for n in ring0]
            for ring1 in other.graphRings:
                v1 = [(n.x, n.y) for n in ring1]
                for a0, a1 in ((1, 1), (-1, -1), (1, -1), (-1, 1)):
                    hit = _find_common_tangent(v0, v1, a0, a1)
                    if hit is not None:
                        out.append((ring0[hit[0]], ring1[hit[1]]))
        return out

    def pointTangents(self, point):
        """Nodes where the tangent lines from an outside point touch this obstacle."""
        out = []
        for ring in self.graphRings:
            verts = [(n.x, n.y) for n in ring]
            for alpha in (1, -1):
                hit = _find_common_tangent([tuple(point)], verts, 1, alpha)
                if hit is not None:
                    out.append(ring[hit[1]])
        return out
