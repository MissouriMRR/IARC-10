from typing import List, Tuple
from shapely.geometry import Polygon, LineString, Point
from flight.pathfinding.nodeField.node import Node
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as polyPlt
import math


def is_clockwise(points):
    signed_area = 0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        signed_area += (x1 * y2 - x2 * y1)
    return signed_area > 0


# ---------------------------------------------------------------------------
# Common tangents of two disjoint simple polygons, following the linear-time
# constant-workspace algorithm (Algorithm 2) of:
#   Mikkel Abrahamsen and Bartosz Walczak, "Common Tangents of Two Disjoint
#   Polygons in Linear Time and Constant Workspace", arXiv:1601.01816.
# This works on general (possibly non-convex) simple polygon corner lists, not
# just convex hulls, and replaces exteriorConnections/ObstacleExtremaWalker as
# the way connectPolygonObstacle finds bridge connections between obstacles.
# ---------------------------------------------------------------------------


def _det(a, b):
    return a[0] * b[1] - b[0] * a[1]


def _det_star3(a, b, c):
    return _det(a, b) + _det(b, c) + _det(c, a)


def _polygon_orientation_sign(vertices):
    n = len(vertices)
    if n < 3:
        # A single point (or a segment) has no orientation. The algorithm only
        # ever indexes such a degenerate side modulo its own length, so any
        # sign works here -- see the single-point specialization used by
        # PolygonObstacle.pointTangents.
        return 1
    total = 0.0
    for i in range(n):
        total += _det(vertices[i], vertices[(i + 1) % n])
    if total > 0:
        return 1
    if total < 0:
        return -1
    return 0


def _in_open_triangle(z, a, b, c):
    d1 = _det_star3(z, a, b)
    d2 = _det_star3(z, b, c)
    d3 = _det_star3(z, c, a)
    return (d1 > 0 and d2 > 0 and d3 > 0) or (d1 < 0 and d2 < 0 and d3 < 0)


def _find_common_tangent(verts0, verts1, alpha0, alpha1):
    """
    Algorithm 2 of arXiv:1601.01816. Finds indices (i0, i1) into verts0/verts1
    such that the line through verts0[i0] and verts1[i1] is the requested
    common tangent:
        alpha0=+1 -> verts0 lies in RHP(tangent); alpha0=-1 -> LHP
        alpha1=+1 -> verts1 lies in RHP(tangent); alpha1=-1 -> LHP
    (alpha0==alpha1 asks for an outer common tangent, alpha0!=alpha1 for a
    separating one). Returns None if no such tangent exists (nested hulls for
    the outer case, overlapping hulls for the separating case).
    """
    n0 = len(verts0)
    n1 = len(verts1)
    if n0 == 0 or n1 == 0:
        return None

    sign0 = _polygon_orientation_sign(verts0)
    sign1 = _polygon_orientation_sign(verts1)
    if sign0 == 0 or sign1 == 0:
        return None

    n = (n0, n1)
    beta = (alpha1 * sign0, -alpha0 * sign1)
    verts = (verts0, verts1)
    alpha = (alpha0, alpha1)

    def real_index(k, i):
        return (beta[k] * i) % n[k]

    def point(k, i):
        return verts[k][real_index(k, i)]

    s = [0, 0]
    v = [0, 0]
    blocked = [False, False]
    u = 0

    # The paper proves at most 6*(n0+n1) iterations; the cap below is only a
    # floating-point safety net (e.g. near-collinear corners) and is not part
    # of the algorithm itself.
    max_iterations = 6 * (n0 + n1) + 10
    iterations = 0

    while s[0] < 2 * n0 and s[1] < 2 * n1 and (v[0] < s[0] + n0 or v[1] < s[1] + n1):
        iterations += 1
        if iterations > max_iterations:
            return None

        v[u] += 1
        a = point(0, s[0])
        b = point(1, s[1])
        c = point(u, v[u])
        if alpha[u] * _det_star3(a, b, c) > 0 and not blocked[u]:
            other = 1 - u
            if _in_open_triangle(point(other, s[other]), point(u, s[u]), point(u, v[u] - 1), c):
                blocked[u] = True
            else:
                s[u] = v[u]
                v[other] = s[other]
                blocked[other] = False
        u = 1 - u

    if s[0] >= 2 * n0 or s[1] >= 2 * n1 or blocked[0] or blocked[1]:
        return None

    return (real_index(0, s[0]), real_index(1, s[1]))


def _wrap_angle(angle, pivot):
    return ((angle - pivot + math.pi) % (2 * math.pi)) - math.pi


def _find_point_tangent(viewpoint, verts, alpha, seed_index=0):
    """
    Single-sided specialization of _find_common_tangent for a 1-vertex
    "polygon" on side 0 (viewpoint), seeded from seed_index instead of always
    starting cold at 0 -- used to build per-vertex occlusion arcs quickly,
    seeding from whichever of commonTangents' up-to-4 touch points is
    available as a fast starting guess.

    Known limitation (found and validated during algorithm design): the point
    side can never trigger its own wrong-side test, so it can never clear the
    polygon side's `blocked` flag once set, and this can cause a false `None`
    on a specific edge case (a stuck-blocked-flag situation). Callers must
    treat a None return as "use _brute_force_arc for this side", not as
    "no tangent exists" -- unlike _find_common_tangent's None, which IS a
    reliable "no such tangent" signal for the general two-polygon case.
    """
    n1 = len(verts)
    if n1 == 0:
        return None
    sign1 = _polygon_orientation_sign(verts)
    if sign1 == 0:
        return None
    beta1 = -1 * sign1

    def real_index(i):
        return (beta1 * i) % n1

    def point(i):
        return verts[real_index(i)]

    s1 = (beta1 * seed_index) % n1
    v1 = s1
    blocked1 = False
    max_iterations = 6 * n1 + 10
    iterations = 0
    while s1 < 2 * n1 and v1 < s1 + n1:
        iterations += 1
        if iterations > max_iterations:
            return None
        v1 += 1
        b = point(s1)
        c = point(v1)
        if alpha * _det_star3(viewpoint, b, c) > 0 and not blocked1:
            if _in_open_triangle(viewpoint, point(s1), point(v1 - 1), c):
                blocked1 = True
            else:
                s1 = v1
                blocked1 = False
    if s1 >= 2 * n1 or blocked1:
        return None
    return real_index(s1)


def _brute_force_arc(viewpoint, verts, pivot_point):
    """O(len(verts)) fallback used only when _find_point_tangent returns None
    for either side of a vertex's arc (the stuck-blocked-flag edge case) --
    not a general-purpose replacement for the seeded fast path."""
    pivot = math.atan2(pivot_point[1] - viewpoint[1], pivot_point[0] - viewpoint[0])
    angles = [_wrap_angle(math.atan2(v[1] - viewpoint[1], v[0] - viewpoint[0]), pivot) for v in verts]
    return min(angles), max(angles)


class PolygonObstacle:
    def __init__(self, vertices,nodes:List[Node], wrapping:bool): #Assumes nodes are already connected
        self.isWrapping = True
        self.vertices = vertices

        if len(self.vertices) < 2:
            return

        self.xMin = min(v[0] for v in self.vertices)
        self.xMax = max(v[0] for v in self.vertices)
        self.yMin = min(v[1] for v in self.vertices)
        self.yMax = max(v[1] for v in self.vertices)

        self.nodes = nodes


        self.polygon = Polygon(self.vertices)

    # A vertex's index in self.vertices is not guaranteed to line up with its
    # node's index in self.nodes (e.g. hull-derived or merged/union vertex
    # lists), so look the node up by matching coordinates instead of assuming
    # the indices correspond.
    def _nodeAtVertexIndex(self, index: int) -> "Node | None":
        x, y = self.vertices[index]
        for node in self.nodes:
            if node.x == x and node.y == y:
                return node
        return None

    # Finds up to four common tangents (two outer, two separating) between this
    # obstacle and other_obstacle via the linear-time constant-workspace algorithm
    # of Abrahamsen & Walczak (arXiv:1601.01816, Algorithm 2), returning the
    # (self, other) Node pairs each tangent connects.
    def commonTangents(self, other_obstacle: 'PolygonObstacle') -> List[Tuple[Node, Node]]:
        connections = []
        for alpha0, alpha1 in ((1, 1), (-1, -1), (1, -1), (-1, 1)):
            result = _find_common_tangent(self.vertices, other_obstacle.vertices, alpha0, alpha1)
            if result is not None:
                selfIndex, otherIndex = result
                selfNode = self._nodeAtVertexIndex(selfIndex)
                otherNode = other_obstacle._nodeAtVertexIndex(otherIndex)
                if selfNode is not None and otherNode is not None:
                    connections.append((selfNode, otherNode))
        return connections

    # Returns the (self, other) Node pairs that should be connected.
    # commonTangents already returns [] when no tangent exists, so no
    # separate existence-check gate is needed here.
    def connectPolygonObstacle(self, other_obstacle: 'PolygonObstacle'):
        return self.commonTangents(other_obstacle)
    
    # Finds the (up to two) tangent nodes of this obstacle as seen from an
    # external point, using the same common-tangent algorithm (arXiv:1601.01816)
    # specialized to a single-point "polygon" on one side -- see the note in
    # _polygon_orientation_sign. Replaces the FloatingExtremaWalker heuristic.
    def pointTangents(self, point: Tuple[float, float]) -> List[Node]:
        tangents = []
        for alpha in (1, -1):
            result = _find_common_tangent([point], self.vertices, 1, alpha)
            if result is not None:
                _, obstacleIndex = result
                node = self._nodeAtVertexIndex(obstacleIndex)
                if node is not None:
                    tangents.append(node)
        return tangents

    #Returns the connections the floating node should make
    def connectFloatingNode(self,floating_node:Node):
        return self.pointTangents((floating_node.x, floating_node.y))



    def obstacleOverlap(self,otherObstacle):
        return  self.polygon.intersects(otherObstacle.polygon)
    @staticmethod
    def midPoint(vertex1, vertex2):
        return ((vertex1[0] + vertex2[0]) / 2, (vertex1[1] + vertex2[1]) / 2)

    def contains_point(self, point):
        return self.polygon.contains(Point(point))

    def intersects(self, line):
        line = LineString(line)
        # crosses() alone misses a segment that lies entirely inside this
        # obstacle without touching its boundary (both endpoints inside a
        # dense/concave obstacle after a batch expand+merge, in particular).
        return self.polygon.crosses(line) or self.polygon.contains(line)
