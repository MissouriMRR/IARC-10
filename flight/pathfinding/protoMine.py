"""protoMine: the unsafe squares and outline of one mine, before it is added to a Field.

- squares: (col, row) of every CELL_FT square the safety circle touches (the same rule as
  pathnode.geometry.unsafe_cells)
- nodeVertices: the mine's nodes, made the original way:
    - for every unsafe square and each exposed side, a point at the CENTRE of the neighbouring
      square, nudged NUDGE_FT outward, tagged with that one side (LEFT/DOWN/RIGHT/UP)
    - the convex hull of those points; its (extreme) vertices, counter-clockwise, are the nodes
    - each node keeps the tag of the side it came from; BlockMine.expand moves it along that tag
- a real mine (3 ft radius, 2 ft squares) gives a DIAMOND: UP/LEFT/DOWN/RIGHT tips at 6.05 ft
  from the square centre (see test_diamond_geometry)
"""

import math

from shapely.geometry import box
from shapely.ops import unary_union

from flight.pathfinding.common import CELL_FT, SAFETY_RADIUS_FT, snap_to_cell_center
from flight.pathfinding.common import DIRECTION_VECTOR, nodeDirection
from flight.pathfinding.nodeField.node import MineNode

NUDGE_FT = 0.05  # how far a side point sits beyond the neighbouring square's centre

_SIDES = ((-1, 0, nodeDirection.LEFT), (0, -1, nodeDirection.DOWN),
          (1, 0, nodeDirection.RIGHT), (0, 1, nodeDirection.UP))


def convexHull(points):
    """Indices of the hull's extreme vertices, counter-clockwise (monotone chain).

    - collinear points are dropped, so every index is a true corner
    """
    order = sorted(range(len(points)), key=lambda i: points[i])

    def cross(o, a, b):
        return ((points[a][0] - points[o][0]) * (points[b][1] - points[o][1])
                - (points[a][1] - points[o][1]) * (points[b][0] - points[o][0]))

    chains = []
    for seq in (order, order[::-1]):
        chain = []
        for i in seq:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], i) <= 1e-9:
                chain.pop()
            chain.append(i)
        chains.append(chain[:-1])
    return chains[0] + chains[1]


class protoMine:
    def __init__(self, x, y, radius=SAFETY_RADIUS_FT):
        self.radius = radius
        self.origin = snap_to_cell_center(x, y)
        self.squares = self.generateBlocks()
        self.nodeVertices = self.generateNodes()

    @staticmethod
    def circle_rect_intersects(cx, cy, r, rx, ry, side=CELL_FT):
        """Whether the circle (cx, cy, r) touches the square with corner (rx, ry)."""
        px = min(max(cx, rx), rx + side)
        py = min(max(cy, ry), ry + side)
        return math.hypot(cx - px, cy - py) <= r

    def generateBlocks(self):
        """(col, row) of every square the circle touches."""
        cx, cy = self.origin
        col0, row0 = int(cx // CELL_FT), int(cy // CELL_FT)
        n = math.ceil(self.radius / CELL_FT)
        return [(col0 + dc, row0 + dr)
                for dc in range(-n, n + 1) for dr in range(-n, n + 1)
                if self.circle_rect_intersects(cx, cy, self.radius,
                                               (col0 + dc) * CELL_FT, (row0 + dr) * CELL_FT)]

    def outline(self):
        """Shapely polygon of the union of the unsafe squares (holes dropped)."""
        shape = unary_union([box(c * CELL_FT, r * CELL_FT, (c + 1) * CELL_FT, (r + 1) * CELL_FT)
                             for c, r in self.squares])
        return max(getattr(shape, "geoms", [shape]), key=lambda g: g.area)

    def sidePoints(self):
        """[(x, y, direction)]: one point per exposed side of every unsafe square."""
        unsafe = set(self.squares)
        out = []
        for c, r in self.squares:
            for dc, dr, d in _SIDES:
                if (c + dc, r + dr) not in unsafe:
                    vx, vy = DIRECTION_VECTOR[d]
                    out.append(((c + dc + 0.5) * CELL_FT + vx * NUDGE_FT,
                                (r + dr + 0.5) * CELL_FT + vy * NUDGE_FT, d))
        return out

    def generateNodes(self):
        """MineNodes on the hull of the side points, counter-clockwise, one direction each."""
        pts = self.sidePoints()
        return [MineNode(pts[i][0], pts[i][1], pts[i][2])
                for i in convexHull([(x, y) for x, y, _ in pts])]
