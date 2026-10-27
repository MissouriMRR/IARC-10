from flight.pathfinding.common import DIRECTION_VECTOR, SAFETY_RADIUS_FT
from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle


class BlockMine(PolygonObstacle):
    """One mine: the convex hull of the side points of its unsafe squares (protoMine).

    - nodes: one per hull vertex, each with ONE direction (a real mine: a 4-node diamond)
    - origin: the mine's snapped square centre, fixed for good (mineHash uses it)
    - radius: current safety radius (original radius + growth)
    - growth: total distance the nodes have moved from their original positions
    - squares: (col, row) of the unsafe squares at discovery
    - grows: True (blocked zones do not)
    """

    grows = True

    def __init__(self, mineNodes, origin, radius=SAFETY_RADIUS_FT, squares=()):
        super().__init__([list(mineNodes)], origin)
        self.baseRadius = radius
        self.radius = radius
        self.growth = 0.0
        self.squares = list(squares)
        self._base = [(n.x, n.y) for n in self.rings[0]]  # the original diamond, kept for good

    def expand(self, distance):
        """Grow `distance` further: growTo(growth + distance)."""
        self.growTo(self.growth + distance)

    def growTo(self, distance):
        """Put every node `distance` from its ORIGINAL position, along its one direction.

        - absolute: the result does not depend on earlier growth, so shrinking works too
        - a diamond stays a diamond: nodes only move along the axes, never diagonally
        """
        ring = self.rings[0]
        for node, (x, y) in zip(ring, self._base):
            vx, vy = DIRECTION_VECTOR[node.direction]
            node.x = x + vx * distance
            node.y = y + vy * distance
        self.growth = distance
        self.radius = self.baseRadius + distance
        self._setRings([ring])
