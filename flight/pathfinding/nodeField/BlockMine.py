from flight.pathfinding.common import DIRECTION_VECTOR, SAFETY_RADIUS_FT
from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle


class BlockMine(PolygonObstacle):
    """One mine: the convex hull of the side points of its unsafe squares (protoMine).

    - nodes: one per hull vertex, each with ONE direction (a real mine: a 4-node diamond)
    - origin: the mine's snapped square centre, fixed for good (mineHash uses it)
    - radius: current safety radius; expand() adds to it
    - squares: (col, row) of the unsafe squares at discovery
    - grows: True (blocked zones do not)
    """

    grows = True

    def __init__(self, mineNodes, origin, radius=SAFETY_RADIUS_FT, squares=()):
        super().__init__([list(mineNodes)], origin)
        self.radius = radius
        self.squares = list(squares)

    def expand(self, distance):
        """Move every node `distance` along its one direction.

        - a diamond stays a diamond: nodes only move along the axes, never diagonally
        """
        ring = self.rings[0]
        for node in ring:
            vx, vy = DIRECTION_VECTOR[node.direction]
            node.x += vx * distance
            node.y += vy * distance
        self.radius += distance
        self._setRings([ring])
