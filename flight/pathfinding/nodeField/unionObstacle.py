from shapely.ops import unary_union

from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle


class unionObstacle(PolygonObstacle):
    """Merged obstacles, outlined by the exact union (interior holes ignored).

    - grows: True (its mines do; zones inside it do not)
    - obstacleList: the leaf obstacles (mines, zones) it is made of, never other unions
    """

    def __init__(self, obstacleList):
        leaves = []
        for o in obstacleList:
            leaves.extend(o.obstacleList if isinstance(o, unionObstacle) else [o])
        self.obstacleList = leaves
        super().__init__([], (leaves[0].origin), shape=self._union())

    grows = True

    def _union(self):
        return unary_union([o.shape for o in self.obstacleList])

    def mines(self):
        """The BlockMines inside (skips plain zones)."""
        return [o for o in self.obstacleList if hasattr(o, "squares")]

    def updateGeometry(self):
        """Re-derive the outline after a constituent changed shape."""
        self._setShape(self._union())

    def expand(self, distance):
        for o in self.obstacleList:
            if o.grows:
                o.expand(distance)
        self.updateGeometry()
