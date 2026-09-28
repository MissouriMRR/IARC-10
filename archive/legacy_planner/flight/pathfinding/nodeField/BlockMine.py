from shapely.geometry import Polygon
from flight.pathfinding.common import nodeDirection
from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle


class BlockMine(PolygonObstacle):
    def __init__(self, mineNodes: list, origin: tuple = None):
        vertices = [(n.x, n.y) for n in mineNodes]
        super().__init__(vertices, mineNodes, True)
        # Globally-unique-per-drone identifier -- assigned by Field
        # (_selfConnect), None until then.
        self.id = None
        # The mine's position at first detection, in the field's shared
        # local coordinate frame -- fixed at construction and never touched
        # by expand(), so Field.mineHash (and this obstacle's own
        # obstacle_hash, see PolygonObstacle) stay stable across expansion
        # rather than drifting as the safety radius grows. Overrides
        # PolygonObstacle's own default (that polygon's plain centroid,
        # set moments ago by super().__init__() above) with the mine's
        # true discovery position when one is given -- falls back to that
        # same centroid default for a BlockMine built without going
        # through Field.addFromProtoMine (e.g. directly in tests).
        self.origin = (
            origin if origin is not None else (self.polygon.centroid.x, self.polygon.centroid.y)
        )

    def expand(self, distance):
        for node in self.nodes:
            if node.direction == nodeDirection.UP:
                node.y += distance
            elif node.direction == nodeDirection.DOWN:
                node.y -= distance
            elif node.direction == nodeDirection.LEFT:
                node.x -= distance
            elif node.direction == nodeDirection.RIGHT:
                node.x += distance
        self.vertices = [(n.x, n.y) for n in self.nodes]
        self.xMin = min(v[0] for v in self.vertices)
        self.xMax = max(v[0] for v in self.vertices)
        self.yMin = min(v[1] for v in self.vertices)
        self.yMax = max(v[1] for v in self.vertices)
        self.polygon = Polygon(self.vertices)
