from shapely import unary_union
from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle
class unionObstacle(PolygonObstacle):
    def __init__(self,obstacleList,nodes,wrapped):
        self.obstacleList=obstacleList
        self.nodes=nodes
        
        self.combinedPolygon=unary_union([i.polygon for i in self.obstacleList])
        self.wrappedPolygon=self.combinedPolygon.convex_hull
        self.polygon=self.combinedPolygon.convex_hull
        self.vertices=self.polygon.exterior.coords
        super().__init__(self.vertices,nodes,wrapped)
    
    def updateGeometry(self):
        self.combinedPolygon=unary_union([i.polygon for i in self.obstacleList])

    def getGeomtery(self):
        return self.combinedPolygon

    # Merge-eligibility must be checked against the true (possibly concave)
    # combined shape, not self.polygon (the convex hull) -- otherwise a new
    # obstacle sitting entirely inside a concave notch, touching no real
    # material, would incorrectly be judged as overlapping and get merged in.
    def obstacleOverlap(self,otherObstacle):
        return self.combinedPolygon.intersects(otherObstacle.polygon)