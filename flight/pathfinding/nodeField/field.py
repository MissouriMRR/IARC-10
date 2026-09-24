import time
import hashlib

import quads
from typing import Callable
import numpy as np
from matplotlib import pyplot
import random
import math
import bisect
import random

from shapely.geometry import Polygon, LineString, Point
from shapely.prepared import prep
from enum import Enum
from flight.pathfinding.nodeField.BlockMine import BlockMine
from flight.pathfinding.nodeField.node import Node
from flight.pathfinding.nodeField.polygonObstacle import (
    PolygonObstacle,
    _find_point_tangent,
    _brute_force_arc,
    _wrap_angle,
)
from flight.pathfinding.nodeField.unionObstacle import unionObstacle
from flight.pathfinding.protoMine import protoMine
from shapely import coverage_union, coverage_union_all, unary_union

SQUARE_SIZE = 2

# Buffer applied in mergePolygons when unary_union can't fuse touching polygons.
# Tiny: only turns a point/edge touch into a real overlap, doesn't change danger radius.
_MERGE_TOUCH_FUSE_EPSILON = 1e-6


# Field generates nodes off of mines, generates mines too
class Field:

    debugPoints = []  # purely for debuging and testing, field will plot these points

    def __init__(self, simFieldSizeFT: list, fieldCorners: list, droneNumber: int = 0):
        """
        Takes:
        - simFieldSizeFT: [width, height] in feet. Bottom-left is (0,0).
        - fieldCorners: the 4 corners in EXACTLY this order:
          [top-left, top-right, bottom-left, bottom-right].
          Any other order (e.g. walk-around-the-quad) builds the wrong edges,
          so withinField rejects the whole field.
          Reorder first if your source uses another order
          (e.g. SimToLatLonTransformer.get_arb_corners()); see Pathfinder for an example.
        - droneNumber: prefix for every node/mine id ("3-0", "3-1", ...),
          keeps ids unique across drones.
        """
        self.droneNumber = droneNumber
        self._nextId = 0

        simCorners = [
            (0, simFieldSizeFT[1]),
            (simFieldSizeFT[0], simFieldSizeFT[1]),
            (0, 0),
            (simFieldSizeFT[0], 0),
        ]
        self.rawCorners = fieldCorners
        self.boxes = [[], []]
        # For simulation bounded view
        self.simVertPairLeft = [simCorners[0], simCorners[2]]
        self.simVertPairRight = [simCorners[1], simCorners[3]]
        self.simHorzPairUpper = [simCorners[2], simCorners[3]]
        self.simHorzPairLower = [simCorners[0], simCorners[1]]
        # For field bounds
        self.fieldVertPairLeft = [self.rawCorners[0], self.rawCorners[2]]
        self.fieldVertPairRight = [self.rawCorners[1], self.rawCorners[3]]
        self.fieldHorzPairUpper = [self.rawCorners[0], self.rawCorners[1]]
        self.fieldHorzPairLower = [self.rawCorners[2], self.rawCorners[3]]

        self.xMin = min(simCorners[0][0], simCorners[2][0])
        self.xMax = max(simCorners[1][0], simCorners[3][0])
        self.yMin = min(simCorners[2][1], simCorners[3][1])
        self.yMax = min(simCorners[0][1], simCorners[1][1])

        # To be used for comparing if nodes are within the valid field
        self.leftLine, self.leftSlope = Field.getLine(
            self.fieldVertPairLeft[0], self.fieldVertPairLeft[1]
        )
        self.rightLine, self.rightSlope = Field.getLine(
            self.fieldVertPairRight[0], self.fieldVertPairRight[1]
        )
        self.upperLine, self.upperSlope = Field.getLine(
            self.fieldHorzPairUpper[0], self.fieldHorzPairUpper[1]
        )
        self.lowerLine, self.lowerSlope = Field.getLine(
            self.fieldHorzPairLower[0], self.fieldHorzPairLower[1]
        )

        self.floatingNodes = (
            []
        )  # List of floating nodes, necessary so we can connect floating nodes to mines, if mines are created afterwards.
        self.mines = []
        self.unionObstacles = []
        self.mineQuadTree = quads.QuadTree(
            (self.xMin + self.xMax / 2, self.yMin + self.yMax / 2), self.xMax, self.yMax
        )  # Used for collision detection, holds mines

        self.polygonObstacles = []
        self.fieldConnection = FieldConnections(self)

    # Next id: "<droneNumber>-<counter>". Only needs to be unique, carries no meaning.
    def _generateId(self) -> str:
        newId = f"{self.droneNumber}-{self._nextId}"
        self._nextId += 1
        return newId

    # Hash of all live mines (standalone + inside unions).
    # - Keyed on each mine's origin (position when first detected; see BlockMine.origin).
    # - Ignores ids and discovery order, so two drones that found the same mines match.
    # - Uses hashlib, not hash(): hash() is randomized per process.
    def mineHash(self, precision: int = 6) -> str:
        allMines = list(self.mines) + self._collect_mines(self.unionObstacles)
        positions = sorted(
            (round(mine.origin[0], precision), round(mine.origin[1], precision))
            for mine in allMines
        )
        payload = repr(positions).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    # Connect floating node fNode to obstacle's tangent points.
    # - obstacle.connectFloatingNode only knows about its own hull, so a tangent
    #   can still pass through some OTHER obstacle. Skip those.
    # - Shared by addFloatingNode, addObstacle, expandField.
    def _connectFloatingNodeToObstacle(self, obstacle, fNode: "Node") -> None:
        # Cheap bounding-box test first, exact .intersects() only if boxes overlap
        # (skipped ~79% of exact checks on a 70-mine field).
        others = [
            o for o in self.polygonObstacles + self.mines + self.unionObstacles if o is not obstacle
        ]
        for tangentNode in obstacle.connectFloatingNode(fNode):
            seg_min_x, seg_max_x = min(fNode.x, tangentNode.x), max(fNode.x, tangentNode.x)
            seg_min_y, seg_max_y = min(fNode.y, tangentNode.y), max(fNode.y, tangentNode.y)
            seg = ((fNode.x, fNode.y), (tangentNode.x, tangentNode.y))
            blocked = any(
                not (
                    seg_max_x < other.xMin
                    or seg_min_x > other.xMax
                    or seg_max_y < other.yMin
                    or seg_min_y > other.yMax
                )
                and other.intersects(seg)
                for other in others
            )
            if not blocked:
                self.fieldConnection.connectNode(fNode, tangentNode)

    # Connect two floating nodes directly if no obstacle is in the way.
    # - Needed for an empty field: with no obstacles nothing else creates
    #   connections, so start/end nodes stay isolated and Dijkstra KeyErrors.
    # - Same bounding-box-then-intersects check as above.
    def _connectFloatingNodeToFloatingNode(self, nodeA: "Node", nodeB: "Node") -> None:
        seg_min_x, seg_max_x = min(nodeA.x, nodeB.x), max(nodeA.x, nodeB.x)
        seg_min_y, seg_max_y = min(nodeA.y, nodeB.y), max(nodeA.y, nodeB.y)
        seg = ((nodeA.x, nodeA.y), (nodeB.x, nodeB.y))
        blocked = any(
            not (
                seg_max_x < other.xMin
                or seg_min_x > other.xMax
                or seg_max_y < other.yMin
                or seg_min_y > other.yMax
            )
            and other.intersects(seg)
            for other in self.polygonObstacles + self.mines + self.unionObstacles
        )
        if not blocked:
            self.fieldConnection.connectNode(nodeA, nodeB)

    def addFloatingNode(self, x: float, y: float, ndType: str = None) -> "Node":
        """
        Given a coordinate position, place a floating node onto the field
        """
        fNode = Node(x, y, True, nType=ndType, id=self._generateId())  # Floating Node
        for mine in self.mines:
            self._connectFloatingNodeToObstacle(mine, fNode)
        for obstacle in self.polygonObstacles:
            self._connectFloatingNodeToObstacle(obstacle, fNode)
        for unionObstacle in self.unionObstacles:
            self._connectFloatingNodeToObstacle(unionObstacle, fNode)
        for other in self.floatingNodes:
            self._connectFloatingNodeToFloatingNode(other, fNode)

        # Track every floating node: addObstacle uses this list to connect
        # obstacles added LATER back to nodes that already exist.
        self.floatingNodes.append(fNode)
        return fNode

    # Due to the current node stucture, right now this only modifies the nodeGraph
    def placeStartNode(self, xVal: float, yVal: float) -> "Node":
        """
        Given a coordinate, place a start node onto the field
        """
        return self.addFloatingNode(xVal, yVal, "start")

    # Places density amount of end nodes equidistance along the y coordinate and between xMin and xMax
    def placeEndNodesLine(self, yVal: float, density: int):
        """
        Given a y-value and density amount of nodes, places the end Nodes onto the field
        """
        returnList = []
        if density > 1:
            xVals = [
                self.xMin + (i * ((self.xMax - self.xMin) / density - 1)) for i in range(density)
            ]
            for x in xVals:
                returnList.append(self.addFloatingNode(x, yVal, "end"))
        else:
            returnList.append(self.addFloatingNode((self.xMin + self.xMax) / 2, yVal, "end"))
        return returnList

    def placeEndNodesPositions(self, position: list[tuple[float, float]]):
        """
        Given a list positions [(x,y)..]
        \nPlace end Nodes at those points
        """
        returnList = []
        for pos in position:
            returnList.append(self.addFloatingNode(pos[0], pos[1], "end"))
        return returnList

    # Returns which competition square a set of coordinates is in.
    def getSquareCoordinates(self, x: float, y: float) -> tuple[int, int]:
        """
        Given a set of coordinates, return which competition square it is in
        """
        if x < 0 or y < 0:
            raise ValueError("Coordinates must be positive")
        if x > self.xMax or y > self.yMax:
            raise ValueError("Coordinates must be within the field bounds")
        squareX = int(x // 2)
        squareY = int(y // 2)
        return (squareX, squareY)

    def withinField(self, x, y) -> bool:

        # Left line check
        if not (self.isPointRightofLine(self.leftLine, self.leftSlope, (x, y))):
            return False
        # Right line check
        elif not (self.isPointLeftofLine(self.rightLine, self.rightSlope, (x, y))):
            return False
        # Upper line check
        elif not (self.isPointBelowLine(self.upperLine, self.upperSlope, (x, y))):
            return False
        # Lower line check
        elif not (self.isPointAboveLine(self.lowerLine, self.lowerSlope, (x, y))):
            return False
        else:
            return True

    # Merge 2+ overlapping obstacles into one unionObstacle.
    # - Purges the constituents' node connections; the union gets its own nodes.
    # - Constituent nodes still exist but are unused.
    def mergePolygons(self, polygonObstacleList: list[PolygonObstacle]):
        polygonList = []
        for i in polygonObstacleList:
            for node in i.nodes:
                self.fieldConnection.purgeConnections(node)
            polygonList.append(i.polygon)

        unionPolygon = unary_union(polygonList)
        if unionPolygon.geom_type != "Polygon":
            # Polygons that only TOUCH (point/edge) count as overlapping upstream
            # (.intersects()), but unary_union returns a MultiPolygon for them,
            # which used to crash here. A tiny buffer makes them truly overlap
            # so they fuse into one Polygon.
            unionPolygon = unary_union(
                [p.buffer(_MERGE_TOUCH_FUSE_EPSILON) for p in polygonList]
            )
        if unionPolygon.geom_type != "Polygon":
            # Should be unreachable; fail loudly rather than mis-shape a danger zone.
            raise ValueError(
                f"mergePolygons: unary_union produced a {unionPolygon.geom_type},"
                " not a Polygon, even after the touch-fusing buffer"
            )
        nodes = []
        for i in unionPolygon.exterior.coords[:-1]:
            # print(i)
            nodes.append(Node(i[0], i[1], False))
        wrapped = True

        newObstacle = unionObstacle(polygonObstacleList, nodes, wrapped)
        for i in polygonObstacleList:
            i.mergedInto = newObstacle
        # self.unionObstacles.append(newObstacle)
        return newObstacle, wrapped

    def removeObstaclesFromList(self, obstacles):
        # They are getting sent to the union obstacles
        for i in obstacles:
            if isinstance(i, BlockMine):
                self.mines.remove(i)
            elif isinstance(i, unionObstacle):
                self.unionObstacles.remove(i)
            else:
                self.polygonObstacles.remove(i)

    def checkForObstacleCollisions(self, polygonObstacle):
        overlaps = []
        for i in self.polygonObstacles + self.mines + self.unionObstacles:
            if i.obstacleOverlap(polygonObstacle):
                overlaps.append(i)
        return overlaps

    # (nearest, farthest) distance between two obstacles. Cheap; used to group
    # obstacles by distance range before any exact geometry check.
    def _obstacleDistanceRange(self, a, b):
        nearest = a.polygon.distance(b.polygon)
        farthest = 0.0
        for p in a.vertices:
            for q in b.vertices:
                d = math.hypot(p[0] - q[0], p[1] - q[1])
                if d > farthest:
                    farthest = d
        return (nearest, farthest)

    @staticmethod
    def _vertexIndexAt(vertices, x, y):
        for i, v in enumerate(vertices):
            if v[0] == x and v[1] == y:
                return i
        return 0

    # Delete existing edges that polygonObstacle now crosses.
    # Split from connectPolygon so expandField can order it vs _addOwnConnections.
    def _cleanupInvalidatedConnections(self, polygonObstacle):
        nodeGraph = self.fieldConnection.nodeGraph
        edges = [
            (n1, n2) for n1, neighbors in nodeGraph.items() for n2 in neighbors if id(n1) < id(n2)
        ]
        connectionsToRemove = find_crossed_edges(edges, list(polygonObstacle.vertices[:-1]))
        for i in connectionsToRemove:
            self.fieldConnection.deleteConnection(i[0], i[1])

    # Add polygonObstacle's tangent connections to every other obstacle
    # (group + occlusion-arc algorithm). Split from connectPolygon like the above.
    def _addOwnConnections(self, polygonObstacle):
        others = [
            o
            for o in self.polygonObstacles + self.mines + self.unionObstacles
            if o is not polygonObstacle
        ]
        ranges = {o: self._obstacleDistanceRange(polygonObstacle, o) for o in others}
        sorted_others = sorted(others, key=lambda o: ranges[o][0])

        def arcContains(theta, loAngle, hiAngle, pivotAngle):
            t = _wrap_angle(theta, pivotAngle)
            return loAngle - 1e-9 <= t <= hiAngle + 1e-9

        remaining = list(sorted_others)
        while remaining:
            group = []
            combined_far = None
            for o in remaining:
                near, far = ranges[o]
                if combined_far is None or near <= combined_far:
                    group.append(o)
                    combined_far = far if combined_far is None else max(combined_far, far)
                else:
                    break
            remaining = remaining[len(group) :]

            # Up to 4 (selfNode, otherNode) tangent pairs per group member.
            candidate_pairs = {o: polygonObstacle.connectPolygonObstacle(o) for o in group}

            # Occlusion arcs for the NEW obstacle's vertices only, seeded from each
            # member's tangent pair. Older obstacles' arcs are never recomputed.
            new_arcs = (
                {}
            )  # Node -> list of (loAngle, hiAngle, pivotAngle, blockerObstacle, near, far)
            for other in group:
                pairs = candidate_pairs[other]
                if not pairs:
                    continue
                near, far = ranges[other]
                centroid = (other.polygon.centroid.x, other.polygon.centroid.y)
                _, seedLoNode = pairs[0]
                _, seedHiNode = pairs[-1]
                seedLoIdx = self._vertexIndexAt(other.vertices, seedLoNode.x, seedLoNode.y)
                seedHiIdx = self._vertexIndexAt(other.vertices, seedHiNode.x, seedHiNode.y)
                for node in polygonObstacle.nodes:
                    pivotAngle = math.atan2(centroid[1] - node.y, centroid[0] - node.x)
                    if other.polygon.covers(Point(node.x, node.y)):
                        # Tangent math needs a viewpoint OUTSIDE the hull. A node inside
                        # it (e.g. a concave union's notch) is blocked in every direction.
                        loAngle, hiAngle = -math.pi, math.pi
                    else:
                        loIdx = _find_point_tangent((node.x, node.y), other.vertices, 1, seedLoIdx)
                        hiIdx = _find_point_tangent((node.x, node.y), other.vertices, -1, seedHiIdx)
                        if loIdx is None or hiIdx is None:
                            loAngle, hiAngle = _brute_force_arc(
                                (node.x, node.y), other.vertices, centroid
                            )
                        else:
                            a1 = _wrap_angle(
                                math.atan2(
                                    other.vertices[loIdx][1] - node.y,
                                    other.vertices[loIdx][0] - node.x,
                                ),
                                pivotAngle,
                            )
                            a2 = _wrap_angle(
                                math.atan2(
                                    other.vertices[hiIdx][1] - node.y,
                                    other.vertices[hiIdx][0] - node.x,
                                ),
                                pivotAngle,
                            )
                            loAngle, hiAngle = min(a1, a2), max(a1, a2)
                    new_arcs.setdefault(node, []).append(
                        (loAngle, hiAngle, pivotAngle, other, near, far)
                    )

            for other in group:
                for selfNode, otherNode in candidate_pairs[other]:
                    theta = math.atan2(otherNode.y - selfNode.y, otherNode.x - selfNode.x)
                    overlapping = [
                        e
                        for e in new_arcs.get(selfNode, [])
                        if e[3] is not other and arcContains(theta, e[0], e[1], e[2])
                    ]

                    blocked = False
                    if overlapping:
                        seg = ((selfNode.x, selfNode.y), (otherNode.x, otherNode.y))
                        blocked = any(e[3].intersects(seg) for e in overlapping)

                    if not blocked:
                        near, far = ranges[other]
                        blocked = selfNode.crossGroupOccluded(
                            otherNode.x, otherNode.y, near, far, exclude_obstacles={other}
                        )

                    if not blocked:
                        self.fieldConnection.addGraph(
                            selfNode, otherNode
                        )  # bypasses validPath deliberately

            for node, arc_list in new_arcs.items():
                for entry in arc_list:
                    node.recordOcclusionArc(*entry)

    # Cleanup, then add own connections. For callers that don't need the phases split.
    def connectPolygon(self, polygonObstacle):
        self._cleanupInvalidatedConnections(polygonObstacle)
        self._addOwnConnections(polygonObstacle)

    # UNION OBSTACLES MUST BOTH CONNECT THEIR CONVEX HULL AND ACTUAL SHAPE
    def createPolygonObstacle(self, vertices):
        start = time.time()

        if len(vertices) == 2:
            return
        nodeList = []
        for i in vertices[:-1]:
            nodeList.append(Node(i[0], i[1], False))

        newObstacle = PolygonObstacle(vertices, nodeList, True)
        self.addObstacle(newObstacle, isWrapping=True)

    # Merge newObstacle with anything it overlaps (geometry + list bookkeeping only).
    # - No node wiring. Does NOT add the result to any field list; caller decides
    #   (a fresh merge may need re-checking before it's settled).
    # Returns (resolvedObstacle, isWrapping, wasMerged).
    def _resolveOverlaps(self, newObstacle: PolygonObstacle):
        overlaps = self.checkForObstacleCollisions(newObstacle)
        if overlaps:
            self.removeObstaclesFromList(overlaps)
            newObstacle, isWrapping = self.mergePolygons(overlaps + [newObstacle])
            return newObstacle, isWrapping, True
        return newObstacle, False, False

    # Wire obstacle's OWN nodes (perimeter + convex hull). Never links to other obstacles.
    # wasMerged: also wire the convex-hull chain (only matters for a fresh union).
    def _selfConnect(self, obstacle: PolygonObstacle, isWrapping: bool, wasMerged: bool):
        # Assign ids to nodes that lack one. Every obstacle passes through here
        # exactly once; a union's constituents keep their old ids.
        if isinstance(obstacle, BlockMine) and obstacle.id is None:
            obstacle.id = self._generateId()
        for node in obstacle.nodes:
            if node.id is None:
                node.id = self._generateId()

        convexHullPoints = obstacle.polygon.convex_hull.exterior.coords

        # Connect both the convex hull and polygon shape, but don't double connect any two nodes
        for i in range(len(obstacle.nodes) - 1):

            currentNode = obstacle.nodes[i]
            self.fieldConnection.connectNode(currentNode, obstacle.nodes[i + 1])
        if isWrapping:
            self.fieldConnection.connectNode(obstacle.nodes[-1], obstacle.nodes[0])

        if wasMerged and (len(obstacle.nodes) > 2):
            currentNodeIndex = 0
            laggingConvexNode = None
            firstConvexNode = None
            for i in convexHullPoints:
                if not self.withinField(i[0], i[1]):
                    continue
                while True:
                    currentNode = obstacle.nodes[currentNodeIndex]
                    if currentNode.x == i[0] and currentNode.y == i[1]:
                        if laggingConvexNode == None:
                            firstConvexNode = currentNode
                        elif laggingConvexNode in list(
                            self.fieldConnection.nodeGraph.get(currentNode, {}).keys()
                        ):
                            pass
                        elif not (currentNodeIndex == len(obstacle.nodes) - 1 and not isWrapping):
                            self.fieldConnection.connectNode(laggingConvexNode, currentNode)

                        laggingConvexNode = currentNode
                        break

                    currentNodeIndex += 1
                    currentNodeIndex %= len(obstacle.nodes)

            if (
                laggingConvexNode != None
                and firstConvexNode != None
                and laggingConvexNode != firstConvexNode
            ):
                self.fieldConnection.connectNode(laggingConvexNode, firstConvexNode)

    # Merge newObstacle with overlaps, then wire only its OWN nodes.
    # - Connections BETWEEN obstacles are deferred to connectPolygon/_addOwnConnections.
    # - Split from addObstacle so expandField can finish all merges first.
    def _mergeAndSelfConnect(
        self, newObstacle: PolygonObstacle, isWrapping: bool = False, isMine: bool = False
    ):
        newObstacle, mergedWrapping, wasMerged = self._resolveOverlaps(newObstacle)
        if wasMerged:
            self.unionObstacles.append(newObstacle)
            isWrapping = mergedWrapping
        elif isMine:
            self.mines.append(newObstacle)
        else:
            self.polygonObstacles.append(newObstacle)

        self._selfConnect(newObstacle, isWrapping, wasMerged)
        return newObstacle, wasMerged

    # Merge + self-connect, then connect to every other obstacle.
    def addObstacle(
        self, newObstacle: PolygonObstacle, isWrapping: bool = False, isMine: bool = False
    ) -> bool:
        newObstacle, wasMerged = self._mergeAndSelfConnect(newObstacle, isWrapping, isMine)
        self.connectPolygon(newObstacle)
        # Safety re-check of newObstacle's own new edges against EVERY obstacle.
        # - _addOwnConnections' grouping can miss a blocker outside its group
        #   (seen as a real bad edge in the two-pair sweep).
        # - Test = edge MIDPOINT inside an obstacle. find_crossed_edges misses
        #   shallow (<1ft) slivers; a real tangent's midpoint is never inside.
        # - Only newObstacle's own edges (the ones that just changed).
        own_edges = [
            (node, neighbor)
            for node in newObstacle.nodes
            for neighbor in self.fieldConnection.nodeGraph.get(node, {})
        ]
        if own_edges:
            obstacles = [
                o for o in self.polygonObstacles + self.mines + self.unionObstacles if o is not newObstacle
            ]
            to_remove = []
            for n1, n2 in own_edges:
                mx, my = (n1.x + n2.x) / 2.0, (n1.y + n2.y) / 2.0
                for o in obstacles:
                    if o.contains_point((mx, my)):
                        to_remove.append((n1, n2))
                        break
            for n1, n2 in to_remove:
                self.fieldConnection.deleteConnection(n1, n2)

        # Reverse check: does newObstacle invalidate an OLD edge elsewhere?
        # Same midpoint test, since _cleanupInvalidatedConnections
        # (find_crossed_edges) misses shallow slivers. Similar cost to that scan.
        own_node_set = set(newObstacle.nodes)
        other_edges = [
            (n1, n2)
            for n1, neighbors in self.fieldConnection.nodeGraph.items()
            for n2 in neighbors
            if id(n1) < id(n2) and n1 not in own_node_set and n2 not in own_node_set
        ]
        to_remove2 = []
        for n1, n2 in other_edges:
            mx, my = (n1.x + n2.x) / 2.0, (n1.y + n2.y) / 2.0
            if newObstacle.contains_point((mx, my)):
                to_remove2.append((n1, n2))
        for n1, n2 in to_remove2:
            self.fieldConnection.deleteConnection(n1, n2)
        # Floating nodes placed BEFORE this obstacle never connected to it,
        # so connect them now (mirror of the loop in addFloatingNode).
        for fNode in self.floatingNodes:
            self._connectFloatingNodeToObstacle(newObstacle, fNode)
        return wasMerged

    def addFromProtoMine(self, protoMine: protoMine) -> bool:
        if len(protoMine.nodeVertices) == 0:
            return False
        newMine = BlockMine(protoMine.nodeVertices, origin=tuple(protoMine.mineLocation))
        return self.addObstacle(newMine, isWrapping=True, isMine=True)

    # All BlockMines in obstacle_list, including ones nested inside unions
    # (a union can be part of a later union).
    def _collect_mines(self, obstacle_list):
        found = []
        for o in obstacle_list:
            if isinstance(o, BlockMine):
                found.append(o)
            elif isinstance(o, unionObstacle):
                found.extend(self._collect_mines(o.obstacleList))
        return found

    # Grow every live mine (standalone or inside a union) by `distance`, then rebuild.
    # Phases:
    #   1. Teardown + expand: geometry only, no connections.
    #   2. Merge pass: resolve ALL overlaps to a fixed point, then self-wire each
    #      final obstacle once. A one-shot scan can miss merges (X settles, then
    #      Y+Z merge into a union that overlaps X), leaving a node inside an
    #      unmerged obstacle, which breaks the tangent math.
    #   3. Connection phase: cleanup for all affected obstacles, then add own
    #      connections for all. (Cleanup-first was ~6% faster; both orders were safe.)
    def expandField(self, distance):
        standalone_mines = list(self.mines)
        affected_unions = [u for u in self.unionObstacles if self._collect_mines(u.obstacleList)]

        if not standalone_mines and not affected_unions:
            return

        all_expanded_mines = list(standalone_mines)

        for mine in standalone_mines:
            for node in mine.nodes:
                self.fieldConnection.purgeConnections(node)
            self.mines.remove(mine)

        union_constituents = []
        for u in affected_unions:
            for node in u.nodes:
                self.fieldConnection.purgeConnections(node)
            self.unionObstacles.remove(u)
            union_constituents.extend(u.obstacleList)

        all_expanded_mines.extend(self._collect_mines(union_constituents))
        for mine in all_expanded_mines:
            mine.expand(distance)

        # Phase 2: merge pass (fixed point; see note above the method).
        isMineFlag = {id(m): True for m in standalone_mines}
        for o in union_constituents:
            isMineFlag.setdefault(id(o), isinstance(o, BlockMine))

        pending = list(standalone_mines) + list(union_constituents)
        while pending:
            obj = pending.pop()
            obj, _, wasMerged = self._resolveOverlaps(obj)
            if wasMerged:
                # The merged shape may overlap something else; re-check it.
                pending.append(obj)
            elif isinstance(obj, unionObstacle):
                self.unionObstacles.append(obj)
            elif isMineFlag.get(id(obj), False):
                self.mines.append(obj)
            else:
                self.polygonObstacles.append(obj)

        # Rebuild the affected set from the field's lists (an early mine may
        # have been absorbed into a later union).
        expanded_ids = {id(m) for m in all_expanded_mines}
        final_affected = [m for m in self.mines if id(m) in expanded_ids]
        for u in self.unionObstacles:
            if any(id(m) in expanded_ids for m in self._collect_mines(u.obstacleList)):
                final_affected.append(u)

        # Self-wire each final obstacle once (safe now that merges are settled).
        for o in final_affected:
            self._selfConnect(o, True, isinstance(o, unionObstacle))

        for o in final_affected:
            self._cleanupInvalidatedConnections(o)
        for o in final_affected:
            self._addOwnConnections(o)

        # The purge at the top also dropped links to floating (start/end) nodes,
        # and the steps above only restore obstacle-to-obstacle links. Reconnect them.
        for o in final_affected:
            for fNode in self.floatingNodes:
                self._connectFloatingNodeToObstacle(o, fNode)

    # Purely for debugging will have a growing list of parameters
    def plotField(
        self,
        labeled: bool = False,
        path: list["Node"] = [],
        title: str = "",
        xlabel: str = "",
        labelPath: bool = False,
        pastPath: list["Node"] = [],
    ) -> None:
        """
        Using the matplotlib library and various optional debug options, plots the current iteration of the field
        """
        plt = pyplot
        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        padding = 10
        plt.xlim(self.xMin - padding, self.xMax + padding)
        plt.ylim(self.yMin - padding, self.yMax + padding)

        # Mines, plain polygon obstacles, and unions all expose (vertices, polygon, nodes).
        obstacles = self.mines + self.polygonObstacles + self.unionObstacles

        if len(title) <= 0:
            title = f"Obstacles({len(obstacles)}) and Potential Paths"
        xlabel += "KEY:\n"
        # Obstacles are not filled: a hull fill would hide the real (non-convex) shape.
        for obstacle in obstacles:
            if not hasattr(obstacle, "color"):
                obstacle.color = (
                    random.randint(20, 80) / 100,
                    random.randint(20, 80) / 100,
                    random.randint(20, 80) / 100,
                )
            # Separate outline color, so it isn't confused with the node-connection color.
            if not hasattr(obstacle, "outlineColor"):
                obstacle.outlineColor = (
                    random.randint(20, 80) / 100,
                    random.randint(20, 80) / 100,
                    random.randint(20, 80) / 100,
                )

        for i, obstacle in enumerate(obstacles):
            if not getattr(obstacle, "vertices", None):
                continue
            centerX, centerY = obstacle.polygon.centroid.x, obstacle.polygon.centroid.y
            if labeled:
                vertalignment = ["top", "bottom", "baseline", "center_baseline"]
                horzalignment = ["left", "right", "center"]
                plt.text(
                    centerX,
                    centerY,
                    f"{type(obstacle).__name__} {i}",
                    horizontalalignment=random.choice(horzalignment),
                    verticalalignment=random.choice(vertalignment),
                    bbox=dict(facecolor=(0.5, 0.5, 0.5), alpha=0.3, linewidth=0),
                )
            plt.plot(centerX, centerY, "x", color=(1, 1, 1))

        # Obstacle outline (vertices + edges), independent of the node graph drawn below.
        for obstacle in obstacles:
            vertices = getattr(obstacle, "vertices", None)
            if not vertices:
                continue
            xs = [v[0] for v in vertices] + [vertices[0][0]]
            ys = [v[1] for v in vertices] + [vertices[0][1]]
            plt.plot(xs, ys, marker="o", color=obstacle.outlineColor)

        nodeSymbol = ""  # Empty string makes either lines or invisible points; otherwise points are displayed using the symbol
        # node -> obstacle color / index, so edges on one polygon match and nodes show their index.
        nodeObstacleColor = {}
        nodeObstacleIndex = {}
        for obstacle in obstacles:
            for index, node in enumerate(getattr(obstacle, "nodes", [])):
                nodeObstacleColor[node] = obstacle.color
                nodeObstacleIndex[node] = index
        if len(path) > 0:
            xlabel += "\nBlack = A* path"
            if labelPath:
                for node in path:
                    node.labeled = True

        for node in self.fieldConnection.nodeGraph.keys():
            if labeled or node.labeled:
                vertalignment = ["top", "bottom", "baseline", "center_baseline"]
                horzalignment = ["left", "right", "center"]
                nodeLabel = str(node)
                if node in nodeObstacleIndex:
                    nodeLabel += f" [{nodeObstacleIndex[node]}]"
                plt.text(
                    node.x,
                    node.y,
                    nodeLabel,
                    horizontalalignment=random.choice(horzalignment),
                    verticalalignment=random.choice(vertalignment),
                    c=(0.0, 0.0, 0.0),
                )

            # Node marker: colored by its obstacle, labeled with its index in that obstacle.
            plt.plot(
                node.x, node.y, "o", color=nodeObstacleColor.get(node, (0, 0, 0)), markersize=4
            )
            if node in nodeObstacleIndex:
                plt.text(
                    node.x,
                    node.y,
                    str(nodeObstacleIndex[node]),
                    horizontalalignment="center",
                    verticalalignment="center",
                    fontsize=6,
                    c=(1, 1, 1),
                )

            if not node.plotted:
                for connectedNode in self.fieldConnection.nodeGraph[node].keys():
                    sharedObstacleColor = nodeObstacleColor.get(node)
                    # Same obstacle: use the obstacle's color.
                    if (
                        sharedObstacleColor is not None
                        and nodeObstacleColor.get(connectedNode) is sharedObstacleColor
                    ):
                        plt.plot(
                            [node.x, connectedNode.x],
                            [node.y, connectedNode.y],
                            nodeSymbol,
                            color=sharedObstacleColor,
                        )
                    # If it is an arc connection, same parent mines, then draw a curve
                    elif connectedNode.parentMine == node.parentMine and node.parentMine != None:
                        plt.plot([node.x, connectedNode.x], [node.y, connectedNode.y], nodeSymbol)
                    else:
                        # Otherwise, draw a line
                        # pass
                        try:

                            plt.plot(
                                [node.x, connectedNode.x], [node.y, connectedNode.y], nodeSymbol
                            )
                        except AttributeError:
                            plt.plot([node.x], [node.y], nodeSymbol)
        xlabel += "Colors = Potential Paths"
        xlabel += "\nLight Gray = Simulated Boundary"
        xlabel += "\nDark Gray = Field Boundary"
        xlabel += "\nX = Obstacles' centers"
        xlabel += "\nO = Node, colored by obstacle; number = index within its obstacle's node list"
        xlabel += (
            "\nColored outline = obstacle's own vertices/edges (separate from node connections)"
        )
        # If a path is passed in, display the path as a black line
        if len(path) > 0:
            for i, node in enumerate(path):
                if i < len(path) - 1:
                    nextNode = path[i + 1]
                    plt.plot([node.x, nextNode.x], [node.y, nextNode.y], color=(0, 0, 0))

        if len(Field.debugPoints) > 0:  # Points that are plotted for debugging only
            print("Plotting debug points")
            for point in Field.debugPoints:
                plt.plot(point[0], point[1], "o", color=(0, 0, 0))

        # Plot simulation boundaries
        for pair in [
            self.simHorzPairUpper,
            self.simHorzPairLower,
            self.simVertPairLeft,
            self.simVertPairRight,
        ]:
            plt.plot([pair[0][0], pair[1][0]], [pair[0][1], pair[1][1]], color=(0.5, 0.5, 0.5))
        # Plot field boundaries
        for pair in [
            self.fieldVertPairLeft,
            self.fieldVertPairRight,
            self.fieldHorzPairUpper,
            self.fieldHorzPairLower,
        ]:
            plt.plot([pair[0][0], pair[1][0]], [pair[0][1], pair[1][1]], color=(0.3, 0.3, 0.3))

        # Previous path, if any (useful when calling plotField twice).
        if len(pastPath) > 0:
            for i, node in enumerate(pastPath):
                if i < len(path) - 1:
                    nextNode = path[i + 1]
                    plt.plot([node.x, nextNode.x], [node.y, nextNode.y], color=(0, 0, 0))

        print("Done plotting")
        print("Displaying field...")

        plt.title(title)
        plt.xlabel(xlabel)
        plt.show()
        print("Done displaying field.")

    def graphAtRadius(self, radius: int):
        shallowCopy = self.nodeGraph.copy()
        for node1 in shallowCopy.keys():
            deepCopy = shallowCopy[node1].copy()
            shallowCopy[node1] = deepCopy

        for node1 in shallowCopy.keys():
            deepCopy = shallowCopy[node1].copy()
            for node2 in deepCopy:
                connection = FieldConnections(node1, node2)
                if connection.connectionType == seg.ARC:
                    connection.deleteConnection()

    def increaseRadius(self, step: int):
        pass

    @staticmethod  # Given two points, get the line equation and slope (to determine negative or positive slope)
    def getLine(point1: tuple, point2: tuple) -> tuple[Callable[[float], float], float]:
        """
        Given two points as a tuple of floats each, get a line function and its slope
        """
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]
        try:
            slope = (y2 - y1) / (x2 - x1)
        except ZeroDivisionError:  # Infinite/Vertical slope
            # x means nothing in this case, for all values of Y, its x is x1 and x2
            return (lambda x: x1 + 0 * x, "undef")

        offset = y2 - slope * x2

        f = lambda x: (slope * x) + offset
        return (f, slope)

    # Given a line function and a point, detect if the point
    @staticmethod  # lies to the left of the line
    def isPointLeftofLine(
        line: Callable[
            [tuple[float, float], tuple[float, float]], tuple[Callable[[float], float], float]
        ],
        slope: float,
        point: tuple[float, float],
    ) -> bool:
        """
        Given a line function, point, and slope;
        Check if the point lies to the left of the line
        """
        """
        If the slope between p1 and p2 is negative, p3's y-value must be
        below the line for it to be to the left of line
        p1
         `
          `
           `
            `
          p3 `
              `
              p2
        The logic will be adjusted for positive and undefined(vertical line) slope.

        """
        x = point[0]
        y = point[1]
        if isinstance(slope, str):
            if slope == "undef":  # Verticle line
                if x < line(x):
                    return True
        if isinstance(slope, float):
            if slope < 0:  # Negative slope
                if y < line(x):
                    return True
            elif slope > 0:  # Positive slope
                if y > line(x):
                    return True
            else:
                # If the points are horizontal, and since this is checking a *line*
                # A point will always be within the line <-----*--->
                # So technically cant be left of the line
                return False
        return False

    # Given a line function and a point, detects if the point
    @staticmethod  # lies to the right of the line
    def isPointRightofLine(
        line: Callable[
            [tuple[float, float], tuple[float, float]], tuple[Callable[[float], float], float]
        ],
        slope: float,
        point: tuple[float, float],
    ) -> bool:
        """
        Given a line function, point, and slope;
        Check if the point lies to the right of the line
        """
        """
        If the slope between p1 and p2 is negative, p3's y-value must be
        above the line for it to be to the right of line
        p1
         `
          ` p3
           `
            `
             `
              `
              p2
        The logic will be adjusted for positive and undefined(vertical line) slope
        """
        # point[0],point[1] = x,y
        x = point[0]
        y = point[1]
        if isinstance(slope, str):
            if slope == "undef":  # Verticle line
                if x > line(x):
                    return True
        if isinstance(slope, float):
            if slope < 0:  # Negative Slope
                if y > line(x):
                    return True
            elif slope > 0:  # Positive Slope
                if y < line(x):
                    return True
            else:  # Horizontal
                return False
        return False

    # Given a line function and a point, detects if the point
    @staticmethod  # lies above the line
    def isPointAboveLine(
        line: Callable[
            [tuple[float, float], tuple[float, float]], tuple[Callable[[float], float], float]
        ],
        slope: float,
        point: tuple[float, float],
    ):
        """
        Given a line function, point, and slope;
        Check if the point lies above the line
        """
        x = point[0]
        y = point[1]
        if isinstance(slope, str):
            if slope == "undef":  # Vertical line
                return True
        if isinstance(slope, float):
            if y > line(x):
                return True
        return False

    # Given a line function and a point, detects if the point
    @staticmethod  # lies below the line
    def isPointBelowLine(
        line: Callable[
            [tuple[float, float], tuple[float, float]], tuple[Callable[[float], float], float]
        ],
        slope: float,
        point: tuple[float, float],
    ):
        """
        Given a line function, point, and slope;
        Check if the point lies below the line
        """
        x = point[0]
        y = point[1]
        if isinstance(slope, str):
            if slope == "undef":  # Vertical line
                return None
        if isinstance(slope, float):
            if y < line(x):
                return True
        return False


class seg(Enum):
    ARC = 1
    LINE = 2


class FieldConnections:
    def __init__(self, field: "Field", mineRadius: float = -1):
        self.nodeGraph = {}
        self.field = field
        """
        self.mineRadius = (
            mineRadius if mineRadius != -1 else Mine.radius
        )  # Default to the first mine's radius if not specified, but should be updated to a more dynamic value


        self.distance = self.updateDistance()

        # checking for a valid path and updating the graph must be done manually
        """

    def getConnectionType(self, node1, node2):
        connectionType = None
        if node1.parentMine != node2.parentMine or node1.floating or node2.floating:
            connectionType = seg.LINE
        else:
            connectionType = seg.ARC

        if node1.parentMine and node2.parentMine:

            if node1.parentMine != node2.parentMine:
                connectionType = seg.LINE
            else:
                connectionType = seg.ARC
        else:
            connectionType = seg.LINE
        return connectionType

    # DISTANCE
    def getDistance(self, node1, node2):
        distance = 0.0
        connectionType = self.getConnectionType(node1, node2)
        if connectionType == seg.ARC:  # Nodes are on the same mine

            # Get two different angle differences, one for major arc, the other for minor arc

            nodeAngle1 = node1.angle
            nodeAngle2 = node2.angle
            angleTheta = abs(nodeAngle1 - nodeAngle2)
            if abs(node1.mineOrder - node2.mineOrder) == 1:
                angleTheta = min(angleTheta, 2 * np.pi - angleTheta)

            mineRadius = self.node1.parentMine.radius
            distance = angleTheta * mineRadius

        else:  # Nodes are on seperate mines
            distance = np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
            distance = float(distance)
        return distance

    # Graph organization:
    """
    (Connection objects are two-way and shared)
    {
    Node1: {Node2:ConnectionObj 1<->2, Node3:ConnectionObj 1<->3},
    Node2: {Node1:ConnectionObj 1<->2},
    Node3: {Node1:ConnectionObj 1<->3}
    }
    """

    # Establishes a connection between nodes
    # Does not add it to the nodegraph yet however
    def connectNode(self, node1, node2) -> bool:
        if node1 == node2:
            raise TypeError("Same nodes")
        validPath = self.validPath(node1, node2)
        if validPath:
            self.addGraph(node1, node2)
        return validPath

    def addGraph(self, node1, node2):
        # Bounds check only applies when neither node is floating
        # (start/end nodes may sit outside the field).
        if not (node1.floating or node2.floating):
            if not (
                self.field.withinField(node1.x, node1.y)
                and self.field.withinField(node2.x, node2.y)
            ):
                return
        distance = self.getDistance(node1, node2)
        if node1 not in self.nodeGraph:
            self.nodeGraph.update({node1: {node2: distance}})

        elif node2 not in self.nodeGraph[node1]:
            self.nodeGraph[node1].update({node2: distance})
        # else: # Im not sure why this would be considered broken
        # print("node1 is in field.nodeGraph AND node2 in node1's nodeGraph")
        # print("Something Broke")

        if (
            node2 not in self.nodeGraph
        ):  # Needed for its first connection: When a node is made, it's key is automatically added to nodeGraph with a none value.
            self.nodeGraph.update(
                {node2: {node1: distance}}
            )  # Must use = to get rid of the none value
        elif node1 not in self.nodeGraph[node2]:
            self.nodeGraph[node2].update({node1: distance})
        # else: # Im not sure why this would be considered broken
        #     print("node2 is in field.nodeGraph AND node1 in node2's nodeGraph")
        #     print("Something Broke")

    def purgeConnections(self, node1):
        if node1 not in self.nodeGraph.keys():
            return
        connections = list(self.nodeGraph[node1].keys())

        for i in connections:
            self.deleteConnection(node1, i)

    def removeNode(self, node) -> None:
        """
        Remove node from the graph.
        - Purges every edge, then deletes the node's own key (no 0-edge orphan).
        - purgeConnections alone never removes the key (purgeNodes is always False).
        """
        self.purgeConnections(node)
        self.nodeGraph.pop(node, None)

    def deleteConnection(self, node1, node2):
        purgeNodes = False

        field = self.field
        purgeNodes = False

        if node1 in self.nodeGraph:
            if node2 in self.nodeGraph[node1]:
                del self.nodeGraph[node1][node2]
            if len(self.nodeGraph[node1]) == 0 and purgeNodes:
                del self.nodeGraph[node1]
                # node1.deleteNode()
        else:
            pass
            # node1.deleteNode()

        if node2 in self.nodeGraph:
            if node1 in self.nodeGraph[node2]:
                del self.nodeGraph[node2][node1]
            if len(self.nodeGraph[node2]) == 0 and purgeNodes:
                del self.nodeGraph[node2]
                # node2.deleteNode()
        else:
            pass
            # node2.deleteNode()

        # Run this to remove nodes that have no associated connection, ie, {node: None}

    def cleanNodeGraph(self):
        """
        Removes nodes that have no associated connection from the node graph
        \nSuch as:
        \n{node: None}
        """
        if self.nodeGraph != None:
            for node in self.nodeGraph.copy():
                if self.nodeGraph[node] == None:
                    del self.nodeGraph[node]
            return self.nodeGraph
        else:
            print("Node graph is empty")

    # Checks if a newly created path is valid, checks all mines for collisions
    def validPath(self, node1, node2):

        if node1 == node2:
            return False

        x1 = float(node1.x)
        y1 = float(node1.y)
        x2 = float(node2.x)
        y2 = float(node2.y)
        field = self.field

        """
        Check if the the current connection's nodes are within
        field boundries.
        """
        #  Node landing outside of field boundaries
        """
                               p2
                            `     `   n2
                        `           `
                   Up                  Ri
               `           n1            `
           `                                `
         p1                                   `
            `                                  p4
               `                             `
                  `                        `
                     Le                  Lo
                        `              `
                          `         `
                             `   `
                               p3(Origin)
        """
        """

        """
        # Connection intersecting mine test

        if self.getConnectionType(node1, node2) == seg.LINE:
            pass
            """
            boundingBox = quads.BoundingBox(
                min_x=min(x1, x2) - self.mineRadius,
                min_y=min(y1, y2) - self.mineRadius,
                max_x=max(x1, x2) + self.mineRadius,
                max_y=max(y1, y2) + self.mineRadius,
            )
            minesToCheck =self.field.mineQuadTree.within_bb(boundingBox)
            for mine in minesToCheck:

                mine = mine.data
                x3 = mine.x
                y3 = mine.y

                # Fraction of segment between nodes that the mine lands perpendicular to segment
                uNumerator = ((x3 - x1) * (x2 - x1)) + ((y3 - y1) * (y2 - y1))
                uDenominator = ((x1 - x2) ** 2) + ((y1 - y2) ** 2)
                if uDenominator == 0:
                    u = 0
                else:
                    u = np.clip(
                        uNumerator / uDenominator, 0, 1
                    )  # Restrict to the constraints of a segment

                # Adjust accordingly, determines how close a mine can be to a node before the node terminates
                uMin = 0.03
                uMax = 0.98

                if uMin <= u <= uMax:
                    # Point on segment that is tangent and perpendicular to mine
                    tangePoint = (x1 + (u * (x2 - x1)), y1 + (u * (y2 - y1)))

                    distanceFromMine = np.sqrt(
                        (mine.x - tangePoint[0]) ** 2 + (mine.y - tangePoint[1]) ** 2
                    )
                    if distanceFromMine < mine.radius:
                        return False

                # Check if node is in mine
                n1distance = np.sqrt(((x1 - x3) ** 2) + ((y1 - y3) ** 2))
                n2distance = np.sqrt(((x2 - x3) ** 2) + ((y2 - y3) ** 2))

                if node1.parentMine != mine:
                    if n1distance <= mine.radius:
                        return False
                if node2.parentMine != mine:
                    if n2distance <= mine.radius:
                        return False
            """
        for polygonObstacle in (
            self.field.polygonObstacles + self.field.mines + self.field.unionObstacles
        ):

            if polygonObstacle.intersects(((x1, y1), (x2, y2))):
                # print("Polygon Obstacle Intersected")
                return False
        """
        if self.connectionType == seg.ARC:
            parentMine = node1.parentMine
            validEdge = True

            for mine in parentMine.mineDistances.keys():
                if mine.mineDistances[parentMine] >= parentMine.radius + mine.radius:
                    continue
                # Other than being None, there should only be 2 values
                intersectionPoints, intersectionAngle, offsetAngle = (
                    self.generateIntersectionPoints(parentMine, mine)
                )

                if intersectionPoints != None:
                    validEdge = validEdge and self.validHuggingEdge(intersectionAngle, offsetAngle)
                else:
                    print("Something went really wrong with midpoint & intersectionpoints")
            return validEdge

        return True
        """
        return True

    # checks if a path collides with a specific mine
    def mineNodeCollision(self, mine, node1, node2) -> bool:
        if node1 == node2:
            return True

        x1 = float(node1.x)
        y1 = float(node1.y)
        x2 = float(node2.x)
        y2 = float(node2.y)
        if self.getConnectionType(node1, node2) == seg.LINE:

            x3 = mine.x
            y3 = mine.y

            # Fraction of segment between nodes that the mine lands perpendicular to segment
            uNumerator = ((x3 - x1) * (x2 - x1)) + ((y3 - y1) * (y2 - y1))
            uDenominator = ((x1 - x2) ** 2) + ((y1 - y2) ** 2)
            if uDenominator == 0:
                u = 0
            else:
                u = np.clip(
                    uNumerator / uDenominator, 0, 1
                )  # Restrict to the constraints of a segment

            # Adjust accordingly, determines how close a mine can be to a node before the node terminates
            uMin = 0.03
            uMax = 0.98

            if uMin <= u <= uMax:
                # Point on segment that is tangent and perpendicular to mine
                tangePoint = (x1 + (u * (x2 - x1)), y1 + (u * (y2 - y1)))

                distanceFromMine = np.sqrt(
                    (mine.x - tangePoint[0]) ** 2 + (mine.y - tangePoint[1]) ** 2
                )
                if distanceFromMine < mine.radius:
                    return True

            # Check if node is in mine
            n1distance = np.sqrt(((x1 - x3) ** 2) + ((y1 - y3) ** 2))
            n2distance = np.sqrt(((x2 - x3) ** 2) + ((y2 - y3) ** 2))

            if node1.parentMine != mine:
                if n1distance <= mine.radius:
                    return True
            if node2.parentMine != mine:
                if n2distance <= mine.radius:
                    return True
        elif self.connectionType == seg.ARC:
            parentMine = node1.parentMine

            # Other than being None, there should only be 2 values
            intersectionPoints, intersectionAngle, offsetAngle = self.generateIntersectionPoints(
                parentMine, mine
            )

            if intersectionPoints != None:
                validEdge = self.validHuggingEdge(intersectionAngle, offsetAngle)
                return not (validEdge)

        return False

    def __str__(self):
        return f"{self.node1} <-> {self.node2}"

    def __repr__(self):
        return self.__str__()


def _link():
    # Dead code: never called. Imports are local so its stale legacy
    # dependencies (flight/newPathfinding was removed) can't break importing field.py.
    from flight.pathfinding.nodeField.archive import mine as m
    from flight.pathfinding.nodeField import node as n
    from flight.pathfinding.nodeField import node_connection as nc
    from flight.newPathfinding import diamondMine as dM

    global Mine, FieldConnections, Node, MineNode, seg, Field, BlockyObstacle
    Mine = m.Mine
    FieldConnections = nc.Connection
    Node = n.Node
    MineNode = n.MineNode
    seg = nc.seg
    BlockyObstacle = dM.BlockyObstacle


def _in_circle(c, p, eps=1e-9):
    return math.hypot(p[0] - c[0], p[1] - c[1]) <= c[2] + eps + 1e-12 * c[2]


def _circle_from_two(a, b):
    cx, cy = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
    return (cx, cy, math.hypot(a[0] - b[0], a[1] - b[1]) / 2.0)


def _circle_from_three(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:  # collinear -> fall back to the widest pair
        best = None
        pts = (a, b, c)
        for i in range(3):
            for j in range(i + 1, 3):
                cand = _circle_from_two(pts[i], pts[j])
                if all(_in_circle(cand, pts[k]) for k in range(3)):
                    if best is None or cand[2] < best[2]:
                        best = cand
        return best
    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return (ux, uy, math.hypot(ux - ax, uy - ay))


def minimum_enclosing_circle(points):
    """Return (cx, cy, r) of the smallest circle enclosing all `points`."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    random.shuffle(pts)
    c = None
    for i, p in enumerate(pts):
        if c is None or not _in_circle(c, p):
            c = (p[0], p[1], 0.0)
            for j in range(i):
                q = pts[j]
                if not _in_circle(c, q):
                    c = _circle_from_two(p, q)
                    for k in range(j):
                        s = pts[k]
                        if not _in_circle(c, s):
                            c = _circle_from_three(p, q, s)
    return c


# --------------------------------------------------------------------------- #
# Angular interval query on a sorted list of angles in [0, 2*pi)
# --------------------------------------------------------------------------- #
def _angular_query(vals, idx, lo, hi):
    if lo <= hi:
        left = bisect.bisect_left(vals, lo)
        right = bisect.bisect_right(vals, hi)
        return idx[left:right]
    # window wraps past 0 / 2*pi -> two sub-ranges
    left = bisect.bisect_left(vals, lo)
    right = bisect.bisect_right(vals, hi)
    return idx[left:] + idx[:right]


# --------------------------------------------------------------------------- #
# Main routine
# --------------------------------------------------------------------------- #
def find_colliding_pairs(nodes, obstacle):
    """
    Parameters
    ----------
    nodes : list of objects each exposing ``.x`` and ``.y``
    obstacle : list of (x, y) vertices describing a polygon

    Returns
    -------
    list of (node_a, node_b) tuples whose connecting segment intersects the obstacle.
    """
    n = len(nodes)
    if n < 2:
        return []

    cx, cy, R = minimum_enclosing_circle(obstacle)

    poly = Polygon(obstacle)
    prepared = prep(poly)  # speeds up repeated .intersects() calls

    TWO_PI = 2.0 * math.pi
    PAD = 1e-9  # inclusive-boundary safety margin; only ever adds candidates

    # polar coordinates of every node about the circle centre
    radius = [0.0] * n
    angle = [0.0] * n
    for i, nd in enumerate(nodes):
        dx = nd.x - cx
        dy = nd.y - cy
        radius[i] = math.hypot(dx, dy)
        angle[i] = math.atan2(dy, dx) % TWO_PI

    # angle-sorted arrays for interval queries
    by_angle = sorted(range(n), key=lambda i: angle[i])
    ang_vals = [angle[i] for i in by_angle]
    ang_idx = by_angle

    # closest-first processing order; tie-break by index so each pair has one owner
    order = sorted(range(n), key=lambda i: (radius[i], i))
    rank = [0] * n
    for pos, i in enumerate(order):
        rank[i] = pos

    results = []
    for i in order:
        r_i = radius[i]

        # window half-width beta = 2 * arcsin(R / d); inside the circle -> full sweep
        if r_i <= R:
            candidates = ang_idx
        else:
            beta = 2.0 * math.asin(R / r_i) + PAD
            if beta >= math.pi:
                candidates = ang_idx
            else:
                centre = (angle[i] + math.pi) % TWO_PI
                lo = (centre - beta) % TWO_PI
                hi = (centre + beta) % TWO_PI
                candidates = _angular_query(ang_vals, ang_idx, lo, hi)

        node_i = nodes[i]
        rank_i = rank[i]
        for j in candidates:
            if rank[j] <= rank_i:  # pair only with strictly-farther nodes
                continue
            node_j = nodes[j]
            seg = LineString(((node_i.x, node_i.y), (node_j.x, node_j.y)))
            if prepared.crosses(seg):
                results.append((node_i, node_j))

    return results


# Like find_colliding_pairs, but tests only EXISTING edges instead of every node pair.
# - Same result (deleting a non-edge was a no-op), ~4.5x less work on a 70-mine field.
# - Separate function so connectPolygonTimingCheck.py can still call the old one.
def find_crossed_edges(edges, obstacle):
    """
    Takes:
    - edges: list of (node_a, node_b), each with .x/.y (the real existing connections).
    - obstacle: list of (x, y) polygon vertices.

    Returns:
    - The subset of edges that cross the obstacle boundary OR lie fully inside it
      (a merge can grow an obstacle around an existing edge; crosses() alone misses that).
    """
    if not edges:
        return []
    poly = Polygon(obstacle)
    prepared = prep(poly)
    obs_min_x = min(v[0] for v in obstacle)
    obs_max_x = max(v[0] for v in obstacle)
    obs_min_y = min(v[1] for v in obstacle)
    obs_max_y = max(v[1] for v in obstacle)
    results = []
    for node_a, node_b in edges:
        # Bounding boxes that don't overlap can't cross: cheap, exact rejection
        # (~4x faster on a 70-mine field).
        seg_min_x = node_a.x if node_a.x < node_b.x else node_b.x
        seg_max_x = node_a.x if node_a.x > node_b.x else node_b.x
        if seg_max_x < obs_min_x or seg_min_x > obs_max_x:
            continue
        seg_min_y = node_a.y if node_a.y < node_b.y else node_b.y
        seg_max_y = node_a.y if node_a.y > node_b.y else node_b.y
        if seg_max_y < obs_min_y or seg_min_y > obs_max_y:
            continue
        seg = LineString(((node_a.x, node_a.y), (node_b.x, node_b.y)))
        # contains() covers an edge fully inside the obstacle (see PolygonObstacle.intersects).
        if prepared.crosses(seg) or prepared.contains(seg):
            results.append((node_a, node_b))
    return results
