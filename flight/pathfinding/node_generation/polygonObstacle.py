from typing import List, Tuple
from shapely.geometry import Polygon, LineString, Point
from scipy.spatial import ConvexHull
from flight.pathfinding.node_generation.node import Connection, Node
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as polyPlt
import numpy as np


def is_clockwise(points):
    signed_area = 0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        signed_area += (x1 * y2 - x2 * y1)
    return signed_area > 0


class PolygonObstacle:
    def __init__(self, vertices: List[Tuple[float, float]], maxX, maxY, minX, minY):
        hull = ConvexHull(vertices, qhull_options='Qx')
        self.isWrapping = True
        self.vertices = [vertices[i] for i in hull.vertices]
        indices_to_remove = []
        for i in reversed(self.vertices):
            if i[0] < minX or i[0] > maxX or i[1] < minY or i[1] > maxY:
                self.isWrapping = False
                indices_to_remove.append(self.vertices.index(i))
        if len(indices_to_remove) > 0:
            if indices_to_remove[0] < len(self.vertices) - 1:
                rotated_vertices = self.vertices[indices_to_remove[0] + 1 :]
                del self.vertices[indices_to_remove[-1] :]
                self.vertices = rotated_vertices + self.vertices
            else:
                for i in indices_to_remove:
                    del self.vertices[i]

        if len(self.vertices) < 2:
            return

        self.xMin = min(v[0] for v in self.vertices)
        self.xMax = max(v[0] for v in self.vertices)
        self.yMin = min(v[1] for v in self.vertices)
        self.yMax = max(v[1] for v in self.vertices)

        prev_node = None
        self.nodes = []
        for point in self.vertices:
            new_node = Node(point[0], point[1], False)
            if prev_node is not None:
                prev_node.connectNode(new_node)
            self.nodes.append(new_node)
            prev_node = new_node

        if self.isWrapping:
            self.nodes[-1].connectNode(self.nodes[0])

        self.polygon = Polygon(self.vertices)

    def findClosestNodes(self, other_obstacle: 'PolygonObstacle') -> List[Tuple[Node, Node]]:
        connected_nodes = []
        closestCombo=[]
        closestValue=9999999999
        for node1 in self.nodes:
            for node2 in other_obstacle.nodes:
                connection = Connection(node1, node2)
                if connection.validPath():
                    distance=np.sqrt((node1.x-node2.x)**2+(node1.y-node2.y)**2)
                    if(distance)<closestValue:
                        closestCombo=[node1,node2]
                        closestValue=distance
                    
        if(closestCombo==[]):
            return ()
        

        return closestCombo
    
        
    def connectPolygonObstacle(self, other_obstacle: 'PolygonObstacle'):
        self_node, other_node = self.findClosestNodes(other_obstacle)
        self_node.connectNode(other_node)
        self_connected_index = 0
        other_connected_index = 0
        
        for i in range(len(self.vertices)):
            if self.vertices[i][0] == self_node.x and self.vertices[i][1] == self_node.y:
                self_connected_index = i
                break
        for i in range(len(other_obstacle.vertices)):
            if other_obstacle.vertices[i][0] == other_node.x and other_obstacle.vertices[i][1] == other_node.y:
                other_connected_index = i
                break
        print("Extrema Walker 1")
        self.extremaWalker(other_obstacle, self_connected_index, other_connected_index, 1, -1)
        print("Extrema Walker 2")
        self.extremaWalker(other_obstacle, self_connected_index, other_connected_index, -1, 1)
        print("Extrema Walker 3")
        self.extremaWalker(other_obstacle, self_connected_index, other_connected_index, 1, 1)
        print("Extrema Walker 4")
        self.extremaWalker(other_obstacle, self_connected_index, other_connected_index, -1, -1)
    def extremaWalker(self, other_obstacle: 'PolygonObstacle', self_connected_index, other_connected_index, self_rotation_direction, other_rotation_direction):
        self_index = self_connected_index
        other_index = other_connected_index
        self_visited_vertices = set()
        other_visited_vertices = set()
        blocked = False
        self_turn = True

        while True:
            if self_turn:
                next_index = (self_index + self_rotation_direction) % len(self.vertices)
                self_index = next_index
            else:
                next_index = (other_index + other_rotation_direction) % len(other_obstacle.vertices)
                other_index = next_index

            self_point_to_check = self.vertices[self_index]
            other_point_to_check = other_obstacle.vertices[other_index]
            other_blocked = self.intersects((self_point_to_check, other_point_to_check))
            self_blocked = other_obstacle.intersects((self_point_to_check, other_point_to_check))
            new_blocked = other_blocked or self_blocked

            if self_turn:
                if new_blocked:
                    self_index = (self_index - self_rotation_direction) % len(self.vertices)
                    self_visited_vertices.add(other_index)
                else:
                    self_visited_vertices = set()
                self_turn = False
            else:
                if new_blocked:
                    other_index = (other_index - other_rotation_direction) % len(other_obstacle.vertices)
                    other_visited_vertices.add(self_index)
                else:
                    other_visited_vertices = set()
                self_turn = True

            if new_blocked and blocked:
                break
            blocked = new_blocked

            if len(self_visited_vertices) == len(self.vertices):
                break
            if len(other_visited_vertices) == len(other_obstacle.vertices):
                break

        for i in self.nodes:
            if i.x == self.vertices[self_index][0] and i.y == self.vertices[self_index][1]:
                self_node = i
                break
        for i in other_obstacle.nodes:
            if i.x == other_obstacle.vertices[other_index][0] and i.y == other_obstacle.vertices[other_index][1]:
                other_node = i
                break

        self_node.connectNode(other_node)

    @staticmethod
    def midPoint(vertex1, vertex2):
        return ((vertex1[0] + vertex2[0]) / 2, (vertex1[1] + vertex2[1]) / 2)

    def contains_point(self, point):
        return self.polygon.contains(Point(point))

    def intersects(self, line):
        line = LineString(line)
        return self.polygon.crosses(line)
