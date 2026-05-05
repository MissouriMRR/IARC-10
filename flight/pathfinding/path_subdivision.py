import sys, os
#sys.path.append(os.path.abspath(".."))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
import math as m
import numpy as np
import random as rand
import matplotlib.pyplot as plt
from flight.pathfinding.node_generation import Node, Mine, Field, Connection, seg
from flight.pathfinding.path_calculation import Graph
'''
Attributes:
        total_lin_distance (float) : Cumulative linear distance of the path (feet)
        total_arc_length (float)   : Cumulative arc distance of the path (feet)
        total_path_length (float)  : Sum of total_distance and total_arc_length (feet)
        finalGotoList (list[tuple]): List of (x, y) coordinates representing waypoints [(x1,y1), (x2,y2)] 
        segmentedList (list[tuple]): List of segments [((x1, y1), (x2, y2), isArc)]

    Methods:
        ground_covered_image(altitude: float, fovDeg: float) -> float
            Calculates ground distance covered by a single drone image
            given camera FOV and altitude (feet).

        generate_goto_points(nodeList: tuple[Node], overlap: float, altitude: float, fovDeg: float) -> [list[tuple], list[tuple]]
            Generates waypoints along a path connecting nodes.
            Handles both linear and arc segments.
            Populates finalGotoList and segmentedList.

        num_score_points(flightMin: int, minesMissed: int, optimumPath: float, pathWidth: float, droneWeight: float) -> float
            Computes a performance score based on flight time, mines missed, path length,
            path width, and drone weight.

'''

class Path: 
    def __init__(self):
        self.total_lin_distance = 0  #linear distance
        self.total_arc_length = 0
        self.total_path_length = 0
        self.finalGotoList = []
        self.segmentedList = []
        
    #altitude in feet
    #returns distance covered by image in feet.
    def ground_covered_image(self, altitude: float, fovDeg: float): 
        fovRad = m.radians(fovDeg)
        return 2 * altitude * m.tan(fovRad/2)
        

    #overlap is percent
    def generate_goto_points(self, nodeList: tuple[Node], altitude: float, fovDeg: float, path_width: float, vertical_image_overlap: float = 0.1, horizontal_image_overlap: float = 0.1, scan_edge_overlap: float = 0.3, standardDist: float = 0.0):
        
        image_size = self.ground_covered_image(altitude, fovDeg)
        # number of side by side waypoints
        x_temp = m.ceil((path_width-image_size*(horizontal_image_overlap+2*scan_edge_overlap))/(image_size*(1-horizontal_image_overlap)))
        num_image = x_temp + (x_temp + 1) % 2 # Makes sure number of waypoints is on in order to have a waypoint on the path
        step = image_size * (1-vertical_image_overlap) # distance between goto points (FEET)
        horizontal_separation = image_size - horizontal_image_overlap*image_size
        #finalGotoList = []
        #segmentedList = []
        
        for i in range(len(nodeList) - 1):
            isArc = False
            n1 = nodeList[i] #first node
            n2 = nodeList[i + 1] #second node in each iteration
            
            connect = Connection(n1,n2)
            # linear gotos or floating points
            #if n1.parentMine!=n2.parentMine or n1.floating or n2.floating:
            if connect.connectionType == seg.LINE:
    
                self.total_lin_distance += connect.distance
                numPoints = max(1, m.ceil(connect.distance / step))
                x_vals = np.linspace(n1.x, n2.x, numPoints)
                y_vals = np.linspace(n1.y, n2.y, numPoints) 
                for x, y in zip(x_vals, y_vals):
                    #distance variable that has the distance between each point, so you can use that ot maintain space in arc nodes spacing.
                    standardDist = m.dist((n1.x, n1.y), (n2.x, n2.y)) #used to compare dist in arc as well
                    path_angle = m.atan2(n2.y - n1.y, n2.x - n1.x)
                    node_addition_angle = path_angle + m.pi/2
                    direction_unit_vector = [m.cos(node_addition_angle), m.sin(node_addition_angle)]
                    adjusted_vector = [a * horizontal_separation for a in direction_unit_vector]
                    self.finalGotoList.append((float(x), float(y), path_angle))
                    self.segmentedList.append([(n1), (n2), isArc])
                    
                    for i in range(1, (num_image//2) + 1):
                        self.finalGotoList.append((float(x) + i*adjusted_vector[0], float(y) + i*adjusted_vector[1], path_angle))
                        self.segmentedList.append([(n1), (n2), isArc])
                        self.finalGotoList.append((float(x) - i*adjusted_vector[0], float(y) - i*adjusted_vector[1], path_angle))
                        self.segmentedList.append([(n1), (n2), isArc])
            
            #arc gotos
            elif connect.connectionType == seg.ARC:
                isArc = True
                
                #get center coords
                mine = n1.parentMine
                cx, cy = mine.getPos()
                r = mine.getRadius()

                # Compute angles of each node around the circle
                angle1 = m.atan2(n1.y - cy, n1.x - cx)
                angle2 = m.atan2(n2.y - cy, n2.x - cx)

                #Choose the smaller arc to follow
                delta_theta = angle2 - angle1
                if delta_theta > m.pi:
                    delta_theta -= 2*m.pi
                elif delta_theta < -m.pi:
                    delta_theta += 2*m.pi
                
                self.total_arc_length += connect.distance 
                
                numPoints = max(1, m.ceil(connect.distance/ step)) 
                
                # Generate arc points
                angles = np.linspace(angle1, angle1 + delta_theta, numPoints)
                
                last_point = None
                
                for a in angles:
                    x = cx + r * m.cos(a)
                    y = cy + r * m.sin(a)
                
                    current_point = (x, y)

                    if last_point is not None:
                        if m.dist(current_point, last_point) < standardDist:
                            continue

                    last_point = current_point
                  
                    path_angle = angle1 + delta_theta + m.pi/2
                    node_addition_angle = path_angle + m.pi/2
                    direction_unit_vector = [m.cos(node_addition_angle), m.sin(node_addition_angle)]
                    adjusted_vector = [a * horizontal_separation for a in direction_unit_vector]

                    self.finalGotoList.append((float(x), float(y), path_angle))
                    self.segmentedList.append([(n1), (n2), isArc])
                    
                    for i in range(1, (num_image//2) + 1):
                        self.finalGotoList.append((float(x) + i*adjusted_vector[0], float(y) + i*adjusted_vector[1], path_angle))
                        self.segmentedList.append([(n1), (n2), isArc])
                        self.finalGotoList.append((float(x) - i*adjusted_vector[0], float(y) - i*adjusted_vector[1], path_angle))
                        self.segmentedList.append([(n1), (n2), isArc])
                
        self.total_path_length = self.total_lin_distance + self.total_arc_length       
        return self.finalGotoList, self.segmentedList 
    
    #optimumPath: path center-line length in feet
    #pathWidth: narrowest width of path in feet
    def num_score_points(self, flightMin: int, minesMissed: int, optimumPath: float, pathWidth: float, droneWeight: float ): 
        if (flightMin > 7): 
            score = 0
        else:
            score = ((150000 * pathWidth) / ((1 + minesMissed) * optimumPath * (1 + 7 * flightMin + (100 * droneWeight))))
        return score


#set up 
if __name__ == "__main__":
        
    field = Field([200, 200], [[0, 200], [200, 200], [0, 0], [200, 0]])

    field.addMine(80, 30, 20) 
    field.addMine(70, 90, 20) 
    field.addMine(140, 30, 20) 
    field.addMine(170, 100, 20)

    start=field.placeStartNode(110,0)
    end=field.placeEndNodesPositions([(180, 125)])

    for node in end:
        node.connectNode(start)
    for mine in field.mines:
        print(mine,'connected to',','.join(m.__str__() for m in mine.connectedMines))
        mine.connectMineNodes()
        
    nodeList = []


    newGraph=Graph(field.nodeGraph)
    nodeList=newGraph.shortest_path(start,end)
    print(nodeList)

    #field.plotField(labeled=True)
    """
    for mine in field.mines:
        nodeList.extend(mine.getNodes()) # gives all the nodes generated by adding the mines. 
    """

    '''
    #make-shift mines for testing. radius = 3 feet
    mine1 = Mine(rand.randrange(200), rand.randrange(200), 3)
    mine2 = Mine(rand.randrange(200), rand.randrange(200), 3)
    mine3 = Mine(rand.randrange(200), rand.randrange(200), 3)
    mine4 = Mine(rand.randrange(200), rand.randrange(200), 3)
    mine5 = Mine(rand.randrange(200), rand.randrange(200), 3)

    n1 = Node(mine1, mine2)
    n2 = Node(mine2, mine3)
    n3 = Node(mine3, mine4)
    n4 = Node(mine4, mine5)

    nodeCorList = [n1, n2, n3, n4]
    '''

    #Function Calls
    pathObj = Path()
    finalPath, segmentedList = pathObj.generate_goto_points(nodeList, 64, 10, 30)  

    pathWidth = Mine.getRadius(Mine)
    print("radius", pathWidth)
    droneWeight = 1 #ounces over 1 pound weight limit
    flightMin = 7  #worst case scenario
    minesMissed = 0
    optimumPath = pathObj.total_path_length
    #score = pathObj.num_score_points(flightMin, minesMissed, optimumPath, pathWidth, droneWeight)
    #print("This is the score: ", score)


    #print("goto x and goto y")
    # Extract x and y from finalGotoList (path) [(x1,y1), (x2,y2)]
    goto_x = [coord[0] for coord in finalPath]
    goto_y = [coord[1] for coord in finalPath]
    #print(segmentedList)


    #Display
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    plt.xlim(-10,210)
    plt.ylim(-10,210)

    plt.plot(goto_x, goto_y, marker='*', color='red', linestyle='None', markersize=4, label='Goto Points')

    for node in nodeList:
        plt.plot(node.x, node.y, marker='o', color='blue', markersize= 4, label='Node')

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Nodes and Goto Points")
    plt.grid(True)
    #plt.show()
    field.plotField(labeled=True)
