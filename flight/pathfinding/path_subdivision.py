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
    def ground_covered_image(altitude: float, fovDeg: float): 
        fovRad = m.radians(fovDeg)
        return 2 * altitude * m.tan(fovRad/2)
        
    
    #def generate_goto_points(self, nodeList: Node, step: int = 10):
    def generate_goto_points(self, nodeList: tuple[Node], overlap: float, altitude: float, fovDeg: float): #overlap is percent
        
        imageSize = self.ground_covered_image(altitude, fovDeg)
        step = imageSize * (1-overlap) # distance between goto points (FEET)
        #finalGotoList = []   
        #segmentedList = []
        isArc = False
        for i in range(len(nodeList) - 1):
            n1 = nodeList[i] #first node
            n2 = nodeList[i + 1] #second node in each iteration
            
            connect = Connection(n1,n2)
            # linear gotos or floating points
            #if n1.parentMine!=n2.parentMine or n1.floating or n2.floating:
            if connect.connectionType == seg.LINE:
                self.segmentedList.append(((float(n1.x), float(n1.y)), (float(n2.x), float(n2.y)), isArc))
    
                self.total_lin_distance += connect.distance
                numPoints = max(1, int(connect.distance / step)) 
                x_vals = np.linspace(n1.x, n2.x, numPoints)
                y_vals = np.linspace(n1.y, n2.y, numPoints)
                for x, y in zip(x_vals, y_vals):
                    self.finalGotoList.append((float(x), float(y)))
            
            #arc gotos
            elif connect.connectionType == seg.ARC:
                isArc = True
                self.segmentedList.append(((float(n1.x), float(n1.y)), (float(n2.x), float(n2.y)), isArc))
                
                #get center coords
                mine = n1.parentMine
                cx, cy = mine.getPos()
                r = mine.getRadius()

                # Compute angles of each node around the circle
                angle1 = n1.angle
                angle2 = n2.angle

                #Choose the smaller arc to follow
                delta_theta = angle2 - angle1
                if delta_theta > m.pi:
                    delta_theta -= 2*m.pi
                elif delta_theta < -m.pi:
                    delta_theta += 2*m.pi
                
                self.total_arc_length += connect.distance 
                
                numPoints = max(1, int(connect.distance/ step)) 
                
                # Generate arc points
                angles = np.linspace(angle1, angle1 + delta_theta, numPoints)
                for a in angles:
                    x = cx + r * m.cos(a)
                    y = cy + r * m.sin(a)
                    self.finalGotoList.append((float(x), float(y)))
                
        print(step)    
        self.total_path_length = self.total_distance + self.total_arc_length       
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
field = Field(0, 200, 0, 200)

field.addMine(80, 30, 20) 
field.addMine(70, 90, 20) 
field.addMine(140, 30, 20) 
field.addMine(170, 100, 20)

start=field.placeStartNode(110,0)
end=field.placeEndNodes(190,2)

for node in end:
    node.connectNode(start)
for mine in field.mines:
    print(mine,'connected to',','.join(m.__str__() for m in mine.connectedMines))
    mine.connectMineNodes()
    
nodeList = []


newGraph=Graph(field.nodeGraph)
nodeList=newGraph.shortest_path(start,end)
print(nodeList)

field.plotField(labeled=True)
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
finalPath, segmentedList = pathObj.generate_goto_points(nodeList, 0.3, 64)  

pathWidth = Mine.getRadius(Mine)
print("radius", pathWidth)
droneWeight = 1 #ounces over 1 pound weight limit
flightMin = 7  #worst case scenario
minesMissed = 0
optimumPath = pathObj.total_path_length
score = pathObj.num_score_points(flightMin, minesMissed, optimumPath, pathWidth, droneWeight)
print("This is the score: ", score)


#print("goto x and goto y")
# Extract x and y from finalGotoList (path) [(x1,y1), (x2,y2)]
goto_x = [coord[0] for coord in finalPath]
goto_y = [coord[1] for coord in finalPath]
#print(segmentedList)


#Display
fig, ax = plt.subplots()
ax.set_aspect('equal')

plt.plot(goto_x, goto_y, marker='o', color='red', linestyle='-', markersize=4, label='Goto Points')

for node in nodeList:
    plt.plot(node.x, node.y, marker='o', color='blue', markersize= 4, label='Node')

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Nodes and Goto Points")
plt.grid(True)
plt.show()


    
# TODO: LOOK at what mapping code has
    #ask about overlap with mapping team and if that is being accounted for? 
# is the given fov horizontal or vertical? I don't think it matter because using distance covered by image for points.
    # imageHeight = 2⋅h⋅tan(fovv​/2) 
