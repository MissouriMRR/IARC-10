import sys, os
#sys.path.append(os.path.abspath(".."))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
import math as m
import numpy as np
import random as rand
import matplotlib.pyplot as plt
from flight.pathfinding.node_generation import Node, Mine, Field, Connection, seg
from flight.pathfinding.path_calculation import Graph
#purpose: takes in list of nodes outputted from dijkstras algorthm, and creates goto points between each node (arc/line) 
#with hardcoded lengths (arclength/step) between each point, outputting a final path of points for drone to follow.
#hello

#todo: Modify goto point generation to take the photo size and change the width between goto points as needed. 
#coordconvert finla path to actual coordinates
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

class Path: 
    def __init__(self):
        self.total_distance = 0  #linear distance
        self.total_arc_length = 0
        self.total_path_length = 0
        self.finalGotoList = []
        self.segmentedList = []
    
    
    #create setter functions for these variables
    #def set
        

    
    #def generate_goto_points(self, nodeList: Node, step: int = 10):
    def generate_goto_points(self, nodeList: Node, overlap: float, photoWidth: float): #overlap is percent
        step = photoWidth * (1-overlap) # distance between goto points (FEET)
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
                '''
                dx = n2.x - n1.x
                dy = n2.y - n1.y
                distance = m.sqrt(dx**2 + dy**2)
                '''
                self.total_distance += connect.distance
                
                numPoints = max(1, int(connect.distance / step)) # 1 is the min num of points --> subject to change.
                x_vals = np.linspace(n1.x, n2.x, numPoints)
                y_vals = np.linspace(n1.y, n2.y, numPoints)

                for x, y in zip(x_vals, y_vals):
                    self.finalGotoList.append((float(x), float(y)))
            
            #arc gotos
            #elif n1.parentMine==n2.parentMine :
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
                
                #determine number of points based on arc length
                #arc_length = abs(delta_theta) * r 
                self.total_arc_length += connect.distance #arc_length
                
                numPoints = max(1, int(connect.distance/ step)) # 3 is the min num of points --> subject to change. #note to self: If I had done delta_theta/(pre-determined radian value), it would uniformly space number of points based on angle, but as radius changes, distance between generated points would vary. 
                
                # Generate arc points
                angles = np.linspace(angle1, angle1 + delta_theta, numPoints)
                for a in angles:
                    x = cx + r * m.cos(a)
                    y = cy + r * m.sin(a)
                    self.finalGotoList.append((float(x), float(y)))
                    
      
        #print("segment List:", segmentList)  
        print(step)    
        self.total_path_length = self.total_distance + self.total_arc_length       
        return self.finalGotoList, self.segmentedList 
    
    #optimumPath: path center-line length in feet
    #pathWidth: narrowest width of path in feet
    def numScorePoints(self, flightMin: int, minesMissed: int, optimumPath: float, pathWidth: float, droneWeight: float ): 
        if (flightMin > 7): 
            score = 0
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

#Function Calls
pathObj = Path()
finalPath, segmentedList = pathObj.generate_goto_points(nodeList, 0.3, 64)  

pathWidth = Mine.getRadius(Mine)
print("radius", pathWidth)
droneWeight = 1 #ounces over 1 pound weight limit
flightMin = 7  #worst case scenario
minesMissed = 0
optimumPath = pathObj.total_path_length
score = pathObj.numScorePoints(flightMin, minesMissed, optimumPath, pathWidth, droneWeight)
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


    
    
    

