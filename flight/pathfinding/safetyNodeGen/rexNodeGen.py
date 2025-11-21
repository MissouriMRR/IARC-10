import math as m
import matplotlib.pyplot as plt
import random as rand
import numpy as np
from rexAlg.nodeGen import Node, Mine
#todo
#function takes in nodeCorlist and then creates goto points between each pair in list. 
#between every node point, print/append goto points to final goto points path list that is returned at the end. 
#constantly check for path type, and implement arc gotos. 
#steps:
#1. visual output should show all the nodes and goto points between them on screen. 
#2. work on appending to final goto-list after going through nodecorlist and return final goto-list.
#3. check node path type everytime, and if arc, create arc gotos, if line, use linspace.

#arc length: angle * radius of circle  (16 , subject to change)

#have a list of mine coordinates and then put a check if 

plt.xlim(0, 200)
plt.ylim(0, 200)

#make-shift mines for testing. radius = 3 feet
mine1 = Mine(rand.randrange(200), rand.randrange(200), 3)
mine2 = Mine(rand.randrange(200), rand.randrange(200), 3)
mine3 = Mine(rand.randrange(200), rand.randrange(200), 3)
mine4 = Mine(rand.randrange(200), rand.randrange(200), 3)
n1 = Node(mine1, mine2)
n2 = Node(mine3, mine4)
nodeCorList = [n1, n2]

'''
mx1 = rand.randrange(200) #1st node coordinate
my1 = rand.randrange(200)  

mx2 = rand.randrange(200) #2nd node coordinates
my2 = rand.randrange(200)  

nodeCorList = [[mx1,my1], #node 1
               [mx2,my2]]  #node 2
'''
#step for linear, arc_step for arc to determine number of gotopts.
def generateGotoPoints(nodeCorList, step = 10, arc_step=0.05):
    finalGotoList = []

    # Gets angle of a node around a center (mine coord)
    def angle_of_point(px, py, cx, cy):
        return m.atan2(py - cy, px - cx) #atan2 because it identifies the right quadrant and has inbuilt undefined checks.

    for i in range(len(nodeCorList) - 1):
        n1 = nodeCorList[i] #first node
        n2 = nodeCorList[i + 1] #second node in each iteration

        pathType = n1.getPathType(n2)[0]   # "line" or "arc". Don't need to know the rest of details for that node.

        # linear gotos
        if pathType == "line":
            x_vals = np.linspace(n1.x, n2.x, step)
            y_vals = np.linspace(n1.y, n2.y, step)

            for x, y in zip(x_vals, y_vals):
                finalGotoList.append((x, y))

        #arc gotos
        #get center coords
        mine = n1.parentMine
        cx, cy = mine.getPos()
        r = mine.getRadius()

        # Compute angles of each node around the circle
        angle1 = angle_of_point(n1.x, n1.y, cx, cy)
        angle2 = angle_of_point(n2.x, n2.y, cx, cy)

        #Choose the smaller arc
        delta_theta = angle2 - angle1

        if delta_theta > m.pi:
            delta_theta -= 2*m.pi
        elif delta_theta < -m.pi:
            delta_theta += 2*m.pi
        
        
        numPoints = max(20, int(abs(delta_theta) / arc_step)) # 20 is the min num of points --> subject to change.
        
        # Generate arc points
        angles = np.linspace(angle1, angle1 + delta_theta, numPoints)
        for a in angles:
            x = cx + r * m.cos(a)
            y = cy + r * m.sin(a)
            finalGotoList.append((x, y))

        else:
            print("WARNING: Unknown path type:", pathType)
            continue

    return finalGotoList

path = generateGotoPoints([n1, n2])
print(path)



'''
def generateGotoPts(nodeCorList, step, arc_step ):
    finalGotoList = []

    #distance between node calculations
    distance = int((m.sqrt((mx2 - mx1)**2 + (my2 - my1)**2))/10)

    #endpoint = false removes last node point in x & y array.
    x_values = np.linspace(mx1, mx2, num = distance, endpoint = False) #gives array of x_values coords= [mx1, mx2]
    y_values = np.linspace(my1, my2, num = distance, endpoint = False) 

    #creates tuple of first node point and goto points [(x1,y1),...etc] 
    allPts = list(zip(x_values, y_values))

    #Gets rid of first node point, so now gotoPts does not have either node coords.
    gotoPts = allPts[1:] 

    #plots generated goto points.
    for x, y in gotoPts:  
        plt.plot(x, y, marker='o', linestyle='-', color='red', markersize=8, markerfacecolor='red')

    #nodes plotted to show up
    plt.plot(mx1, my1, marker='o', linestyle='-', color='blue', markersize=8, markerfacecolor='blue')
    plt.plot(mx2, my2, marker='o', linestyle='-', color='blue', markersize=8, markerfacecolor='blue')

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("2 coordinates and generated points")
    plt.grid(True)

    plt.show()
    
    #return list of points in between.
    return [[float(x), float(y)] for x, y in gotoPts]
 

print(generateGotoPts(nodeCorList))
'''



'''
mineCorList = [[6,7], [7,9]] #2D array of mine coordinates.
connectionTracker = [[1,2,3]]  #primary node, secondary node, distance between nodes. integer index values that correpsonden to nodeCorList
def addMine(x, y): 
    mineCorList.append([x, y])
    return mineCorList

def addNode(x, y):
    nodeCorList.append([x, y])
    return nodeCorList

def addConnection(primaryNodeIndex, secondaryNodeIndex, distance):
    connectionTracker.append([primaryNodeIndex, secondaryNodeIndex, distance])
    return connectionTracker
'''