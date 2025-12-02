import math as m
import matplotlib.pyplot as plt
import random as rand
import numpy as np
from flight.pathfinding.rexAlg.nodeGen import Node, Mine, Field

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
field = Field(0, 200, 0, 200)
field.addMine(100, 100, 10)
field.addMine(100, 200, 10) #same mine to see arc

nodeList = []
for mine in field.mines:
    nodeList.extend(mine.getNodes())
  
#step for linear, arc_step for arc to determine number of gotopts.
def generateGotoPoints(nodeList, step = 10, arc_step=0.05): #arc step in radians
    finalGotoList = []

    for i in range(len(nodeList) - 1):
        n1 = nodeList[i] #first node
        n2 = nodeList[i + 1] #second node in each iteration

        pathType = n1.getPathType(n2)   # "line" or "arc". Don't need to know the rest of details for that node.

        # linear gotos
        if n1.parentMine!=n2.parentMine or n1.floating or n2.floating:
            x_vals = np.linspace(n1.x, n2.x, step)
            y_vals = np.linspace(n1.y, n2.y, step)

            for x, y in zip(x_vals, y_vals):
                finalGotoList.append((float(x), float(y)))
        elif n1.parentMine==n2.parentMine:
            #arc gotos
            print("there is an arc")
            #get center coords
            mine = n1.parentMine
            cx, cy = mine.getPos()
            r = mine.getRadius()

            # Compute angles of each node around the circle
            angle1 = n1.angle
            angle2 = n2.angle

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
                finalGotoList.append((float(x), float(y)))
            
        

    # Extract x and y from finalGotoList
    goto_x = [x for x, y in finalGotoList]
    goto_y = [y for x, y in finalGotoList]

    # Plot the goto points in red
    plt.plot(goto_x, goto_y, marker='o', color='red', linestyle='-', label='Goto Points')

    # Plot the nodes in blue
    for node in nodeList:
        plt.plot(node.x, node.y, marker='o', color='blue', markersize=8, label='Node')

    # Add labels and grid
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Nodes and Goto Points")
    plt.grid(True)
    plt.show()
    return finalGotoList

path = generateGotoPoints(nodeList)
print(path)
