import math as m
import matplotlib.pyplot as plt
import random as rand
import numpy as np
#from rexAlg import nodeGen as ng

#todo
#function takes in nodeCorlist and then creates goto points between each pair in list. 
#between every node point, print/append goto points to final goto points path list that is returned at the end. 
#constantly check for path type, and implement arc gotos. 
#steps:
#1. visual output should show all the nodes and goto points between them on screen. 
#2. work on appending to final goto-list after going through nodecorlist and return it.
#3. check node path type everytime, and if arc, create arc gotos, if line, use linspace.

#arc length: angle * radius of circle  (16)

#have a list of mine coordinates and then put a check if 

plt.xlim(0, 200)
plt.ylim(0, 200)

mx1 = rand.randrange(200) #1st node coordinate
my1 = rand.randrange(200)  

mx2 = rand.randrange(200) #2nd node coordinates
my2 = rand.randrange(200)  

def gotoPath(nodeCorList):
    (mx1, my1) = nodeCorList[0]
        
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
 
 
nodeCorList = [[mx1,my1], #node 1
               [mx2,my2]]  #node 2

print(gotoPath(nodeCorList))




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