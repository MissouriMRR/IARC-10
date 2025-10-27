#appends a list with new mine coords of 2D array stored as coorrdinate points.
import numpy as np #for array functions
#each element of the array will have a x and y value.

      
mineCorList = [[6,7]] #2D array of mine coordinates.
nodeCorList = [[1,1]]
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

