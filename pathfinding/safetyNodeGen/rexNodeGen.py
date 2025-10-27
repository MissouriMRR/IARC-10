#appends a list with new mine coords of 2D array stored as coorrdinate points.
import numpy as np #for array functions
#each element of the array will have a x and y value.

      
mineCorList = [[x,y]] #2D array of mine coordinates.
mineCorList[0][0]  #x value of first mine
nodeCorList = [[]]
connectionTracker = [[]]  #primary node, secondary node, distance between nodes. integer index values that correpsonden to nodeCorList

def addMine(x, y): 
    mineCorList.append([x, y])
    return mineCorList

def addNode(x, y):
    nodeCorList.append([x, y])
    return nodeCorList

def addConnection(primaryNodeIndex, secondaryNodeIndex, distance):
    connectionTracker.append([primaryNodeIndex, secondaryNodeIndex, distance])
    return connectionTracker

