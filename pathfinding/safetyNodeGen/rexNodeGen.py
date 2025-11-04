import math as m
import numpy as np
import turtle as t

t.speed(0)
t.setworldcoordinates(0, 0, 3937, 3937)
t.penup()
screen = t.Screen()
screen.tracer(0)
     
     
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

#Next Steps:
# use feet to represent distances.
# you have Coordinates of two nodes, generate positions that are equidistant from each other along the path between two nodes. create a 
# function using matplotlib, that creates points along the distance, line or arc. 
#plug in two coordinate points, and a distance value in function, and the function returns a list of coordinates along the path between the two nodes, that are the distance value apart from each other.

#linspace method returns evenly spaced numbers over a specific interval.

#for testing
np.random.seed(0)

start_point = (1, 1)
end_point = (5, 5)

#x_values = np.linspace(start_x, stop_x, num_points_x)
#y_values = np.linspace(start_y, stop_y, num_points_y)