import math as m
import matplotlib.pyplot as plt

import numpy as np
import turtle as t


t.speed(0)
t.setworldcoordinates(0, 0, 3937, 3937)
t.penup()
screen = t.Screen()
screen.tracer(0)
     
     
mineCorList = [[6,7]] #2D array of mine coordinates.
nodeCorList = [[1,1],[2,1]]
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

start_x = nodeCorList[0][0]
start_y = nodeCorList[0][1]

stop_x = nodeCorList[1][0] 
stop_y = nodeCorList[1][1]

#arrays of goto points
x_values = np.linspace(start_x, stop_x, 5) #gives array of 2 x coor --> start and end --> x_values = [start_x, stop_x]
y_values = np.linspace(start_y, stop_y, 5) # gives array of 2 y coor --> start and end

plt.plot(start_x, start_y, marker='o', linestyle='-', color='blue', markersize=8, markerfacecolor='red')
plt.plot(stop_x, stop_y, marker='o', linestyle='-', color='blue', markersize=8, markerfacecolor='red')

#creates [(x1,y1),...etc]
gotoPts = zip(x_values, y_values)

#plots generated goto points.
for x, y in gotoPts: 
    plt.plot(x, y, marker='o', linestyle='-', color='blue', markersize=8, markerfacecolor='red')

plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("2 coordinates and generated point")

plt.show()