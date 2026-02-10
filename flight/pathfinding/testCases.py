from flight.pathfinding.genNodesFromMines import Field, Mine, Node
from flight.pathfinding.genPathFromNodes import Graph
from random import randint, seed
import math
import time
"""
Use this file for getting the node graph.
This will generate 50 mines that are manually placed. 
The output will be at the bottom.
"""
field = Field(-500,500,-500,500)
radius = 50
position = [0,0]


numMines = 40 # Only adjust this for now.
# seed(10) # Comment out or dont for randomness.


for num in range(numMines):
    while True: # To make sure generated mines arent clipping off the edges of the field
        position[0], position[1] = randint(-450,450+1),randint(-400,400+1)
        invalidPosition = False
        for mine in Mine.mines:
            if (mine.getPos()[0] <= position[0] <= mine.getPos()[0]) and (mine.getPos()[1] <= position[1] <= mine.getPos()[1]):
                invalidPosition = True
                break
        if invalidPosition:
            continue
        if position[0] <= -500 + radius or position[0] >= 500 - radius or position[1] <= -500 + radius or position[1] >= 500 - radius:
            continue
        break
    field.addMine(position[0],position[1],radius)
    print("added a mine")

print("done adding mines, connecting nodes on min")

start = field.placeStartNode(0,-460)
endPoints = field.placeEndNodes(460,10)
field.cleanNodeGraph()
pathSolve = Graph(field.nodeGraph)
temp = time.time()
path = pathSolve.shortest_path(start,endPoints)
dijkstraTime = time.time()-temp
print("optimal path:",path)
#field.plotField()

def yMax(node):
    return (460-node.y)

aStarPathSolve = Graph(field.nodeGraph)
temp = time.time()
aStarPath = aStarPathSolve.a_star(start,endPoints,yMax)
aStarTime = time.time()-temp
print("A* path:",path)

dijkstraPathLength = 0
for i in range(len(path)-1):
    dijkstraPathLength += math.hypot((path[i].x-path[i+1].x),(path[i].y-path[i+1].y))

aStarPathLength = 0
for i in range(len(aStarPath)-1):
    aStarPathLength += math.hypot((aStarPath[i].x-aStarPath[i+1].x),(aStarPath[i].y-aStarPath[i+1].y))
print(f" Best Path Length: {dijkstraPathLength} \n A* Path Length: {aStarPathLength} \n difference: {(aStarPathLength-dijkstraPathLength)}")
print(f"A* is {(aStarPathLength/dijkstraPathLength)*100-100:.3f}% longer")
print(f" Best Path Time: {dijkstraTime} \n A* time: {aStarTime} \n difference: {(dijkstraTime-aStarTime)}")
print(f"A* is {(dijkstraTime/aStarTime-1):.1f} times faster")