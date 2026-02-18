from flight.pathfinding.node_generation import Field, Mine, Node
from flight.pathfinding.path_calculation import Graph
from random import randint, seed
import math
import time
"""
Use this file for getting the node graph.
This will generate 10 (or however many you want) mines that are placed. 
The output will be at the bottom.
"""
# seed(2020) # make random or not
numMines = 20
radius = 32
pathFindingType = "none"  # dijkstra OR A* OR both OR none
xMin = -numMines*radius
xMax = numMines*radius
yMin = -numMines*radius
yMax = numMines*radius
field = Field(xMin,xMax,yMin,yMax)
genXMin = -radius*(numMines//4)
genXMax = radius*(numMines//4)
genYMin =-radius*(numMines//4)
genYMax = radius*(numMines//4)
position = [0,0]
mineGenTolerance = 0*radius

# Mine generation, do not add floating nodes before this point
for num in range(numMines):
    while True: # To make sure generated mines arent clipping off the edges of the field
        position[0], position[1] = randint(genXMin,genXMax+1),randint(genYMin,genYMax+1)
        invalidPosition = False
        for mine in Mine.mines:
            if (mine.getPos()[0] - mineGenTolerance <= position[0] <= mine.getPos()[0] + mineGenTolerance) and (mine.getPos()[1] - mineGenTolerance <= position[1] <= mine.getPos()[1] + mineGenTolerance):
                invalidPosition = True
                break
        if invalidPosition:
            continue
        if position[0] <= xMin + radius or position[0] >= xMax - radius or position[1] <= yMin + radius or position[1] >= yMax - radius:
            continue
        break
    field.addMine(position[0],position[1],radius)
    
    print("added a mine")
print("done adding mines\n")
start = field.placeStartNode(0,yMin + (radius*1.5))
endPoints = field.placeEndNodes(yMax - (radius*1.5),10)
for mine in field.mines:
    mine.connectMineNodes()
    print("Connecting mine nodes")
if pathFindingType == "dijkstra" or pathFindingType == "both":
    pathSolve = Graph(field.nodeGraph)
    temp = time.time()
    path = pathSolve.shortest_path(start,endPoints)
    dijkstraTime = time.time()-temp
    print("optimal path:",path)

    dijkstraPathLength = 0
    for i in range(len(path)-1):
       dijkstraPathLength += math.hypot((path[i].x-path[i+1].x),(path[i].y-path[i+1].y))


field.plotField()
field.increaseRadius(100)
field.plotField()

if pathFindingType == "A*" or pathFindingType == "both":
    def yMax(node):
        return (460-node.y)

    aStarPathSolve = Graph(field.nodeGraph)
    temp = time.time()
    aStarPath = aStarPathSolve.a_star(start,endPoints,yMax)
    aStarTime = time.time()-temp
    print("A* path:",path)


    aStarPathLength = 0
    for i in range(len(aStarPath)-1):
        aStarPathLength += math.hypot((aStarPath[i].x-aStarPath[i+1].x),(aStarPath[i].y-aStarPath[i+1].y))
    print(f" Best Path Length: {dijkstraPathLength} \n A* Path Length: {aStarPathLength} \n difference: {(aStarPathLength-dijkstraPathLength)}")
    print(f"A* is {(aStarPathLength/dijkstraPathLength)*100-100:.3f}% longer")
    print(f" Best Path Time: {dijkstraTime} \n A* time: {aStarTime} \n difference: {(dijkstraTime-aStarTime)}")
    print(f"A* is {(dijkstraTime/aStarTime-1):.1f} times faster")