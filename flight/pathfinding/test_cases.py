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
numMines = 40
radius = 32
mineHistory = "["

# Paste a past list of mine coords as a string, paste the printed mineHistory
# If wanting to go back to randomized, leave it as empty list
recordedMineCoords =[(59,-10), (0,24), (-86,-48), (-13,-112), (140,40), (-140,-111), (-129,105), (-6,-92), (122,40), (-27,79), (17,108), (67,71), (-108,-101), (-122,104), (97,-29), (-134,-132), (16,29), (143,108), (143,-74), (110,-98)]

"""
Iteration of mine coordinates that a bug has appeared:

Bug in: Hugging Edges
List of mines coords:
[(59,-10), (0,24), (-86,-48), (-13,-112), (140,40), (-140,-111), (-129,105), (-6,-92), (122,40), (-27,79), (17,108), (67,71), (-108,-101), (-122,104), (97,-29), (-134,-132), (16,29), (143,108), (143,-74), (110,-98)]
"""

if (len(recordedMineCoords) > 0):
    numMines = len(recordedMineCoords)

pathFindingType = "A*"  # dijkstra OR A* OR both OR none 

stepDebug = False # True if you want to step through mines being added, 
                  # closing the generated window moves onto to the next step.
                  # NOTE:In order to fully end the program you need to run ctrl+C in the terminal
                  # or fully iterate through numMines times
labeled = False
if numMines >= 20:
    xMin = -numMines*radius
    xMax = numMines*radius
    yMin = -numMines*radius
    yMax = numMines*radius

    genXMin = -radius*(numMines//4)
    genXMax = radius*(numMines//4)
    genYMin =-radius*(numMines//4)
    genYMax = radius*(numMines//4)
else:
    xMin = -numMines*radius*4
    xMax = numMines*radius*4
    yMin = -numMines*radius*4
    yMax = numMines*radius*4

    genXMin = -radius*(numMines//2)
    genXMax = radius*(numMines//2)
    genYMin =-radius*(numMines//2)
    genYMax = radius*(numMines//2)

field = Field(xMin,xMax,yMin,yMax)
position = [0,0]
mineGenTolerance = 0*radius
step = 0

# Mine generation, do not add floating nodes before this point
for num in range(numMines):
    step += 1
    if len(recordedMineCoords) <= 0: # Run normally if no mine cords have been inputted
        if num != 0 and num != numMines:
            mineHistory += ", "
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
        mineHistory += "(" + str(position[0]) + "," + str(position[1]) + ")"
        print("added a mine")
    else:
        field.addMine(recordedMineCoords[num][0], recordedMineCoords[num][1], radius)
        print("added a mine")
    if stepDebug:
        field.plotField(labeled=labeled,xlabel="[ctr+c] in the terminal to force end the program.\n(If it doesn't close initially, focus on the generated window)",title="Mines and Potential Paths:\n" + "Number of Mines: " + str(step) + "/" + str(numMines))
    else:
        continue
print("done adding mines\n")

if len(recordedMineCoords) <= 0:
    mineHistory += "]"
    print(mineHistory)
start = field.placeStartNode(0,0.5*yMin)
endPoints = field.placeEndNodes(yMax - (radius*1.5),1) # A density of one defaults to the end node at (0,yVal)
#CONNECTS FLOATING NODES TOGETHER, DONT REMOVE
for node in endPoints:
    node.connectNode(start)

print("Connecting nodes on same mine (PLEASE DON'T REMOVE AGAIN I BEG)")
for mine in field.mines:
    mine.connectMineNodes()


dijkstraPathLength = 0
aStarPathLength = 0
if pathFindingType == "dijkstra" or pathFindingType == "both":
        print("Calculating dijkstra's")
        pathSolve = Graph(field.nodeGraph)
        temp = time.time()
        path = pathSolve.shortest_path(start,endPoints)
        dijkstraTime = time.time()-temp
        print("optimal path:",path)

        dijkstraPathLength = 0
        for i in range(len(path)-1):
            dijkstraPathLength += math.hypot((path[i].x-path[i+1].x),(path[i].y-path[i+1].y))

        print(f"Dijkstra path length {dijkstraPathLength}")
        print(f"Dijkstra path time {dijkstraTime}")
if pathFindingType == "A*" or pathFindingType == "both":
    print("Calculating A*")
    def yMax(node):
        return (460-node.y)

    aStarPathSolve = Graph(field.nodeGraph)
    temp = time.time()
    aStarPath = aStarPathSolve.a_star(start,endPoints,yMax)

    aStarTime = time.time()-temp
    print("A* path:",aStarPath)


    aStarPathLength = 0
    for i in range(len(aStarPath)-1):
        aStarPathLength += math.hypot((aStarPath[i].x-aStarPath[i+1].x),(aStarPath[i].y-aStarPath[i+1].y))
    print(f"A* path length {aStarPathLength}")
    print(f"A* path time {aStarTime}")
    
if pathFindingType=="both":
    print(f" Best Path Length Difference: {(aStarPathLength-dijkstraPathLength)}")
    print(f"A* is {(aStarPathLength/dijkstraPathLength)*100-100:.3f}% longer")
    print(f" Best Path Time: {dijkstraTime} \n A* time: {aStarTime} \n difference: {(dijkstraTime-aStarTime)}")
    print(f"A* is {(dijkstraTime/aStarTime-1):.1f} times faster")

if not stepDebug:
    field.plotField(path=aStarPath)

field.increaseRadius(100)
field.plotField()