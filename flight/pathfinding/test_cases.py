from flight.pathfinding.node_generation import Field, Mine, Node
from flight.pathfinding.path_calculation import Graph
from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer as coordCon
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
radius = 10
mineHistory = "["

# Paste a past list of mine coords as a string, paste the printed mineHistory
# If wanting to go back to randomized, leave it as empty list
recordedMineCoords = []

if (len(recordedMineCoords) > 0):
    numMines = len(recordedMineCoords)

pathFindingType = "A*"  # dijkstra OR A* OR both OR none 

stepDebug = False # True if you want to step through mines being added, 
                  # closing the generated window moves onto to the next step.
                  # NOTE:In order to fully end the program you need to run ctrl+C in the terminal and focus onto the graph window
                  # or fully iterate through numMines times

# Get converted dimensions of field

lat_lon1 = [36.021683, -95.941831] # *
lat_lon2 = [36.020694, -95.941856] # **
lat_lon3 = [36.021694, -95.942372] # ***
lat_lon4 = [36.020703, -95.942397]
labeled = False

converter = coordCon([lat_lon1,lat_lon2,lat_lon3,lat_lon4],360)
arbCorners = converter.get_arb_corners()
rawCorners = None # To be recieved later, represents field corners before normalizing to a rectangle
fieldSimCoords = {
    "xMin": arbCorners[0][0],
    "xMax": arbCorners[1][0],
    "yMin": arbCorners[3][1],
    "yMax": arbCorners[1][1]
}
genXMin = int(fieldSimCoords["xMin"])
genXMax = int(fieldSimCoords["xMax"])
genYMin = int(fieldSimCoords["yMin"])
genYMax = int(fieldSimCoords["yMax"])

field = Field(arbCorners, rawCorners)
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
            if position[0] <= fieldSimCoords["xMin"] + radius or position[0] >= fieldSimCoords["xMax"] - radius or position[1] <= fieldSimCoords["yMin"] + radius or position[1] >= fieldSimCoords["yMax"] - radius:
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
    print("Mine positions:")
    print(mineHistory)
    print()
start = field.placeStartNode(-5,-5)
endPoints = field.placeEndNodes(fieldSimCoords["yMax"],1) # A density of one defaults to the end node at (x range midpoint,yVal)
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

print(f"Increasing radius from {radius} to {radius*2}")
field.increaseRadius(radius*2)
field.plotField()