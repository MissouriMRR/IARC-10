from flight.pathfinding.node_generation import Field, Mine, Node
from flight.pathfinding.path_calculation import Graph
from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer as coordCon
from random import randint, seed, uniform
import math
import time
"""
Use this file for getting the node graph.
This will generate 10 (or however many you want) mines that are placed. 
The output will be at the bottom.
"""
# seed(2020) # make random or not
numMines = 150
radius = 10
mineHistory = "["

# Paste a past list of mine coords as a string, paste the printed mineHistory outputted previously
# If wanting to go back to randomized, leave it as empty list7
recordedMineCoords = [(126,70), (78,128), (335,96), (16,16), (159,118), (55,13), (323,59), (231,45), (193,138), (277,88), (154,86), (138,68), (289,41), (348,27), (342,49), (162,127), (246,105), (171,41), (266,127), (39,11), (128,76), (292,14), (324,33), (206,64), (190,97), (156,32), (95,140), (34,38), (236,54), (162,17), (182,53), (314,96), (223,67), (325,111), (111,50), (255,88), (300,79), (318,141), (43,47), (194,139), (19,49), (280,101), (221,129), (91,95), (85,73), (132,37), (159,110), (73,72), (346,16), (134,130), (195,61), (230,39), (285,130), (132,109), (260,94), (90,123), (253,97), (152,45), (96,50), (242,22), (103,15), (219,59), (30,13), (227,77), (251,98), (114,32), (105,88), (11,91), (89,134), (207,105), (146,43), (243,111), (273,45), (44,96), (26,15), (67,29), (273,99), (68,61), (142,28), (297,70), (302,129), (58,112), (259,110), (76,62), (312,47), (209,137), (140,134), (180,44), (166,97), (165,46), (306,130), (60,141), (57,82), (135,76), (299,107), (334,86), (27,75), (334,40), (95,66), (218,79), (51,35), (152,72), (339,80), (339,109), (305,128), (159,138), (315,129), (171,42), (67,72), (137,72), (164,72), (216,34), (321,147), (128,111), (82,32), (190,76), (174,127), (345,83), (25,41), (254,38), (79,75), (26,120), (262,124), (181,104), (166,78), (172,58), (327,143), (45,70), (322,138), (224,141), (32,119), (78,138), (201,121), (30,33), (80,66), (231,123), (102,44), (34,97), (218,130), (56,122), (303,41), (340,17), (314,16), (189,66), (287,86), (314,51), (177,91), (175,74), (191,83), (254,24)]
"""
Rather interesting path found (Not neccesarily wrong):
Mine Positions:
[(126,70), (78,128), (335,96), (16,16), (159,118), (55,13), (323,59), (231,45), (193,138), (277,88), (154,86), (138,68), (289,41), (348,27), (342,49), (162,127), (246,105), (171,41), (266,127), (39,11), (128,76), (292,14), (324,33), (206,64), (190,97), (156,32), (95,140), (34,38), (236,54), (162,17), (182,53), (314,96), (223,67), (325,111), (111,50), (255,88), (300,79), (318,141), (43,47), (194,139), (19,49), (280,101), (221,129), (91,95), (85,73), (132,37), (159,110), (73,72), (346,16), (134,130), (195,61), (230,39), (285,130), (132,109), (260,94), (90,123), (253,97), (152,45), (96,50), (242,22), (103,15), (219,59), (30,13), (227,77), (251,98), (114,32), (105,88), (11,91), (89,134), (207,105), (146,43), (243,111), (273,45), (44,96), (26,15), (67,29), (273,99), (68,61), (142,28), (297,70), (302,129), (58,112), (259,110), (76,62), (312,47), (209,137), (140,134), (180,44), (166,97), (165,46), (306,130), (60,141), (57,82), (135,76), (299,107), (334,86), (27,75), (334,40), (95,66), (218,79), (51,35), (152,72), (339,80), (339,109), (305,128), (159,138), (315,129), (171,42), (67,72), (137,72), (164,72), (216,34), (321,147), (128,111), (82,32), (190,76), (174,127), (345,83), (25,41), (254,38), (79,75), (26,120), (262,124), (181,104), (166,78), (172,58), (327,143), (45,70), (322,138), (224,141), (32,119), (78,138), (201,121), (30,33), (80,66), (231,123), (102,44), (34,97), (218,130), (56,122), (303,41), (340,17), (314,16), (189,66), (287,86), (314,51), (177,91), (175,74), (191,83), (254,24)]
"""

if (len(recordedMineCoords) > 0):
    numMines = len(recordedMineCoords)

pathFindingType = "dijkstra"  # dijkstra OR A* OR both OR none 
def calcAstar(field,start,endPoints):
    print("Calculating A*")
    def yMax(node):
        return (460-node.y)

    aStarPathSolve = Graph(field.nodeGraph)
    temp = time.time()
    aStarPath = aStarPathSolve.a_star(start,endPoints,yMax)

    aStarTime = time.time()-temp
    
    aStarPathLength = 0
    for i in range(len(aStarPath)-1):
        aStarPathLength += math.hypot((aStarPath[i].x-aStarPath[i+1].x),(aStarPath[i].y-aStarPath[i+1].y))
    return aStarPath,aStarPathLength,aStarTime

stepDebug = False # True if you want to step through mines being added, 
                  # closing the generated window moves onto to the next step.
                  # NOTE:In order to fully end the program you need to run ctrl+C in the terminal and focus onto the graph window
                  # or fully iterate through numMines times

# Get converted dimensions of field

lat_lon1 = [36.021683, -95.941831] # *
lat_lon1_alt = [36.021695, -95.941831]
lat_lon2 = [36.020694, -95.941856] # **
lat_lon3 = [36.021694, -95.942372] # ***
lat_lon4 = [36.020703, -95.942397]

# converter = coordCon([lat_lon1,lat_lon2,lat_lon3,lat_lon4],360)
converter = coordCon([lat_lon1_alt, lat_lon2, lat_lon3, lat_lon4], 360)

arbCorners = converter.get_arb_corners()
# sim_field_size = [width, height]
sim_field_size = [max([arbCorners[0][0], arbCorners[1][0], arbCorners[2][0], arbCorners[3][0]]) - min([arbCorners[0][0], arbCorners[1][0], arbCorners[2][0], arbCorners[3][0]]), max([arbCorners[0][1], arbCorners[1][1], arbCorners[2][1], arbCorners[3][1]]) - min([arbCorners[0][1], arbCorners[1][1], arbCorners[2][1], arbCorners[3][1]])]
simCorners = [(0,sim_field_size[1]),
              (sim_field_size[0],sim_field_size[1]),
              (0,0),
              (sim_field_size[0],0)]
fieldSimCoords = {
    "xMin": simCorners[0][0],
    "xMax": simCorners[1][0],
    "yMin": simCorners[3][1],
    "yMax": simCorners[1][1]
}
genXMin = int(fieldSimCoords["xMin"])
genXMax = int(fieldSimCoords["xMax"])
genYMin = int(fieldSimCoords["yMin"])
genYMax = int(fieldSimCoords["yMax"])

field = Field(sim_field_size, arbCorners)

position = [0,0]
mineGenTolerance = 0*radius
step = 0
labeled = False
# Mine generation, do not add floating nodes before this point
"""
Note to self:
Though mines can be right at the edge of the field,
only their nodes will be checked for elimination
"""
start=time.time()
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
print(f"Mine generation time: {time.time()-start}")
if len(recordedMineCoords) <= 0:
    mineHistory += "]"
    print("Mine positions:")
    print(mineHistory)
    print()

# start = field.placeStartNode((simCorners[2][0]+simCorners[3][0])/2,((simCorners[2][1]-simCorners[3][1])/2)-5)
start = field.placeStartNode(125.0,2.0)
# Randomize endpoints, place endpoints along a horizontal line, or place endpoints manually
# endPoints = field.placeEndNodesLine(fieldSimCoords["yMax"],1) # A density of one defaults to the end node at (x range midpoint,yVal)
# endPoints = field.placeEndNodesPositions([(uniform(simCorners[0][0],simCorners[1][0]),simCorners[1][1]+5)])
# endPoints = field.placeEndNodesPositions([(start.x,((simCorners[0][1]+simCorners[1][1])/2)+5)])
endPoints = field.placeEndNodesPositions([(125,248)])
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
else:
    path = []
if pathFindingType == "A*" or pathFindingType == "both":
    aStarPath, pathLength, aStarTime = calcAstar(field,start, endPoints)
    print("A* path:",aStarPath)
    print(f"A* path length {pathLength}")
    print(f"A* path time {aStarTime}")
else:
    aStarPath = []
if pathFindingType=="both":
    print(f" Best Path Length Difference: {(aStarPathLength-dijkstraPathLength)}")
    print(f"A* is {(aStarPathLength/dijkstraPathLength)*100-100:.3f}% longer")
    print(f" Best Path Time: {dijkstraTime} \n A* time: {aStarTime} \n difference: {(dijkstraTime-aStarTime)}")
    print(f"A* is {(dijkstraTime/aStarTime-1):.1f} times faster")

if not stepDebug:
    if pathFindingType == "A*":
        print("Before A*")
        field.plotField()
        print("After A*")
        field.plotField(path=aStarPath)
    elif pathFindingType == "dijkstra":
        field.plotField(path=path)

# oldPath = aStarPath
# print(f"Increasing radius from {radius} to {radius+1}")
# field.increaseRadius(1)
# newPath, pathLength, aStarTime = calcAstar(field,start, endPoints)
# print("A* path:",newPath)
# print(f"A* path length {pathLength}")
# print(f"A* path time {aStarTime}")
# field.plotField(path=newPath,pastPath=oldPath)