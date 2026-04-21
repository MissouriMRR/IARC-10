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
# If wanting to go back to randomized, leave it as empty list
recordedMineCoords = []
"""
[(338,54), (228,20), (126,27), (280,123), (146,140), (190,28), (91,66), (87,39), (210,32), (295,14), (180,121), (83,85), (51,128), (23,11), (218,116), (127,105), (304,89), (239,53), (101,69), (225,117), (122,22), (98,115), (81,62), (77,31), (48,115), (216,121), (290,55), (317,140), (187,99), (142,76), (237,71), (315,133), (319,126), (174,73), (229,31), (305,124), (151,64), (339,95), (27,69), (137,111), (65,116), (25,75), (294,22), (343,70), (12,14), (268,37), (285,82), (26,104), (173,54), (133,128), (245,46), (55,76), (15,136), (268,133), (200,103), (11,52), (125,146), (134,29), (204,49), (262,79), (338,97), (238,80), (190,18), (199,81), (303,14), (156,28), (122,52), (181,68), (229,43), (83,52), (271,24), (154,123), (302,89), (245,124), (125,53), (83,105), (38,71), (215,18), (22,122), (129,130), (80,135), (90,84), (350,80), (237,80), (314,37), (313,49), (170,67), (66,78), (204,108), (221,71), (259,101), (45,20), (77,48), (74,54), (180,80), (288,26), (221,53), (186,115), (33,68), (214,66), (221,140), (256,104), (279,85), (121,111), (160,58), (182,42), (168,44), (200,33), (150,29), (207,131), (56,131), (54,25), (169,116), (165,145), (309,114), (246,74), (195,143), (275,105), (199,106), (210,27), (220,35), (216,131), (25,13), (192,94), (142,46), (106,113), (269,33), (33,105), (289,13), (60,49), (13,92), (314,70), (304,105), (182,46), (301,52), (47,119), (190,19), (223,106), (247,33), (85,98), (45,76), (264,125), (232,92), (343,132), (282,126), (333,101), (184,86), (199,27), (338,99), (120,120)]

Potential hugging edge bug with 100 mines:
[(210,97), (213,122), (224,121), (348,113), (179,87), (272,112), (13,116), (298,71), (334,28), (336,32), (328,78), (307,18), (253,83), (236,72), (211,57), (246,38), (63,48), (13,79), (311,23), (111,31), (40,114), (250,113), (301,119), (197,25), (252,25), (147,25), (127,115), (283,49), (15,81), (98,90), (69,99), (288,122), (46,105), (124,115), (309,71), (195,142), (211,37), (112,101), (110,48), (289,127), (80,62), (107,99), (242,49), (337,143), (67,29), (350,41), (197,65), (151,108), (195,133), (230,41), (268,75), (206,56), (68,75), (232,36), (293,108), (148,118), (271,131), (272,51), (54,137), (79,36), (167,147), (140,141), (304,108), (203,133), (46,23), (32,120), (83,141), (182,18), (26,128), (155,67), (122,65), (303,115), (206,146), (268,70), (19,88), (195,145), (299,89), (18,137), (339,87), (54,48), (180,78), (116,110), (101,22), (295,95), (34,94), (132,76), (61,28), (251,37), (30,144), (321,116), (139,65), (174,68), (76,83), (67,76), (344,77), (150,85), (239,141), (332,57), (151,13), (209,52)]
start and endNodes are directly across from each other, start node is centered at the bottom of the field
The given path showcases a sharp turn backwards instead of running clockwise around a mine
to the next node.

Potential hugging edge bug with 120 mines:
[(150,32), (188,79), (78,44), (326,101), (164,106), (235,33), (263,126), (218,91), (75,115), (126,114), (341,120), (300,74), (106,126), (236,120), (16,140), (137,76), (96,112), (201,56), (19,49), (154,144), (312,104), (137,30), (62,89), (211,134), (182,136), (58,120), (255,42), (338,122), (118,35), (138,69), (275,90), (327,125), (65,48), (333,95), (46,95), (43,22), (177,106), (289,105), (61,132), (312,128), (191,69), (124,76), (14,36), (96,17), (171,27), (90,81), (18,82), (172,46), (330,128), (266,17), (185,84), (161,68), (107,147), (57,51), (322,15), (142,31), (182,70), (243,33), (349,142), (11,45), (274,30), (316,103), (155,36), (125,119), (277,27), (348,115), (107,79), (261,76), (84,76), (323,44), (174,114), (280,83), (185,32), (285,133), (169,88), (84,18), (198,132), (258,29), (60,145), (316,118), (212,78), (256,129), (256,118), (59,15), (153,135), (60,112), (125,103), (345,112), (329,79), (136,49), (305,85), (148,134), (271,80), (306,58), (178,60), (155,62), (23,46), (299,128), (190,31), (335,140), (35,141), (88,84), (42,95), (246,141), (266,28), (93,92), (278,76), (129,143), (209,110), (225,123), (317,61), (254,109), (118,138), (72,11), (249,131), (184,42), (103,13), (24,140), (222,74), (319,142)]
start and endNodes are directly across from each other, start node is centered at the bottom of the simulated field
The given path showaces a sharp turn backwards for some reason.
"""

if (len(recordedMineCoords) > 0):
    numMines = len(recordedMineCoords)

pathFindingType = "A*"  # dijkstra OR A* OR both OR none 

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

start = field.placeStartNode((simCorners[2][0]+simCorners[3][0])/2,((simCorners[2][1]-simCorners[3][1])/2)-5)
# Randomize endpoints, place endpoints along a horizontal line, or place endpoints manually
# endPoints = field.placeEndNodesLine(fieldSimCoords["yMax"],1) # A density of one defaults to the end node at (x range midpoint,yVal)
# endPoints = field.placeEndNodesPositions([(uniform(simCorners[0][0],simCorners[1][0]),simCorners[1][1]+5)])
endPoints = field.placeEndNodesPositions([(start.x,((simCorners[0][1]+simCorners[1][1])/2)+5)])

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
else:
    aStarPath = []
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