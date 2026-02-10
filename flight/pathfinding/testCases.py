from flight.pathfinding.genNodesFromMines import Field, Mine, Node
from flight.pathfinding.genPathFromNodes import Graph
from random import randint, seed
"""
Use this file for getting the node graph.
This will generate 10 (or however many you want) mines that are placed. 
The output will be at the bottom.
"""
# seed(10) make random or not
numMines = 20
radius = 16
xMin = -numMines*radius
xMax = numMines*radius
yMin = -numMines*radius
yMax = numMines*radius
field = Field(xMin,xMax,yMin,yMax)
genXMin = -radius*(numMines//2)
genXMax = radius*(numMines//2)
genYMin =-radius*(numMines//2)
genYMax = radius*(numMines//2)
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

for mine in field.mines:
    mine.connectMineNodes()

start = field.placeStartNode(0,yMin + (radius*1.5))
endPoints = field.placeEndNodes(yMax - (radius*1.5),10)
pathSolve = Graph(field.nodeGraph)
path = pathSolve.shortest_path(start,endPoints)
print("optimal path:",path,"\n")

field.plotField()