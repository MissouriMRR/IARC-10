from flight.pathfinding.genNodesFromMines import Field, Mine, Node
from flight.pathfinding.genPathFromNodes import Graph
from random import randint, seed
"""
Use this file for getting the node graph.
This will generate 50 mines that are manually placed. 
The output will be at the bottom.
"""
field = Field(-500,500,-500,500)
radius = 50
position = [0,0]


numMines = 50 # Only adjust this for now.
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

for mine in field.mines:
    mine.connectMineNodes()

start = field.placeStartNode(0,-460)
endPoints = field.placeEndNodes(460,5)
field.cleanNodeGraph()
pathSolve = Graph(field.nodeGraph)
path = pathSolve.shortest_path(start,endPoints)
print("optimal path:",path)
field.plotField()