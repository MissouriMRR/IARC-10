import flight.pathfinding.rexAlg.nodeGen as nodeg
import flight.pathfinding.sightWeights.seenByDrone as seebd
import flight.pathfinding.dijkstrasPathfindingAlg.basicDijkstras as dijk
import flight.pathfinding.safetyNodeGen.intermediateNodeGen as gotoDiv
import numpy as np
import time as t
from PIL import Image, ImageDraw

# Function for generating polygon masks based on node to node connections
# To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
def polygonMask(node1:nodeg.Node, node2:nodeg.Node, array_size:tuple[int, int]):
    x1 = node1.parentMine.x
    y1 = node1.parentMine.y
    x2 = node2.parentMine.x
    y2 = node2.parentMine.y
    polygon = [(x1,y1),(2(node1.x-x1)+x1,2(node1.y-y1)+y1),(x2,y2),(2(node2.x-x2)+x2,2(node2.y-y2)+y2)]

    img = Image.new('L', array_size, 0)
    ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
    return np.array(img)

# This class represents the actual drones
class Drone:
    def __init__(self, coords:tuple[int,int] = (1800, 960), state:str = "Awaiting Task", mineRadius:int = 36, id:int = 0):
        self.x = coords[0]
        self.y = coords[1]
        self.state = state
        self.visionRange = ((1,1), (1,1), (1,1), (1,1))
        self.mineRadius = mineRadius
        self.tasks = []
        self.id = id

    # Updates the corners tracking what the drone is seeing
    def updateVision(self, corners:tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]] = ((1,1), (1,1), (1,1), (1,1))):
        self.visionRange = corners

    def updateTasks(self, gotoCoords:tuple[tuple[int, int]]):
        self.tasks = []
        for i in range(len(gotoCoords)):
            self.tasks.append(gotoCoords[i])
    
    # Checks to see if the area needs to be checked
    def checkSeen(self, centerCoords:tuple[int, int]):
        self.x = centerCoords[0]

    # This will be the function that sends a drone to a given location
    # Right now its built a simple placeholder
    def goto(self, coords:tuple[int, int]):
        self.x = coords[0]
        self.y = coords[1]

    def completeTasks(self):
        for i in range(len(self.tasks)):
            self.goto(self.tasks[i])
            photoStorage = self.takePhoto(cameraLocal) # Small Placeholder should be self explainitory
            self.addMines(self.processPhoto(photoStorage)) # Big Placeholder (Will need to be in consideration with the current path and mine list)

            '''
            FUTURE IMPLEMENTATIONS
            # Meet with Jack to figure out how his code is designed to work (this is were the path is recalculated also current path should be updated)
            # Recalculation is contingent on found mine being exigent
            # Call for a remeet if path is changed, clears current task cache is that happens and breaks
            '''
        # Clears task cache
    
    #Smart landing sequence, Should be usable in final product!!
    def recall(self):
        if (fieldSizeX - self.x < fieldSizeY - self.y):
            landAt(fieldSizeX*round(self.x / fieldSizeX), self.y)
        else:
            landAt(self.x, fieldSizeY*round(self.y / fieldSizeY))
    
    # This is a hard coded mess and will need to be done
    def selfLoop(self, path:Path):
        for i in range(10):
            gotoCoords = gotoDiv.gotoPath(currentPath)
            diviedGoto = []
            for y in range(len(gotoCoords)/len(drones)):
                diviedGoto.append(gotoCoords[(id-0)*6+y])
            self.updateTasks(diviedGoto)
            self.completeTasks()
            for i in range(4):
                if (id-1 == i):
                    # Broadcast new minedata and new sight data
                    broadcast()
                else:
                    # Recieve new minedata and new sight data
                    recieve()
        self.recall()    


        # Some kind of logic to calculate the best path the drone can take to leave the confines of the mine field
        # This happens as fast as possible as this could disqualify us
        # Simple idea is drawing a line from the point center and finding that intersect with the bounds and sending the dorne there to land
        self.x
        self.y
    
    # Recieve mine coordinates and adds it to an existing field
    def addMine(self,field:gotoDiv.Field,x:int,y:int):
        field.addMine(x,y,self.mineRadius)

# This is a place holder for the output from generating the fastest path.
class Path:
    def __init__(self, chain:tuple[nodeg.Node], length:float = 9999999, width:float = 1):
        self.length = length
        self.chain = chain
        self.width = width
        self.weight = length/width


timeLimit = 120 # Time limit in seconds
fieldSizeX = 3600 # The max size of the field in inches
fieldSizeY = 960 # The max size of the field in inches
startTime = t.time() # Starting time (Based on global clock)
previousPath = Path() # Previous Path DOES NOT WORK AS LAYED OUT NEEDS UPDATE TO PATH CLASS AND THE STRUCTURE OF JACK'S NODE GENERATION
currentPath = Path() # Current Working Path SEE ABOVE
drones = [Drone(id=1), Drone(id=2), Drone(id=3), Drone(id=4)] # Note that the mine radii is in inches and defaults to 
stopCondition = "Timed out"

drones[0].selfLoop(currentPath)
drones[1].selfLoop(currentPath)
drones[2].selfLoop(currentPath)
drones[3].selfLoop(currentPath)

'''
# Vestigial at the moment, but will be the "lead drone's" commanding thread
def mainLoop():
    while (True):
        if (t.time() - startTime > timeLimit):

            # Dumps the current path to app

            # Rudimentary implementation
            for i in range(len(drones)):
                        drones[i].recall()

            stopCondition = "Timed out"
            break
        if (previousPath.weight < currentPath.weight):
            stopCondition = "Optimal Path Found"

            # Dumps the most optimal path to the app (This would be a function)

            # Rudimentary implementation
            for i in range(len(drones)):
                drones[i].recall()

            break
        else:
            # Recursion needed here
            gotoCoords = gotoDiv.gotoPath(currentPath)
            for i in range(len(drones)):
                diviedGoto = []
                for y in range(len(gotoCoords)/len(drones)):
                    diviedGoto.append(gotoCoords[i*6+y])
                drones[i].updateTasks(diviedGoto)
                drones[i].completeTasks() # This will likely need to be changed to allow for the code to continue while this runs
            
            # Something to wait for all drones to finish their tasks

            # Drones reconvene after finishing performing their respective tasks confirming the found path and saving it as the previous path before incrementing
            for i in range(len(drones)):
                drones[i].goto(gotoCoords[len(gotoCoords)/2]) # DO NOT LEAVE THIS IN AND THEN FLY THE DRONES, THIS NEEDS TO BE REPLACED WITH SOMETHING SMARTER

                # Confirm Current Path
                # Save Current Path as Previous
                drones[i].mineRadius += 1
                # Recalculate Path
                # Loop (⌐▨_▨)
    print(stopCondition)

'''
