import numpy as np
import flight.pathfinding.utils.maskGen as maskGen
from PIL import Image, ImageDraw
import time as t
import random

# Begining of the sight tracking class, the goal of this is to track what all the drone has seen so far
class sightTracker:
    # Initializes the field as a greyscale image with pixel size equating to the field's size and the
    # stored int value equating to the "confidence"/"known" value
    def __init__(self, fieldSize:tuple[int, int]):
        self.fieldSize = fieldSize
        self.map = np.array(Image.new("L", fieldSize, 0))
    
    # Takes the corner coords from a taken picture to increment the "confidence value"
    def notePic(self, cornerCoords:tuple[int, int, int, int]):
        picShape = Image.new("L", self.fieldSize, 0)
        ImageDraw.Draw(picShape).polygon(cornerCoords, outline=1, fill=1)
        self.map += np.array(picShape)

    # Checks for holes within the sightTracker map, within the predicted photo area, and within the given path clearence
    # a True means a there is a hole and a False means theres no hole
    def checkHoles(self, prePhoto:maskGen.PolygonMask, pathSegment:maskGen.PolygonMask): # Maybe include the previous and following path segments?
        result = True
        for i in range(pathSegment.bottomX, pathSegment.topX):
            for j in range(pathSegment.bottomY, pathSegment.topY):
                if (self.map[i][j] == 0 and prePhoto.body[i-prePhoto.bottomX][j-prePhoto.bottomY] > 0 and pathSegment.body[i-pathSegment.bottomX][j-pathSegment.bottomY] > 0):
                    result = False
        return result


# Old after this point, unlikely to be used but I will keep this around till 
# I finish with new usage


timeLimit = 120 # Time limit in seconds
fieldSizeX = 3600 # The max size of the field in inches
fieldSizeY = 960 # The max size of the field in inches

# The circle mask for messing with numpy arrays
def circle_mask(arraySize:tuple[int, int], circleCenter:tuple[int, int], circleRad:int):
    hight, width = arraySize
    y, x = np.ogrid[:hight, :width]
    mask = np.sqrt((x - circleCenter[0])**2 + (y - circleCenter[1])**2) <= circleRad
    return mask

# Randomly moves a given simulated drone in a direction, used for testing
def random_move(droneNumber, droneLocation):
    newX = droneLocation[droneNumber][0] + random.randint(-1, 1)
    newY = droneLocation[droneNumber][1] + random.randint(-1, 1)
    if 0 <= newX < fieldSizeX:
        newX = droneLocation[droneNumber][0]
    if 0 <= newY < fieldSizeY:
        newY = droneLocation[droneNumber][1]
    return [newX, newY]

radiusOfVision = 120 # Set to 120 for a radius of 10ft
droneCoords = [[450,960], [1350,960], [2250,960], [3150,960]] # Starting locations for all the drones, edit as you please
droneSightArr = np.empty((4, fieldSizeX, fieldSizeY)) # Indevidual sight data for each drone
mergedSeen = np.empty((fieldSizeX, fieldSizeY)) # Combines all the seen data so decisions can be made off of it

# Increments the values in an area around the drones representative of what the drones are seeing
# This repeats until a time value is reached
startTime = t.time()
while True:
    if t.time() - startTime <= timeLimit:
        for i in range(len(droneCoords)):
            droneSightArr[i][circle_mask([fieldSizeX,fieldSizeY], droneCoords[i], radiusOfVision)] += 1
            droneCoords[i] = random_move(i, droneCoords)
        mergedSeen = droneSightArr[0] + droneSightArr[1] + droneSightArr[2] + droneSightArr[3]
    else:
        print("Done")
        print(np.max(mergedSeen))
        break