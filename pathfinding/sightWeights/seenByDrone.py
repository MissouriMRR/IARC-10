import numpy as np
import time as t
import random
from PIL import Image, ImageDraw

timeLimit = 120 # Time limit in seconds
fieldSizeX = 3600 # The max size of the field in inches
fieldSizeY = 960 # The max size of the field in inches

# The circle mask for messing with numpy arrays
def circle_mask(arraySize, circleCenter, circleRad):
    hight, width = arraySize
    y, x = np.ogrid[:hight, :width]
    mask = np.sqrt((x - circleCenter[0])**2 + (y - circleCenter[1])**2) <= circleRad
    return mask

# Function for generating polygon masks based on node to node connections
# To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
def polygonMask(node1, node2, array_size):
    x1 = node1.mine.x
    y1 = node1.mine.y
    x2 = node2.mine.x
    y2 = node2.mine.y
    polygon = [(x1,y1),(2(node1.x-x1)+x1,2(node1.y-y1)+y1),(x2,y2),(2(node2.x-x2)+x2,2(node2.y-y2)+y2)]

    img = Image.new('L', array_size, 0)
    ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
    return np.array(img)

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