import numpy as np
import time as t
import random

timeLimit = 120 # Time limit in seconds
fieldSizeX = 3600 # The max size of the field in inches
fieldSizeY = 960 # The max size of the field in inches

# The circle mask for messing with numpy arrays
def circle_mask(arraySize, circleCenter, circleRad):
    hight, width = arraySize
    y, x = np.ogrid[:hight, :width]
    mask = np.sqrt((x - circleCenter[0])**2 + (y - circleCenter[1])**2) <= circleRad
    return mask

# Randomly moves a given drone in a direction
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