import numpy as np
import flight.pathfinding.utils.maskGen as maskGen
from PIL import Image, ImageDraw
import time as t
import random

class SightTracker:
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