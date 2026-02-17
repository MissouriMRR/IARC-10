import numpy as np
import flight.pathfinding.utils.mask_gen as mask_gen
from PIL import Image, ImageDraw
import time as t
import random

class SightTracker:
    # Initializes the field as a greyscale image with pixel size equating to the field's size and the
    # stored int value equating to the "confidence"/"known" value
    def __init__(self, field_size:tuple[int, int]):
        self.fieldSize = field_size
        self.map = np.array(Image.new("L", field_size, 0))
    
    # Takes the corner coords from a taken picture to increment the "confidence value"
    def note_pic(self, corner_coords:tuple[int, int, int, int]):
        picShape = Image.new("L", self.fieldSize, 0)
        ImageDraw.Draw(picShape).polygon(corner_coords, outline=1, fill=1)
        self.map += np.array(picShape)

    # Checks for holes within the sightTracker map, within the predicted photo area, and within the given path clearence
    # a True means a there is a hole and a False means theres no hole
    def check_holes(self, pre_photo:mask_gen.PolygonMask, path_segment:mask_gen.PolygonMask): # Maybe include the previous and following path segments?
        result = True
        for i in range(path_segment.bottom_x, path_segment.top_x):
            for j in range(path_segment.bottom_y, path_segment.top_y):
                if (self.map[i][j] == 0 and pre_photo.body[i-pre_photo.bottom_x][j-pre_photo.bottom_y] > 0 and path_segment.body[i-path_segment.bottom_x][j-path_segment.bottom_y] > 0):
                    result = False
        return result