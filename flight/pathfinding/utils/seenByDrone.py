import numpy as np
import flight.pathfinding.utils.mask_gen as maskGen
import flight.pathfinding.path_subdivision as pathSub
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
    def note_pic(self, cornerCoords:tuple[int, int, int, int]):
        picShape = Image.new("L", self.fieldSize, 0)
        ImageDraw.Draw(picShape).polygon(cornerCoords, outline=1, fill=1)
        self.map += np.array(picShape)

    # Checks for holes within the sightTracker map, within the predicted photo area, and within the given path clearence
    # a True means a there is a hole and a False means theres no hole
    def check_holes(self, prePhoto:maskGen.PolygonMask, pathSegment:maskGen.PolygonMask): # Maybe include the previous and following path segments?
        result = True
        for i in range(pathSegment.bottom_x, pathSegment.top_x):
            for j in range(pathSegment.bottom_y, pathSegment.top_y):
                if (self.map[i][j] == 0 and prePhoto.body[i-prePhoto.bottom_x][j-prePhoto.bottom_y] > 0 and pathSegment.body[i-pathSegment.bottom_x][j-pathSegment.bottom_y] > 0):
                    result = False
        return result

# Removes coords that are redundant to check
def remove_extra_coords(seen:SightTracker, goto_points:tuple[tuple[float,float]], data_sup:tuple[tuple[bool,pathSub.Node,pathSub.Node]], cam_size:tuple[float,float]):
    updated_goto_points:tuple[tuple[float,float]]
    for i in range(len(goto_points)):
        if (data_sup[i][0] == True):
            seg_mask = maskGen.PolygonMask(data_sup[i][1],data_sup[i][2],True)
            tan_angle = np.arctan((goto_points[i][1] - data_sup[i][1].parentMine.y) / (goto_points[i][0] - data_sup[i][1].parentMine.x)) + (np.pi/2)
            proto_photo = maskGen.PolygonMask(goto_points[i], tan_angle, cam_size)
        else:
            seg_mask = maskGen.PolygonMask(data_sup[i][1],data_sup[i][2])
            tan_angle = np.arctan((data_sup[i][1].y - data_sup[i][2].y) / (data_sup[i][1].x - data_sup[i][2].x))
            proto_photo = maskGen.PolygonMask(goto_points[i], tan_angle, cam_size)
        
        if (seen.check_holes(proto_photo, seg_mask)):
            updated_goto_points.append(goto_points[i])
    
    return updated_goto_points
