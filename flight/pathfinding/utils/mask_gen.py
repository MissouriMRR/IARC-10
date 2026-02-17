import flight.pathfinding.node_generation as nodeg
import numpy as np
from PIL import Image, ImageDraw

class PolygonMask:
    # Function for generating polygon masks based on node to node connections on differing mines
    # To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
    # Array size is the dimensions of the sight array (which should be the same size as the minefield simulation array)
    def __init__(self, node1:nodeg.Node, node2:nodeg.Node):
        x1 = node1.parentMine.x
        y1 = node1.parentMine.y
        x2 = node2.parentMine.x
        y2 = node2.parentMine.y
        self.top_x = max([x1, 2(node1.x-x1)+x1, x2, 2(node2.x-x2)+x2])
        self.bottom_x = min([x1, 2(node1.x-x1)+x1, x2, 2(node2.x-x2)+x2])
        self.top_y = max([y1, 2(node1.y-y1)+y1, y2, 2(node2.y-y2)+y2])
        self.bottom_y = min([y1, 2(node1.y-y1)+y1, y2, 2(node2.y-y2)+y2])
        polygon = [(x1-self.bottom_x, y1-self.bottom_y),(2(node1.x-x1)+x1-self.bottom_x, 2(node1.y-y1)+y1-self.bottom_y),(x2-self.bottom_x, y2-self.bottom_y),(2(node2.x-x2)+x2-self.bottom_x, 2(node2.y-y2)+y2-self.bottom_y)]

        
        img = Image.new('L', [self.top_x-self.bottom_x, self.top_y-self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
        self.body = np.array(img)

    # Overload of the Polygon Mask function, this one is for specifically generating a predicted image area
    # should a picture be taken at a given path coord and orientation
    def __init__(self, center_coord:tuple[float, float], tan_angle:int, cam_size:tuple[float, float], array_size:tuple[int, int]):
        pass

    # Here is where a future Arc/Pie slice shaped mask function will go
    def __init__(self, node1:nodeg.Node, node2:nodeg.Node, pie:bool):
        pass