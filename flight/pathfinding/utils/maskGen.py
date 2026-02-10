import flight.pathfinding.nodeGeneration as nodeg
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
        self.topX = max([x1, 2(node1.x-x1)+x1, x2, 2(node2.x-x2)+x2])
        self.bottomX = min([x1, 2(node1.x-x1)+x1, x2, 2(node2.x-x2)+x2])
        self.topY = max([y1, 2(node1.y-y1)+y1, y2, 2(node2.y-y2)+y2])
        self.bottomY = min([y1, 2(node1.y-y1)+y1, y2, 2(node2.y-y2)+y2])
        polygon = [(x1-self.bottomX, y1-self.bottomY),(2(node1.x-x1)+x1-self.bottomX, 2(node1.y-y1)+y1-self.bottomY),(x2-self.bottomX, y2-self.bottomY),(2(node2.x-x2)+x2-self.bottomX, 2(node2.y-y2)+y2-self.bottomY)]

        
        img = Image.new('L', [self.topX-self.bottomX, self.topY-self.bottomY], 0)
        ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
        self.body = np.array(img)

    # Overload of the Polygon Mask function, this one is for specifically generating a predicted image area
    # should a picture be taken at a given path coord and orientation
    def __init__(self, centerCoord:tuple[float, float], tanAngle:int, camSize:tuple[float, float], array_size:tuple[int, int]):
        pass

    # Here is where a future Arc/Pie slice shaped mask function will go
    def __init__(self, node1:nodeg.Node, node2:nodeg.Node, pie:bool):
        pass