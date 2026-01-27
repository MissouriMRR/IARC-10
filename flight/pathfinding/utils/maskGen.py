import flight.pathfinding.genNodesFromMines as nodeg
import numpy as np
from PIL import Image, ImageDraw

# Function for generating polygon masks based on node to node connections on differing mines
# To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
# Array size is the dimensions of the sight array (which should be the same size as the minefield simulation array)
def polygonMask(node1:nodeg.Node, node2:nodeg.Node, array_size:tuple[int, int]):
    x1 = node1.parentMine.x
    y1 = node1.parentMine.y
    x2 = node2.parentMine.x
    y2 = node2.parentMine.y
    polygon = [(x1,y1),(2(node1.x-x1)+x1,2(node1.y-y1)+y1),(x2,y2),(2(node2.x-x2)+x2,2(node2.y-y2)+y2)]

    img = Image.new('L', array_size, 0)
    ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
    return np.array(img)

# Here is where a future Arc/Pie slice shaped mask function will go