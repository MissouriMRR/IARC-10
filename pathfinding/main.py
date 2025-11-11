import rexAlg.nodeGen as ng
import sightWeights.seenByDrone as sbd
import dijkstrasPathfindingAlg.basicDijkstras as dijk
import numpy as np
import time as t
from PIL import Image, ImageDraw

# Function for generating polygon masks based on node to node connections
# To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
def polygonMask(node1:ng.Node, node2:ng.Node, array_size:tuple[int, int]):
    x1 = node1.parentMine.x
    y1 = node1.parentMine.y
    x2 = node2.parentMine.x
    y2 = node2.parentMine.y
    polygon = [(x1,y1),(2(node1.x-x1)+x1,2(node1.y-y1)+y1),(x2,y2),(2(node2.x-x2)+x2,2(node2.y-y2)+y2)]

    img = Image.new('L', array_size, 0)
    ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
    return np.array(img)

