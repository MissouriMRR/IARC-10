import flight.pathfinding.node_generation as nodeg
import numpy as np
from PIL import Image, ImageDraw
import math

class PolygonMask:
    def __init__(self):
        x1 = 1
        x2 = 2
        y1 = 1
        y2 = 2
        self.top_x = max([x1, x1, x2, x2])
        self.bottom_x = min([x1, x1, x2, x2])
        self.top_y = max([y1, y1, y2, y2])
        self.bottom_y = min([y1, y1, y2, y2])
        polygon = [(x1-self.bottom_x, y1-self.bottom_y),(x1-self.bottom_x, y1-self.bottom_y),(x2-self.bottom_x, y2-self.bottom_y),(x2-self.bottom_x, y2-self.bottom_y)]


    # Function for generating polygon masks based on node to node connections on differing mines
    # To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
    # Array size is the dimensions of the sight array (which should be the same size as the minefield simulation array)
    def create_self_straight(self, node_1:nodeg.Node, node_2:nodeg.Node):
        if (node_1.getParentMine() != None):
            x1 = node_1.parentMine.x
            y1 = node_1.parentMine.y
        else:
            x1 = node_1.x + nodeg.Mine.radius
            y1 = node_1.y

        if (node_2.getParentMine() != None):
            x2 = node_2.parentMine.x
            y2 = node_2.parentMine.y
        else:
            x2 = node_2.x + nodeg.Mine.radius 
            y2 = node_2.y
            
        self.top_x = int(np.ceil(max([x1, 2*(node_1.x-x1)+x1, x2, 2*(node_2.x-x2)+x2])))
        self.bottom_x = int(np.floor(min([x1, 2*(node_1.x-x1)+x1, x2, 2*(node_2.x-x2)+x2])))
        self.top_y = int(np.ceil(max([y1, 2*(node_1.y-y1)+y1, y2, 2*(node_2.y-y2)+y2])))
        self.bottom_y = int(np.floor(min([y1, 2*(node_1.y-y1)+y1, y2, 2*(node_2.y-y2)+y2])))
        polygon = [(x1-self.bottom_x, y1-self.bottom_y),(2*(node_1.x-x1)+x1-self.bottom_x, 2*(node_1.y-y1)+y1-self.bottom_y),(x2-self.bottom_x, y2-self.bottom_y),(2*(node_2.x-x2)+x2-self.bottom_x, 2*(node_2.y-y2)+y2-self.bottom_y)]
        
        img = Image.new('L', [self.top_x-self.bottom_x, self.top_y-self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1, width=1)
        img.save("last_straight.jpeg")
        self.body = np.array(img)
        return self

    # Overload of the Polygon Mask function, this one is for specifically generating a predicted image area
    # should a picture be taken at a given path coord and orientation
    def create_self_rect(self, center:tuple[float, float], tan_angle:float, cam_size:tuple[float, float]):
        corner_1 = (center[0]+(cam_size[0]/2)*np.cos(tan_angle)-(cam_size[1]/2)*np.sin(tan_angle), center[1]+(cam_size[0]/2)*np.sin(tan_angle)+(cam_size[1]/2)*np.cos(tan_angle))
        corner_2 = (center[0]-(cam_size[0]/2)*np.cos(tan_angle)-(cam_size[1]/2)*np.sin(tan_angle), center[1]-(cam_size[0]/2)*np.sin(tan_angle)+(cam_size[1]/2)*np.cos(tan_angle))
        corner_3 = (center[0]-(cam_size[0]/2)*np.cos(tan_angle)+(cam_size[1]/2)*np.sin(tan_angle), center[1]-(cam_size[0]/2)*np.sin(tan_angle)-(cam_size[1]/2)*np.cos(tan_angle))
        corner_4 = (center[0]+(cam_size[0]/2)*np.cos(tan_angle)+(cam_size[1]/2)*np.sin(tan_angle), center[1]+(cam_size[0]/2)*np.sin(tan_angle)-(cam_size[1]/2)*np.cos(tan_angle))
        self.top_x = int(np.ceil(max(corner_1[0],corner_2[0],corner_3[0],corner_4[0])))
        self.bottom_x = int(np.floor(min(corner_1[0],corner_2[0],corner_3[0],corner_4[0])))
        self.top_y = int(np.ceil(max(corner_1[1],corner_2[1],corner_3[1],corner_4[1])))
        self.bottom_y = int(np.floor(min(corner_1[1],corner_2[1],corner_3[1],corner_4[1])))
        corner_1 = (corner_1[0] - self.bottom_x, corner_1[1] - self.bottom_y)
        corner_2 = (corner_2[0] - self.bottom_x, corner_2[1] - self.bottom_y)
        corner_3 = (corner_3[0] - self.bottom_x, corner_3[1] - self.bottom_y)
        corner_4 = (corner_4[0] - self.bottom_x, corner_4[1] - self.bottom_y)
        corners = [corner_1, corner_2, corner_3, corner_4]

        img = Image.new('L', [self.top_x-self.bottom_x, self.top_y-self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(corners, outline=1, fill=1, width=1)
        img.save("last_rect.jpeg")
        self.body = np.array(img)
        return self

    # Here is where a future Arc/Pie slice shaped mask function will go
    def create_self_pie(self, node1:nodeg.Node, node2:nodeg.Node):
        if(node1.parentMine!=node2.parentMine):
            raise ValueError("must have same parrent")
        x1 = node1.x-node1.parentMine.x
        y1 = node1.y-node1.parentMine.y
        x2 = node2.x-node2.parentMine.x
        y2 = node2.y-node2.parentMine.y
        
        radius=int(np.sqrt(x1**2 + y1**2))
       
       
        angle1 = math.degrees(math.atan2(y1,x1))
        angle2 = math.degrees(math.atan2(y2,x2))
        print([angle1, angle2])

        self.top_y=int(np.ceil(node1.parentMine.y+(radius)))

        self.bottom_y=int(np.floor(node1.parentMine.y-(radius)))

        self.bottom_x=int(np.floor(node1.parentMine.x-(radius)))

        self.top_x=int(np.ceil(node1.parentMine.x+(radius)))

        img =  Image.new("L", (2*radius, 2*radius), 0)
        ImageDraw.Draw(img).pieslice([(0, 0), (2*radius, 2*radius)], angle1, angle2, 1, 1, 1)
        img.save("last_pie.jpeg")
        self.body = np.array(img)
        return self