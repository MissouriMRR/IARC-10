import flight.pathfinding.node_generation as nodeg
import numpy as np
from PIL import Image, ImageDraw
import math


class PolygonMask:
    # Function for generating polygon masks based on node to node connections on differing mines
    # To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
    # Array size is the dimensions of the sight array (which should be the same size as the minefield simulation array)
    def __init__(self, node_1: nodeg.Node, node_2: nodeg.Node):
        if node_1.getParentMine() != None:
            x1 = node_1.parentMine.x
            y1 = node_1.parentMine.y
        else:
            x1 = node_1.x + nodeg.Mine.radius
            y1 = node_1.y

        if node_2.getParentMine() != None:
            x2 = node_2.parentMine.x
            y2 = node_2.parentMine.y
        else:
            x2 = node_2.x + nodeg.Mine.radius
            y2 = node_2.y

        self.top_x = max([x1, 2 * (node_1.x - x1) + x1, x2, 2 * (node_2.x - x2) + x2])
        self.bottom_x = min([x1, 2 * (node_1.x - x1) + x1, x2, 2 * (node_2.x - x2) + x2])
        self.top_y = max([y1, 2 * (node_1.y - y1) + y1, y2, 2 * (node_2.y - y2) + y2])
        self.bottom_y = min([y1, 2 * (node_1.y - y1) + y1, y2, 2 * (node_2.y - y2) + y2])
        polygon = [
            (x1 - self.bottom_x, y1 - self.bottom_y),
            (2 * (node_1.x - x1) + x1 - self.bottom_x, 2 * (node_1.y - y1) + y1 - self.bottom_y),
            (x2 - self.bottom_x, y2 - self.bottom_y),
            (2 * (node_2.x - x2) + x2 - self.bottom_x, 2 * (node_2.y - y2) + y2 - self.bottom_y),
        ]

        img = Image.new("L", [self.top_x - self.bottom_x, self.top_y - self.bottom_y], 0)
        img = Image.new("L", [self.top_x - self.bottom_x, self.top_y - self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
        self.body = np.array(img)

    # Overload of the Polygon Mask function, this one is for specifically generating a predicted image area
    # should a picture be taken at a given path coord and orientation
    def __init__(
        self, center: tuple[float, float], tan_angle: float, cam_size: tuple[float, float]
    ):
        corner_1 = (
            center[0]
            + (cam_size[0] / 2) * np.cos(tan_angle)
            - (cam_size[1] / 2) * np.sin(tan_angle),
            center[1]
            + (cam_size[0] / 2) * np.sin(tan_angle)
            + (cam_size[1] / 2) * np.cos(tan_angle),
        )
        corner_2 = (
            center[0]
            - (cam_size[0] / 2) * np.cos(tan_angle)
            - (cam_size[1] / 2) * np.sin(tan_angle),
            center[1]
            - (cam_size[0] / 2) * np.sin(tan_angle)
            + (cam_size[1] / 2) * np.cos(tan_angle),
        )
        corner_3 = (
            center[0]
            - (cam_size[0] / 2) * np.cos(tan_angle)
            + (cam_size[1] / 2) * np.sin(tan_angle),
            center[1]
            - (cam_size[0] / 2) * np.sin(tan_angle)
            - (cam_size[1] / 2) * np.cos(tan_angle),
        )
        corner_4 = (
            center[0]
            + (cam_size[0] / 2) * np.cos(tan_angle)
            + (cam_size[1] / 2) * np.sin(tan_angle),
            center[1]
            + (cam_size[0] / 2) * np.sin(tan_angle)
            - (cam_size[1] / 2) * np.cos(tan_angle),
        )
        self.top_x = max(corner_1[0], corner_2[0], corner_3[0], corner_4[0])
        self.bottom_x = min(corner_1[0], corner_2[0], corner_3[0], corner_4[0])
        self.top_y = max(corner_1[1], corner_2[1], corner_3[1], corner_4[1])
        self.bottom_y = min(corner_1[1], corner_2[1], corner_3[1], corner_4[1])
        corner_1 = np.subtract(corner_1, [self.bottom_x, self.bottom_y])
        corner_2 = np.subtract(corner_2, [self.bottom_x, self.bottom_y])
        corner_3 = np.subtract(corner_3, [self.bottom_x, self.bottom_y])
        corner_4 = np.subtract(corner_4, [self.bottom_x, self.bottom_y])
        corners = [corner_1, corner_2, corner_3, corner_4]

        img = Image.new("L", [self.top_x - self.bottom_x, self.top_y - self.bottom_y], 0)
        img = Image.new("L", [self.top_x - self.bottom_x, self.top_y - self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(corners, outline=1, fill=1)
        self.body = np.array(img)

    # Here is where a future Arc/Pie slice shaped mask function will go
    def __init__(self, node1: nodeg.Node, node2: nodeg.Node, pie: bool):
        if node1.parentMine != node2.parentMine:
            raise ValueError("must have same parrent")
        x1 = node1.x - node1.parentMine.x
        y1 = node1.y - node1.parentMine.y
        x2 = node2.x - node2.parentMine.x
        y2 = node2.y - node2.parentMine.y

        radius = int(math.hypot(x1, y1))

        angle1 = math.degrees(math.atan2 * (y1, x1))
        angle2 = math.degrees(math.atan2 * (y2, x2))
        if angle1 < 0:
            angle1 += 360
        if angle2 < 0:
            angle2 += 360

        if abs(angle1 - angle2) > 180:
            angle3 = angle1
            angle1 = angle2
            angle2 = angle3

        angle1 *= -1
        angle2 *= -1

        if angle1 > angle2:
            angle3 = angle1
            angle1 = angle2
            angle2 = angle3

        if angle1 < -270 and angle2 > -90:
            angle3 = angle1
            angle1 = angle2
            angle2 = angle3

        if angle1 < 90 and angle2 > 90:
            self.top_y = node1.ParentMine.y + (2 * radius)
        else:
            self.top_y = max(
                node1.ParentMine.y,
                max(2 * radius * math.cos(angle1), 2 * radius * math.cos(angle2)),
            )

        if angle1 < 270 and angle2 > 270:
            self.bottom_y.ParentMine.y - (2 * radius)
        else:
            self.bottom_y = min(
                node1.ParentMine.y,
                min(2 * radius * math.cos(angle1), 2 * radius * math.cos(angle2)),
            )

        if angle1 < 180 and angle2 > 180:
            self.bottom_x = node1.ParentMine.y - (2 * radius)
        else:
            self.bottom_x = min(
                node1.ParentMine.x,
                min(2 * radius * math.sin(angle1), 2 * radius * math.sin(angle2)),
            )

        if angle1 >= 270 and angle2 <= 90:
            self.top_x = node1.ParentMine.x + (2 * radius)
        else:
            self.top_x = min(
                node1.ParentMine.x,
                min(2 * radius * math.sin(angle1), 2 * radius * math.sin(angle2)),
            )

        img = Image.new("RGBA", (2 * radius, 2 * radius), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.pieslice((0, 0, 2 * radius, 2 * radius), angle1, angle2, (255, 255, 255, 255))

        self.body = np.array(img)
