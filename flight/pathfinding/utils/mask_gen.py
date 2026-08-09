import flight.pathfinding.nodeField as nodeg
import numpy as np
from PIL import Image, ImageDraw
import math


class PolygonMask:
    # Python has no overloading -- the mask variants below used to all be named
    # __init__, so only the last definition survived and the others were
    # unreachable. They are now named constructors returning self, which is the
    # form seen_by_drone.py already calls them with.
    def __init__(self):
        self.top_x = 0
        self.bottom_x = 0
        self.top_y = 0
        self.bottom_y = 0
        self.body = None

    # Function for generating polygon masks based on node to node connections on differing mines
    # To be used for sight tracking and understanding where things need to be filled in on th ecurrent path
    # Array size is the dimensions of the sight array (which should be the same size as the minefield simulation array)
    def create_self_straight(self, node_1: nodeg.Node, node_2: nodeg.Node) -> "PolygonMask":
        # x1/y1 is the center the node sweeps around, so the far edge of the
        # swept band is the node mirrored across it. A floating node has no
        # parent mine to sweep around, so it is its own center and contributes
        # no width -- the old code reached for Mine.radius here, a class
        # attribute of the archived Mine that the live BlockMine has no
        # equivalent for.
        if node_1.getParentMine() is not None:
            x1 = node_1.parentMine.x
            y1 = node_1.parentMine.y
        else:
            x1 = node_1.x
            y1 = node_1.y

        if node_2.getParentMine() is not None:
            x2 = node_2.parentMine.x
            y2 = node_2.parentMine.y
        else:
            x2 = node_2.x
            y2 = node_2.y

        far_1_x = 2 * (node_1.x - x1) + x1
        far_1_y = 2 * (node_1.y - y1) + y1
        far_2_x = 2 * (node_2.x - x2) + x2
        far_2_y = 2 * (node_2.y - y2) + y2

        self.top_x = max([x1, far_1_x, x2, far_2_x])
        self.bottom_x = min([x1, far_1_x, x2, far_2_x])
        self.top_y = max([y1, far_1_y, y2, far_2_y])
        self.bottom_y = min([y1, far_1_y, y2, far_2_y])
        polygon = [
            (x1 - self.bottom_x, y1 - self.bottom_y),
            (far_1_x - self.bottom_x, far_1_y - self.bottom_y),
            (x2 - self.bottom_x, y2 - self.bottom_y),
            (far_2_x - self.bottom_x, far_2_y - self.bottom_y),
        ]

        img = Image.new("L", [self.top_x - self.bottom_x, self.top_y - self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(polygon, outline=1, fill=1)
        self.body = np.array(img)
        return self

    # Overload of the Polygon Mask function, this one is for specifically generating a predicted image area
    # should a picture be taken at a given path coord and orientation
    def create_self_rect(
        self, center: tuple[float, float], tan_angle: float, cam_size: tuple[float, float]
    ) -> "PolygonMask":
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
        corners = [tuple(c) for c in (corner_1, corner_2, corner_3, corner_4)]

        img = Image.new("L", [self.top_x - self.bottom_x, self.top_y - self.bottom_y], 0)
        ImageDraw.Draw(img).polygon(corners, outline=1, fill=1)
        self.body = np.array(img)
        return self

    # Here is where a future Arc/Pie slice shaped mask function will go
    def create_self_pie(self, node1: nodeg.Node, node2: nodeg.Node) -> "PolygonMask":
        if node1.parentMine != node2.parentMine:
            raise ValueError("must have same parrent")
        x1 = node1.x - node1.parentMine.x
        y1 = node1.y - node1.parentMine.y
        x2 = node2.x - node2.parentMine.x
        y2 = node2.y - node2.parentMine.y

        radius = int(math.hypot(x1, y1))

        angle1 = math.degrees(math.atan2(y1, x1))
        angle2 = math.degrees(math.atan2(y2, x2))
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
            self.top_y = node1.parentMine.y + (2 * radius)
        else:
            self.top_y = max(
                node1.parentMine.y,
                max(2 * radius * math.cos(angle1), 2 * radius * math.cos(angle2)),
            )

        if angle1 < 270 and angle2 > 270:
            self.bottom_y = node1.parentMine.y - (2 * radius)
        else:
            self.bottom_y = min(
                node1.parentMine.y,
                min(2 * radius * math.cos(angle1), 2 * radius * math.cos(angle2)),
            )

        if angle1 < 180 and angle2 > 180:
            self.bottom_x = node1.parentMine.x - (2 * radius)
        else:
            self.bottom_x = min(
                node1.parentMine.x,
                min(2 * radius * math.sin(angle1), 2 * radius * math.sin(angle2)),
            )

        if angle1 >= 270 and angle2 <= 90:
            self.top_x = node1.parentMine.x + (2 * radius)
        else:
            self.top_x = max(
                node1.parentMine.x,
                max(2 * radius * math.sin(angle1), 2 * radius * math.sin(angle2)),
            )

        img = Image.new("L", (2 * radius, 2 * radius), 0)
        draw = ImageDraw.Draw(img)
        draw.pieslice((0, 0), angle1, angle2, 1, 1)

        self.body = np.array(img)
        return self
