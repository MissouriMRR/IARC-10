from . import mine as m
from . import node_connection as nc

from . import field as f
import numpy as np


# Node class keeps track of node positions
class Node:
    nodeNum = 0
    connectionList = []

    def __init__(
        self,
        xPosition: float,
        yPosition: float,
        floating: bool,
        angle: float = 0,
        name: str = "",
        labeled: bool = False,
        nType: str = "default",
    ):
        """
        Create node based off of (x,y) coordinate, whether or not it is floating,
        its angle, optional name, whether or not it is named, labeled,
        and the kind of Node it is(for selective elimination purposes)
        \nTypes(case sensitive):\n "default","start","end"
        """
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "FNID: " + str(Node.nodeNum)
        else:
            self.name = name
        self.type = type
        self.labeled = labeled  # Purely for debugging and visually isolating nodes
        self.x = xPosition
        self.y = yPosition
        self.plotted = False  # To prevent hopefully duplicate plotting
        # self.nodeGraph.update({self:None})
        self.parentMine = None
        self.floating = floating
        self.angle = angle  # will stay 0 if node doesnt have an angle, AKA it is floating

        # For selective elimination(dont want to delete end or start nodes)
        # Types:
        # - "default"
        # - "start"
        # - "end"
        self.nType = nType

    # Establishes a connection between nodes
    # Does not add it to the nodegraph yet however
    def connectNode(self, node: "Node") -> Connection:
        if self == node:
            raise TypeError("Same nodes")
        nodeConnection = Connection(self, node)  # connection initialization
        if nodeConnection.validPath():
            nodeConnection.addGraph()
        else:
            nodeConnection.deleteConnection()
        # self.connected = True
        # node.connected = True
        """
        if node.parentMine not in self.parentMine.connectedMines and self.parentMine not in node.parentMine.connectedMines:
            self.parentMine.connectedMines.append(node.parentMine)
            node.parentMine.connectedMines.append(self.parentMine)

        """
        return nodeConnection

    def deleteNode(self):
        if self.parentMine != None:
            self.parentMine.removeNode(self)
        if self in Connection.field.nodeGraph:
            del Connection.field.nodeGraph[self]

    def getPos(self) -> float:
        return (round(float(self.x), 3), round(float(self.y), 3))

    def getTargetMine(self) -> Mine:
        return self.__targetMine

    def getParentMine(self) -> Mine:
        return self.parentMine

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.__str__()

    def __gt__(self, node1: "Node"):
        return self.y > node1.y


class MineNode(Node):

    def __init__(
        self,
        parentMine: "Mine" = None,
        targetMine: "Mine" = None,
        internal: bool = True,
        primary: bool = True,
        connectedToFloating: bool = False,
        floatingNode: "Node" = None,
        name: str = "",
    ):
        Node.nodeNum += 1
        self.mineOrder = -1  # Used to determine order on mine, Set at connectMineNodes functions.
        self.parentMine = parentMine
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.connected = False
        self.plotted = False  # To prevent hopefully duplicate plotting
        self.targetMine = targetMine
        self.terminate = (
            False  # If node would be generated illegally, mark for termination/ignoring
        )
        self.internal = internal
        self.primary = primary
        # These should not be used if connectedToFloating is False
        self.connectedToFloating = connectedToFloating
        self.floatingNode = floatingNode

        self.calculateAndAssignPosition()
        if len(name) < 1:
            self.name = "CID:" + str(parentMine.number) + "." + str(Node.nodeNum)
        else:
            self.name = name

        super().__init__(self.x, self.y, False, angle=self.angle, name=self.name)
        self.parentMine = parentMine  # VERY NECESSARY DO NOT REMOVE

    def calculateAndAssignPosition(self):

        if (
            not self.connectedToFloating
        ):  # MineNode will have slightly different variables depending on this

            # categorize nodes
            if self.internal and self.primary:
                self.type = "internal primary"
            elif self.internal and not self.primary:
                self.type = "internal secondary"
            elif not self.internal and self.primary:
                self.type = "external primary"
            elif not self.internal and not self.primary:
                self.type = "external secondary"

            d = np.sqrt(
                (self.parentMine.x - self.targetMine.x) ** 2
                + (self.parentMine.y - self.targetMine.y) ** 2
            )  # Algabraic Distance Formula

            # Primary node is the first node where it is placed
            # (typically towards the top of the circle)
            # Create Angle Offset(relative to target mine). It changes slightly depending on mines' positions
            # Formula is: arccos(x1-x2)/d+pi

            if self.parentMine.y > self.targetMine.y:
                offsetAngle = (
                    np.arccos(np.clip((self.parentMine.x - self.targetMine.x) / d, -1, 1)) + np.pi
                )
            elif self.parentMine.y < self.targetMine.y:
                offsetAngle = (
                    -np.arccos(np.clip((self.parentMine.x - self.targetMine.x) / d, -1, 1)) + np.pi
                )
            elif self.parentMine.y == self.targetMine.y:
                if self.parentMine.x < self.targetMine.x:
                    offsetAngle = (
                        np.arccos(np.clip((self.parentMine.x - self.targetMine.x) / d, -1, 1))
                        + np.pi
                    )
                elif self.parentMine.x > self.targetMine.x:
                    offsetAngle = (
                        -np.arccos(np.clip((self.parentMine.x - self.targetMine.x) / d, -1, 1))
                        + np.pi
                    )

            # Offset Angle is the same for internal and external bitangents
            """Each pair of angles are mirrored to each other about the offset angle"""
            if self.internal:
                # Create internal angle
                internalArccosParameter = ((self.parentMine.radius) + (self.targetMine.radius)) / d
                internalAngle = np.arccos(np.clip(internalArccosParameter, -1, 1))

                if self.primary:
                    self.angle = internalAngle + offsetAngle
                    self.x = ((self.parentMine.radius) * np.cos(self.angle)) + self.parentMine.x
                    self.y = ((self.parentMine.radius) * np.sin(self.angle)) + self.parentMine.y
                else:
                    self.angle = internalAngle - (offsetAngle)
                    self.x = (self.parentMine.radius) * np.cos(self.angle) + self.parentMine.x
                    self.y = (self.parentMine.radius) * np.sin(
                        self.angle + np.pi
                    ) + self.parentMine.y
            else:
                # Create external angle
                externalArccosParameter = (self.parentMine.radius - self.targetMine.radius) / d
                externalAngle = np.arccos(np.clip(np.abs(externalArccosParameter), -1, 1))

                if self.primary:
                    self.angle = externalAngle + offsetAngle
                    self.x = ((self.parentMine.radius) * np.cos(self.angle)) + self.parentMine.x
                    self.y = ((self.parentMine.radius) * np.sin(self.angle)) + self.parentMine.y
                else:
                    self.angle = externalAngle - offsetAngle + np.pi
                    self.x = (self.parentMine.radius) * np.cos(
                        self.angle - np.pi
                    ) + self.parentMine.x
                    self.y = (self.parentMine.radius) * np.sin(self.angle) + self.parentMine.y

            self.x = round(self.x, 3)
            self.y = round(self.y, 3)

        else:  # This assumes there is no targetMine, only a parentMine,primary, and floatingNode
            d = np.sqrt(
                (self.parentMine.x - self.floatingNode.x) ** 2
                + (self.parentMine.y - self.floatingNode.y) ** 2
            )  # Algabraic Distance Formula
            internalArccosParameter = ((self.parentMine.radius)) / d
            internalAngle = np.arccos(np.clip(internalArccosParameter, -1, 1))

            if self.parentMine.y > self.floatingNode.y:
                offsetAngle = (
                    np.arccos(np.clip((self.parentMine.x - self.floatingNode.x) / d, -1, 1)) + np.pi
                )
            elif self.parentMine.y < self.floatingNode.y:
                offsetAngle = (
                    -np.arccos(np.clip((self.parentMine.x - self.floatingNode.x) / d, -1, 1))
                    + np.pi
                )
            elif self.parentMine.y == self.floatingNode.y:
                if self.parentMine.x < self.floatingNode.x:
                    offsetAngle = (
                        np.arccos(np.clip((self.parentMine.x - self.floatingNode.x) / d, -1, 1))
                        + np.pi
                    )
                elif self.parentMine.x > self.floatingNode.x:
                    offsetAngle = (
                        -np.arccos(np.clip((self.parentMine.x - self.floatingNode.x) / d, -1, 1))
                        + np.pi
                    )

            if self.primary:
                self.angle = internalAngle + offsetAngle
                self.x = ((self.parentMine.radius) * np.cos(self.angle)) + self.parentMine.x
                self.y = ((self.parentMine.radius) * np.sin(self.angle)) + self.parentMine.y
            else:
                self.angle = internalAngle - (offsetAngle)
                self.x = (self.parentMine.radius) * np.cos(self.angle) + self.parentMine.x
                self.y = (self.parentMine.radius) * np.sin(self.angle + np.pi) + self.parentMine.y

        self.angle = (
            np.atan2(self.y - self.parentMine.y, self.x - self.parentMine.x) + 2 * np.pi
        ) % (2 * np.pi)
        # print(self.angle)
        self.x = round(self.x, 3)
        self.y = round(self.y, 3)

    # Get type of path to a node -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"]
    def getPathType(self, node: "Node") -> list:
        types = []

        # Connection based on parent mines
        if node.parentMine != self.parentMine:
            types.append("line")
        elif node.parentMine == self.parentMine:
            types.append("arc")

        # Connection follows bitangency, such that moving from node to node has no sharp/perpendicular turns
        if (
            self.type == "internal primary"
            and node.type == "internal primary"
            or self.type == "external primary"
            and node.type == "external secondary"
            or self.type == "internal secondary"
            and node.type == "internal secondary"
            or self.type == "external secondary"
            and node.type == "external primary"
        ):
            types.append("bitangent")
        else:
            types.append("normal")

        return types

    def getPos(self) -> float:
        return (round(float(self.x), 3), round(float(self.y), 3))

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.__str__()


def _link():
    global Mine, Connection, Node, MineNode, seg, Field
    Mine = m.Mine
    Connection = nc.Connection

    seg = nc.seg
    Field = f.Field
