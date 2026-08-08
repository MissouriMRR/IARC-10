# Simple class which is used in the nodegraph and holds informatation about distance path and type (straight/arc-ed)
# Moved some useful connection functions here as well.
from enum import Enum
import numpy as np
import quads

from flight.pathfinding.nodeField.archive import mine as m
from flight.pathfinding.nodeField.archive import field as f
from flight.pathfinding.nodeField import node as n


def _link():
    global Mine, Connection, Node, MineNode, seg, Field
    Mine = m.Mine

    Node = n.Node
    MineNode = n.MineNode
    Field = f.Field
