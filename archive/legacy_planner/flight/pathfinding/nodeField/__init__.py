"""Public surface of the nodeField package.

Modules that do ``import flight.pathfinding.nodeField as nodeg`` and then
reference ``nodeg.Node`` / ``nodeg.Field`` rely on these re-exports.

This used to call ``node._link()`` / ``field._link()`` / ``mine._link()`` to
resolve circular forward references; that whole chain was commented out and
went stale (see the "Dead code path" notes in node.py and field.py). Plain
submodule imports are enough -- the live modules already import each other
directly.
"""

from flight.pathfinding.nodeField.node import Node, MineNode
from flight.pathfinding.nodeField.field import Field, FieldConnections, seg
from flight.pathfinding.nodeField.BlockMine import BlockMine
from flight.pathfinding.nodeField.BlockMineNode import BlockMineNode

__all__ = [
    "Node",
    "MineNode",
    "Field",
    "FieldConnections",
    "seg",
    "BlockMine",
    "BlockMineNode",
]
