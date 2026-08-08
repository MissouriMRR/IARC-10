"""
Builds a small standalone CellField directly from a protoMine's own
already-computed blockMatrix -- the raw grid of which competition squares
protoMine determined intersect the mine's safety-radius circle (see
protoMine.generateBlocks/circle_rect_intersects). No re-derivation from
geometry happens here at all; this only reshapes data protoMine already
computed into a CellField.

This is a genuinely SEPARATE representation from a BlockMine's own polygon
(the convex-hull outline generateNodes derives FROM this same blockMatrix,
used for node-graph obstacle connectivity/pathfinding) -- the two can
legitimately differ by a cell or two at the boundary, since a convex hull
of the block-edge wrapping vertices can bulge slightly beyond the exact set
of circle-intersecting squares. Keeping both available (see
droneWorkflowTest.py's diagram, which renders both in different colors) is
deliberate, not redundant -- it's the difference between "what the node
graph treats as the obstacle" and "which competition squares are actually
inside the mine's declared safety radius".
"""
from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.protoMine import SQUARE_SIDE_LENGTH_FT


def build_mine_cell_field(proto_mine) -> CellField:
    """
    Returns a new CellField sized exactly to proto_mine.blockMatrix, with
    real-world bounds matching proto_mine.centerGridOffset -- block (0,0)'s
    corner sits at centerGridOffset exactly (see protoMine.centerOfBlock),
    so this lines up with the rest of the field's local coordinate frame
    with no further translation needed -- and every cell set wherever
    blockMatrix says the mine's safety radius touches that competition
    square.
    """
    block_matrix = proto_mine.blockMatrix
    height = len(block_matrix)
    width = len(block_matrix[0]) if height else 0
    if width == 0 or height == 0:
        raise ValueError("protoMine has an empty blockMatrix -- nothing to build a CellField from")

    min_corner = (proto_mine.centerGridOffset[0], proto_mine.centerGridOffset[1])
    max_corner = (min_corner[0] + width * SQUARE_SIDE_LENGTH_FT, min_corner[1] + height * SQUARE_SIDE_LENGTH_FT)
    field = CellField(width, height, min_corner=min_corner, max_corner=max_corner)
    for y, row in enumerate(block_matrix):
        for x, value in enumerate(row):
            if value:
                field.set(x, y)
    return field
