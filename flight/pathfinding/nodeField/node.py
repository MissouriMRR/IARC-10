"""Nodes of the field: plain points, and corners of an obstacle outline."""

from flight.pathfinding.common import DIRECTION_VECTOR, nodeDirection


class Node:
    """A point in local field feet."""

    def __init__(self, x, y, name=""):
        self.x = float(x)
        self.y = float(y)
        self.name = name

    def getPos(self):
        return (round(self.x, 3), round(self.y, 3))

    def __repr__(self):
        return self.name or f"Node({self.x:.2f}, {self.y:.2f})"


class MineNode(Node):
    """A corner of an obstacle outline.

    - direction: one nodeDirection for a mine's node (BlockMine.expand moves it along that);
      a tuple of them (outward normals of the two edges meeting here) for outline corners
    - parentMine: the obstacle this corner belongs to
    - convex: obstacle interior angle < 180 (only these are bend points of a shortest path)
    """

    def __init__(self, x, y, direction=(), parentMine=None, convex=True):
        super().__init__(x, y)
        self.direction = direction if isinstance(direction, nodeDirection) else tuple(direction)
        self.parentMine = parentMine
        self.convex = convex

    def __repr__(self):
        return f"MineNode({self.x:.2f}, {self.y:.2f})"


BlockMineNode = MineNode  # old name


def _axis_direction(dx, dy):
    """nodeDirection of an axis-aligned unit-ish vector, None for a slanted one."""
    for d, (vx, vy) in DIRECTION_VECTOR.items():
        if abs(dx * vy - dy * vx) < 1e-9 and dx * vx + dy * vy > 0:
            return d
    return None


def outlineNodes(coords, parentMine=None):
    """MineNodes for a closed ring's corners, counter-clockwise, collinear points dropped.

    - direction: outward normals of the two adjacent edges, when they are axis-aligned
    - convex: whether the corner turns left (interior angle < 180)
    """
    pts = [(float(x), float(y)) for x, y in coords]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    if sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1])) < 0:
        pts.reverse()
    keep = True
    while keep and len(pts) > 3:
        keep = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) < 1e-9:
                del pts[i]
                keep = True
                break
    out = []
    for i, b in enumerate(pts):
        a, c = pts[i - 1], pts[(i + 1) % len(pts)]
        turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        # CCW ring: outward normal of an edge (dx, dy) is (dy, -dx)
        dirs = [_axis_direction(dy, -dx) for dx, dy in ((b[0] - a[0], b[1] - a[1]),
                                                        (c[0] - b[0], c[1] - b[1]))]
        out.append(MineNode(b[0], b[1], tuple(d for d in dirs if d is not None),
                            parentMine, convex=turn > 0))
    return out
