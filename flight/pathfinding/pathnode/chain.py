"""The chain: one side's route as path nodes joined by links.

    nodes:  [entry] --link0-- [n1] --link1-- [n2] ... [point A] --forward-- [target]

- Invariant: links[i] joins nodes[i] to nodes[i + 1]; len(links) == len(nodes) - 1; a link's
  polyline starts at its first node and ends at its second (Chain.check).
- A link is CHECKED (photographed), PENDING (a planned detour, still to photograph) or
  FORWARD (point A -> target, the gap still to explore; at most one).
- Point A is the node before the forward link. With no forward link the side has met its
  target, and point A is the target itself.
- The target is virtual: the partner's latest point A, or a far-edge point. Never laid, never
  deleted by a mine.
"""

import itertools
import math
from dataclasses import dataclass, field

from flight.pathfinding.pathnode.config import LinkKind, Placer
from flight.pathfinding.pathnode.polyline import dedupe, poly_len, samples


@dataclass(eq=False)
class PathNode:
    x: float
    y: float
    placed_by: Placer

    @property
    def xy(self):
        return (self.x, self.y)


@dataclass(eq=False)
class Link:
    kind: LinkKind
    poly: list
    uid: int = field(default_factory=itertools.count().__next__)  # names a pending span


class Chain:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links

    @classmethod
    def build(cls, points, kinds):
        """A chain through `points` (first = entry, last = target) joined by straight links of
        `kinds` -- for tests and hand-built cases."""
        n = len(points)
        nodes = [PathNode(float(x), float(y),
                          Placer.ENTRY if i == 0 else Placer.TARGET if i == n - 1 else Placer.FORWARD)
                 for i, (x, y) in enumerate(points)]
        links = [Link(LinkKind(k), [tuple(points[i]), tuple(points[i + 1])])
                 for i, k in enumerate(kinds)]
        chain = cls(nodes, links)
        chain.check()
        return chain

    def check(self):
        """Raise AssertionError unless the invariant holds (see the module doc)."""
        assert len(self.links) == len(self.nodes) - 1, (len(self.nodes), len(self.links))
        for i, k in enumerate(self.links):
            assert math.dist(k.poly[0], self.nodes[i].xy) < 1e-6, i
            assert math.dist(k.poly[-1], self.nodes[i + 1].xy) < 1e-6, i
        assert sum(k.kind == LinkKind.FORWARD for k in self.links) <= 1

    # -- queries ---------------------------------------------------------------------------
    def forward_index(self):
        return next((i for i, k in enumerate(self.links) if k.kind == LinkKind.FORWARD), None)

    def point_a(self):
        f = self.forward_index()
        return self.nodes[f].xy if f is not None else self.nodes[-1].xy

    def route(self):
        """Every point from the entry to the target."""
        pts = [self.nodes[0].xy]
        for k in self.links:
            pts.extend(k.poly[1:])
        return pts

    def all_checked(self):
        return all(k.kind == LinkKind.CHECKED for k in self.links)

    def has_pending(self):
        return any(k.kind == LinkKind.PENDING for k in self.links)

    def unchecked(self):
        """The forward gap and every pending span."""
        return [k for k in self.links if k.kind in (LinkKind.PENDING, LinkKind.FORWARD)]

    def index(self, node):
        return next((k for k, n in enumerate(self.nodes) if n is node), None)

    def section(self, i, j):
        """Length of the chain from node i to node j, or None unless it is all checked."""
        links = self.links[i:j]
        if not links or any(k.kind != LinkKind.CHECKED for k in links):
            return None
        return sum(poly_len(k.poly) for k in links)

    def signature(self):
        """Everything about the chain that the work lists depend on."""
        return (self.nodes[0].xy,
                tuple((k.kind, k.uid, tuple(map(tuple, k.poly))) for k in self.links))

    # -- edits (each keeps the invariant) -------------------------------------------------
    def splice(self, i, new_nodes, new_links):
        """Replace link i with new_links, joined by new_nodes."""
        assert len(new_links) == len(new_nodes) + 1
        self.nodes[i + 1:i + 1] = new_nodes
        self.links[i:i + 1] = new_links

    def replace(self, i, j, poly, placer):
        """Replace everything between nodes i and j with checked links along poly."""
        self.nodes[i + 1:j] = [PathNode(x, y, placer) for x, y in poly[1:-1]]
        self.links[i:j] = [Link(LinkKind.CHECKED, [poly[k], poly[k + 1]])
                           for k in range(len(poly) - 1)]

    def set_target(self, xy):
        self.nodes[-1] = PathNode(xy[0], xy[1], Placer.TARGET)

    def convert_checked(self, i, placer):
        poly = self.links[i].poly
        self.splice(i, [PathNode(x, y, placer) for x, y in poly[1:-1]],
                    [Link(LinkKind.CHECKED, [poly[k], poly[k + 1]]) for k in range(len(poly) - 1)])

    def advance_link(self, link, placer, all_seen):
        """Lay path nodes over the photographed prefix of `link`, from its start (all_seen:
        polyline -> whether it is all photographed). The same Link object stays as the
        unchecked remainder. Returns True once it is fully checked."""
        i = next((j for j, k in enumerate(self.links) if k is link), None)
        if i is None:
            return False
        poly = link.poly
        pts = samples(poly)
        good = 0
        for k in range(1, len(pts)):
            if not all_seen([pts[k - 1][0], pts[k][0]]):
                break
            good = k
        if good == 0:
            return False
        if good == len(pts) - 1:
            self.convert_checked(i, placer)
            return True
        end_pt, seg = pts[good]
        head = dedupe(poly[:seg + 1] + [end_pt])
        tail = dedupe([end_pt] + poly[seg + 1:])
        if len(head) < 2 or len(tail) < 2:
            return False
        link.poly = tail
        self.splice(i, [PathNode(x, y, placer) for x, y in head[1:]],
                    [Link(LinkKind.CHECKED, [head[k], head[k + 1]]) for k in range(len(head) - 1)]
                    + [link])
        return False
