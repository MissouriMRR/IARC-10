"""Field: the obstacles (source of truth) and the visibility graph routes are searched on.

Obstacles:
- BlockMine (a mine's diamond: hull of its side points, see protoMine) and PolygonObstacle
  (a blocked square, never grows) merge into a unionObstacle (exact union outline) when they touch or overlap.
- Everything is tested grown by VISIBILITY_MARGIN_FT; graph corners are the outline pushed out
  VERTEX_INFLATE_FT, so a route can hug a corner without entering the grown obstacle.

Graph (FieldConnections): nodes are the convex outline corners, edges are straight lines clear
of the grown obstacles. Candidate edges come from tangents, never from all corner pairs.
"""

import hashlib
import heapq
import math

import numpy as np
import shapely
from shapely.geometry import Point
from shapely.ops import unary_union

from flight.pathfinding.common import VERTEX_INFLATE_FT, VISIBILITY_MARGIN_FT, snap_to_cell_center
from flight.pathfinding.nodeField.BlockMine import BlockMine
from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle
from flight.pathfinding.nodeField.unionObstacle import unionObstacle

# A line may pass between the pushed corner ring and the grown obstacle: the ring sits
# VERTEX_INFLATE_FT out, the obstacle is grown VISIBILITY_MARGIN_FT. So "both neighbours on
# one side of the line" is tested with this much slack (perpendicular distance).
_SLACK = 1.5 * (VERTEX_INFLATE_FT - VISIBILITY_MARGIN_FT)  # 1.5: mitre corners reach further


def _supports(c1, c2, dist):
    """Line supports a corner: neither neighbour is clearly on each side (c = cross products,
    dist = line length)."""
    t = _SLACK * dist
    return ~(((c1 < -t) & (c2 > t)) | ((c1 > t) & (c2 < -t)))
_CHUNK = 128


class Field:
    """All known obstacles of a field_w x field_h ft field (local feet, origin bottom-left).

    - mines / unionObstacles / polygonObstacles: live standalone mines, unions, other zones
    - proposals: how the graph's candidate edges are found (see FieldConnections)
    """

    def __init__(self, field_w, field_h, proposals="both"):
        self.field_w, self.field_h = field_w, field_h
        self.mines = []
        self.unionObstacles = []
        self.polygonObstacles = []
        self.last_added = None
        self.blocker = None         # every live obstacle grown by the margin (prepared)
        self._byId = {}             # every obstacle ever made, for resolve()
        self._live = {}             # id -> live obstacle
        self._leaves = []           # every mine / zone, however merged
        self._nextId = 0
        self.fieldConnection = FieldConnections(self, proposals)

    # -- adding ----------------------------------------------------------------------------
    def add_mine(self, x, y):
        """Add a mine at (x, y). Returns the id of the obstacle it ended up in."""
        from flight.pathfinding.protoMine import protoMine  # protoMine imports nodeField
        return self.addFromProtoMine(protoMine(x, y))

    def addFromProtoMine(self, proto):
        return self.addObstacle(BlockMine(proto.nodeVertices, proto.origin, proto.radius,
                                          proto.squares))

    def add_zone(self, shape):
        """Add any shapely zone (e.g. a blocked square). Returns its obstacle id."""
        return self.addObstacle(PolygonObstacle.fromShape(shape))

    def addObstacle(self, obstacle):
        """Add an obstacle, merging it with every live obstacle within the margin.
        Returns the id of the live obstacle that now holds it (a union if it merged)."""
        obstacle.id = self._newId()
        self._byId[obstacle.id] = obstacle
        self._leaves.append(obstacle)
        self.last_added = obstacle.grown
        absorbed = self._touching(obstacle)
        live = obstacle
        if absorbed:
            live = unionObstacle([obstacle] + absorbed)
            live.id = self._newId()
            self._byId[live.id] = live
            for o in [obstacle] + absorbed:
                o.mergedInto = live
                self._live.pop(o.id, None)
            self._forget(absorbed)
        self._live[live.id] = live
        self._file(live)
        self.blocker = obstacle.grown if self.blocker is None else \
            unary_union([self.blocker, live.grown])
        shapely.prepare(self.blocker)
        self.fieldConnection.addObstacle(live, absorbed)
        return live.id

    def _touching(self, obstacle):
        live = list(self._live.values())
        if not live:
            return []
        hit = shapely.intersects(np.array([o.grown for o in live], dtype=object), obstacle.grown)
        return [o for o, h in zip(live, hit) if h]

    def _forget(self, gone):
        """Drop obstacles from the type lists."""
        gone = {id(o) for o in gone}
        self.mines = [o for o in self.mines if id(o) not in gone]
        self.unionObstacles = [o for o in self.unionObstacles if id(o) not in gone]
        self.polygonObstacles = [o for o in self.polygonObstacles if id(o) not in gone]

    def _file(self, live):
        if isinstance(live, unionObstacle):
            self.unionObstacles.append(live)
        elif isinstance(live, BlockMine):
            self.mines.append(live)
        else:
            self.polygonObstacles.append(live)

    def _newId(self):
        self._nextId += 1
        return self._nextId - 1

    def resolve(self, oid):
        """The live obstacle id a (possibly since merged) id belongs to now."""
        o = self._byId[oid]
        while o.mergedInto is not None:
            o = o.mergedInto
        return o.id

    # -- expanding -------------------------------------------------------------------------
    def expandField(self, distance):
        """Grow every mine (also those inside unions) by distance, re-merge, rebuild.

        - blocked zones (add_zone) do not grow

        Hook: the growth policy (how far, when) is decided by the caller; this only makes the
        geometry and graph correct for one step.
        """
        if not self._leaves:
            return
        oldUnions = list(self.unionObstacles)
        for leaf in self._leaves:
            if leaf.grows:
                leaf.expand(distance)
            leaf.mergedInto = None
        self._regroup()
        for u in oldUnions:
            leaf = u.obstacleList[0]
            while leaf.mergedInto is not None:
                leaf = leaf.mergedInto
            u.mergedInto = leaf

    def _regroup(self):
        """Merge the leaves into live obstacles from scratch, then rebuild blocker and graph."""
        leaves = self._leaves
        parent = list(range(len(leaves)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        tree = shapely.STRtree([leaf.grown for leaf in leaves])
        for a, b in zip(*tree.query([leaf.grown for leaf in leaves], predicate="intersects")):
            parent[find(a)] = find(b)
        groups = {}
        for i, leaf in enumerate(leaves):
            groups.setdefault(find(i), []).append(leaf)
        self.mines, self.unionObstacles, self.polygonObstacles = [], [], []
        self._live = {}
        for members in groups.values():
            live = members[0]
            if len(members) > 1:
                live = unionObstacle(members)
                live.id = self._newId()
                self._byId[live.id] = live
                for m in members:
                    m.mergedInto = live
            self._live[live.id] = live
            self._file(live)
        self.blocker = unary_union([o.grown for o in self._live.values()])
        shapely.prepare(self.blocker)
        self.fieldConnection.rebuild()

    # -- ZoneMap-compatible queries ----------------------------------------------------------
    @property
    def groups(self):
        """id -> shapely geometry of every live obstacle."""
        return {oid: o.shape for oid, o in self._live.items()}

    @property
    def vertices(self):
        """N x 2 array of the graph's corners."""
        return self.fieldConnection.vertices

    def withinField(self, x, y):
        return 0 <= x <= self.field_w and 0 <= y <= self.field_h

    def contains(self, x, y):
        """Inside some obstacle grown by the margin."""
        return self.blocker is not None and bool(self.blocker.contains(Point(x, y)))

    def crosses(self, poly):
        """Whether a polyline enters any grown obstacle."""
        if self.blocker is None or len(poly) < 2:
            return False
        return bool(shapely.intersects(shapely.linestrings(poly), self.blocker))

    def clear_line(self, p, q):
        return not self.crosses([p, q])

    def shortest_path(self, p, q):
        """Shortest clear polyline p -> q as (points, ids), ids[i] the obstacle whose corner
        points[i] is (None at both ends). None if unreachable or an end is inside an obstacle."""
        p, q = (float(p[0]), float(p[1])), (float(q[0]), float(q[1]))
        if self.contains(*p) or self.contains(*q):
            return None
        if self.clear_line(p, q):
            return [p, q], [None, None]
        return self.fieldConnection.route(p, q)

    def mineHash(self):
        """Order-independent hash of the known mines by their snapped square. Same formula as
        PathNodePlanner.mine_hash; unchanged by expansion (origins never move)."""
        mines = list(self.mines) + [m for u in self.unionObstacles for m in u.mines()]
        cells = sorted(snap_to_cell_center(*m.origin) for m in mines)
        return hashlib.sha256(repr(cells).encode("utf-8")).hexdigest()

    # -- plotting --------------------------------------------------------------------------
    def plotField(self, ax=None, graph=False, path=None, labeled=False, show=False):
        """Draw the field border, obstacles, (optionally) graph edges and a path [(x, y), ...]."""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as PolyPatch

        if ax is None:
            _, ax = plt.subplots(figsize=(6, 6 * self.field_h / self.field_w))
        ax.plot([0, self.field_w, self.field_w, 0, 0], [0, 0, self.field_h, self.field_h, 0],
                color="black", linewidth=1)
        for o in self._live.values():
            color = "tab:purple" if isinstance(o, unionObstacle) else "tab:red"
            for ring in o.rings:
                ax.add_patch(PolyPatch([(n.x, n.y) for n in ring], closed=True, alpha=0.4,
                                       facecolor=color, edgecolor=color))
            if labeled:
                c = o.shape.centroid
                ax.annotate(str(o.id), (c.x, c.y), ha="center", fontsize=7)
        if graph:
            for a, b in self.fieldConnection.edges():
                ax.plot([a.x, b.x], [a.y, b.y], color="tab:gray", linewidth=0.5)
            v = self.vertices
            ax.plot(v[:, 0], v[:, 1], ".", color="tab:blue", markersize=3)
        if path:
            ax.plot([p[0] for p in path], [p[1] for p in path], "-o", color="tab:green",
                    linewidth=2, markersize=4)
        ax.set_aspect("equal")
        if show:
            plt.show()
        return ax


class FieldConnections:
    """The visibility graph over a Field's convex outline corners (pushed-out MineNodes).

    Candidate edges (proposals):
    - "local": corner pairs whose line supports the outline at BOTH ends (a reduced
      visibility graph; complete for non-convex outlines)
    - "tangents": each obstacle's own outline edges plus the common tangents of every
      obstacle pair (PolygonObstacle.commonTangents); query points use pointTangents.
      NOT complete on non-convex outlines: a taut path can bend on a local tangent that
      is not a global one (a notch corner), which the 4 common tangents per pair miss.
    - "both" (default): the union of the two.
    Every candidate is validated against the grown obstacles (batched shapely).

    Stored state:
    - nodeGraph: {MineNode: {MineNode: distance}}, the graph itself; route() searches it
      directly. Updated per obstacle: covered corners and blocked edges are dropped, only the
      new corners are linked. Query points are never stored in it.
    - _nodes / _xy / _alive: corner geometry as arrays (node.k = row), only for the
      vectorised proposal and validation
    """

    def __init__(self, field, proposals="both"):
        assert proposals in ("local", "tangents", "both")
        self.field = field
        self.proposals = proposals
        self._reset()

    def _reset(self):
        self.nodeGraph = {}                        # MineNode -> {MineNode: distance}, THE graph
        self._nodes = []                           # k -> MineNode (node.k is its index)
        self._xy = np.zeros((0, 6))                # per k: x, y, prev x, prev y, next x, next y
        self._alive = np.zeros(0, dtype=bool)

    # -- views -----------------------------------------------------------------------------
    @property
    def vertices(self):
        return self._xy[self._alive][:, :2].reshape(-1, 2)

    def edges(self):
        """Each graph edge once, as (MineNode, MineNode)."""
        return [(a, b) for a, nbrs in self.nodeGraph.items() for b in nbrs if a.k < b.k]

    # -- updating --------------------------------------------------------------------------
    def rebuild(self):
        """Throw the graph away and link every live obstacle."""
        self._reset()
        new = []
        for o in self.field._live.values():
            new += self._register(o)
        self._link(new, list(self.field._live.values()))

    def _kill(self, k):
        """Take corner k out of the graph with its edges."""
        self._alive[k] = False
        node = self._nodes[k]
        for nb in self.nodeGraph.pop(node, ()):
            del self.nodeGraph[nb][node]

    def addObstacle(self, obstacle, absorbed):
        """Fold a new live obstacle into nodeGraph. absorbed: live obstacles it replaced.

        - drop the corners of absorbed obstacles and corners the new grown shape covers
        - drop the edges it blocks (bounding box first, then one batched shapely check)
        - register its own corners and link only those
        """
        gone = {id(o) for o in absorbed}
        for k in np.flatnonzero(self._alive):
            if id(self._nodes[k].parentMine) in gone:
                self._kill(k)
        idx = np.flatnonzero(self._alive)
        if len(idx):
            covered = shapely.contains_xy(obstacle.grown, self._xy[idx, 0], self._xy[idx, 1])
            for k in idx[covered]:
                self._kill(k)
        x0, y0, x1, y1 = obstacle.grown.bounds
        cand = []
        for a, nbrs in self.nodeGraph.items():
            ax, ay = a.x, a.y
            for b in nbrs:
                if a.k < b.k:
                    bx, by = b.x, b.y
                    if not (max(ax, bx) < x0 or min(ax, bx) > x1 or
                            max(ay, by) < y0 or min(ay, by) > y1):
                        cand.append((a, b))
        if cand:
            segs = np.array([[(a.x, a.y), (b.x, b.y)] for a, b in cand])
            for (a, b), hit in zip(cand, shapely.intersects(shapely.linestrings(segs),
                                                            obstacle.grown)):
                if hit:
                    del self.nodeGraph[a][b]
                    del self.nodeGraph[b][a]
        self._link(self._register(obstacle), [obstacle])

    def _register(self, obstacle):
        """Give the obstacle's usable corners (convex, in the field, not covered) graph slots."""
        cand = []
        for ring in obstacle.graphRings:
            n = len(ring)
            for i, node in enumerate(ring):
                node.k = None
                if node.convex and self.field.withinField(node.x, node.y):
                    p, q = ring[i - 1], ring[(i + 1) % n]
                    cand.append((node, (node.x, node.y, p.x, p.y, q.x, q.y)))
        if not cand:
            return []
        if self.field.blocker is not None:
            xs = np.array([c[1][0] for c in cand])
            ys = np.array([c[1][1] for c in cand])
            cand = [c for c, hit in zip(cand, shapely.contains_xy(self.field.blocker, xs, ys))
                    if not hit]
        base = len(self._nodes)
        for node, row in cand:
            node.k = len(self._nodes)
            self._nodes.append(node)
            self.nodeGraph[node] = {}
        self._xy = np.vstack([self._xy, np.array([c[1] for c in cand]).reshape(-1, 6)])
        self._alive = np.concatenate([self._alive, np.ones(len(cand), dtype=bool)])
        return list(range(base, len(self._nodes)))

    def _link(self, newK, newObstacles):
        """Propose, validate and add the edges touching the new corners."""
        if not newK:
            return
        pairs = []
        if self.proposals in ("local", "both"):
            pairs.append(self._localPairs(np.array(newK)))
        if self.proposals in ("tangents", "both"):
            pairs.append(self._tangentPairs(newObstacles))
        pairs = np.vstack(pairs).astype(np.int64).reshape(-1, 2)
        if not len(pairs):
            return
        pairs = np.unique(np.sort(pairs, axis=1), axis=0)
        pairs = pairs[pairs[:, 0] != pairs[:, 1]]
        blocked = np.zeros(len(pairs), dtype=bool)
        for s in range(0, len(pairs), 20000):
            a, b = pairs[s:s + 20000, 0], pairs[s:s + 20000, 1]
            segs = np.stack([self._xy[a, :2], self._xy[b, :2]], axis=1)
            blocked[s:s + 20000] = shapely.intersects(shapely.linestrings(segs), self.field.blocker)
        graph, nodes = self.nodeGraph, self._nodes
        for a, b in pairs[~blocked].tolist():
            na, nb = nodes[a], nodes[b]
            graph[na][nb] = graph[nb][na] = math.hypot(na.x - nb.x, na.y - nb.y)

    def _localPairs(self, newK):
        """Pairs (new corner, any live corner) whose line supports both outlines locally."""
        allK = np.flatnonzero(self._alive)
        isNew = np.zeros(len(self._nodes), dtype=bool)
        isNew[newK] = True
        vx, vy = self._xy[allK, 0], self._xy[allK, 1]
        pvx, pvy = self._xy[allK, 2] - vx, self._xy[allK, 3] - vy
        nvx, nvy = self._xy[allK, 4] - vx, self._xy[allK, 5] - vy
        out = []
        for s in range(0, len(newK), _CHUNK):
            ks = newK[s:s + _CHUNK]
            ux, uy = self._xy[ks, 0][:, None], self._xy[ks, 1][:, None]
            pux, puy = self._xy[ks, 2][:, None] - ux, self._xy[ks, 3][:, None] - uy
            nux, nuy = self._xy[ks, 4][:, None] - ux, self._xy[ks, 5][:, None] - uy
            dx, dy = vx[None, :] - ux, vy[None, :] - uy
            dist = np.hypot(dx, dy)
            ok = _supports(dx * puy - dy * pux, dx * nuy - dy * nux, dist)
            ok &= _supports(dx * pvy - dy * pvx, dx * nvy - dy * nvx, dist)
            ok &= ks[:, None] != allK[None, :]
            ok &= ~(isNew[allK][None, :] & (allK[None, :] < ks[:, None]))
            i, j = np.nonzero(ok)
            out.append(np.stack([ks[i], allK[j]], axis=1))
        return np.vstack(out)

    def _tangentPairs(self, newObstacles):
        """Outline edges of the new obstacles + common tangents new x every live obstacle."""
        out = []
        for o in newObstacles:
            for ring in o.graphRings:
                for i, a in enumerate(ring):
                    out.append((a, ring[(i + 1) % len(ring)]))
            for other in self.field._live.values():
                if other is o or (other in newObstacles and other.id < o.id):
                    continue
                out.extend(o.commonTangents(other))
        return np.array([[a.k, b.k] for a, b in out if self._usable(a) and self._usable(b)],
                        dtype=np.int64).reshape(-1, 2)

    def _usable(self, node):
        k = getattr(node, "k", None)
        return k is not None and k < len(self._nodes) and self._nodes[k] is node and self._alive[k]

    # -- queries ---------------------------------------------------------------------------
    def _candidates(self, p):
        """Live corners the tangent lines from p can touch (not yet checked for visibility):
        local tangency at the corner, or pointTangents when proposals == "tangents"."""
        live = list(self.field._live.values())
        # p between an obstacle's grown shape and its pushed corner ring sees that
        # obstacle's corners at any angle: take them all
        near = [n.k for o, hit in zip(live, shapely.contains_xy(
                np.array([o.pushed for o in live], dtype=object), p[0], p[1])) if hit
                for ring in o.graphRings for n in ring if self._usable(n)]
        if self.proposals == "tangents":
            ks = [n.k for o in live for n in o.pointTangents(p) if self._usable(n)]
            return np.array(sorted(set(ks + near)), dtype=np.int64)
        ks = np.flatnonzero(self._alive)
        xy = self._xy[ks]
        dx, dy = xy[:, 0] - p[0], xy[:, 1] - p[1]
        ok = _supports(dx * (xy[:, 3] - xy[:, 1]) - dy * (xy[:, 2] - xy[:, 0]),
                       dx * (xy[:, 5] - xy[:, 1]) - dy * (xy[:, 4] - xy[:, 0]),
                       np.hypot(dx, dy))
        return np.union1d(ks[ok], np.array(near, dtype=np.int64)).astype(np.int64)

    def _clear(self, a, b):
        return not shapely.intersects(shapely.linestrings([a, b]), self.field.blocker)

    def route(self, p, q):
        """Shortest p -> corners -> q as (points, ids), or None. p, q are outside obstacles.

        A* straight over nodeGraph (neighbours and weights read from nodeGraph[node]). The edges from p and to q are only checked for visibility
        when the search reaches them (most tangent candidates are never needed).
        """
        candP, candQ = self._candidates(p), self._candidates(q)
        if not len(candP) or not len(candQ):
            return None
        graph, nodes = self.nodeGraph, self._nodes
        xy = self._xy
        toQ = np.hypot(xy[:, 0] - q[0], xy[:, 1] - q[1])
        heap = []
        for k in candP:
            g = math.hypot(xy[k, 0] - p[0], xy[k, 1] - p[1])
            heap.append((g + toQ[k], g, int(k), -1))
        heapq.heapify(heap)
        isQ = set(candQ.tolist())
        closed = {}           # k -> (g, parent)
        goal = -2
        while heap:
            f, g, k, parent = heapq.heappop(heap)
            if k in closed:
                continue
            here = p if parent == -1 else None
            if k == goal:
                if not self._clear((xy[parent, 0], xy[parent, 1]), q):
                    continue
                ks = [parent]
                while closed[ks[-1]][1] >= 0:
                    ks.append(closed[ks[-1]][1])
                ks.reverse()
                pts = [p] + [(float(xy[i, 0]), float(xy[i, 1])) for i in ks] + [q]
                return pts, [None] + [self._nodes[i].parentMine.id for i in ks] + [None]
            if here is not None and not self._clear(p, (xy[k, 0], xy[k, 1])):
                continue
            closed[k] = (g, parent)
            if k in isQ:
                d = g + toQ[k]
                heapq.heappush(heap, (d, d, goal, k))
            for node, w in graph[nodes[k]].items():
                v = node.k
                if v not in closed:
                    heapq.heappush(heap, (g + w + toQ[v], g + w, v, k))
        return None
