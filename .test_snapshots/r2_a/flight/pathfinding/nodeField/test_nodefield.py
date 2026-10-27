"""Tests for the nodeField package. Real assertions -- runs under pytest or as a script:

    PYTHONPATH="$PWD" python -m flight.pathfinding.nodeField.test_nodefield

The completeness test is the slow one (30 layouts x 50 routes, each against a brute-force
graph over every corner pair); set NODEFIELD_LAYOUTS to run fewer.
"""
import math
import os
import random
import time

import numpy as np
import shapely
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from flight.pathfinding.common import CELL_FT, DIRECTION_VECTOR, nodeDirection
from flight.pathfinding.nodeField import BlockMine, BlockMineNode, Field, MineNode, unionObstacle
from flight.pathfinding.pathnode.engines.zonemap import ZoneMap
from flight.pathfinding.pathnode.geometry import unsafe_cells
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.protoMine import protoMine

FIELD_W, FIELD_H = 80.0, 300.0
LAYOUTS = int(os.environ.get("NODEFIELD_LAYOUTS", 30))


def _square(c, r):
    return box(c * CELL_FT, r * CELL_FT, (c + 1) * CELL_FT, (r + 1) * CELL_FT)


def _layout(n, seed, zones=3, proposals="both"):
    """Field with n random mines plus a few blocked squares."""
    field = Field(FIELD_W, FIELD_H, proposals=proposals)
    for x, y in generate_true_minefield(n, seed):
        field.add_mine(x, y)
    rng = random.Random(seed)
    for _ in range(zones):
        field.add_zone(_square(rng.randrange(3, 37), rng.randrange(3, 147)))
    return field


# -- protoMine ---------------------------------------------------------------------------
def test_protomine_squares_match_unsafe_cells():
    rng = random.Random(1)
    for _ in range(500):
        x, y = rng.uniform(-3, FIELD_W + 3), rng.uniform(-3, FIELD_H + 3)
        assert sorted(protoMine(x, y).squares) == sorted(unsafe_cells(x, y)), (x, y)


def _diamond(cx, cy, reach):
    return Polygon([(cx, cy + reach), (cx - reach, cy), (cx, cy - reach), (cx + reach, cy)])


def _mine(x, y):
    pm = protoMine(x, y)
    return pm, BlockMine(pm.nodeVertices, pm.origin, pm.radius, pm.squares)


def test_diamond_geometry():
    """A real mine is a 4-node diamond, tips 6.05 ft from its square centre, one tag each."""
    rng = random.Random(2)
    tips = {nodeDirection.UP: (0, 6.05), nodeDirection.LEFT: (-6.05, 0),
            nodeDirection.DOWN: (0, -6.05), nodeDirection.RIGHT: (6.05, 0)}
    for _ in range(500):
        x, y = rng.uniform(0, FIELD_W), rng.uniform(0, FIELD_H)
        pm, mine = _mine(x, y)
        cx, cy = pm.origin
        assert len(pm.squares) == 13 and len(mine.nodes) == 4
        assert all(isinstance(n, MineNode) and isinstance(n.direction, nodeDirection)
                   for n in mine.nodes)
        got = {n.direction: (n.x - cx, n.y - cy) for n in mine.nodes}
        assert set(got) == set(tips)
        for d, (tx, ty) in tips.items():
            assert math.isclose(got[d][0], tx, abs_tol=1e-9)
            assert math.isclose(got[d][1], ty, abs_tol=1e-9)
        assert mine.shape.is_valid and mine.shape.geom_type == "Polygon"
        assert mine.shape.symmetric_difference(_diamond(cx, cy, 6.05)).area < 1e-9
        assert math.isclose(mine.shape.convex_hull.area, mine.shape.area)  # convex
        for c, r in pm.squares:  # every unsafe square inside (touching the boundary is fine)
            assert _square(c, r).difference(mine.shape).area < 1e-9
    assert BlockMineNode is MineNode


def test_hull_matches_scipy():
    """protoMine's hull nodes are hull vertices scipy finds too, and enclose every side point."""
    from scipy.spatial import ConvexHull
    rng = random.Random(3)
    for radius in (3.0, 3.7, 5.0, 6.5):
        for _ in range(20):
            pm = protoMine(rng.uniform(0, FIELD_W), rng.uniform(0, FIELD_H), radius)
            pts = [(x, y) for x, y, _ in pm.sidePoints()]
            want = {tuple(np.round(pts[i], 6)) for i in ConvexHull(pts).vertices}
            got = [(round(n.x, 6), round(n.y, 6)) for n in pm.nodeVertices]
            assert set(got) <= want
            hull = Polygon(got)
            assert hull.is_valid and hull.exterior.is_ccw
            assert all(hull.buffer(1e-6).contains(Point(p)) for p in pts)


# -- merging -----------------------------------------------------------------------------
def test_merge_exact_union_resolve_and_hash():
    a, b, c = (20.5, 50.5), (25.0, 50.5), (60.0, 100.0)
    f = Field(FIELD_W, FIELD_H)
    ia, ib, ic = f.add_mine(*a), f.add_mine(*b), f.add_mine(*c)
    assert len(f.unionObstacles) == 1 and len(f.mines) == 1
    union = f.unionObstacles[0]
    assert isinstance(union, unionObstacle) and f.resolve(ia) == f.resolve(ib) == union.id
    assert f.resolve(ic) == ic and ib == union.id
    exact = unary_union([_mine(*a)[1].shape, _mine(*b)[1].shape])
    assert union.shape.symmetric_difference(Polygon(exact.exterior)).area < 1e-9
    assert union.shape.area < exact.convex_hull.area  # not a hull
    # merging again: the union (not the leaf id) is what a resolve of an old union follows
    f.add_mine(22.5, 56.0)
    assert f.resolve(union.id) == f.unionObstacles[0].id != union.id
    assert set(f.groups) == {f.unionObstacles[0].id, ic}
    # touching only at a corner still merges; a gap of one square does not
    g = Field(FIELD_W, FIELD_H)
    g.add_zone(_square(10, 10))
    g.add_zone(_square(11, 11))
    assert len(g.unionObstacles) == 1
    g.add_zone(_square(14, 10))
    assert len(g.unionObstacles) == 1 and len(g.polygonObstacles) == 1
    # order independence of the hash, and of the live shapes
    pts = generate_true_minefield(40, 7)
    f1, f2 = Field(FIELD_W, FIELD_H), Field(FIELD_W, FIELD_H)
    for p in pts:
        f1.add_mine(*p)
    for p in reversed(pts):
        f2.add_mine(*p)
    assert f1.mineHash() == f2.mineHash()
    assert unary_union(list(f1.groups.values())).symmetric_difference(
        unary_union(list(f2.groups.values()))).area < 1e-9
    h = f1.mineHash()
    f1.expandField(0.5)
    assert f1.mineHash() == h


def test_mine_hash_formula_matches_planner():
    from flight.pathfinding.pathnode.planner import PathNodePlanner
    pl = PathNodePlanner.__new__(PathNodePlanner)
    pts = generate_true_minefield(25, 3)
    pl.discovered = list(pts)
    f = Field(FIELD_W, FIELD_H)
    for p in pts:
        f.add_mine(*p)
    assert f.mineHash() == pl.mine_hash()


def test_zonemap_surface():
    f, z = Field(FIELD_W, FIELD_H), ZoneMap(FIELD_W, FIELD_H)
    for x, y in [(40, 100), (44, 100), (10, 20), (70, 200)]:
        f.add_mine(x, y)
        z.add_mine(x, y)
    assert f.last_added is not None and f.last_added.area > 0
    assert f.contains(40, 100) and not f.contains(40, 130) and f.contains(*f.vertices[0]) is False
    assert f.crosses([(40, 90), (40, 110)]) and not f.crosses([(20, 90), (20, 110)])
    assert f.clear_line((20, 90), (20, 110)) and not f.clear_line((40, 90), (40, 110))
    path, ids = f.shortest_path((40, 80), (40, 120))
    assert ids[0] is None and ids[-1] is None and len(path) == len(ids) >= 3
    assert all(i in f.groups for i in ids[1:-1])
    assert f.shortest_path((40, 100), (40, 120)) is None  # start inside an obstacle
    assert f.shortest_path((0, 0), (1, 1))[0] == [(0.0, 0.0), (1.0, 1.0)]
    assert f.vertices.shape[1] == 2 and len(f.vertices) > 0
    assert f.withinField(0, 0) and not f.withinField(-1, 5) and not f.withinField(5, FIELD_H + 1)


# -- graph correctness -------------------------------------------------------------------
def _brute_graph(field):
    """(coords, sparse edge arrays) over every pushed outline corner (reflex ones too)."""
    pts = []
    for o in field._live.values():
        for ring in o.graphRings:
            for n in ring:
                if field.withinField(n.x, n.y) and not field.contains(n.x, n.y):
                    pts.append((n.x, n.y))
    pts = np.array(pts).reshape(-1, 2)
    ea, eb = [], []
    n = len(pts)
    for i in range(n - 1):
        segs = np.stack([np.broadcast_to(pts[i], (n - i - 1, 2)), pts[i + 1:]], axis=1)
        clear = ~shapely.intersects(shapely.linestrings(segs), field.blocker)
        j = np.flatnonzero(clear) + i + 1
        ea.append(np.full(len(j), i))
        eb.append(j)
    ea = np.concatenate(ea) if ea else np.zeros(0, int)
    eb = np.concatenate(eb) if eb else np.zeros(0, int)
    return pts, ea, eb


def _brute_length(field, graph, p, q):
    if field.contains(*p) or field.contains(*q):
        return None
    if field.clear_line(p, q):
        return math.dist(p, q)
    pts, ea, eb = graph
    n = len(pts)
    rows, cols, w = [ea, eb], [eb, ea], [np.hypot(*(pts[ea] - pts[eb]).T)] * 2
    for node, pt in ((n, p), (n + 1, q)):
        segs = np.stack([np.broadcast_to(np.array(pt), pts.shape), pts], axis=1)
        vis = np.flatnonzero(~shapely.intersects(shapely.linestrings(segs), field.blocker))
        d = np.hypot(*(pts[vis] - np.array(pt)).T)
        rows += [np.full(len(vis), node), vis]
        cols += [vis, np.full(len(vis), node)]
        w += [d, d]
    m = coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(n + 2, n + 2)).tocsr()
    d = dijkstra(m, indices=n)[n + 1]
    return None if math.isinf(d) else float(d)


def _path_len(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def _check_paths(field, path):
    """A returned path never enters a grown obstacle nor touches an unsafe square."""
    assert not field.crosses(path)
    line = shapely.LineString(path)
    for leaf in field._leaves:
        for c, r in getattr(leaf, "squares", ()):
            assert not line.intersects(_square(c, r))
        if not hasattr(leaf, "squares"):
            assert not line.intersects(leaf.shape)


def _compare(field, pairs, tol=1e-6):
    graph = _brute_graph(field)
    bad, checked, reach = [], 0, 0
    for p, q in pairs:
        got = field.shortest_path(p, q)
        want = _brute_length(field, graph, p, q)
        checked += 1
        if got is None or want is None:
            if (got is None) != (want is None):
                bad.append((p, q, None if got is None else _path_len(got[0]), want))
            continue
        reach += 1
        _check_paths(field, got[0])
        if abs(_path_len(got[0]) - want) > tol:
            bad.append((p, q, _path_len(got[0]), want))
    return checked, reach, bad


def _pairs(rng, k=50):
    return [((rng.uniform(0, FIELD_W), rng.uniform(0, FIELD_H)),
             (rng.uniform(0, FIELD_W), rng.uniform(0, FIELD_H))) for _ in range(k)]


def test_completeness_vs_bruteforce():
    """The tangent-proposed graph's shortest paths equal a full corner-pair graph's."""
    total = reach = 0
    t0 = time.time()
    for s in range(LAYOUTS):
        n = 20 + (100 * s) // max(LAYOUTS - 1, 1)
        field = _layout(n, 1000 + s)
        rng = random.Random(s)
        checked, r, bad = _compare(field, _pairs(rng))
        total += checked
        reach += r
        assert not bad, (s, n, bad[:3])
    print(f"completeness: {LAYOUTS} layouts, {total} routes ({reach} reachable), 0 mismatches, "
          f"{time.time() - t0:.0f}s")


def test_tangents_alone_and_subset():
    """Report how often the global common tangents alone miss the shortest path, and check
    every tangent-only edge is also a local-proposal edge."""
    miss = total = 0
    for s in range(5):
        tan = _layout(60, 2000 + s, proposals="tangents")
        loc = _layout(60, 2000 + s, proposals="local")
        edge = lambda fc: {frozenset(((a.x, a.y), (b.x, b.y))) for a, b in fc.edges()}
        assert edge(tan.fieldConnection) <= edge(loc.fieldConnection)
        rng = random.Random(s)
        for p, q in _pairs(rng, 30):
            a, b = tan.shortest_path(p, q), loc.shortest_path(p, q)
            total += 1
            if (a is None) != (b is None) or (a and abs(_path_len(a[0]) - _path_len(b[0])) > 1e-6):
                miss += 1
    print(f"tangents-only graph: {miss}/{total} routes differ from the complete graph")


def _sparse_layout(n, seed, proposals):
    """n mines kept 21 ft (city-block) apart, so none merge: only convex diamonds."""
    rng, pts = random.Random(seed), []
    while len(pts) < n:
        p = (rng.uniform(5, FIELD_W - 5), rng.uniform(5, FIELD_H - 5))
        if all(abs(p[0] - q[0]) + abs(p[1] - q[1]) >= 21 for q in pts):
            pts.append(p)
    field = Field(FIELD_W, FIELD_H, proposals=proposals)
    for p in pts:
        field.add_mine(*p)
    return field


def test_tangents_complete_for_separate_mines():
    """Separate mines are convex diamonds: the common tangents alone find every shortest path.
    (The misses reported above come only from merged, non-convex unions.)"""
    total = 0
    for s in range(10):
        tan, loc = _sparse_layout(45, s, "tangents"), _sparse_layout(45, s, "local")
        assert not tan.unionObstacles and len(tan.mines) == 45
        for p, q in _pairs(random.Random(s), 40):
            a, b = tan.shortest_path(p, q), loc.shortest_path(p, q)
            assert (a is None) == (b is None)
            assert a is None or abs(_path_len(a[0]) - _path_len(b[0])) < 1e-6, (s, p, q)
            total += 1
    print(f"tangents-only on separate mines: 0/{total} routes differ")


def _edge_set(fc):
    return {frozenset(((round(a.x, 6), round(a.y, 6)), (round(b.x, 6), round(b.y, 6))))
            for a, b in fc.edges()}


def _assert_graph_consistent(field):
    """nodeGraph is symmetric, holds only live corners, and weights are the edge lengths."""
    fc = field.fieldConnection
    live = {id(n) for k, n in enumerate(fc._nodes) if fc._alive[k]}
    assert {id(n) for n in fc.nodeGraph} == live
    for a, nbrs in fc.nodeGraph.items():
        for b, d in nbrs.items():
            assert fc.nodeGraph[b][a] == d and abs(d - math.dist((a.x, a.y), (b.x, b.y))) < 1e-9


def test_incremental_graph_equals_rebuild():
    """After adds, merges (mines + zones) and expands, nodeGraph == a from-scratch rebuild."""
    rng = random.Random(11)
    field = Field(FIELD_W, FIELD_H)
    for step in range(40):
        if step % 8 == 7:
            field.add_zone(_square(rng.randrange(3, 37), rng.randrange(3, 147)))
        else:  # clustered so merges are frequent
            field.add_mine(rng.uniform(20, 60), rng.uniform(100, 160))
        _assert_graph_consistent(field)
        got = _edge_set(field.fieldConnection)
        nodes = {(round(n.x, 6), round(n.y, 6)) for n in field.fieldConnection.nodeGraph}
        field.fieldConnection.rebuild()
        assert _edge_set(field.fieldConnection) == got, step
        assert {(round(n.x, 6), round(n.y, 6)) for n in field.fieldConnection.nodeGraph} == nodes
        if step == 25:
            field.expandField(0.5)  # expand rebuilds; adds continue incrementally after it
            _assert_graph_consistent(field)
    print(f"incremental == rebuilt after 40 adds + expand ({len(_edge_set(field.fieldConnection))} edges)")


def test_no_query_point_in_graph():
    field = _layout(30, 5)
    fc = field.fieldConnection
    before = (len(fc.nodeGraph), sum(map(len, fc.nodeGraph.values())), len(fc._nodes))
    rng = random.Random(4)
    for p, q in _pairs(rng, 40):
        field.shortest_path(p, q)
    assert (len(fc.nodeGraph), sum(map(len, fc.nodeGraph.values())), len(fc._nodes)) == before
    assert all(isinstance(n, MineNode) and n.parentMine is not None for n in fc.nodeGraph)


# -- expanding ---------------------------------------------------------------------------
def test_expand():
    field = _layout(30, 77)
    field.mineHash()
    ids = list(field.groups)
    for d in (0.5, 0.5, 1.0):
        field.expandField(d)
        for oid in ids:
            assert field.resolve(oid) in field.groups
        for leaf in field._leaves:
            if not hasattr(leaf, "squares"):
                continue
            cx, cy = leaf.origin
            for c, r in unsafe_cells(cx, cy, leaf.radius):
                sq = _square(c, r)
                near = shapely.shortest_line(Point(cx, cy), sq)
                x, y = shapely.get_coordinates(near)[1]
                inside = any(g.buffer(1e-6).contains(Point(x, y)) for g in field.groups.values())
                assert inside, (leaf.origin, leaf.radius, (c, r))
        assert unary_union(list(field.groups.values())).is_valid
        checked, reach, bad = _compare(field, _pairs(random.Random(5), 25))
        assert not bad, bad[:3]
    print("expand: 3 steps, squares inside, paths equal brute force")


def test_expand_stays_diamond_city_block():
    """After k steps of 2 ft: still a 4-node diamond, every node 2k ft further along its tag.

    The diamond |dx|+|dy| <= 6.05 + 2k is the city-block growth of the unsafe squares. In square
    units the unsafe set is |dc|+|dr| <= 2 and the k-step growth is |dc|+|dr| <= 2+k; its
    outermost corners sit at |dx|+|dy| = 6+2k, so the diamond is that staircase's convex hull
    with a 0.05 ft nudge (it contains all of it, and it is not the staircase itself).
    """
    rng = random.Random(9)
    for _ in range(40):
        pm, mine = _mine(rng.uniform(0, FIELD_W), rng.uniform(0, FIELD_H))
        cx, cy = pm.origin
        c0, r0 = int(cx // CELL_FT), int(cy // CELL_FT)
        start = [(n.direction, n.x, n.y) for n in mine.nodes]
        grown = set(pm.squares)
        for k in range(1, 5):
            mine.expand(CELL_FT)
            assert len(mine.nodes) == 4 and mine.shape.is_valid
            for d, x, y in start:
                vx, vy = DIRECTION_VECTOR[d]
                node = next(n for n in mine.nodes if n.direction == d)
                assert math.isclose(node.x, x + vx * CELL_FT * k)
                assert math.isclose(node.y, y + vy * CELL_FT * k)
            assert mine.shape.symmetric_difference(_diamond(cx, cy, 6.05 + 2 * k)).area < 1e-9
            assert math.isclose(mine.radius, pm.radius + 2 * k)
            # one more city-block step: add the 4 edge neighbours of every square
            grown |= {(c + dc, r + dr) for c, r in grown
                      for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))}
            assert grown == {(c0 + a, r0 + b) for a in range(-2 - k, 3 + k)
                             for b in range(-2 - k, 3 + k) if abs(a) + abs(b) <= 2 + k}
            for c, r in grown:
                assert _square(c, r).difference(mine.shape).area < 1e-9
            # tight: squares two steps past the city-block set are outside. (Squares one step
            # past it are partly inside: the diamond is the staircase's hull, plus the nudge.)
            assert _square(c0 + 4 + k, r0).intersection(mine.shape).area < 1e-9
            assert _square(c0, r0 - 4 - k).intersection(mine.shape).area < 1e-9
            assert _square(c0 + 3 + k, r0 + 2).intersection(mine.shape).area < 1e-9


def test_zones_do_not_grow():
    f = Field(FIELD_W, FIELD_H)
    f.add_zone(_square(10, 10))
    f.add_zone(_square(11, 11))          # a merged pair of zones
    f.add_mine(50, 100)
    window = box(18, 18, 26, 26)
    before = [g.wkt for g in f.groups.values() if g.intersects(window)]
    f.expandField(2.0)
    assert [g.wkt for g in f.groups.values() if g.intersects(window)] == before
    mine = next(g for g in f.groups.values() if g.contains(Point(51, 101)))
    assert mine.symmetric_difference(_diamond(51, 101, 8.05)).area < 1e-9
    # a zone that a growing mine reaches merges with it but keeps its own shape
    f.add_mine(28, 12)
    f.expandField(2.0)
    assert all(l.shape.area == 4.0 for l in f._leaves if not hasattr(l, "squares"))


def test_plot_smoke():
    import matplotlib
    matplotlib.use("Agg")
    field = _layout(15, 3)
    path = field.shortest_path((10, 0), (60, 299))[0]
    assert field.plotField(graph=True, path=path, labeled=True) is not None


ALL = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]

if __name__ == "__main__":
    for t in ALL:
        t0 = time.time()
        t()
        print(f"ok {t.__name__} ({time.time() - t0:.1f}s)")
    print(f"{len(ALL)} tests passed")
