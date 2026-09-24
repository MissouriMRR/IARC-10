"""Tests for the path-node planner. Real assertions -- runs under pytest or as a script.

The orphan rule never fires in ordinary missions (on a near-straight chain a convex mine
deletes one contiguous run of nodes), so it is exercised here with constructed chains.
"""
import math
import random
import subprocess
import sys

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.sim import Link, PathNode, PathNodeMission, evaluate


def _mission(mine, nodes, links):
    m = PathNodeMission([mine])
    m.nodes, m.links = nodes, links
    m.discovered[mine] = 0
    return m


def _consistent(m):
    return all(math.dist(k.poly[0], m.nodes[i].xy) < 1e-6
               and math.dist(k.poly[-1], m.nodes[i + 1].xy) < 1e-6
               for i, k in enumerate(m.links))


def _chain(points, kinds, placers):
    nodes = [PathNode(x, y, p, 1 if y < 150 else 2) for (x, y), p in zip(points, placers)]
    links = [Link(k, [points[i], points[i + 1]], "a1" if k == "pending" else None)
             for i, k in enumerate(kinds)]
    return nodes, links


def test_footprint_model_matches_fill_polygon_covered():
    rng = random.Random(0)
    for _ in range(500):
        x, y = rng.uniform(3, 77), rng.uniform(4, 296)
        f = CellField(40, 150, max_corner=(80, 300))
        f.fill_polygon_covered([(x - 2, y - 3), (x + 2, y - 3), (x + 2, y + 3), (x - 2, y + 3)])
        assert set(f.on_cells()) == PathNodeMission._footprint_cells(x, y)


def test_mid_chain_cut_is_bridged_not_orphaned():
    """A mine cutting the middle of side 1's chain: the far fragment is still connected to
    point A, so it is kept and re-bridged by a pending span for the side's assistant."""
    pts = [(40, 0), (40, 10), (40, 20), (40, 30), (40, 40), (40, 60), (40, 300)]
    nodes, links = _chain(pts, ["checked"] * 4 + ["forward", "checked"],
                          ["entry1", "g1", "g1", "g1", "g1", "g2", "entry2"])
    n1, n3, n4 = nodes[1], nodes[3], nodes[4]
    m = _mission((40.0, 20.0), nodes, links)
    m._on_mine((40.0, 20.0), "g1")
    assert m.failure is None
    assert m.stats["orphans"] == 0
    assert n3 in m.nodes and n4 in m.nodes, "far fragment is connected to point A -- keep it"
    i = m.nodes.index(n1)
    assert m.nodes[i + 1] is n3 and m.links[i].kind == "pending" and m.links[i].owner == "a1"
    assert not m.zones.crosses(m.links[i].poly)
    assert _consistent(m)


def test_frontier_orphan_deleted_and_gambler_jumps():
    """The mine takes point A and a node behind it, stranding a node between them that is
    outside the mine. That node is an orphan and must go; gambler 1 jumps past the mine."""
    pts = [(40, 0), (40, 20), (48, 24), (40, 28), (40, 40), (40, 300)]
    nodes, links = _chain(pts, ["checked", "checked", "checked", "forward", "checked"],
                          ["entry1", "g1", "g1", "g1", "g2", "entry2"])
    stranded = nodes[2]
    m = _mission((40.0, 24.0), nodes, links)
    assert not m.zones.contains(*stranded.xy) if m.zones.blocker else True
    m._on_mine((40.0, 24.0), "g1")
    assert m.failure is None
    assert not m.zones.contains(*stranded.xy), "the orphan is outside the mine..."
    assert stranded not in m.nodes, "...and is deleted anyway, because nothing reaches it"
    assert m.stats["orphans"] == 1
    assert m.stats["jumps"] == 1
    kinds = [k.kind for k in m.links]
    assert kinds == ["pending", "forward", "checked"], kinds
    assert m.links[0].owner == "a1" and m.nodes[1].placed_by == "A1"
    assert all(not m.zones.crosses(k.poly) for k in m.links)
    assert _consistent(m)


def test_island_between_two_cuts_is_orphaned_after_the_join():
    """A joined route that passes the same mine twice. The stretch between the two cuts is
    outside the mine but reaches neither entry, so it is deleted and the two root fragments
    are re-bridged."""
    pts = [(40, 0), (40, 90), (40, 112), (52, 112), (52, 90), (44, 90), (44, 112),
           (60, 112), (60, 300)]
    nodes, links = _chain(pts, ["checked"] * 8,
                          ["entry1", "g1", "g1", "g1", "g1", "g1", "g1", "g1", "entry2"])
    island = nodes[2:6]
    m = _mission((41.0, 101.0), nodes, links)
    m._on_mine((41.0, 101.0), "a1")
    assert m.failure is None
    assert all(n not in m.nodes for n in island)
    assert all(not m.zones.contains(*n.xy) for n in island), "none of the island is in the mine"
    assert m.stats["orphans"] == 4
    assert [k.kind for k in m.links].count("pending") == 1
    assert all(not m.zones.crosses(k.poly) for k in m.links)
    assert _consistent(m)


def test_assistant_keeps_the_span_its_own_discovery_creates():
    pts = [(40, 0), (40, 10), (40, 20), (40, 30), (40, 300)]
    nodes, links = _chain(pts, ["checked"] * 4, ["entry1", "a2", "a2", "a2", "entry2"])
    m = _mission((40.0, 20.0), nodes, links)
    m._on_mine((40.0, 20.0), "a2")
    pend = [k for k in m.links if k.kind == "pending"]
    assert len(pend) == 1 and pend[0].owner == "a2"


def test_full_missions_are_complete_consistent_and_clear_of_known_mines():
    for seed in range(1, 9):
        for n in (70, 90):
            m = PathNodeMission(generate_true_minefield(n, seed)).run()
            e = evaluate(m)
            assert e["complete"], (seed, n, m.failure)
            assert e["links_consistent"] and e["spans_field"], (seed, n)
            assert e["unseen_cells"] == 0 and e["bad_edges_known"] == 0, (seed, n, e)


def test_band_coverage_leaves_no_route_square_in_danger():
    for seed in range(1, 5):
        m = PathNodeMission(generate_true_minefield(70, seed), coverage="band").run()
        assert evaluate(m)["route_cells_in_danger"] == 0, seed


def test_never_loads_the_old_planner():
    """Run a mission in a fresh interpreter and prove no old-planner module was imported,
    so none of its functions could have run."""
    code = (
        "import sys\n"
        "from flight.pathfinding.pathnode.minefield import generate_true_minefield\n"
        "from flight.pathfinding.pathnode.sim import PathNodeMission, evaluate\n"
        "m = PathNodeMission(generate_true_minefield(90, 3)).run(); evaluate(m)\n"
        "print('\\n'.join(sorted(k for k in sys.modules if k.startswith('flight'))))\n"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         check=True).stdout.split()
    banned = ("flight.pathfinder", "flight.pathfinding.nodeField", "flight.pathfinding.protoMine",
              "flight.pathfinding.path_calculation", "flight.pathfinding.blockField",
              "flight.pathfinding.path_subdivision", "flight.pathfinding.tests",
              "flight.pathfinding.mineCellField", "state_machine")
    loaded = [mod for mod in out if mod.startswith(banned)]
    assert not loaded, loaded
    return out


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print(f"\n{len(tests)}/{len(tests)} passed")
