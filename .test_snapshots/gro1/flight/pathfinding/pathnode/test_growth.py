"""Tests for mine_rule.apply_growth: the route re-checked against zones that all grew at once.

Growth is simulated by adding several mines to pl.routes without running the rule in between,
then calling apply_growth once. Plain asserts -- runs under pytest or as a script.
"""
from flight.pathfinding.pathnode import mine_rule
from flight.pathfinding.pathnode.test_pathnode import _clear, _consistent, _planner, _rect_photo

LINE = [(40, y) for y in range(0, 110, 10)] + [(40, 300)]


def _line_planner(points=LINE):
    return _planner(points, ["checked"] * (len(points) - 2) + ["forward"])


def _two_cuts():
    pl = _line_planner()
    kept = pl.nodes[3:7]                       # y = 30..60, between the two cuts
    pl.routes.add_mine(40.0, 20.0)
    pl.routes.add_mine(40.0, 70.0)
    return pl, kept


def test_checked_stretch_between_two_cuts_survives():
    pl, kept = _two_cuts()
    res = mine_rule.apply_growth(pl)
    assert res.feasible and res.reason is None
    assert all(n in pl.nodes for n in kept), "collapsed behaviour would orphan these"
    i = pl.nodes.index(kept[0])
    assert all(k.is_checked for k in pl.links[i:i + 3]), "the stretch is still checked"
    assert pl.stats["orphans"] == 0
    assert _clear(pl) and _consistent(pl)
    pl.chain.check()


def test_one_pending_bridge_per_gap():
    pl, _kept = _two_cuts()
    res = mine_rule.apply_growth(pl)
    assert res.bridged == 2 and res.cut >= 2
    assert len(pl.chain.pending_links()) == 2
    assert [k.kind for k in pl.links].count("forward") == 1


def test_gap_with_no_route_is_infeasible_and_leaves_the_chain_valid():
    pts = [(40, 0), (40, 10), (40, 21), (40, 32), (40, 300)]
    pl = _line_planner(pts)
    before_nodes, before_links = list(pl.nodes), list(pl.links)
    before_polys = [list(k.poly) for k in pl.links]
    for c in range(40):                        # a wall of blocked squares across the field
        pl.routes.add_square(c, 10)
    res = mine_rule.apply_growth(pl)
    assert not res.feasible and res.reason
    assert pl.failure is None
    assert pl.nodes == before_nodes and pl.links == before_links
    assert [k.poly for k in pl.links] == before_polys
    pl.chain.check()


def test_entry_inside_a_grown_zone_shifts():
    pl = _line_planner()
    pl.routes.add_mine(40.0, 2.0)
    assert pl.routes.contains(*pl.nodes[0].xy)
    res = mine_rule.apply_growth(pl)
    assert res.feasible
    assert not pl.routes.contains(*pl.nodes[0].xy) and pl.nodes[0].x != 40
    assert pl.stats["entry_shifts"] == 1
    assert _clear(pl) and _consistent(pl)
    pl.chain.check()


def test_calling_twice_changes_nothing():
    pl, _kept = _two_cuts()
    mine_rule.apply_growth(pl)
    sig = pl.chain.signature()
    nodes = list(pl.nodes)
    res = mine_rule.apply_growth(pl)
    assert res.feasible and res.cut == 0 and res.bridged == 0
    assert pl.chain.signature() == sig and pl.nodes == nodes


def test_advance_closes_bridges_over_photographed_squares():
    pl, _kept = _two_cuts()
    mine_rule.apply_growth(pl)
    spans = pl.chain.pending_links()
    assert len(spans) == 2
    for link in spans:
        for x, y in pl.coverage.places_along(link.poly):
            _rect_photo(pl, x, y)
    pl.advance()
    assert not pl.chain.pending_links()
    assert all(k.is_checked for k in pl.links[:-1])
    pl.chain.check()


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}", flush=True)
    print(f"\n{len(tests)}/{len(tests)} passed")
