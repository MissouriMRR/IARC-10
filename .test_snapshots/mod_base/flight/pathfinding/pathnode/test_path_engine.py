"""Conformance suite for PathEngine implementations. Plain asserts -- runs as a script:

    PYTHONPATH="$PWD" python -m flight.pathfinding.pathnode.test_path_engine

Every test runs against EVERY entry of engines.ENGINES. It checks the contract written in
path_engine.py, not equality between engines. A genuine engine gap is listed in KNOWN_GAPS
((test name, engine name) -> message): it is reported as KNOWN GAP when it fails, never skipped.
"""
import math
import random

import shapely

from flight.pathfinding.pathnode.config import Edge
from flight.pathfinding.pathnode.engines import ENGINES, make_engine
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.polyline import poly_len

W, H = 80.0, 300.0
KNOWN_GAPS = {}


def _engine(name):
    return make_engine(name, W, H)


def _length(route):
    return math.inf if route is None else poly_len(route[0])


def _mines(n, seed):
    return generate_true_minefield(n, seed)


def test_empty_field_gives_the_straight_line(name):
    e = _engine(name)
    p, q = (10.0, 5.0), (60.0, 250.0)
    assert e.shortest_path(p, q) == ([p, q], [None, None])
    assert e.clear_line(p, q) and not e.crosses([p, q]) and not e.contains(*p)
    assert e.corners() == [] and e.obstacle_shapes() == {}


def test_path_ends_are_exactly_p_and_q_and_gids_line_up(name):
    e = _engine(name)
    for x, y in _mines(25, 3):
        e.add_mine(x, y)
    rng = random.Random(1)
    bent = 0
    for _ in range(40):
        p = (rng.uniform(1, W - 1), rng.uniform(1, 20))
        q = (rng.uniform(1, W - 1), rng.uniform(H - 20, H - 1))
        route = e.shortest_path(p, q)
        if route is None:
            continue
        pts, gids = route
        assert pts[0] == p and pts[-1] == q, (pts[0], p, pts[-1], q)
        assert len(gids) == len(pts) and gids[0] is None and gids[-1] is None
        assert all(g is not None for g in gids[1:-1])
        assert all(e.resolve(g) in e.obstacle_shapes() for g in gids[1:-1])
        bent += len(pts) > 2
    assert bent, "the cases never needed a bend, so the check proved nothing"


def test_a_returned_path_does_not_cross(name):
    e = _engine(name)
    for x, y in _mines(40, 5):
        e.add_mine(x, y)
    e.add_square(20, 70)
    rng = random.Random(2)
    n = 0
    for _ in range(40):
        p = (rng.uniform(1, W - 1), rng.uniform(1, H - 1))
        q = (rng.uniform(1, W - 1), rng.uniform(1, H - 1))
        route = e.shortest_path(p, q)
        if route is not None:
            assert not e.crosses(route[0]), (p, q)
            n += 1
    assert n >= 20, n


def test_an_endpoint_inside_an_obstacle_gives_none(name):
    e = _engine(name)
    e.add_mine(40.0, 100.0)
    inside = (41.0, 101.0)
    assert e.contains(*inside)
    assert e.shortest_path(inside, (40.0, 200.0)) is None
    assert e.shortest_path((40.0, 10.0), inside) is None


def test_a_walled_off_target_gives_none(name):
    e = _engine(name)
    for c in range(int(W // 2)):
        e.add_square(c, 50)
    assert e.shortest_path((40.0, 10.0), (40.0, 200.0)) is None
    assert e.shortest_path((40.0, 10.0), (20.0, 30.0)) is not None


def test_add_square_makes_the_squares_centre_contained(name):
    e = _engine(name)
    gid = e.add_square(7, 11)
    assert e.contains(7 * 2.0 + 1.0, 11 * 2.0 + 1.0)
    assert not e.contains(30.0, 30.0)
    assert e.resolve(gid) in e.obstacle_shapes()


def test_overlapping_mines_resolve_to_one_group(name):
    e = _engine(name)
    a = e.add_mine(40.0, 100.0)
    far = e.add_mine(40.0, 200.0)
    b = e.add_mine(42.0, 100.0)
    assert e.resolve(a) == e.resolve(b) == b
    assert e.resolve(far) == far and far != b
    assert set(e.obstacle_shapes()) == {b, far}


def test_corners_are_not_contained(name):
    e = _engine(name)
    for x, y in _mines(30, 7):
        e.add_mine(x, y)
    e.add_square(3, 3)
    pts = e.corners()
    assert pts, "no corners"
    assert all(not e.contains(x, y) for x, y in pts)
    assert all(isinstance(x, float) and isinstance(y, float) for x, y in pts)


def test_adding_an_obstacle_never_shortens_a_path(name):
    rng = random.Random(11)
    for seed in range(3):
        e = _engine(name)
        pairs = [((rng.uniform(1, W - 1), rng.uniform(1, 40)),
                  (rng.uniform(1, W - 1), rng.uniform(H - 40, H - 1))) for _ in range(6)]
        before = [_length(e.shortest_path(p, q)) for p, q in pairs]
        for x, y in _mines(25, seed):
            e.add_mine(x, y)
            after = [_length(e.shortest_path(p, q)) for p, q in pairs]
            assert all(a >= b - 1e-6 for a, b in zip(after, before)), (seed, before, after)
            before = after


def test_same_adds_in_the_same_order_give_identical_answers(name):
    a, b = _engine(name), _engine(name)
    mines = _mines(30, 9)
    for e in (a, b):
        for x, y in mines:
            e.add_mine(x, y)
        e.add_square(5, 90)
    rng = random.Random(4)
    for _ in range(30):
        p = (rng.uniform(1, W - 1), rng.uniform(1, H - 1))
        q = (rng.uniform(1, W - 1), rng.uniform(1, H - 1))
        assert a.shortest_path(p, q) == b.shortest_path(p, q)
    assert a.corners() == b.corners()


def test_last_added_meets_a_segment_through_the_new_mine(name):
    e = _engine(name)
    assert e.last_added is None
    e.add_mine(40.0, 100.0)
    assert shapely.LineString([(30.0, 100.0), (50.0, 100.0)]).intersects(e.last_added)
    e.add_mine(20.0, 200.0)
    assert shapely.LineString([(10.0, 200.0), (30.0, 200.0)]).intersects(e.last_added)
    assert not shapely.LineString([(30.0, 100.0), (50.0, 100.0)]).intersects(e.last_added)


def test_a_planner_built_with_the_field_engine_plans_a_pass():
    pl = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1,), engine="field")
    assert type(pl.engine) is ENGINES["field"]
    plan = pl.plan({1: pl.point_a()}, me=1)
    assert plan[1], "no photos planned"


def _run():
    tests = [(k, v) for k, v in dict(globals()).items()
             if k.startswith("test_") and k != "test_a_planner_built_with_the_field_engine_plans_a_pass"]
    passed, gaps, failed = 0, 0, 0
    for name in ENGINES:
        for tname, t in tests:
            try:
                t(name)
                passed += 1
                print(f"PASS {name:8s} {tname}", flush=True)
            except Exception as err:       # noqa: BLE001 -- report every engine's every check
                if (tname, name) in KNOWN_GAPS:
                    gaps += 1
                    print(f"KNOWN GAP {name:8s} {tname}: {KNOWN_GAPS[(tname, name)]}"
                          f" ({err!r})", flush=True)
                else:
                    failed += 1
                    print(f"FAIL {name:8s} {tname}: {err!r}", flush=True)
    test_a_planner_built_with_the_field_engine_plans_a_pass()
    passed += 1
    print("PASS planner smoke (engine=field)")
    print(f"\n{passed} passed, {gaps} known gaps, {failed} failed "
          f"({len(ENGINES)} engines x {len(tests)} checks + smoke)")
    return failed


if __name__ == "__main__":
    raise SystemExit(1 if _run() else 0)
