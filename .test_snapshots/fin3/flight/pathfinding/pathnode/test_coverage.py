"""Tests for photo coverage and miss handling (coverage.py). Plain asserts -- runs under pytest
or as a script:  PYTHONPATH=<repo> python -m flight.pathfinding.pathnode.test_coverage
"""
import math

from flight.pathfinding.pathnode.config import MISS_BLOCK
from flight.pathfinding.pathnode.coverage import cell_centre
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.session import PathNodeBackend
from flight.pathfinding.pathnode.sim import Mission, evaluate


def _tiny_photo(x, y, half=0.3):
    """A footprint around (x, y) too small to cover any square."""
    return [(x - half, y - half), (x + half, y - half), (x + half, y + half), (x - half, y + half)]


def test_miss_goes_to_the_squares_a_photo_was_planned_for():
    """A path photo can be planned for a square farther than aim_radius from its point; when
    the photo comes out short it must still count a miss for that square (it used to count
    none, so the same point was handed out forever)."""
    cov = PathNodePlanner(80, 300, 40).coverage
    poly = [(18.061854646744074, 86.26903632435094), (33.88833178534092, 103.92110709653359)]
    far = None
    for p in cov.places_along(poly):
        for t in cov.need(poly):
            if math.dist(cell_centre(*t), p) > cov.aim_radius() + 1e-9 and t in cov._aims_for(p):
                far = (p, t)
    assert far, "scenario no longer has a planned square beyond aim_radius"
    (px, py), target = far
    cx, cy = cov.record(_tiny_photo(px, py))
    assert cov.count_misses(cx, cy, {target}) == [] and cov.misses[target] == 1
    for _ in range(MISS_BLOCK - 1):
        hopeless = cov.count_misses(cx, cy, {target})
    assert hopeless == [target]


def test_photo_nowhere_near_its_plan_counts_nothing():
    cov = PathNodePlanner(80, 300, 40).coverage
    cov.places_along([(20.0, 20.0), (20.0, 60.0)])
    assert cov.count_misses(60.0, 200.0, {(30, 100)}) == [] and not cov.misses


def test_rotated_footprints_finish_with_keep_away():
    """Rotated photos with the drones kept apart hit the tick cap (a photo just outside
    aim_radius of its square repeated forever)."""
    for yaw in (45.0, "random"):
        m = Mission(generate_true_minefield(30, 0), drones=2, photo_yaw_deg=yaw, max_ticks=1200,
                    planner_options=dict(keep_away_ft=9.0, keep_clear_ft=7.0)).run()
        e = evaluate(m)
        assert m.failure is None and e["complete"] and e["unseen_cells"] == 0, (yaw, m.failure, e)


def test_climb_retries_are_one_attempt():
    """Scan records a footprint per climb retry, then own_photo once. Eight failed retries
    of one waypoint are ONE miss, so the square is blocked only after MISS_BLOCK waypoints."""
    pl = PathNodePlanner(80, 300, 41.0)
    session = PathNodeBackend.from_planner(pl)
    x, y, _g = pl.plan({1: pl.point_a()})[1][0]
    square = pl.seen.real_to_cell(x, y)
    for waypoint in range(1, MISS_BLOCK + 1):
        for _retry in range(8):
            session.own_footprint(_tiny_photo(x, y))
        assert square not in pl.blocked, ("blocked by retries", waypoint)
        session.own_photo(None, [])
        assert pl.coverage.misses[square] == waypoint
        assert (square in pl.blocked) == (waypoint == MISS_BLOCK)


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    failed = 0
    for n, f in tests:
        try:
            f()
            print("ok  ", n)
        except AssertionError as ex:
            failed += 1
            print("FAIL", n, ex)
    raise SystemExit(1 if failed else 0)
