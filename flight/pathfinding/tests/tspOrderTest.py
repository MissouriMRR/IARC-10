"""
Verification for the module-level path-utility functions in
flight/pathfinder.py: order_waypoints (nearest-neighbor construction + 2-opt
local search, open path), path_length (total polyline length over (x,y)
tuples), and node_path_length (the same, over a list of .x/.y-exposing
objects such as nodeField Node instances).

flight/pathfinder.py transitively imports flight/pathfinding/path_subdivision.py
and flight/pathfinding/utils/seen_by_drone.py, both of which import names
from flight.pathfinding.nodeField's __init__.py that it doesn't actually
export (a pre-existing break, already flagged out of scope earlier this
session -- unrelated to these functions, which only use `math`). Stubbing
those two modules in sys.modules lets us import and test in isolation
without either fixing or being blocked by that unrelated breakage.
"""

import sys
import math
import random
import itertools
import types

for _name in ("flight.pathfinding.path_subdivision", "flight.pathfinding.utils.seen_by_drone"):
    stub = types.ModuleType(_name)
    stub.Path = object
    stub.SightTracker = object
    stub.remove_extra_coords = lambda *a, **k: None
    sys.modules[_name] = stub

from flight.pathfinder import (
    order_waypoints,
    path_length,
    node_path_length,
    _tour_length,
    _nearest_neighbor_tour,
)


class _FakeNode:
    """Minimal .x/.y stand-in for a real nodeField Node, so node_path_length
    can be tested without constructing the (much heavier) real thing."""

    def __init__(self, x, y):
        self.x = x
        self.y = y


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def brute_force_optimal_length(coords):
    n = len(coords)
    best = None
    for perm in itertools.permutations(range(1, n)):
        order = (0,) + perm
        length = sum(dist(coords[order[i]], coords[order[i + 1]]) for i in range(n - 1))
        if best is None or length < best:
            best = length
    return best


def test_returns_permutation_of_input():
    rng = random.Random(1)
    coords = [(rng.uniform(0, 100), rng.uniform(0, 100)) for _ in range(30)]
    result = order_waypoints(coords)

    same_multiset = sorted(result) == sorted(coords)
    same_length = len(result) == len(coords)

    ok = same_multiset and same_length
    print(f"test_returns_permutation_of_input: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_trivial_sizes_unchanged():
    ok = order_waypoints([]) == [] and order_waypoints([(1.0, 2.0)]) == [(1.0, 2.0)]
    two = [(0.0, 0.0), (5.0, 5.0)]
    ok = ok and order_waypoints(two) == two
    print(f"test_trivial_sizes_unchanged: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_beats_input_order_and_single_start_nearest_neighbor():
    # scattered points where input order is a poor tour and a single fixed
    # start (index 0) for nearest-neighbor is a known weak baseline --
    # order_waypoints (multi-start NN + 2-opt) should meaningfully beat both
    rng = random.Random(7)
    coords = [(rng.uniform(0, 200), rng.uniform(0, 200)) for _ in range(40)]

    input_order_length = path_length(coords)
    single_start_tour = _nearest_neighbor_tour(coords, 0)
    single_start_length = _tour_length(coords, single_start_tour)
    result = order_waypoints(coords)
    result_length = path_length(result)

    beats_input_order = result_length < input_order_length
    beats_single_start_nn = result_length <= single_start_length + 1e-6

    ok = beats_input_order and beats_single_start_nn
    print(
        f"test_beats_input_order_and_single_start_nearest_neighbor: "
        f"input_order={input_order_length:.1f} single_start_nn={single_start_length:.1f} "
        f"result={result_length:.1f} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_close_to_brute_force_optimal_for_small_n():
    # small enough (n<=8) that true optimal is tractable via brute force --
    # confirms the heuristic isn't just "a permutation", it's a genuinely
    # good one. 2-opt from a decent start is often exactly optimal at this
    # scale; allow some slack rather than requiring exact equality.
    all_ok = True
    for seed in range(5):
        rng = random.Random(seed)
        n = rng.randint(5, 8)
        coords = [(rng.uniform(0, 50), rng.uniform(0, 50)) for _ in range(n)]

        optimal = brute_force_optimal_length(coords)
        result = order_waypoints(coords)
        result_length = path_length(result)

        within_slack = result_length <= optimal * 1.15 + 1e-6
        if not within_slack:
            all_ok = False
            print(
                f"  seed={seed} n={n} optimal={optimal:.2f} result={result_length:.2f} "
                f"ratio={result_length/optimal:.3f}"
            )

    print(f"test_close_to_brute_force_optimal_for_small_n: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_deterministic():
    rng = random.Random(3)
    coords = [(rng.uniform(0, 100), rng.uniform(0, 100)) for _ in range(15)]
    a = order_waypoints(coords)
    b = order_waypoints(coords)
    ok = a == b
    print(f"test_deterministic: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_duplicate_and_collinear_points_no_crash():
    coords = [(0.0, 0.0), (0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (5.0, 0.0)]
    try:
        result = order_waypoints(coords)
        ok = sorted(result) == sorted(coords)
    except Exception as e:
        ok = False
        print(f"  raised: {e!r}")
    print(f"test_duplicate_and_collinear_points_no_crash: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_path_length_known_cases():
    all_ok = True

    if path_length([]) != 0.0:
        all_ok = False
    if path_length([(1.0, 2.0)]) != 0.0:
        all_ok = False

    # single segment: exact 3-4-5 triangle
    l = path_length([(0.0, 0.0), (3.0, 4.0)])
    if abs(l - 5.0) > 1e-9:
        all_ok = False

    # multi-segment: two right-angle legs, exactly summable
    l = path_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)])
    if abs(l - 7.0) > 1e-9:
        all_ok = False

    # a path back to where it started still counts every leg (not a
    # shortcut / not net displacement)
    l = path_length([(0.0, 0.0), (10.0, 0.0), (0.0, 0.0)])
    if abs(l - 20.0) > 1e-9:
        all_ok = False

    print(f"test_path_length_known_cases: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_path_length_matches_naive_sum():
    rng = random.Random(11)
    coords = [(rng.uniform(-50, 50), rng.uniform(-50, 50)) for _ in range(25)]
    expected = sum(dist(coords[i], coords[i + 1]) for i in range(len(coords) - 1))
    ok = abs(path_length(coords) - expected) < 1e-9
    print(f"test_path_length_matches_naive_sum: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_path_length_order_dependent():
    # reordering the same points changes the length -- path_length measures
    # the given ORDER, it isn't a permutation-invariant property
    coords = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    forward = path_length(coords)
    shuffled = path_length([coords[0], coords[2], coords[1], coords[3]])
    ok = abs(forward - shuffled) > 1e-6
    print(
        f"test_path_length_order_dependent: forward={forward:.2f} shuffled={shuffled:.2f} "
        f"-> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_ordered_path_is_never_longer_than_input_order():
    # order_waypoints should never make total length WORSE than just
    # visiting points in whatever order they were given
    rng = random.Random(21)
    for _ in range(5):
        n = rng.randint(3, 25)
        coords = [(rng.uniform(0, 100), rng.uniform(0, 100)) for _ in range(n)]
        if path_length(order_waypoints(coords)) > path_length(coords) + 1e-6:
            print(
                f"  n={n} ordered={path_length(order_waypoints(coords)):.2f} "
                f"input={path_length(coords):.2f}"
            )
            print("test_ordered_path_is_never_longer_than_input_order: -> FAIL")
            return False
    print("test_ordered_path_is_never_longer_than_input_order: -> PASS")
    return True


def test_node_path_length_matches_path_length():
    rng = random.Random(31)
    coords = [(rng.uniform(-20, 20), rng.uniform(-20, 20)) for _ in range(12)]
    nodes = [_FakeNode(x, y) for x, y in coords]

    ok = abs(node_path_length(nodes) - path_length(coords)) < 1e-9
    print(f"test_node_path_length_matches_path_length: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_node_path_length_known_case():
    nodes = [_FakeNode(0.0, 0.0), _FakeNode(3.0, 0.0), _FakeNode(3.0, 4.0)]
    ok = abs(node_path_length(nodes) - 7.0) < 1e-9
    print(f"test_node_path_length_known_case: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_node_path_length_trivial_sizes():
    ok = node_path_length([]) == 0.0 and node_path_length([_FakeNode(1.0, 1.0)]) == 0.0
    print(f"test_node_path_length_trivial_sizes: -> {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    results = [
        test_returns_permutation_of_input(),
        test_trivial_sizes_unchanged(),
        test_beats_input_order_and_single_start_nearest_neighbor(),
        test_close_to_brute_force_optimal_for_small_n(),
        test_deterministic(),
        test_duplicate_and_collinear_points_no_crash(),
        test_path_length_known_cases(),
        test_path_length_matches_naive_sum(),
        test_path_length_order_dependent(),
        test_ordered_path_is_never_longer_than_input_order(),
        test_node_path_length_matches_path_length(),
        test_node_path_length_known_case(),
        test_node_path_length_trivial_sizes(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
