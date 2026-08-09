"""
Correctness/safety stress test for the group/arc-based obstacle-connection
algorithm in Field.connectPolygon. Builds randomized non-overlapping obstacle
layouts, adds them to a real Field one at a time (real Field/PolygonObstacle/
Node classes, not a scratchpad re-implementation), then compares the
resulting nodeGraph against a brute-force ground truth: for every obstacle
pair's commonTangents candidates, does the segment cross any THIRD obstacle?

Checks for:
  - unsafe edges: accepted in nodeGraph but actually crossed by a third obstacle
  - false negatives: ground truth says valid but missing from nodeGraph
"""

import math
import random
from shapely.geometry import Polygon, LineString
from shapely import MultiPoint, convex_hull

from flight.pathfinding.nodeField.field import Field


def make_poly(cx, cy, n, rmin, rmax, rng):
    angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(n))
    for i in range(1, len(angs)):
        if angs[i] - angs[i - 1] < 1e-3:
            angs[i] += 1e-3
    pts = [
        (cx + rng.uniform(rmin, rmax) * math.cos(a), cy + rng.uniform(rmin, rmax) * math.sin(a))
        for a in angs
    ]
    return list(convex_hull(MultiPoint(pts)).exterior.coords)


def valid_layout(polys, min_gap=1e-6):
    shp = [Polygon(p) for p in polys]
    for s in shp:
        if not s.is_valid:
            return False
    for i in range(len(shp)):
        for j in range(i + 1, len(shp)):
            if shp[i].exterior.crosses(shp[j].exterior) or shp[i].exterior.overlaps(
                shp[j].exterior
            ):
                return False
            if shp[i].distance(shp[j]) < min_gap:
                return False
    return True


def make_layout(n_obstacles, rng, spread=15.0):
    for _ in range(200):
        positions = [
            (rng.uniform(-spread, spread) + 50, rng.uniform(-spread, spread) + 50)
            for _ in range(n_obstacles)
        ]
        polys = []
        for px, py in positions:
            n = rng.randint(3, 7)
            if rng.random() < 0.4:
                rmin, rmax = 0.5, rng.uniform(2, 4)
            else:
                rmin, rmax = 0.8, 1.6
            polys.append(make_poly(px, py, n, rmin, rmax, rng))
        if valid_layout(polys):
            return polys
    return None


def ground_truth_edges(polygonObstacles):
    """For every pair of real PolygonObstacle instances, which commonTangents
    node pairs are NOT crossed by any third obstacle."""
    truth = set()
    for i, A in enumerate(polygonObstacles):
        for j, C in enumerate(polygonObstacles):
            if i >= j:
                continue
            for selfNode, otherNode in A.commonTangents(C):
                seg = LineString([(selfNode.x, selfNode.y), (otherNode.x, otherNode.y)])
                blocked = False
                for k, D in enumerate(polygonObstacles):
                    if k == i or k == j:
                        continue
                    if D.polygon.crosses(seg) or D.polygon.contains(seg):
                        blocked = True
                        break
                if not blocked:
                    truth.add(frozenset([id(selfNode), id(otherNode)]))
    return truth


def actual_cross_obstacle_edges(field):
    """Only edges BETWEEN two different obstacles -- excludes each obstacle's
    own perimeter/self edges, which ground_truth_edges never considers since
    it only evaluates inter-obstacle commonTangents candidates."""
    owner = {}
    for obstacle in field.polygonObstacles:
        for node in obstacle.nodes:
            owner[id(node)] = id(obstacle)

    edges = set()
    for node1, neighbors in field.fieldConnection.nodeGraph.items():
        o1 = owner.get(id(node1))
        for node2 in neighbors.keys():
            o2 = owner.get(id(node2))
            if o1 is not None and o2 is not None and o1 != o2:
                edges.add(frozenset([id(node1), id(node2)]))
    return edges


def run_trial(n_obstacles, rng):
    polys = make_layout(n_obstacles, rng, spread=max(15.0, 4.0 * math.sqrt(n_obstacles)))
    if polys is None:
        return None

    arbCorners = [[0, 200], [200, 200], [0, 0], [200, 0]]
    sim_field_size = [200, 200]
    field = Field(sim_field_size, arbCorners)

    add_order = list(range(n_obstacles))
    rng.shuffle(add_order)
    for idx in add_order:
        field.createPolygonObstacle(polys[idx])

    truth = ground_truth_edges(field.polygonObstacles)
    actual = actual_cross_obstacle_edges(field)

    unsafe = actual - truth
    missing = truth - actual
    return len(unsafe), len(missing), len(truth)


def run_sparse_trial(n_obstacles, rng):
    """Widely-spaced obstacles -- per this session's finding, this produces a
    DENSER connection graph (less mutual occlusion), not a sparser one, so
    it's a distinct case from run_trial's clustered layouts, not a smaller
    version of the same thing."""
    spread = 60.0
    polys = make_layout(n_obstacles, rng, spread=spread)
    if polys is None:
        return None

    # make_layout centers obstacles at (-spread,spread)+50 in both axes, so the
    # field must actually cover that range (plus obstacle radius) -- it must
    # NOT start at (0,0), or a real fraction of vertices land out of bounds.
    margin = spread + 20.0
    lo, hi = 50.0 - margin, 50.0 + margin
    arbCorners = [[lo, hi], [hi, hi], [lo, lo], [hi, lo]]
    field = Field([hi - lo, hi - lo], arbCorners)

    add_order = list(range(n_obstacles))
    rng.shuffle(add_order)
    for idx in add_order:
        field.createPolygonObstacle(polys[idx])

    truth = ground_truth_edges(field.polygonObstacles)
    actual = actual_cross_obstacle_edges(field)

    unsafe = actual - truth
    missing = truth - actual
    return len(unsafe), len(missing), len(truth)


def main():
    random.seed(1234)
    rng = random.Random(1234)
    trials = 300
    total_unsafe = 0
    total_missing = 0
    total_truth = 0
    valid_trials = 0
    for trial in range(trials):
        n = rng.randint(3, 10)
        result = run_trial(n, rng)
        if result is None:
            continue
        valid_trials += 1
        unsafe, missing, truth_count = result
        total_unsafe += unsafe
        total_missing += missing
        total_truth += truth_count
        if unsafe:
            print(f"UNSAFE at trial {trial}: n={n} unsafe_edges={unsafe}")

    print()
    print(f"Valid trials: {valid_trials}")
    print(f"Total unsafe edges: {total_unsafe}")
    print(f"Total missing (false-negative) edges: {total_missing} / {total_truth} true edges")

    print()
    print("Sparse (widely-spaced) layouts:")
    sparse_unsafe = sparse_missing = sparse_truth = sparse_valid = 0
    for trial in range(20):
        n = rng.randint(4, 7)
        result = run_sparse_trial(n, rng)
        if result is None:
            continue
        sparse_valid += 1
        unsafe, missing, truth_count = result
        sparse_unsafe += unsafe
        sparse_missing += missing
        sparse_truth += truth_count
        if unsafe:
            print(f"SPARSE UNSAFE at trial {trial}: n={n} unsafe_edges={unsafe}")

    print(f"Sparse valid trials: {sparse_valid}")
    print(f"Sparse total unsafe edges: {sparse_unsafe}")
    print(
        f"Sparse total missing (false-negative) edges: {sparse_missing} / {sparse_truth} true edges"
    )


if __name__ == "__main__":
    main()
