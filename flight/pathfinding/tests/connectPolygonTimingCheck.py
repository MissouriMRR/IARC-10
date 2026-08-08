"""
Timing sanity check: OLD connectPolygon algorithm (unconditional per-obstacle
loop + connectNode -> validPath's live O(n) obstacle scan) vs the NEW
group/arc-based Field.connectPolygon, on the SAME real classes and the same
obstacle layouts. The old algorithm is reconstructed as a standalone function
using find_colliding_pairs/connectPolygonObstacle/connectNode exactly as they
existed before this change -- none of those were modified except
connectPolygonObstacle's redundant findClosestNodes gate (a pure dead-code
removal, not a behavior change), and validPath itself was never touched.
"""
import math
import random
import time

from flight.pathfinding.nodeField.field import Field, find_colliding_pairs
from flight.pathfinding.tests.connectPolygonStressTest import make_layout


def old_connect_polygon(field, polygonObstacle):
    connectionsToRemove = find_colliding_pairs(
        list(field.fieldConnection.nodeGraph.keys()), list(polygonObstacle.vertices[:-1])
    )
    for other in field.polygonObstacles + field.mines + field.unionObstacles:
        if other is not polygonObstacle:
            nodesToConnect = polygonObstacle.connectPolygonObstacle(other)
            for j in nodesToConnect:
                if j == []:
                    continue
                field.fieldConnection.connectNode(j[0], j[1])
    for i in connectionsToRemove:
        field.fieldConnection.deleteConnection(i[0], i[1])


def build_field_and_add(n_obstacles, polys, add_order, connect_fn):
    arbCorners = [[0, 200], [200, 200], [0, 0], [200, 0]]
    field = Field([200, 200], arbCorners)

    def patched_create(vertices):
        # mirrors createPolygonObstacle up to the perimeter/hull connections,
        # then calls connect_fn instead of the new field.connectPolygon
        if len(vertices) == 2:
            return
        from flight.pathfinding.nodeField.node import Node
        from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle
        nodeList = [Node(v[0], v[1], False) for v in vertices[:-1]]
        wrapping = True
        newObstacle = PolygonObstacle(vertices, nodeList, wrapping)
        overlaps = field.checkForObstacleCollisions(newObstacle)
        if overlaps:
            field.removeObstaclesFromList(overlaps)
            newObstacle, wrapping = field.mergePolygons(overlaps + [newObstacle])
            field.unionObstacles.append(newObstacle)
        else:
            field.polygonObstacles.append(newObstacle)

        for i in range(len(newObstacle.nodes) - 1):
            field.fieldConnection.connectNode(newObstacle.nodes[i], newObstacle.nodes[i + 1])
        if wrapping:
            field.fieldConnection.connectNode(newObstacle.nodes[-1], newObstacle.nodes[0])

        connect_fn(field, newObstacle)

    t0 = time.perf_counter()
    for idx in add_order:
        patched_create(polys[idx])
    t1 = time.perf_counter()
    return field, t1 - t0


def main():
    rng = random.Random(99)
    for n in [10, 20, 30, 40]:
        polys = None
        for seed in range(50):
            rng2 = random.Random(seed + 1000)
            polys = make_layout(n, rng2, spread=max(15.0, 4.0 * math.sqrt(n)))
            if polys is not None:
                break
        if polys is None:
            print(f"n={n}: could not generate a valid layout")
            continue

        add_order = list(range(n))
        rng2.shuffle(add_order)

        _, old_time = build_field_and_add(n, polys, add_order, old_connect_polygon)
        _, new_time = build_field_and_add(n, polys, add_order, lambda f, o: f.connectPolygon(o))

        speedup = old_time / new_time if new_time > 0 else float("inf")
        print(f"n={n:3d}  OLD={old_time*1000:8.1f}ms  NEW={new_time*1000:8.1f}ms  speedup={speedup:.2f}x")


if __name__ == "__main__":
    main()
