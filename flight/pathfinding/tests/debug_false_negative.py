import math
import random
from shapely.geometry import LineString

from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.tests.connectPolygonStressTest import (
    make_layout,
    ground_truth_edges,
    actual_cross_obstacle_edges,
)

random.seed(1234)
rng = random.Random(1234)

found = None
for trial in range(40):
    n = rng.randint(3, 10)
    polys = make_layout(n, rng, spread=max(15.0, 4.0 * math.sqrt(n)))
    if polys is None:
        continue

    arbCorners = [[0, 200], [200, 200], [0, 0], [200, 0]]
    sim_field_size = [200, 200]
    field = Field(sim_field_size, arbCorners)

    add_order = list(range(n))
    rng.shuffle(add_order)
    for idx in add_order:
        field.createPolygonObstacle(polys[idx])

    truth = ground_truth_edges(field.polygonObstacles)
    actual = actual_cross_obstacle_edges(field)
    missing = truth - actual
    if missing:
        found = (trial, n, field, missing)
        break

if found is None:
    print("no false negative found in range")
else:
    trial, n, field, missing = found
    print(f"trial={trial} n={n} obstacles, {len(missing)} missing edges")

    # map id() back to actual Node objects for inspection
    id_to_node = {}
    id_to_obstacle = {}
    for obstacle in field.polygonObstacles:
        for node in obstacle.nodes:
            id_to_node[id(node)] = node
            id_to_obstacle[id(node)] = obstacle

    print("missing frozensets (first 5):", [tuple(m) for m in list(missing)[:5]])
    edge_ids = [m for m in missing if len(m) == 2][0]
    a_id, b_id = tuple(edge_ids)
    nodeA, nodeB = id_to_node[a_id], id_to_node[b_id]
    obA, obB = id_to_obstacle[a_id], id_to_obstacle[b_id]
    print(
        f"missing edge: nodeA=({nodeA.x:.3f},{nodeA.y:.3f}) on obstacle {id(obA)}  "
        f"nodeB=({nodeB.x:.3f},{nodeB.y:.3f}) on obstacle {id(obB)}"
    )

    # confirm it's actually a real commonTangents candidate and genuinely unblocked
    pairs = obA.commonTangents(obB)
    match = [(p[0], p[1]) for p in pairs if p[0] is nodeA and p[1] is nodeB]
    print(f"is a real commonTangents candidate from obA->obB: {bool(match)}")

    seg = LineString([(nodeA.x, nodeA.y), (nodeB.x, nodeB.y)])
    blockers = [
        o
        for o in field.polygonObstacles
        if o is not obA and o is not obB and (o.polygon.crosses(seg) or o.polygon.contains(seg))
    ]
    print(f"third obstacles actually blocking this segment (should be empty): {len(blockers)}")

    # inspect nodeA's occlusion arcs to see which one incorrectly fired
    print(f"nodeA has {len(nodeA.occlusionArcs)} recorded occlusion arcs")
    theta = math.atan2(nodeB.y - nodeA.y, nodeB.x - nodeA.x)
    for i, (lo, hi, pivot, blockerOb, near, far) in enumerate(nodeA.occlusionArcs):
        t = ((theta - pivot + math.pi) % (2 * math.pi)) - math.pi
        inrange = lo - 1e-9 <= t <= hi + 1e-9
        print(
            f"  arc[{i}]: blocker={id(blockerOb)} lo={lo:.3f} hi={hi:.3f} pivot={pivot:.3f} "
            f"near={near:.3f} far={far:.3f}  theta_wrapped={t:.3f}  CONTAINS_THETA={inrange}"
        )
        if inrange:
            realSeg = (nodeA.x, nodeA.y), (nodeB.x, nodeB.y)
            print(
                f"    -> blockerOb.intersects(seg) = {blockerOb.intersects(realSeg)}  (this determined the block)"
            )
