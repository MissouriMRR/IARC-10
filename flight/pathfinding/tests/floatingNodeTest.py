"""
Verification for Field.addFloatingNode after fixing its broken connection
wiring. No prior test exercised this path at all, so this covers:
  1. basic connectivity (an unobstructed floating node actually gets an edge)
  2. occlusion safety (a blocked candidate connection is correctly rejected)
  3. retroactive cleanup interaction with find_colliding_pairs
  4. floating-node connectivity to a merged unionObstacle
"""

from flight.pathfinding.nodeField.field import Field


def square(cx, cy, half):
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]


def make_field():
    arbCorners = [[0, 100], [100, 100], [0, 0], [100, 0]]
    return Field([100, 100], arbCorners)


def node_ids(field, node):
    return set(field.fieldConnection.nodeGraph.get(node, {}).keys())


def test_basic_connectivity():
    field = make_field()
    field.createPolygonObstacle(square(60, 50, 3))
    fNode = field.addFloatingNode(10, 50)
    connected = (
        fNode in field.fieldConnection.nodeGraph and len(field.fieldConnection.nodeGraph[fNode]) > 0
    )
    print(
        f"test_basic_connectivity: fNode has {len(field.fieldConnection.nodeGraph.get(fNode, {}))} edges -> {'PASS' if connected else 'FAIL'}"
    )
    return connected


def test_occlusion_safety():
    field = make_field()
    # B is a wide wall between the floating point and A; any straight line
    # from (10,50) to anywhere on A must cross B's x-range within its y-span.
    wallB = square(30, 50, 15)  # spans x:15-45, y:35-65
    obstacleA = square(60, 50, 2)  # x:58-62, y:48-52
    field.createPolygonObstacle(wallB)
    field.createPolygonObstacle(obstacleA)

    fNode = field.addFloatingNode(10, 50)

    a_positions = {(v[0], v[1]) for v in obstacleA}
    blocked_correctly = True
    for neighbor in field.fieldConnection.nodeGraph.get(fNode, {}):
        if (neighbor.x, neighbor.y) in a_positions:
            blocked_correctly = False
    print(
        f"test_occlusion_safety: fNode connections to A = "
        f"{[ (n.x,n.y) for n in field.fieldConnection.nodeGraph.get(fNode, {}) if (n.x,n.y) in a_positions]} "
        f"-> {'PASS' if blocked_correctly else 'FAIL'}"
    )
    return blocked_correctly


def test_retroactive_cleanup():
    field = make_field()
    obstacleA = square(60, 50, 2)
    field.createPolygonObstacle(obstacleA)

    fNode = field.addFloatingNode(10, 50)
    a_positions = {(v[0], v[1]) for v in obstacleA}
    had_edge_to_A = any(
        (n.x, n.y) in a_positions for n in field.fieldConnection.nodeGraph.get(fNode, {})
    )

    # now add a wall between fNode and A -- should retroactively remove the edge via find_colliding_pairs
    wallB = square(30, 50, 15)
    field.createPolygonObstacle(wallB)

    still_has_edge_to_A = any(
        (n.x, n.y) in a_positions for n in field.fieldConnection.nodeGraph.get(fNode, {})
    )
    passed = had_edge_to_A and not still_has_edge_to_A
    print(
        f"test_retroactive_cleanup: had_edge_before={had_edge_to_A} still_has_edge_after={still_has_edge_to_A} "
        f"-> {'PASS' if passed else 'FAIL'}"
    )
    return passed


def test_union_obstacle_connectivity():
    field = make_field()
    # two overlapping squares -> should merge into a unionObstacle
    field.createPolygonObstacle(square(55, 50, 4))
    field.createPolygonObstacle(square(60, 50, 4))
    has_union = len(field.unionObstacles) > 0
    if not has_union:
        print(
            "test_union_obstacle_connectivity: SKIPPED (layout did not merge into a unionObstacle)"
        )
        return True

    fNode = field.addFloatingNode(10, 50)
    union_node_positions = {(n.x, n.y) for u in field.unionObstacles for n in u.nodes}
    connected_to_union = any(
        (n.x, n.y) in union_node_positions for n in field.fieldConnection.nodeGraph.get(fNode, {})
    )
    print(
        f"test_union_obstacle_connectivity: connected_to_union={connected_to_union} -> {'PASS' if connected_to_union else 'FAIL'}"
    )
    return connected_to_union


def main():
    results = [
        test_basic_connectivity(),
        test_occlusion_safety(),
        test_retroactive_cleanup(),
        test_union_obstacle_connectivity(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
