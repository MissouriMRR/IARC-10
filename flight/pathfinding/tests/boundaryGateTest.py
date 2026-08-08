"""
Verification for the new connection-time bounds gate (FieldConnections.addGraph)
that replaced vertex-deletion boundary handling (removed Field.wrapInBoundPolygon).

Confirms: an obstacle with a vertex outside the field boundary keeps ALL its
vertices/nodes (nothing deleted), edges between two in-bounds vertices still
get added, and any edge touching the out-of-bounds vertex is silently skipped
rather than replaced with a fabricated "corner hugging" shortcut edge.
"""

from flight.pathfinding.nodeField.field import Field


def make_field():
    arbCorners = [[0, 100], [100, 100], [0, 0], [100, 0]]
    return Field([100, 100], arbCorners)


def test_out_of_bounds_vertex_kept_but_edges_gated():
    field = make_field()
    # v1 is outside the field (x=105 > xMax=100); v0 and v2 are in bounds.
    v0 = (80, 40)
    v1 = (105, 50)
    v2 = (80, 60)
    field.createPolygonObstacle([v0, v1, v2, v0])  # closed ring, last==first per convention

    obstacle = field.polygonObstacles[0]

    # (a) nothing deleted -- all 3 vertices/nodes still present
    has_all_vertices = len(obstacle.vertices) == 4  # includes the closing duplicate, as passed in
    has_all_nodes = len(obstacle.nodes) == 3
    print(f"kept all vertices: {has_all_vertices}  kept all nodes: {has_all_nodes}")

    node_positions = [(n.x, n.y) for n in obstacle.nodes]
    v1_node = next(n for n in obstacle.nodes if (n.x, n.y) == v1)
    v0_node = next(n for n in obstacle.nodes if (n.x, n.y) == v0)
    v2_node = next(n for n in obstacle.nodes if (n.x, n.y) == v2)

    # (b) the in-bounds-to-in-bounds closing edge (v2 -> v0) should exist
    v2_v0_connected = (
        v2_node in field.fieldConnection.nodeGraph
        and v0_node in field.fieldConnection.nodeGraph.get(v2_node, {})
    )

    # (c) any edge touching v1 (out of bounds) should be absent
    v1_has_no_edges = (
        v1_node not in field.fieldConnection.nodeGraph
        or len(field.fieldConnection.nodeGraph[v1_node]) == 0
    )

    # (d) v1 is still reachable via the obstacle's own node list (not deleted)
    v1_still_in_obstacle = v1_node in obstacle.nodes

    passed = (
        has_all_vertices
        and has_all_nodes
        and v2_v0_connected
        and v1_has_no_edges
        and v1_still_in_obstacle
    )
    print(
        f"v2-v0 edge exists: {v2_v0_connected}  v1 has no graph edges: {v1_has_no_edges}  "
        f"v1 still in obstacle.nodes: {v1_still_in_obstacle}"
    )
    print(f"test_out_of_bounds_vertex_kept_but_edges_gated: -> {'PASS' if passed else 'FAIL'}")
    return passed


def test_fully_in_bounds_obstacle_unaffected():
    field = make_field()
    field.createPolygonObstacle([(20, 20), (30, 20), (30, 30), (20, 30), (20, 20)])
    obstacle = field.polygonObstacles[0]
    all_nodes_connected = all(
        node in field.fieldConnection.nodeGraph and len(field.fieldConnection.nodeGraph[node]) > 0
        for node in obstacle.nodes
    )
    print(
        f"test_fully_in_bounds_obstacle_unaffected: all_nodes_connected={all_nodes_connected} "
        f"-> {'PASS' if all_nodes_connected else 'FAIL'}"
    )
    return all_nodes_connected


def test_isolated_in_bounds_vertex_in_convex_hull_merge():
    """A merged (union) obstacle whose in-bounds vertices are each isolated
    (flanked by out-of-bounds neighbors on both sides) used to raise KeyError
    in addObstacle's convex-hull connection loop, since that loop indexed
    self.fieldConnection.nodeGraph[currentNode] directly -- an assumption
    that broke once addGraph started silently skipping out-of-bounds edges
    (a node whose own perimeter neighbors are both out of bounds never gets
    a nodeGraph entry from the perimeter loop at all)."""
    field = make_field()
    E = [
        (30, 45),
        (-10, 40),
        (-10, -50),
        (-50, -50),
        (-50, 150),
        (-10, 150),
        (-10, 70),
        (30, 65),
        (-10, 60),
        (-10, 50),
        (30, 45),
    ]
    field.createPolygonObstacle(E)
    D = [(-51, -51), (-49, -51), (-49, 149), (-51, 149), (-51, -51)]
    try:
        field.createPolygonObstacle(D)
        merged = len(field.unionObstacles) > 0
        print(
            f"test_isolated_in_bounds_vertex_in_convex_hull_merge: no crash, merged={merged} -> "
            f"{'PASS' if merged else 'FAIL'}"
        )
        return merged
    except KeyError as e:
        print(f"test_isolated_in_bounds_vertex_in_convex_hull_merge: KeyError raised -> FAIL ({e})")
        return False


def main():
    results = [
        test_out_of_bounds_vertex_kept_but_edges_gated(),
        test_fully_in_bounds_obstacle_unaffected(),
        test_isolated_in_bounds_vertex_in_convex_hull_merge(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
