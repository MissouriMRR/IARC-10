"""
Verification for completed mine support: BlockMine construction via
addFromProtoMine, and Field.expandField's teardown/expand/rebuild cycle.
"""
import math
from shapely.geometry import LineString

from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.nodeField.BlockMine import BlockMine
from flight.pathfinding.protoMine import protoMine


def make_field(size=200):
    arbCorners = [[0, size], [size, size], [0, 0], [size, 0]]
    return Field([size, size], arbCorners)


def ground_truth_edges(obstacles):
    truth = set()
    for i, A in enumerate(obstacles):
        for j, C in enumerate(obstacles):
            if i >= j:
                continue
            for selfNode, otherNode in A.commonTangents(C):
                seg = LineString([(selfNode.x, selfNode.y), (otherNode.x, otherNode.y)])
                blocked = False
                for k, D in enumerate(obstacles):
                    if k == i or k == j:
                        continue
                    if D.polygon.crosses(seg) or D.polygon.contains(seg):
                        blocked = True
                        break
                if not blocked:
                    truth.add(frozenset([id(selfNode), id(otherNode)]))
    return truth


def actual_cross_obstacle_edges(field, obstacles):
    owner = {}
    for obstacle in obstacles:
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


def all_obstacles(field):
    return field.polygonObstacles + field.mines + field.unionObstacles


def check_safety(field, label):
    obstacles = all_obstacles(field)
    truth = ground_truth_edges(obstacles)
    actual = actual_cross_obstacle_edges(field, obstacles)
    unsafe = actual - truth
    missing = truth - actual
    ok = len(unsafe) == 0
    print(f"  [{label}] unsafe={len(unsafe)} missing={len(missing)} true_edges={len(truth)} -> "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_basic_construction():
    field = make_field()
    field.createPolygonObstacle([(140, 100), (150, 100), (150, 110), (140, 110), (140, 100)])
    pm = protoMine(3, (0.0, 0.0), (100, 100))
    field.addFromProtoMine(pm)

    has_mine = len(field.mines) == 1 and isinstance(field.mines[0], BlockMine)
    connected = all(len(field.fieldConnection.nodeGraph.get(n, {})) > 0 for n in field.mines[0].nodes)
    safe = check_safety(field, "basic_construction")
    passed = has_mine and connected and safe
    print(f"test_basic_construction: has_mine={has_mine} connected={connected} -> {'PASS' if passed else 'FAIL'}")
    return passed


def test_expand_standalone_repeated():
    field = make_field()
    field.createPolygonObstacle([(140, 100), (150, 100), (150, 110), (140, 110), (140, 100)])
    pm = protoMine(3, (0.0, 0.0), (100, 100))
    field.addFromProtoMine(pm)
    mine = field.mines[0]

    areas = [mine.polygon.area]
    for _ in range(3):
        field.expandField(2.0)
        mine = field.mines[0] if field.mines else None
        if mine is None:
            print("test_expand_standalone_repeated: mine vanished -> FAIL")
            return False
        areas.append(mine.polygon.area)

    grew_each_time = all(areas[i] < areas[i + 1] for i in range(len(areas) - 1))
    safe = check_safety(field, "expand_standalone_repeated")
    passed = grew_each_time and safe
    print(f"test_expand_standalone_repeated: areas={[round(a,1) for a in areas]} grew_each_time={grew_each_time} "
          f"-> {'PASS' if passed else 'FAIL'}")
    return passed


def test_expand_triggers_new_merge():
    field = make_field()
    # mine and a plain obstacle positioned close but not overlapping initially
    pm = protoMine(3, (0.0, 0.0), (100, 100))
    field.addFromProtoMine(pm)
    field.createPolygonObstacle([(110, 97), (120, 97), (120, 103), (110, 103), (110, 97)])

    before_unions = len(field.unionObstacles)
    field.expandField(6.0)
    after_unions = len(field.unionObstacles)
    merged = after_unions > before_unions
    safe = check_safety(field, "expand_triggers_new_merge")
    passed = merged and safe
    print(f"test_expand_triggers_new_merge: before_unions={before_unions} after_unions={after_unions} "
          f"-> {'PASS' if passed else 'FAIL'}")
    return passed


def test_expand_on_already_merged_mine():
    field = make_field()
    # mine overlapping a plain obstacle from the start -> merges immediately
    field.createPolygonObstacle([(103, 97), (115, 97), (115, 103), (103, 103), (103, 97)])
    pm = protoMine(3, (0.0, 0.0), (100, 100))
    field.addFromProtoMine(pm)

    started_merged = len(field.unionObstacles) == 1 and len(field.mines) == 0
    field.expandField(3.0)
    still_valid = (len(field.unionObstacles) >= 1) or (len(field.mines) == 1)
    safe = check_safety(field, "expand_on_already_merged_mine")
    passed = started_merged and still_valid and safe
    print(f"test_expand_on_already_merged_mine: started_merged={started_merged} "
          f"still_valid_after={still_valid} -> {'PASS' if passed else 'FAIL'}")
    return passed


def main():
    results = [
        test_basic_construction(),
        test_expand_standalone_repeated(),
        test_expand_triggers_new_merge(),
        test_expand_on_already_merged_mine(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
