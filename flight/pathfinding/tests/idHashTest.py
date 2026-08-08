"""
Verification for per-drone node/mine ids (Field.droneNumber-prefixed,
Field._generateId) and Field.mineHash (order/id-independent, stable across
expansion, agreeing across drones that share the same mines).
"""

from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.protoMine import protoMine


def make_field(droneNumber=0, size=200):
    arbCorners = [[0, size], [size, size], [0, 0], [size, 0]]
    return Field([size, size], arbCorners, droneNumber)


def all_ids(field):
    ids = []
    for ob in field.polygonObstacles + field.mines + field.unionObstacles:
        if hasattr(ob, "id"):
            ids.append(ob.id)
        for node in ob.nodes:
            ids.append(node.id)
    return ids


def test_ids_start_with_drone_number_and_are_unique():
    field = make_field(droneNumber=7)
    pm1 = protoMine(3, (0.0, 0.0), (40, 40))
    pm2 = protoMine(3, (0.0, 0.0), (100, 100))
    field.addFromProtoMine(pm1)
    field.addFromProtoMine(pm2)
    field.createPolygonObstacle([(150, 150), (160, 150), (160, 160), (150, 160), (150, 150)])
    field.addFloatingNode(5, 5)

    ids = all_ids(field)
    all_prefixed = all(i is not None and i.startswith("7-") for i in ids)
    all_unique = len(ids) == len(set(ids))
    all_present = all(i is not None for i in ids)

    ok = all_prefixed and all_unique and all_present
    print(
        f"test_ids_start_with_drone_number_and_are_unique: count={len(ids)} "
        f"all_prefixed={all_prefixed} all_unique={all_unique} all_present={all_present} "
        f"-> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_mine_gets_its_own_id():
    field = make_field(droneNumber=2)
    pm = protoMine(3, (0.0, 0.0), (50, 50))
    field.addFromProtoMine(pm)
    mine = field.mines[0]
    ok = mine.id is not None and mine.id.startswith("2-")
    print(f"test_mine_gets_its_own_id: mine.id={mine.id} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_different_drones_different_ids_same_hash():
    positions = [(40, 40), (100, 100), (150, 60)]

    fieldA = make_field(droneNumber=1)
    for pos in positions:
        fieldA.addFromProtoMine(protoMine(3, (0.0, 0.0), pos))

    fieldB = make_field(droneNumber=99)
    for pos in positions:
        fieldB.addFromProtoMine(protoMine(3, (0.0, 0.0), pos))

    idsA = {m.id for m in fieldA.mines}
    idsB = {m.id for m in fieldB.mines}
    ids_disjoint = idsA.isdisjoint(idsB)
    hashes_match = fieldA.mineHash() == fieldB.mineHash()

    ok = ids_disjoint and hashes_match
    print(
        f"test_different_drones_different_ids_same_hash: ids_disjoint={ids_disjoint} "
        f"hashA_prefix={fieldA.mineHash()[:12]} hashB_prefix={fieldB.mineHash()[:12]} "
        f"hashes_match={hashes_match} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_hash_independent_of_add_order():
    positions = [(40, 40), (100, 100), (150, 60), (70, 130)]

    fieldA = make_field(droneNumber=1)
    for pos in positions:
        fieldA.addFromProtoMine(protoMine(3, (0.0, 0.0), pos))

    fieldB = make_field(droneNumber=1)
    for pos in reversed(positions):
        fieldB.addFromProtoMine(protoMine(3, (0.0, 0.0), pos))

    ok = fieldA.mineHash() == fieldB.mineHash()
    print(f"test_hash_independent_of_add_order: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_hash_differs_for_different_mines():
    fieldA = make_field()
    fieldA.addFromProtoMine(protoMine(3, (0.0, 0.0), (40, 40)))
    fieldA.addFromProtoMine(protoMine(3, (0.0, 0.0), (100, 100)))

    fieldB = make_field()
    fieldB.addFromProtoMine(protoMine(3, (0.0, 0.0), (40, 40)))
    fieldB.addFromProtoMine(protoMine(3, (0.0, 0.0), (105, 100)))  # different mine

    ok = fieldA.mineHash() != fieldB.mineHash()
    print(f"test_hash_differs_for_different_mines: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_hash_stable_across_expand_and_merge():
    field = make_field()
    field.addFromProtoMine(protoMine(3, (0.0, 0.0), (40, 40)))
    field.addFromProtoMine(protoMine(3, (0.0, 0.0), (100, 100)))

    before = field.mineHash()
    field.expandField(3.0)  # grows mines, possibly merges some -- origin shouldn't move
    after = field.mineHash()

    ok = before == after
    print(
        f"test_hash_stable_across_expand_and_merge: mines={len(field.mines)} "
        f"unions={len(field.unionObstacles)} before={before[:12]} after={after[:12]} "
        f"-> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_hash_accounts_for_mines_inside_unions():
    # two mines close enough to merge on add
    field = make_field()
    field.addFromProtoMine(protoMine(3, (0.0, 0.0), (50, 50)))
    field.addFromProtoMine(protoMine(3, (0.0, 0.0), (52, 50)))

    merged = len(field.unionObstacles) >= 1 and len(field.mines) == 0
    nested_mines = field._collect_mines(field.unionObstacles)

    empty_field = make_field()
    hash_with_union = field.mineHash()
    hash_empty = empty_field.mineHash()

    ok = merged and len(nested_mines) == 2 and hash_with_union != hash_empty
    print(
        f"test_hash_accounts_for_mines_inside_unions: merged={merged} "
        f"nested_mine_count={len(nested_mines)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_empty_field_hash_is_deterministic():
    a = make_field(droneNumber=5).mineHash()
    b = make_field(droneNumber=9).mineHash()
    ok = a == b
    print(f"test_empty_field_hash_is_deterministic: -> {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    results = [
        test_ids_start_with_drone_number_and_are_unique(),
        test_mine_gets_its_own_id(),
        test_different_drones_different_ids_same_hash(),
        test_hash_independent_of_add_order(),
        test_hash_differs_for_different_mines(),
        test_hash_stable_across_expand_and_merge(),
        test_hash_accounts_for_mines_inside_unions(),
        test_empty_field_hash_is_deterministic(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
