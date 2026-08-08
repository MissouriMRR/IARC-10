"""
Verification for CellField: a bit-packed, single-bit-per-cell grid backed by
one Python arbitrary-precision int, with a row-buffer trick that lets a raw
shift move columns without corrupting adjacent rows.
"""
import random

from flight.pathfinding.cellField.cellField import CellField


def make_field_from_cells(width, height, cells, buffer=1):
    field = CellField(width, height, buffer)
    for x, y in cells:
        field.set(x, y)
    return field


def random_cells(width, height, count, seed):
    rng = random.Random(seed)
    cells = set()
    while len(cells) < count:
        cells.add((rng.randrange(width), rng.randrange(height)))
    return cells


def test_get_set_roundtrip():
    width, height = 17, 11
    field = CellField(width, height)
    corners = [(0, 0), (width - 1, 0), (0, height - 1), (width - 1, height - 1)]
    all_positions = [(x, y) for x in range(width) for y in range(height)]

    ok = True
    for x, y in all_positions:
        if field.get(x, y):
            ok = False
    for x, y in corners:
        field.set(x, y)
    for x, y in all_positions:
        expected = (x, y) in corners
        if field.get(x, y) != expected:
            ok = False
    for x, y in corners:
        field.clear(x, y)
        if field.get(x, y):
            ok = False

    print(f"test_get_set_roundtrip: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_clear_all():
    width, height = 15, 12
    field = CellField(width, height)
    for x, y in random_cells(width, height, 40, seed=321):
        field.set(x, y)
    had_cells = field.count() > 0

    field.clear_all()
    all_off = field.count() == 0 and list(field.on_cells()) == []

    # still usable afterward
    field.set(3, 3)
    still_works = field.get(3, 3) and field.count() == 1

    ok = had_cells and all_off and still_works
    print(f"test_clear_all: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_vertical_slice_correctness():
    width, height = 20, 16
    all_ok = True
    cells = random_cells(width, height, 60, seed=555)
    field = make_field_from_cells(width, height, cells)

    for start_frac, end_frac in [(0.0, 0.25), (0.75, 1.0), (0.25, 0.75), (0.0, 1.0)]:
        sliced = field.vertical_slice(start_frac, end_frac)
        start_row = round(start_frac * height)
        end_row = round(end_frac * height)
        # same-size output: cells inside the band keep their ORIGINAL (x,y),
        # everything outside the band is cleared
        expected = {(x, y) for x, y in cells if start_row <= y < end_row}
        actual = set(sliced.on_cells())
        right_height = sliced.height == height
        right_width = sliced.width == width
        if actual != expected or not right_height or not right_width:
            all_ok = False
            print(f"  MISMATCH [{start_frac},{end_frac}): expected={len(expected)} "
                  f"actual={len(actual)} right_height={right_height} right_width={right_width}")

    print(f"test_vertical_slice_correctness: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_vertical_slice_index_matches_fraction():
    width, height = 20, 20
    cells = random_cells(width, height, 40, seed=556)
    field = make_field_from_cells(width, height, cells)

    ok = True
    for n, m in [(0, 4), (1, 4), (3, 4), (0, 1), (2, 5)]:
        by_index = field.vertical_slice_index(n, m)
        by_fraction = field.vertical_slice(n / m, (n + 1) / m)
        if set(by_index.on_cells()) != set(by_fraction.on_cells()):
            ok = False

    print(f"test_vertical_slice_index_matches_fraction: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_vertical_slice_corners_and_coordinates():
    field = CellField(10, 20, min_corner=(0.0, 0.0), max_corner=(10.0, 40.0))  # cell_size = (1,2)
    top_quarter = field.vertical_slice(0.75, 1.0)

    # same-size output: corners/cell_size match the ORIGINAL field exactly,
    # so real_to_cell/cell_to_real on the result stay consistent with self
    same_min = top_quarter.min_corner == field.min_corner
    same_max = top_quarter.max_corner == field.max_corner
    same_cell_size = top_quarter.cell_size == field.cell_size
    same_shape = (top_quarter.width, top_quarter.height) == (field.width, field.height)
    # a point in the ORIGINAL field's own frame maps consistently on both
    right_mapping = top_quarter.real_to_cell(5.0, 31.0) == field.real_to_cell(5.0, 31.0) == (5, 15)

    ok = same_min and same_max and same_cell_size and same_shape and right_mapping
    print(f"test_vertical_slice_corners_and_coordinates: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_vertical_slice_invalid_raises():
    field = CellField(10, 10)
    results = []
    for start, end in [(-0.1, 0.5), (0.5, 1.1), (0.6, 0.4)]:
        try:
            field.vertical_slice(start, end)
            results.append(False)
        except ValueError:
            results.append(True)
    for n, m in [(4, 4), (-1, 4)]:
        try:
            field.vertical_slice_index(n, m)
            results.append(False)
        except ValueError:
            results.append(True)

    ok = all(results)
    print(f"test_vertical_slice_invalid_raises: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_vertical_slice_reasonably_fast_for_huge_field():
    # Same-size output means this can't be cheaper than O(field size) --
    # the output itself is that big -- but it should still be a plain bulk
    # byte-copy plus two range-clears, not per-cell work, so it should stay
    # fast in absolute terms even at a large scale.
    import time

    width, height = 20000, 20000
    field = CellField(width, height)
    for x, y in [(10, 5), (15, 8), (100, 12)]:
        field.set(x, y)

    t0 = time.perf_counter()
    sliced = field.vertical_slice(0.0, 0.001)  # top 20 rows of 20000
    elapsed = time.perf_counter() - t0

    correct = set(sliced.on_cells()) == {(10, 5), (15, 8), (100, 12)}
    same_size = (sliced.width, sliced.height) == (width, height)
    budget = 1.0
    fast = elapsed <= budget
    ok = correct and same_size and fast
    print(f"test_vertical_slice_reasonably_fast_for_huge_field: elapsed={elapsed*1000:.3f}ms "
          f"(budget={budget*1000:.0f}ms) correct={correct} same_size={same_size} -> {'PASS' if ok else 'FAIL'}")
    return ok


def reconstruct_coverage(field, shape, shape_center, centers):
    """Inverts cover_with_shape's center math to recover each placement's
    cell-space offset, then unions what the shape would cover from there --
    used to independently verify the returned centers actually cover
    everything, without trusting cover_with_shape's own bookkeeping."""
    shape_cells = list(shape.on_cells())
    covered = set()
    for cx, cy in centers:
        ox = (cx - field.min_corner[0]) / field.cell_size[0] - shape_center[0]
        oy = (cy - field.min_corner[1]) / field.cell_size[1] - shape_center[1]
        ox, oy = round(ox), round(oy)
        for sx, sy in shape_cells:
            covered.add((sx + ox, sy + oy))
    return covered


def test_cover_with_shape_single_placement_suffices():
    width, height = 20, 20
    target = CellField(width, height)
    target.fill_rect(5, 5, 8, 8)  # a small 3x3 cluster

    shape = CellField(6, 6, buffer=0)
    shape.fill_rect(0, 0, 6, 6)  # a shape big enough to cover it in one placement
    shape_center = (3.0, 3.0)

    centers = target.cover_with_shape(shape, shape_center=shape_center)
    covered = reconstruct_coverage(target, shape, shape_center, centers)
    target_cells = set(target.on_cells())

    fully_covered = target_cells.issubset(covered)
    single_placement = len(centers) == 1
    ok = fully_covered and single_placement
    print(f"test_cover_with_shape_single_placement_suffices: placements={len(centers)} "
          f"fully_covered={fully_covered} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_multiple_placements():
    width, height = 40, 40
    target = CellField(width, height)
    # three well-separated clusters -- no single small shape placement can
    # cover more than one of them
    for cx, cy in [(5, 5), (30, 6), (18, 32)]:
        target.fill_rect(cx, cy, cx + 3, cy + 3)

    shape = CellField(3, 3, buffer=0)
    shape.fill_rect(0, 0, 3, 3)
    shape_center = (1.0, 1.0)

    centers = target.cover_with_shape(shape, shape_center=shape_center)
    covered = reconstruct_coverage(target, shape, shape_center, centers)
    target_cells = set(target.on_cells())

    fully_covered = target_cells.issubset(covered)
    reasonable_count = len(centers) == 3  # exactly one placement per cluster is optimal here
    ok = fully_covered and reasonable_count
    print(f"test_cover_with_shape_multiple_placements: placements={len(centers)} "
          f"fully_covered={fully_covered} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_empty_target():
    target = CellField(20, 20)
    shape = CellField(3, 3, buffer=0)
    shape.fill_rect(0, 0, 3, 3)

    centers = target.cover_with_shape(shape)
    ok = centers == []
    print(f"test_cover_with_shape_empty_target: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_no_shape_cells_raises():
    target = CellField(20, 20)
    target.set(5, 5)
    empty_shape = CellField(3, 3, buffer=0)  # nothing set

    try:
        target.cover_with_shape(empty_shape)
        ok = False
    except ValueError:
        ok = True
    print(f"test_cover_with_shape_no_shape_cells_raises: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_center_shifts_result_correctly():
    # shape_center is only used to convert a chosen placement OFFSET into a
    # real-world center -- it never affects which offset the greedy loop
    # picks (a single isolated target cell ties across every shape cell
    # that could touch it, so the winning offset itself is arbitrary/
    # iteration-order-dependent). What must hold regardless of that tie is:
    # the same run picks the same offset for any shape_center, so two calls
    # differing only in shape_center must return centers differing by
    # exactly that shape_center delta.
    target = CellField(10, 10)
    target.set(5, 5)
    shape = CellField(4, 4, buffer=0)
    shape.fill_rect(0, 0, 4, 4)

    centers_a = target.cover_with_shape(shape, shape_center=(0.0, 0.0))
    centers_b = target.cover_with_shape(shape, shape_center=(2.0, 2.0))

    ok = (
        len(centers_a) == 1 and len(centers_b) == 1
        and abs((centers_b[0][0] - centers_a[0][0]) - 2.0) < 1e-9
        and abs((centers_b[0][1] - centers_a[0][1]) - 2.0) < 1e-9
    )
    print(f"test_cover_with_shape_center_shifts_result_correctly: a={centers_a} b={centers_b} "
          f"-> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_offfield_center_falls_back_to_cutoff_center():
    # A single target cell right at the field's bottom-left corner, covered
    # by a shape much bigger than the field -- the greedy tiebreak (lowest
    # y, then lowest x) picks the offset that covers the target with the
    # shape's OWN top-right-most cell, so the shape's bulk (and its nominal
    # bounding-box center) ends up hanging far off the field into negative
    # coordinates. The returned point must NOT be that off-field center --
    # it must be the centroid of just the on-field portion, which here is
    # exactly the one on-field cell (0,0) -- its own center, (0.5, 0.5).
    target = CellField(10, 10)
    target.set(0, 0)

    centers = target.cover_with_shape((8, 8))  # much bigger than the 10x10 field

    ok = len(centers) == 1
    if ok:
        cx, cy = centers[0]
        in_bounds = 0.0 <= cx <= 10.0 and 0.0 <= cy <= 10.0
        matches_cutoff_center = abs(cx - 0.5) < 1e-9 and abs(cy - 0.5) < 1e-9
        ok = in_bounds and matches_cutoff_center
    print(f"test_cover_with_shape_offfield_center_falls_back_to_cutoff_center: "
          f"centers={centers} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_rect_tuple_matches_equivalent_cellfield():
    # shape=(w,h) should behave EXACTLY as if the caller had built a solid
    # w x h CellField and passed that instead -- same placements, same
    # default center -- not just "also produces valid coverage".
    width, height = 30, 25
    target = CellField(width, height)
    for cx, cy in [(5, 5), (20, 15), (25, 3)]:
        target.fill_rect(cx, cy, cx + 5, cy + 5)

    rect_shape = CellField(2, 3, buffer=0)
    rect_shape.fill_rect(0, 0, 2, 3)

    via_tuple = target.cover_with_shape((2, 3))
    via_cellfield = target.cover_with_shape(rect_shape)

    ok = via_tuple == via_cellfield and len(via_tuple) > 0
    print(f"test_cover_with_shape_rect_tuple_matches_equivalent_cellfield: "
          f"placements={len(via_tuple)} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_rect_tuple_covers_everything():
    width, height = 30, 20
    target = CellField(width, height)
    for cx, cy in [(2, 2), (20, 4), (10, 14)]:
        target.fill_rect(cx, cy, cx + 4, cy + 6)

    shape_w, shape_h = 2, 3
    shape_center = (shape_w / 2.0, shape_h / 2.0)
    centers = target.cover_with_shape((shape_w, shape_h), shape_center=shape_center)

    covered = set()
    for cx, cy in centers:
        ox = round((cx - target.min_corner[0]) / target.cell_size[0] - shape_center[0])
        oy = round((cy - target.min_corner[1]) / target.cell_size[1] - shape_center[1])
        for sx in range(shape_w):
            for sy in range(shape_h):
                covered.add((sx + ox, sy + oy))

    target_cells = set(target.on_cells())
    ok = target_cells.issubset(covered)
    print(f"test_cover_with_shape_rect_tuple_covers_everything: placements={len(centers)} "
          f"-> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_rect_tuple_default_center():
    target = CellField(10, 10)
    target.set(5, 5)
    centers = target.cover_with_shape((2, 3))  # default center -> (1.0, 1.5)
    ok = len(centers) == 1
    print(f"test_cover_with_shape_rect_tuple_default_center: centers={centers} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_rect_tuple_invalid_raises():
    target = CellField(10, 10)
    target.set(5, 5)
    results = []
    for bad_shape in [(2,), (2, 3, 4), (0, 3), (2, -1), (0, 0)]:
        try:
            target.cover_with_shape(bad_shape)
            results.append(False)
        except ValueError:
            results.append(True)
    ok = all(results)
    print(f"test_cover_with_shape_rect_tuple_invalid_raises: -> {'PASS' if ok else 'FAIL'}")
    return ok


def _placement_footprints(shape_size, shape_center, centers, field):
    w, h = shape_size
    shape_cells = [(x, y) for x in range(w) for y in range(h)]
    footprints = []
    for cx, cy in centers:
        ox = round((cx - field.min_corner[0]) / field.cell_size[0] - shape_center[0])
        oy = round((cy - field.min_corner[1]) / field.cell_size[1] - shape_center[1])
        footprints.append({(sx + ox, sy + oy) for sx, sy in shape_cells})
    return footprints


def test_cover_with_shape_perfect_tiling_has_zero_overlap():
    # A target region that divides EXACTLY into whole shape tiles has a
    # zero-overlap optimal tiling available -- confirms the overlap
    # tiebreaker actually finds it, not just "some" cover.
    width, height = 20, 20
    target = CellField(width, height)
    target.fill_rect(2, 2, 10, 8)  # 8x6 region, divides evenly by 2x2 -> 12 tiles exactly

    shape_size = (2, 2)
    shape_center = (1.0, 1.0)
    centers = target.cover_with_shape(shape_size, shape_center=shape_center)
    footprints = _placement_footprints(shape_size, shape_center, centers, target)

    all_covered = set().union(*footprints) if footprints else set()
    total_cells = sum(len(f) for f in footprints)
    overlap_cells = total_cells - len(all_covered)

    optimal_count = 12
    right_count = len(centers) == optimal_count
    zero_overlap = overlap_cells == 0
    covers_target = set(target.on_cells()).issubset(all_covered)

    ok = right_count and zero_overlap and covers_target
    print(f"test_cover_with_shape_perfect_tiling_has_zero_overlap: placements={len(centers)} "
          f"(optimal={optimal_count}) overlap_cells={overlap_cells} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_cover_with_shape_overlap_tiebreak_reduces_overlap_vs_no_tiebreak():
    # Reference implementation WITHOUT the overlap tiebreaker (plain greedy,
    # ties broken arbitrarily by iteration order) -- confirms the real
    # method's total overlap is never worse, and is strictly better on a
    # case where plain greedy's arbitrary tie-breaking creates avoidable
    # overlap.
    def naive_cover(field, shape_size, shape_center):
        w, h = shape_size
        shape_cells = [(x, y) for x in range(w) for y in range(h)]
        remaining = set(field.on_cells())
        placements = []
        while remaining:
            best_offset = None
            best_covered = None
            tried = set()
            for rx, ry in remaining:
                for sx, sy in shape_cells:
                    offset = (rx - sx, ry - sy)
                    if offset in tried:
                        continue
                    tried.add(offset)
                    covered = {(sx2 + offset[0], sy2 + offset[1]) for sx2, sy2 in shape_cells} & remaining
                    if best_covered is None or len(covered) > len(best_covered):
                        best_offset = offset
                        best_covered = covered
            placements.append(best_offset)
            remaining -= best_covered
        centers = []
        for ox, oy in placements:
            cx = field.min_corner[0] + (ox + shape_center[0]) * field.cell_size[0]
            cy = field.min_corner[1] + (oy + shape_center[1]) * field.cell_size[1]
            centers.append((cx, cy))
        return centers

    def total_overlap(shape_size, shape_center, centers, field):
        footprints = _placement_footprints(shape_size, shape_center, centers, field)
        total = sum(len(f) for f in footprints)
        union = len(set().union(*footprints)) if footprints else 0
        return total - union

    width, height = 30, 20
    target = CellField(width, height)
    # an irregular target (union of overlapping disks) is exactly the kind
    # of shape where naive greedy's ties create avoidable overlap
    target.fill_disk(8, 8, 5)
    target.fill_disk(20, 10, 4)
    target.fill_disk(14, 14, 3)

    shape_size = (2, 2)
    shape_center = (1.0, 1.0)

    naive_centers = naive_cover(target, shape_size, shape_center)
    real_centers = target.cover_with_shape(shape_size, shape_center=shape_center)

    naive_overlap = total_overlap(shape_size, shape_center, naive_centers, target)
    real_overlap = total_overlap(shape_size, shape_center, real_centers, target)

    naive_footprints = _placement_footprints(shape_size, shape_center, naive_centers, target)
    real_footprints = _placement_footprints(shape_size, shape_center, real_centers, target)
    target_cells = set(target.on_cells())
    naive_covers = target_cells.issubset(set().union(*naive_footprints))
    real_covers = target_cells.issubset(set().union(*real_footprints))

    # Greedy tie-breaking that reduces overlap can occasionally cost a small
    # number of extra placements later on (different choices cascade), so
    # this isn't asserted as strictly non-worse -- just not worse by much,
    # in exchange for a real overlap reduction, which is the property that
    # actually matters here.
    count_margin = 3
    count_not_much_worse = len(real_centers) <= len(naive_centers) + count_margin
    less_or_equal_overlap = real_overlap <= naive_overlap
    strictly_better_here = real_overlap < naive_overlap

    ok = count_not_much_worse and less_or_equal_overlap and strictly_better_here and naive_covers and real_covers
    print(f"test_cover_with_shape_overlap_tiebreak_reduces_overlap_vs_no_tiebreak: "
          f"naive(placements={len(naive_centers)}, overlap={naive_overlap}) "
          f"real(placements={len(real_centers)}, overlap={real_overlap}) -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_bitwise_ops():
    width, height = 12, 9
    seed_a = random_cells(width, height, 20, seed=1)
    seed_b = random_cells(width, height, 20, seed=2)
    a = make_field_from_cells(width, height, seed_a)
    b = make_field_from_cells(width, height, seed_b)

    and_ref = seed_a & seed_b
    or_ref = seed_a | seed_b
    xor_ref = seed_a ^ seed_b

    and_actual = set((a & b).on_cells())
    or_actual = set((a | b).on_cells())
    xor_actual = set((a ^ b).on_cells())

    functional_ok = and_actual == and_ref and or_actual == or_ref and xor_actual == xor_ref

    # in-place variants must match and mutate self, leaving the functional
    # forms' inputs untouched above
    a_and = a.copy()
    a_and &= b
    a_or = a.copy()
    a_or |= b
    a_xor = a.copy()
    a_xor ^= b

    inplace_ok = (
        set(a_and.on_cells()) == and_ref
        and set(a_or.on_cells()) == or_ref
        and set(a_xor.on_cells()) == xor_ref
    )

    ok = functional_ok and inplace_ok
    print(f"test_bitwise_ops: functional={functional_ok} inplace={inplace_ok} -> {'PASS' if ok else 'FAIL'}")
    return ok


def naive_shift(cells, width, height, dx, dy):
    result = set()
    for x, y in cells:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height:
            result.add((nx, ny))
    return result


def test_shift_correctness():
    width, height, buffer = 10, 8, 1
    # cells at every row's leftmost/rightmost column, plus a scattering of
    # interior cells -- exactly the positions the buffer trick protects
    edge_cells = {(0, y) for y in range(height)} | {(width - 1, y) for y in range(height)}
    interior_cells = random_cells(width, height, 15, seed=42)
    cells = edge_cells | interior_cells

    all_ok = True
    for dx in range(-buffer, buffer + 1):
        for dy in list(range(-height - 1, height + 2)):
            field = make_field_from_cells(width, height, cells, buffer)
            expected = naive_shift(cells, width, height, dx, dy)

            functional_actual = set(field.shift(dx, dy).on_cells())
            still_unchanged = set(field.on_cells()) == cells

            inplace_field = make_field_from_cells(width, height, cells, buffer)
            inplace_field.shift_inplace(dx, dy)
            inplace_actual = set(inplace_field.on_cells())

            if functional_actual != expected or inplace_actual != expected or not still_unchanged:
                all_ok = False
                print(f"  MISMATCH dx={dx} dy={dy}: expected={sorted(expected)} "
                      f"functional={sorted(functional_actual)} inplace={sorted(inplace_actual)} "
                      f"shift_left_original_unchanged={still_unchanged}")

    print(f"test_shift_correctness: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def naive_expand(cells, width, height):
    result = set()
    for x, y in cells:
        for nx, ny in ((x, y), (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < width and 0 <= ny < height:
                result.add((nx, ny))
    return result


def test_expand_correctness():
    width, height, buffer = 10, 8, 1
    # cells at every row's leftmost/rightmost column (where a horizontal
    # expand would spill into the buffer zone) and at the top/bottom rows
    # (where a vertical expand would spill past the field entirely), plus a
    # scattering of interior cells.
    edge_cells = {(0, y) for y in range(height)} | {(width - 1, y) for y in range(height)}
    edge_cells |= {(x, 0) for x in range(width)} | {(x, height - 1) for x in range(width)}
    interior_cells = random_cells(width, height, 15, seed=43)
    cells = edge_cells | interior_cells
    expected = naive_expand(cells, width, height)

    field = make_field_from_cells(width, height, cells, buffer)
    functional_actual = set(field.expand().on_cells())
    unchanged_after = set(field.on_cells()) == cells
    distinct_object = field.expand() is not field

    inplace_field = make_field_from_cells(width, height, cells, buffer)
    inplace_field.expand_inplace()
    inplace_actual = set(inplace_field.on_cells())

    ok = functional_actual == expected and inplace_actual == expected and unchanged_after and distinct_object
    if not ok:
        print(f"  expected={sorted(expected)} functional={sorted(functional_actual)} "
              f"inplace={sorted(inplace_actual)} unchanged_after={unchanged_after}")
    print(f"test_expand_correctness: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_expand_empty_and_full_field():
    width, height = 6, 5
    empty = CellField(width, height)
    empty_ok = empty.expand().count() == 0

    full = CellField(width, height)
    for x in range(width):
        for y in range(height):
            full.set(x, y)
    full_expanded = full.expand()
    full_ok = full_expanded.count() == width * height  # already saturated, expand is a no-op

    ok = empty_ok and full_ok
    print(f"test_expand_empty_and_full_field: empty_ok={empty_ok} full_ok={full_ok} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_expand_requires_buffer_raises():
    field = CellField(10, 10, buffer=0)
    field.set(3, 3)
    raised_functional = False
    raised_inplace = False
    try:
        field.expand()
    except ValueError:
        raised_functional = True
    try:
        field.expand_inplace()
    except ValueError:
        raised_inplace = True

    ok = raised_functional and raised_inplace
    print(f"test_expand_requires_buffer_raises: functional={raised_functional} "
          f"inplace={raised_inplace} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_shift_dx_exceeds_buffer_raises():
    field = CellField(10, 10, buffer=1)
    raised_functional = False
    raised_inplace = False
    try:
        field.shift(2, 0)
    except ValueError:
        raised_functional = True
    try:
        field.shift_inplace(-2, 0)
    except ValueError:
        raised_inplace = True

    ok = raised_functional and raised_inplace
    print(f"test_shift_dx_exceeds_buffer_raises: functional={raised_functional} "
          f"inplace={raised_inplace} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_inplace_vs_functional_identity():
    width, height = 8, 8
    cells_a = random_cells(width, height, 10, seed=5)
    cells_b = random_cells(width, height, 10, seed=6)
    a = make_field_from_cells(width, height, cells_a)
    b = make_field_from_cells(width, height, cells_b)

    a_snapshot = set(a.on_cells())
    shifted = a.shift(1, 1)
    unchanged_after_shift = set(a.on_cells()) == a_snapshot
    distinct_object_shift = shifted is not a

    anded = a & b
    unchanged_after_and = set(a.on_cells()) == a_snapshot
    distinct_object_and = anded is not a

    a2 = a.copy()
    ret = a2.__iand__(b)
    mutated_inplace = set(a2.on_cells()) == (cells_a & cells_b)
    returns_self = ret is a2

    ok = (
        unchanged_after_shift
        and distinct_object_shift
        and unchanged_after_and
        and distinct_object_and
        and mutated_inplace
        and returns_self
    )
    print(f"test_inplace_vs_functional_identity: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_to_from_bytes_roundtrip():
    width, height, buffer = 13, 9, 1
    cells = random_cells(width, height, 25, seed=7)
    field = make_field_from_cells(width, height, cells, buffer)

    data = field.to_bytes()
    restored = CellField.from_bytes(data, width, height, buffer)

    ok = set(restored.on_cells()) == cells
    print(f"test_to_from_bytes_roundtrip: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_on_cells_matches_naive_scan():
    all_ok = True
    for count, seed in [(5, 10), (60, 11)]:  # sparse and dense
        width, height = 12, 10
        cells = random_cells(width, height, min(count, width * height), seed)
        field = make_field_from_cells(width, height, cells)
        naive = {(x, y) for x in range(width) for y in range(height) if field.get(x, y)}
        actual = set(field.on_cells())
        if actual != cells or actual != naive:
            all_ok = False

    print(f"test_on_cells_matches_naive_scan: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_mismatched_shape_raises():
    a = CellField(10, 10, buffer=1)
    b = CellField(11, 10, buffer=1)
    c = CellField(10, 10, buffer=2)

    results = []
    for other in (b, c):
        for op in ("__and__", "__or__", "__xor__", "__iand__", "__ior__", "__ixor__"):
            try:
                getattr(a.copy(), op)(other)
                results.append(False)
            except ValueError:
                results.append(True)

    ok = all(results)
    print(f"test_mismatched_shape_raises: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_large_scale_sanity():
    # 10000x10000 (~12.5 MB) with many individual set() calls -- large
    # enough to have caught two real regressions found during development:
    # an O(height^2) row-mask construction, and set()/on_cells() implementations
    # that degraded to O(field size) per call/bit due to reallocating or
    # rescanning the whole field. A generous time budget catches a return of
    # either without being a flaky timing test on ordinary hardware.
    import time

    width, height, buffer = 10000, 10000, 1
    time_budget_s = 15.0

    t0 = time.perf_counter()

    cells = random_cells(width, height, 20000, seed=99)
    field = make_field_from_cells(width, height, cells, buffer)

    shifted = field.shift(1, 3)
    expected = naive_shift(cells, width, height, 1, 3)
    shift_ok = set(shifted.on_cells()) == expected

    count_ok = field.count() == len(cells)

    other_cells = random_cells(width, height, 20000, seed=100)
    other = make_field_from_cells(width, height, other_cells, buffer)
    combined = field | other
    or_ref = cells | other_cells
    or_ok = set(combined.on_cells()) == or_ref

    elapsed = time.perf_counter() - t0
    time_ok = elapsed <= time_budget_s

    ok = shift_ok and count_ok and or_ok and time_ok
    print(f"test_large_scale_sanity: shift_ok={shift_ok} count_ok={count_ok} or_ok={or_ok} "
          f"elapsed={elapsed:.2f}s (budget={time_budget_s}s) -> {'PASS' if ok else 'FAIL'}")
    return ok


def naive_disk_cells(cx, cy, radius, width, height):
    cells = set()
    for y in range(max(0, cy - radius), min(height - 1, cy + radius) + 1):
        for x in range(max(0, cx - radius), min(width - 1, cx + radius) + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius * radius:
                cells.add((x, y))
    return cells


def test_fill_rect():
    width, height = 20, 15
    all_ok = True

    # interior rect, fully in bounds
    field = CellField(width, height)
    field.fill_rect(3, 2, 10, 8)
    expected = {(x, y) for x in range(3, 10) for y in range(2, 8)}
    if set(field.on_cells()) != expected:
        all_ok = False

    # clipped at every edge (negative/over-wide bounds)
    field2 = CellField(width, height)
    field2.fill_rect(-5, -5, width + 5, height + 5)
    expected2 = {(x, y) for x in range(width) for y in range(height)}
    if set(field2.on_cells()) != expected2:
        all_ok = False

    # fill then clear the same region
    field3 = CellField(width, height)
    field3.fill_rect(2, 2, 12, 12)
    field3.fill_rect(4, 4, 8, 8, value=False)
    expected3 = {(x, y) for x in range(2, 12) for y in range(2, 12)} - {
        (x, y) for x in range(4, 8) for y in range(4, 8)
    }
    if set(field3.on_cells()) != expected3:
        all_ok = False

    # degenerate/empty ranges are no-ops
    field4 = CellField(width, height)
    field4.fill_rect(5, 5, 5, 5)
    field4.fill_rect(10, 3, 3, 10)
    if field4.count() != 0:
        all_ok = False

    print(f"test_fill_rect: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_fill_disk():
    width, height = 40, 40
    all_ok = True

    for cx, cy, radius in [(20, 20, 8), (0, 0, 5), (39, 39, 6), (5, 35, 10)]:
        field = CellField(width, height)
        field.fill_disk(cx, cy, radius)
        expected = naive_disk_cells(cx, cy, radius, width, height)
        actual = set(field.on_cells())
        if actual != expected:
            all_ok = False
            print(f"  MISMATCH disk center=({cx},{cy}) r={radius}: "
                  f"expected={len(expected)} actual={len(actual)}")

    field = CellField(width, height)
    field.fill_disk(20, 20, -1)  # negative radius is a no-op
    if field.count() != 0:
        all_ok = False

    print(f"test_fill_disk: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_fill_polygon_covered_axis_aligned_rect():
    # A grid-aligned rectangle should cover EXACTLY the cells inside it,
    # including cells whose corner exactly touches the rectangle's own
    # boundary -- not just the strict interior.
    field = CellField(10, 10)
    field.fill_polygon_covered([(2, 2), (6, 2), (6, 6), (2, 6)])
    actual = set(field.on_cells())
    expected = {(x, y) for x in range(2, 6) for y in range(2, 6)}
    ok = actual == expected
    print(f"test_fill_polygon_covered_axis_aligned_rect: count={len(actual)} (expected {len(expected)}) "
          f"-> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_covered_offset_rect():
    # Same shape, NOT grid-aligned (a more realistic case -- real image
    # corners essentially never land exactly on cell boundaries) -- exactly
    # the cells fully inside should be set, nothing partially clipped.
    field = CellField(20, 20)
    field.fill_polygon_covered([(3.3, 3.3), (9.7, 3.3), (9.7, 9.7), (3.3, 9.7)])
    actual = set(field.on_cells())
    expected = {(x, y) for x in range(4, 9) for y in range(4, 9)}
    ok = actual == expected
    print(f"test_fill_polygon_covered_offset_rect: count={len(actual)} (expected {len(expected)}) "
          f"-> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_covered_excludes_partially_clipped_cells():
    # A polygon smaller than a single cell must cover ZERO cells -- no cell
    # is FULLY inside it, even though it clips one cell's interior.
    field = CellField(10, 10)
    field.fill_polygon_covered([(5.1, 5.1), (5.4, 5.1), (5.4, 5.4), (5.1, 5.4)])
    ok = field.count() == 0
    print(f"test_fill_polygon_covered_excludes_partially_clipped_cells: count={field.count()} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_covered_rotated():
    # A rotated (diamond) polygon -- not axis-aligned at all -- should still
    # only set cells strictly/fully within it, and should be symmetric for
    # a symmetric input.
    field = CellField(10, 10)
    field.fill_polygon_covered([(5, 2), (8, 5), (5, 8), (2, 5)])
    on = set(field.on_cells())
    cx, cy = 5, 5
    symmetric = all((2 * cx - 1 - x, 2 * cy - 1 - y) in on for x, y in on)
    ok = len(on) > 0 and symmetric
    print(f"test_fill_polygon_covered_rotated: count={len(on)} symmetric={symmetric} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_covered_invalid_raises():
    field = CellField(10, 10)
    raised = False
    try:
        field.fill_polygon_covered([(1, 1), (2, 2)])  # only 2 vertices
    except ValueError:
        raised = True
    print(f"test_fill_polygon_covered_invalid_raises: -> {'PASS' if raised else 'FAIL'}")
    return raised


def test_fill_polygon_touched_axis_aligned_rect():
    # Grid-aligned, so touched should equal exactly the same 4x4 block
    # fill_polygon_covered gets -- no partially-clipped border cells exist
    # for a perfectly grid-aligned rectangle.
    field = CellField(10, 10)
    field.fill_polygon_touched([(2, 2), (6, 2), (6, 6), (2, 6)])
    actual = set(field.on_cells())
    expected = {(x, y) for x in range(2, 6) for y in range(2, 6)}
    ok = actual == expected
    print(f"test_fill_polygon_touched_axis_aligned_rect: count={len(actual)} (expected {len(expected)}) "
          f"-> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_touched_includes_clipped_border_but_not_diagonal_neighbors():
    # An offset (non-grid-aligned) rectangle: touched must include the ring
    # of cells the rectangle only PARTIALLY clips (fill_polygon_covered
    # excludes those, since they're not FULLY inside) -- a real, larger set
    # than "covered" -- but must NOT include cells with only a zero-area
    # (corner or edge) boundary contact, which a naive closed-boundary
    # containment test would wrongly include (verified empirically to
    # inflate a 4x4 true footprint to 6x6 before this was fixed).
    field = CellField(20, 20)
    field.fill_polygon_touched([(3.3, 3.3), (9.7, 3.3), (9.7, 9.7), (3.3, 9.7)])
    actual = set(field.on_cells())
    expected = {(x, y) for x in range(3, 10) for y in range(3, 10)}
    ok = actual == expected
    print(f"test_fill_polygon_touched_includes_clipped_border_but_not_diagonal_neighbors: "
          f"count={len(actual)} (expected {len(expected)}) -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_touched_single_cell_no_diagonal_bleed():
    # A polygon that's exactly one grid-aligned cell must touch ONLY that
    # cell -- not its edge- or corner-adjacent neighbors, which share zero
    # area with it.
    field = CellField(10, 10)
    field.fill_polygon_touched([(5, 5), (6, 5), (6, 6), (5, 6)])
    actual = set(field.on_cells())
    ok = actual == {(5, 5)}
    print(f"test_fill_polygon_touched_single_cell_no_diagonal_bleed: cells={sorted(actual)} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_touched_covers_more_than_fill_polygon_covered():
    # touched is always a SUPERSET of covered for the same polygon -- and
    # for a small (sub-few-cell) shape like a real mine's safety-zone
    # polygon, covered can come back completely empty while touched
    # correctly still marks its actual footprint.
    covered_field = CellField(10, 10)
    covered_field.fill_polygon_covered([(5.1, 5.1), (5.9, 5.1), (5.9, 5.9), (5.1, 5.9)])
    touched_field = CellField(10, 10)
    touched_field.fill_polygon_touched([(5.1, 5.1), (5.9, 5.1), (5.9, 5.9), (5.1, 5.9)])
    covered = set(covered_field.on_cells())
    touched = set(touched_field.on_cells())
    ok = covered == set() and touched == {(5, 5)} and covered.issubset(touched)
    print(f"test_fill_polygon_touched_covers_more_than_fill_polygon_covered: "
          f"covered={len(covered)} touched={len(touched)} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_polygon_touched_invalid_raises():
    field = CellField(10, 10)
    raised = False
    try:
        field.fill_polygon_touched([(1, 1), (2, 2)])  # only 2 vertices
    except ValueError:
        raised = True
    print(f"test_fill_polygon_touched_invalid_raises: -> {'PASS' if raised else 'FAIL'}")
    return raised


def test_fill_aligned_rect_matches_polygon_equivalents():
    width, height = 20, 16
    boxes = [
        (2.0, 3.0, 9.0, 11.0),      # exactly on cell boundaries
        (2.4, 3.6, 8.7, 10.1),      # arbitrary, no boundary alignment
        (0.0, 0.0, 20.0, 16.0),     # the whole field
        (5.0, 5.0, 5.001, 9.0),     # a hairline-thin box (much thinner than a cell)
    ]
    all_ok = True
    for x0, y0, x1, y1 in boxes:
        poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

        covered_ref = CellField(width, height)
        covered_ref.fill_polygon_covered(poly)
        covered_fast = CellField(width, height)
        covered_fast.fill_aligned_rect_covered(x0, y0, x1, y1)
        covered_ok = set(covered_ref.on_cells()) == set(covered_fast.on_cells())

        touched_ref = CellField(width, height)
        touched_ref.fill_polygon_touched(poly)
        touched_fast = CellField(width, height)
        touched_fast.fill_aligned_rect_touched(x0, y0, x1, y1)
        touched_ok = set(touched_ref.on_cells()) == set(touched_fast.on_cells())

        if not (covered_ok and touched_ok):
            all_ok = False
            print(f"  MISMATCH box=({x0},{y0},{x1},{y1}) covered_ok={covered_ok} touched_ok={touched_ok}")
            print(f"    covered ref={sorted(covered_ref.on_cells())} fast={sorted(covered_fast.on_cells())}")
            print(f"    touched ref={sorted(touched_ref.on_cells())} fast={sorted(touched_fast.on_cells())}")

    print(f"test_fill_aligned_rect_matches_polygon_equivalents: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_fill_aligned_rect_reversed_bounds():
    # x1<x0 and/or y1<y0 should behave identically to the normalized order
    width, height = 12, 12
    a = CellField(width, height)
    a.fill_aligned_rect_covered(2.0, 3.0, 8.0, 9.0)
    b = CellField(width, height)
    b.fill_aligned_rect_covered(8.0, 9.0, 2.0, 3.0)
    covered_ok = set(a.on_cells()) == set(b.on_cells()) and a.count() > 0

    c = CellField(width, height)
    c.fill_aligned_rect_touched(2.0, 3.0, 8.0, 9.0)
    d = CellField(width, height)
    d.fill_aligned_rect_touched(8.0, 9.0, 2.0, 3.0)
    touched_ok = set(c.on_cells()) == set(d.on_cells()) and c.count() > 0

    ok = covered_ok and touched_ok
    print(f"test_fill_aligned_rect_reversed_bounds: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_fill_aligned_rect_faster_than_polygon():
    import time

    width, height = 2000, 2000
    field = CellField(width, height, max_corner=(width, height))
    box = (300.0, 300.0, 1700.0, 1700.0)
    poly = [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])]

    t0 = time.perf_counter()
    field.fill_aligned_rect_covered(*box)
    fast_time = time.perf_counter() - t0
    fast_cells = set(field.on_cells())

    poly_field = CellField(width, height, max_corner=(width, height))
    t1 = time.perf_counter()
    poly_field.fill_polygon_covered(poly)
    poly_time = time.perf_counter() - t1
    poly_cells = set(poly_field.on_cells())

    correct = fast_cells == poly_cells
    faster = fast_time < poly_time
    print(f"test_fill_aligned_rect_faster_than_polygon: fast={fast_time*1000:.3f}ms "
          f"poly={poly_time*1000:.3f}ms correct={correct} faster={faster} -> "
          f"{'PASS' if (correct and faster) else 'FAIL'}")
    return correct and faster


def test_fill_disk_faster_than_per_cell_set():
    import time

    width, height = 2000, 2000
    field = CellField(width, height)
    t0 = time.perf_counter()
    field.fill_disk(1000, 1000, 200)
    bulk_time = time.perf_counter() - t0

    naive = CellField(width, height)
    cells = naive_disk_cells(1000, 1000, 200, width, height)
    t1 = time.perf_counter()
    for x, y in cells:
        naive.set(x, y)
    naive_time = time.perf_counter() - t1

    correct = set(field.on_cells()) == cells
    faster = bulk_time < naive_time
    ok = correct and faster
    print(f"test_fill_disk_faster_than_per_cell_set: cells={len(cells)} "
          f"bulk={bulk_time*1000:.3f}ms naive={naive_time*1000:.3f}ms "
          f"correct={correct} faster={faster} -> {'PASS' if ok else 'FAIL'}")
    return ok


def naive_apply_mask(base_cells, mask_cells, ox, oy, width, height, op):
    shifted = set()
    for mx, my in mask_cells:
        tx, ty = mx + ox, my + oy
        if 0 <= tx < width and 0 <= ty < height:
            shifted.add((tx, ty))

    if op == "or":
        return base_cells | shifted
    if op == "xor":
        return base_cells ^ shifted

    # "and" and "set" only affect cells within the mask's placed footprint
    # (its bounding box, clipped to the field) -- like or/xor, anything
    # outside that footprint is left exactly as it was.
    footprint = set()
    mask_w = max((mx for mx, _ in mask_cells), default=-1) + 1
    mask_h = max((my for _, my in mask_cells), default=-1) + 1
    for mx in range(mask_w):
        for my in range(mask_h):
            tx, ty = mx + ox, my + oy
            if 0 <= tx < width and 0 <= ty < height:
                footprint.add((tx, ty))
    outside = base_cells - footprint

    if op == "and":
        return outside | (base_cells & shifted)
    if op == "set":
        return outside | shifted
    raise ValueError(op)


def test_apply_mask():
    width, height = 30, 25
    all_ok = True

    mask = CellField(9, 9, buffer=0)
    mask.fill_disk(4, 4, 4)
    mask_cells = set(mask.on_cells())

    base = random_cells(width, height, 30, seed=200)

    for op in ("or", "and", "xor", "set"):
        for ox, oy in [(10, 10), (-3, -3), (width - 3, height - 3), (-20, 5), (5, -20), (100, 100)]:
            field = make_field_from_cells(width, height, base)
            field.apply_mask(mask, ox, oy, op=op)
            expected = naive_apply_mask(base, mask_cells, ox, oy, width, height, op)
            actual = set(field.on_cells())
            if actual != expected:
                all_ok = False
                print(f"  MISMATCH op={op} offset=({ox},{oy}): "
                      f"expected={len(expected)} actual={len(actual)}")

    field = CellField(width, height)
    try:
        field.apply_mask(mask, 0, 0, op="bogus")
        all_ok = False
    except ValueError:
        pass

    print(f"test_apply_mask: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_apply_mask_cost_independent_of_field_size():
    import time

    mask = CellField(41, 41, buffer=0)
    mask.fill_disk(20, 20, 20)

    times = []
    for size in (500, 5000, 20000):
        field = CellField(size, size)
        t0 = time.perf_counter()
        field.apply_mask(mask, size // 2 - 20, size // 2 - 20)
        times.append(time.perf_counter() - t0)

    # cost should stay roughly flat (proportional to the mask, not the
    # field) -- allow generous slack since this is a coarse timing check,
    # not a precise complexity proof
    ratio = times[-1] / times[0] if times[0] > 0 else float("inf")
    ok = ratio < 20
    print(f"test_apply_mask_cost_independent_of_field_size: times={[f'{t*1000:.3f}ms' for t in times]} "
          f"ratio(largest/smallest)={ratio:.1f} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_real_cell_roundtrip():
    all_ok = True

    # trivial default mapping: 1 unit per cell, origin at (0,0)
    field = CellField(10, 8)
    for col, row in [(0, 0), (9, 7), (5, 3)]:
        x, y = field.cell_to_real(col, row)
        back_col, back_row = field.real_to_cell(x, y)
        if (back_col, back_row) != (col, row):
            all_ok = False

    # non-trivial corners/cell size
    field2 = CellField(20, 20, min_corner=(100.0, -50.0), max_corner=(140.0, -10.0))
    cs = field2.cell_size
    if abs(cs[0] - 2.0) > 1e-9 or abs(cs[1] - 2.0) > 1e-9:
        all_ok = False
    for col, row in [(0, 0), (19, 19), (10, 3)]:
        x, y = field2.cell_to_real(col, row)
        back_col, back_row = field2.real_to_cell(x, y)
        if (back_col, back_row) != (col, row):
            all_ok = False
    # a point at the exact min corner maps to cell (0,0)
    if field2.real_to_cell(100.0, -50.0) != (0, 0):
        all_ok = False

    print(f"test_real_cell_roundtrip: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def dense_sample_path_cells(field, path, samples_per_unit_length=20):
    cells = set()
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        length = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        n = max(2, int(length * samples_per_unit_length))
        for k in range(n + 1):
            t = k / n
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            col, row = field.real_to_cell(x, y)
            if 0 <= col < field.width and 0 <= row < field.height:
                cells.add((col, row))
    if len(path) == 1:
        col, row = field.real_to_cell(*path[0])
        if 0 <= col < field.width and 0 <= row < field.height:
            cells.add((col, row))
    return cells


def test_mark_path_dense_sampling():
    width, height = 30, 25
    all_ok = True
    paths = [
        [(2.0, 2.0), (2.0, 20.0)],  # vertical
        [(2.0, 2.0), (25.0, 2.0)],  # horizontal
        [(1.0, 1.0), (28.0, 22.0)],  # shallow diagonal
        [(1.0, 1.0), (1.0, 10.0), (15.0, 10.0), (15.0, 22.0), (28.0, 22.0)],  # zigzag
        [(0.5, 0.5), (5.3, 7.9), (12.1, 3.2), (20.0, 20.0)],  # irregular waypoints
    ]
    for path in paths:
        field = CellField(width, height)
        field.mark_path(path)
        actual = set(field.on_cells())
        # dense sampling is a coarse (not exact) reference near cell
        # boundaries, but it should be a SUBSET of what a correct supercover
        # traversal marks -- any sampled cell missing from `actual` means a
        # real gap in the traversal.
        expected_subset = dense_sample_path_cells(field, path)
        missing = expected_subset - actual
        if missing:
            all_ok = False
            print(f"  GAP for path={path}: missing cells sampled-but-not-marked={missing}")

    print(f"test_mark_path_dense_sampling: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_mark_path_exact_corner_ties():
    # a perfect 45-degree segment with unit cells crosses grid corners
    # exactly -- confirms the tie-handling branch doesn't leave gaps
    width, height = 10, 10
    all_ok = True

    field = CellField(width, height)
    field.mark_path([(0.0, 0.0), (7.0, 7.0)])
    actual = set(field.on_cells())
    # every integer diagonal cell must be present, and since the line
    # touches each corner exactly, the two cells adjacent to each corner
    # crossing must also be present (no diagonal-only "thread" gap)
    for i in range(7):
        if (i, i) not in actual:
            all_ok = False
        if i > 0 and not ((i - 1, i) in actual or (i, i - 1) in actual):
            all_ok = False

    print(f"test_mark_path_exact_corner_ties: cells={sorted(actual)} -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_from_path_factory():
    path = [(1.0, 1.0), (18.0, 1.0), (18.0, 13.0)]
    field = CellField.from_path(path, min_corner=(0.0, 0.0), max_corner=(20.0, 15.0), width=20, height=15)

    reference = CellField(20, 15, min_corner=(0.0, 0.0), max_corner=(20.0, 15.0))
    reference.mark_path(path)

    matches_manual = set(field.on_cells()) == set(reference.on_cells())
    right_shape = (field.width, field.height) == (20, 15)
    right_bounds = field.min_corner == (0.0, 0.0) and field.max_corner == (20.0, 15.0)

    ok = matches_manual and right_shape and right_bounds
    print(f"test_from_path_factory: matches_manual={matches_manual} right_shape={right_shape} "
          f"right_bounds={right_bounds} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_mismatched_corners_raises():
    a = CellField(10, 10, min_corner=(0.0, 0.0), max_corner=(10.0, 10.0))
    b = CellField(10, 10, min_corner=(5.0, 5.0), max_corner=(15.0, 15.0))

    results = []
    for op in ("__and__", "__or__", "__xor__", "__iand__", "__ior__", "__ixor__"):
        try:
            getattr(a.copy(), op)(b)
            results.append(False)
        except ValueError:
            results.append(True)

    ok = all(results)
    print(f"test_mismatched_corners_raises: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_out_of_bounds_path_no_crash():
    width, height = 20, 15
    all_ok = True

    # entirely outside -- nothing should be marked, no exception
    field = CellField(width, height)
    try:
        field.mark_path([(-100.0, -100.0), (-50.0, -50.0)])
    except Exception as e:
        all_ok = False
        print(f"  entirely-outside path raised: {e!r}")
    if field.count() != 0:
        all_ok = False

    # crosses fully through -- clipped to exactly the row it passes through
    field2 = CellField(width, height)
    try:
        field2.mark_path([(-10.0, 5.5), (25.0, 5.5)])
    except Exception as e:
        all_ok = False
        print(f"  crossing path raised: {e!r}")
    expected = {(c, 5) for c in range(width)}
    if set(field2.on_cells()) != expected:
        all_ok = False
        print(f"  crossing path mismatch: expected {len(expected)} cells, got {field2.count()}")

    # enters through a corner
    field3 = CellField(width, height)
    try:
        field3.mark_path([(-5.0, -5.0), (5.0, 5.0)])
    except Exception as e:
        all_ok = False
        print(f"  corner-entry path raised: {e!r}")
    if field3.count() == 0:
        all_ok = False

    print(f"test_out_of_bounds_path_no_crash: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_out_of_bounds_path_stays_fast():
    import time

    field = CellField(20, 15)
    t0 = time.perf_counter()
    field.mark_path([(-1e9, -1e9), (1e9, 1e9)])
    elapsed = time.perf_counter() - t0

    budget = 1.0
    ok = elapsed <= budget
    print(f"test_out_of_bounds_path_stays_fast: elapsed={elapsed*1000:.3f}ms "
          f"(budget={budget*1000:.0f}ms) -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_block_commands_simple():
    all_ok = True
    field = CellField(20, 20)

    path = [(2.0, 2.0), (2.0, 12.0), (7.0, 12.0), (7.0, 8.0), (13.0, 8.0)]
    commands = field.block_commands(path)
    expected = [("U", 10), ("R", 5), ("D", 4), ("R", 6)]
    if commands != expected:
        all_ok = False
        print(f"  MISMATCH: expected={expected} actual={commands}")

    print(f"test_block_commands_simple: -> {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_block_commands_diagonal_reconstructs_displacement():
    field = CellField(30, 30)
    path = [(1.0, 1.0), (20.0, 20.0)]
    commands = field.block_commands(path)

    net = {"U": 0, "D": 0, "L": 0, "R": 0}
    total_steps = 0
    for direction, count in commands:
        net[direction] += count
        total_steps += count

    dx = net["R"] - net["L"]
    dy = net["U"] - net["D"]

    start_cell = field.real_to_cell(*path[0])
    end_cell = field.real_to_cell(*path[1])
    expected_dx = end_cell[0] - start_cell[0]
    expected_dy = end_cell[1] - start_cell[1]

    only_cardinal = all(c in "UDLR" for c, _ in commands)
    correct_displacement = (dx, dy) == (expected_dx, expected_dy)
    # a pure 45-degree diagonal alternates single steps -- no run should
    # ever compress more than 1 in each direction here
    alternating = all(count == 1 for _, count in commands)

    ok = only_cardinal and correct_displacement and alternating
    print(f"test_block_commands_diagonal_reconstructs_displacement: "
          f"displacement=({dx},{dy}) expected=({expected_dx},{expected_dy}) "
          f"steps={total_steps} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_block_commands_out_of_bounds_gap_no_crash():
    field = CellField(20, 15)
    # goes from inside, way out, then back inside -- must not crash, and
    # should still produce commands for the in-bounds portions
    path = [(2.0, 2.0), (2.0, 8.0), (1000.0, 1000.0), (15.0, 3.0), (15.0, 10.0)]

    try:
        commands = field.block_commands(path)
        raised = False
    except Exception as e:
        commands = []
        raised = True
        print(f"  raised: {e!r}")

    ok = not raised and len(commands) > 0 and all(c in "UDLR" for c, _ in commands)
    print(f"test_block_commands_out_of_bounds_gap_no_crash: commands={commands} -> {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    results = [
        test_get_set_roundtrip(),
        test_clear_all(),
        test_vertical_slice_correctness(),
        test_vertical_slice_index_matches_fraction(),
        test_vertical_slice_corners_and_coordinates(),
        test_vertical_slice_invalid_raises(),
        test_vertical_slice_reasonably_fast_for_huge_field(),
        test_cover_with_shape_single_placement_suffices(),
        test_cover_with_shape_multiple_placements(),
        test_cover_with_shape_empty_target(),
        test_cover_with_shape_no_shape_cells_raises(),
        test_cover_with_shape_center_shifts_result_correctly(),
        test_cover_with_shape_offfield_center_falls_back_to_cutoff_center(),
        test_cover_with_shape_rect_tuple_matches_equivalent_cellfield(),
        test_cover_with_shape_rect_tuple_covers_everything(),
        test_cover_with_shape_rect_tuple_default_center(),
        test_cover_with_shape_rect_tuple_invalid_raises(),
        test_cover_with_shape_perfect_tiling_has_zero_overlap(),
        test_cover_with_shape_overlap_tiebreak_reduces_overlap_vs_no_tiebreak(),
        test_bitwise_ops(),
        test_shift_correctness(),
        test_shift_dx_exceeds_buffer_raises(),
        test_expand_correctness(),
        test_expand_empty_and_full_field(),
        test_expand_requires_buffer_raises(),
        test_inplace_vs_functional_identity(),
        test_to_from_bytes_roundtrip(),
        test_on_cells_matches_naive_scan(),
        test_mismatched_shape_raises(),
        test_large_scale_sanity(),
        test_fill_rect(),
        test_fill_disk(),
        test_fill_polygon_covered_axis_aligned_rect(),
        test_fill_polygon_covered_offset_rect(),
        test_fill_polygon_covered_excludes_partially_clipped_cells(),
        test_fill_polygon_covered_rotated(),
        test_fill_polygon_covered_invalid_raises(),
        test_fill_polygon_touched_axis_aligned_rect(),
        test_fill_polygon_touched_includes_clipped_border_but_not_diagonal_neighbors(),
        test_fill_polygon_touched_single_cell_no_diagonal_bleed(),
        test_fill_polygon_touched_covers_more_than_fill_polygon_covered(),
        test_fill_polygon_touched_invalid_raises(),
        test_fill_aligned_rect_matches_polygon_equivalents(),
        test_fill_aligned_rect_reversed_bounds(),
        test_fill_aligned_rect_faster_than_polygon(),
        test_fill_disk_faster_than_per_cell_set(),
        test_apply_mask(),
        test_apply_mask_cost_independent_of_field_size(),
        test_real_cell_roundtrip(),
        test_mark_path_dense_sampling(),
        test_mark_path_exact_corner_ties(),
        test_from_path_factory(),
        test_mismatched_corners_raises(),
        test_out_of_bounds_path_no_crash(),
        test_out_of_bounds_path_stays_fast(),
        test_block_commands_simple(),
        test_block_commands_diagonal_reconstructs_displacement(),
        test_block_commands_out_of_bounds_gap_no_crash(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
