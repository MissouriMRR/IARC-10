"""
Integration test for CellField.cover_with_shape + pathfinder.order_waypoints
used together: cover a field's "on" cells with a small shape, then order the
resulting placement centers into a decent visiting route. Also renders a
diagram of the whole pipeline (target cells, shape footprints, ordered
route) for a visual sanity check, not just numeric assertions.

flight/pathfinder.py transitively imports flight/pathfinding/path_subdivision.py
and flight/pathfinding/utils/seen_by_drone.py, both of which import names
from flight.pathfinding.nodeField's __init__.py that it doesn't actually
export (the same pre-existing, out-of-scope break used in tspOrderTest.py).
Stubbing those two modules in sys.modules lets us import order_waypoints
without either fixing or being blocked by that unrelated breakage.
"""
import sys
import types
import random

for _name in ("flight.pathfinding.path_subdivision", "flight.pathfinding.utils.seen_by_drone"):
    stub = types.ModuleType(_name)
    stub.Path = object
    stub.SightTracker = object
    stub.remove_extra_coords = lambda *a, **k: None
    sys.modules[_name] = stub

from flight.pathfinder import order_waypoints, path_length
from flight.pathfinding.cellField.cellField import CellField

WIDTH, HEIGHT = 200, 150
SHAPE_SIZE = (2, 2)  # plain (width, height) rectangle -- cover_with_shape's tuple input
SHAPE_CENTER = (SHAPE_SIZE[0] / 2.0, SHAPE_SIZE[1] / 2.0)
NUM_BLOBS = 30
MIN_BLOB_SPACING = 14


def build_target_field(seed=42):
    """Many scattered blobs -- e.g. detected areas that need coverage --
    placed via rejection sampling so they stay a minimum distance apart."""
    field = CellField(WIDTH, HEIGHT)
    rng = random.Random(seed)
    margin = 10
    centers = []
    attempts = 0
    while len(centers) < NUM_BLOBS and attempts < NUM_BLOBS * 200:
        attempts += 1
        cx = rng.randint(margin, WIDTH - margin)
        cy = rng.randint(margin, HEIGHT - margin)
        if all((cx - ox) ** 2 + (cy - oy) ** 2 >= MIN_BLOB_SPACING ** 2 for ox, oy in centers):
            centers.append((cx, cy))
    for cx, cy in centers:
        r = rng.randint(1, 3)
        field.fill_disk(cx, cy, r)
    return field


def reconstruct_coverage(field, shape_size, shape_center, centers):
    """Inverts cover_with_shape's center math to recover each placement's
    cell offset, then unions what the shape covers from there -- verifies
    the returned centers independently, without trusting cover_with_shape's
    own bookkeeping."""
    w, h = shape_size
    shape_cells = [(x, y) for x in range(w) for y in range(h)]
    covered = set()
    for cx, cy in centers:
        ox = (cx - field.min_corner[0]) / field.cell_size[0] - shape_center[0]
        oy = (cy - field.min_corner[1]) / field.cell_size[1] - shape_center[1]
        ox, oy = round(ox), round(oy)
        for sx, sy in shape_cells:
            covered.add((sx + ox, sy + oy))
    return covered


def test_cover_then_order_covers_everything():
    field = build_target_field()
    centers = field.cover_with_shape(SHAPE_SIZE, shape_center=SHAPE_CENTER)
    covered = reconstruct_coverage(field, SHAPE_SIZE, SHAPE_CENTER, centers)
    target_cells = set(field.on_cells())

    fully_covered = target_cells.issubset(covered)
    ok = fully_covered and len(centers) > 0
    print(f"test_cover_then_order_covers_everything: placements={len(centers)} "
          f"fully_covered={fully_covered} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_ordered_route_is_permutation_of_placements():
    field = build_target_field()
    centers = field.cover_with_shape(SHAPE_SIZE, shape_center=SHAPE_CENTER)
    ordered = order_waypoints(centers)

    ok = sorted(ordered) == sorted(centers) and len(ordered) == len(centers)
    print(f"test_ordered_route_is_permutation_of_placements: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_ordering_does_not_worsen_route_length():
    field = build_target_field()
    centers = field.cover_with_shape(SHAPE_SIZE, shape_center=SHAPE_CENTER)
    ordered = order_waypoints(centers)

    ok = path_length(ordered) <= path_length(centers) + 1e-6
    print(f"test_ordering_does_not_worsen_route_length: input_order={path_length(centers):.2f} "
          f"ordered={path_length(ordered):.2f} -> {'PASS' if ok else 'FAIL'}")
    return ok


def render_diagram(save_path):
    import numpy as np
    from matplotlib import pyplot as plt
    from matplotlib.patches import Rectangle

    field = build_target_field()
    centers = field.cover_with_shape(SHAPE_SIZE, shape_center=SHAPE_CENTER)
    ordered = order_waypoints(centers)

    arr = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    for x, y in field.on_cells():
        arr[y, x] = 1

    fig, ax = plt.subplots(figsize=(16, 12), dpi=100)
    ax.imshow(arr, cmap="Greys", vmin=0, vmax=1, interpolation="nearest",
              origin="lower", alpha=0.9, extent=(0, WIDTH, 0, HEIGHT))

    w, h = SHAPE_SIZE
    for cx, cy in ordered:
        # placement's lower-left corner, in real-world units (cell_size=1 here)
        llx, lly = cx - SHAPE_CENTER[0], cy - SHAPE_CENTER[1]
        ax.add_patch(Rectangle((llx, lly), w, h, facecolor="tab:blue",
                                edgecolor="tab:blue", alpha=0.25, linewidth=0.8))
        ax.add_patch(Rectangle((llx, lly), w, h, facecolor="none",
                                edgecolor="tab:blue", alpha=0.7, linewidth=0.8))

    xs = [c[0] for c in ordered]
    ys = [c[1] for c in ordered]
    ax.plot(xs, ys, color="tab:orange", linewidth=1.0, zorder=5, alpha=0.85)

    ax.set_xlim(0, WIDTH)
    ax.set_ylim(0, HEIGHT)
    ax.set_aspect("equal")
    ax.set_title(
        f"cover_with_shape((2, 2)) + order_waypoints\n"
        f"{len(ordered)} placements, route length {path_length(ordered):.1f} units",
        fontsize=13,
    )
    ax.set_xlabel("Black = target cells needing coverage   |   Blue rectangles = 2x2 shape placements   "
                  "|   Orange = visiting order", fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)


def main():
    results = [
        test_cover_then_order_covers_everything(),
        test_ordered_route_is_permutation_of_placements(),
        test_ordering_does_not_worsen_route_length(),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
