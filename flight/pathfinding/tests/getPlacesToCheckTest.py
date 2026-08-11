"""
End-to-end test for Pathfinder.getPlacesToCheck: builds an 80ft x 300ft
field for a single drone (droneID=1), builds the node field,
adds discovered mines, then runs getPlacesToCheck (both its default
method="path", which walks the shortest path's own geometry via
path_cover.py, and method="cellgrid", the original cover_with_shape-based
approach kept available for any future arbitrary-region use case) and
checks the results. Also renders one combined diagnostic image: the drone's
path footprint, the discovered mines, the default method's shape
placements with their centers and the outputted route between them, and
just the nodeField's shortest-path connection (not its full potential-paths
graph) overlaid on the same real-world axes.

flight.pathfinder used to require stubbing flight.pathfinding.path_subdivision
and flight.pathfinding.utils.seen_by_drone in sys.modules to import at all
(both imported dead names -- Node/Mine/Field/Connection/seg -- from
nodeField's empty __init__.py). That whole chain (including
utils/mask_gen.py's own broken import) has since been fixed to import from
the classes' real current locations, so flight.pathfinder now imports
directly, no stubbing needed.
"""

import math
import random

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Polygon as MplPolygon
import numpy as np

from flight.pathfinder import Pathfinder, WIDTHOFFIELD, HEIGHTOFFIELD, WIDTHOFSQUARE
from flight.pathfinding.path_cover import (
    path_cover,
    path_cover_unseen,
    unseen_path_runs,
    _polyline_normals_per_vertex,
)

SCRATCH_DIR = (
    r"C:\Users\harpe\AppData\Local\Temp\claude\c--Users-harpe-Multirotor-IARC-10"
    r"\2567942b-0ada-473d-af3b-214803e7410d\scratchpad"
)

# A field-sized rectangle of lat/lon corners, built from a base point purely
# by offsetting in meters -- WIDTHOFFIELD/HEIGHTOFFIELD (80ft x 300ft) are
# Pathfinder's own hardcoded field-size constants, so this is the field size
# the rest of Pathfinder actually assumes regardless of what's passed in.
_BASE_LAT, _BASE_LON = 36.0, -95.9
_M_PER_LAT = 111320.0
_M_PER_LON = 111320.0 * math.cos(math.radians(_BASE_LAT))
_FT_TO_M = 0.3048


def _field_corners():
    width_m = WIDTHOFFIELD * _FT_TO_M
    height_m = HEIGHTOFFIELD * _FT_TO_M
    c1 = (_BASE_LAT, _BASE_LON)  # origin
    c2 = (_BASE_LAT, _BASE_LON + width_m / _M_PER_LON)  # +x / width corner
    c3 = (_BASE_LAT + height_m / _M_PER_LAT, _BASE_LON + width_m / _M_PER_LON)  # diagonal
    c4 = (_BASE_LAT + height_m / _M_PER_LAT, _BASE_LON)  # +y / height corner
    return (c1, c2, c3, c4)


def build_pathfinder():
    pf = Pathfinder(_field_corners(), altitude=20.0, fov_deg=60.0, droneID=1)
    start_latlon = pf.coord_converter.local_to_latlon(WIDTHOFFIELD / 2, -1)
    pf.buildNodeField(start_latlon)
    return pf


# Local (x, y) positions -- feet -- for the mines this test adds: a large,
# randomly scattered field rather than a hand-placed handful. Overlapping
# detections (which merge into a unionObstacle) and detections close enough
# to the field's edge that their safety-zone polygon hangs off it are both
# fine and left in on purpose, not avoided -- that's exactly the kind of
# case a real minefield can produce and the connection logic needs to
# handle. Positions stay within (0, WIDTHOFFIELD) x (0, HEIGHTOFFIELD)
# themselves though (open interval): add_discovered_mine's own square
# lookup raises ValueError for a DETECTED point that's actually outside the
# field, which is a different thing from its resulting safety zone
# extending past the edge.
NUM_MINES = 70
_MINE_RNG_SEED = 20240607


def _generate_mine_positions(n=NUM_MINES, seed=_MINE_RNG_SEED):
    rng = random.Random(seed)
    return [
        (rng.uniform(0.5, WIDTHOFFIELD - 0.5), rng.uniform(0.5, HEIGHTOFFIELD - 0.5))
        for _ in range(n)
    ]


MINE_LOCAL_POSITIONS = _generate_mine_positions()


def add_test_mines(pf):
    for x, y in MINE_LOCAL_POSITIONS:
        lat, lon = pf.coord_converter.local_to_latlon(x, y)
        pf.add_discovered_mine(lat, lon)


def test_mines_land_near_their_detected_square(pf):
    # Overlapping detections are allowed and expected to merge (into a
    # unionObstacle, or even collapse two detections landing in the exact
    # same square into one live mine), so this doesn't assume a 1:1 count --
    # just that every requested position has SOME real, live mine (standalone
    # or nested in a union, collected the same way Field.mineHash does) near
    # where it was detected.
    all_mines = list(pf.nodeField.mines) + pf.nodeField._collect_mines(pf.nodeField.unionObstacles)
    ok = len(all_mines) > 0
    for x, y in MINE_LOCAL_POSITIONS:
        dist = min(math.hypot(m.origin[0] - x, m.origin[1] - y) for m in all_mines)
        # Mines snap to their competition square's center (WIDTHOFSQUARE=2ft
        # squares), so the nearest real mine should land within one
        # square-diagonal of the detected point, not just "somewhere".
        ok = ok and dist <= WIDTHOFSQUARE * math.sqrt(2)

    print(
        f"test_mines_land_near_their_detected_square: "
        f"requested={len(MINE_LOCAL_POSITIONS)} placed={len(all_mines)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_shortest_path_connects_start_to_end(pf):
    path = pf.get_shortest_path()

    ok = len(path) >= 2 and path[0] in pf.startingNodes and path[-1] in pf.endingNodes
    print(
        f"test_shortest_path_connects_start_to_end: nodes={len(path)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_get_places_to_check_returns_points(pf):
    ok = True
    for method in ("path", "cellgrid"):
        latLonPoints = pf.getPlacesToCheck(method=method)
        method_ok = len(latLonPoints) > 0
        for lat, lon in latLonPoints:
            method_ok = method_ok and isinstance(lat, float) and isinstance(lon, float)
            # Round-trip consistency: converting back to local and forward again
            # should reproduce the same lat/lon (independent check that doesn't
            # trust getPlacesToCheck's own arithmetic).
            x, y = pf.coord_converter.latlon_to_local(lat, lon)
            lat2, lon2 = pf.coord_converter.local_to_latlon(x, y)
            method_ok = method_ok and abs(lat2 - lat) < 1e-9 and abs(lon2 - lon) < 1e-9
        print(f"  method={method}: points={len(latLonPoints)} -> {'PASS' if method_ok else 'FAIL'}")
        ok = ok and method_ok
    print(f"test_get_places_to_check_returns_points: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_get_places_to_check_matches_expected_local_points(pf):
    """Independently reconstructs the expected local (x, y) points -- the
    same path_cover + order_waypoints steps getPlacesToCheck's default
    "path" method uses internally -- and converts them to lat/lon with NO
    extra scaling. Catches the double-scaling bug getPlacesToCheck used to
    have (multiplying an already-real-world-feet point by WIDTHOFSQUARE
    again, which doubled every point's distance from the origin -- e.g. the
    field's true center (40,150) was being converted as if it were (80,300),
    its far corner)."""
    path = pf.get_shortest_path()
    path_points = [(n.x, n.y) for n in path]
    matSizeCells = max(1, round(pf.matSize / WIDTHOFSQUARE))
    shape_size_ft = matSizeCells * WIDTHOFSQUARE
    shapesToVisit = path_cover_unseen(
        path_points,
        pf.seen_tracker,
        shape_size_ft,
    )
    # getPlacesToCheck anchors the tour to whatever it left as
    # nextPlaceToCheckLocal on its own PREVIOUS call -- must pass that same
    # value here (read before the getPlacesToCheck call below updates it)
    # to reconstruct the exact same tour, not just the free-choice "no
    # fixed_first" ordering.
    orderedShapes = pf.order_waypoints(shapesToVisit, fixed_first=pf.nextPlaceToCheckLocal)
    expected = [tuple(pf.coord_converter.local_to_latlon(x, y)) for x, y in orderedShapes]

    actual = pf.getPlacesToCheck(method="path")

    ok = len(actual) == len(expected)
    for (alat, alon), (elat, elon) in zip(actual, expected):
        ok = ok and abs(alat - elat) < 1e-9 and abs(alon - elon) < 1e-9
    print(
        f"test_get_places_to_check_matches_expected_local_points: "
        f"points={len(actual)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def _shapes_to_visit_cellgrid(pf, path):
    """Reconstructs the same intermediate values getPlacesToCheck's
    method="cellgrid" computes internally (ourPortion, matSizeCells,
    ShapesToVisit) without needing it to expose them -- used by both the
    coverage-correctness check and the diagram renderer below."""
    ourPortion = pf.rasterize_node_path(path)
    matSizeCells = max(1, round(pf.matSize / WIDTHOFSQUARE))
    shapesToVisit = ourPortion.cover_with_shape((matSizeCells, matSizeCells))
    return ourPortion, matSizeCells, shapesToVisit


def _shapes_to_visit_path(pf, path):
    """Same role as _shapes_to_visit_cellgrid, but for the default
    method="path" -- walks the shortest path's own geometry via
    path_cover_unseen (respecting pf.seen_tracker, matching
    getPlacesToCheck itself) instead of rasterizing to a CellField."""
    path_points = [(n.x, n.y) for n in path]
    matSizeCells = max(1, round(pf.matSize / WIDTHOFSQUARE))
    shape_size_ft = matSizeCells * WIDTHOFSQUARE
    shapesToVisit = path_cover_unseen(
        path_points,
        pf.seen_tracker,
        shape_size_ft,
    )
    return shape_size_ft, shapesToVisit


def test_shapes_cover_the_path_footprint(pf):
    path = pf.get_shortest_path()
    ourPortion, matSizeCells, shapesToVisit = _shapes_to_visit_cellgrid(pf, path)

    shape_center = (matSizeCells / 2.0, matSizeCells / 2.0)
    covered = set()
    for cx, cy in shapesToVisit:
        ox = round((cx - ourPortion.min_corner[0]) / ourPortion.cell_size[0] - shape_center[0])
        oy = round((cy - ourPortion.min_corner[1]) / ourPortion.cell_size[1] - shape_center[1])
        for sx in range(matSizeCells):
            for sy in range(matSizeCells):
                covered.add((sx + ox, sy + oy))

    target = set(ourPortion.on_cells())
    ok = target.issubset(covered) and len(shapesToVisit) > 0
    print(
        f"test_shapes_cover_the_path_footprint (cellgrid): target_cells={len(target)} "
        f"placements={len(shapesToVisit)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_path_cover_covers_the_shortest_path(pf):
    """Independent coverage check for the default "path" method: densely
    samples the in-field portion of the shortest path and confirms every
    sample lands inside at least one placed square."""
    path = pf.get_shortest_path()
    path_points = [(n.x, n.y) for n in path]
    shape_size_ft, shapesToVisit = _shapes_to_visit_path(pf, path)

    samples = []
    for i in range(len(path_points) - 1):
        p0, p1 = path_points[i], path_points[i + 1]
        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(1, int(seg_len / 0.5))
        for k in range(n + 1):
            t = k / n
            samples.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))
    in_field = [
        (x, y) for x, y in samples if 0.0 <= x <= WIDTHOFFIELD and 0.0 <= y <= HEIGHTOFFIELD
    ]

    half = shape_size_ft / 2.0

    def is_covered(pt):
        x, y = pt
        return any(abs(x - cx) <= half and abs(y - cy) <= half for cx, cy in shapesToVisit)

    uncovered = [s for s in in_field if not is_covered(s)]
    ok = len(uncovered) == 0 and len(shapesToVisit) > 0
    print(
        f"test_path_cover_covers_the_shortest_path: samples={len(in_field)} "
        f"uncovered={len(uncovered)} placements={len(shapesToVisit)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_next_place_to_check_stays_fixed_across_replan():
    """Uses its OWN fresh Pathfinder (not the shared `pf` other tests use)
    so this isn't tangled up in whatever nextPlaceToCheckLocal state other
    tests' getPlacesToCheck calls happen to leave behind. Verifies the
    actual point of tracking it: two back-to-back calls with nothing
    changed give the identical first stop (not just an equally-good but
    different one), and after that first stop gets marked seen (simulating
    having just visited/photographed it), the new first stop is anchored
    near where the drone just was -- not wherever a from-scratch "best
    overall tour" search happens to prefer instead, which could easily be
    clear across the field."""
    pf = build_pathfinder()
    add_test_mines(pf)

    first_call = pf.getPlacesToCheck(method="path")
    second_call = pf.getPlacesToCheck(method="path")
    unchanged_matches = len(first_call) > 1 and first_call == second_call

    lat, lon = second_call[0]
    x, y = pf.coord_converter.latlon_to_local(lat, lon)
    matSizeCells = max(1, round(pf.matSize / WIDTHOFSQUARE))
    half = (matSizeCells * WIDTHOFSQUARE) / 2.0
    corners_local = [
        (x - half, y - half),
        (x + half, y - half),
        (x + half, y + half),
        (x - half, y + half),
    ]
    corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
    pf.accept_image_corner_coord(corners_latlon)

    third_call = pf.getPlacesToCheck(method="path")
    moved_on = len(third_call) > 0 and third_call[0] != second_call[0]

    # Weak but real sanity bound (2-opt CAN still shuffle the tour right
    # after a fixed/anchor start, so this doesn't assert exact-nearest,
    # just "anchored near it, not the tour's own farthest remaining point"):
    remaining_local = [pf.coord_converter.latlon_to_local(*p) for p in second_call[1:]]
    dists = [math.hypot(px - x, py - y) for px, py in remaining_local]
    new_first_local = pf.coord_converter.latlon_to_local(*third_call[0]) if third_call else None
    anchored = (
        not remaining_local
        or new_first_local is None
        or math.hypot(new_first_local[0] - x, new_first_local[1] - y) <= max(dists)
    )

    ok = unchanged_matches and moved_on and anchored
    print(
        f"test_next_place_to_check_stays_fixed_across_replan: "
        f"unchanged_matches={unchanged_matches} moved_on={moved_on} anchored={anchored} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_path_cover_overlap_tightens_spacing(pf):
    """overlap=0.0 (default) vs overlap=0.3 for the same single-row path:
    more placements, still full coverage. Confirms the overlap knob does
    what it says without breaking the coverage guarantee."""
    path = pf.get_shortest_path()
    path_points = [(n.x, n.y) for n in path]
    matSizeCells = max(1, round(pf.matSize / WIDTHOFSQUARE))
    shape_size_ft = matSizeCells * WIDTHOFSQUARE

    baseline = path_cover(path_points, shape_size_ft)
    overlapped = path_cover(path_points, shape_size_ft, overlap=0.3)

    half = shape_size_ft / 2.0

    def is_covered(pt, centers):
        x, y = pt
        return any(abs(x - cx) <= half and abs(y - cy) <= half for cx, cy in centers)

    samples = []
    for i in range(len(path_points) - 1):
        p0, p1 = path_points[i], path_points[i + 1]
        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(1, int(seg_len / 0.5))
        for k in range(n + 1):
            t = k / n
            samples.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))
    in_field = [
        (x, y) for x, y in samples if 0.0 <= x <= WIDTHOFFIELD and 0.0 <= y <= HEIGHTOFFIELD
    ]
    uncovered = [s for s in in_field if not is_covered(s, overlapped)]

    ok = len(overlapped) > len(baseline) and len(uncovered) == 0
    print(
        f"test_path_cover_overlap_tightens_spacing: baseline={len(baseline)} "
        f"overlap=0.3->{len(overlapped)} uncovered={len(uncovered)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_path_cover_wide_path_uses_multiple_rows(pf):
    """path_width well beyond shape_size should use multiple parallel rows
    and cover the FULL corridor width, not just the centerline -- densely
    samples both along the path AND across the corridor (via the same
    per-vertex normals path_cover itself offsets by). Uses overlap=0.15,
    the documented recommendation for path_width > shape_size (see
    path_cover.py's module docstring for why overlap=0.0 there can leave
    razor-thin floating-point seams)."""
    path = pf.get_shortest_path()
    path_points = [(n.x, n.y) for n in path]
    matSizeCells = max(1, round(pf.matSize / WIDTHOFSQUARE))
    shape_size_ft = matSizeCells * WIDTHOFSQUARE
    path_width = shape_size_ft * 1.5

    centers = path_cover(
        path_points,
        shape_size_ft,
        overlap=0.15,
        path_width=path_width,
    )

    normals = _polyline_normals_per_vertex(path_points)
    samples = []
    for i in range(len(path_points) - 1):
        p0, p1 = path_points[i], path_points[i + 1]
        n0, n1 = normals[i], normals[i + 1]
        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n_along = max(1, int(seg_len / 1.0))
        for k in range(n_along + 1):
            t = k / n_along
            cx, cy = p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])
            nx, ny = n0[0] + t * (n1[0] - n0[0]), n0[1] + t * (n1[1] - n0[1])
            nlen = math.hypot(nx, ny)
            if nlen > 0:
                nx, ny = nx / nlen, ny / nlen
            for frac in (-0.45, -0.2, 0.0, 0.2, 0.45):
                off = frac * path_width
                samples.append((cx + off * nx, cy + off * ny))
    in_field = [
        (x, y) for x, y in samples if 0.0 <= x <= WIDTHOFFIELD and 0.0 <= y <= HEIGHTOFFIELD
    ]

    half = shape_size_ft / 2.0

    def is_covered(pt):
        x, y = pt
        return any(abs(x - cx) <= half and abs(y - cy) <= half for cx, cy in centers)

    uncovered = [s for s in in_field if not is_covered(s)]
    used_multiple_rows = len(centers) > len(
        path_cover(
            path_points,
            shape_size_ft,
            overlap=0.15,
        )
    )
    ok = used_multiple_rows and len(uncovered) == 0
    print(
        f"test_path_cover_wide_path_uses_multiple_rows: path_width={path_width:.1f} "
        f"shape_size={shape_size_ft:.1f} placements={len(centers)} "
        f"cross_width_samples={len(in_field)} uncovered={len(uncovered)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_accept_image_corner_coord_marks_seen_cells(pf):
    """accept_image_corner_coord (fixed to call CellField.fill_polygon_covered,
    since self.seen_tracker is a CellField now, not the old SightTracker it
    was written against) should mark exactly the cells FULLY inside the
    given image's corners as seen. Resets seen_tracker afterward so later
    tests aren't affected by this one's side effect (Pathfinder is built
    once and shared across every test function in this file)."""
    corners_local = [(10.0, 10.0), (30.0, 10.0), (30.0, 30.0), (10.0, 30.0)]
    corners_latlon = [pf.coord_converter.local_to_latlon(x, y) for x, y in corners_local]

    ok = pf.seen_tracker.count() == 0  # sanity: starts empty
    pf.accept_image_corner_coord(corners_latlon)

    inside_col, inside_row = pf.seen_tracker.real_to_cell(20.0, 20.0)
    outside_col, outside_row = pf.seen_tracker.real_to_cell(5.0, 5.0)
    ok = ok and pf.seen_tracker.get(inside_col, inside_row)
    ok = ok and not pf.seen_tracker.get(outside_col, outside_row)
    ok = ok and pf.seen_tracker.count() > 0

    pf.seen_tracker.clear_all()
    print(f"test_accept_image_corner_coord_marks_seen_cells: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_get_places_to_check_respects_seen_cells(pf):
    """End-to-end: mark a real chunk of the shortest path as already-seen
    via accept_image_corner_coord (the actual public API a caller would
    use), then confirm getPlacesToCheck's default method="path" returns
    FEWER placements than with nothing seen, and none of them land back
    inside the now-seen band -- proves the discontinuous-path support
    (path_cover_unseen) is wired all the way through Pathfinder, not just
    exercised at the path_cover_unseen level directly. Resets seen_tracker
    afterward."""
    baseline = pf.getPlacesToCheck(method="path")

    path = pf.get_shortest_path()
    ys = sorted(n.y for n in path)
    y_lo, y_hi = ys[len(ys) // 3], ys[2 * len(ys) // 3]
    corners_local = [(0.0, y_lo), (WIDTHOFFIELD, y_lo), (WIDTHOFFIELD, y_hi), (0.0, y_hi)]
    corners_latlon = [pf.coord_converter.local_to_latlon(x, y) for x, y in corners_local]
    pf.accept_image_corner_coord(corners_latlon)

    after = pf.getPlacesToCheck(method="path")

    def in_seen_band(latlon):
        # Cell-based, not a raw y_lo<=y<=y_hi compare: a placement sitting
        # in a cell that straddles the band boundary is legitimately still
        # unseen (fill_polygon_covered only marks a cell seen when the
        # WHOLE cell is inside the photographed rectangle, the same rule
        # every other "seen" check in this codebase uses) even though its
        # exact (x, y) can land a hair inside [y_lo, y_hi] numerically --
        # a real path vertex right at the boundary is exactly this case.
        x, y = pf.coord_converter.latlon_to_local(*latlon)
        col, row = pf.seen_tracker.real_to_cell(x, y)
        if not (0 <= col < pf.seen_tracker.width and 0 <= row < pf.seen_tracker.height):
            return False
        return pf.seen_tracker.get(col, row)

    none_in_seen_band = all(not in_seen_band(p) for p in after)
    ok = len(after) < len(baseline) and none_in_seen_band and pf.seen_tracker.count() > 0

    pf.seen_tracker.clear_all()
    print(
        f"test_get_places_to_check_respects_seen_cells: baseline={len(baseline)} "
        f"after_seen={len(after)} none_in_seen_band={none_in_seen_band} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def render_combined_image(pf, save_path):
    """
    One image, overlaying:
      - the drone's path footprint (black cells -- rasterized purely for
        this background visual, not used to compute the shapes below)
      - the discovered mines, as their actual shaded safety-zone polygons
        (red) -- overlapping/merged and off-field-edge mines included
      - the DEFAULT method="path" placements ("shapes to check", blue
        squares, computed via path_cover -- see getPlacesToCheck) and their
        centers (orange dots)
      - the outputted route between those centers (orange line) -- the
        actual order_waypoints result getPlacesToCheck would fly
      - JUST the nodeField shortest-path connection (magenta line) -- not
        the full potential-paths graph Field.plotField would otherwise draw
    """
    path = pf.get_shortest_path()
    shape_side_ft, shapesToVisit = _shapes_to_visit_path(pf, path)
    orderedShapes = pf.order_waypoints(shapesToVisit)

    # Background rasterization purely for the visual -- not part of the
    # method="path" computation itself, which never touches a cell grid.
    background = pf.rasterize_node_path(path)
    width_cells, height_cells = background.width, background.height
    arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in background.on_cells():
        arr[y, x] = 1

    fig, ax = plt.subplots(figsize=(8, 20), dpi=100)
    ax.imshow(
        arr,
        cmap="Greys",
        vmin=0,
        vmax=1,
        interpolation="nearest",
        origin="lower",
        alpha=0.9,
        extent=(0, WIDTHOFFIELD, 0, HEIGHTOFFIELD),
    )

    # Mines are drawn as their actual safety-zone polygon, not a point
    # marker -- includes merged unionObstacles (overlapping mines are
    # allowed and expected to merge), each still exposing .vertices the same
    # way a standalone BlockMine does.
    for obstacle in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles):
        ax.add_patch(
            MplPolygon(
                list(obstacle.vertices),
                closed=True,
                facecolor="firebrick",
                edgecolor="darkred",
                alpha=0.4,
                linewidth=0.7,
                zorder=4,
            )
        )

    for cx, cy in shapesToVisit:
        llx, lly = cx - shape_side_ft / 2.0, cy - shape_side_ft / 2.0
        ax.add_patch(
            Rectangle(
                (llx, lly),
                shape_side_ft,
                shape_side_ft,
                facecolor="tab:blue",
                edgecolor="tab:blue",
                alpha=0.2,
                linewidth=0.8,
                zorder=2,
            )
        )
        ax.add_patch(
            Rectangle(
                (llx, lly),
                shape_side_ft,
                shape_side_ft,
                facecolor="none",
                edgecolor="tab:blue",
                alpha=0.7,
                linewidth=0.8,
                zorder=3,
            )
        )

    # the outputted route between shape centers, drawn under the centers
    # themselves so the dots stay visible on top of it
    oxs = [c[0] for c in orderedShapes]
    oys = [c[1] for c in orderedShapes]
    ax.plot(oxs, oys, color="tab:orange", linewidth=1.1, zorder=5)
    sxs = [c[0] for c in shapesToVisit]
    sys_ = [c[1] for c in shapesToVisit]
    ax.scatter(sxs, sys_, color="tab:orange", s=16, zorder=6, edgecolors="white", linewidths=0.4)

    # ONLY the shortest-path connection itself -- not Field.plotField's full
    # potential-paths graph
    pxs = [n.x for n in path]
    pys = [n.y for n in path]
    ax.plot(pxs, pys, color="magenta", linewidth=2.2, zorder=7)

    ax.set_xlim(0, WIDTHOFFIELD)
    ax.set_ylim(0, HEIGHTOFFIELD)
    ax.set_aspect("equal")
    ax.set_title(
        f'getPlacesToCheck method="path" (droneID=1)\n'
        f"{len(shapesToVisit)} shapes to check, {shape_side_ft:.1f}ft square each",
        fontsize=11,
    )
    ax.set_xlabel(
        "Black = flight-path footprint   |   Red = mine safety zones   |   Magenta = nodeField shortest path\n"
        "Blue squares = camera footprints to check   |   Orange dots/line = shape centers + outputted route",
        fontsize=9,
    )
    ax.set_xticks([])
    ax.set_yticks([])

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)


def main():
    # Built ONCE and reused by every check below (each one only reads the
    # field or does its own idempotent recompute -- get_shortest_path,
    # cover_with_shape, order_waypoints etc. don't mutate nodeField/mines) --
    # re-adding 70 mines from scratch per check would dominate the whole
    # run for no reason: the mine-connection algorithms (shapely-based
    # occlusion/tangent checks) scale with graph size, so repeating setup
    # 5x doesn't just cost 5x the mine-adds, it costs noticeably more.
    pf = build_pathfinder()
    add_test_mines(pf)

    results = [
        test_mines_land_near_their_detected_square(pf),
        test_shortest_path_connects_start_to_end(pf),
        test_get_places_to_check_returns_points(pf),
        test_get_places_to_check_matches_expected_local_points(pf),
        test_next_place_to_check_stays_fixed_across_replan(),
        test_shapes_cover_the_path_footprint(pf),
        test_path_cover_covers_the_shortest_path(pf),
        test_path_cover_overlap_tightens_spacing(pf),
        test_path_cover_wide_path_uses_multiple_rows(pf),
        test_accept_image_corner_coord_marks_seen_cells(pf),
        test_get_places_to_check_respects_seen_cells(pf),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")

    combined_path = SCRATCH_DIR + r"\getPlacesToCheck_combined.png"
    render_combined_image(pf, combined_path)
    print(f"saved combined image to {combined_path}")


if __name__ == "__main__":
    main()
