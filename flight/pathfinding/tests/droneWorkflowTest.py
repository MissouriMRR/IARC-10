"""
End-to-end simulation of a single drone's real mission workflow -- not just
calling getPlacesToCheck once on a pre-loaded minefield (see
getPlacesToCheckTest.py for that), but the actual discover-as-you-fly loop:

  1. A ground-truth minefield exists, but the drone starts knowing NOTHING
     about it -- the minefield is generated and kept ONLY in this test
     file, never handed to the Pathfinder up front.
  2. Build an empty Pathfinder (buildNodeField only, zero mines known).
  3. Ask it for places to check, in order.
  4. "Visit" each one: if a not-yet-discovered true mine is under that
     photo's footprint, add it (add_discovered_mine) and mark the photo
     seen (accept_image_corner_coord) -- then STOP visiting this queue and
     replan from scratch, since a newly discovered mine can change the
     node graph and so the shortest path itself. If no mine is there,
     just mark it seen and move on to the next queued place.
  5. Repeat until getPlacesToCheck returns nothing left to check, which
     happens exactly when the current shortest path's whole cell footprint
     is already marked seen (the two termination conditions the task
     describes -- "seen graph == [path] field graph" and "queue is
     empty" -- are the same condition; path_cover_unseen returns [] the
     moment they're equal).

This is also the first real exercise of a genuinely empty Pathfinder (zero
mines/obstacles anywhere): doing so surfaced a real bug in Field itself --
see the addFloatingNode fix in nodeField/field.py this test depends on.
"""
import math
import random

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Polygon as MplPolygon

from flight.pathfinder import Pathfinder, WIDTHOFFIELD, HEIGHTOFFIELD, WIDTHOFSQUARE

SCRATCH_DIR = (
    r"C:\Users\harpe\AppData\Local\Temp\claude\c--Users-harpe-Multirotor-IARC-10"
    r"\2567942b-0ada-473d-af3b-214803e7410d\scratchpad"
)

_BASE_LAT, _BASE_LON = 36.0, -95.9
_M_PER_LAT = 111320.0
_M_PER_LON = 111320.0 * math.cos(math.radians(_BASE_LAT))
_FT_TO_M = 0.3048


def _field_corners():
    width_m = WIDTHOFFIELD * _FT_TO_M
    height_m = HEIGHTOFFIELD * _FT_TO_M
    c1 = (_BASE_LAT, _BASE_LON)
    c2 = (_BASE_LAT, _BASE_LON + width_m / _M_PER_LON)
    c3 = (_BASE_LAT + height_m / _M_PER_LAT, _BASE_LON + width_m / _M_PER_LON)
    c4 = (_BASE_LAT + height_m / _M_PER_LAT, _BASE_LON)
    return (c1, c2, c3, c4)


def build_empty_pathfinder():
    """An empty Pathfinder: node field built (start/end floating nodes
    exist), but not a single mine has been discovered yet."""
    pf = Pathfinder(_field_corners(), altitude=20.0, fov_deg=60.0, droneID=1, numOfDrones=1)
    pf.buildNodeField()
    return pf


# Ground-truth minefield -- kept ONLY here, in this test. The Pathfinder
# never sees this list directly; it only learns of a mine when the
# simulated flight below actually photographs it.
TRUE_MINEFIELD_SEED = 909090
NUM_TRUE_MINES = 70


def generate_true_minefield(n=NUM_TRUE_MINES, seed=TRUE_MINEFIELD_SEED):
    # rng.uniform gives continuous real-world positions, not snapped to any
    # square-center grid -- every mine here lands slightly off whatever
    # competition square it falls in (the true position vs. the square
    # center add_discovered_mine snaps it to are essentially never equal),
    # which is deliberate: it's what actually stresses the
    # polygon-vs-safety-radius-blocks discrepancy the diagram shows, rather
    # than only ever exercising the tidy square-centered case.
    rng = random.Random(seed)
    return [
        (rng.uniform(6.0, WIDTHOFFIELD - 6.0), rng.uniform(6.0, HEIGHTOFFIELD - 6.0))
        for _ in range(n)
    ]


def mines_under_footprint(true_mines, discovered, cx, cy, half_x, half_y):
    """Which true mines (not already discovered) fall under an axis-aligned
    2*half_x by 2*half_y rectangle centered at (cx, cy)."""
    return [
        (mx, my) for mx, my in true_mines
        if (mx, my) not in discovered and abs(mx - cx) <= half_x and abs(my - cy) <= half_y
    ]


def _snapshot_frame(pf, replans, discovered, visited):
    """A cheap copy of everything render_workflow_gif needs to redraw the
    field as of right now -- CellField.copy() is O(field size) but small
    here, and the mine polygons are copied out as plain vertex-tuple lists
    (not references to the live obstacles, which union merges can later
    replace/mutate) so an earlier frame can't retroactively change once a
    later replan alters the node graph."""
    return {
        "replan": replans,
        "seen": pf.seen_tracker.copy(),
        "mine_blocks": pf.mineFieldTracker.copy(),
        "polygons": [list(o.vertices) for o in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles)],
        "path_nodes": [(n.x, n.y) for n in pf.get_shortest_path()],
        "discovered": dict(discovered),
        "visited": list(visited),
    }


def simulate_one_drone(
    pf, true_mines, overlap=0.1, path_width=0.0, shape_size_ft=None,
    max_replans=50, max_waypoints=500, record_frames=False,
):
    """
    Runs the actual discover-replan loop described in this file's module
    docstring. Returns a dict of everything the tests/diagram below need:
    which true mines got discovered (and in which replan generation, for
    the visualization), every waypoint actually visited (in flight order),
    and whether the loop terminated cleanly (getPlacesToCheck ran dry) or
    hit one of the safety caps (which would indicate a bug -- a correct
    run can't loop more than len(true_mines)+1 times, GIVEN overlap > 0).

    overlap defaults to 0.1, not 0.0: accept_image_corner_coord's "seen"
    tracking (CellField.fill_polygon_covered) only marks a cell seen once
    the WHOLE cell -- not just the path line through it -- is inside a
    photographed footprint, so a cell sitting exactly on the zero-margin
    seam between two overlap=0.0 placements can end up fully enclosed by
    NEITHER. That's not incorrect (the loop still converges -- an
    unresolved seam cell just becomes its own tiny run on the next
    replan), but it costs extra pure-cleanup replanning passes that finding
    zero new mines: verified empirically, overlap=0.0 needed 5 replans for
    3 real discoveries on this field (2 wasted on seam cleanup) vs exactly
    4 (3 discoveries + 1 final empty check) at overlap=0.1. See
    path_cover.py's module docstring for the full writeup.

    shape_size_ft: None (default) reproduces the original matSize-derived
    square footprint; otherwise a plain number (square) or an
    (along, across) pair -- `along` is the dimension parallel to the
    direction of travel (mostly +y across this field's north-south path),
    `across` is perpendicular to it (mostly +x) -- passed straight through
    to Pathfinder.getPlacesToCheck, and used here too so the simulated
    camera footprint used for mine discovery/accept_image_corner_coord
    matches exactly what getPlacesToCheck itself just planned for.

    record_frames: if True, appends a state snapshot (see _snapshot_frame)
    to the returned "frames" list after every replan pass, for
    render_workflow_gif to animate. Off by default since it's extra work
    the plain pass/fail tests below don't need.
    """
    if shape_size_ft is None:
        square = max(1, round(pf.matSize / WIDTHOFSQUARE)) * WIDTHOFSQUARE
        shape_along_ft, shape_across_ft = square, square
    elif isinstance(shape_size_ft, tuple):
        shape_along_ft, shape_across_ft = shape_size_ft
    else:
        shape_along_ft, shape_across_ft = shape_size_ft, shape_size_ft
    half_along, half_across = shape_along_ft / 2.0, shape_across_ft / 2.0

    discovered = {}  # (x, y) -> replan generation it was found in
    visited = []  # (x, y, replan_generation) in actual flight order
    frames = []
    replans = 0
    total_waypoints = 0
    hit_cap = False

    while True:
        if replans >= max_replans:
            hit_cap = True
            break
        places = pf.getPlacesToCheck(overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft)
        if not places:
            break
        replans += 1

        found_mine_this_pass = False
        for lat, lon in places:
            if total_waypoints >= max_waypoints:
                hit_cap = True
                break
            total_waypoints += 1
            x, y = pf.coord_converter.latlon_to_local(lat, lon)
            visited.append((x, y, replans))

            # across (x) x along (y): the footprint rectangle is axis-aligned
            # to the field, not rotated to the path's local heading -- fine
            # here since this field's path runs essentially north-south
            # throughout.
            llx, lly = x - half_across, y - half_along
            corners_local = [
                (llx, lly), (llx + shape_across_ft, lly),
                (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
            ]
            corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
            # Mark this photo's footprint seen regardless of what's in it --
            # it was photographed either way.
            pf.accept_image_corner_coord(corners_latlon)

            newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
            if newly_found:
                for mx, my in newly_found:
                    discovered[(mx, my)] = replans
                    mine_lat, mine_lon = pf.coord_converter.local_to_latlon(mx, my)
                    pf.add_discovered_mine(mine_lat, mine_lon)
                found_mine_this_pass = True
                break  # the graph just changed -- stop this queue, replan

        if record_frames:
            frames.append(_snapshot_frame(pf, replans, discovered, visited))

        if hit_cap:
            break

    return {
        "discovered": discovered,
        "visited": visited,
        "replans": replans,
        "total_waypoints": total_waypoints,
        "hit_cap": hit_cap,
        "shape_along_ft": shape_along_ft,
        "shape_across_ft": shape_across_ft,
        "frames": frames,
    }


def test_workflow_terminates_cleanly(shape_size_ft=None, record_frames=False):
    pf = build_empty_pathfinder()
    true_mines = generate_true_minefield()
    result = simulate_one_drone(pf, true_mines, shape_size_ft=shape_size_ft, record_frames=record_frames)

    # A correct run can't loop forever -- discovering a mine always makes
    # monotonic progress (never rediscovered), and once a whole pass finds
    # nothing new the loop ends. But each productive pass can also cost a
    # handful of pure "seam cleanup" passes that find no mine: accept_image_
    # corner_coord's strict full-encompass "seen" semantics can strand a
    # cell exactly on the zero-margin boundary between two adjacent
    # placements (see path_cover.py's module docstring), and it becomes its
    # own tiny run needing one more replan to resolve. How many of these
    # accumulate scales with how many (small) placements were made in
    # total, NOT with the mine count alone -- a fine-grained footprint
    # (e.g. 4x6ft vs. the default ~24x24ft square) makes many more, smaller
    # placements over the same path and so has proportionally more seams --
    # so the allowance below is expressed against total_waypoints rather
    # than a fixed "+1" (which only ever held for the large-footprint,
    # few-placements-per-pass case).
    max_cleanup_passes = max(2, round(result["total_waypoints"] / 25))
    ok = (not result["hit_cap"]) and result["replans"] <= len(result["discovered"]) + max_cleanup_passes
    print(f"test_workflow_terminates_cleanly: replans={result['replans']} "
          f"discovered={len(result['discovered'])} waypoints={result['total_waypoints']} "
          f"hit_cap={result['hit_cap']} -> {'PASS' if ok else 'FAIL'}")
    return ok, pf, true_mines, result


def test_final_queue_is_empty(pf):
    places = pf.getPlacesToCheck()
    ok = places == []
    print(f"test_final_queue_is_empty: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_seen_covers_final_path(pf):
    """The concrete form of "seen graph == field graph": the FINAL shortest
    path's own cell footprint must be a subset of what's marked seen --
    recomputed independently via get_cell_path/bitwise ops, not by trusting
    getPlacesToCheck's own internal accounting."""
    path = pf.get_shortest_path()
    path_footprint = pf.get_cell_path(path)
    unseen_on_path = path_footprint & ~pf.seen_tracker
    ok = unseen_on_path.count() == 0
    print(f"test_seen_covers_final_path: path_cells={path_footprint.count()} "
          f"unseen_on_path={unseen_on_path.count()} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_discovered_mines_match_true_mines_found(pf, true_mines, result):
    """Every true mine that was actually flown over got added to the field
    exactly once (no duplicates, none missed) -- cross-checked against the
    field's own live mine set (standalone + nested in any union, the same
    way Field.mineHash collects them)."""
    discovered = result["discovered"]
    all_field_mines = list(pf.nodeField.mines) + pf.nodeField._collect_mines(pf.nodeField.unionObstacles)

    ok = len(discovered) > 0  # this minefield/path combo should find at least one
    for mx, my in discovered:
        nearest = min(math.hypot(m.origin[0] - mx, m.origin[1] - my) for m in all_field_mines)
        ok = ok and nearest <= WIDTHOFSQUARE * math.sqrt(2)
    # merges can only ever reduce live-mine count relative to discoveries,
    # never inflate it past the number of distinct detections made
    ok = ok and len(all_field_mines) <= len(discovered)
    print(f"test_discovered_mines_match_true_mines_found: true_mines={len(true_mines)} "
          f"discovered={len(discovered)} live_field_mines={len(all_field_mines)} -> {'PASS' if ok else 'FAIL'}")
    return ok


def render_workflow_diagram(pf, true_mines, result, save_path):
    """
    One image showing the whole simulated mission:
      - the shortest path's cell footprint, rasterized onto a CellField
        (black blocks) via get_cell_path/mark_path
      - every discovered mine's actual NODE-GRAPH polygon (BlockMine's
        convex-hull outline, used for pathfinding/obstacle connectivity) --
        kept as an actual vector polygon (firebrick outline/fill), NOT
        rasterized -- this is the shape the node graph itself reasons
        about, so it's shown in its own native (non-cell) form
      - every discovered mine's raw SAFETY-RADIUS block grid -- pulled
        straight from pf.mineFieldTracker, which add_discovered_mine now
        populates directly from each protoMine's own already-computed
        blockMatrix, no re-derivation from geometry (darkorange blocks)
      These two mine layers are DIFFERENT representations, not a redundant
      double-draw: the polygon is a convex hull of the block-edge wrapping
      vertices (what the node graph treats as the obstacle), while
      mineFieldTracker is the raw "which competition square is within the
      safety radius" grid protoMine computed directly -- they usually
      closely agree but can legitimately differ by a cell or two at the
      boundary, which is exactly why both are drawn (polygon outline over
      the block grid) rather than picked one to show.
      - true minefield: hollow gray dashed = never flown over (ground
        truth the drone never learned about; discovered ones are the
        polygon + blocks above instead)
      - the actual flight trail across every replan pass (colored by
        generation, so you can see the drone re-route after each discovery)
      - seen_tracker coverage (light green shading, underneath everything)
      - the FINAL shortest path (magenta line, for the exact geometry the
        black path cells were rasterized from)
    """
    import numpy as np
    from matplotlib.colors import ListedColormap

    path = pf.get_shortest_path()
    path_footprint = pf.get_cell_path(path)
    width_cells, height_cells = path_footprint.width, path_footprint.height
    extent = (0, WIDTHOFFIELD, 0, HEIGHTOFFIELD)

    seen_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in pf.seen_tracker.on_cells():
        seen_arr[y, x] = 1

    path_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in path_footprint.on_cells():
        path_arr[y, x] = 1

    # The raw safety-radius block grid, straight from mineFieldTracker --
    # no rasterization step here at all, it's already a CellField.
    mine_blocks_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in pf.mineFieldTracker.on_cells():
        mine_blocks_arr[y, x] = 1

    fig, ax = plt.subplots(figsize=(8, 20), dpi=100)
    ax.imshow(seen_arr, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
              interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1)
    ax.imshow(path_arr, cmap=ListedColormap(["none", "black"]), vmin=0, vmax=1,
              interpolation="nearest", origin="lower", alpha=0.9, extent=extent, zorder=2)
    ax.imshow(mine_blocks_arr, cmap=ListedColormap(["none", "darkorange"]), vmin=0, vmax=1,
              interpolation="nearest", origin="lower", alpha=0.6, extent=extent, zorder=3)

    # The node-graph polygon itself, kept as a real vector shape (not
    # rasterized) -- includes merged unionObstacles, each still exposing
    # .vertices the same way a standalone BlockMine does.
    for obstacle in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles):
        ax.add_patch(MplPolygon(list(obstacle.vertices), closed=True, facecolor="none",
                                 edgecolor="firebrick", linewidth=1.3, zorder=3.5))

    discovered = result["discovered"]
    for mx, my in true_mines:
        if (mx, my) in discovered:
            continue
        ax.add_patch(Circle((mx, my), WIDTHOFSQUARE * 1.5, facecolor="none",
                             edgecolor="gray", linestyle="--", linewidth=1.2, zorder=4))

    cmap = plt.get_cmap("plasma")
    max_gen = max((g for _, _, g in result["visited"]), default=1)
    for i in range(len(result["visited"]) - 1):
        x0, y0, g0 = result["visited"][i]
        x1, y1, g1 = result["visited"][i + 1]
        if g0 != g1:
            continue  # don't draw a line across a replan jump
        color = cmap(g0 / max(1, max_gen))
        ax.plot([x0, x1], [y0, y1], color=color, linewidth=1.0, zorder=5, alpha=0.8)
    vxs = [v[0] for v in result["visited"]]
    vys = [v[1] for v in result["visited"]]
    vgens = [v[2] for v in result["visited"]]
    ax.scatter(vxs, vys, c=vgens, cmap="plasma", s=14, zorder=6, edgecolors="white", linewidths=0.3)

    pxs = [n.x for n in path]
    pys = [n.y for n in path]
    ax.plot(pxs, pys, color="magenta", linewidth=1.6, zorder=7, alpha=0.9)

    ax.set_xlim(0, WIDTHOFFIELD)
    ax.set_ylim(0, HEIGHTOFFIELD)
    ax.set_aspect("equal")
    ax.set_title(
        f"Single-drone discover-as-you-fly simulation\n"
        f"{len(discovered)}/{len(true_mines)} true mines found over {result['replans']} replans, "
        f"{result['total_waypoints']} photos taken",
        fontsize=11,
    )
    ax.set_xlabel(
        "Green = seen   |   Black cells = path footprint   |   Red outline = mine node-graph polygon\n"
        "Orange cells = mine safety-radius blocks (mineFieldTracker)   |   Dashed gray = undiscovered true mine\n"
        "Dots/trail colored by replan generation   |   Magenta = final shortest path",
        fontsize=9,
    )
    ax.set_xticks([])
    ax.set_yticks([])

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)


def render_workflow_gif(pf, true_mines, result, save_path, max_frames=40, frame_duration_ms=350, hold_last_frames=4):
    """
    Animates simulate_one_drone's recorded per-replan snapshots (see
    _snapshot_frame -- requires it was called with record_frames=True) into
    a GIF: each frame is the same kind of combined seen/mine-blocks/mine-
    polygon/flight-trail/current-path picture render_workflow_diagram draws
    for the final state, but as of that replan generation, so watching the
    GIF shows the field getting covered and the path re-routing around each
    newly discovered mine one replan at a time.

    Rendering every single replan gets expensive (and the GIF pointlessly
    long) on a dense minefield with many replans, so frames are evenly
    subsampled down to `max_frames` first (always keeping the first and
    last) -- this only thins out which generations get drawn, it doesn't
    change what simulate_one_drone actually did.
    """
    import io
    import numpy as np
    from PIL import Image
    from matplotlib.colors import ListedColormap

    frames_data = result["frames"]
    if not frames_data:
        raise ValueError("render_workflow_gif needs frames -- call simulate_one_drone with record_frames=True")

    if len(frames_data) > max_frames:
        idxs = sorted({round(i * (len(frames_data) - 1) / (max_frames - 1)) for i in range(max_frames)})
        frames_data = [frames_data[i] for i in idxs]

    width_cells = WIDTHOFFIELD // WIDTHOFSQUARE
    height_cells = HEIGHTOFFIELD // WIDTHOFSQUARE
    extent = (0, WIDTHOFFIELD, 0, HEIGHTOFFIELD)
    max_gen = max(1, result["replans"])

    pil_frames = []
    for frame in frames_data:
        seen_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["seen"].on_cells():
            seen_arr[y, x] = 1
        mine_blocks_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["mine_blocks"].on_cells():
            mine_blocks_arr[y, x] = 1

        fig, ax = plt.subplots(figsize=(6, 15), dpi=80)
        ax.imshow(seen_arr, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
                  interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1)
        ax.imshow(mine_blocks_arr, cmap=ListedColormap(["none", "darkorange"]), vmin=0, vmax=1,
                  interpolation="nearest", origin="lower", alpha=0.6, extent=extent, zorder=2)

        for verts in frame["polygons"]:
            ax.add_patch(MplPolygon(verts, closed=True, facecolor="none",
                                     edgecolor="firebrick", linewidth=1.1, zorder=2.5))

        discovered = frame["discovered"]
        for mx, my in true_mines:
            if (mx, my) in discovered:
                continue
            ax.add_patch(Circle((mx, my), WIDTHOFSQUARE * 1.5, facecolor="none",
                                 edgecolor="gray", linestyle="--", linewidth=1.0, zorder=3))

        visited = frame["visited"]
        for i in range(len(visited) - 1):
            x0, y0, g0 = visited[i]
            x1, y1, g1 = visited[i + 1]
            if g0 != g1:
                continue  # don't draw a line across a replan jump
            color = plt.get_cmap("plasma")(g0 / max_gen)
            ax.plot([x0, x1], [y0, y1], color=color, linewidth=1.0, zorder=4, alpha=0.85)
        if visited:
            ax.scatter([v[0] for v in visited], [v[1] for v in visited], c=[v[2] for v in visited],
                       cmap="plasma", vmin=1, vmax=max_gen, s=10, zorder=5, edgecolors="white", linewidths=0.2)

        pxs = [p[0] for p in frame["path_nodes"]]
        pys = [p[1] for p in frame["path_nodes"]]
        ax.plot(pxs, pys, color="magenta", linewidth=1.4, zorder=6, alpha=0.9)

        ax.set_xlim(0, WIDTHOFFIELD)
        ax.set_ylim(0, HEIGHTOFFIELD)
        ax.set_aspect("equal")
        ax.set_title(
            f"replan {frame['replan']}/{result['replans']}   "
            f"{len(discovered)}/{len(true_mines)} mines found",
            fontsize=10,
        )
        ax.set_xticks([])
        ax.set_yticks([])
        plt.tight_layout()

        buf = io.BytesIO()
        plt.savefig(buf, format="png")
        plt.close(fig)
        buf.seek(0)
        pil_frames.append(Image.open(buf).convert("RGB"))

    pil_frames.extend([pil_frames[-1]] * hold_last_frames)
    pil_frames[0].save(
        save_path, save_all=True, append_images=pil_frames[1:],
        duration=frame_duration_ms, loop=0, optimize=True,
    )


def main():
    import time

    # 4x6ft rectangular footprint (along=6ft parallel to the path's
    # north-south travel, across=4ft perpendicular to it -- see
    # simulate_one_drone's docstring) instead of the default matSize-derived
    # square, and record per-replan snapshots for the GIF below.
    shape_size_ft = (6.0, 4.0)

    t0 = time.perf_counter()
    term_ok, pf, true_mines, result = test_workflow_terminates_cleanly(
        shape_size_ft=shape_size_ft, record_frames=True
    )
    elapsed = time.perf_counter() - t0

    results = [
        term_ok,
        test_final_queue_is_empty(pf),
        test_seen_covers_final_path(pf),
        test_discovered_mines_match_true_mines_found(pf, true_mines, result),
    ]
    print()
    print(f"{sum(results)}/{len(results)} passed")
    print(f"true_mines={len(true_mines)} shape={shape_size_ft[0]}x{shape_size_ft[1]}ft "
          f"replans={result['replans']} waypoints={result['total_waypoints']} elapsed={elapsed:.3f}s")

    diagram_path = SCRATCH_DIR + r"\droneWorkflow_combined.png"
    render_workflow_diagram(pf, true_mines, result, diagram_path)
    print(f"saved workflow diagram to {diagram_path}")

    gif_path = SCRATCH_DIR + r"\droneWorkflow_process.gif"
    render_workflow_gif(pf, true_mines, result, gif_path)
    print(f"saved workflow gif to {gif_path}")


if __name__ == "__main__":
    main()
