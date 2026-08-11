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
     replan from scratch (Pathfinder.get_shortest_path, plain greedy: a
     fresh Dijkstra search from the true field entry with hysteresis --
     checkpoint pinning was tried and archived, see pathfinder.py), since
     a newly discovered mine can change the node graph and so the
     shortest path itself. If no mine is there, just mark it seen and
     move on to the next queued place.
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

from shapely.geometry import Point

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Polygon as MplPolygon

from flight.pathfinder import Pathfinder, WIDTHOFFIELD, HEIGHTOFFIELD, WIDTHOFSQUARE, path_length

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


def build_empty_pathfinder(start_edge="bottom"):
    """An empty Pathfinder: node field built (start/end floating nodes
    exist), but not a single mine has been discovered yet. start_edge
    picks which field edge this drone launches from -- "bottom" (default,
    unchanged for every existing caller) or "top" (the second pair,
    working from the opposite end)."""
    pf = Pathfinder(_field_corners(), altitude=20.0, fov_deg=60.0, droneID=1)
    start_y = -1 if start_edge == "bottom" else HEIGHTOFFIELD + 1
    start_latlon = pf.coord_converter.local_to_latlon(WIDTHOFFIELD / 2, start_y)
    pf.buildNodeField(start_latlon, startEdge=start_edge)
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
        (mx, my)
        for mx, my in true_mines
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
        "polygons": [
            list(o.vertices) for o in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles)
        ],
        "path_nodes": [(n.x, n.y) for n in pf.get_shortest_path()],
        "discovered": dict(discovered),
        "visited": list(visited),
    }


def _snapshot_maze_frame(pf, step, discovered, visited):
    """Same idea as _snapshot_frame, but for simulate_one_drone_maze: the
    flight trail's per-point label is "A"/"B" (not a replan generation),
    and the current C/B/A segments are captured separately (as plain
    (x, y) tuple lists, not live node references) so render_maze_workflow_gif
    can redraw the same gold/blue/black split render_maze_workflow_diagram
    uses, at whatever point this segment split was in as of this step.

    Also captures the helper-node machinery's own state, if
    use_helper_nodes is on (harmless empty lists otherwise): the
    not-yet-promoted trail (candidate breadcrumbs still being tracked)
    and the currently-promoted nodes (real graph nodes, still referenced
    somewhere in C/B/A). Comparing this across consecutive frames is what
    lets render_maze_workflow_gif show a promoted node appearing when
    start_helper_node_detour creates it and disappearing if
    check_path_envelopment later removes it -- there's no separate
    "just deleted" list, the removal shows up as simply not being in the
    next frame's promoted set."""
    return {
        "step": step,
        "seen": pf.seen_tracker.copy(),
        "mine_blocks": pf.mineFieldTracker.copy(),
        "polygons": [
            list(o.vertices) for o in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles)
        ],
        "c_path": [(n.x, n.y) for n in pf.maze_confirmed_path],
        "b_path": [(n.x, n.y) for n in pf.maze_b_path],
        "a_path": [(n.x, n.y) for n in pf.maze_a_path],
        "discovered": dict(discovered),
        "visited": list(visited),
        "helper_trail": [(x, y) for _d, x, y in pf.helper_node_trail],
        "promoted_helper_nodes": [(e["node"].x, e["node"].y) for e in pf.promoted_helper_nodes],
    }


def simulate_one_drone(
    pf,
    true_mines,
    overlap=0.1,
    path_width=0.0,
    shape_size_ft=None,
    max_replans=50,
    max_waypoints=500,
    record_frames=False,
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
        places = pf.getPlacesToCheck(
            overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
        )
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
                (llx, lly),
                (llx + shape_across_ft, lly),
                (llx + shape_across_ft, lly + shape_along_ft),
                (llx, lly + shape_along_ft),
            ]
            corners_latlon = [
                pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local
            ]
            # Mark this photo's footprint seen regardless of what's in it --
            # it was photographed either way.
            pf.accept_image_corner_coord(corners_latlon)

            newly_found = mines_under_footprint(
                true_mines, discovered, x, y, half_across, half_along
            )
            if newly_found:
                for mx, my in newly_found:
                    discovered[(mx, my)] = replans
                    mine_lat, mine_lon = pf.coord_converter.local_to_latlon(mx, my)
                    pf.add_discovered_mine(mine_lat, mine_lon)  # (obstacle, was_merged, rewound), unused here
                found_mine_this_pass = True
                # The graph just changed -- stop this queue, replan from
                # scratch (checkpoint pinning is archived, see pathfinder.py).
                break

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


def simulate_one_drone_maze(pf, true_mines, overlap=0.1, path_width=0.0, shape_size_ft=None, max_steps=300, max_waypoints=3000, point_a_mode="pinned", granular_confirm=False, record_frames=False, use_helper_nodes=False):
    """
    Same mission as simulate_one_drone, but driving Pathfinder's
    MAZE-STYLE incremental replanning (start_maze_navigation /
    on_forward_mine_discovered / reroute_b_segment / confirm_b_into_c /
    get_places_to_check_maze) instead of the plain get_shortest_path-based
    getPlacesToCheck.

    A small state machine, mode in {"A", "B"}:
      - mode "A": check segment A (initially the whole path; later just
        the small stub before the field end). A mine found here applies
        Rule 1 (on_forward_mine_discovered: recompute from point_C,
        re-split) and switches to mode "B". A's list draining to empty
        with no mine found means: if B is also empty, the mission is
        complete (this is the only way out of the loop); otherwise
        (shouldn't normally happen, but keeps the state machine total)
        switch to mode "B" anyway.
      - mode "B": check segment B. A mine found here applies one of three
        point_a_mode variants (see below) and stays in mode "B". B's list
        draining to empty confirms it into C (confirm_b_into_c) and
        switches back to mode "A" either way.

    point_a_mode: which rule fires when a mine is found in segment B --
      "pinned" (default) = Rule 2 (reroute_b_segment): reroute between the
        SAME fixed point_C/point_A, point_A never moves.
      "floating" = Rule 1 too (on_forward_mine_discovered): a full
        recompute from point_C that lets point_A land wherever that fresh
        path's own final edge ends up, no matter which mine or end node
        it's near.
      "same_mine" = reroute_b_segment_same_mine: point_A is free to slide
        to a DIFFERENT tangent node of the SAME mine it's already
        anchored to, but falls back to "pinned" behavior (Rule 2) the
        moment a fresh recompute would move it to a different mine or end
        node entirely.
    granular_confirm: False (default) = confirm_b_into_c only folds B into
      C once the WHOLE of B drains clean. True = advance_b_prefix_into_c
      is also called after every B visit pass, folding each leading edge
      of B into C as soon as ITS footprint alone is fully seen, so a later
      reroute only has to re-search whatever's left unconfirmed.
    record_frames: if True, appends a state snapshot (see
      _snapshot_maze_frame) to the returned "frames" list after every
      step, for render_maze_workflow_gif to animate. Off by default.
    use_helper_nodes: False (default) = every A-side discovery uses
      on_forward_mine_discovered (Rule 1), a full recompute from point_C.
      True = every A-side discovery first tries
      Pathfinder.start_helper_node_detour (seeded at a breadcrumb dropped
      during A-checking instead of from point_C), falling back to Rule 1
      only if that returns False (trail too short / all inside the new
      mine). Independent of point_a_mode/granular_confirm -- only changes
      A-side handling, B-side stays whatever point_a_mode says.
    """
    if point_a_mode not in ("pinned", "floating", "same_mine"):
        raise ValueError(f"point_a_mode must be 'pinned', 'floating', or 'same_mine', got {point_a_mode!r}")
    if shape_size_ft is None:
        square = max(1, round(pf.matSize / WIDTHOFSQUARE)) * WIDTHOFSQUARE
        shape_along_ft, shape_across_ft = square, square
    elif isinstance(shape_size_ft, tuple):
        shape_along_ft, shape_across_ft = shape_size_ft
    else:
        shape_along_ft, shape_across_ft = shape_size_ft, shape_size_ft
    half_along, half_across = shape_along_ft / 2.0, shape_across_ft / 2.0

    pf.start_maze_navigation()

    discovered = {}
    visited = []
    frames = []
    steps = 0
    total_waypoints = 0
    hit_cap = False
    last_added_obstacle = None  # set whenever add_discovered_mine finds something
    last_rewound = False  # rewound for that same last add_discovered_mine call

    def visit_places(places, label):
        """Visits `places` in order; stops and returns the first newly
        discovered mine's local (x, y), or None if the whole list was
        checked clean. Marks every photo seen regardless."""
        nonlocal total_waypoints, hit_cap, last_added_obstacle, last_rewound
        for lat, lon in places:
            if total_waypoints >= max_waypoints:
                hit_cap = True
                return None
            total_waypoints += 1
            x, y = pf.coord_converter.latlon_to_local(lat, lon)
            visited.append((x, y, steps, label))

            llx, lly = x - half_across, y - half_along
            corners_local = [
                (llx, lly), (llx + shape_across_ft, lly),
                (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
            ]
            corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
            pf.accept_image_corner_coord(corners_latlon)

            if use_helper_nodes and label == "A":
                pf.record_helper_node_candidate(x, y)

            newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
            if newly_found:
                for mx, my in newly_found:
                    discovered[(mx, my)] = steps
                    mine_lat, mine_lon = pf.coord_converter.local_to_latlon(mx, my)
                    last_added_obstacle, _, last_rewound = pf.add_discovered_mine(mine_lat, mine_lon)
                if record_frames:
                    frames.append(_snapshot_maze_frame(pf, steps, discovered, visited))
                return newly_found[0]
            if record_frames:
                frames.append(_snapshot_maze_frame(pf, steps, discovered, visited))
        return None

    mode = "A"
    while True:
        steps += 1
        if steps > max_steps:
            hit_cap = True
            break

        places = pf.get_places_to_check_maze(overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft)

        if mode == "A":
            if not places["a"]:
                if not places["b"]:
                    break  # both segments clean -- mission complete
                mode = "B"
                continue
            found = visit_places(places["a"], "A")
            if hit_cap:
                break
            if found is not None:
                if last_rewound:
                    pass  # add_discovered_mine's check_merge_rewind already re-routed
                elif use_helper_nodes and last_added_obstacle is not None:
                    if not pf.start_helper_node_detour(last_added_obstacle):
                        pf.on_forward_mine_discovered()  # fallback: trail unusable
                else:
                    pf.on_forward_mine_discovered()  # Rule 1
                mode = "B"
            # else: "a" fully visited clean -- next loop's places["a"] will be []
        else:  # mode == "B"
            if not places["b"]:
                pf.confirm_b_into_c()
                mode = "A"
                continue
            found = visit_places(places["b"], "B")
            if hit_cap:
                break
            if granular_confirm:
                pf.advance_b_prefix_into_c()
            if found is not None:
                if last_rewound:
                    pass  # add_discovered_mine's check_merge_rewind already re-routed
                elif point_a_mode == "pinned":
                    pf.reroute_b_segment()  # Rule 2, stays in mode "B"
                elif point_a_mode == "floating":
                    pf.on_forward_mine_discovered()  # Rule 1 for B too -- floating point_A
                else:  # "same_mine"
                    pf.reroute_b_segment_same_mine()
            # else: "b" fully visited clean -- next loop's places["b"] will be []

        if record_frames:
            frames.append(_snapshot_maze_frame(pf, steps, discovered, visited))

    return {
        "discovered": discovered,
        "visited": visited,
        "steps": steps,
        "total_waypoints": total_waypoints,
        "total_distance": path_length([(x, y) for x, y, _s, _l in visited]),
        "hit_cap": hit_cap,
        "frames": frames,
    }


def simulate_one_pair_maze(pf, true_mines, overlap=0.1, path_width=0.0, shape_size_ft=None, max_steps=300, max_waypoints=3000, point_a_mode="pinned", granular_confirm=False, record_frames=False, use_helper_nodes=True):
    """
    Sibling to simulate_one_drone_maze, but for a GAMBLER/ASSISTANT pair
    sharing one Pathfinder (one node graph, one seen_tracker, one mine list
    -- "discoveries shared between all drones" and "identical deterministic
    thoughts" both fall out for free from sharing the object, no
    synchronization code needed for a single pair).

    Where simulate_one_drone_maze alternates between checking segment A and
    segment B (one role doing both jobs serially), this drives BOTH every
    round: the gambler always works segment A (on_forward_mine_discovered /
    start_helper_node_detour on a discovery -- exactly Rule 1/the helper-node
    alternative, unchanged), the assistant always works segment B
    (reroute_b_segment* / confirm_b_into_c / advance_b_prefix_into_c --
    exactly Rule 2, unchanged). Neither role's own rules changed; only the
    driver that decides who acts when did.

    Each round: places_to_check_maze()'s "a"/"b" lists are snapshotted ONCE
    at the top, then gambler and assistant round-robin one photo at a time
    (gambler's turn, then assistant's turn, repeat) through their OWN
    snapshot until each either finds a mine or drains its list. This is
    what makes it a genuine simultaneity approximation rather than a
    disguised alternation: if gambler finds a mine on turn 3 of 20, it stops
    taking further turns this round, but the assistant KEEPS going on its
    own already-snapshotted (now possibly stale) segment B for the rest of
    the round instead of instantly seeing the update -- a real assistant
    drone doesn't get gambler's news until the round's over either. An
    earlier version re-derived segment B immediately after any gambler
    discovery, which meant the assistant never once flew on stale
    information; that understated the real rework cost of the gambler never
    waiting to reverify, which this round-robin version now actually
    incurs (still only an approximation -- the round boundary is a coarser
    proxy for reaction lag than continuous time would give; see the plan
    file for the fully faithful version deferred to later).

    Both roles' discovery rules are applied once, after the round-robin
    finishes -- add_discovered_mine's own graph repair (check_merge_rewind/
    check_path_envelopment) already runs immediately inside each turn
    regardless; what's deferred is only the segment-A/B-recompute rule
    (start_helper_node_detour/on_forward_mine_discovered/reroute_b_segment*).
    Each role tracks its OWN last-added-obstacle/rewound state separately
    (not shared nonlocals) -- both roles can legitimately find a mine in the
    same round, and sharing that state would let whichever role's turn
    happened to land last silently clobber the other's rule inputs.

    `visited` entries are still (x, y, round, label) with label in
    {"A", "B"} -- the exact shape _snapshot_maze_frame/
    render_maze_workflow_gif/render_maze_workflow_diagram already consume,
    so none of those need to change.

    Same params as simulate_one_drone_maze except `steps` is renamed `rounds`
    conceptually (returned under the same "steps" key for drop-in
    compatibility with the render functions) -- one round is both roles
    getting a turn, not one role finishing its whole queue.
    """
    if point_a_mode not in ("pinned", "floating", "same_mine"):
        raise ValueError(f"point_a_mode must be 'pinned', 'floating', or 'same_mine', got {point_a_mode!r}")
    if shape_size_ft is None:
        square = max(1, round(pf.matSize / WIDTHOFSQUARE)) * WIDTHOFSQUARE
        shape_along_ft, shape_across_ft = square, square
    elif isinstance(shape_size_ft, tuple):
        shape_along_ft, shape_across_ft = shape_size_ft
    else:
        shape_along_ft, shape_across_ft = shape_size_ft, shape_size_ft
    half_along, half_across = shape_along_ft / 2.0, shape_across_ft / 2.0

    pf.start_maze_navigation()

    discovered = {}
    visited = []
    frames = []
    rounds = 0
    total_waypoints = 0
    hit_cap = False
    # Per-role, NOT shared -- both roles can find a mine in the same round
    # (round-robin interleaving means neither one's turns are guaranteed to
    # finish before the other's), and each role's rule application below
    # needs its OWN last discovery, not whichever role's add_discovered_mine
    # call happened to run most recently.
    last_added_obstacle_a = None
    last_rewound_a = False
    last_rewound_b = False

    def visit_one(lat, lon, label):
        """One photo: same body as simulate_one_drone_maze's visit_places
        does per iteration. Returns the newly discovered mine's local
        (x, y) if one was found there, else None."""
        nonlocal total_waypoints, hit_cap, last_added_obstacle_a, last_rewound_a, last_rewound_b
        if total_waypoints >= max_waypoints:
            hit_cap = True
            return None
        total_waypoints += 1
        x, y = pf.coord_converter.latlon_to_local(lat, lon)
        visited.append((x, y, rounds, label))

        llx, lly = x - half_across, y - half_along
        corners_local = [
            (llx, lly), (llx + shape_across_ft, lly),
            (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
        ]
        corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
        pf.accept_image_corner_coord(corners_latlon)

        if use_helper_nodes and label == "A":
            pf.record_helper_node_candidate(x, y)

        newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
        if newly_found:
            obstacle = rewound = None
            for mx, my in newly_found:
                discovered[(mx, my)] = rounds
                mine_lat, mine_lon = pf.coord_converter.local_to_latlon(mx, my)
                obstacle, _, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
            if label == "A":
                last_added_obstacle_a, last_rewound_a = obstacle, rewound
            else:
                last_rewound_b = rewound
            if record_frames:
                frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))
            return newly_found[0]
        if record_frames:
            frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))
        return None

    while True:
        rounds += 1
        if rounds > max_steps:
            hit_cap = True
            break

        places = pf.get_places_to_check_maze(overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft)
        a_places, b_places = places["a"], places["b"]
        if not a_places and not b_places:
            pf.confirm_b_into_c()  # no-op if b is already empty -- safety net
            break

        # Round-robin, one photo per turn, alternating -- see the docstring
        # for why this (not "gambler's whole queue, then assistant's") is
        # what makes the assistant able to end up flying on stale segment-B
        # info for the rest of THIS round if the gambler finds something
        # partway through.
        a_idx = b_idx = 0
        a_stopped = b_stopped = False
        found_a = found_b = None
        while (a_idx < len(a_places) and not a_stopped) or (b_idx < len(b_places) and not b_stopped):
            if a_idx < len(a_places) and not a_stopped:
                lat, lon = a_places[a_idx]
                a_idx += 1
                mine = visit_one(lat, lon, "A")
                if hit_cap:
                    break
                if mine is not None:
                    found_a = mine
                    a_stopped = True
            if b_idx < len(b_places) and not b_stopped:
                lat, lon = b_places[b_idx]
                b_idx += 1
                mine = visit_one(lat, lon, "B")
                if hit_cap:
                    break
                if mine is not None:
                    found_b = mine
                    b_stopped = True
        if hit_cap:
            break

        # Gambler's discovery rule, applied once the round-robin settles.
        if found_a is not None:
            if last_rewound_a:
                pass  # add_discovered_mine's check_merge_rewind already re-routed
            elif use_helper_nodes and last_added_obstacle_a is not None:
                if not pf.start_helper_node_detour(last_added_obstacle_a):
                    pf.on_forward_mine_discovered()  # fallback: trail unusable
            else:
                pf.on_forward_mine_discovered()  # Rule 1

        # Assistant's discovery rule -- and whether segment B has actually
        # drained. This re-derives current B state fresh for the CONFIRM
        # decision only (never for what the assistant actually flew, which
        # was b_places, the pre-round stale snapshot, above) -- the
        # gambler's rule may have just replaced maze_b_path with a brand
        # new, not-yet-photographed corridor, and confirming based on the
        # stale pre-round b_places snapshot would silently fold
        # unphotographed ground straight into "checked" segment C.
        current_b_places = b_places if found_a is None else pf.get_places_to_check_maze(
            overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
        )["b"]
        if not current_b_places:
            pf.confirm_b_into_c()
        else:
            if granular_confirm:
                pf.advance_b_prefix_into_c()
            if found_b is not None:
                if last_rewound_b:
                    pass  # add_discovered_mine's check_merge_rewind already re-routed
                elif point_a_mode == "pinned":
                    pf.reroute_b_segment()  # Rule 2, assistant keeps working "B"
                elif point_a_mode == "floating":
                    pf.on_forward_mine_discovered()  # Rule 1 for B too -- floating point_A
                else:  # "same_mine"
                    pf.reroute_b_segment_same_mine()
            # else: "b" fully visited clean this round -- next round's
            # places["b"] comes back empty and gets confirmed then (same
            # one-round lag simulate_one_drone_maze already has).

        if record_frames:
            frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))

    # Distance traveled per role -- gambler and assistant are two physically
    # distinct drones, so this is two separate chronological trajectories
    # (filtering visited by label preserves each role's own visit order),
    # not one combined path_length over the interleaved list.
    gambler_distance = path_length([(x, y) for x, y, _r, label in visited if label == "A"])
    assistant_distance = path_length([(x, y) for x, y, _r, label in visited if label == "B"])
    gambler_waypoints = sum(1 for _x, _y, _r, label in visited if label == "A")
    assistant_waypoints = sum(1 for _x, _y, _r, label in visited if label == "B")

    return {
        "discovered": discovered,
        "visited": visited,
        "steps": rounds,
        "total_waypoints": total_waypoints,
        "gambler_waypoints": gambler_waypoints,
        "assistant_waypoints": assistant_waypoints,
        "gambler_distance": gambler_distance,
        "assistant_distance": assistant_distance,
        "total_distance": gambler_distance + assistant_distance,
        "hit_cap": hit_cap,
        "frames": frames,
    }


def simulate_leader_follower_pair(pf, true_mines, overlap=0.1, path_width=0.0, shape_size_ft=None, max_steps=300, max_waypoints=3000, point_a_mode="pinned", granular_confirm=False, record_frames=False, use_helper_nodes=True):
    """
    Same GAMBLER/ASSISTANT pair, same round-robin approximation, same
    return shape as simulate_one_pair_maze (existing render/test helpers
    apply unchanged) -- but restructured so the ASSISTANT ("follower")
    side of every round only ever touches `pf` through the two functions
    marked LEADER<-FOLLOWER BOUNDARY below, instead of calling
    pf.accept_image_corner_coord/pf.add_discovered_mine directly the way
    simulate_one_pair_maze's shared-object version does.

    This distinction matters for exactly one reason: simulate_one_pair_maze
    is only valid when gambler and assistant are the SAME Python process
    sharing one Pathfinder object, which is true in every test/sweep this
    session but NOT true of two physical, WiFi-connected drones -- a real
    ASSISTANT never builds a Pathfinder at all (see configureField's
    Role.ASSISTANT check) and has no way to touch pf's internals. Only the
    LEADER (the GAMBLER's device) runs a real Pathfinder here; the
    follower's own "device" is modeled as nothing more than the plain
    lat/lon waypoint list it was handed and the plain lat/lon reports it
    sends back -- exactly the data that would have to cross the wire.

    The two boundary functions are where a real implementation plugs in
    actual interdrone messages -- deliberately left as descriptions, not
    live Message/Interdrone calls (that wiring, and the leader/follower
    vs. flat-broadcast questions it raises, is out of scope here; see the
    coordinating-4-drones plan). What they doc says is the real contract:
    what data has to cross the wire, and in which direction, for this
    round-robin approximation to remain valid once it's actually split
    across two devices.
    """
    if point_a_mode not in ("pinned", "floating", "same_mine"):
        raise ValueError(f"point_a_mode must be 'pinned', 'floating', or 'same_mine', got {point_a_mode!r}")
    if shape_size_ft is None:
        square = max(1, round(pf.matSize / WIDTHOFSQUARE)) * WIDTHOFSQUARE
        shape_along_ft, shape_across_ft = square, square
    elif isinstance(shape_size_ft, tuple):
        shape_along_ft, shape_across_ft = shape_size_ft
    else:
        shape_along_ft, shape_across_ft = shape_size_ft, shape_size_ft
    half_along, half_across = shape_along_ft / 2.0, shape_across_ft / 2.0

    pf.start_maze_navigation()

    discovered = {}
    visited = []
    frames = []
    rounds = 0
    total_waypoints = 0
    hit_cap = False
    last_added_obstacle_a = None
    last_rewound_a = False
    last_rewound_b = False

    def _leader_visit_a(lat, lon):
        """LEADER side, segment A: the gambler is flying its own device's
        Pathfinder-planned route, so this needs no relay at all -- same
        body as simulate_one_pair_maze's visit_one(..., "A")."""
        nonlocal total_waypoints, hit_cap, last_added_obstacle_a, last_rewound_a
        if total_waypoints >= max_waypoints:
            hit_cap = True
            return None
        total_waypoints += 1
        x, y = pf.coord_converter.latlon_to_local(lat, lon)
        visited.append((x, y, rounds, "A"))

        llx, lly = x - half_across, y - half_along
        corners_local = [
            (llx, lly), (llx + shape_across_ft, lly),
            (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
        ]
        corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
        pf.accept_image_corner_coord(corners_latlon)

        if use_helper_nodes:
            pf.record_helper_node_candidate(x, y)

        newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
        if newly_found:
            obstacle = rewound = None
            for mx, my in newly_found:
                discovered[(mx, my)] = rounds
                mine_lat, mine_lon = pf.coord_converter.local_to_latlon(mx, my)
                obstacle, _, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
            last_added_obstacle_a, last_rewound_a = obstacle, rewound
            if record_frames:
                frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))
            return newly_found[0]
        if record_frames:
            frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))
        return None

    def _follower_visit_b(lat, lon):
        """FOLLOWER side, segment B: models the assistant's OWN device
        flying to (lat, lon) and taking a photo -- it has no Pathfinder,
        so it can only report back what it physically observed, in plain
        lat/lon: the photo's footprint corners (for coverage) and,
        separately, any true mine physically under that footprint (for
        the leader to add to its own graph).

        Real wire equivalent (not implemented here -- see the module
        docstring): the follower flies this waypoint like any other
        (existing REACHED_WAYPOINT confirms arrival), then reports the
        photo back via a message shaped like MessageType.SHARE_PHOTOS
        (image corner coords + any mine coordinates found in it) --
        NOT MessageType.NEW_WAYPOINTS, which only flows leader->follower.
        Returns (photo_report, mine_reports): photo_report is the plain
        (lat, lon) to report to _leader_apply_follower_report for coverage;
        mine_reports is a list of (lat, lon) for every true mine physically
        under this footprint (can be more than one, or empty -- mirrors
        simulate_one_pair_maze's visit_one, which loops over every entry
        mines_under_footprint returns, not just the first)."""
        nonlocal total_waypoints, hit_cap
        if total_waypoints >= max_waypoints:
            hit_cap = True
            return None, None
        total_waypoints += 1
        # The follower has no coord_converter of its own in a real
        # deployment either -- lat/lon IS the shared coordinate frame
        # both devices already agree on (mission_field_corners), so no
        # conversion is needed to report a position back. Using pf's
        # converter here only because this is one simulated process；in
        # a real split the follower would report these exact lat/lon
        # values without ever touching a Pathfinder.
        x, y = pf.coord_converter.latlon_to_local(lat, lon)
        visited.append((x, y, rounds, "B"))
        newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
        photo_report = (lat, lon)  # -> SHARE_PHOTOS: this waypoint's corner coords
        mine_reports = []
        for mx, my in newly_found:
            discovered[(mx, my)] = rounds
            mine_reports.append(pf.coord_converter.local_to_latlon(mx, my))  # -> SHARE_PHOTOS: mines[]
        return photo_report, mine_reports

    def _leader_apply_follower_report(photo_lat, photo_lon, mine_reports):
        """LEADER<-FOLLOWER BOUNDARY. Applies one follower photo report to
        the leader's own Pathfinder -- the only place segment B's
        coverage/discoveries ever touch `pf` in this design. Real wire
        equivalent: the body of a SHARE_PHOTOS receive-side handler (see
        state_machine/interdrone.py -- currently send-only, no case
        MessageType.SHARE_PHOTOS in the receive dispatch yet; left as a
        placeholder comment there deliberately, see this session's plan).
        Returns the FIRST newly discovered mine's local (x, y) if
        mine_reports is non-empty, else None -- same contract as
        simulate_one_pair_maze's visit_one (return newly_found[0]) for the
        caller's found_b tracking, even though every entry in mine_reports
        gets added to the graph below, same as that function's own loop."""
        nonlocal last_rewound_b
        x, y = pf.coord_converter.latlon_to_local(photo_lat, photo_lon)
        llx, lly = x - half_across, y - half_along
        corners_local = [
            (llx, lly), (llx + shape_across_ft, lly),
            (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
        ]
        corners_latlon = [pf.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
        pf.accept_image_corner_coord(corners_latlon)
        if not mine_reports:
            if record_frames:
                frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))
            return None
        rewound = None
        for mine_lat, mine_lon in mine_reports:
            _obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
        last_rewound_b = rewound
        first_mx, first_my = pf.coord_converter.latlon_to_local(*mine_reports[0])
        if record_frames:
            frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))
        return (first_mx, first_my)

    while True:
        rounds += 1
        if rounds > max_steps:
            hit_cap = True
            break

        # LEADER<-FOLLOWER BOUNDARY: places["b"] (plain lat/lon) is what a
        # real deployment sends the follower this round via
        # MessageType.NEW_WAYPOINTS -- everything from here down to the
        # round-robin loop below is still leader-side planning, unchanged
        # from simulate_one_pair_maze.
        places = pf.get_places_to_check_maze(overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft)
        a_places, b_places = places["a"], places["b"]
        if not a_places and not b_places:
            pf.confirm_b_into_c()  # no-op if b is already empty -- safety net
            break

        a_idx = b_idx = 0
        a_stopped = b_stopped = False
        found_a = found_b = None
        while (a_idx < len(a_places) and not a_stopped) or (b_idx < len(b_places) and not b_stopped):
            if a_idx < len(a_places) and not a_stopped:
                lat, lon = a_places[a_idx]
                a_idx += 1
                mine = _leader_visit_a(lat, lon)
                if hit_cap:
                    break
                if mine is not None:
                    found_a = mine
                    a_stopped = True
            if b_idx < len(b_places) and not b_stopped:
                lat, lon = b_places[b_idx]
                b_idx += 1
                photo_report, mine_reports = _follower_visit_b(lat, lon)
                if hit_cap:
                    break
                mine = _leader_apply_follower_report(photo_report[0], photo_report[1], mine_reports)
                if mine is not None:
                    found_b = mine
                    b_stopped = True
        if hit_cap:
            break

        if found_a is not None:
            if last_rewound_a:
                pass
            elif use_helper_nodes and last_added_obstacle_a is not None:
                if not pf.start_helper_node_detour(last_added_obstacle_a):
                    pf.on_forward_mine_discovered()
            else:
                pf.on_forward_mine_discovered()

        current_b_places = b_places if found_a is None else pf.get_places_to_check_maze(
            overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
        )["b"]
        if not current_b_places:
            pf.confirm_b_into_c()
        else:
            if granular_confirm:
                pf.advance_b_prefix_into_c()
            if found_b is not None:
                if last_rewound_b:
                    pass
                elif point_a_mode == "pinned":
                    pf.reroute_b_segment()
                elif point_a_mode == "floating":
                    pf.on_forward_mine_discovered()
                else:
                    pf.reroute_b_segment_same_mine()

        if record_frames:
            frames.append(_snapshot_maze_frame(pf, rounds, discovered, visited))

    gambler_distance = path_length([(x, y) for x, y, _r, label in visited if label == "A"])
    assistant_distance = path_length([(x, y) for x, y, _r, label in visited if label == "B"])
    gambler_waypoints = sum(1 for _x, _y, _r, label in visited if label == "A")
    assistant_waypoints = sum(1 for _x, _y, _r, label in visited if label == "B")

    return {
        "discovered": discovered,
        "visited": visited,
        "steps": rounds,
        "total_waypoints": total_waypoints,
        "gambler_waypoints": gambler_waypoints,
        "assistant_waypoints": assistant_waypoints,
        "gambler_distance": gambler_distance,
        "assistant_distance": assistant_distance,
        "total_distance": gambler_distance + assistant_distance,
        "hit_cap": hit_cap,
        "frames": frames,
    }


def _snapshot_two_pair_frame(pf1, pf2, step, discovered, visited):
    """Same idea as _snapshot_maze_frame, but captures BOTH pairs' C/B/A
    segments and helper-node state in one frame -- pf1/pf2 share
    seen_tracker/mineFieldTracker (same physical field), so only one copy
    of each is needed; polygons are the union of both pairs' own
    nodeField.mines/unionObstacles (each pair only has an obstacle in its
    own Field once it's locally known -- pf1's own list can lag pf2's
    briefly between relay steps, or vice versa, which is fine/expected)."""
    obstacles = list(
        {id(o): o for o in (
            list(pf1.nodeField.mines) + list(pf1.nodeField.unionObstacles)
            + list(pf2.nodeField.mines) + list(pf2.nodeField.unionObstacles)
        )}.values()
    )
    return {
        "step": step,
        "seen": pf1.seen_tracker.copy(),
        "mine_blocks": pf1.mineFieldTracker.copy(),
        "polygons": [list(o.vertices) for o in obstacles],
        "c_path_1": [(n.x, n.y) for n in pf1.maze_confirmed_path],
        "b_path_1": [(n.x, n.y) for n in pf1.maze_b_path],
        "a_path_1": [(n.x, n.y) for n in pf1.maze_a_path],
        "c_path_2": [(n.x, n.y) for n in pf2.maze_confirmed_path],
        "b_path_2": [(n.x, n.y) for n in pf2.maze_b_path],
        "a_path_2": [(n.x, n.y) for n in pf2.maze_a_path],
        "discovered": dict(discovered),
        "visited": list(visited),
    }


def simulate_two_pairs_maze(pf1, pf2, true_mines, overlap=0.1, path_width=0.0, shape_size_ft=None,
                             max_steps=300, max_waypoints=6000, point_a_mode="pinned",
                             granular_confirm=False, record_frames=False, use_helper_nodes=True):
    """
    Two gambler/assistant pairs (simulate_one_pair_maze's per-pair round-robin,
    unmodified) launching from OPPOSITE ends of the same field --
    pf1 = build_empty_pathfinder("bottom"), pf2 = build_empty_pathfinder("top")
    -- sharing pf1.seen_tracker/mineFieldTracker with pf2 (assigned by the
    caller before this runs: same physical field, both pairs' coverage and
    known-obstacle-blocks need to be ONE record, not two independent ones).
    path_tracker stays separate per pf (a scratch buffer rasterize_node_path
    clears and rewrites every call -- sharing it would have one pair's call
    clobber the other's mid-use).

    Each pair's segment A targets the OTHER pair's current point_A (see
    Pathfinder.retarget_approach_target), not the field's far edge -- so the
    two pairs converge toward a meeting point roughly in the middle instead
    of each redundantly traversing the whole field. _sync_approach_target
    (below) keeps that retargeting in sync: once right after both pairs'
    start_maze_navigation() (bootstrapped against the real far edge first,
    since neither pair has a point_A yet to aim at), then again after every
    step_pair() call in the main loop, so a mine-driven reroute that moves
    one pair's own point_A propagates to the other within one round.

    Each super-round: pf1's gambler+assistant round-robin step, then pf2's
    (same per-pair body as simulate_one_pair_maze, just factored so it runs
    against either pf). Whenever either pf's own visit_one finds a NEW mine,
    it's relayed into the OTHER pf's add_discovered_mine too (prefer_local_
    patch=True, since a mine crossing the other pair's ALREADY-CONFIRMED
    history should get a bounded local splice, not check_path_envelopment's
    full recompute-to-the-end -- see patch_confirmed_span). Two further
    cross-pair effects, both from the RECEIVING pf's own new methods:
      - record_remote_mine_on_segment_a: if the relayed mine crosses the
        receiving pair's segment-A line (not yet flown), records a
        placeholder breadcrumb there (no graph changes yet). The gambler's
        OWN visit_one loop checks _check_remote_placeholder_reached after
        every A-segment photo; reaching it promotes the placeholder into a
        real detour exactly like a local discovery. Finding something else
        first just lets it go stale (cleared alongside every maze_a_path-
        replacing rule) -- the mine's already in the graph either way, so
        the normal Rule-1 fallback already accounts for it.
      - patch_confirmed_span's result (if the relay hit confirmed history)
        is appended to the DISCOVERING pf's own cross_pair_patches -- that
        pair's assistant, not the owning pair's, verifies/photographs the
        patched stretch, since it's the one physically nearby. Folded into
        the assistant's own segment-B queue each round (prepended ahead of
        the normal B places) rather than tracked as a separate turn.
      - A remote mine landing in the receiving pair's ACTIVE segment B needs
        NEITHER new mechanism: add_discovered_mine's own check_merge_rewind/
        check_path_envelopment (run for every mine regardless of who
        relayed it) already scans maze_b_path for exactly this.

    `visited` entries are (x, y, round, label, pair_id) -- pair_id 1 or 2
    added onto simulate_one_pair_maze's shape, so a renderer can tell all
    four drones apart. `discovered` is shared across both pairs (a mine is
    "discovered" once, regardless of which pair's camera actually found it
    first) so neither pair's own mines_under_footprint re-triggers on it.
    """
    if point_a_mode not in ("pinned", "floating", "same_mine"):
        raise ValueError(f"point_a_mode must be 'pinned', 'floating', or 'same_mine', got {point_a_mode!r}")
    if shape_size_ft is None:
        square = max(1, round(pf1.matSize / WIDTHOFSQUARE)) * WIDTHOFSQUARE
        shape_along_ft, shape_across_ft = square, square
    elif isinstance(shape_size_ft, tuple):
        shape_along_ft, shape_across_ft = shape_size_ft
    else:
        shape_along_ft, shape_across_ft = shape_size_ft, shape_size_ft
    half_along, half_across = shape_along_ft / 2.0, shape_across_ft / 2.0

    def _sync_approach_target(pf_self, pf_other):
        """If pf_self's own point_A has moved since pf_other was last
        aimed at it, relays that position to pf_other via
        retarget_approach_target -- which (see its own docstring) no
        longer walks the whole way there in one hop; it only advances
        pf_other's point_A to the HALFWAY point along pf_other's own
        real path toward pf_self's position, so pf_other's gambler can
        never chain its way across the frontier into pf_self's
        still-unexplored territory before the two pairs actually meet
        there.

        No-ops if nothing moved (avoids forcing a recompute + floating-
        node churn on pf_other every round when nothing changed -- point_A
        only actually moves on a mine-driven reroute or a prior retarget,
        not on ordinary flight progress). Also no-ops while pf_self's own
        point_A is still literally its un-advanced starting position --
        see retarget_approach_target's own docstring for why relaying a
        still-degenerate position is worse than useless."""
        if not pf_self.maze_a_path:
            return
        a0 = pf_self.maze_a_path[0]
        pos = (a0.x, a0.y)
        if any(
            abs(pos[0] - s.x) < 1e-6 and abs(pos[1] - s.y) < 1e-6
            for s in pf_self.startingNodes
        ):
            return
        last = pf_other._last_synced_target
        if last is not None and abs(last[0] - pos[0]) < 1e-6 and abs(last[1] - pos[1]) < 1e-6:
            return
        pf_other.retarget_approach_target(pos[0], pos[1])
        pf_other._last_synced_target = pos

    pf1.start_maze_navigation()
    pf2.start_maze_navigation()
    # Both pairs start out aimed at the field's real far edge (an empty
    # field's own point_A is just each pair's un-advanced starting
    # position -- nothing meaningful to sync yet, see
    # _sync_approach_target's own-start guard). These calls are no-ops
    # until either pair's point_A first moves via a real reroute; kept
    # here (rather than assuming that always happens inside the main
    # loop below) so a caller that never enters the loop still ends up
    # in a consistent state.
    _sync_approach_target(pf1, pf2)
    _sync_approach_target(pf2, pf1)

    discovered = {}
    visited = []
    frames = []
    rounds = 0
    total_waypoints = 0
    hit_cap = False
    pair_state = {
        1: {"last_added_obstacle_a": None, "last_rewound_a": False, "last_rewound_b": False, "done": False},
        2: {"last_added_obstacle_a": None, "last_rewound_a": False, "last_rewound_b": False, "done": False},
    }
    pfs = {1: pf1, 2: pf2}

    def visit_one(pair_id, lat, lon, label):
        nonlocal total_waypoints, hit_cap
        pf_self = pfs[pair_id]
        pf_other = pfs[2 if pair_id == 1 else 1]
        st = pair_state[pair_id]
        if total_waypoints >= max_waypoints:
            hit_cap = True
            return None
        total_waypoints += 1
        x, y = pf_self.coord_converter.latlon_to_local(lat, lon)
        visited.append((x, y, rounds, label, pair_id))

        llx, lly = x - half_across, y - half_along
        corners_local = [
            (llx, lly), (llx + shape_across_ft, lly),
            (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
        ]
        corners_latlon = [pf_self.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
        pf_self.accept_image_corner_coord(corners_latlon)

        if use_helper_nodes and label == "A":
            pf_self.record_helper_node_candidate(x, y)
            remote_obstacle = pf_self._check_remote_placeholder_reached(x, y)
            if remote_obstacle is not None:
                st["last_added_obstacle_a"], st["last_rewound_a"] = remote_obstacle, False
                if record_frames:
                    frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
                return remote_obstacle

        newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
        if newly_found:
            obstacle = rewound = None
            for mx, my in newly_found:
                discovered[(mx, my)] = rounds
                mine_lat, mine_lon = pf_self.coord_converter.local_to_latlon(mx, my)
                obstacle, _, rewound = pf_self.add_discovered_mine(mine_lat, mine_lon)
                other_obstacle, _, _ = pf_other.add_discovered_mine(mine_lat, mine_lon, prefer_local_patch=True)
                if other_obstacle is not None:
                    pf_other.record_remote_mine_on_segment_a(other_obstacle)
                    if pf_other.last_patched_span is not None:
                        pf_self.cross_pair_patches.append(pf_other.last_patched_span)
            if label == "A":
                st["last_added_obstacle_a"], st["last_rewound_a"] = obstacle, rewound
            else:
                st["last_rewound_b"] = rewound
            if record_frames:
                frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
            return newly_found[0]
        if record_frames:
            frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
        return None

    def step_pair(pair_id):
        """One round-robin round for one pair -- identical logic to
        simulate_one_pair_maze's per-round body, just parameterized, plus
        cross_pair_patches prepended onto the assistant's own segment-B
        queue each round (drained the same way, no separate confirm step --
        a patch is just consumed/removed once its footprint is fully seen)."""
        pf_self = pfs[pair_id]
        st = pair_state[pair_id]

        places = pf_self.get_places_to_check_maze(overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft)
        patch_places = pf_self.get_cross_pair_patch_places_to_check(
            overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
        ) if pf_self.cross_pair_patches else []
        a_places = places["a"]
        b_places = patch_places + places["b"]
        if not a_places and not b_places:
            # confirm_b_into_c/advance_b_prefix_into_c only retry a
            # queued cross-pair retarget (_try_apply_pending_approach_target)
            # as part of actually confirming/advancing something -- if
            # THIS pair's own queue is already empty, neither ever runs,
            # so a still-pending retarget (halved short of the
            # CLOSE_ENOUGH_TO_STOP_HALVING_FT threshold, re-queued rather
            # than dropped -- see retarget_approach_target) would never
            # get another chance once this pair marks itself done and
            # stops being ticked at all. Retry explicitly here, before
            # declaring done, so a genuinely unfinished convergence keeps
            # being worked on instead of silently stalling.
            if pf_self._pending_approach_target is not None:
                pf_self._try_apply_pending_approach_target()
                places = pf_self.get_places_to_check_maze(
                    overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                )
                patch_places = pf_self.get_cross_pair_patch_places_to_check(
                    overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                ) if pf_self.cross_pair_patches else []
                a_places = places["a"]
                b_places = patch_places + places["b"]
        if not a_places and not b_places:
            pf_self.confirm_b_into_c()
            if pf_self.cross_pair_patches and not patch_places:
                pf_self.cross_pair_patches.pop(0)
            # confirm_b_into_c's own trailing _try_apply_pending_approach_
            # target call (see its docstring) can ITSELF revive
            # maze_a_path/maze_b_path: draining maze_b_path here can free
            # up a cross_pair_target_chain slot that was still blocking
            # the pre-check retry above (chain pruning only happens
            # inside _try_apply_pending_approach_target itself, so it
            # can't have taken effect until this exact confirm_b_into_c
            # call actually cleared maze_b_path). Re-derive places one
            # more time before committing to "done", or the pair can end
            # up permanently marked done in the very round
            # confirm_b_into_c quietly recreated a fresh, non-empty
            # approach segment for it -- confirmed directly as a real
            # 1-3 cell coverage gap on the two-pair round-robin safety
            # sweep, always immediately following the pair's own final
            # confirm_b_into_c call.
            places = pf_self.get_places_to_check_maze(
                overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
            )
            patch_places = pf_self.get_cross_pair_patch_places_to_check(
                overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
            ) if pf_self.cross_pair_patches else []
            a_places = places["a"]
            b_places = patch_places + places["b"]
            if not a_places and not b_places:
                st["done"] = True
                return

        a_idx = b_idx = 0
        a_stopped = b_stopped = False
        found_a = found_b = None
        while (a_idx < len(a_places) and not a_stopped) or (b_idx < len(b_places) and not b_stopped):
            if a_idx < len(a_places) and not a_stopped:
                lat, lon = a_places[a_idx]
                a_idx += 1
                mine = visit_one(pair_id, lat, lon, "A")
                if hit_cap:
                    break
                if mine is not None:
                    found_a = mine
                    a_stopped = True
            if b_idx < len(b_places) and not b_stopped:
                lat, lon = b_places[b_idx]
                b_idx += 1
                mine = visit_one(pair_id, lat, lon, "B")
                if hit_cap:
                    break
                if mine is not None:
                    found_b = mine
                    b_stopped = True
        if hit_cap:
            return

        if found_a is not None:
            if st["last_rewound_a"]:
                pass
            elif use_helper_nodes and st["last_added_obstacle_a"] is not None:
                if not pf_self.start_helper_node_detour(st["last_added_obstacle_a"]):
                    pf_self.on_forward_mine_discovered()
            else:
                pf_self.on_forward_mine_discovered()

        # Reuse this round's original (patch+segment-b) snapshot unless the
        # gambler's rule above just replaced maze_b_path -- same reasoning
        # as simulate_one_pair_maze's own current_b_places: re-deriving
        # unconditionally would mean the assistant never once works on
        # stale info, understating real rework cost (see that function's
        # docstring for the fuller writeup).
        if found_a is None:
            current_b_places = b_places
        else:
            current_b_places = (
                pf_self.get_cross_pair_patch_places_to_check(
                    overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                ) if pf_self.cross_pair_patches else []
            ) + pf_self.get_places_to_check_maze(
                overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
            )["b"]
        if not current_b_places:
            pf_self.confirm_b_into_c()
        else:
            if granular_confirm:
                pf_self.advance_b_prefix_into_c()
            if found_b is not None:
                if st["last_rewound_b"]:
                    pass
                elif point_a_mode == "pinned":
                    pf_self.reroute_b_segment()
                elif point_a_mode == "floating":
                    pf_self.on_forward_mine_discovered()
                else:
                    pf_self.reroute_b_segment_same_mine()

        # A patch is done (popped, no separate confirm step -- it doesn't
        # fold into anything) once its OWN footprint is fully seen,
        # independent of whatever else happened to segment B this round.
        if pf_self.cross_pair_patches and not pf_self.get_cross_pair_patch_places_to_check(
            overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
        ):
            pf_self.cross_pair_patches.pop(0)

    while True:
        rounds += 1
        if rounds > max_steps:
            hit_cap = True
            break
        pair_state[1]["done"] = pair_state[2]["done"] = False
        step_pair(1)
        if hit_cap:
            break
        _sync_approach_target(pf1, pf2)
        step_pair(2)
        if hit_cap:
            break
        _sync_approach_target(pf2, pf1)
        if record_frames:
            frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
        if pair_state[1]["done"] and pair_state[2]["done"]:
            break

    gambler1_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "A" and pid == 1])
    assistant1_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "B" and pid == 1])
    gambler2_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "A" and pid == 2])
    assistant2_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "B" and pid == 2])
    gambler1_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "A" and pid == 1)
    assistant1_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "B" and pid == 1)
    gambler2_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "A" and pid == 2)
    assistant2_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "B" and pid == 2)

    return {
        "discovered": discovered,
        "visited": visited,
        "steps": rounds,
        "total_waypoints": total_waypoints,
        "gambler1_waypoints": gambler1_waypoints,
        "assistant1_waypoints": assistant1_waypoints,
        "gambler2_waypoints": gambler2_waypoints,
        "assistant2_waypoints": assistant2_waypoints,
        "gambler1_distance": gambler1_distance,
        "assistant1_distance": assistant1_distance,
        "gambler2_distance": gambler2_distance,
        "assistant2_distance": assistant2_distance,
        "total_distance": gambler1_distance + assistant1_distance + gambler2_distance + assistant2_distance,
        "hit_cap": hit_cap,
        "frames": frames,
    }


def simulate_two_pairs_maze_round_robin(
    pf1, pf2, true_mines, overlap=0.1, path_width=0.0, shape_size_ft=None,
    max_steps=300, max_waypoints=6000, point_a_mode="pinned",
    granular_confirm=False, record_frames=False, use_helper_nodes=True,
):
    """
    Same setup, mine-relay, and per-queue reroute logic as
    simulate_two_pairs_maze -- the only difference is the visiting order.
    There, each super-round runs pf1's ENTIRE gambler+assistant round-robin
    (alternating A/B within pf1, draining both queues or stopping early on
    a mine) before starting pf2's -- so within one round, pf1's drones
    always finish before pf2's even start. Here, all FOUR queues
    (gambler1, assistant1, gambler2, assistant2) tick in a fixed
    [A1, B1, A2, B2] rotation at single-waypoint granularity: each pass
    through the rotation visits at most one place from each queue that
    still has places left and wasn't just stopped by a mine this round,
    before moving to the next queue -- so all four drones make roughly
    even progress through a round instead of one pair racing ahead of the
    other. A mine found in one queue still only stops THAT queue for the
    rest of the round (identical semantics to the A/B interleave
    simulate_two_pairs_maze already does within one pair); the other three
    keep ticking.

    Each pair's own done/confirm/reroute handling is otherwise byte-for-
    byte the same logic as simulate_two_pairs_maze's step_pair -- just
    evaluated once per pair after a whole round-robin pass instead of
    inline within a single pair's own while loop.
    """
    if point_a_mode not in ("pinned", "floating", "same_mine"):
        raise ValueError(f"point_a_mode must be 'pinned', 'floating', or 'same_mine', got {point_a_mode!r}")
    if shape_size_ft is None:
        square = max(1, round(pf1.matSize / WIDTHOFSQUARE)) * WIDTHOFSQUARE
        shape_along_ft, shape_across_ft = square, square
    elif isinstance(shape_size_ft, tuple):
        shape_along_ft, shape_across_ft = shape_size_ft
    else:
        shape_along_ft, shape_across_ft = shape_size_ft, shape_size_ft
    half_along, half_across = shape_along_ft / 2.0, shape_across_ft / 2.0

    def _sync_approach_target(pf_self, pf_other):
        if not pf_self.maze_a_path:
            return
        a0 = pf_self.maze_a_path[0]
        pos = (a0.x, a0.y)
        if any(
            abs(pos[0] - s.x) < 1e-6 and abs(pos[1] - s.y) < 1e-6
            for s in pf_self.startingNodes
        ):
            return
        last = pf_other._last_synced_target
        if last is not None and abs(last[0] - pos[0]) < 1e-6 and abs(last[1] - pos[1]) < 1e-6:
            return
        pf_other.retarget_approach_target(pos[0], pos[1])
        pf_other._last_synced_target = pos

    pf1.start_maze_navigation()
    pf2.start_maze_navigation()
    _sync_approach_target(pf1, pf2)
    _sync_approach_target(pf2, pf1)

    discovered = {}
    visited = []
    frames = []
    rounds = 0
    total_waypoints = 0
    hit_cap = False
    pair_state = {
        1: {"last_added_obstacle_a": None, "last_rewound_a": False, "last_rewound_b": False, "done": False},
        2: {"last_added_obstacle_a": None, "last_rewound_a": False, "last_rewound_b": False, "done": False},
    }
    pfs = {1: pf1, 2: pf2}

    def visit_one(pair_id, lat, lon, label):
        nonlocal total_waypoints, hit_cap
        pf_self = pfs[pair_id]
        pf_other = pfs[2 if pair_id == 1 else 1]
        st = pair_state[pair_id]
        if total_waypoints >= max_waypoints:
            hit_cap = True
            return None
        total_waypoints += 1
        x, y = pf_self.coord_converter.latlon_to_local(lat, lon)
        visited.append((x, y, rounds, label, pair_id))

        llx, lly = x - half_across, y - half_along
        corners_local = [
            (llx, lly), (llx + shape_across_ft, lly),
            (llx + shape_across_ft, lly + shape_along_ft), (llx, lly + shape_along_ft),
        ]
        corners_latlon = [pf_self.coord_converter.local_to_latlon(cx, cy) for cx, cy in corners_local]
        pf_self.accept_image_corner_coord(corners_latlon)

        if use_helper_nodes and label == "A":
            pf_self.record_helper_node_candidate(x, y)
            remote_obstacle = pf_self._check_remote_placeholder_reached(x, y)
            if remote_obstacle is not None:
                st["last_added_obstacle_a"], st["last_rewound_a"] = remote_obstacle, False
                if record_frames:
                    frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
                return remote_obstacle

        newly_found = mines_under_footprint(true_mines, discovered, x, y, half_across, half_along)
        if newly_found:
            obstacle = rewound = None
            for mx, my in newly_found:
                discovered[(mx, my)] = rounds
                mine_lat, mine_lon = pf_self.coord_converter.local_to_latlon(mx, my)
                obstacle, _, rewound = pf_self.add_discovered_mine(mine_lat, mine_lon)
                other_obstacle, _, _ = pf_other.add_discovered_mine(mine_lat, mine_lon, prefer_local_patch=True)
                if other_obstacle is not None:
                    pf_other.record_remote_mine_on_segment_a(other_obstacle)
                    if pf_other.last_patched_span is not None:
                        pf_self.cross_pair_patches.append(pf_other.last_patched_span)
            if label == "A":
                st["last_added_obstacle_a"], st["last_rewound_a"] = obstacle, rewound
            else:
                st["last_rewound_b"] = rewound
            if record_frames:
                frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
            return newly_found[0]
        if record_frames:
            frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
        return None

    rotation = [(1, "A"), (1, "B"), (2, "A"), (2, "B")]

    while True:
        rounds += 1
        if rounds > max_steps:
            hit_cap = True
            break
        pair_state[1]["done"] = pair_state[2]["done"] = False

        queues = {}
        # Snapshot each pair's OWN maze_b_path identity (not content -- a
        # reroute always replaces the list object wholesale, see
        # reroute_b_segment/on_forward_mine_discovered/check_path_envelopment,
        # never mutates in place) at the same moment its b_places got
        # computed. simulate_two_pairs_maze's own step_pair can safely
        # reuse a stale b_places snapshot later (see _react below) because
        # NOTHING else can touch this pair's maze_b_path mid-round there --
        # pf1's entire round (ticks AND reactions) finishes before pf2's
        # even starts. Here the whole point is interleaving: the OTHER
        # pair's relayed mine discovery can invoke check_merge_rewind/
        # check_path_envelopment on THIS pair mid-round (via
        # add_discovered_mine(..., prefer_local_patch=True) inside
        # visit_one), which can replace this pair's maze_b_path entirely,
        # silently invalidating the snapshot. Comparing identity here is
        # what lets _react tell "nothing but my own found_a touched this"
        # apart from "the other pair's relay already replaced it" --
        # without this, confirm_b_into_c/advance_b_prefix_into_c could
        # fold a freshly-replaced, still-unseen maze_b_path into frozen
        # confirmed history based on a check against the OLD path instead,
        # a real coverage gap observed directly in the round-robin safety
        # sweep before this fix (58 permanently-unseen cells on one seed).
        b_path_snapshot = {}
        for pair_id in (1, 2):
            pf_self = pfs[pair_id]
            places = pf_self.get_places_to_check_maze(overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft)
            patch_places = pf_self.get_cross_pair_patch_places_to_check(
                overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
            ) if pf_self.cross_pair_patches else []
            a_places = places["a"]
            b_places = patch_places + places["b"]
            b_path_snapshot[pair_id] = pf_self.maze_b_path
            if not a_places and not b_places:
                # See the matching comment in simulate_two_pairs_maze's
                # step_pair: a still-queued cross-pair retarget only ever
                # gets retried inside confirm_b_into_c/advance_b_prefix_into_c,
                # both of which no-op when this pair's own queue is already
                # empty -- so retry it explicitly before declaring done, or
                # a genuinely unfinished convergence goes permanently idle.
                if pf_self._pending_approach_target is not None:
                    pf_self._try_apply_pending_approach_target()
                    places = pf_self.get_places_to_check_maze(
                        overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                    )
                    patch_places = pf_self.get_cross_pair_patch_places_to_check(
                        overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                    ) if pf_self.cross_pair_patches else []
                    a_places = places["a"]
                    b_places = patch_places + places["b"]
                    b_path_snapshot[pair_id] = pf_self.maze_b_path
            if not a_places and not b_places:
                pf_self.confirm_b_into_c()
                if pf_self.cross_pair_patches and not patch_places:
                    pf_self.cross_pair_patches.pop(0)
                # confirm_b_into_c's own trailing _try_apply_pending_
                # approach_target call (see its docstring) can ITSELF
                # revive maze_a_path/maze_b_path: draining maze_b_path
                # here can free up a cross_pair_target_chain slot that
                # was still blocking the pre-check retry above (chain
                # pruning only happens inside
                # _try_apply_pending_approach_target itself, so it can't
                # have taken effect until THIS confirm_b_into_c call
                # actually cleared maze_b_path). Re-derive places one
                # more time before committing to "done", or the pair
                # ends up permanently marked done in the very round
                # confirm_b_into_c quietly recreated a fresh, non-empty
                # approach segment for it -- confirmed directly as a
                # real 1-3 cell coverage gap on the two-pair round-robin
                # safety sweep, always immediately following the pair's
                # own final confirm_b_into_c call.
                places = pf_self.get_places_to_check_maze(
                    overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                )
                patch_places = pf_self.get_cross_pair_patch_places_to_check(
                    overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                ) if pf_self.cross_pair_patches else []
                a_places = places["a"]
                b_places = patch_places + places["b"]
                b_path_snapshot[pair_id] = pf_self.maze_b_path
            if not a_places and not b_places:
                pair_state[pair_id]["done"] = True
                queues[(pair_id, "A")] = []
                queues[(pair_id, "B")] = []
            else:
                queues[(pair_id, "A")] = a_places
                queues[(pair_id, "B")] = b_places

        if pair_state[1]["done"] and pair_state[2]["done"]:
            break

        idx = {key: 0 for key in queues}
        stopped = {key: False for key in queues}
        found = {key: None for key in queues}
        # Tracks which pairs have already had their Rule 1/Rule 2 reaction
        # applied THIS round -- a pair stops contending for ticks the
        # instant both its own queues are drained/stopped, exactly
        # mirroring simulate_two_pairs_maze's step_pair (which finishes
        # pair 1's ENTIRE round, reaction included, before pair 2 starts
        # ticking at all). Reacting immediately here (rather than only
        # after the WHOLE 4-way rotation finishes) matters for
        # correctness, not just style: cross-pair mine relay
        # (visit_one -> add_discovered_mine(..., prefer_local_patch=True))
        # happens instantly inside the OTHER pair's own visit, so a
        # deferred reaction window lets the other pair's mid-round
        # discoveries mutate this pair's graph/state while this pair's
        # OWN reaction to ITS OWN discovery is still pending -- observed
        # directly as a real bad edge and two real coverage gaps in the
        # 200-seed safety sweep before this fix.
        reacted = {1: pair_state[1]["done"], 2: pair_state[2]["done"]}

        def _pair_finished_ticking(pair_id):
            return all(
                idx[(pair_id, lbl)] >= len(queues[(pair_id, lbl)]) or stopped[(pair_id, lbl)]
                for lbl in ("A", "B")
            )

        def _react(pair_id):
            pf_self = pfs[pair_id]
            st = pair_state[pair_id]
            found_a = found[(pair_id, "A")]
            found_b = found[(pair_id, "B")]
            b_places = queues[(pair_id, "B")]

            if found_a is not None:
                if st["last_rewound_a"]:
                    pass
                elif use_helper_nodes and st["last_added_obstacle_a"] is not None:
                    if not pf_self.start_helper_node_detour(st["last_added_obstacle_a"]):
                        pf_self.on_forward_mine_discovered()
                else:
                    pf_self.on_forward_mine_discovered()

            b_path_replaced = pf_self.maze_b_path is not b_path_snapshot[pair_id]
            if found_a is None and not b_path_replaced:
                current_b_places = b_places
            else:
                current_b_places = (
                    pf_self.get_cross_pair_patch_places_to_check(
                        overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                    ) if pf_self.cross_pair_patches else []
                ) + pf_self.get_places_to_check_maze(
                    overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
                )["b"]
            if not current_b_places:
                pf_self.confirm_b_into_c()
            else:
                if granular_confirm:
                    pf_self.advance_b_prefix_into_c()
                if found_b is not None:
                    if st["last_rewound_b"]:
                        pass
                    elif point_a_mode == "pinned":
                        pf_self.reroute_b_segment()
                    elif point_a_mode == "floating":
                        pf_self.on_forward_mine_discovered()
                    else:
                        pf_self.reroute_b_segment_same_mine()

            if pf_self.cross_pair_patches and not pf_self.get_cross_pair_patch_places_to_check(
                overlap=overlap, path_width=path_width, shape_size_ft=shape_size_ft
            ):
                pf_self.cross_pair_patches.pop(0)

            reacted[pair_id] = True

        active = True
        while active:
            active = False
            for key in rotation:
                pair_id, label = key
                if reacted[pair_id]:
                    continue
                if idx[key] < len(queues[key]) and not stopped[key]:
                    active = True
                    lat, lon = queues[key][idx[key]]
                    idx[key] += 1
                    mine = visit_one(pair_id, lat, lon, label)
                    if hit_cap:
                        break
                    if mine is not None:
                        found[key] = mine
                        stopped[key] = True
                    if not reacted[pair_id] and _pair_finished_ticking(pair_id):
                        _react(pair_id)
            if hit_cap:
                break
        if hit_cap:
            break

        for pair_id in (1, 2):
            if not reacted[pair_id]:
                _react(pair_id)

        _sync_approach_target(pf1, pf2)
        _sync_approach_target(pf2, pf1)

        if record_frames:
            frames.append(_snapshot_two_pair_frame(pf1, pf2, rounds, discovered, visited))
        if pair_state[1]["done"] and pair_state[2]["done"]:
            break

    gambler1_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "A" and pid == 1])
    assistant1_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "B" and pid == 1])
    gambler2_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "A" and pid == 2])
    assistant2_distance = path_length([(x, y) for x, y, _r, label, pid in visited if label == "B" and pid == 2])
    gambler1_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "A" and pid == 1)
    assistant1_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "B" and pid == 1)
    gambler2_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "A" and pid == 2)
    assistant2_waypoints = sum(1 for _x, _y, _r, label, pid in visited if label == "B" and pid == 2)

    return {
        "discovered": discovered,
        "visited": visited,
        "steps": rounds,
        "total_waypoints": total_waypoints,
        "gambler1_waypoints": gambler1_waypoints,
        "assistant1_waypoints": assistant1_waypoints,
        "gambler2_waypoints": gambler2_waypoints,
        "assistant2_waypoints": assistant2_waypoints,
        "gambler1_distance": gambler1_distance,
        "assistant1_distance": assistant1_distance,
        "gambler2_distance": gambler2_distance,
        "assistant2_distance": assistant2_distance,
        "total_distance": gambler1_distance + assistant1_distance + gambler2_distance + assistant2_distance,
        "hit_cap": hit_cap,
        "frames": frames,
    }


def test_workflow_terminates_cleanly(shape_size_ft=None, record_frames=False):
    pf = build_empty_pathfinder()
    true_mines = generate_true_minefield()
    result = simulate_one_drone(
        pf, true_mines, shape_size_ft=shape_size_ft, record_frames=record_frames
    )

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
    ok = (not result["hit_cap"]) and result["replans"] <= len(
        result["discovered"]
    ) + max_cleanup_passes
    print(
        f"test_workflow_terminates_cleanly: replans={result['replans']} "
        f"discovered={len(result['discovered'])} waypoints={result['total_waypoints']} "
        f"hit_cap={result['hit_cap']} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok, pf, true_mines, result


def test_final_queue_is_empty(pf):
    places = pf.getPlacesToCheck()
    ok = places == []
    print(f"test_final_queue_is_empty: -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_seen_covers_final_path(pf):
    """The concrete form of "seen graph == field graph": the FINAL shortest
    path's own cell footprint must be a subset of what's marked seen --
    recomputed independently via rasterize_node_path/bitwise ops, not by trusting
    getPlacesToCheck's own internal accounting."""
    path = pf.get_shortest_path()
    path_footprint = pf.rasterize_node_path(path)
    unseen_on_path = path_footprint & ~pf.seen_tracker
    ok = unseen_on_path.count() == 0
    print(
        f"test_seen_covers_final_path: path_cells={path_footprint.count()} "
        f"unseen_on_path={unseen_on_path.count()} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_discovered_mines_match_true_mines_found(pf, true_mines, result):
    """Every true mine that was actually flown over got added to the field
    exactly once (no duplicates, none missed) -- cross-checked against the
    field's own live mine set (standalone + nested in any union, the same
    way Field.mineHash collects them)."""
    discovered = result["discovered"]
    all_field_mines = list(pf.nodeField.mines) + pf.nodeField._collect_mines(
        pf.nodeField.unionObstacles
    )

    ok = len(discovered) > 0  # this minefield/path combo should find at least one
    for mx, my in discovered:
        nearest = min(math.hypot(m.origin[0] - mx, m.origin[1] - my) for m in all_field_mines)
        ok = ok and nearest <= WIDTHOFSQUARE * math.sqrt(2)
    # merges can only ever reduce live-mine count relative to discoveries,
    # never inflate it past the number of distinct detections made
    ok = ok and len(all_field_mines) <= len(discovered)
    print(
        f"test_discovered_mines_match_true_mines_found: true_mines={len(true_mines)} "
        f"discovered={len(discovered)} live_field_mines={len(all_field_mines)} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_pair_terminates_cleanly(shape_size_ft=None, record_frames=False, **kwargs):
    """Same idea as test_workflow_terminates_cleanly, but for
    simulate_one_pair_maze -- bound by rounds instead of replans (a round
    is both roles getting a turn, so the same "no worse than one round per
    discovery plus a little cleanup" allowance applies)."""
    pf = build_empty_pathfinder()
    true_mines = generate_true_minefield()
    result = simulate_one_pair_maze(
        pf, true_mines, shape_size_ft=shape_size_ft, record_frames=record_frames, **kwargs
    )
    max_cleanup_rounds = max(2, round(result["total_waypoints"] / 25))
    ok = (not result["hit_cap"]) and result["steps"] <= len(result["discovered"]) + max_cleanup_rounds
    print(
        f"test_pair_terminates_cleanly: rounds={result['steps']} "
        f"discovered={len(result['discovered'])} waypoints={result['total_waypoints']} "
        f"hit_cap={result['hit_cap']} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok, pf, true_mines, result


def test_pair_seen_covers_final_path(pf):
    """Same check as test_seen_covers_final_path, against
    pf.get_maze_path() (the route actually committed to/checked so far)
    instead of pf.get_shortest_path() -- the right target for maze-style
    (including pair) runs, same as simulate_one_drone_maze's own sweep
    scripts already check ad hoc."""
    path = pf.get_maze_path()
    path_footprint = pf.rasterize_node_path(path)
    unseen_on_path = path_footprint & ~pf.seen_tracker
    ok = unseen_on_path.count() == 0
    print(
        f"test_pair_seen_covers_final_path: path_cells={path_footprint.count()} "
        f"unseen_on_path={unseen_on_path.count()} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


# 1e-6: skip a midpoint sitting exactly ON an obstacle's own boundary --
# floating-point noise from the geometry pipeline, not a real crossing.
_BAD_EDGE_BOUNDARY_ARTIFACT_DIST = 1e-6


def _bad_edge_count(pf, path):
    """How many consecutive-node edges in `path` have their midpoint
    genuinely inside a live obstacle's safety polygon -- the actual safety
    invariant maze-mode planning must never violate (a drone flying that
    edge would cross into a mine's own danger radius), independent of
    which specific driver produced `path`. Used by both
    test_pair_flown_path_is_safe and test_leader_follower_flown_path_is_safe
    so a real bad edge can never slip through un-caught the way it did
    earlier this session (see pathfinder.py's check_path_envelopment /
    _prepend_seam_if_needed docstrings for the bugs this exact check
    caught)."""
    obstacles = list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles)
    bad = 0
    for i in range(len(path) - 1):
        p1, p2 = path[i], path[i + 1]
        mx, my = (p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0
        for o in obstacles:
            if not o.contains_point((mx, my)):
                continue
            dist = o.polygon.boundary.distance(Point(mx, my))
            if dist < _BAD_EDGE_BOUNDARY_ARTIFACT_DIST:
                continue
            bad += 1
            break
    return bad


def test_pair_flown_path_is_safe(pf):
    """The actually-flown route (pf.get_maze_path()) never cuts through a
    live obstacle's safety polygon -- the concrete form of "a drone
    following this exact route never enters a mine's danger radius".
    Works for any single-Pathfinder maze-mode driver's result (both
    simulate_one_pair_maze and simulate_leader_follower_pair use it)."""
    path = pf.get_maze_path()
    bad = _bad_edge_count(pf, path)
    ok = bad == 0
    print(f"test_pair_flown_path_is_safe: path_edges={len(path)-1} bad_edges={bad} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_leader_follower_terminates_cleanly(shape_size_ft=None, record_frames=False, **kwargs):
    """Same idea as test_pair_terminates_cleanly, but for
    simulate_leader_follower_pair -- the leader/follower device-boundary
    restructuring (see that function's own docstring) should terminate
    with the exact same round/waypoint/discovery behavior as the
    shared-object version, since it calls the same underlying Pathfinder
    methods on the same sequence of positions; this and
    test_pair_terminates_cleanly are expected to report identical numbers
    for the same true_mines/seed."""
    pf = build_empty_pathfinder()
    true_mines = generate_true_minefield()
    result = simulate_leader_follower_pair(
        pf, true_mines, shape_size_ft=shape_size_ft, record_frames=record_frames, **kwargs
    )
    max_cleanup_rounds = max(2, round(result["total_waypoints"] / 25))
    ok = (not result["hit_cap"]) and result["steps"] <= len(result["discovered"]) + max_cleanup_rounds
    print(
        f"test_leader_follower_terminates_cleanly: rounds={result['steps']} "
        f"discovered={len(result['discovered'])} waypoints={result['total_waypoints']} "
        f"hit_cap={result['hit_cap']} -> {'PASS' if ok else 'FAIL'}"
    )
    return ok, pf, true_mines, result


def render_workflow_diagram(pf, true_mines, result, save_path):
    """
    One image showing the whole simulated mission:
      - the shortest path's cell footprint, rasterized onto a CellField
        (black blocks) via rasterize_node_path/mark_path
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
    path_footprint = pf.rasterize_node_path(path)
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
    ax.imshow(
        seen_arr,
        cmap=ListedColormap(["none", "#8fd19e"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
        origin="lower",
        alpha=0.55,
        extent=extent,
        zorder=1,
    )
    ax.imshow(
        path_arr,
        cmap=ListedColormap(["none", "black"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
        origin="lower",
        alpha=0.9,
        extent=extent,
        zorder=2,
    )
    ax.imshow(
        mine_blocks_arr,
        cmap=ListedColormap(["none", "darkorange"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
        origin="lower",
        alpha=0.6,
        extent=extent,
        zorder=3,
    )

    # The node-graph polygon itself, kept as a real vector shape (not
    # rasterized) -- includes merged unionObstacles, each still exposing
    # .vertices the same way a standalone BlockMine does.
    for obstacle in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles):
        ax.add_patch(
            MplPolygon(
                list(obstacle.vertices),
                closed=True,
                facecolor="none",
                edgecolor="firebrick",
                linewidth=1.3,
                zorder=3.5,
            )
        )

    discovered = result["discovered"]
    for mx, my in true_mines:
        if (mx, my) in discovered:
            continue
        ax.add_patch(
            Circle(
                (mx, my),
                WIDTHOFSQUARE * 1.5,
                facecolor="none",
                edgecolor="gray",
                linestyle="--",
                linewidth=1.2,
                zorder=4,
            )
        )

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


def render_maze_workflow_diagram(pf, true_mines, result, save_path):
    """
    Same picture as render_workflow_diagram, but for a simulate_one_drone_maze
    result instead of simulate_one_drone's: no "replans" counter (maze uses
    "steps", one per visit-pass over segment A or B), and the flight trail
    carries a segment label ("A"/"B") per point instead of a replan
    generation -- colored accordingly (gold=A/approach, steel blue=B/bridge)
    so the C/B/A structure is visible directly in the plot, not just
    inferable from a color gradient.

    Takes: pf (post-mission Pathfinder), true_mines (ground truth), result
      (dict from simulate_one_drone_maze: discovered/visited/steps/
      total_waypoints/hit_cap).
    Does: renders seen/path/mine layers identically to
      render_workflow_diagram, then the maze-specific trail/path coloring.
    Returns: nothing, saves a PNG to save_path.
    """
    import numpy as np
    from matplotlib.colors import ListedColormap

    path = pf.get_maze_path()
    path_footprint = pf.rasterize_node_path(path)
    width_cells, height_cells = path_footprint.width, path_footprint.height
    extent = (0, WIDTHOFFIELD, 0, HEIGHTOFFIELD)

    seen_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in pf.seen_tracker.on_cells():
        seen_arr[y, x] = 1

    path_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in path_footprint.on_cells():
        path_arr[y, x] = 1

    mine_blocks_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
    for x, y in pf.mineFieldTracker.on_cells():
        mine_blocks_arr[y, x] = 1

    fig, ax = plt.subplots(figsize=(8, 20), dpi=100)
    ax.imshow(
        seen_arr, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
        interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1,
    )
    ax.imshow(
        path_arr, cmap=ListedColormap(["none", "black"]), vmin=0, vmax=1,
        interpolation="nearest", origin="lower", alpha=0.9, extent=extent, zorder=2,
    )
    ax.imshow(
        mine_blocks_arr, cmap=ListedColormap(["none", "darkorange"]), vmin=0, vmax=1,
        interpolation="nearest", origin="lower", alpha=0.6, extent=extent, zorder=3,
    )

    for obstacle in list(pf.nodeField.mines) + list(pf.nodeField.unionObstacles):
        ax.add_patch(
            MplPolygon(
                list(obstacle.vertices), closed=True, facecolor="none",
                edgecolor="firebrick", linewidth=1.3, zorder=3.5,
            )
        )

    discovered = result["discovered"]
    for mx, my in true_mines:
        if (mx, my) in discovered:
            continue
        ax.add_patch(
            Circle(
                (mx, my), WIDTHOFSQUARE * 1.5, facecolor="none",
                edgecolor="gray", linestyle="--", linewidth=1.2, zorder=4,
            )
        )

    label_color = {"A": "#c9972b", "B": "#2e6f95"}
    visited = result["visited"]
    for i in range(len(visited) - 1):
        x0, y0, s0, lab0 = visited[i]
        x1, y1, s1, lab1 = visited[i + 1]
        if lab0 != lab1:
            continue  # don't draw a line across a mode switch
        ax.plot(
            [x0, x1], [y0, y1], color=label_color[lab0], linewidth=1.0,
            zorder=5, alpha=0.8,
        )
    vxs = [v[0] for v in visited]
    vys = [v[1] for v in visited]
    vcolors = [label_color[v[3]] for v in visited]
    ax.scatter(vxs, vys, c=vcolors, s=14, zorder=6, edgecolors="white", linewidths=0.3)

    # Final path: the confirmed prefix (segment C, what get_maze_path folds
    # start..point_C into) drawn separately from the trailing approach stub
    # (segment A, the final edge to the true end) -- at mission completion B
    # is always empty (drained into C), so the path is just these two runs.
    c_path = pf.maze_confirmed_path
    if c_path:
        cxs = [n.x for n in c_path]
        cys = [n.y for n in c_path]
        ax.plot(cxs, cys, color="black", linewidth=1.8, zorder=7, alpha=0.9)
    a_path = pf.maze_a_path
    if a_path:
        axs = [n.x for n in a_path]
        ays = [n.y for n in a_path]
        ax.plot(axs, ays, color="#c9972b", linewidth=1.8, zorder=7, alpha=0.95)

    ax.set_xlim(0, WIDTHOFFIELD)
    ax.set_ylim(0, HEIGHTOFFIELD)
    ax.set_aspect("equal")
    ax.set_title(
        f"Maze-style incremental replanning simulation\n"
        f"{len(discovered)}/{len(true_mines)} true mines found over {result['steps']} steps, "
        f"{result['total_waypoints']} photos taken",
        fontsize=11,
    )
    ax.set_xlabel(
        "Green = seen   |   Black cells = path footprint   |   Red outline = mine node-graph polygon\n"
        "Orange cells = mine safety-radius blocks (mineFieldTracker)   |   Dashed gray = undiscovered true mine\n"
        "Gold dots/line = segment A (approach)   |   Blue dots = segment B (bridge)   |   Black line = segment C (confirmed)",
        fontsize=9,
    )
    ax.set_xticks([])
    ax.set_yticks([])

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)


def render_maze_workflow_gif(
    pf, true_mines, result, save_path, max_frames=300, frame_duration_ms=90, hold_last_frames=6,
    dual_role_markers=False,
):
    """
    Same idea as render_workflow_gif, but animates simulate_one_drone_maze's
    (or simulate_one_pair_maze's) recorded snapshots (see
    _snapshot_maze_frame -- requires it was called with record_frames=True).
    One frame per individual photo actually taken (not just one per step),
    plus one at the end of every step -- so watching the GIF shows exactly
    where the drone checks next each time, not just where segment C ends up
    after a whole batch. max_frames defaults much higher than
    render_workflow_gif's since a real run can have hundreds of individual
    checks and the whole point here is seeing them; frame_duration_ms
    defaults faster to keep total playback reasonable at that frame count.

    dual_role_markers: False (default, single-drone) highlights just the
    single most recent photo. True (pair runs) highlights the gambler's
    (segment A) and assistant's (segment B) most recent photo SEPARATELY --
    two simultaneous positions, since they're two physically distinct
    drones -- instead of one, which would otherwise misleadingly suggest a
    single drone jumping between segments.
    """
    import io
    import numpy as np
    from PIL import Image
    from matplotlib.colors import ListedColormap

    frames_data = result["frames"]
    if not frames_data:
        raise ValueError(
            "render_maze_workflow_gif needs frames -- call simulate_one_drone_maze with record_frames=True"
        )

    if len(frames_data) > max_frames:
        idxs = sorted(
            {round(i * (len(frames_data) - 1) / (max_frames - 1)) for i in range(max_frames)}
        )
        frames_data = [frames_data[i] for i in idxs]

    width_cells = WIDTHOFFIELD // WIDTHOFSQUARE
    height_cells = HEIGHTOFFIELD // WIDTHOFSQUARE
    extent = (0, WIDTHOFFIELD, 0, HEIGHTOFFIELD)
    label_color = {"A": "#c9972b", "B": "#2e6f95"}

    pil_frames = []
    for frame in frames_data:
        seen_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["seen"].on_cells():
            seen_arr[y, x] = 1
        mine_blocks_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["mine_blocks"].on_cells():
            mine_blocks_arr[y, x] = 1

        fig, ax = plt.subplots(figsize=(6, 15), dpi=80)
        ax.imshow(
            seen_arr, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
            interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1,
        )
        ax.imshow(
            mine_blocks_arr, cmap=ListedColormap(["none", "darkorange"]), vmin=0, vmax=1,
            interpolation="nearest", origin="lower", alpha=0.6, extent=extent, zorder=2,
        )

        for verts in frame["polygons"]:
            ax.add_patch(
                MplPolygon(verts, closed=True, facecolor="none", edgecolor="firebrick", linewidth=1.1, zorder=2.5)
            )

        discovered = frame["discovered"]
        for mx, my in true_mines:
            if (mx, my) in discovered:
                continue
            ax.add_patch(
                Circle((mx, my), WIDTHOFSQUARE * 1.5, facecolor="none", edgecolor="gray",
                       linestyle="--", linewidth=1.0, zorder=3)
            )

        visited = frame["visited"]
        for i in range(len(visited) - 1):
            x0, y0, s0, lab0 = visited[i]
            x1, y1, s1, lab1 = visited[i + 1]
            if lab0 != lab1:
                continue
            ax.plot([x0, x1], [y0, y1], color=label_color[lab0], linewidth=1.0, zorder=4, alpha=0.85)
        if visited:
            ax.scatter(
                [v[0] for v in visited], [v[1] for v in visited],
                c=[label_color[v[3]] for v in visited], s=10, zorder=5, edgecolors="white", linewidths=0.2,
            )

        c_path = frame["c_path"]
        if c_path:
            ax.plot([p[0] for p in c_path], [p[1] for p in c_path], color="black", linewidth=1.6, zorder=6, alpha=0.9)
        a_path = frame["a_path"]
        if a_path:
            ax.plot([p[0] for p in a_path], [p[1] for p in a_path], color="#c9972b", linewidth=1.6, zorder=6, alpha=0.95)

        # Highlight ring(s) on the most recent photo -- "the drone is HERE
        # right now" -- so a single frame reads as one specific check, not
        # just another dot lost in the growing scatter. dual_role_markers
        # shows the gambler's and assistant's latest position separately
        # (two real, simultaneous drones); otherwise just the single most
        # recent photo overall.
        role_marker_color = {"A": "magenta", "B": "darkorchid"}
        cur_xy = None
        role_positions = {}
        if dual_role_markers:
            for label in ("A", "B"):
                entry = next((v for v in reversed(visited) if v[3] == label), None)
                if entry is not None:
                    role_positions[label] = (entry[0], entry[1])
                    ax.add_patch(
                        Circle(role_positions[label], WIDTHOFSQUARE * 2.2, facecolor="none",
                               edgecolor=role_marker_color[label], linewidth=1.8, zorder=7)
                    )
        elif visited:
            cur_xy = (visited[-1][0], visited[-1][1])
            ax.add_patch(
                Circle(cur_xy, WIDTHOFSQUARE * 2.2, facecolor="none", edgecolor="magenta", linewidth=1.8, zorder=7)
            )

        # Helper-node machinery, if use_helper_nodes was on: the trail of
        # not-yet-promoted candidate breadcrumbs (small hollow gray
        # diamonds -- these are just tracked positions, never graph
        # nodes) and the currently-promoted real graph nodes (solid cyan
        # diamonds). A promoted node simply stops appearing in later
        # frames if check_path_envelopment later removes it --
        # that disappearance IS the "deleted" event, there's no separate
        # marker for it.
        helper_trail = frame.get("helper_trail", [])
        if helper_trail:
            ax.scatter(
                [p[0] for p in helper_trail], [p[1] for p in helper_trail],
                marker="D", s=18, facecolors="none", edgecolors="dimgray", linewidths=0.9, zorder=6.5,
            )
        promoted = frame.get("promoted_helper_nodes", [])
        if promoted:
            ax.scatter(
                [p[0] for p in promoted], [p[1] for p in promoted],
                marker="D", s=34, facecolors="cyan", edgecolors="black", linewidths=0.8, zorder=6.6,
            )

        ax.set_xlim(0, WIDTHOFFIELD)
        ax.set_ylim(0, HEIGHTOFFIELD)
        ax.set_aspect("equal")
        title = f"step {frame['step']}/{result['steps']}   {len(discovered)}/{len(true_mines)} mines found"
        if dual_role_markers:
            if "A" in role_positions:
                title += f"   gambler ({role_positions['A'][0]:.0f}, {role_positions['A'][1]:.0f})"
            if "B" in role_positions:
                title += f"   assistant ({role_positions['B'][0]:.0f}, {role_positions['B'][1]:.0f})"
        elif cur_xy:
            title += f"   checking ({cur_xy[0]:.0f}, {cur_xy[1]:.0f})"
        if helper_trail or promoted:
            title += f"\ntrail={len(helper_trail)} hollow gray, promoted={len(promoted)} solid cyan"
        ax.set_title(title, fontsize=10)
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
        save_path,
        save_all=True,
        append_images=pil_frames[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=True,
    )


def render_two_pairs_workflow_gif(
    true_mines, result, save_path, max_frames=300, frame_duration_ms=90, hold_last_frames=6
):
    """
    Same idea as render_maze_workflow_gif, but for simulate_two_pairs_maze's
    recorded snapshots (_snapshot_two_pair_frame -- requires it was called
    with record_frames=True) -- four drones instead of two, so eight colors:
    each pair's gambler/assistant trail dots use its own gold/blue-family
    pair (pair 1: the same gold/steel-blue single-pair GIFs already use;
    pair 2: brick-red/green, so the two pairs stay visually distinct even
    where their paths cross near the middle of the field), plus four
    "currently here" rings (one per drone) instead of two.
    """
    import io
    import numpy as np
    from PIL import Image
    from matplotlib.colors import ListedColormap

    frames_data = result["frames"]
    if not frames_data:
        raise ValueError(
            "render_two_pairs_workflow_gif needs frames -- call simulate_two_pairs_maze with record_frames=True"
        )

    if len(frames_data) > max_frames:
        idxs = sorted(
            {round(i * (len(frames_data) - 1) / (max_frames - 1)) for i in range(max_frames)}
        )
        frames_data = [frames_data[i] for i in idxs]

    width_cells = WIDTHOFFIELD // WIDTHOFSQUARE
    height_cells = HEIGHTOFFIELD // WIDTHOFSQUARE
    extent = (0, WIDTHOFFIELD, 0, HEIGHTOFFIELD)
    # (label, pair_id) -> trail/scatter color.
    label_color = {
        ("A", 1): "#c9972b", ("B", 1): "#2e6f95",
        ("A", 2): "#c0392b", ("B", 2): "#27ae60",
    }
    # (label, pair_id) -> "currently here" ring color.
    ring_color = {
        ("A", 1): "magenta", ("B", 1): "darkorchid",
        ("A", 2): "orange", ("B", 2): "cyan",
    }
    role_name = {("A", 1): "gambler1", ("B", 1): "assistant1", ("A", 2): "gambler2", ("B", 2): "assistant2"}

    pil_frames = []
    for frame in frames_data:
        seen_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["seen"].on_cells():
            seen_arr[y, x] = 1
        mine_blocks_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["mine_blocks"].on_cells():
            mine_blocks_arr[y, x] = 1

        fig, ax = plt.subplots(figsize=(6, 15), dpi=80)
        ax.imshow(
            seen_arr, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
            interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1,
        )
        ax.imshow(
            mine_blocks_arr, cmap=ListedColormap(["none", "darkorange"]), vmin=0, vmax=1,
            interpolation="nearest", origin="lower", alpha=0.6, extent=extent, zorder=2,
        )

        for verts in frame["polygons"]:
            ax.add_patch(
                MplPolygon(verts, closed=True, facecolor="none", edgecolor="firebrick", linewidth=1.1, zorder=2.5)
            )

        discovered = frame["discovered"]
        for mx, my in true_mines:
            if (mx, my) in discovered:
                continue
            ax.add_patch(
                Circle((mx, my), WIDTHOFSQUARE * 1.5, facecolor="none", edgecolor="gray",
                       linestyle="--", linewidth=1.0, zorder=3)
            )

        visited = frame["visited"]
        for i in range(len(visited) - 1):
            x0, y0, s0, lab0, pid0 = visited[i]
            x1, y1, s1, lab1, pid1 = visited[i + 1]
            if lab0 != lab1 or pid0 != pid1:
                continue
            ax.plot([x0, x1], [y0, y1], color=label_color[(lab0, pid0)], linewidth=1.0, zorder=4, alpha=0.85)
        if visited:
            ax.scatter(
                [v[0] for v in visited], [v[1] for v in visited],
                c=[label_color[(v[3], v[4])] for v in visited], s=10, zorder=5, edgecolors="white", linewidths=0.2,
            )

        for pid in (1, 2):
            c_path = frame[f"c_path_{pid}"]
            if c_path:
                ax.plot([p[0] for p in c_path], [p[1] for p in c_path], color="black", linewidth=1.6, zorder=6, alpha=0.9)
            a_path = frame[f"a_path_{pid}"]
            if a_path:
                ax.plot(
                    [p[0] for p in a_path], [p[1] for p in a_path],
                    color=label_color[("A", pid)], linewidth=1.6, zorder=6, alpha=0.95,
                )

        # One "currently here" ring per drone -- four real, simultaneous
        # positions, same reasoning as render_maze_workflow_gif's
        # dual_role_markers, just doubled for the second pair.
        role_positions = {}
        for key in (("A", 1), ("B", 1), ("A", 2), ("B", 2)):
            entry = next((v for v in reversed(visited) if (v[3], v[4]) == key), None)
            if entry is not None:
                role_positions[key] = (entry[0], entry[1])
                ax.add_patch(
                    Circle(role_positions[key], WIDTHOFSQUARE * 2.2, facecolor="none",
                           edgecolor=ring_color[key], linewidth=1.8, zorder=7)
                )

        ax.set_xlim(0, WIDTHOFFIELD)
        ax.set_ylim(0, HEIGHTOFFIELD)
        ax.set_aspect("equal")
        title = f"step {frame['step']}/{result['steps']}   {len(discovered)}/{len(true_mines)} mines found"
        for key, pos in role_positions.items():
            title += f"\n{role_name[key]} ({pos[0]:.0f}, {pos[1]:.0f})" if key == ("A", 1) else f"   {role_name[key]} ({pos[0]:.0f}, {pos[1]:.0f})"
        ax.set_title(title, fontsize=9)
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
        save_path,
        save_all=True,
        append_images=pil_frames[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=True,
    )


def render_workflow_gif(
    pf, true_mines, result, save_path, max_frames=40, frame_duration_ms=350, hold_last_frames=4
):
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
        raise ValueError(
            "render_workflow_gif needs frames -- call simulate_one_drone with record_frames=True"
        )

    if len(frames_data) > max_frames:
        idxs = sorted(
            {round(i * (len(frames_data) - 1) / (max_frames - 1)) for i in range(max_frames)}
        )
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
        ax.imshow(
            seen_arr,
            cmap=ListedColormap(["none", "#8fd19e"]),
            vmin=0,
            vmax=1,
            interpolation="nearest",
            origin="lower",
            alpha=0.55,
            extent=extent,
            zorder=1,
        )
        ax.imshow(
            mine_blocks_arr,
            cmap=ListedColormap(["none", "darkorange"]),
            vmin=0,
            vmax=1,
            interpolation="nearest",
            origin="lower",
            alpha=0.6,
            extent=extent,
            zorder=2,
        )

        for verts in frame["polygons"]:
            ax.add_patch(
                MplPolygon(
                    verts,
                    closed=True,
                    facecolor="none",
                    edgecolor="firebrick",
                    linewidth=1.1,
                    zorder=2.5,
                )
            )

        discovered = frame["discovered"]
        for mx, my in true_mines:
            if (mx, my) in discovered:
                continue
            ax.add_patch(
                Circle(
                    (mx, my),
                    WIDTHOFSQUARE * 1.5,
                    facecolor="none",
                    edgecolor="gray",
                    linestyle="--",
                    linewidth=1.0,
                    zorder=3,
                )
            )

        visited = frame["visited"]
        for i in range(len(visited) - 1):
            x0, y0, g0 = visited[i]
            x1, y1, g1 = visited[i + 1]
            if g0 != g1:
                continue  # don't draw a line across a replan jump
            color = plt.get_cmap("plasma")(g0 / max_gen)
            ax.plot([x0, x1], [y0, y1], color=color, linewidth=1.0, zorder=4, alpha=0.85)
        if visited:
            ax.scatter(
                [v[0] for v in visited],
                [v[1] for v in visited],
                c=[v[2] for v in visited],
                cmap="plasma",
                vmin=1,
                vmax=max_gen,
                s=10,
                zorder=5,
                edgecolors="white",
                linewidths=0.2,
            )

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
        save_path,
        save_all=True,
        append_images=pil_frames[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=True,
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
    print(
        f"true_mines={len(true_mines)} shape={shape_size_ft[0]}x{shape_size_ft[1]}ft "
        f"replans={result['replans']} waypoints={result['total_waypoints']} elapsed={elapsed:.3f}s"
    )

    diagram_path = SCRATCH_DIR + r"\droneWorkflow_combined.png"
    render_workflow_diagram(pf, true_mines, result, diagram_path)
    print(f"saved workflow diagram to {diagram_path}")

    gif_path = SCRATCH_DIR + r"\droneWorkflow_process.gif"
    render_workflow_gif(pf, true_mines, result, gif_path)
    print(f"saved workflow gif to {gif_path}")


if __name__ == "__main__":
    main()
