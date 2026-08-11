# This class needs to initialize and store the following as member variables:
# Arbitrary coordinate code object
# Field object
# Best path object
# Sight Tracker object

# As a black box the object basically only needs the following functions + whatever private functions are needed to make the following work:
# Accept mission's current corner coordinates during its initialization.
# Accept corner coordinates of any image taken and update "already seen" mat.
# Add discovered mines to field given their lat/long position
# Return lat/long waypoints of waypoints that need to be visited.

# The class needs a static variable used to store an instance of itself for use in the state machine.

# .. and the constants for arbitrary things like end node density etc.
import math

from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer
from flight.pathfinding.path_subdivision import Path
from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.nodeField.node import Node
from flight.pathfinding.path_calculation import Graph
from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.protoMine import protoMine
from flight.pathfinding.path_cover import path_cover_unseen, _normalize_shape_size
from flight.pathfinding.mineCellField import build_mine_cell_field

simWidth = 100


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _nearest_neighbor_tour(coords: list[tuple[float, float]], start_idx: int) -> list[int]:
    n = len(coords)
    unvisited = set(range(n))
    unvisited.remove(start_idx)
    tour = [start_idx]
    current = start_idx
    while unvisited:
        nxt = min(unvisited, key=lambda i: _dist(coords[current], coords[i]))
        tour.append(nxt)
        unvisited.remove(nxt)
        current = nxt
    return tour


def _tour_length(coords: list[tuple[float, float]], tour: list[int]) -> float:
    return sum(_dist(coords[tour[i]], coords[tour[i + 1]]) for i in range(len(tour) - 1))


def _two_opt(
    coords: list[tuple[float, float]], tour: list[int], max_passes: int = 200
) -> list[int]:
    # Open-path 2-opt: repeatedly reverse a sub-segment if that shortens the
    # tour, until a full pass finds no improvement or max_passes is hit.
    n = len(tour)
    for _ in range(max_passes):
        improved = False
        for i in range(n - 1):
            a, b = tour[i], tour[i + 1]
            old_ab = _dist(coords[a], coords[b])
            for j in range(i + 2, n):
                c = tour[j]
                d = tour[j + 1] if j + 1 < n else None
                old_cost = old_ab
                new_cost = _dist(coords[a], coords[c])
                if d is not None:
                    old_cost += _dist(coords[c], coords[d])
                    new_cost += _dist(coords[b], coords[d])
                if new_cost < old_cost - 1e-9:
                    tour[i + 1 : j + 1] = reversed(tour[i + 1 : j + 1])
                    improved = True
                    break
            if improved:
                break
        if not improved:
            break
    return tour


def _find_matching_index(
    coords: list[tuple[float, float]], point: tuple[float, float], tol: float = 1e-6
) -> int | None:
    for i, c in enumerate(coords):
        if abs(c[0] - point[0]) <= tol and abs(c[1] - point[1]) <= tol:
            return i
    return None


def order_waypoints(
    coords: list[tuple[float, float]], fixed_first: tuple[float, float] = None
) -> list[tuple[float, float]]:
    """Reorders local (x,y) coords into a short visiting tour (nearest-
    neighbor + 2-opt). fixed_first, if given, pins the tour's start there
    (matched by tolerance, or used as an external unreturned anchor if not
    present in coords) so a replan doesn't reorder the first stop."""
    if fixed_first is None:
        n = len(coords)
        if n <= 2:
            return list(coords)

        max_starts = min(n, 20)
        start_indices = sorted({round(i * (n - 1) / (max_starts - 1)) for i in range(max_starts)})

        best_tour = None
        best_length = None
        for start_idx in start_indices:
            tour = _nearest_neighbor_tour(coords, start_idx)
            length = _tour_length(coords, tour)
            if best_length is None or length < best_length:
                best_tour, best_length = tour, length

        best_tour = _two_opt(coords, best_tour)
        return [coords[i] for i in best_tour]

    match_idx = _find_matching_index(coords, fixed_first)
    if match_idx is not None:
        working = list(coords)
        start_idx = match_idx
        prepended = False
    else:
        working = [fixed_first] + list(coords)
        start_idx = 0
        prepended = True

    n = len(working)
    if n <= 2:
        tour = [start_idx] + [i for i in range(n) if i != start_idx]
    else:
        tour = _two_opt(working, _nearest_neighbor_tour(working, start_idx))

    result = [working[i] for i in tour]
    return result[1:] if prepended else result


def path_length(coords: list[tuple[float, float]]) -> float:
    """Takes: local (x,y) coords in order. Returns: total polyline length (0.0 if <2 points)."""
    return sum(_dist(coords[i], coords[i + 1]) for i in range(len(coords) - 1))


def _footprint_box(
    cx: float, cy: float, along_ft: float, across_ft: float
) -> tuple[float, float, float, float]:
    """Takes: center (cx,cy), along/across extents in feet. Returns: axis-aligned
    (x0,y0,x1,y1) box a photo there would cover (along -> y, across -> x)."""
    half_along, half_across = along_ft / 2.0, across_ft / 2.0
    return cx - half_across, cy - half_along, cx + half_across, cy + half_along


def node_path_length(nodePath) -> float:
    """Takes: list of objects with .x/.y (e.g. graph Nodes). Returns: total path length."""
    return path_length([(n.x, n.y) for n in nodePath])


WIDTHOFSQUARE = 2
WIDTHOFFIELD = 80
HEIGHTOFFIELD = 300

# Hysteresis margin for ending-node choice (fraction of best route length) --
# see _dijkstra_path_with_hysteresis. 0.03 covers the sub-1ft near-ties seen
# between candidate ending nodes on a ~300ft field, without masking a real
# multi-foot detour.
PATH_HYSTERESIS_TOLERANCE = 0.03

# Cap on simultaneous outstanding (not yet confirmed into C) helper-node
# detours -- see start_helper_node_detour. Measured empirically (200 pair
# seeds): real chains almost never exceed depth 3 on their own, so this
# cap is rarely the thing actually limiting gambler -- it exists as a
# backstop against a pathological run letting gambler race arbitrarily
# far ahead of an assistant that's stuck reworking a difficult stretch.
MAX_CHAIN_DEPTH = 3

# Below this remaining distance, Pathfinder._try_apply_pending_approach_target
# closes the rest of the way to a cross-pair target instead of halving it
# again. Halving unconditionally is a real Zeno's-paradox convergence
# problem: repeatedly closing "half the remaining gap" gets arbitrarily
# close to zero but never actually REACHES it, so two pairs converging
# this way can take unboundedly many rounds and, in the worst case, leave
# a permanent single-cell coverage gap exactly at the sliver neither
# pair's target ever quite reaches -- confirmed directly on the two-pair
# safety sweep (hit_cap and a real coverage_fail both appeared only once
# halving was introduced). WIDTHOFSQUARE * 4 is a small multiple of the
# base grid cell, not tied to any particular camera footprint size.
CLOSE_ENOUGH_TO_STOP_HALVING_FT = WIDTHOFSQUARE * 4


class Pathfinder:
    # Checkpoint-pinned path planning is archived: see
    # flight/pathfinding/archive/pathfinder_unused.py.
    instance: "Pathfinder" = None  # Static variable for singleton access

    def __init__(
        self,
        real_corner_coords: tuple[tuple[float, float]],
        altitude: float,
        fov_deg: float,
        droneID: int,
    ):
        self.droneID = droneID
        self.SIM_WIDTH: float = (
            2  # Confirm with nat what this is exactly, this should be an internal constant
        )

        self.OVERLAP = 3  # This will be an internal constant

        self.coord_converter = SimToLatLonTransformer(real_corner_coords, WIDTHOFFIELD)

        self.arb_corner_coords = self.coord_converter.get_arb_corners()

        self.arb_field_size = [
            max(
                [
                    self.arb_corner_coords[0][0],
                    self.arb_corner_coords[1][0],
                    self.arb_corner_coords[2][0],
                    self.arb_corner_coords[3][0],
                ]
            )
            - min(
                [
                    self.arb_corner_coords[0][0],
                    self.arb_corner_coords[1][0],
                    self.arb_corner_coords[2][0],
                    self.arb_corner_coords[3][0],
                ]
            ),
            max(
                [
                    self.arb_corner_coords[0][1],
                    self.arb_corner_coords[1][1],
                    self.arb_corner_coords[2][1],
                    self.arb_corner_coords[3][1],
                ]
            )
            - min(
                [
                    self.arb_corner_coords[0][1],
                    self.arb_corner_coords[1][1],
                    self.arb_corner_coords[2][1],
                    self.arb_corner_coords[3][1],
                ]
            ),
        ]

        self.mine_saftey_radius = 3  # Get from the coord converter

        self.real_corner_coords = real_corner_coords

        # Field expects [top-left, top-right, bottom-left, bottom-right], not
        # arb_corner_coords' own [origin, +x, diagonal, +y] order.
        field_corners = [
            self.arb_corner_coords[3],  # +y / top-left
            self.arb_corner_coords[2],  # diagonal / top-right
            self.arb_corner_coords[0],  # origin / bottom-left
            self.arb_corner_coords[1],  # +x / bottom-right
        ]
        self.nodeField = Field(self.arb_field_size, field_corners, droneNumber=self.droneID)
        self.mineFieldTracker = CellField(
            WIDTHOFFIELD // WIDTHOFSQUARE,
            HEIGHTOFFIELD // WIDTHOFSQUARE,
            max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD),
        )

        self.seen_tracker = CellField(
            WIDTHOFFIELD // WIDTHOFSQUARE,
            HEIGHTOFFIELD // WIDTHOFSQUARE,
            max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD),
        )  # This is a placeholder, replace with the actual SightTracker object

        self.path_tracker = CellField(
            WIDTHOFFIELD // WIDTHOFSQUARE,
            HEIGHTOFFIELD // WIDTHOFSQUARE,
            max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD),
        )  # This is a placeholder, replace with the actual SightTracker object
        # self.seen_tracker = seen_by_drone.SightTracker(self.arb_field_size)

        # self.seen_tracker.note_field_borders(self.arb_corner_coords)



        self.best_node_List = []

        self.startingNodes = []
        self.endingNodes = []
        self._original_ending_nodes = None
        self.protoMines = []

        self.best_path = Path()
        self.altitude = altitude
        self.fov_deg = fov_deg

        self.matSize = math.tan(math.radians(self.fov_deg / 2)) * self.altitude * WIDTHOFSQUARE
        # self.viewMat=CellField(matSize,matSize)

        # Anchor for getPlacesToCheck's TSP ordering -- kept fixed across
        # replans so a new discovery doesn't reorder the first stop.
        self.nextPlaceToCheckLocal = None

        # Route hysteresis -- see _dijkstra_path_with_hysteresis.
        self.previousEndNode = None

        # Archived checkpoint-pinning state.
        self.checkpointNode = None
        self.flownPrefixNodes = []

        # MAZE-STYLE state (start_maze_navigation). A = approach (final
        # edge, fixed anchor point_A), B = bridge (reroutable), C =
        # confirmed (frozen history).
        self.maze_a_path = []
        self.maze_b_path = []
        self.maze_confirmed_path = []
        # Per-segment TSP anchors, same idea as nextPlaceToCheckLocal.
        self.next_place_to_check_maze_a = None
        self.next_place_to_check_maze_b = None

        # HELPER-NODE state (use_helper_nodes). helper_node_trail:
        # not-yet-graph (distance, x, y) breadcrumbs from segment A, oldest
        # first. promoted_helper_nodes:
        # [{"node", "previous_point_a", "obstacle"}] for every breadcrumb
        # actually promoted to a graph node.
        self.helper_node_trail = []
        self.promoted_helper_nodes = []

        # CROSS-PAIR state. remote_mine_placeholders: [{"position", "obstacle"}]
        # for other-pair mines crossing THIS pair's segment A that haven't been
        # physically reached yet (see record_remote_mine_on_segment_a /
        # _check_remote_placeholder_reached). cross_pair_patches: pending
        # node-list patches THIS pair's assistant owes a verification flight to
        # (see patch_confirmed_span / get_cross_pair_patch_places_to_check) --
        # spliced into the OTHER pair's confirmed history, not this one's.
        self.remote_mine_placeholders = []
        self.cross_pair_patches = []
        # Result of the most recent add_discovered_mine(..., prefer_local_patch=True)
        # call's patch_confirmed_span attempt -- read by the caller right after,
        # not part of add_discovered_mine's return tuple (keeps every other
        # caller's unpacking unaffected).
        self.last_patched_span = None
        # retarget_approach_target's own state: the floating node
        # self.endingNodes currently holds once retargeted away from the
        # field's real far edge, and the (x, y) it was last set to (lets
        # the driver skip a no-op retarget when the other pair's point_A
        # hasn't actually moved). cross_pair_target_chain mirrors
        # promoted_helper_nodes but for cross-pair retarget hops (chained
        # via _bridge_up_to, capped at MAX_CHAIN_DEPTH, same backstop
        # start_helper_node_detour uses -- see retarget_approach_target).
        # _pending_approach_target is the latest (x, y) queued by a
        # retarget_approach_target call the chain didn't have room for
        # yet; retried once confirm_b_into_c/advance_b_prefix_into_c free
        # up a slot.
        self._approach_target_node = None
        self._last_synced_target = None
        self.cross_pair_target_chain = []
        self._pending_approach_target = None

        # Set the singleton instance
        if Pathfinder.instance is None:
            Pathfinder.instance = self

    # Thin wrapper -- see module-level order_waypoints (kept self-free so
    # it's testable without a full Pathfinder).
    def order_waypoints(
        self, coords: list[tuple[float, float]], fixed_first: tuple[float, float] = None
    ) -> list[tuple[float, float]]:
        return order_waypoints(coords, fixed_first=fixed_first)

    def buildNodeField(self, startLocation: tuple[float, float], startEdge: str = "bottom"):
        """startEdge picks which field edge this drone launches from:
        "bottom" (default, y=-1 entry / top row of end nodes) or "top"
        (mirrored -- for a pair working from the opposite end, whose end
        row is the same edge the "bottom" pair enters from). Only
        "bottom" is exercised by any current caller/test."""
        startLocationX, startLocationY = self.coord_converter.latlon_to_local(startLocation[0], startLocation[1])
        entry_y = -1 if startEdge == "bottom" else HEIGHTOFFIELD + 1
        target_y = HEIGHTOFFIELD + 1 if startEdge == "bottom" else -1

        self.startingNodes.append(self.nodeField.addFloatingNode(startLocationX, entry_y))

        for i in range(WIDTHOFFIELD // (WIDTHOFSQUARE * 2)):
            self.endingNodes.append(
                self.nodeField.addFloatingNode(
                    i * WIDTHOFSQUARE * 2 + WIDTHOFSQUARE // 2, target_y
                )
            )
        # Snapshot of the field's real far edge, kept separately from
        # self.endingNodes -- retarget_approach_target overwrites
        # self.endingNodes with a single cross-pair target node, and
        # _dijkstra_path_with_hysteresis falls back to this row if that
        # single node ever becomes unreachable (see its own docstring).
        self._original_ending_nodes = list(self.endingNodes)

    def _obstacle_containing(self, x: float, y: float):
        """Returns whichever live mine/union obstacle's polygon contains
        (x,y), or None. A merge can replace mines[-1] with a unionObstacle
        elsewhere in the list, so this looks up by position, not index."""
        for obstacle in list(self.nodeField.mines) + list(self.nodeField.unionObstacles):
            if obstacle.contains_point((x, y)):
                return obstacle
        return None

    def add_discovered_mine(self, mine_lat: float, mine_lon: float, prefer_local_patch: bool = False):
        """Adds a discovered mine and repairs any path state it invalidates.
        Returns: (obstacle, was_merged, rewound). obstacle is whichever
        live mine/union now sits at this position (see
        _obstacle_containing), was_merged is Field.addFromProtoMine's
        report of whether this add merged with an existing obstacle,
        rewound is check_merge_rewind's report of whether it already
        fully re-routed maze_a_path/maze_b_path -- callers should skip
        their own normal discovery handler when rewound is True.

        prefer_local_patch: [CROSS-PAIR] if True, a maze_confirmed_path
        hit is offered to patch_confirmed_span (bounded local splice)
        BEFORE check_path_envelopment's own always-safe but less targeted
        full recompute -- use when relaying a mine discovered by the
        OTHER pair, so this pair's own progress past the hit doesn't get
        needlessly discarded. Result (if any) is left on
        self.last_patched_span rather than added to this method's return
        tuple, so every existing caller's unpacking stays unaffected."""
        self.last_patched_span = None
        x, y = self.coord_converter.latlon_to_local(mine_lat, mine_lon)
        Xsquare, Ysquare = self.nodeField.getSquareCoordinates(x, y)
        # mid centers the safety-radius block grid on (Xsquare, Ysquare).
        mid = int(self.mine_saftey_radius * 2 + 1) // 2
        Xoffset = WIDTHOFSQUARE * (Xsquare - mid)
        Yoffset = WIDTHOFSQUARE * (Ysquare - mid)
        newProtoMine = protoMine(self.mine_saftey_radius, (mine_lat, mine_lon), (Xoffset, Yoffset))
        self.protoMines.append(newProtoMine)

        was_merged = self.nodeField.addFromProtoMine(newProtoMine)

        mineBlockField = build_mine_cell_field(newProtoMine)
        self.mineFieldTracker.apply_mask(mineBlockField, Xsquare - mid, Ysquare - mid, op="or")

        new_mine_obstacle = self._obstacle_containing(x, y)
        rewound = False
        if new_mine_obstacle is not None:
            # check_merge_rewind first (more targeted, less disruptive);
            # patch_confirmed_span next if requested (cross-pair, bounded
            # local splice); check_path_envelopment last as the general,
            # always-safe catch-all (finds nothing left to do in
            # maze_confirmed_path if patch_confirmed_span already handled it).
            rewound = self.check_merge_rewind(new_mine_obstacle, was_merged)
            if prefer_local_patch:
                self.last_patched_span = self.patch_confirmed_span(new_mine_obstacle)
            self.check_path_envelopment(new_mine_obstacle)
        return new_mine_obstacle, was_merged, rewound

    def accept_image_corner_coord(self, corner_coords_latlon: tuple[tuple[float, float]]):
        """Takes: an image's corner coords (lat/lon, in order). Marks every
        seen_tracker cell FULLY enclosed by them (not partially-clipped ones)."""
        local_corners = []
        for [lat, lon] in corner_coords_latlon:
            x, y = self.coord_converter.latlon_to_local(lat, lon)
            local_corners.append((x, y))
        self.seen_tracker.fill_polygon_covered(local_corners)

    def increase_radius(self, mine_radius_increment):
        self.nodeField.expandField(mine_radius_increment)

    def _dijkstra_path_with_hysteresis(self, start_nodes: list, hysteresis_tolerance: float):
        """Dijkstra from each of start_nodes to the best ending node,
        except keeps using self.previousEndNode instead if it's still
        within hysteresis_tolerance of the best length (damps flapping
        between near-tied ending nodes). Returns node list start to end,
        or [] if unreachable. Writes self.previousEndNode.

        If self.endingNodes -- possibly narrowed to a single cross-pair
        retarget target by retarget_approach_target -- is entirely
        unreachable from every start_nodes candidate, falls back to
        self._original_ending_nodes (the field's real far edge,
        snapshotted once in buildNodeField) before giving up. A
        retargeted self.endingNodes is a single node with none of the
        normal far-edge row's redundancy: if that one node gets removed
        (e.g. check_path_envelopment purging it because a new mine's
        merge grew to cover its exact position) or otherwise
        disconnected, every caller of this method would otherwise
        unconditionally wipe maze_a_path/maze_b_path to [] on its next
        recompute -- discarding real, safe, already-established corridor
        for a reason that has nothing to do with that corridor's own
        safety -- confirmed directly as a real ~20-cell coverage gap on
        the two-pair safety sweep. Falling back here, at the single
        shared root all those callers already go through, fixes it for
        all of them at once instead of patching each call site."""
        has_fallback = (
            self._original_ending_nodes is not None
            and self.endingNodes is not self._original_ending_nodes
        )

        best = None  # (length, end_node, predecessors)
        fallback_best = None  # same, but against self._original_ending_nodes
        stable = None  # same, but restricted to self.previousEndNode

        for i in start_nodes:
            newGraph = Graph(self.nodeField.fieldConnection.nodeGraph)
            distances, predecessors = newGraph.shortest_distances(i)

            for e in self.endingNodes:
                d = distances.get(e, math.inf)
                if d < math.inf and (best is None or d < best[0]):
                    best = (d, e, predecessors)

            if has_fallback:
                for e in self._original_ending_nodes:
                    d = distances.get(e, math.inf)
                    if d < math.inf and (fallback_best is None or d < fallback_best[0]):
                        fallback_best = (d, e, predecessors)

            if self.previousEndNode is not None:
                d = distances.get(self.previousEndNode, math.inf)
                if d < math.inf and (stable is None or d < stable[0]):
                    stable = (d, self.previousEndNode, predecessors)

        if best is None:
            best = fallback_best

        if best is None:
            self.previousEndNode = None
            return []

        _, chosen_end, chosen_predecessors = best
        if stable is not None and stable[0] <= best[0] * (1.0 + hysteresis_tolerance):
            _, chosen_end, chosen_predecessors = stable

        path_nodes = []
        current = chosen_end
        while current:
            path_nodes.append(current)
            current = chosen_predecessors[current]
        path_nodes.reverse()

        self.previousEndNode = chosen_end
        return path_nodes

    def get_shortest_path(self, hysteresis_tolerance: float = PATH_HYSTERESIS_TOLERANCE):
        """Plain greedy path: Dijkstra from self.startingNodes (true field
        entry) to the best ending node, recomputed fresh every call, with
        hysteresis."""
        return self._dijkstra_path_with_hysteresis(self.startingNodes, hysteresis_tolerance)

    def rasterize_node_path(self, nodePath):
        """Returns self.path_tracker, cleared and re-marked with nodePath's
        cells (mutates in place)."""
        self.path_tracker.clear_all()
        path = []
        for i in nodePath:
            path.append((i.x, i.y))

        self.path_tracker.mark_path(path)
        return self.path_tracker

    def _verify_places_to_check_cover_need(self, need_field, placements_local, along_ft, across_ft):
        """Rasterizes each placement's footprint and checks their union
        covers need_field entirely; raises RuntimeError (with sample
        missed cells) if not."""
        if need_field.count() == 0:
            return

        to_be_checked = CellField(
            need_field.width,
            need_field.height,
            min_corner=need_field.min_corner,
            max_corner=need_field.max_corner,
        )
        for x, y in placements_local:
            to_be_checked.fill_aligned_rect_touched(*_footprint_box(x, y, along_ft, across_ft))

        missed = need_field & ~to_be_checked
        missed_count = missed.count()
        if missed_count > 0:
            sample = [missed.cell_to_real(x, y) for x, y in list(missed.on_cells())[:5]]
            raise RuntimeError(
                f"getPlacesToCheck: {missed_count} cell(s) that need checking are not "
                f"covered by any of the {len(placements_local)} returned placement(s) "
                f"-- e.g. real-world {sample}"
            )

    def getPlacesToCheck(
        self,
        method: str = "path",
        overlap: float = 0.0,
        path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ):
        """Plans against get_shortest_path. method="path" walks the path
        geometry directly; "cellgrid" is the rasterize-then-greedy-cover
        approach for arbitrary non-path targets. overlap/path_width
        (method="path" only): placement spacing margin / corridor width.
        shape_size_ft: footprint override (number or (along, across)),
        else derived from matSize. Builds footprint placements covering
        this drone's unseen slice, orders them, verifies coverage (raises
        on a gap), converts to lat/lon."""
        shortest_path = self.get_shortest_path()
        self.best_node_List = shortest_path
        return self._places_to_check_for_path(
            shortest_path, method=method, overlap=overlap, path_width=path_width,
            shape_size_ft=shape_size_ft, fixed_first=self.nextPlaceToCheckLocal,
            next_place_attr="nextPlaceToCheckLocal",
        )

    def _places_to_check_for_path(
        self, path_nodes, method="path", overlap=0.0, path_width=0.0,
        shape_size_ft=None, fixed_first=None, next_place_attr=None,
    ):
        """Shared engine behind getPlacesToCheck and the maze places-to-check
        methods, for an explicit node path instead of get_shortest_path().
        fixed_first is the TSP anchor; next_place_attr, if given, is the
        attribute name to write the new anchor back to (lets each
        independent queue, e.g. maze A vs B, track its own)."""
        matSizeCells = max(1, round(self.matSize / WIDTHOFSQUARE))

        # The whole field/path, never a per-drone slice: territory division
        # across drones is Side/pairing's job (which field edge a pair
        # starts from) plus retarget_approach_target's convergence, not a
        # fixed horizontal-band split -- an older strategy this class no
        # longer implements (see path_cover.py's own history for the
        # y-slicing it used to do here).
        whole_field = self.rasterize_node_path(path_nodes)

        if method == "path":
            path_points = [(n.x, n.y) for n in path_nodes]
            shape_size = (
                shape_size_ft if shape_size_ft is not None else matSizeCells * WIDTHOFSQUARE
            )
            ShapesToVisit = path_cover_unseen(
                path_points,
                self.seen_tracker,
                shape_size,
                overlap=overlap,
                path_width=path_width,
            )
            along_ft, across_ft = _normalize_shape_size(shape_size)
            # method="path" only needs what's still unseen, not the whole
            # field (cover_with_shape/"cellgrid" targets that unconditionally).
            need_field = whole_field & ~self.seen_tracker
        elif method == "cellgrid":
            if shape_size_ft is not None:
                raise ValueError("shape_size_ft override is only supported for method='path'")
            # self.path_tracker is the same footprint rasterize_node_path
            # just rebuilt above (whole_field).
            self.best_path = self.path_tracker
            ourPortion = whole_field
            ShapesToVisit = ourPortion.cover_with_shape((matSizeCells, matSizeCells))
            along_ft = across_ft = matSizeCells * WIDTHOFSQUARE
            need_field = ourPortion
        else:
            raise ValueError(
                f"getPlacesToCheck: unknown method {method!r}, expected 'path' or 'cellgrid'"
            )

        orderWaypoints = self.order_waypoints(ShapesToVisit, fixed_first=fixed_first)
        if next_place_attr is not None:
            setattr(self, next_place_attr, orderWaypoints[0] if orderWaypoints else None)
        self._verify_places_to_check_cover_need(need_field, orderWaypoints, along_ft, across_ft)
        latLonPoints = []
        for i in orderWaypoints:
            # cover_with_shape/path_cover already return real-world feet --
            # multiplying by WIDTHOFSQUARE again here was a doubling bug.
            lat, lon = self.coord_converter.local_to_latlon(i[0], i[1])
            latLonPoints.append((lat, lon))
        return latLonPoints

    # ==================================================================
    # MAZE-STYLE incremental replanning.
    #
    # Three segments: C ("confirmed", start..point_C) is frozen, never
    # revisited. B ("bridge", point_C..point_A) is the actively-worked,
    # reroutable segment. A ("approach", point_A..true end) is just the
    # final edge of the last computed path -- point_A is the stability
    # anchor, pinning it structurally instead of by hysteresis margin.
    #
    # Rule 1 (mine found in A, or the first pass before any split exists):
    # recompute fresh from point_C, re-split at the fresh path's final
    # edge (new A), rest becomes B. Rule 2 (mine found in B): reroute B
    # between the same fixed point_C/point_A.
    #
    # B draining clean confirms it into C and returns to checking A.
    #
    # Usage:
    #   pf.start_maze_navigation()              # once, up front
    #   pf.get_places_to_check_maze()            # {"a": [...], "b": [...]}
    #   ... mine found while checking "a":
    #   pf.on_forward_mine_discovered()          # Rule 1
    #   ... mine found while checking "b":
    #   pf.reroute_b_segment()                   # Rule 2
    #   ... "b" drains to []:
    #   pf.confirm_b_into_c()
    # ==================================================================

    def _resolve_node_near(self, x: float, y: float):
        """Returns whichever node with at least one live edge is closest to
        (x,y) -- re-resolves a stale reference by position since a merge
        can replace a node with a new object at the same tangent point.
        Skips 0-edge orphans (dead ends for Dijkstra)."""
        best_node = None
        best_dist = math.inf
        for node, neighbors in self.nodeField.fieldConnection.nodeGraph.items():
            if not neighbors:
                continue
            d = math.hypot(node.x - x, node.y - y)
            if d < best_dist:
                best_dist = d
                best_node = node
        return best_node

    def _maze_c_start_nodes(self) -> list:
        """Takes: nothing. Returns: [node] to search FROM for a Rule-1
        recompute -- wherever self.maze_confirmed_path (point_C) currently
        ends, re-resolved to a live node, or self.startingNodes if C is
        still empty (nothing confirmed yet)."""
        if not self.maze_confirmed_path:
            return self.startingNodes
        last = self.maze_confirmed_path[-1]
        node = self._resolve_node_near(last.x, last.y)
        return [node] if node is not None else self.startingNodes

    def _prepend_seam_if_needed(self, path_nodes: list) -> list:
        """If path_nodes[0] (possibly a re-resolved substitute for point_C,
        see _resolve_node_near) isn't the same point as point_C's real
        end, prepends the REAL graph-connected route between them --
        otherwise a later confirm/advance same_point check would silently
        splice a never-checked jump into the path. Returns path_nodes, or
        route[:-1] + path_nodes where route is
        _shortest_path_between(point_C's real end, path_nodes[0]).

        Routing through _shortest_path_between (rather than assuming a
        bare straight line is safe) matters because every OTHER edge in
        this file only ever comes from the graph itself, which by
        construction never contains an edge crossing a KNOWN obstacle --
        but a bare [seam_node, first] pair is invented here, not looked
        up, so it was never checked against anything. That's fine as
        long as _resolve_node_near always lands on a node visible in a
        dead straight line from point_C's end, but there's no guarantee
        of that -- confirmed directly as a real bad edge on the two-pair
        round-robin safety sweep, a ~38ft straight-line seam cutting
        through a mine that was ALREADY known (not one discovered
        afterward -- check_path_envelopment's own edge scan only
        re-validates confirmed-path edges against FUTURE discoveries, so
        an already-known obstacle a seam happens to cross is never
        caught). Falls back to the old unchecked straight line only if
        the graph genuinely has no path between them at all (seam_node
        and first should almost always be mutually reachable -- both are
        live, graph-connected nodes -- so this should be rare)."""
        if not self.maze_confirmed_path or not path_nodes:
            return path_nodes
        seam_node = self.maze_confirmed_path[-1]
        first = path_nodes[0]
        same_point = seam_node is first or (
            abs(seam_node.x - first.x) < 1e-6 and abs(seam_node.y - first.y) < 1e-6
        )
        if same_point:
            return path_nodes
        route = self._shortest_path_between(seam_node, first)
        if route is None:
            return [seam_node] + path_nodes
        return route[:-1] + path_nodes

    def start_maze_navigation(self) -> None:
        """Initializes maze-navigation state -- the first pass treats the
        whole current shortest path as segment A."""
        self.maze_a_path = self.get_shortest_path()
        self.maze_b_path = []
        self.maze_confirmed_path = []
        self.helper_node_trail = []
        self.promoted_helper_nodes = []
        self.remote_mine_placeholders = []
        self.cross_pair_patches = []
        self.last_patched_span = None
        self.next_place_to_check_maze_a = None
        self.next_place_to_check_maze_b = None
        self._approach_target_node = None
        self._last_synced_target = None
        self.cross_pair_target_chain = []
        self._pending_approach_target = None

    def record_helper_node_candidate(self, x: float, y: float) -> None:
        """[HELPER-NODE] Records a real-world (x,y) just photographed while
        checking segment A into self.helper_node_trail (sorted ascending
        by distance from maze_a_path[0] -- geometric distance, not visit
        order, since order_waypoints' TSP tour doesn't walk the path in
        order). Skips cells out of bounds or already inside a known
        obstacle. Trims entries more than 2x the safety margin behind the
        furthest-along one seen so far -- a rolling relevance bound, not
        the safety check itself (that's start_helper_node_detour, once a
        mine position exists to measure against)."""
        if len(self.maze_a_path) < 1:
            return
        col, row = self.seen_tracker.real_to_cell(x, y)
        if not (0 <= col < self.seen_tracker.width and 0 <= row < self.seen_tracker.height):
            return
        cx, cy = self.seen_tracker.cell_to_real(col, row)
        for obstacle in list(self.nodeField.mines) + list(self.nodeField.unionObstacles):
            if obstacle.contains_point((cx, cy)):
                return

        a0 = self.maze_a_path[0]
        distance = math.hypot(cx - a0.x, cy - a0.y)
        entry = (distance, cx, cy)
        if entry not in self.helper_node_trail:
            self.helper_node_trail.append(entry)
            self.helper_node_trail.sort(key=lambda e: e[0])

        trim_window = 2 * (self.mine_saftey_radius * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE)
        furthest_along = self.helper_node_trail[-1][0]
        self.helper_node_trail = [
            e for e in self.helper_node_trail if furthest_along - e[0] <= trim_window
        ]

    def _prune_promoted_helper_nodes(self) -> None:
        """Drops any self.promoted_helper_nodes entry whose node is no
        longer part of the current maze_confirmed_path/maze_b_path/
        maze_a_path. Call after ANY of these three lists get replaced --
        not just check_path_envelopment's own hit (the original caller of
        this pattern) -- since reroute_b_segment/reroute_b_segment_same_mine/
        on_forward_mine_discovered can ALSO replace maze_b_path/maze_a_path
        independently of the helper-node tracking. Without this, a stale
        entry's "previous_point_a"/"node" pairing can silently stop
        matching the ACTUAL current bridge structure, and a later
        check_merge_rewind repair built on it can splice in a stale (or
        even unsafe -- e.g. an old, never-independently-revalidated raw
        connectNode edge from the original detour) connection instead of
        the real current route."""
        self.promoted_helper_nodes = [
            entry
            for entry in self.promoted_helper_nodes
            if any(
                n is entry["node"]
                for n in self.maze_confirmed_path + self.maze_b_path + self.maze_a_path
            )
        ]

    def _settle_confirmed_helper_nodes(self) -> None:
        """Drops any self.promoted_helper_nodes entry whose node has been
        folded into maze_confirmed_path -- that detour is now settled
        (safely confirmed), not outstanding, so start_helper_node_detour's
        one-outstanding-detour-at-a-time guard should stop counting it.
        Call after confirm_b_into_c/advance_b_prefix_into_c grow
        maze_confirmed_path. Deliberately the OPPOSITE keep-condition from
        _prune_promoted_helper_nodes (which keeps anything still
        referenced anywhere, confirmed included) -- without this, the
        first-ever detour's entry would never leave the list, and the
        one-at-a-time guard would permanently block every later detour,
        including in single-drone mode (which used to never chain, but
        also never had a reason to prune a settled entry either)."""
        self.promoted_helper_nodes = [
            entry
            for entry in self.promoted_helper_nodes
            if not any(n is entry["node"] for n in self.maze_confirmed_path)
        ]

    # ==================================================================
    # CROSS-PAIR: two pairs on opposite ends sharing discoveries. Mine
    # relay itself lives in the simulation driver (each pair's
    # add_discovered_mine gets called for every mine, regardless of who
    # found it) -- these two methods are what the RECEIVING pair does with
    # a mine it didn't discover itself, for segment A (not yet flown, so
    # nothing in the current graph state references it) and confirmed
    # history (see patch_confirmed_span below) respectively. A remote mine
    # landing in the receiving pair's ACTIVE segment B needs neither: its
    # own check_merge_rewind/check_path_envelopment (already run by
    # add_discovered_mine for every mine, local or relayed) already scans
    # maze_b_path for exactly this.
    # ==================================================================

    def record_remote_mine_on_segment_a(self, new_mine) -> None:
        """Call on the pair that did NOT discover new_mine, right after
        relaying it into add_discovered_mine, if it crosses THIS pair's
        own segment A line (point_A -> current approach target, which in
        two-pair mode is the OTHER pair's own point_A -- see
        retarget_approach_target -- rather than a fixed field edge).
        Segment A hasn't been flown yet -- add_discovered_mine's own
        check_path_envelopment/
        check_merge_rewind only look at nodes/edges ALREADY in
        maze_a_path/maze_b_path/maze_confirmed_path, so they can't catch
        this. Computes where a real flown breadcrumb would have landed
        (same safety-margin logic start_helper_node_detour uses) and
        saves it as a placeholder -- no graph changes yet. If gambler's
        own flight later reaches that position (_check_remote_placeholder_
        reached), it gets promoted into a real detour exactly like a
        local discovery; if gambler finds something else first, it's
        just dropped (see the clear alongside every maze_a_path-replacing
        rule) and the normal fallback already accounts for this mine
        anyway, since it's in the graph either way by then."""
        if len(self.maze_a_path) < 2:
            return
        p0, p1 = self.maze_a_path[0], self.maze_a_path[1]
        if not new_mine.intersects(((p0.x, p0.y), (p1.x, p1.y))):
            return
        if any(entry["obstacle"] is new_mine for entry in self.remote_mine_placeholders):
            return
        mine_x, mine_y = getattr(new_mine, "origin", None) or (
            new_mine.polygon.centroid.x, new_mine.polygon.centroid.y
        )
        safety_margin = self.mine_saftey_radius * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE
        dist_to_mine = math.hypot(mine_x - p0.x, mine_y - p0.y)
        place_at = max(0.0, dist_to_mine - safety_margin)
        ux = (mine_x - p0.x) / max(dist_to_mine, 1e-9)
        uy = (mine_y - p0.y) / max(dist_to_mine, 1e-9)
        self.remote_mine_placeholders.append({
            "position": (p0.x + ux * place_at, p0.y + uy * place_at),
            "obstacle": new_mine,
        })

    def _check_remote_placeholder_reached(self, x: float, y: float):
        """Call from the gambler's own segment-A visit loop after each
        photo. If (x,y) has reached or passed a pending placeholder along
        segment A's own line, injects it into self.helper_node_trail
        (start_helper_node_detour's normal candidate list, as though a
        real photo had just landed there) and returns the associated
        obstacle -- caller treats this exactly like a freshly discovered
        local mine (calls start_helper_node_detour(returned_obstacle)).
        Returns None if nothing's been reached yet."""
        if not self.remote_mine_placeholders or not self.maze_a_path:
            return None
        a0 = self.maze_a_path[0]
        here_dist = math.hypot(x - a0.x, y - a0.y)
        for i, entry in enumerate(self.remote_mine_placeholders):
            px, py = entry["position"]
            place_dist = math.hypot(px - a0.x, py - a0.y)
            if here_dist >= place_dist:
                self.remote_mine_placeholders.pop(i)
                self.helper_node_trail.append((place_dist, px, py))
                self.helper_node_trail.sort(key=lambda e: e[0])
                return entry["obstacle"]
        return None

    def on_forward_mine_discovered(self) -> None:
        """[Rule 1] Recomputes fresh from point_C (or the true start), then
        splits at the fresh path's own final edge: last two nodes become
        the new segment A, rest becomes B. Call when a mine is found
        while checking segment A."""
        recomputed = self._dijkstra_path_with_hysteresis(
            self._maze_c_start_nodes(), PATH_HYSTERESIS_TOLERANCE
        )
        recomputed = self._prepend_seam_if_needed(recomputed)
        if len(recomputed) < 2:
            self.maze_a_path = recomputed
            self.maze_b_path = []
            self._prune_promoted_helper_nodes()
            self.remote_mine_placeholders = []
            return
        self.maze_a_path = recomputed[-2:]
        self.maze_b_path = recomputed[:-1]
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []

    def _bridge_up_to(self, anchor_node) -> list:
        """The real, already-established route from wherever's currently
        confirmed up through anchor_node (inclusive): anchor_node's own
        position in self.maze_b_path if it's there (anchor_node is itself
        mid-bridge -- a chained helper-node detour/repair, not point_C's
        real end), else just anchor_node with the usual seam guard (it IS
        effectively point_C's end -- first-ever detour, or nothing chained
        yet). Used by start_helper_node_detour, check_merge_rewind, and
        _check_forward_of_helper_node so none of them silently drop the
        real chain back to point_C in favor of a straight, never-validated
        jump when splicing in a new stretch.

        Deliberately does NOT also search self.maze_confirmed_path for
        anchor_node: node objects get legitimately reused/re-resolved
        across unrelated points in the graph's history (_resolve_node_near
        picks "closest live node," which can land on the same tangent node
        object more than once), so an identity match there isn't reliable
        proof anchor_node's CURRENT role is the same as its earlier one --
        an earlier version of this method searched maze_confirmed_path too
        and truncated it on a match, which silently discarded large swaths
        of legitimately-confirmed history on a false-positive identity
        match (measured: 133/200 seeds developed real coverage gaps).
        Anchor nodes already folded into maze_confirmed_path fall through
        to the seam guard instead -- less precise in the rare stale-entry
        case (see check_merge_rewind's docstring) but not destructive."""
        for i, node in enumerate(self.maze_b_path):
            if node is anchor_node:
                return self.maze_b_path[: i + 1]
        return self._prepend_seam_if_needed([anchor_node])

    def start_helper_node_detour(self, new_mine) -> bool:
        """[HELPER-NODE alternative to Rule 1] By the time this is called,
        add_discovered_mine has already run check_merge_rewind for this
        discovery -- callers skip this method entirely when that returns
        rewound=True. Picks the self.helper_node_trail candidate with the
        smallest margin (>= safety margin, closest to the mine while still
        safe) behind new_mine, promotes it to a real graph Node, connects
        it to new_mine's tangents and to point_A as it stood before this
        discovery, and re-splits like on_forward_mine_discovered but
        seeded at the new node -- collapsing the already-photographed
        stretch behind it into one edge. Records the promotion in
        self.promoted_helper_nodes for later repair (check_path_envelopment).

        The new maze_b_path is spliced onto whatever bridge already exists
        (self.maze_b_path itself, if non-empty) rather than re-derived from
        scratch every call, via _bridge_up_to -- this is what makes
        CHAINING safe: a pair's gambler can find a second (or third...)
        mine along A before the assistant has confirmed the earlier
        bridge into C, in which case previous_point_a is a prior helper
        node (or one of ITS mine's tangent nodes -- see
        _repair_helper_node_along_segment's docstring for exactly which),
        not point_C's real end. A naive from-scratch seam-guarded
        recompute would silently drop that real, already-established chain
        and replace it with a straight, never-validated jump from point_C
        directly to the newest helper node -- _bridge_up_to instead finds
        wherever the real chain currently ends and builds onto it.
        Measured empirically (200 seeds): chains that form this way are
        rare (~0.6 discoveries/seed hit this) and shallow (max depth 3
        observed, confirmed into C within 1-4 rounds) -- gambler doesn't
        actually race far ahead of the assistant in practice, but the
        splice still has to be correct on the occasions it does happen.

        An EARLIER, already-confirmed link in the chain (assistant caught
        up to and folded it into maze_confirmed_path since it was created)
        is a different case, handled not here but in check_merge_rewind:
        it defers entirely to check_path_envelopment (which safely
        handles confirmed-history repairs via full recompute, including
        edge crossings) rather than risking a splice into frozen ground --
        see check_merge_rewind's docstring.

        Capped at MAX_CHAIN_DEPTH simultaneous outstanding (unconfirmed)
        entries -- once gambler is that far ahead of the assistant, it
        stops opening new detours and falls back to the plain full
        recompute (on_forward_mine_discovered) instead, effectively
        waiting where it is until the assistant confirms at least one
        entry and the count drops back under the cap. This is a backstop,
        not the common case: real chains rarely reach it (see above).

        Returns True if a detour was built, False if the trail had nothing
        usable, the chain is already at MAX_CHAIN_DEPTH, OR if maze_a_path
        is currently empty -- a concurrent discovery (e.g. the assistant's
        own find, in a round-robin pair driver) can already have run
        check_path_envelopment and fully invalidated it before this call --
        either way, caller falls back to on_forward_mine_discovered."""
        if len(self.promoted_helper_nodes) >= MAX_CHAIN_DEPTH:
            return False
        if not self.maze_a_path:
            return False
        previous_point_a = self.maze_a_path[0]
        # Measured from new_mine's CENTER, not nearest polygon vertex --
        # the polygon boundary already IS the safety radius, so measuring
        # from the boundary would double-count it.
        mine_x, mine_y = getattr(new_mine, "origin", None) or (
            new_mine.polygon.centroid.x, new_mine.polygon.centroid.y
        )
        mine_center_distance = math.hypot(mine_x - previous_point_a.x, mine_y - previous_point_a.y)
        safety_margin = self.mine_saftey_radius * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE

        # contains_point re-check on top of the margin one: order_waypoints'
        # travel-minimizing visit order can check placements past the mine
        # before the mine itself is discovered.
        chosen = None
        chosen_margin = None
        for distance, cx, cy in self.helper_node_trail:
            margin = mine_center_distance - distance
            if margin < safety_margin:
                continue
            if new_mine.contains_point((cx, cy)):
                continue
            if chosen_margin is None or margin < chosen_margin:
                chosen_margin = margin
                chosen = (cx, cy)
        self.helper_node_trail = []
        if chosen is None:
            return False

        helper_node = Node(chosen[0], chosen[1], True, id=self.nodeField._generateId())
        self.nodeField._connectFloatingNodeToObstacle(new_mine, helper_node)
        connected_directly = self.nodeField.fieldConnection.connectNode(helper_node, previous_point_a)

        if not self.nodeField.fieldConnection.nodeGraph.get(helper_node):
            # Every connection attempt failed (every candidate tangent AND
            # the direct link to previous_point_a all cross some other
            # obstacle) -- helper_node is unusably isolated. Graph.
            # shortest_distances doesn't handle a disconnected seed node
            # (it unconditionally sets distances[source]=0 and then
            # indexes self.graph[source], KeyError-ing if source was never
            # added as a key) -- same "trail unusable" contract as chosen
            # being None, caller falls back to on_forward_mine_discovered.
            return False

        recomputed = self._dijkstra_path_with_hysteresis([helper_node], PATH_HYSTERESIS_TOLERANCE)
        if len(recomputed) < 2:
            self.maze_a_path = recomputed
            self.maze_b_path = []
        elif connected_directly:
            full = self._bridge_up_to(previous_point_a) + recomputed
            self.maze_a_path = full[-2:]
            self.maze_b_path = full[:-1]
        else:
            # connectNode above correctly rejected the direct link (it
            # would have crossed some other obstacle), but helper_node
            # still connected to something (the mine's own tangents,
            # normally) -- naively concatenating _bridge_up_to(previous_
            # point_a) with recomputed anyway (the ONLY thing this branch
            # used to do) juxtaposes previous_point_a and helper_node as
            # consecutive path nodes with NO real edge between them,
            # producing a phantom, never-validated "edge" that later gets
            # flown and, if later folded into confirmed history via
            # advance_b_prefix_into_c, frozen there permanently unsafe --
            # confirmed directly as a real bad edge on a two-pair sweep
            # seed, traced via connectNode itself returning False for
            # this exact pair right before it happened. Find the REAL
            # (possibly indirect, routing around whatever blocked the
            # direct link) path between them instead.
            link = self._shortest_path_between(previous_point_a, helper_node)
            if link is None:
                # Genuinely disconnected from the rest of the current
                # bridge -- same "trail unusable" contract as the
                # isolated-node case above.
                self.nodeField.fieldConnection.removeNode(helper_node)
                return False
            full = self._bridge_up_to(previous_point_a) + link[1:] + recomputed[1:]
            self.maze_a_path = full[-2:]
            self.maze_b_path = full[:-1]

        self.promoted_helper_nodes.append(
            {"node": helper_node, "previous_point_a": previous_point_a, "obstacle": new_mine}
        )
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []
        return True

    def check_merge_rewind(self, new_mine, was_merged: bool) -> bool:
        """[HELPER-NODE, no-op if self.promoted_helper_nodes is empty]
        Scans self.promoted_helper_nodes newest-first (least disruptive
        match wins) for an entry now suspect via any of three triggers,
        checked against new_mine:
          - identity: was_merged and the entry's obstacle's .mergedInto
            chain resolves to new_mine (an earlier detour's target mine
            got absorbed into the same merge).
          - position: the entry's own helper node position is inside
            new_mine's polygon.
          - segment: new_mine intersects the segment from the entry's
            helper node back to its previous_point_a (a mine can be
            discovered while checking that fresh edge, without covering
            either endpoint).
        On a match, tries a local repair (_repair_helper_node_along_segment):
        reuse the helper node in place if still safe, or reconnect further
        back along the same segment. Only if that's impossible does it
        fall back to walking back through EARLIER entries for a safe
        previous_point_a, discarding everything after (Field.removeNode
        on each dropped helper node). Either way, truncates
        self.maze_confirmed_path to the reseed point if needed and
        re-runs the standard hysteresis recompute + seam guard from there.

        Entries whose node has ALREADY been folded into
        self.maze_confirmed_path (confirm_b_into_c/advance_b_prefix_into_c
        ran since this entry was created -- normal progress, common once a
        gambler chains several detours ahead of a slower assistant) are
        skipped entirely, deferring to check_path_envelopment instead:
        splicing a repair into already-frozen history needs to know
        exactly how much of maze_confirmed_path is genuinely still valid
        around that node, which only check_path_envelopment's own
        node/edge scan of maze_confirmed_path can determine safely --
        attempting it here risked either discarding unrelated confirmed
        history or inserting an unvalidated jump (both measured; see git
        history for this method around the session this was found).

        If none of the three (backward-looking) triggers match, also
        tries _check_forward_of_helper_node -- a mine can turn up AHEAD
        of a helper node instead, toward its own target mine's tangent.

        Returns True if a rewind was performed (routing already resolved
        for this discovery), False otherwise (caller proceeds normally).
        """
        if not self.promoted_helper_nodes:
            return False

        hit_index = None
        for index in range(len(self.promoted_helper_nodes) - 1, -1, -1):
            entry = self.promoted_helper_nodes[index]
            identity_hit = False
            if was_merged:
                resolved = entry.get("obstacle")
                while resolved is not None and getattr(resolved, "mergedInto", None) is not None:
                    resolved = resolved.mergedInto
                identity_hit = resolved is new_mine
            node = entry["node"]
            previous_point_a = entry["previous_point_a"]
            position_hit = new_mine.contains_point((node.x, node.y))
            segment_hit = new_mine.intersects(
                ((previous_point_a.x, previous_point_a.y), (node.x, node.y))
            )
            if identity_hit or position_hit or segment_hit:
                hit_index = index
                break
        if hit_index is None:
            return self._check_forward_of_helper_node(new_mine)

        entry = self.promoted_helper_nodes[hit_index]
        old_node, previous_point_a = entry["node"], entry["previous_point_a"]
        if any(n is old_node for n in self.maze_confirmed_path):
            # Already folded into confirmed history (normal progress since
            # this entry was created -- common once a gambler chains
            # several detours ahead of a slower assistant). Defer entirely
            # to check_path_envelopment's own node/edge scan of
            # maze_confirmed_path (which runs right after this, for every
            # add_discovered_mine call, and now also catches edge crossings
            # there) instead of risking a repair-splice into frozen ground.
            # Checked AFTER hit_index is chosen, not during the scan above,
            # so this can never shift which (unrelated) entries the
            # subsequent "stale" cleanup below removes.
            return False
        prefix_path, tracked_node = self._repair_helper_node_along_segment(
            old_node, previous_point_a, new_mine
        )

        if prefix_path is not None:
            truncate_at = next(
                (i for i, n in enumerate(self.maze_confirmed_path) if n is old_node), None
            )
            if truncate_at is not None:
                self.maze_confirmed_path = self.maze_confirmed_path[:truncate_at]
            # prefix_path[0] is previous_point_a itself -- splice it onto the
            # real bridge already established up through previous_point_a
            # (not just a seam straight to point_C's end; see _bridge_up_to)
            # before dropping the duplicate.
            full_prefix = self._bridge_up_to(previous_point_a) + prefix_path[1:]

            for stale in self.promoted_helper_nodes[hit_index + 1 :]:
                self.nodeField.fieldConnection.removeNode(stale["node"])
            self.promoted_helper_nodes = self.promoted_helper_nodes[:hit_index]
            self.promoted_helper_nodes.append(
                {"node": tracked_node, "previous_point_a": previous_point_a, "obstacle": new_mine}
            )
            self.helper_node_trail = []

            recomputed = self._dijkstra_path_with_hysteresis([old_node], PATH_HYSTERESIS_TOLERANCE)
            full = full_prefix if len(recomputed) < 2 else full_prefix[:-1] + recomputed
            if len(full) < 2:
                self.maze_a_path = full
                self.maze_b_path = []
            else:
                self.maze_a_path = full[-2:]
                self.maze_b_path = full[:-1]
            self._prune_promoted_helper_nodes()
            self.remote_mine_placeholders = []
            return True

        # Local repair impossible (old_node's own position isn't safe,
        # or nothing connects it to previous_point_a at all even after a
        # full reconnect) -- coarser fallback: walk back through EARLIER
        # entries for a safe anchor, discarding everything after.
        reseed_node = previous_point_a
        while new_mine.contains_point((reseed_node.x, reseed_node.y)):
            hit_index -= 1
            if hit_index < 0:
                reseed_node = self._maze_c_start_nodes()[0]
                break
            reseed_node = self.promoted_helper_nodes[hit_index]["previous_point_a"]
        hit_index = max(hit_index, 0)

        for stale in self.promoted_helper_nodes[hit_index:]:
            self.nodeField.fieldConnection.removeNode(stale["node"])
        self.promoted_helper_nodes = self.promoted_helper_nodes[:hit_index]

        # Same reasoning as the local-repair branch above: reseed_node can
        # itself be mid-chain (an earlier entry's previous_point_a), not
        # point_C's real end -- _bridge_up_to preserves the real route back
        # to it instead of a naive seam straight from point_C.
        prior_bridge = self._bridge_up_to(reseed_node)

        truncate_at = next(
            (i for i, n in enumerate(self.maze_confirmed_path) if n is reseed_node), None
        )
        if truncate_at is not None:
            self.maze_confirmed_path = self.maze_confirmed_path[: truncate_at + 1]

        self.helper_node_trail = []

        recomputed = self._dijkstra_path_with_hysteresis([reseed_node], PATH_HYSTERESIS_TOLERANCE)
        if len(recomputed) < 2:
            self.maze_a_path = recomputed
            self.maze_b_path = []
        else:
            full = prior_bridge[:-1] + recomputed
            self.maze_a_path = full[-2:]
            self.maze_b_path = full[:-1]
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []
        return True

    def _check_forward_of_helper_node(self, new_mine) -> bool:
        """[Called by check_merge_rewind once none of its own backward
        triggers match] A tracked helper node's FORWARD connection
        (toward its own target mine's tangents, not previous_point_a)
        can also get crossed by a new mine. Walks self.promoted_helper_nodes
        newest first for the first whose forward connection new_mine now
        crosses. Nothing behind the helper node is in question here, so
        the fix is just to recalculate point A from it onward: truncate
        self.maze_confirmed_path to just after the node if needed (keeping
        the node itself), drop later tracked entries, and reseed the
        standard recompute at the helper node. Returns True if performed,
        False if nothing is affected (caller proceeds normally)."""
        nodeGraph = self.nodeField.fieldConnection.nodeGraph
        for index in range(len(self.promoted_helper_nodes) - 1, -1, -1):
            entry = self.promoted_helper_nodes[index]
            node = entry["node"]
            target_obstacle = entry.get("obstacle")
            if target_obstacle is None:
                continue
            neighbors = nodeGraph.get(node, {})
            forward_hit = any(
                tangent in neighbors
                and new_mine.intersects(((node.x, node.y), (tangent.x, tangent.y)))
                for tangent in target_obstacle.nodes
            )
            if not forward_hit:
                continue

            for stale in self.promoted_helper_nodes[index + 1 :]:
                self.nodeField.fieldConnection.removeNode(stale["node"])
            self.promoted_helper_nodes = self.promoted_helper_nodes[: index + 1]
            self.helper_node_trail = []

            truncate_at = next(
                (i for i, n in enumerate(self.maze_confirmed_path) if n is node), None
            )
            if truncate_at is not None:
                self.maze_confirmed_path = self.maze_confirmed_path[: truncate_at + 1]

            recomputed = self._dijkstra_path_with_hysteresis([node], PATH_HYSTERESIS_TOLERANCE)
            if len(recomputed) < 2:
                self.maze_a_path = recomputed
                self.maze_b_path = []
            else:
                # node can itself be mid-chain -- _bridge_up_to (not a
                # from-scratch seam to point_C) preserves the real route
                # back to it; dedupe since recomputed[0] IS node too.
                full = self._bridge_up_to(node)[:-1] + recomputed
                self.maze_a_path = full[-2:]
                self.maze_b_path = full[:-1]
            self._prune_promoted_helper_nodes()
            self.remote_mine_placeholders = []
            return True
        return False

    def _shortest_path_between(self, start_node, end_node) -> list | None:
        """Shortest-path node list from start_node to end_node (inclusive,
        in order) on the current graph, or None if unreachable."""
        newGraph = Graph(self.nodeField.fieldConnection.nodeGraph)
        distances, predecessors = newGraph.shortest_distances(start_node)
        if distances.get(end_node, math.inf) == math.inf:
            return None
        path_nodes = []
        current = end_node
        while current:
            path_nodes.append(current)
            current = predecessors[current]
        path_nodes.reverse()
        return path_nodes

    @staticmethod
    def _path_halfway_point(path: list) -> tuple[float, float]:
        """[CROSS-PAIR] Returns the (x, y) position HALFWAY along path
        (a node list, e.g. from _shortest_path_between) by cumulative
        flown distance -- interpolated between whichever two consecutive
        nodes straddle the halfway mark, not snapped to an existing node.
        Used by _try_apply_pending_approach_target so a cross-pair
        retarget only ever advances a pair's own point_A to the midpoint
        of the REAL (obstacle-respecting) path to the other pair's
        position, never a straight-line average that could cut through
        territory neither pair has actually reached yet."""
        seg_lengths = [
            math.hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y)
            for i in range(len(path) - 1)
        ]
        total = sum(seg_lengths)
        if total <= 0:
            return (path[0].x, path[0].y)
        half = total / 2.0
        acc = 0.0
        for i, seg_len in enumerate(seg_lengths):
            if acc + seg_len >= half:
                t = (half - acc) / seg_len if seg_len > 1e-9 else 0.0
                return (
                    path[i].x + t * (path[i + 1].x - path[i].x),
                    path[i].y + t * (path[i + 1].y - path[i].y),
                )
            acc += seg_len
        return (path[-1].x, path[-1].y)

    def _repair_helper_node_along_segment(self, old_node, previous_point_a, new_mine):
        """[Called by check_merge_rewind] old_node is a promoted helper
        node whose target mine's merger requires repair; previous_point_a
        is the fixed anchor it was originally wired to; new_mine is the
        current, possibly-bigger obstacle to reconnect against.

        If old_node's own position is still safe (same margin
        start_helper_node_detour uses, plus clear of every other known
        obstacle), fully refreshes its connections (same procedure
        Field.addFloatingNode uses for a brand new floating node -- not
        just a new_mine-specific patch), then computes the actual
        shortest path between previous_point_a and old_node on the
        refreshed graph (_shortest_path_between), rather than assuming a
        straight line or testing one candidate point at a time -- a real
        graph search can route around whatever's in the way.

        Gives up if old_node's position isn't safe, or no path exists
        even after reconnecting -- can't defend against a mine discovered
        LATER; that's Field._cleanupInvalidatedConnections' job for the
        graph edges and check_path_envelopment's for stale path entries.

        Returns (prefix_path, tracked_node): prefix_path is the full node
        list from previous_point_a to old_node (inclusive), ready to
        splice into maze_b_path, or None if repair isn't possible.
        tracked_node is old_node when repair succeeded."""
        mine_x, mine_y = getattr(new_mine, "origin", None) or (
            new_mine.polygon.centroid.x, new_mine.polygon.centroid.y
        )
        safety_margin = self.mine_saftey_radius * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE
        other_obstacles = [
            o
            for o in list(self.nodeField.mines) + list(self.nodeField.unionObstacles)
            if o is not new_mine
        ]

        def point_safe(x, y):
            if math.hypot(x - mine_x, y - mine_y) < safety_margin or new_mine.contains_point((x, y)):
                return False
            return not any(o.contains_point((x, y)) for o in other_obstacles)  # margin only applies to new_mine

        if not point_safe(old_node.x, old_node.y):
            return None, None

        self.nodeField.fieldConnection.purgeConnections(old_node)
        for mine in self.nodeField.mines:
            self.nodeField._connectFloatingNodeToObstacle(mine, old_node)
        for union in self.nodeField.unionObstacles:
            self.nodeField._connectFloatingNodeToObstacle(union, old_node)
        self.nodeField._connectFloatingNodeToFloatingNode(old_node, previous_point_a)

        prefix_path = self._shortest_path_between(previous_point_a, old_node)
        if prefix_path is None:
            return None, None
        return prefix_path, old_node

    def check_path_envelopment(self, new_mine) -> None:
        """
        [Call after EVERY add_discovered_mine, for every maze variant --
        general catch-all, not specific to use_helper_nodes] Scans
        maze_confirmed_path, then maze_b_path, then maze_a_path (in path
        order) for the first node whose position now falls inside
        new_mine's polygon -- not just confirmed history, since a node
        can be sitting unconfirmed in B/A at the exact moment a merge
        grows to cover it, later folded into C unchecked. Also scans
        maze_confirmed_path's, then maze_b_path's, then maze_a_path's,
        own consecutive EDGES for one new_mine now crosses without
        containing either endpoint (mirrors check_merge_rewind's
        segment_hit trigger, but for every edge, not just ones anchored
        by a tracked promoted helper node -- this is what
        check_merge_rewind now defers to for any entry whose node has
        already been folded into confirmed history, rather than risking
        a repair-splice into frozen ground). The maze_a_path scan matters
        for a cross-pair retarget's freshly-created hop (see
        _try_apply_pending_approach_target): it can sit entirely within
        maze_a_path -- not yet advanced into maze_b_path -- at the exact
        moment a mine is discovered that grazes it without containing
        either endpoint; confirmed directly as a real bad edge on the
        two-pair safety sweep before this scan was added. The maze_b_path
        edge scan matters most for a wide-spanning Dijkstra result (e.g.
        patch_confirmed_span's local splice or reroute_b_segment's fresh
        search) that happens to route through an ordinary tangent-to-
        tangent edge between two unrelated mines -- not anchored by any
        tracked helper node, so check_merge_rewind's own segment_hit can't
        see it, and NODE-containment alone won't catch a mine crossing the
        edge without covering either endpoint.

        If a NODE hit, strips its connections so a recompute can't select
        it again: purgeConnections if it's an ordinary mine tangent node
        (the obstacle keeps the node itself for Field.expandField to
        re-wire later), else removeNode (a floating/helper node has no
        obstacle identity to preserve) -- an EDGE hit needs no such
        stripping, since Field's own _cleanupInvalidatedConnections already
        removes the specific invalidated edge from the graph the moment
        new_mine is added. Either way, truncates maze_confirmed_path to
        just before/at the hit and unconditionally re-derives
        maze_a_path/maze_b_path with the standard recompute + seam guard.
        Prunes self.promoted_helper_nodes to what's still referenced
        afterward.
        """
        # Scan maze_confirmed_path index by index, checking node-containment
        # and this index's own outgoing edge together at each step, so
        # whichever problem occurs FIRST (by path order) is what
        # determines truncation -- not "always prefer a node hit." A node
        # hit found by scanning the whole list first (the previous
        # approach: full node scan, THEN a separate full edge scan only if
        # no node hit) can land on a node that's contained by new_mine
        # PURELY BY COINCIDENCE much later in the path than a real edge
        # crossing earlier on, silently truncating far too little and
        # leaving the actually-crossed edge sitting in "confirmed" history
        # -- confirmed directly as a real bad edge on the two-pair
        # round-robin safety sweep: a node near the END of an 8-node
        # confirmed path happened to fall inside a newly-discovered mine,
        # so only that last node got truncated, while an earlier edge (at
        # index 4) that plainly crosses the SAME mine was never reached
        # because the node hit short-circuited the edge scan entirely.
        hit = None
        hit_in_confirmed = False
        edge_hit_index = None
        for i, node in enumerate(self.maze_confirmed_path):
            if new_mine.contains_point((node.x, node.y)):
                hit = node
                hit_in_confirmed = True
                break
            if i < len(self.maze_confirmed_path) - 1:
                p1, p2 = node, self.maze_confirmed_path[i + 1]
                if new_mine.intersects(((p1.x, p1.y), (p2.x, p2.y))):
                    edge_hit_index = i
                    break
        edge_hit_in_confirmed = edge_hit_index is not None
        if hit is None and edge_hit_index is None:
            hit = next(
                (n for n in self.maze_b_path + self.maze_a_path if new_mine.contains_point((n.x, n.y))),
                None,
            )
        if hit is None and edge_hit_index is None:
            for i in range(len(self.maze_b_path) - 1):
                p1, p2 = self.maze_b_path[i], self.maze_b_path[i + 1]
                if new_mine.intersects(((p1.x, p1.y), (p2.x, p2.y))):
                    edge_hit_index = i
                    break
        if hit is None and edge_hit_index is None:
            # maze_a_path's own edge(s) -- same crossing-without-containing-
            # either-endpoint case as maze_b_path above, just further out.
            # Missing this let a cross-pair retarget's freshly-created hop
            # (see _try_apply_pending_approach_target) sit unvalidated
            # against a mine discovered while that hop was still sitting
            # ENTIRELY within maze_a_path (before advancing into
            # maze_b_path on the next hop) -- node-containment already
            # covers maze_b_path + maze_a_path together above, but the
            # edge-crossing scan previously stopped at maze_b_path,
            # missing a crossing that grazes an edge without containing
            # either of ITS endpoints. edge_hit_index here is never used
            # for confirmed-path truncation (edge_hit_in_confirmed was
            # already fixed False before this scan runs), so it's safe to
            # reuse the same variable.
            for i in range(len(self.maze_a_path) - 1):
                p1, p2 = self.maze_a_path[i], self.maze_a_path[i + 1]
                if new_mine.intersects(((p1.x, p1.y), (p2.x, p2.y))):
                    edge_hit_index = i
                    break
        if hit is None and edge_hit_index is None:
            return

        if edge_hit_index is not None:
            if edge_hit_in_confirmed:
                self.maze_confirmed_path = self.maze_confirmed_path[: edge_hit_index + 1]
            # else: hit is in maze_b_path -- no confirmed-path truncation
            # needed (it isn't there); the unconditional recompute below
            # already discards/replaces the whole of maze_b_path/maze_a_path
            # regardless of where within them the invalidated edge was.
        else:
            if self._node_mine(hit) is not None:
                self.nodeField.fieldConnection.purgeConnections(hit)
            else:
                self.nodeField.fieldConnection.removeNode(hit)

            if hit_in_confirmed:
                truncate_at = next(i for i, n in enumerate(self.maze_confirmed_path) if n is hit)
                self.maze_confirmed_path = self.maze_confirmed_path[:truncate_at]

        recomputed = self._dijkstra_path_with_hysteresis(
            self._maze_c_start_nodes(), PATH_HYSTERESIS_TOLERANCE
        )
        recomputed = self._prepend_seam_if_needed(recomputed)
        if len(recomputed) < 2:
            self.maze_a_path = recomputed
            self.maze_b_path = []
        else:
            self.maze_a_path = recomputed[-2:]
            self.maze_b_path = recomputed[:-1]

        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []

    def patch_confirmed_span(self, new_mine):
        """[CROSS-PAIR] Call (via add_discovered_mine's prefer_local_patch=
        True) for a mine relayed from the OTHER pair, before
        check_path_envelopment gets a chance at it. Runs the identical
        node-containment-then-edge-crossing scan of maze_confirmed_path
        check_path_envelopment does, but on a hit, pins the two boundary
        nodes around it and splices in a fresh LOCAL shortest path between
        just those two -- not a full recompute of everything since. This
        pair likely moved well past this point by the time a remote
        discovery lands here (it's not this pair's own mine), so
        check_path_envelopment's usual "recompute from here to the end"
        would needlessly discard all of that progress; "theoretically it
        shouldn't impact the other pair at all, except them having a
        longer path" only holds if the fix stays local.

        Returns the new sub-path's node list (inclusive of both boundary
        nodes) if a patch was made -- the caller hands this to the
        DISCOVERING pair (not this one) to go verify/photograph, since
        that pair is the one physically nearby, even though the patch
        landed in this pair's confirmed history. Returns None if nothing
        hit, or if hit but the boundary is at either end of
        maze_confirmed_path (no room for a bounded patch) or no local
        path exists between the two boundary nodes -- either way,
        check_path_envelopment (called unconditionally right after this
        by add_discovered_mine) still handles it via its own, less
        targeted, always-safe full recompute."""
        # Scan index by index (node-containment, then this index's own
        # outgoing edge) so whichever problem occurs FIRST by path order
        # wins -- same fix as check_path_envelopment's identical scan: a
        # full node-containment pass over the whole list, falling back to
        # an edge scan only when NO node hit at all, can land on a node
        # that's contained purely by coincidence much later in the path
        # than a real, earlier edge crossing, patching around the wrong
        # (later) span while leaving the true crossing outside it.
        # check_path_envelopment (run unconditionally right after this)
        # still catches the real crossing on its own independent scan, so
        # this couldn't leave an unsafe edge behind -- but it could patch
        # a span that didn't need it while the actually-affected span
        # falls through to check_path_envelopment's far more expensive
        # full recompute instead of this method's cheap local splice.
        hit = None
        edge_hit_index = None
        for i, node in enumerate(self.maze_confirmed_path):
            if new_mine.contains_point((node.x, node.y)):
                hit = node
                break
            if i < len(self.maze_confirmed_path) - 1:
                p1, p2 = node, self.maze_confirmed_path[i + 1]
                if new_mine.intersects(((p1.x, p1.y), (p2.x, p2.y))):
                    edge_hit_index = i
                    break
        if hit is not None:
            hit_idx = next(i for i, n in enumerate(self.maze_confirmed_path) if n is hit)
            left_idx, right_idx = hit_idx - 1, hit_idx + 1
        elif edge_hit_index is not None:
            left_idx, right_idx = edge_hit_index, edge_hit_index + 1
        else:
            return None

        if left_idx < 0 or right_idx >= len(self.maze_confirmed_path):
            return None
        left_node = self.maze_confirmed_path[left_idx]
        right_node = self.maze_confirmed_path[right_idx]

        sub_path = self._shortest_path_between(left_node, right_node)
        if sub_path is None:
            return None

        self.maze_confirmed_path = (
            self.maze_confirmed_path[: left_idx + 1]
            + sub_path[1:-1]
            + self.maze_confirmed_path[right_idx:]
        )
        self._settle_confirmed_helper_nodes()
        return sub_path

    def retarget_approach_target(self, target_x: float, target_y: float) -> None:
        """[CROSS-PAIR] Queues (target_x, target_y) -- the OTHER pair's
        current point_A -- as this pair's next cross-pair rendezvous
        reference, then immediately tries to apply it (see
        _try_apply_pending_approach_target). Call whenever the other
        pair's point_A changes, including the first time (right after
        both pairs' start_maze_navigation()).

        Deliberately does NOT force a full recompute from point_C (an
        earlier version of this method did, and was reverted): that
        discards this pair's OWN in-progress local exploration every
        time the other pair's point_A merely shifts, which happens
        constantly whenever the other pair is doing the bulk of a
        mission's discovery work -- observed directly on a real seed:
        one pair's gambler got retargeted 9 times over a mission and
        NEVER made forward progress, because every retarget threw away
        wherever it currently was and restarted from point_C. Chaining a
        bounded hop from wherever this pair's gambler ALREADY is (see
        _try_apply_pending_approach_target) instead lets local,
        mine-driven exploration (on_forward_mine_discovered/
        start_helper_node_detour) keep running completely undisturbed in
        between cross-pair retargets.

        Also deliberately does NOT hop all the way to (target_x,
        target_y) -- only to the HALFWAY point along this pair's own
        real (obstacle-respecting) path there (see
        _path_halfway_point). Reaching all the way to the other pair's
        actual current position would mean this pair's gambler flying
        into territory that's the OTHER pair's job to explore, before
        the two pairs have actually met -- redundant coverage at best, a
        collision risk at worst. Halving it every time means the gap
        between the two pairs' point_A's keeps closing without either
        side ever crossing into the other's still-unexplored half."""
        self._pending_approach_target = (target_x, target_y)
        self._try_apply_pending_approach_target()

    def _try_apply_pending_approach_target(self) -> None:
        """[CROSS-PAIR] Applies self._pending_approach_target (if any) as
        a single chained hop extending the CURRENT approach target
        (maze_a_path[-1], continuing forward) to a fresh floating node at
        the halfway position -- same MAX_CHAIN_DEPTH cap as
        start_helper_node_detour so gambler can't race arbitrarily far
        ahead of its own assistant, but NOT the same bridge splice: a
        mine detour discards the old segment A from its near boundary
        (previous_point_a = maze_a_path[0]) since an obstacle now sits
        somewhere along it, while a cross-pair retarget only ever extends
        PAST wherever the last hop already reached, preserving the whole
        existing maze_b_path + maze_a_path rather than re-deriving it via
        _bridge_up_to (see the continuing_chain branch below).

        Call this after anything that might free up a chain slot
        (confirm_b_into_c/advance_b_prefix_into_c, which fold a chained
        hop's node into confirmed history the moment its footprint is
        fully seen) as well as from retarget_approach_target itself --
        "unless bound by its assistant catching up": if the chain is
        already at MAX_CHAIN_DEPTH, this leaves the target queued and
        does nothing, exactly like start_helper_node_detour falls back to
        waiting rather than opening a new detour past the cap.

        No-ops if there's no target queued, or if this pair's own graph
        isn't set up yet at all (self.startingNodes still empty -- before
        buildNodeField has ever run)."""
        if self._pending_approach_target is None:
            return
        self.cross_pair_target_chain = [
            entry for entry in self.cross_pair_target_chain
            if any(n is entry["node"] for n in self.maze_b_path + self.maze_a_path)
        ]
        if len(self.cross_pair_target_chain) >= MAX_CHAIN_DEPTH:
            return

        target_x, target_y = self._pending_approach_target
        # Continue extending from wherever the LAST retarget hop actually
        # left off (maze_a_path[-1], the current approach target), not
        # from point_A's own near boundary (maze_a_path[0]) -- the user's
        # own spec is "continue walking along the path... attach to the
        # PREVIOUS EDGE and the new point A," i.e. keep going forward,
        # don't restart from the segment's start every call. Anchoring at
        # [0] unconditionally was a real bug: nothing else ever advances
        # maze_a_path[0] once cross-pair retargeting takes over (it isn't
        # touched by confirm_b_into_c/advance_b_prefix_into_c, which only
        # fold maze_b_path forward), so every subsequent call recomputed
        # the exact same distance to the exact same halfway point forever
        # -- confirmed directly as the cause of a real hit_cap: seed 6 of
        # the two-pair safety sweep got stuck re-confirming one
        # never-advancing waypoint for 20+ straight rounds.
        #
        # Only treat this as "continuing" when self._approach_target_node
        # (this pair's own last-set target) is STILL maze_a_path's current
        # end -- if a local mine-driven reroute reset maze_a_path in
        # between (on_forward_mine_discovered/check_path_envelopment can
        # still land back on _approach_target_node via _resolve_node_near,
        # but can just as easily land elsewhere), that's a fresh start and
        # should anchor at the real current point_A like the first-ever
        # retarget does.
        #
        # maze_a_path can also be EMPTY here (not just "not continuing") --
        # this pair fully arrived at its own previous target and folded
        # everything into maze_confirmed_path, or check_path_envelopment
        # just removed self._approach_target_node itself (e.g. a new mine's
        # merge grew to cover that exact floating-node position) without
        # anything else ever resetting self.endingNodes afterward. Either
        # way, self.endingNodes can now be a dead reference: every future
        # recompute in this file targets self.endingNodes unconditionally,
        # so with the ONLY entry unreachable, _dijkstra_path_with_hysteresis
        # returns [] forever, collapsing maze_a_path to empty permanently --
        # and this method used to just return here, discarding the queued
        # target forever too, since nothing else can ever re-populate
        # maze_a_path to satisfy the old guard. Falling back to
        # _maze_c_start_nodes() (same anchor on_forward_mine_discovered
        # uses from a cold start) lets a fresh hop -- and thus a fresh,
        # LIVE self.endingNodes -- get established again on success,
        # self-healing the dead reference. Confirmed directly: at a denser
        # mine field (90 vs the usual 70), this hit on the large majority
        # of seeds; at 70 it was still common but happened to always occur
        # after this pair's own coverage was already complete, masking it
        # as a real coverage gap purely by chance of timing, not by design.
        if not self.maze_a_path:
            start_candidates = self._maze_c_start_nodes()
            if not start_candidates:
                return
            continuing_chain = False
            previous_point_a = start_candidates[0]
        else:
            continuing_chain = (
                self._approach_target_node is not None
                and self.maze_a_path[-1] is self._approach_target_node
            )
            previous_point_a = self.maze_a_path[-1] if continuing_chain else self.maze_a_path[0]

        # Measure the REAL (graph, obstacle-respecting) path toward the
        # other pair's raw position first, using a throwaway probe node --
        # removed again immediately, since what we actually want isn't
        # "how do I reach the other pair's point_A" but "how do I reach
        # the HALFWAY point along that real path" (see
        # _path_halfway_point). A straight-line (x, y) average of the two
        # positions was tried first and rejected: it ignores obstacles
        # entirely, so "halfway" by straight-line distance can be a very
        # different, and sometimes unreachable-without-a-detour, position
        # than halfway by actual flying distance.
        probe = self.nodeField.addFloatingNode(target_x, target_y)
        full_path = self._shortest_path_between(previous_point_a, probe)
        self.nodeField.fieldConnection.removeNode(probe)
        if full_path is None or len(full_path) < 2:
            hop = None
            self._pending_approach_target = None
        else:
            # Once the remaining real-path distance is already small,
            # close it fully instead of halving again -- see
            # CLOSE_ENOUGH_TO_STOP_HALVING_FT's own comment for why
            # unconditional halving never actually finishes. If we're
            # NOT closing fully this call, re-queue the SAME (target_x,
            # target_y) -- rather than clearing it -- so a later call
            # (confirm_b_into_c/advance_b_prefix_into_c already retry
            # _try_apply_pending_approach_target every round) keeps
            # chipping away at the remaining half instead of the gap
            # getting silently abandoned the moment the OTHER pair's own
            # point_A happens to stop moving (which is exactly what
            # _sync_approach_target's own "did it change" check is keyed
            # on -- it has no way to know a PAST retarget only partially
            # closed the gap). Confirmed directly: without this, halving
            # can leave the two pairs permanently a fraction of a foot
            # apart, missing the final cell and hitting the round cap
            # waiting for a retry that would otherwise never come.
            fully_closing = node_path_length(full_path) <= CLOSE_ENOUGH_TO_STOP_HALVING_FT
            if fully_closing:
                halfway_x, halfway_y = target_x, target_y
                self._pending_approach_target = None
            else:
                halfway_x, halfway_y = self._path_halfway_point(full_path)
                self._pending_approach_target = (target_x, target_y)
            # Only remove the PREVIOUS retarget's floating node if it isn't
            # currently serving as point_A itself. A local, mine-driven
            # reroute in between two retargets (on_forward_mine_discovered/
            # check_path_envelopment, via _maze_c_start_nodes' own
            # _resolve_node_near) picks whichever live node is geometrically
            # CLOSEST to wherever point_C currently ends -- if the previous
            # retarget's own target node happens to be that closest node
            # (increasingly likely as retargeting narrows the gap between the
            # two pairs), it can legitimately become the new point_A. Removing
            # it here regardless would leave previous_point_a referencing a
            # node with no graph entry at all, crashing the
            # _shortest_path_between call below with a KeyError -- confirmed
            # directly as the actual cause of a real crash on a two-pair
            # sweep seed.
            if self._approach_target_node is not None and self._approach_target_node is not previous_point_a:
                self.nodeField.fieldConnection.removeNode(self._approach_target_node)
            new_target = self.nodeField.addFloatingNode(halfway_x, halfway_y)
            self._approach_target_node = new_target
            self.endingNodes = [new_target]
            hop = self._shortest_path_between(previous_point_a, new_target)

        if hop is None:
            # Not reachable directly from wherever gambler currently is
            # (e.g. an obstacle now sits between them) -- self-heals via
            # the same full-recompute-from-point_C fallback every other
            # "can't get there from here" case in this file already uses.
            recomputed = self._dijkstra_path_with_hysteresis(
                self._maze_c_start_nodes(), PATH_HYSTERESIS_TOLERANCE
            )
            recomputed = self._prepend_seam_if_needed(recomputed)
            if len(recomputed) < 2:
                self.maze_a_path = recomputed
                self.maze_b_path = []
            else:
                self.maze_a_path = recomputed[-2:]
                self.maze_b_path = recomputed[:-1]
            self._prune_promoted_helper_nodes()
            self.remote_mine_placeholders = []
            return

        # _bridge_up_to(previous_point_a) only finds real prior history
        # when previous_point_a is ITSELF a maze_b_path member (the
        # start_helper_node_detour case: the whole old segment A is being
        # thrown away and replaced from its near boundary). Continuing an
        # existing chain instead extends PAST the current end -- the
        # entire current maze_b_path + maze_a_path (which already ends at
        # previous_point_a) needs to be kept, not re-derived, or every
        # call would discard the real bridge same as the bug above.
        full = (
            self.maze_b_path + self.maze_a_path + hop[1:]
            if continuing_chain
            else self._bridge_up_to(previous_point_a) + hop[1:]
        )
        self.cross_pair_target_chain.append({"node": new_target, "previous_point_a": previous_point_a})
        self.maze_a_path = full[-2:]
        self.maze_b_path = full[:-1]
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []

    def get_cross_pair_patch_places_to_check(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ) -> list:
        """[CROSS-PAIR] Places-to-check for the oldest pending entry in
        self.cross_pair_patches -- a confirmed-history patch this pair's
        assistant owes a verification flight to (patch_confirmed_span ran
        on the OTHER pair, in response to a mine THIS pair discovered).
        The graph fix already happened when the patch was created; this
        is purely a coverage/verification task so the mission's "everything
        got photographed" invariant still holds for the patched stretch.
        Caller pops self.cross_pair_patches[0] once its footprint is fully
        seen (same drain pattern as segment B)."""
        if not self.cross_pair_patches:
            return []
        return self._places_to_check_for_path(
            self.cross_pair_patches[0], method=method, overlap=overlap,
            path_width=path_width, shape_size_ft=shape_size_ft,
        )

    def reroute_b_segment(self) -> None:
        """[Rule 2] Re-searches directly between point_C and point_A
        (fixed) on the current graph and replaces self.maze_b_path --
        a mine in B can only make the route more roundabout, never move
        either endpoint. Both ends get the same seam-guard as
        on_forward_mine_discovered."""
        if len(self.maze_a_path) < 1:
            return
        start_node = self._maze_c_start_nodes()[0]
        target_pos = (self.maze_a_path[0].x, self.maze_a_path[0].y)
        target_node = self._resolve_node_near(*target_pos)
        newGraph = Graph(self.nodeField.fieldConnection.nodeGraph)
        distances, predecessors = newGraph.shortest_distances(start_node)
        if distances.get(target_node, math.inf) == math.inf:
            return  # structural break, not just "needs a detour" -- self-heals next round
        path_nodes = []
        current = target_node
        while current:
            path_nodes.append(current)
            current = predecessors[current]
        path_nodes.reverse()
        path_nodes = self._prepend_seam_if_needed(path_nodes)
        real_a_start = self.maze_a_path[0]
        same_point = target_node is real_a_start or (
            abs(target_node.x - real_a_start.x) < 1e-6 and abs(target_node.y - real_a_start.y) < 1e-6
        )
        if not same_point:
            path_nodes.append(real_a_start)
        self.maze_b_path = path_nodes
        self._prune_promoted_helper_nodes()

    def _node_mine(self, node):
        """Returns the BlockMine/union obstacle whose .nodes list contains
        node, or None. Tangent nodes carry no parentMine back-reference,
        so this is the only way to ask "whose obstacle is this"."""
        for obstacle in list(self.nodeField.mines) + list(self.nodeField.unionObstacles):
            if node in obstacle.nodes:
                return obstacle
        return None

    def reroute_b_segment_same_mine(self) -> None:
        """[Rule 2 variant] Recomputes fresh from point_C like Rule 1, but
        only adopts the new point_A if its final-edge node sits on the
        SAME mine point_A is currently pinned to (free to slide to a
        different tangent of that mine, never jump to a different one).
        Falls back to reroute_b_segment (point_A held fixed) otherwise."""
        if not self.maze_a_path:
            self.reroute_b_segment()
            return
        current_a_node = self.maze_a_path[0]
        current_mine = self._node_mine(current_a_node)
        if current_mine is None:
            self.reroute_b_segment()
            return

        recomputed = self._dijkstra_path_with_hysteresis(
            self._maze_c_start_nodes(), PATH_HYSTERESIS_TOLERANCE
        )
        recomputed = self._prepend_seam_if_needed(recomputed)
        if len(recomputed) < 2:
            self.reroute_b_segment()
            return
        candidate_a_node = recomputed[-2]
        if self._node_mine(candidate_a_node) is not current_mine:
            self.reroute_b_segment()
            return

        self.maze_a_path = recomputed[-2:]
        self.maze_b_path = recomputed[:-1]
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []

    def _extend_confirmed_path(self, new_content: list) -> None:
        """Appends new_content onto self.maze_confirmed_path, dropping the
        duplicate leading node when new_content[0] is the same point as
        maze_confirmed_path[-1] (the ordinary case). If new_content
        re-visits a node ALREADY earlier in maze_confirmed_path -- a real
        loop, not a coincidence: field start/end and mine tangent nodes
        are each created exactly once, so an identity match here means
        the route genuinely backtracked to a point already confirmed, not
        two unrelated positions sharing a re-resolved node reference (the
        failure mode _bridge_up_to's own confirmed-path search hit
        earlier this session -- see its docstring) -- truncates back to
        just before that node's first occurrence first, so the node ends
        up appearing exactly once instead of the path visiting it twice.
        This can happen after check_path_envelopment/patch_confirmed_span
        remove a node early in the mission, when few alternate routes are
        established yet and the only remaining path has to double back
        near an already-confirmed point."""
        if not new_content:
            return
        if not self.maze_confirmed_path:
            self.maze_confirmed_path = list(new_content)
            return
        for i, node in enumerate(new_content):
            for j, existing in enumerate(self.maze_confirmed_path):
                if node is existing:
                    self.maze_confirmed_path = self.maze_confirmed_path[:j] + new_content[i:]
                    return
        first, last = new_content[0], self.maze_confirmed_path[-1]
        same_point = first is last or (abs(first.x - last.x) < 1e-6 and abs(first.y - last.y) < 1e-6)
        self.maze_confirmed_path.extend(new_content[1:] if same_point else new_content)

    def confirm_b_into_c(self) -> None:
        """Appends self.maze_b_path onto self.maze_confirmed_path (frozen
        history of the route actually flown and verified clean) via
        _extend_confirmed_path, then clears maze_b_path. Call once
        get_places_to_check_maze()'s "b" list drains to empty; idempotent
        if b is already empty."""
        if not self.maze_b_path:
            return
        self._extend_confirmed_path(self.maze_b_path)
        self.maze_b_path = []
        self._settle_confirmed_helper_nodes()
        # Folding B into confirmed can free up a cross_pair_target_chain
        # slot (see _try_apply_pending_approach_target) -- retry a
        # retarget that was queued because the chain was previously at
        # MAX_CHAIN_DEPTH, now that the assistant has caught up.
        self._try_apply_pending_approach_target()

    def advance_b_prefix_into_c(self) -> None:
        """Folds every leading edge of self.maze_b_path whose rasterized
        footprint is already fully seen into self.maze_confirmed_path (via
        _extend_confirmed_path) -- not just the whole of B once it drains
        clean -- shrinking what a later reroute has to re-search."""
        if len(self.maze_b_path) < 2:
            return

        best_idx = 0
        for i in range(1, len(self.maze_b_path)):
            prefix_cells = self.rasterize_node_path(self.maze_b_path[: i + 1])
            if (prefix_cells & ~self.seen_tracker).count() == 0:
                best_idx = i
            else:
                break
        if best_idx == 0:
            return

        self._extend_confirmed_path(self.maze_b_path[: best_idx + 1])
        self.maze_b_path = self.maze_b_path[best_idx:]
        self._settle_confirmed_helper_nodes()
        self._try_apply_pending_approach_target()

    def get_maze_path(self):
        """Returns maze_confirmed_path + maze_b_path + maze_a_path, joined
        without duplicating a shared boundary node (only drops one when
        it's actually the same point -- see confirm_b_into_c). The actual
        route committed to and checked so far -- verify coverage against
        this, not get_shortest_path (a different, independently
        recomputed route)."""

        def _join(pieces, next_piece):
            if not pieces:
                pieces.extend(next_piece)
                return
            first, last = next_piece[0], pieces[-1]
            same_point = first is last or (abs(first.x - last.x) < 1e-6 and abs(first.y - last.y) < 1e-6)
            pieces.extend(next_piece[1:] if same_point else next_piece)

        pieces = []
        if self.maze_confirmed_path:
            pieces.extend(self.maze_confirmed_path)
        if self.maze_b_path:
            _join(pieces, self.maze_b_path)
        if self.maze_a_path:
            _join(pieces, self.maze_a_path)
        return pieces

    def get_places_to_check_maze(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ) -> dict:
        """Same coverage params as getPlacesToCheck. Computes places-to-check
        separately for maze_a_path and maze_b_path. Segment B's TSP
        ordering is anchored to next_place_to_check_maze_b (mirrors
        nextPlaceToCheckLocal) so a replan doesn't reorder the tour to
        start from the opposite side of the segment. Segment A's is
        instead anchored directly to maze_a_path[0] (point_A) itself,
        not the remembered next_place_to_check_maze_a: point_A is
        ALWAYS this segment's own current near/home-side boundary by
        construction, so it can't go stale the way a remembered "wherever
        the tour started last time" value can once the segment gets
        reshaped elsewhere (a cross-pair retarget, Rule 1 firing on a
        newly-discovered mine, etc.) without that value ever being
        cleared. A stale anchor doesn't just cost one extra hop -- since
        order_waypoints seeds nearest-neighbor from it when it doesn't
        exactly match any current candidate, whichever real candidate
        happens to be closest to the STALE anchor becomes the tour's
        first stop, which can be the far end of the segment instead of
        the near end, dragging gambler's entire visiting order backwards
        (observed directly: a gambler's tour started right next to the
        far target and walked back toward its own point_A instead of the
        reverse). Anchoring to point_A directly is immune to this since
        there's no remembered value to go stale in the first place.
        Returns {"a": [(lat,lon),...], "b": [...], "count": total}."""
        a_anchor = None
        if self.maze_a_path:
            a0 = self.maze_a_path[0]
            a_anchor = (a0.x, a0.y)
        a_places = (
            self._places_to_check_for_path(
                self.maze_a_path, method=method, overlap=overlap,
                path_width=path_width, shape_size_ft=shape_size_ft,
                fixed_first=a_anchor,
                next_place_attr="next_place_to_check_maze_a",
            )
            if len(self.maze_a_path) >= 2 else []
        )
        b_places = (
            self._places_to_check_for_path(
                self.maze_b_path, method=method, overlap=overlap,
                path_width=path_width, shape_size_ft=shape_size_ft,
                fixed_first=self.next_place_to_check_maze_b,
                next_place_attr="next_place_to_check_maze_b",
            )
            if len(self.maze_b_path) >= 2 else []
        )
        return {"a": a_places, "b": b_places, "count": len(a_places) + len(b_places)}

    # GAMBLER/ASSISTANT pair roles: within a pair sharing this Pathfinder,
    # the gambler pushes segment A (on_forward_mine_discovered/
    # start_helper_node_detour) and the assistant clears segment B
    # (reroute_b_segment*/confirm_b_into_c) -- these just slice
    # get_places_to_check_maze's combined dict down to one role's queue.

    def get_gambler_places_to_check(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ) -> list:
        """Segment-A places-to-check only -- what the gambler in a pair
        should fly next."""
        return self.get_places_to_check_maze(method, overlap, path_width, shape_size_ft)["a"]

    def get_assistant_places_to_check(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ) -> list:
        """Segment-B places-to-check only -- what the assistant in a pair
        should fly next."""
        return self.get_places_to_check_maze(method, overlap, path_width, shape_size_ft)["b"]

    # Checkpoint-pinned path planning, and a pre-hysteresis waypoint
    # generator, are archived: see flight/pathfinding/archive/pathfinder_unused.py.
