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
import hashlib
import logging
import math

from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer
from flight.pathfinding.path_subdivision import Path
from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.nodeField.node import Node
from flight.pathfinding.nodeField.polygonObstacle import hash_position
from flight.pathfinding.path_calculation import Graph
from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.protoMine import protoMine, safety_cells
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

# format_iarc_path lives in flight/pathfinding/iarc_report.py so either planner can
# report without importing the other; re-exported here for existing callers.
from flight.pathfinding.iarc_report import format_iarc_path  # noqa: E402,F401

# Ending-node hysteresis (fraction of best route length); see _dijkstra_path_with_hysteresis.
# Absorbs sub-1ft near-ties without hiding a real detour.
PATH_HYSTERESIS_TOLERANCE = 0.03

# Max outstanding (unconfirmed) helper-node detours; see start_helper_node_detour.
# Chains rarely exceed 3 anyway; this is a backstop against the gambler racing far
# ahead of a stuck assistant.
MAX_CHAIN_DEPTH = 3


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

        # A mine's danger-zone radius in FEET (not cells) -- protoMine takes it as feet and
        # derives its cell count via safety_cells. TODO: get from the coord converter.
        # The safety_margin sites below read it as CELLS instead (radius * WIDTHOFSQUARE),
        # so at 3 they model a 6ft radius against protoMine's 3ft -- conservative, so left
        # alone deliberately; tightening them changes flown margins and needs its own sweep.
        self.mine_safety_radius_ft = 3

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

        # Position hashes of discovered mines, in discovery order.
        # - Hashed from the RAW discovered position (not the merged obstacle).
        # - self_mine_order: own finds. other_mine_order: relayed from the other pair/solo.
        # - Never reset by start_maze_navigation (mission-long history).
        # - Backs the order-SENSITIVE self/other_mine_order_hash, which catches a relay
        #   that arrived out of order or dropped (Field.mineHash ignores order).
        self.self_mine_order: list[str] = []
        self.other_mine_order: list[str] = []

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

        # MAZE-STYLE state (start_maze_navigation): A = approach (final edge, anchor
        # point_A), B = bridge (reroutable), C = confirmed (frozen history).
        self.maze_a_path = []
        self.maze_b_path = []
        self.maze_confirmed_path = []
        # Per-segment TSP anchors, same idea as nextPlaceToCheckLocal.
        self.next_place_to_check_maze_a = None
        self.next_place_to_check_maze_b = None

        # HELPER-NODE state (use_helper_nodes).
        # - helper_node_trail: (distance, x, y) breadcrumbs from segment A, oldest first.
        # - promoted_helper_nodes: [{"node", "previous_point_a", "obstacle"}] for breadcrumbs
        #   promoted to graph nodes.
        self.helper_node_trail = []
        self.promoted_helper_nodes = []

        # CROSS-PAIR state.
        # - remote_mine_placeholders: [{"position", "obstacle"}] other-pair mines on this
        #   pair's segment A not yet reached (see record_remote_mine_on_segment_a).
        # - cross_pair_patches: patches this pair's assistant owes a verification flight
        #   (see patch_confirmed_span); they were spliced into the OTHER pair's history.
        self.remote_mine_placeholders = []
        self.cross_pair_patches = []
        # Nodes of the midline seam run, once join_cross_pair_seam has closed it.
        self.cross_pair_seam_span = []
        # True once replan_toward asked the gambler to fly segment A from its far end.
        self.approach_from_far_end = False
        # Result of the last add_discovered_mine(prefer_local_patch=True) patch attempt.
        # Kept off the return tuple so other callers' unpacking is unaffected.
        self.last_patched_span = None

        # Set the singleton instance
        if Pathfinder.instance is None:
            Pathfinder.instance = self

    # Thin wrapper over the module-level order_waypoints (testable without a Pathfinder).
    def order_waypoints(
        self, coords: list[tuple[float, float]], fixed_first: tuple[float, float] = None
    ) -> list[tuple[float, float]]:
        return order_waypoints(coords, fixed_first=fixed_first)

    def buildNodeField(self, startLocation: tuple[float, float], startEdge: str = "bottom"):
        """
        Takes:
        - startLocation: (lat, lon) of the entry point.
        - startEdge: "bottom" (default: enter at y=-1, end row at the top) or "top"
          (mirrored, for a pair working from the opposite end).
        """
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
        # Fallback ending row for _dijkstra_path_with_hysteresis if endingNodes becomes
        # unreachable (set_cross_pair_home_boundary replaces it for two-pair missions).
        self._original_ending_nodes = list(self.endingNodes)

    def set_cross_pair_home_boundary(self, boundary_y: float) -> None:
        """
        [CROSS-PAIR] Give this pair a FIXED target row (its own half of the field).

        Takes:
        - boundary_y: y of the row, usually the field midline.

        Does:
        - Replaces endingNodes AND _original_ending_nodes with a fresh row of floating
          nodes at boundary_y (same spacing/redundancy as buildNodeField's far-edge row).
        - Call once, after buildNodeField and before start_maze_navigation.

        Why:
        - Replaces the old chase-the-other-pair retargeting (retarget_approach_target).
          Its halving had no lower bound, and its single target node often became
          unreachable in dense minefields; the fallback then hit the far edge, deep in the
          other pair's half. Seen on 5/5 dense seeds.
        - A retry node at the same spot was tried and didn't help: the whole area is
          usually walled off, not just one node.
        - A fixed redundant row has nothing to go stale.

        Limitation: a pair that finishes early can't help the other; that needs a separate
        load-balancing mechanism.
        """
        for node in list(self.endingNodes):
            self.nodeField.fieldConnection.removeNode(node)
        self.endingNodes = []
        for i in range(WIDTHOFFIELD // (WIDTHOFSQUARE * 2)):
            self.endingNodes.append(
                self.nodeField.addFloatingNode(
                    i * WIDTHOFSQUARE * 2 + WIDTHOFSQUARE // 2, boundary_y
                )
            )
        self._original_ending_nodes = list(self.endingNodes)

    def join_cross_pair_seam(self, other_endpoint: tuple[float, float]) -> bool:
        """
        [CROSS-PAIR] Close the lateral gap between this pair's route end and the other's.

        Takes:
        - other_endpoint: the OTHER pair's final path endpoint, (x, y) local.

        Does:
        - No-op if the two ends are already within one grid cell of each other.
        - Otherwise routes from this pair's own endpoint to other_endpoint on the live
          graph and appends that run to maze_b_path, so it gets photographed like any
          other segment-B stretch rather than being assumed safe.

        Why:
        - set_cross_pair_home_boundary gives each pair a redundant ROW of ending nodes, so
          the two pairs can pick different columns and stop up to the field's width apart.
          Measured: a nonzero gap in 3 of 8 seeds, up to 24ft. The mission deliverable is
          ONE connected path, so an unphotographed gap at the join is the one place a
          discontinuity cannot be tolerated -- and format_iarc_path now raises on it
          rather than silently emitting a route that skips it.
        - Call once both pairs have finished their own halves and their endpoints are known.

        Returns: True if a joining run was queued (caller must keep flying this pair until
        segment B drains), False if no join was needed or no route exists.
        """
        own_path = self.get_maze_path()
        if not own_path:
            return False
        own_end = own_path[-1]
        if math.hypot(own_end.x - other_endpoint[0], own_end.y - other_endpoint[1]) <= WIDTHOFSQUARE:
            return False

        target = self.nodeField.addFloatingNode(*other_endpoint)
        seam = self._shortest_path_between(own_end, target)
        if seam is None or len(seam) < 2:
            self.nodeField.fieldConnection.removeNode(target)
            return False

        # The seam has to extend the END of the whole route. get_maze_path() returns
        # C + B + A, so appending to B would leave segment A trailing after the seam and
        # the route would still end where it did before. The caller only calls this once
        # both queues have drained -- i.e. everything flown is already photographed -- so
        # freeze the current route as confirmed and make the seam the new segment B.
        self.maze_confirmed_path = list(own_path)
        self.maze_a_path = []
        self.maze_b_path = seam
        # Recorded for diagnostics. Not exempt from the home-half check: a seam rounding a
        # mine ON the midline bows a few feet past it, which the check's bounded tolerance
        # already admits -- exempting seam nodes instead once hid a 115ft excursion.
        self.cross_pair_seam_span = list(seam)
        self._settle_confirmed_helper_nodes()
        return True

    def _split_index_by_unseen(self, route: list) -> int:
        """Index splitting `route` into two halves of roughly equal UNPHOTOGRAPHED work.

        Splitting by node count would be meaningless -- nodes are obstacle tangents, so
        they cluster wherever mines are, not where the work is.
        """
        total = (self.rasterize_node_path(route) & ~self.seen_tracker).count()
        if total == 0:
            return max(1, len(route) - 2)
        half = total / 2.0
        for i in range(1, len(route)):
            if (self.rasterize_node_path(route[: i + 1]) & ~self.seen_tracker).count() >= half:
                return min(max(i, 1), len(route) - 2)
        return max(1, len(route) - 2)

    def replan_toward(self, target_xy: tuple[float, float], backwards: bool = True) -> bool:
        """
        Scrap accumulated detour/patch state and lay one clean route to target_xy.

        Takes:
        - target_xy: where to aim -- in a two-pair mission, the OTHER pair's route end.
        - backwards: have the gambler work segment A from its far end (see below).

        Does:
        - Drops the repair scaffolding: promoted helper nodes (and their graph nodes),
          the breadcrumb trail, remote placeholders and pending cross-pair patches.
          maze_confirmed_path is KEPT -- it is photographed work, not scaffolding.
        - Recomputes a single shortest route from point_C's end to target_xy.
        - Splits the route's remaining unphotographed work in half: the near half becomes
          segment B (assistant), the far half segment A (gambler).

        Why split this way, and why backwards:
        - Normally the gambler pushes A forward and the assistant follows, so the
          assistant is always chasing a frontier that keeps moving (measured at 1.58x a
          solo drone's photo count). Anchoring A at its far end instead sends the two
          roles toward each other, so the unverified span shrinks from both ends and
          neither chases the other.
        - A mine found afterwards is handled by the normal rules, which collapse A back to
          a single edge -- this is a one-shot reset, not a new steady state.

        Returns: True if a route was laid, False if none exists (caller keeps what it had).
        """
        start_nodes = self._maze_c_start_nodes()
        if not start_nodes:
            return False
        target = self.nodeField.addFloatingNode(*target_xy)
        route = self._shortest_path_between(start_nodes[0], target)
        if route is None or len(route) < 2:
            self.nodeField.fieldConnection.removeNode(target)
            return False
        if len(route) == 2:
            # A straight shot has no interior node to split at, which is the common case
            # once the gap is short. Insert one at the midpoint: the edge is already a
            # graph edge, so it is clear of every known obstacle along its whole length.
            (x0, y0), (x1, y1) = (route[0].x, route[0].y), (route[1].x, route[1].y)
            mid = self.nodeField.addFloatingNode((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            route = [route[0], mid, route[1]]

        for entry in self.promoted_helper_nodes:
            self.nodeField.fieldConnection.removeNode(entry["node"])
        self.promoted_helper_nodes = []
        self.helper_node_trail = []
        self.remote_mine_placeholders = []
        self.cross_pair_patches = []

        route = self._prepend_seam_if_needed(route)
        idx = self._split_index_by_unseen(route)
        self.maze_b_path = route[: idx + 1]
        self.maze_a_path = route[idx:]
        self.approach_from_far_end = backwards
        return True

    def _obstacle_containing(self, x: float, y: float):
        """Returns the live mine/union containing (x,y), or None.
        Looks up by position, not index (a merge can move obstacles around the list)."""
        for obstacle in list(self.nodeField.mines) + list(self.nodeField.unionObstacles):
            if obstacle.contains_point((x, y)):
                return obstacle
        return None

    def add_discovered_mine(self, mine_lat: float, mine_lon: float, prefer_local_patch: bool = False):
        """
        Add a discovered mine and repair any path state it invalidates.

        Takes:
        - mine_lat, mine_lon: the mine position.
        - prefer_local_patch: [CROSS-PAIR] set True for a mine relayed from the OTHER pair.
          A confirmed-path hit is then offered to patch_confirmed_span (small local splice)
          before check_path_envelopment's full recompute, so this pair keeps its progress.
          Also picks which order list records the mine (other vs self).

        Returns: (obstacle, was_merged, rewound)
        - obstacle: the live mine/union now at this position.
        - was_merged: whether it merged with an existing obstacle.
        - rewound: whether check_merge_rewind already re-routed A/B (caller should then
          skip its own discovery handler).

        Touches: self.last_patched_span (patch result, kept off the return tuple).
        """
        self.last_patched_span = None
        x, y = self.coord_converter.latlon_to_local(mine_lat, mine_lon)
        Xsquare, Ysquare = self.nodeField.getSquareCoordinates(x, y)
        # mid centers the safety-radius block grid on (Xsquare, Ysquare). Must match
        # protoMine's own placeholderGridSideLength // 2 -- hence the shared safety_cells.
        mid = safety_cells(self.mine_safety_radius_ft)
        Xoffset = WIDTHOFSQUARE * (Xsquare - mid)
        Yoffset = WIDTHOFSQUARE * (Ysquare - mid)
        newProtoMine = protoMine(self.mine_safety_radius_ft, (mine_lat, mine_lon), (Xoffset, Yoffset))
        self.protoMines.append(newProtoMine)
        # Hash the mine's own position, not the obstacle it merges into (a union's
        # hash changes as it grows; this mine's identity must stay fixed).
        mine_hash = hash_position(*newProtoMine.mineLocation)
        (self.other_mine_order if prefer_local_patch else self.self_mine_order).append(mine_hash)

        was_merged = self.nodeField.addFromProtoMine(newProtoMine)

        mineBlockField = build_mine_cell_field(newProtoMine)
        self.mineFieldTracker.apply_mask(mineBlockField, Xsquare - mid, Ysquare - mid, op="or")

        new_mine_obstacle = self._obstacle_containing(x, y)
        rewound = False
        if new_mine_obstacle is not None:
            # Order: check_merge_rewind (most targeted), then patch_confirmed_span if
            # requested, then check_path_envelopment (safe catch-all).
            rewound = self.check_merge_rewind(new_mine_obstacle, was_merged)
            if prefer_local_patch:
                self.last_patched_span = self.patch_confirmed_span(new_mine_obstacle)
            self.check_path_envelopment(new_mine_obstacle)
        return new_mine_obstacle, was_merged, rewound

    def self_mine_order_hash(self) -> str:
        """
        ORDER-sensitive hash of self.self_mine_order (own discoveries, in order).
        - Field.mineHash ignores order; this changes if the same mines came in a
          different order.
        - Compare against the other side's other_mine_order_hash: a mismatch means a
          relay arrived out of order, duplicated, or dropped.
        - Not wired to an interdrone message yet.
        """
        return hashlib.sha256(repr(self.self_mine_order).encode("utf-8")).hexdigest()

    def other_mine_order_hash(self) -> str:
        """Like self_mine_order_hash, for other_mine_order (relayed mines, in the order
        this Pathfinder applied them)."""
        return hashlib.sha256(repr(self.other_mine_order).encode("utf-8")).hexdigest()

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

    # Safety cap on repair_path_after_expansion's loop (generous headroom, not tuned).
    MAX_EXPANSION_REPAIR_PASSES = 8

    def repair_path_after_expansion(self) -> int:
        """
        [Call right after increase_radius] Re-check the route against the expanded mines.

        Why:
        - expandField keeps obstacle-to-obstacle edges consistent, but never revisits
          floating-to-floating edges (start/end/helper). One that was clear can now
          cross a grown mine.

        Does:
        - Runs check_path_envelopment over every obstacle, repeating full passes until one
          finds nothing (a repair recompute can itself pick a new bad edge).
        - Capped at MAX_EXPANSION_REPAIR_PASSES.

        Returns: number of passes run (1 = first pass was clean).
        Notes: an ASSISTANT has no Pathfinder and never calls this.
        """
        for pass_num in range(1, self.MAX_EXPANSION_REPAIR_PASSES + 1):
            changed = False
            for obstacle in list(self.nodeField.mines) + list(self.nodeField.unionObstacles):
                if self.check_path_envelopment(obstacle):
                    changed = True
            if not changed:
                return pass_num
        return self.MAX_EXPANSION_REPAIR_PASSES

    def _dijkstra_path_with_hysteresis(self, start_nodes: list, hysteresis_tolerance: float):
        """
        Dijkstra from start_nodes to the best ending node.

        Takes:
        - start_nodes: nodes to search from.
        - hysteresis_tolerance: keep self.previousEndNode if its route is within this
          fraction of the best (stops flapping between near-tied ending nodes).

        Returns: node list start -> end, or [] if unreachable.
        Touches: writes self.previousEndNode.

        Fallback: if self.endingNodes is unreachable from every start node, use
        self._original_ending_nodes before giving up. A single retargeted node can vanish
        (e.g. a mine merge covers it), and returning [] would wipe good A/B paths. Done
        here so every caller gets it.
        """
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
        """
        Plan photo waypoints along get_shortest_path.

        Takes:
        - method: "path" (walk the path geometry) or "cellgrid" (rasterize + greedy cover).
        - overlap, path_width: spacing margin / corridor width (method="path" only).
        - shape_size_ft: footprint override (number or (along, across)); default from matSize.

        Does: places footprints over the unseen cells, orders them, verifies coverage
        (raises on a gap), converts to lat/lon.
        """
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
        """
        Shared engine for getPlacesToCheck and the maze places-to-check methods.

        Takes:
        - path_nodes: explicit node path (not get_shortest_path()).
        - fixed_first: TSP start anchor.
        - next_place_attr: attribute name to write the new anchor back to, so each queue
          (e.g. maze A vs B) tracks its own.
        """
        matSizeCells = max(1, round(self.matSize / WIDTHOFSQUARE))

        # Whole path, never a per-drone slice (drones split the field by launch edge/pairing).
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
            # Only what's still unseen ("cellgrid" targets the whole path).
            need_field = whole_field & ~self.seen_tracker
        elif method == "cellgrid":
            if shape_size_ft is not None:
                raise ValueError("shape_size_ft override is only supported for method='path'")
            # path_tracker == whole_field (just rebuilt above).
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
            # Already real-world feet; don't scale by WIDTHOFSQUARE again.
            lat, lon = self.coord_converter.local_to_latlon(i[0], i[1])
            latLonPoints.append((lat, lon))
        return latLonPoints

    # ==================================================================
    # MAZE-STYLE incremental replanning.
    #
    # Segments:
    #   C (confirmed): start..point_C. Frozen.
    #   B (bridge):    point_C..point_A. Reroutable.
    #   A (approach):  point_A..true end (final edge). point_A is the fixed anchor.
    #
    # Rules:
    #   Rule 1 (mine in A, or first pass): recompute from point_C, split at the new
    #          path's final edge (new A); the rest becomes B.
    #   Rule 2 (mine in B): reroute B between the same point_C/point_A.
    #   B drains clean -> confirm it into C, go back to checking A.
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
        """Returns the closest node with at least one live edge to (x,y).
        Re-resolves stale references (a merge can replace a node with a new object at
        the same point). Skips 0-edge orphans."""
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

    def _resolve_starting_nodes(self) -> list:
        """
        self.startingNodes re-resolved to live graph nodes (see _resolve_node_near);
        drops any that no longer resolve.
        - Usually a no-op. But an expanded mine can swallow the entry node and get it
          removed from the graph while self.startingNodes still lists it.
        - Without this, Dijkstra would KeyError on a start node that isn't in the graph.
        - Mirror of the _original_ending_nodes fallback, on the start side.
        """
        resolved = []
        for node in self.startingNodes:
            live = self._resolve_node_near(node.x, node.y)
            if live is not None:
                resolved.append(live)
        return resolved

    def _maze_c_start_nodes(self) -> list:
        """Returns: [node] to search FROM for a Rule-1 recompute: where
        maze_confirmed_path (point_C) ends (re-resolved to a live node), or the
        re-resolved startingNodes if nothing is confirmed yet."""
        if not self.maze_confirmed_path:
            return self._resolve_starting_nodes()
        last = self.maze_confirmed_path[-1]
        node = self._resolve_node_near(last.x, last.y)
        return [node] if node is not None else self._resolve_starting_nodes()

    def _prepend_seam_if_needed(self, path_nodes: list) -> list:
        """
        Make sure path_nodes starts exactly at point_C's end (the "seam").

        Takes: path_nodes, whose first node may be a re-resolved substitute for point_C.
        Returns: path_nodes unchanged if it already starts at point_C's end, else
        route[:-1] + path_nodes, where route = _shortest_path_between(point_C end, path_nodes[0]).

        Why a graph route, not a straight line:
        - Graph edges never cross KNOWN obstacles; an invented [seam, first] edge was never
          checked (seen: a ~38ft seam cutting through an already-known mine).
        - check_path_envelopment only re-checks confirmed edges against FUTURE discoveries.
        - Falls back to the straight line only if the graph has no route at all (rare).
        """
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
        """Reset maze state. The first pass treats the whole shortest path as segment A."""
        self.maze_a_path = self.get_shortest_path()
        self.maze_b_path = []
        self.maze_confirmed_path = []
        self.helper_node_trail = []
        self.promoted_helper_nodes = []
        self.remote_mine_placeholders = []
        self.cross_pair_patches = []
        # Nodes of the midline seam run, once join_cross_pair_seam has closed it.
        self.cross_pair_seam_span = []
        # True once replan_toward asked the gambler to fly segment A from its far end.
        self.approach_from_far_end = False
        self.last_patched_span = None
        self.next_place_to_check_maze_a = None
        self.next_place_to_check_maze_b = None

    def record_helper_node_candidate(self, x: float, y: float) -> None:
        """
        [HELPER-NODE] Record a photographed (x,y) from segment A as a breadcrumb.

        Does:
        - Appends to helper_node_trail, kept sorted by geometric distance from
          maze_a_path[0] (not visit order: the TSP tour doesn't walk the path in order).
        - Skips cells out of bounds or inside a known obstacle.
        - Trims entries more than 2x the safety margin behind the furthest one (a rolling
          relevance bound; the real safety check is start_helper_node_detour).
        """
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

        trim_window = 2 * (self.mine_safety_radius_ft * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE)
        furthest_along = self.helper_node_trail[-1][0]
        self.helper_node_trail = [
            e for e in self.helper_node_trail if furthest_along - e[0] <= trim_window
        ]

    def _prune_promoted_helper_nodes(self) -> None:
        """
        Drop promoted_helper_nodes entries whose node is no longer in C, B, or A.

        When: after ANY of the three paths is replaced (reroute_b_segment*,
        on_forward_mine_discovered, check_path_envelopment).
        Why: a stale entry no longer matches the real bridge, and a later
        check_merge_rewind repair built on it could splice in a stale or unsafe connection.
        """
        self.promoted_helper_nodes = [
            entry
            for entry in self.promoted_helper_nodes
            if any(
                n is entry["node"]
                for n in self.maze_confirmed_path + self.maze_b_path + self.maze_a_path
            )
        ]

    def _settle_confirmed_helper_nodes(self) -> None:
        """
        Drop promoted_helper_nodes entries whose node is now in maze_confirmed_path.

        Why: a confirmed detour is settled, not outstanding, so it must stop counting
        toward start_helper_node_detour's outstanding-detour cap (else the first detour
        would block every later one).
        When: after confirm_b_into_c / advance_b_prefix_into_c.
        Note: opposite keep-rule from _prune_promoted_helper_nodes (which keeps confirmed).
        """
        self.promoted_helper_nodes = [
            entry
            for entry in self.promoted_helper_nodes
            if not any(n is entry["node"] for n in self.maze_confirmed_path)
        ]

    # ==================================================================
    # CROSS-PAIR: two pairs on opposite ends sharing discoveries.
    # - Mine relay itself lives in the simulation driver (add_discovered_mine runs for
    #   every mine, whoever found it).
    # - These methods are what the RECEIVING pair does with a mine it didn't find:
    #   record_remote_mine_on_segment_a (A, not yet flown) and patch_confirmed_span
    #   (confirmed history).
    # - A remote mine in the ACTIVE segment B needs neither: check_merge_rewind /
    #   check_path_envelopment already scan maze_b_path.
    # ==================================================================

    def record_remote_mine_on_segment_a(self, new_mine) -> None:
        """
        Call on the pair that did NOT find new_mine (after add_discovered_mine) if it
        crosses this pair's segment A line.

        Why: segment A isn't flown yet, and add_discovered_mine's checks only look at
        nodes/edges already in A/B/C, so they miss it.

        Does:
        - Computes where a real breadcrumb would have landed (same safety margin as
          start_helper_node_detour) and saves it as a placeholder. No graph changes yet.
        - If the gambler reaches that spot (_check_remote_placeholder_reached), it becomes
          a real detour, like a local find.
        - If something else gets found first, the placeholder is dropped; the mine is
          in the graph anyway.
        """
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
        safety_margin = self.mine_safety_radius_ft * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE
        dist_to_mine = math.hypot(mine_x - p0.x, mine_y - p0.y)
        place_at = max(0.0, dist_to_mine - safety_margin)
        ux = (mine_x - p0.x) / max(dist_to_mine, 1e-9)
        uy = (mine_y - p0.y) / max(dist_to_mine, 1e-9)
        self.remote_mine_placeholders.append({
            "position": (p0.x + ux * place_at, p0.y + uy * place_at),
            "obstacle": new_mine,
        })

    def _check_remote_placeholder_reached(self, x: float, y: float):
        """
        Call after each photo in the gambler's segment-A loop.

        Does: if (x,y) reached or passed a pending placeholder along segment A, adds it to
        helper_node_trail as if a real photo landed there.
        Returns: the placeholder's obstacle (treat like a fresh local mine: call
        start_helper_node_detour with it), or None if nothing was reached.
        """
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
        """
        [Rule 1] Call when a mine is found while checking segment A.

        Does:
        - Folds any fully-photographed prefix of B into C first, then recomputes from
          point_C (or the true start). Last two nodes become the new A, the rest B.

        Why fold first:
        - The recompute starts at point_C, so anything the assistant has already
          photographed but not yet had folded in is thrown away and re-planned against a
          different route. Since B is replaced wholesale here, the assistant ends up
          chasing a moving target: measured at 1.83x the photos of a solo drone covering
          the same field, with the assistant flying 1857ft vs the solo's 1175ft total.
          Folding first makes the recompute start from the furthest point actually
          verified, so completed work survives the replan.
        """
        self.advance_b_prefix_into_c()

        # The mine is on segment A, which is FORWARD of B -- so B itself is still a valid
        # route. Try replanning only from B's far end, which keeps the assistant's whole
        # queue intact. Falls through to the full recompute below if no route exists from
        # there (the new mine can wall off B's end), so this can only preserve work, never
        # strand it.
        if len(self.maze_b_path) >= 2:
            forward = self._dijkstra_path_with_hysteresis(
                [self.maze_b_path[-1]], PATH_HYSTERESIS_TOLERANCE
            )
            if len(forward) >= 2:
                full = self.maze_b_path + forward[1:]
                self._set_segments(full)
                self._prune_promoted_helper_nodes()
                self.remote_mine_placeholders = []
                return

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
        self._set_segments(recomputed)
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []

    def _set_segments(self, route: list) -> None:
        """Split `route` into segments B and A.

        Default: A is the final edge (the frontier) and B is everything behind it.
        Under approach_from_far_end, the remaining unphotographed work is halved instead,
        so the gambler keeps a real share and the two roles keep closing the gap from both
        ends. Without this the split laid down by replan_toward decays on the very next
        discovery -- measured: it handed the assistant MORE work (+40% distance), which is
        the opposite of the intent, because the assistant was already the bottleneck.
        """
        if len(route) < 2:
            self.maze_a_path = route
            self.maze_b_path = []
            return
        if self.approach_from_far_end and len(route) >= 3:
            idx = self._split_index_by_unseen(route)
            self.maze_b_path = route[: idx + 1]
            self.maze_a_path = route[idx:]
            return
        self.maze_a_path = route[-2:]
        self.maze_b_path = route[:-1]

    def _bridge_up_to(self, anchor_node) -> list:
        """
        The established route from point_C up through anchor_node (inclusive).

        Returns:
        - maze_b_path up to anchor_node, if it's there (anchor is mid-bridge: a chained
          detour/repair).
        - else [anchor_node] with the seam guard (anchor is effectively point_C's end).

        Used by start_helper_node_detour, check_merge_rewind, _check_forward_of_helper_node
        so splicing a new stretch keeps the real chain instead of a straight, unchecked jump.

        Does NOT search maze_confirmed_path: nodes get re-resolved to the same object at
        unrelated times, so an identity match there is unreliable. An earlier version did
        and discarded confirmed history (133/200 seeds got coverage gaps). Anchors already
        confirmed fall through to the seam guard instead: less precise, but not destructive.
        """
        for i, node in enumerate(self.maze_b_path):
            if node is anchor_node:
                return self.maze_b_path[: i + 1]
        return self._prepend_seam_if_needed([anchor_node])

    def start_helper_node_detour(self, new_mine) -> bool:
        """
        [HELPER-NODE alternative to Rule 1] Detour around new_mine via a breadcrumb node.

        Takes: new_mine, already processed by add_discovered_mine (callers skip this
        when check_merge_rewind returned rewound=True).

        Does:
        - Picks the helper_node_trail breadcrumb behind new_mine with the smallest margin
          that is still >= the safety margin.
        - Promotes it to a graph Node, connects it to new_mine's tangents and to point_A
          as it was, and re-splits like on_forward_mine_discovered (seeded at the new node).
        - Records the promotion in promoted_helper_nodes.

        Chaining:
        - New B is spliced onto the existing bridge via _bridge_up_to, so a second or
          third mine along A (before the assistant confirms the first) keeps the real chain
          instead of a straight, unchecked jump from point_C.
        - Capped at MAX_CHAIN_DEPTH outstanding entries; at the cap it falls back to
          on_forward_mine_discovered (waits for the assistant). Rarely reached.
        - An earlier link already confirmed into C is handled in check_merge_rewind
          (defers to check_path_envelopment), not here.

        Returns: True if a detour was built. False if the trail had nothing usable, the
        chain is at MAX_CHAIN_DEPTH, or maze_a_path is empty (a concurrent discovery can
        have invalidated it). On False, the caller falls back to on_forward_mine_discovered.
        """
        if len(self.promoted_helper_nodes) >= MAX_CHAIN_DEPTH:
            return False
        if not self.maze_a_path:
            return False
        previous_point_a = self.maze_a_path[0]
        # Measure from the mine's CENTER (the polygon boundary already includes the safety radius).
        mine_x, mine_y = getattr(new_mine, "origin", None) or (
            new_mine.polygon.centroid.x, new_mine.polygon.centroid.y
        )
        mine_center_distance = math.hypot(mine_x - previous_point_a.x, mine_y - previous_point_a.y)
        safety_margin = self.mine_safety_radius_ft * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE

        # Also skip points inside the mine: the visit order can photograph past it first.
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
            # Isolated: every tangent and the direct link cross other obstacles.
            # Graph.shortest_distances KeyErrors on a disconnected source, so give up.
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
            # The direct link was rejected (it crosses an obstacle) but helper_node
            # connected elsewhere. Don't just concatenate: that would put previous_point_a
            # and helper_node side by side with NO real edge, a phantom edge that could get
            # frozen into confirmed history (seen as a real bad edge in a two-pair sweep).
            # Find the real, possibly indirect, route instead.
            link = self._shortest_path_between(previous_point_a, helper_node)
            if link is None:
                # Disconnected from the bridge: same as the isolated case above.
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

    def repair_via_span_handoff(self, new_mine) -> bool:
        """
        [HELPER-NODE, alternative to start_helper_node_detour] Hand the broken span to the
        assistant as one fresh sub-route instead of splicing a chained detour.

        Does:
        - Picks the furthest-along breadcrumb that is clear of new_mine by the safety
          margin, outside every known obstacle, and ALREADY PHOTOGRAPHED (accepted) --
          so the ground behind it is verified and nothing has to be spliced into frozen
          history.
        - Recomputes one clean route from that anchor to the current target, bridges to it
          from point_C with the usual graph-route seam guard, and re-splits. The span lands
          in segment B, i.e. the assistant verifies it, acting as a solo gambler over it.

        Why:
        - It does NOT promote a chained helper node. check_merge_rewind and
          _check_forward_of_helper_node only ever trigger on promoted_helper_nodes entries,
          so leaving that list empty makes the whole rewind / helper-repair path
          unreachable -- and that machinery is where most of this file's known defects
          live. check_path_envelopment still runs, so the safety catch-all is unchanged.

        Returns: True if a span was handed off, False if no accepted breadcrumb is usable
        or no route exists from it (caller falls back to on_forward_mine_discovered).
        """
        if not self.maze_a_path:
            return False
        obstacles = list(self.nodeField.mines) + list(self.nodeField.unionObstacles)
        margin = self.mine_safety_radius_ft * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE
        mine_xy = getattr(new_mine, "origin", None) or (
            new_mine.polygon.centroid.x, new_mine.polygon.centroid.y
        )

        anchor_xy = None
        for _distance, cx, cy in reversed(self.helper_node_trail):
            if math.hypot(cx - mine_xy[0], cy - mine_xy[1]) < margin:
                continue
            if any(o.contains_point((cx, cy)) for o in obstacles):
                continue
            col, row = self.seen_tracker.real_to_cell(cx, cy)
            if not self.seen_tracker.get(col, row):
                continue  # not photographed yet -- not an accepted anchor
            anchor_xy = (cx, cy)
            break
        if anchor_xy is None:
            return False

        anchor = self.nodeField.addFloatingNode(*anchor_xy)
        span = self._dijkstra_path_with_hysteresis([anchor], PATH_HYSTERESIS_TOLERANCE)
        if len(span) < 2:
            self.nodeField.fieldConnection.removeNode(anchor)
            return False

        # Connect the anchor back to point_C explicitly. _bridge_up_to falls through to
        # _prepend_seam_if_needed, which returns its input UNCHANGED when confirmed history
        # is empty -- that left the route starting at the anchor mid-field with nothing
        # joining it to the entry, i.e. a shorter path that simply skipped the job. It read
        # as a 58% improvement until the continuity guard caught it.
        start_nodes = self._maze_c_start_nodes()
        prefix = self._shortest_path_between(start_nodes[0], anchor) if start_nodes else None
        if prefix is None or len(prefix) < 1:
            self.nodeField.fieldConnection.removeNode(anchor)
            return False
        full = prefix[:-1] + span
        self._set_segments(full)
        self.helper_node_trail = []
        self.remote_mine_placeholders = []
        return True

    def check_merge_rewind(self, new_mine, was_merged: bool) -> bool:
        """
        [HELPER-NODE; no-op if promoted_helper_nodes is empty]
        Check whether new_mine invalidates an earlier helper-node detour, and repair.

        Takes: new_mine, was_merged (from add_discovered_mine).

        Does:
        - Scans promoted_helper_nodes newest-first (least disruptive match wins) for a hit:
          - identity: was_merged and the entry's obstacle .mergedInto chain resolves to
            new_mine.
          - position: the entry's helper node is inside new_mine's polygon.
          - segment: new_mine intersects helper node -> previous_point_a (can be found while
            checking that edge without covering an endpoint).
        - On a hit, tries a local repair (_repair_helper_node_along_segment): reuse the node
          if still safe, else reconnect further back along the same segment.
        - If that fails, walks back through EARLIER entries for a safe previous_point_a and
          discards everything after (Field.removeNode on each dropped node).
        - Then truncates maze_confirmed_path to the reseed point if needed and reruns the
          hysteresis recompute + seam guard.
        - If no backward trigger hits, tries _check_forward_of_helper_node (a mine AHEAD of
          a helper node, toward its target mine's tangent).

        Skips entries already folded into maze_confirmed_path (normal once the gambler chains
        ahead of a slower assistant). Those defer to check_path_envelopment, the only thing
        that can tell how much confirmed history is still valid; splicing here risked
        discarding history or inserting an unchecked jump.

        Returns: True if a rewind was performed (routing already resolved), else False.
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
            # Already confirmed: leave it to check_path_envelopment (runs right after)
            # instead of splicing into frozen history. Checked after hit_index is chosen
            # so it can't change which entries the stale cleanup below removes.
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
            # prefix_path[0] is previous_point_a: splice onto the real bridge up to it
            # (see _bridge_up_to), dropping the duplicate.
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

        # Local repair impossible (position unsafe, or nothing connects it to
        # previous_point_a): walk back through EARLIER entries for a safe anchor.
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

        # reseed_node can be mid-chain; _bridge_up_to keeps the real route back to it.
        prior_bridge = self._bridge_up_to(reseed_node)

        truncate_at = next(
            (i for i, n in enumerate(self.maze_confirmed_path) if n is reseed_node), None
        )
        if truncate_at is not None:
            self.maze_confirmed_path = self.maze_confirmed_path[: truncate_at + 1]

        self.helper_node_trail = []

        recomputed = self._dijkstra_path_with_hysteresis([reseed_node], PATH_HYSTERESIS_TOLERANCE)
        if len(recomputed) < 2:
            # No route forward from the anchor -- typically because the merge invalidated
            # every connection it had (in the graph, zero neighbours). Assigning the empty
            # result here used to wipe A and B, and with C empty the pair was left with no
            # route at all while a valid one from the start still existed (seen: seeds 78,
            # 109). A repair must never leave less than a route that exists: fall back to
            # the same full recompute from point_C that Rule 1 uses.
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
        else:
            full = prior_bridge[:-1] + recomputed
            self.maze_a_path = full[-2:]
            self.maze_b_path = full[:-1]
        self._prune_promoted_helper_nodes()
        self.remote_mine_placeholders = []
        return True

    def _check_forward_of_helper_node(self, new_mine) -> bool:
        """
        [Called by check_merge_rewind when no backward trigger matched]
        A helper node's FORWARD connection (toward its target mine's tangents) can also be
        crossed by a new mine.

        Does:
        - Finds the newest promoted helper node whose forward connection new_mine crosses.
        - Recomputes point A from that node: truncates maze_confirmed_path to just after it
          (keeping it), drops later entries, reseeds the recompute at the node.

        Returns: True if performed, False if nothing is affected.
        """
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
                # node can be mid-chain; _bridge_up_to keeps the real route back to it
                # (drop the duplicate: recomputed[0] is node too).
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

    def _repair_helper_node_along_segment(self, old_node, previous_point_a, new_mine):
        """
        [Called by check_merge_rewind] Repair a promoted helper node after its target mine merged.

        Takes:
        - old_node: the promoted helper node.
        - previous_point_a: the anchor it was originally wired to.
        - new_mine: the current (possibly bigger) obstacle to reconnect against.

        Does:
        - If old_node is still safe (same margin as start_helper_node_detour, clear of all
          other obstacles): refresh its connections like a brand new floating node, then
          take the real shortest path previous_point_a -> old_node (routes around obstacles).
        - Gives up if the position is unsafe or no path exists. Mines found LATER are handled
          by Field._cleanupInvalidatedConnections and check_path_envelopment.

        Returns: (prefix_path, tracked_node). prefix_path = full node list previous_point_a ->
        old_node, ready to splice into maze_b_path, or None if repair isn't possible.
        tracked_node is old_node on success.
        """
        mine_x, mine_y = getattr(new_mine, "origin", None) or (
            new_mine.polygon.centroid.x, new_mine.polygon.centroid.y
        )
        safety_margin = self.mine_safety_radius_ft * WIDTHOFSQUARE + 2 * WIDTHOFSQUARE
        other_obstacles = [
            o
            for o in list(self.nodeField.mines) + list(self.nodeField.unionObstacles)
            if o is not new_mine
        ]

        def point_safe(x, y):
            if math.hypot(x - mine_x, y - mine_y) < safety_margin or new_mine.contains_point((x, y)):
                return False
            return not any(o.contains_point((x, y)) for o in other_obstacles)  # margin applies to new_mine only

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

    def check_path_envelopment(self, new_mine) -> bool:
        """
        [Call after EVERY add_discovered_mine, for every maze variant] General catch-all:
        repair the path if new_mine invalidated it.

        Scans (in path order: confirmed C, then B, then A):
        - NODES: the first one now inside new_mine. B/A included, since an unconfirmed node
          can be covered by a merge and later folded into C unchecked.
        - EDGES: any consecutive edge new_mine crosses without containing an endpoint.
          - Covers what check_merge_rewind's segment_hit can't (edges not anchored by a
            helper node); check_merge_rewind defers here for already-confirmed entries.
          - The A scan matters for a fresh cross-pair retarget hop (bad edge seen in the
            two-pair sweep). The B scan matters for wide Dijkstra results (patch_confirmed_span,
            reroute_b_segment) that use tangent-to-tangent edges between unrelated mines.

        On a hit:
        - NODE hit: strip its connections so a recompute can't pick it again.
          purgeConnections for a mine tangent node (obstacle keeps the node for expandField),
          else removeNode (floating/helper node).
        - EDGE hit: no stripping needed; Field._cleanupInvalidatedConnections already removed it.
        - Truncate maze_confirmed_path to just before/at the hit, re-derive A/B with the
          standard recompute + seam guard, prune promoted_helper_nodes.

        Returns: True if a hit was repaired, False if nothing was invalidated.
        (repair_path_after_expansion sweeps this over every obstacle until it reads False.)
        """
        # Scan confirmed C index by index, checking the node and its outgoing edge together,
        # so whichever problem comes FIRST in path order sets the truncation. (Scanning all
        # nodes first can find a coincidental late hit and leave an earlier crossed edge in
        # "confirmed" history; seen as a bad edge in the round-robin sweep.)
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
            # maze_a_path edges: same crossing-without-an-endpoint case as B, one step out.
            # (Missed before, letting a fresh retarget hop go unvalidated.) edge_hit_index
            # is never used for confirmed truncation here (edge_hit_in_confirmed is already
            # False), so reusing it is safe.
            for i in range(len(self.maze_a_path) - 1):
                p1, p2 = self.maze_a_path[i], self.maze_a_path[i + 1]
                if new_mine.intersects(((p1.x, p1.y), (p2.x, p2.y))):
                    edge_hit_index = i
                    break
        if hit is None and edge_hit_index is None:
            return False

        if edge_hit_index is not None:
            if edge_hit_in_confirmed:
                self.maze_confirmed_path = self.maze_confirmed_path[: edge_hit_index + 1]
            # else: hit is in B/A; the recompute below replaces both anyway.
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
        return True

    def patch_confirmed_span(self, new_mine):
        """
        [CROSS-PAIR] Patch confirmed history around a mine relayed from the OTHER pair.
        Called via add_discovered_mine(prefer_local_patch=True), before check_path_envelopment.

        Does:
        - Runs the same node-then-edge scan of maze_confirmed_path as check_path_envelopment.
        - On a hit, pins the two boundary nodes around it and splices in a fresh LOCAL
          shortest path between just those two.
        - Why local: this pair has probably moved well past that point, and a full recompute
          to the end would discard that progress.

        Returns:
        - The new sub-path's node list (both boundary nodes included) if patched. The caller
          hands it to the DISCOVERING pair to verify/photograph (it is the one physically near).
        - None if nothing hit, the boundary is at either end of maze_confirmed_path, or no
          local path exists. check_path_envelopment (called right after) then handles it with
          a safe full recompute.
        """
        # Scan index by index (node, then its outgoing edge) so the FIRST problem in path
        # order wins (same fix as check_path_envelopment). check_path_envelopment still
        # catches anything missed, but a wrong patch would force its costlier full recompute.
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

    def get_cross_pair_patch_places_to_check(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ) -> list:
        """
        [CROSS-PAIR] Places-to-check for the oldest entry in cross_pair_patches.
        - That is a confirmed-history patch this pair's assistant owes a verification
          flight (patch_confirmed_span ran on the OTHER pair for a mine THIS pair found).
        - The graph fix is already done; this is coverage only, keeping the "everything
          got photographed" invariant.
        - Caller pops cross_pair_patches[0] once its footprint is fully seen (like B).
        """
        if not self.cross_pair_patches:
            return []
        return self._places_to_check_for_path(
            self.cross_pair_patches[0], method=method, overlap=overlap,
            path_width=path_width, shape_size_ft=shape_size_ft,
        )

    def reroute_b_segment(self) -> None:
        """
        [Rule 2] Re-search point_C -> point_A (fixed) on the current graph and replace
        maze_b_path.
        - A mine in B only makes the route more roundabout; neither endpoint moves.
        - Both ends get the same seam guard as on_forward_mine_discovered.
        """
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
        """Returns the obstacle (BlockMine/union) whose .nodes contains node, or None.
        (Tangent nodes have no parentMine back-reference.)"""
        for obstacle in list(self.nodeField.mines) + list(self.nodeField.unionObstacles):
            if node in obstacle.nodes:
                return obstacle
        return None

    def reroute_b_segment_same_mine(self) -> None:
        """
        [Rule 2 variant] Recompute from point_C like Rule 1, but adopt the new point_A only
        if its final-edge node is on the SAME mine point_A is pinned to (may slide to
        another tangent of it, never jump to a different mine).
        - Otherwise falls back to reroute_b_segment (point_A fixed).
        """
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
        """
        Does: appends new_content onto maze_confirmed_path.
        - Drops the duplicate leading node when new_content[0] is the same point as
          maze_confirmed_path[-1].
        - If new_content revisits a node already in maze_confirmed_path (a real loop: nodes
          are created once, so identity match = genuine backtrack), first truncates back to
          just before that node's first occurrence so it appears once.
        - Can happen after check_path_envelopment/patch_confirmed_span remove an early node
          and the only remaining route doubles back.
        """
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
        """
        Does: appends maze_b_path onto maze_confirmed_path (_extend_confirmed_path), clears B.
        - Call once get_places_to_check_maze()'s "b" list drains. Idempotent on empty B.
        """
        if not self.maze_b_path:
            return
        self._extend_confirmed_path(self.maze_b_path)
        self.maze_b_path = []
        self._settle_confirmed_helper_nodes()
        # Folding may free a cross_pair_target_chain slot; retry a queued retarget.

    def advance_b_prefix_into_c(self) -> None:
        """
        Does: folds every leading edge of maze_b_path whose footprint is fully seen into
        maze_confirmed_path (via _extend_confirmed_path), not just B once it fully drains.
        - Shrinks what a later reroute has to re-search.
        """
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

    def get_maze_path(self):
        """
        Returns: maze_confirmed_path + maze_b_path + maze_a_path, dropping a shared
        boundary node only when it is the same point (see confirm_b_into_c).
        - The route committed to and checked so far; verify coverage against this,
          not get_shortest_path.
        """

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

    def get_iarc_path(self, buffer_width: int = 0) -> str:
        """
        Does: get_maze_path() in the IARC text-reporting format (this Pathfinder only).
        - Feeds local (x, y) straight to format_iarc_path (no lat/lon round trip; that
          loses precision at cell boundaries).
        - Mission-wide report across pairs: Drone.get_mission_iarc_path.

        Raises: ValueError if get_maze_path() is empty or entirely outside the field.
        """
        path_xy = [(n.x, n.y) for n in self.get_maze_path()]
        return format_iarc_path(path_xy, buffer_width)

    def get_places_to_check_maze(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ) -> dict:
        """
        Takes: same coverage params as getPlacesToCheck.

        Does:
        - Computes places-to-check separately for maze_a_path and maze_b_path.
        - B: TSP order anchored to next_place_to_check_maze_b, so a replan does not
          restart the tour from the opposite side.
        - A: anchored directly to maze_a_path[0] (point_A), not a remembered value.
          - point_A is always A's near boundary; a remembered anchor goes stale after a
            retarget or Rule 1, and order_waypoints would then start the tour at the far
            end (seen: tour walked back toward its own point_A).

        Returns: {"a": [(lat,lon),...], "b": [...], "count": total}.
        """
        a_anchor = None
        if self.maze_a_path:
            # Normally the near boundary (point_A). After replan_toward(backwards=True) the
            # gambler works A from its FAR end back toward the assistant, so the tour is
            # anchored there instead and the two roles close the gap from both sides.
            a0 = self.maze_a_path[-1] if self.approach_from_far_end else self.maze_a_path[0]
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
