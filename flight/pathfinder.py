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
from flight.pathfinding.path_calculation import Graph
import flight.pathfinding.utils.seen_by_drone as seen_by_drone
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


def _two_opt(coords: list[tuple[float, float]], tour: list[int], max_passes: int = 200) -> list[int]:
    # Classic 2-opt for an OPEN path (no closing edge back to the start):
    # for each pair of edges (tour[i],tour[i+1]) and (tour[j],tour[j+1]),
    # check whether reversing the segment between them shortens the path
    # (tour[j+1] may not exist if j is the last index -- that edge just
    # isn't part of the swap's cost in that case). Repeats until a full
    # pass finds no improving swap, or max_passes is hit as a safety cap --
    # 2-opt is monotonically improving so it always terminates on a finite
    # point set, but the cap guards against float-precision edge cases
    # that could in principle stall convergence.
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
                    tour[i + 1:j + 1] = reversed(tour[i + 1:j + 1])
                    improved = True
                    break
            if improved:
                break
        if not improved:
            break
    return tour


def _find_matching_index(coords: list[tuple[float, float]], point: tuple[float, float], tol: float = 1e-6) -> int | None:
    for i, c in enumerate(coords):
        if abs(c[0] - point[0]) <= tol and abs(c[1] - point[1]) <= tol:
            return i
    return None


def order_waypoints(
    coords: list[tuple[float, float]], fixed_first: tuple[float, float] = None
) -> list[tuple[float, float]]:
    """
    Given a list of local (x,y) coordinates, returns them reordered into a
    decent (not optimal -- exact TSP is NP-hard) visiting order: nearest-
    neighbor construction, tried from several candidate starting points and
    keeping whichever gives the shortest initial tour, then 2-opt local
    search (reverse a sub-segment whenever doing so shortens the total
    path) until no single such swap helps anymore. Treated as an open path
    with no forced return to the start, matching a one-way flight plan
    rather than a round trip.

    `fixed_first`, if given, pins the returned order to always start there
    instead of letting the heuristic pick whichever start minimizes total
    tour length -- e.g. so replanning after a new discovery doesn't yank
    the drone toward a totally different first stop than the one it's
    already committed to (see Pathfinder.getPlacesToCheck). It's matched
    (within a small tolerance, not exact equality -- coordinates recomputed
    on a replan won't bit-for-bit match an earlier call's) against `coords`
    if present there; if not, it's treated as an external anchor (e.g. the
    drone's current position) that the tour departs from but that isn't
    itself one of the places to check, so it's excluded from the return
    value. 2-opt never moves index 0 of an open-path tour (every swap it
    considers reverses a sub-segment starting at index >= 1), so pinning
    the start doesn't just seed construction -- it survives local-search
    optimization of everything after it too.
    """
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
    """
    Total Euclidean length of the polyline through `coords` (local (x,y)
    coordinates, in visiting order) -- the straight-line distance between
    each consecutive pair, summed. 0.0 for fewer than two points.
    """
    return sum(_dist(coords[i], coords[i + 1]) for i in range(len(coords) - 1))


def _footprint_box(cx: float, cy: float, along_ft: float, across_ft: float) -> tuple[float, float, float, float]:
    """The axis-aligned (x0, y0, x1, y1) rectangle a photo taken at (cx, cy)
    would actually cover -- `along` (parallel to the direction of travel)
    maps to y, `across` (perpendicular to it) maps to x, matching the same
    axis convention used everywhere else a footprint rectangle gets built
    in this codebase (see droneWorkflowTest.simulate_one_drone) -- this
    field's paths run essentially north-south throughout, so this
    axis-aligned rectangle doesn't need to rotate to the path's local
    heading."""
    half_along, half_across = along_ft / 2.0, across_ft / 2.0
    return cx - half_across, cy - half_along, cx + half_across, cy + half_along


def _footprint_corners(cx: float, cy: float, along_ft: float, across_ft: float) -> list[tuple[float, float]]:
    """Same rectangle as _footprint_box, as an explicit corner-point
    polygon -- for callers (e.g. accept_image_corner_coord-style code) that
    need actual corner coordinates rather than a box."""
    x0, y0, x1, y1 = _footprint_box(cx, cy, along_ft, across_ft)
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def node_path_length(nodePath) -> float:
    """
    Total Euclidean length of the path through `nodePath` -- a list of
    objects exposing `.x`/`.y` (e.g. nodeField Node instances), in visiting
    order. Same measure as path_length, just reading positions directly off
    each node instead of requiring the caller to convert to (x,y) tuples
    first (matching the conversion get_cell_path already does inline).
    """
    return path_length([(n.x, n.y) for n in nodePath])


WIDTHOFSQUARE=2
WIDTHOFFIELD=80
HEIGHTOFFIELD=300
class Pathfinder:
    def __init__(
        self, real_corner_coords: tuple[tuple[float, float]], altitude: float, fov_deg: float, droneID:int, numOfDrones:int
    ):
        self.droneID=droneID
        self.numOfDrones=numOfDrones
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

        # arb_corner_coords is in SimToLatLonTransformer's own corner order
        # ([origin/bottom-left, +x/bottom-right, diagonal/top-right,
        # +y/top-left]) -- but Field's fieldVertPairLeft/Right/HorzPairUpper/
        # Lower (and so withinField, which gates every non-floating-to-
        # non-floating connection) are built assuming [top-left, top-right,
        # bottom-left, bottom-right] row-major order instead (matching how
        # every hand-built Field in this repo's own tests passes corners --
        # see e.g. connectPolygonVisualCheck.py). Passing arb_corner_coords
        # directly makes withinField reject every interior point (verified:
        # every point in an 80x300 field came back False), which silently
        # drops every mine-to-mine tangent connection field-wide even though
        # floating-node connections (which skip the withinField gate
        # entirely) still work -- reorder to the convention Field actually
        # implements rather than the one its docstring describes.
        field_corners = [
            self.arb_corner_coords[3],  # +y / top-left
            self.arb_corner_coords[2],  # diagonal / top-right
            self.arb_corner_coords[0],  # origin / bottom-left
            self.arb_corner_coords[1],  # +x / bottom-right
        ]
        self.nodeField = Field(self.arb_field_size, field_corners, droneNumber=self.droneID)
        self.mineFieldTracker = CellField(
            WIDTHOFFIELD // WIDTHOFSQUARE, HEIGHTOFFIELD // WIDTHOFSQUARE,
            max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD),
        )

        self.seen_tracker = CellField(
            WIDTHOFFIELD // WIDTHOFSQUARE, HEIGHTOFFIELD // WIDTHOFSQUARE,
            max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD),
        )  # This is a placeholder, replace with the actual SightTracker object

        self.path_tracker = CellField(
            WIDTHOFFIELD // WIDTHOFSQUARE, HEIGHTOFFIELD // WIDTHOFSQUARE,
            max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD),
        )  # This is a placeholder, replace with the actual SightTracker object
        #self.seen_tracker = seen_by_drone.SightTracker(self.arb_field_size)

        #self.seen_tracker.note_field_borders(self.arb_corner_coords)

        self.best_node_List = []

        self.startingNodes=[]
        self.endingNodes=[]
        self.protoMines=[]
        
        self.best_path = Path()
        self.altitude = altitude
        self.fov_deg = fov_deg

        self.matSize=math.tan(math.radians(self.fov_deg/2))*self.altitude*WIDTHOFSQUARE
        #self.viewMat=CellField(matSize,matSize)

        # The local (x, y) of whichever place-to-check is CURRENTLY next in
        # line -- kept fixed across replans (see getPlacesToCheck) so a new
        # discovery mid-mission doesn't reorder the drone toward a totally
        # different first stop than the one it's already committed to
        # flying toward. None until the first getPlacesToCheck call.
        self.nextPlaceToCheckLocal = None

    # Thin wrapper -- see the module-level order_waypoints for the actual
    # heuristic (kept free of self so it's directly testable without
    # constructing a full Pathfinder).
    def order_waypoints(
        self, coords: list[tuple[float, float]], fixed_first: tuple[float, float] = None
    ) -> list[tuple[float, float]]:
        return order_waypoints(coords, fixed_first=fixed_first)

    # Thin wrapper -- see the module-level path_length (kept free of self
    # for the same reason as order_waypoints above).
    def path_length(self, coords: list[tuple[float, float]]) -> float:
        return path_length(coords)

    # Thin wrapper -- see the module-level node_path_length.
    def node_path_length(self, nodePath) -> float:
        return node_path_length(nodePath)

    def buildNodeField(self):
        #STARTING EDGE NODES
        #for i in range( WIDTHOFFIELD // (WIDTHOFSQUARE*2)):
            
        self.startingNodes.append(self.nodeField.addFloatingNode(WIDTHOFFIELD//2, -1))

        for i in range( WIDTHOFFIELD // (WIDTHOFSQUARE*2)):
            self.endingNodes.append(self.nodeField.addFloatingNode(i * WIDTHOFSQUARE*2 +WIDTHOFSQUARE//2, HEIGHTOFFIELD+1))


    def add_discovered_mine(self, mine_lat: float, mine_lon: float):
        x, y = self.coord_converter.latlon_to_local(mine_lat, mine_lon)
        Xsquare, Ysquare = self.nodeField.getSquareCoordinates(x, y)
        # protoMine's centerGridOffset is the real-world position of its own
        # block-grid origin (block (0,0)'s center, before SQUARE_SIDE_LENGTH_FT/2
        # is added back in centerOfBlock) -- NOT an offset within the mine's
        # own square. It must place the grid's center block (mid, mid) at the
        # real-world center of square (Xsquare, Ysquare), matching protoMine's
        # own placeholderGridSideLength//2 exactly, or every mine collapses
        # toward a small fixed offset near the origin instead of its detected
        # square (verified: (45.3, 210.7) -> (9.3, 8.7) before this fix).
        mid = int(self.mine_saftey_radius * 2 + 1) // 2
        Xoffset = WIDTHOFSQUARE * (Xsquare - mid)
        Yoffset = WIDTHOFSQUARE * (Ysquare - mid)
        newProtoMine=protoMine(self.mine_saftey_radius, (mine_lat, mine_lon), (Xoffset, Yoffset))
        self.protoMines.append(newProtoMine)

        self.nodeField.addFromProtoMine(newProtoMine)

        # Stamp this mine's own safety-radius block grid -- already
        # computed by protoMine itself, not re-derived here (see
        # mineCellField.py) -- into the accumulated mineFieldTracker, at
        # the same square offset (Xsquare-mid, Ysquare-mid) used to place
        # the mine's own block grid, so mineFieldTracker becomes a real,
        # live record of every discovered mine's cell-level footprint
        # instead of an unused stub.
        mineBlockField = build_mine_cell_field(newProtoMine)
        self.mineFieldTracker.apply_mask(mineBlockField, Xsquare - mid, Ysquare - mid, op="or")


    def add_discovered_mines(self, discovered_mines_latlon: list[tuple[float, float]]):
        for lat, lon in discovered_mines_latlon:
            self.add_discovered_mine(lat, lon)

    def accept_image_corner_coord(self, corner_coords_latlon: tuple[tuple[float, float]]):
        """
        Given an image's corner coordinates (lat/lon, in order around the
        image), marks every "seen" cell fully enclosed by them -- not cells
        the image only partially clips (see CellField.fill_polygon_covered).
        """
        local_corners = []
        for [lat, lon] in corner_coords_latlon:
            x, y = self.coord_converter.latlon_to_local(lat, lon)
            local_corners.append((x, y))
        self.seen_tracker.fill_polygon_covered(local_corners)

    def increase_radius(self, mine_radius_increment):
        self.nodeField.expandField(mine_radius_increment)
        #self.nodeField.increaseRadius(mine_radius_increment)

    def get_shortest_path(self):
        start = self.startingNodes
        end = self.endingNodes
        shortestPathLength=math.inf
        bestPathNodes=[]
        for i in start:
            newGraph = Graph(self.nodeField.fieldConnection.nodeGraph)
            newPath = newGraph.shortest_path(i, end)
            
            if(node_path_length(newPath) < shortestPathLength):
                shortestPathLength=node_path_length(newPath)
                bestPathNodes=newPath

                
        return bestPathNodes


    def get_cell_path(self,nodePath):
        self.path_tracker.clear_all()
        #xy path gen
        path=[]
        for i in nodePath:
            path.append((i.x, i.y))

        self.path_tracker.mark_path(path)
        return self.path_tracker

    def _verify_places_to_check_cover_need(self, need_field, placements_local, along_ft, across_ft):
        """
        Safety-net invariant, checked on every getPlacesToCheck call: the
        union of footprints about to be visited must fully encompass every
        cell this round of checking was actually supposed to resolve
        (`need_field`), or some cell would silently never get photographed
        even though getPlacesToCheck reports it handled.

        `need_field` is built from get_cell_path's mark_path rasterization
        -- the same "ground truth" footprint get_shortest_path/
        test_seen_covers_final_path already trust -- deliberately NOT by
        recomputing whatever path_cover/cover_with_shape's own internal
        cell reasoning already did, so a bug specific to either of those
        algorithms can't also fool this check the same way it fooled the
        algorithm (this is exactly the kind of gap the mark_path vs.
        path_cells corner-tie disagreement produced -- see path_cells'
        include_tie_neighbors docstring). This is the runtime form of the
        same test droneWorkflowTest.py's test_seen_covers_final_path checks
        after a full simulated mission, moved here so a coverage gap is
        caught on the very call that created it, not just eventually by an
        end-to-end test.

        Raises RuntimeError rather than silently returning something
        incomplete -- a caller trusting this list to fully resolve
        `need_field` is exactly the contract getPlacesToCheck promises.

        Uses fill_aligned_rect_touched (ANY overlap), not
        fill_aligned_rect_covered (the WHOLE cell inside), to build
        `to_be_checked` -- need_field itself (mark_path) is already a "does
        the path geometrically pass through this cell at all" test, not a
        full-encompass one, so comparing it against the covered/whole-cell
        standard would be comparing two different things: a cell the path
        only grazes at one edge, and that a single zero-overlap photo only
        partially overlaps, is expected (see path_cover.py's "seam" note --
        that's what the NEXT replan's seen-tracking cleans up), not a real
        gap. A real gap is a cell no returned placement's footprint touches
        AT ALL. Uses the axis-aligned fast path (fill_aligned_rect_touched,
        an O(rows) direct cell-range fill), not the general polygon path
        (fill_polygon_touched, O(candidate cells) of per-cell tests) --
        every footprint built here is a known, never-rotated rectangle (see
        _footprint_box), so the general polygon math buys nothing but cost.
        """
        if need_field.count() == 0:
            return

        to_be_checked = CellField(
            need_field.width, need_field.height,
            min_corner=need_field.min_corner, max_corner=need_field.max_corner,
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

    def path_or_seen_any_set(self) -> bool:
        """
        Bitwise-ORs the current shortest path's cell footprint ("path
        graph", via get_cell_path/mark_path -- the same ground-truth
        rasterization _verify_places_to_check_cover_need and
        test_seen_covers_final_path use) together with self.seen_tracker
        ("seen graph"), and returns whether the combined field has any cell
        set at all. True as soon as either one does (an OR, not an AND) --
        this is NOT "is there anything left to check" (that's need_field's
        job inside getPlacesToCheck: path minus seen); it's a cheap
        non-empty guard over their union, e.g. for detecting a genuinely
        untouched Pathfinder (no path exists yet AND nothing has been
        photographed) before running logic that assumes one or the other
        has some content.
        """
        path_field = self.get_cell_path(self.get_shortest_path())
        combined = path_field | self.seen_tracker
        return combined.count() > 0

    #THE ACTUAL FUNCTION
    def getPlacesToCheck(
        self, method: str = "path", overlap: float = 0.0, path_width: float = 0.0,
        shape_size_ft: float | tuple[float, float] | None = None,
    ):
        """
        Returns lat/lon waypoints for the camera footprints ("shapes to
        check") needed to cover this drone's slice of the shortest path.

        `method`:
          - "path" (default): walks the shortest path's own geometry
            directly (see path_cover.py) -- the target here is always a
            thin polyline, not an arbitrary 2D region, so this is exact
            (not a heuristic) and orders of magnitude faster than
            rasterizing the path to cells and greedy-covering them.
          - "cellgrid": the original approach (get_cell_path ->
            vertical_slice_index -> CellField.cover_with_shape). Kept
            available as a separate option since cover_with_shape covers
            an ARBITRARY set of cells, not specifically a path -- needed
            for any future case where what needs checking isn't simply
            "along the shortest path" (e.g. a standalone area of interest).

        `overlap`/`path_width` only apply to method="path" (see
        path_cover.py): `overlap` tightens the spacing between consecutive
        placements for redundancy margin; `path_width`, once it exceeds the
        camera footprint's own side length, covers a corridor wider than a
        single shape with multiple parallel rows instead of just the
        centerline. Both default to 0.0, reproducing the original
        zero-overlap, zero-width-path behavior exactly.

        `shape_size_ft` (method="path" only): overrides the footprint size
        that would otherwise be derived from matSize -- a plain number for a
        square footprint, or an (along, across) pair for a rectangular one
        (see path_cover._normalize_shape_size). None (default) reproduces
        the original matSize-derived square exactly.

        method="path" always excludes cells self.seen_tracker already marks
        seen (via accept_image_corner_coord) -- "only checking areas we
        haven't already checked". self.seen_tracker starts empty, so this
        is a no-op until something has actually been marked seen. Excluding
        already-seen cells can leave the remaining unseen path fragmented
        into several disconnected stretches (not just from droneID's
        y-slice anymore) -- path_cover_unseen/place_along_runs treat every
        such stretch independently, so no placement ever spans across a
        seen gap.

        Before returning, verifies (see _verify_places_to_check_cover_need)
        that the returned placements' footprints actually cover everything
        this call needed to cover -- raises RuntimeError if not, rather
        than silently returning an incomplete list.
        """
        shortest_path=self.get_shortest_path()
        self.best_node_List=shortest_path

        # matSize is the camera footprint in feet; both methods below need
        # it as an integer cell count (cover_with_shape's tuple form) or a
        # real-world side length (path_cover) -- computed once, shared.
        matSizeCells = max(1, round(self.matSize / WIDTHOFSQUARE))

        # droneID is a 1-indexed drone number (matches the id prefix Field
        # hands out, e.g. "1-0"), but vertical_slice_index needs a 0-indexed
        # slice -- droneID=1, numOfDrones=1 is "drone 1 of 1", i.e. slice 0 of 1.
        ourSlice = self.get_cell_path(shortest_path).vertical_slice_index(self.droneID - 1, self.numOfDrones)

        if method == "path":
            path_points = [(n.x, n.y) for n in shortest_path]
            shape_size = shape_size_ft if shape_size_ft is not None else matSizeCells * WIDTHOFSQUARE
            min_corner = (0.0, 0.0)
            max_corner = (WIDTHOFFIELD, HEIGHTOFFIELD)
            ShapesToVisit = path_cover_unseen(
                path_points, self.seen_tracker, shape_size, min_corner, max_corner,
                self.droneID, self.numOfDrones, overlap=overlap, path_width=path_width,
            )
            along_ft, across_ft = _normalize_shape_size(shape_size)
            # method="path" only ever needs to cover what's still unseen --
            # see the docstring above -- so that's this call's "need", not
            # the whole slice (which cover_with_shape/"cellgrid" targets
            # unconditionally, seen or not).
            need_field = ourSlice & ~self.seen_tracker
        elif method == "cellgrid":
            if shape_size_ft is not None:
                raise ValueError("shape_size_ft override is only supported for method='path'")
            # self.path_tracker is the FULL (unsliced) footprint get_cell_path
            # just rebuilt above -- ourSlice is its y-sliced view.
            self.best_path = self.path_tracker
            ourPortion = ourSlice
            ShapesToVisit = ourPortion.cover_with_shape((matSizeCells, matSizeCells))
            along_ft = across_ft = matSizeCells * WIDTHOFSQUARE
            need_field = ourPortion
        else:
            raise ValueError(f"getPlacesToCheck: unknown method {method!r}, expected 'path' or 'cellgrid'")

        orderWaypoints=self.order_waypoints(ShapesToVisit, fixed_first=self.nextPlaceToCheckLocal)
        self.nextPlaceToCheckLocal = orderWaypoints[0] if orderWaypoints else None
        self._verify_places_to_check_cover_need(need_field, orderWaypoints, along_ft, across_ft)
        latLonPoints=[]
        for i in orderWaypoints:
            # cover_with_shape/path_cover both already return real-world
            # (feet) coordinates -- multiplying by WIDTHOFSQUARE again here
            # was a bug (doubled every point's distance from the origin;
            # e.g. the field's true center (40,150) was being treated as
            # (80,300), its far corner).
            lat, lon = self.coord_converter.local_to_latlon(i[0], i[1])
            latLonPoints.append((lat, lon))
        return latLonPoints


    # returns final goto list
    def get_way_points_latlon(self,cellField:CellField):


        # what coords do I give here
        start = self.nodeField.placeStartNode()
        end = self.nodeField.placeEndNodes()

        newGraph = Graph(self.nodeField.nodeGraph)
        self.best_node_list = newGraph.shortest_path(start, end)

        self.best_way_points_local, best_wp_seg_info = self.best_path.generate_goto_points(
            self.best_node_list, self.OVERLAP, self.altitude, self.fov_deg
        )

        self.best_way_points_local = seen_by_drone.remove_extra_coords(
            self.seen_tracker,
            self.best_way_points_local,
            best_wp_seg_info,
            [
                self.best_path.ground_covered_image(self.altitude, self.fov_deg),
                self.best_path.ground_covered_image(self.altitude, self.fov_deg),
            ],
        )

        for x, y in self.best_way_points_local:
            lat, lon = self.coord_converter.local_to_latlon(x, y)
            self.best_way_points_latlon.append((lat, lon))

        return self.best_way_points_latlon
        
