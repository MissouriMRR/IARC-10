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


def order_waypoints(coords: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """
    Given a list of local (x,y) coordinates, returns them reordered into a
    decent (not optimal -- exact TSP is NP-hard) visiting order: nearest-
    neighbor construction, tried from several candidate starting points and
    keeping whichever gives the shortest initial tour, then 2-opt local
    search (reverse a sub-segment whenever doing so shortens the total
    path) until no single such swap helps anymore. Treated as an open path
    with no forced return to the start, matching a one-way flight plan
    rather than a round trip.
    """
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


def path_length(coords: list[tuple[float, float]]) -> float:
    """
    Total Euclidean length of the polyline through `coords` (local (x,y)
    coordinates, in visiting order) -- the straight-line distance between
    each consecutive pair, summed. 0.0 for fewer than two points.
    """
    return sum(_dist(coords[i], coords[i + 1]) for i in range(len(coords) - 1))


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
        self, real_corner_coords: tuple[tuple[float, float]], altitude: float, fov_deg: float, droneID:int
    ):

        self.SIM_WIDTH: float = (
            2  # Confirm with nat what this is exactly, this should be an internal constant
        )

        self.OVERLAP = 3  # This will be an internal constant

        self.coord_converter = SimToLatLonTransformer(real_corner_coords, self.SIM_WIDTH)

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

        self.nodeField = Field(0, self.arb_field_size[0], 0, self.arb_field_size[1])
        self.mineFieldTracker = CellField(WIDTHOFFIELD/WIDTHOFSQUARE, HEIGHTOFFIELD/WIDTHOFSQUARE)

        self.seen_tracker= CellField(WIDTHOFFIELD/WIDTHOFSQUARE, HEIGHTOFFIELD/WIDTHOFSQUARE)  # This is a placeholder, replace with the actual SightTracker object

        self.path_tracker=CellField(WIDTHOFFIELD/WIDTHOFSQUARE, HEIGHTOFFIELD/WIDTHOFSQUARE)  # This is a placeholder, replace with the actual SightTracker object
        #self.seen_tracker = seen_by_drone.SightTracker(self.arb_field_size)

        #self.seen_tracker.note_field_borders(self.arb_corner_coords)

        self.best_node_List = []
        self.best_way_points_latlon = []  # stores best path
        self.best_way_points_local = []

        self.startingNodes=[]
        self.endingNodes=[]
        self.protoMines=[]
        
        self.best_path = Path()
        self.altitude = altitude
        self.fov_deg = fov_deg

    # Thin wrapper -- see the module-level order_waypoints for the actual
    # heuristic (kept free of self so it's directly testable without
    # constructing a full Pathfinder).
    def order_waypoints(self, coords: list[tuple[float, float]]) -> list[tuple[float, float]]:
        return order_waypoints(coords)

    # Thin wrapper -- see the module-level path_length (kept free of self
    # for the same reason as order_waypoints above).
    def path_length(self, coords: list[tuple[float, float]]) -> float:
        return path_length(coords)

    # Thin wrapper -- see the module-level node_path_length.
    def node_path_length(self, nodePath) -> float:
        return node_path_length(nodePath)

    def buildNodeField(self):
        #STARTING EDGE NODES
        for i in range( WIDTHOFFIELD // (WIDTHOFSQUARE*2)):
            
            self.startingNodes.append(self.nodeField.addFloatingNode(i * WIDTHOFSQUARE*2 +WIDTHOFSQUARE//2, -1))

        for i in range( WIDTHOFFIELD // (WIDTHOFSQUARE*2)):
            self.endingNodes.append(self.nodeField.addFloatingNode(i * WIDTHOFSQUARE*2 +WIDTHOFSQUARE//2, HEIGHTOFFIELD+1))


    def add_discovered_mine(self, mine_lat: float, mine_lon: float):
        x, y = self.coord_converter.latlon_to_local(mine_lat, mine_lon)
        Xsquare, Ysquare = self.nodeField.getSquareCoordinates(x, y)
        Xoffset = (x - Xsquare*WIDTHOFSQUARE+WIDTHOFSQUARE/2) 
        Yoffset = (y - Ysquare*WIDTHOFSQUARE+WIDTHOFSQUARE/2) 
        newProtoMine=protoMine(self.mine_saftey_radius, (mine_lat, mine_lon), (Xoffset, Yoffset))
        self.protoMines.append(newProtoMine)

        self.nodeField.addFromProtoMine(newProtoMine)


    def add_discovered_mines(self, discovered_mines_latlon: list[tuple[float, float]]):
        for lat, lon in discovered_mines_latlon:
            self.add_discovered_mine(lat, lon)

    def accept_image_corner_coord(self, corner_coords_latlon: tuple[tuple[float, float]]):
        local_corners = []
        for [lat, lon] in corner_coords_latlon:
            x, y = self.coord_converter.latlon_to_local(lat, lon)
            local_corners.append([x, y])
        self.seen_tracker.note_pic(local_corners)

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
        
