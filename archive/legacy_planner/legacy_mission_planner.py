"""ARCHIVED -- the legacy maze planner's adapter, cut from flight/pathfinding/mission_planner.py.

It adapted flight/pathfinder.py (archived beside this file) to the MissionPlanner interface
while both planners were selectable (FlightSettings.planner == "legacy"). Kept for reference
only; see README.md in this folder.
"""

from flight.pathfinding.iarc_report import format_iarc_path
from flight.pathfinding.mission_planner import MissionPlanner


class LegacyMissionPlanner(MissionPlanner):
    """The original maze planner. Every method is the inline Pathfinder usage the states had
    before this interface existed, moved here unchanged."""

    kind = "legacy"

    def __init__(self, corners_latlon, start_latlon, start_edge, has_partner, has_assistant,
                 altitude, drone_id):
        from flight.pathfinder import HEIGHTOFFIELD, WIDTHOFFIELD, Pathfinder
        self.field_w, self.field_h = float(WIDTHOFFIELD), float(HEIGHTOFFIELD)
        self._has_assistant = has_assistant
        self.pf = Pathfinder(corners_latlon, altitude, 90, drone_id)
        self.pf.buildNodeField(start_latlon, startEdge=start_edge)
        if has_partner:
            self.pf.set_cross_pair_home_boundary(HEIGHTOFFIELD / 2.0)

    def latlon_to_local(self, lat, lon):
        x, y = self.pf.coord_converter.latlon_to_local(lat, lon)
        return float(x), float(y)

    def local_to_latlon(self, x, y):
        lat, lon = self.pf.coord_converter.local_to_latlon(x, y)
        return float(lat), float(lon)

    def start_if_needed(self):
        pf = self.pf
        if pf.maze_confirmed_path or pf.maze_a_path or pf.maze_b_path:
            return None
        pf.start_maze_navigation()
        return bool(pf.maze_a_path)

    def record_footprint(self, corners_latlon):
        self.pf.accept_image_corner_coord(corners_latlon)

    def cell_seen_at(self, lat, lon):
        x, y = self.latlon_to_local(lat, lon)
        col, row = self.pf.seen_tracker.real_to_cell(x, y)
        if not (0 <= col < self.pf.seen_tracker.width and 0 <= row < self.pf.seen_tracker.height):
            return None
        return self.pf.seen_tracker.get(col, row)

    def add_mine(self, lat, lon, label):
        pf = self.pf
        obstacle, _was_merged, rewound = pf.add_discovered_mine(lat, lon)
        if rewound:
            return None
        if label == "A":
            if not pf.start_helper_node_detour(obstacle):
                pf.on_forward_mine_discovered()
        else:
            pf.reroute_b_segment()
        return obstacle.obstacle_hash

    def add_remote_mine(self, lat, lon):
        pf = self.pf
        pf.add_discovered_mine(lat, lon, prefer_local_patch=True)
        if pf.last_patched_span is None:
            return None
        return [self.local_to_latlon(n.x, n.y) for n in pf.last_patched_span]

    def record_assistant_photo(self, corners_latlon, mines_latlon):
        pf = self.pf
        pf.accept_image_corner_coord(corners_latlon)
        for mine_lat, mine_lon in mines_latlon:
            _obstacle, _was_merged, rewound = pf.add_discovered_mine(mine_lat, mine_lon)
            if not rewound:
                pf.reroute_b_segment()
        return []

    def next_batch(self, overlap, shape_size_ft, positions=None):
        places = self.pf.get_places_to_check_maze(overlap=overlap, shape_size_ft=shape_size_ft)
        a, b = places["a"], places["b"]
        own = [(lat, lon, "A") for lat, lon in a]
        if self._has_assistant:
            assistant = list(b)
        else:
            own += [(lat, lon, "B") for lat, lon in b]
            assistant = []
        return own, assistant, not a and not b

    def complete(self, mission_path):
        pf = self.pf
        settled = [self.local_to_latlon(n.x, n.y) for n in pf.maze_b_path]
        pf.confirm_b_into_c()
        return list(mission_path) + settled

    def mine_hash(self):
        return self.pf.nodeField.mineHash()

    def expand_margin(self, ft):
        pf = self.pf
        pf.increase_radius(ft)
        passes = pf.repair_path_after_expansion()
        return {"expansion_ft": ft, "mines": len(pf.nodeField.mines),
                "unions": len(pf.nodeField.unionObstacles), "repair_passes": passes}

    def route_latlon(self):
        return [self.local_to_latlon(n.x, n.y) for n in self.pf.get_maze_path()]

    def iarc_path(self, route_latlon, buffer_width=0):
        path_xy = [self.latlon_to_local(lat, lon) for lat, lon in route_latlon]
        return format_iarc_path(path_xy, buffer_width, strict=False,
                                field_w=self.field_w, field_h=self.field_h)
