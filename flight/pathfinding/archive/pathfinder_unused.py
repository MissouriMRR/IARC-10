"""
Dead code pulled out of flight/pathfinder.py: zero callers anywhere in the
repo, including tests, as of this move. Not imported by anything -- kept
for reference only. See git history for how each piece attaches to
Pathfinder (self.checkpointNode, self.flownPrefixNodes, etc.) if reviving
any of it.
"""


# ==================================================================
# Checkpoint-pinned path planning
#
# Superseded by the plain greedy get_shortest_path (per explicit request:
# "archive the current pinning setup and revert back to the old greedy
# algorithm"). To re-enable: point getPlacesToCheck/path_or_seen_any_set at
# get_pinned_node_path/get_full_node_path instead of get_shortest_path, and
# call advance_checkpoint() wherever a mine gets discovered.
#
# What it did: pinned path planning to start from the drone's last
# PROVEN-checked node (via seen_tracker coverage) instead of the true field
# entry, so a replan never re-touched already-flown ground. Measured
# (70-mine simulated field): mean waypoints 285->234 (-18%), mean replans
# 13.5->11.5, stdev 96.8->62.6 vs. hysteresis alone.
# ==================================================================

def get_pinned_node_path(self, hysteresis_tolerance):
    """Path from self.checkpointNode (or self.startingNodes if unset) to
    the best ending node. Falls back to a fresh search from
    self.startingNodes if the checkpoint node has become disconnected."""
    if self.checkpointNode is None:
        return self._dijkstra_path_with_hysteresis(self.startingNodes, hysteresis_tolerance)

    path = self._dijkstra_path_with_hysteresis([self.checkpointNode], hysteresis_tolerance)
    if path:
        return path

    self.checkpointNode = None
    self.flownPrefixNodes = []
    return self._dijkstra_path_with_hysteresis(self.startingNodes, hysteresis_tolerance)


def get_full_node_path(self, hysteresis_tolerance):
    """self.flownPrefixNodes (frozen, already-flown) + get_pinned_node_path()'s
    current plan for what's still ahead."""
    active = self.get_pinned_node_path(hysteresis_tolerance)
    if self.flownPrefixNodes:
        return self.flownPrefixNodes[:-1] + active  # last entry duplicates active[0]
    return active


def advance_checkpoint(self) -> None:
    """Pins self.checkpointNode to the furthest node whose full prefix is
    already covered by self.seen_tracker -- derived from real coverage,
    not which waypoint triggered a discovery (TSP order != path order)."""
    active_path = self.get_pinned_node_path()
    if len(active_path) < 2:
        return

    best_idx = 0
    for i in range(1, len(active_path)):
        prefix_cells = self.rasterize_node_path(active_path[: i + 1])
        if (prefix_cells & ~self.seen_tracker).count() == 0:
            best_idx = i
        else:
            break

    if best_idx == 0:
        return

    if not self.flownPrefixNodes:
        self.flownPrefixNodes.append(active_path[0])
    self.flownPrefixNodes.extend(active_path[1 : best_idx + 1])
    self.checkpointNode = active_path[best_idx]


# ==================================================================
# Pre-hysteresis, pre-checkpoint-pinning waypoint generator. Never called
# from anywhere (was already commented out in pathfinder.py before this
# move).
# ==================================================================

def get_way_points_latlon(self, cellField):
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


# ==================================================================
# Miscellaneous unused Pathfinder methods
# ==================================================================

def add_discovered_mines(self, discovered_mines_latlon):
    """Plural wrapper around add_discovered_mine. Nothing calls it."""
    return [self.add_discovered_mine(lat, lon) for lat, lon in discovered_mines_latlon]


def path_or_seen_any_set(self) -> bool:
    """True if get_shortest_path()'s cells OR self.seen_tracker have
    anything set (cheap non-empty guard, not "anything left to check")."""
    path_field = self.rasterize_node_path(self.get_shortest_path())
    combined = path_field | self.seen_tracker
    return combined.count() > 0


def path_length(self, coords):
    """Thin wrapper -- see module-level path_length in pathfinder.py."""
    from flight.pathfinder import path_length as _path_length
    return _path_length(coords)


def node_path_length(self, nodePath):
    """Thin wrapper -- see module-level node_path_length in pathfinder.py."""
    from flight.pathfinder import node_path_length as _node_path_length
    return _node_path_length(nodePath)


# ==================================================================
# Miscellaneous unused module-level helper
# ==================================================================

def _footprint_corners(cx, cy, along_ft, across_ft):
    """Same box as _footprint_box, returned as 4 corner points."""
    from flight.pathfinder import _footprint_box
    x0, y0, x1, y1 = _footprint_box(cx, cy, along_ft, across_ft)
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
