

import flight.pathfinding.utils.seen_by_drone as seen_by_drone

class Pathfinder:
    def __init__(self, field_size, mine_radius):

        self.field_size: tuple[int, int] = field_size
        self.mine_radius = mine_radius
        self.seen_tracker = seen_by_drone.SightTracker(self.field_size)