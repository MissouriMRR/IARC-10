class Waypoint:
    def __init__(self, drone_id: int, lat: float, long: float):
        self.drone_id = drone_id
        self.lat = lat
        self.long = long

        self.has_visited = False
        self.has_to_wait = False
        self.waypoints_to_reach: list[Waypoint] = []

    def _get_drone_id(self) -> int:
        return self.drone_id

    def _get_latitude(self) -> float:
        return self.lat

    def _get_longitude(self) -> float:
        return self.long

    def _get_has_visited(self) -> bool:
        return self.has_visited

    def _get_has_to_wait(self) -> bool:
        return self.has_to_wait

    def _get_waypoints_to_reach(self) -> list[Waypoint]:
        return self.waypoints_to_reach

    def _set_latitude(self, new_lat: float):
        self.lat = new_lat

    def _set_longitude(self, new_long: float):
        self.long = new_long

    def _set_has_visited(self, new_has_visited: bool):
        self.has_visited = new_has_visited

    def _set_has_to_wait(self, new_has_to_wait: bool):
        self.has_to_wait = new_has_to_wait

    def _add_waypoints_to_reach(self, new_waypoint: Waypoint):
        self.waypoints_to_reach.append(new_waypoint)

    def check_wait(self) -> None:
        for waypoint in self.waypoints_to_reach:
            if not waypoint._get_has_visited():
                self._set_has_to_wait(True)
        self._set_has_to_wait(False)
