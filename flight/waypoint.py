from typing import Any, NamedTuple
import json

from numpy import random


class WaypointGroups(NamedTuple):
    drone1_waypoints: tuple[Waypoint, Waypoint]
    drone2_waypoints: tuple[Waypoint, Waypoint]


class Line:
    def __init__(self, start: Waypoint, end: Waypoint):
        self.start = start
        self.end = end

    @property
    def dx(self) -> float:
        return self.end._get_longitude() - self.start._get_longitude()

    @property
    def dy(self) -> float:
        return self.end._get_latitude() - self.start._get_latitude()


class Waypoint:
    def __init__(
        self, drone_id: int, lat: float, long: float, waypoint_id: int = -1, name: str = ""
    ):

        self.drone_id: int = drone_id
        self.lat: float = lat
        self.long: float = long
        if waypoint_id != -1:
            self.waypoint_id = waypoint_id
        else:
            self.waypoint_id = self.drone_id * 100000000 + random.randint(0, 1000000)

        self.name = name  # Optional name for the waypoint, can be used for easier identification

        self.has_visited = False
        self.has_to_wait = False
        self.waypoints_to_reach: list[Waypoint] = []

    def __str__(self) -> str:
        return f"Waypoint(id={self.waypoint_id}, drone_id={self.drone_id}, lat={self.lat}, long={self.long}, name='{self.name}')"

    def _get_drone_id(self) -> int:
        return self.drone_id

    def _get_waypoint_id(self) -> int:
        return self.waypoint_id

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

    def to_dict(self) -> dict[str, Any]:
        return {
            "waypoint_id": self.waypoint_id,
            "drone_id": self.drone_id,
            "lat": self.lat,
            "long": self.long,
            "name": self.name,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Waypoint":
        return cls(d["waypoint_id"], d["drone_id"], d["lat"], d["long"], d.get("name", ""))

    def check_wait(self) -> None:
        needs_to_wait: bool = False

        for waypoint in self.waypoints_to_reach:
            if not waypoint._get_has_visited():
                needs_to_wait = True
                break

        self._set_has_to_wait(needs_to_wait)

    @staticmethod
    def getChecksum(waypoints: list[Waypoint]) -> int:
        return sum(wp._get_waypoint_id() for wp in waypoints)

    @staticmethod
    def check_for_collision(
        drone1_waypoints: list[Waypoint], drone2_waypoints: list[Waypoint]
    ) -> list[WaypointGroups]:
        drone1_lines = [
            Line(drone1_waypoints[i], drone1_waypoints[i + 1])
            for i in range(len(drone1_waypoints) - 1)
        ]
        drone2_lines = [
            Line(drone2_waypoints[i], drone2_waypoints[i + 1])
            for i in range(len(drone2_waypoints) - 1)
        ]

        waypoint_groups: list[WaypointGroups] = []

        for i in range(len(drone1_waypoints) - 1):
            for j in range(len(drone2_waypoints) - 1):
                s1_x: float = drone1_lines[i].dx
                s1_y: float = drone1_lines[i].dy
                s2_x: float = drone2_lines[j].dx
                s2_y: float = drone2_lines[j].dy

                denom: float = (s1_x * s2_y) - (s2_x * s1_y)

                EPS = 1e-9
                if abs(denom) < EPS:
                    continue  # Lines are parallel, no collision

                denom_is_positive: bool = denom > 0

                s02_x: float = (
                    drone1_lines[i].start._get_longitude() - drone2_lines[j].start._get_longitude()
                )
                s02_y: float = (
                    drone1_lines[i].start._get_latitude() - drone2_lines[j].start._get_latitude()
                )

                s_numer: float = (s1_x * s02_y) - (s1_y * s02_x)

                if (s_numer < 0) == denom_is_positive:
                    continue  # No collision

                t_numer: float = (s2_x * s02_y) - (s2_y * s02_x)

                if (t_numer < 0) == denom_is_positive:
                    continue  # No collision

                if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive:
                    continue  # No collision

                # Collision detected
                waypoint_groups.append(
                    WaypointGroups(
                        (drone1_waypoints[i], drone1_waypoints[i + 1]),
                        (drone2_waypoints[j], drone2_waypoints[j + 1]),
                    )
                )
        return waypoint_groups


# Testing code
if __name__ == "__main__":
    with open("./data/waypoints_test.json", "r") as f:
        data = json.load(f)

    id = 0
    waypoints_1 = []

    for waypoint in data["Set 1"]:
        waypoints_1.append(
            Waypoint(id, id + 1, waypoint["latitude"], waypoint["longitude"], name=waypoint["name"])
        )
        id += 1

    id = 0
    waypoints_2 = []

    for waypoint in data["Set 2"]:
        waypoints_2.append(
            Waypoint(id, id + 1, waypoint["latitude"], waypoint["longitude"], name=waypoint["name"])
        )
        id += 1

    waypoint_groups = Waypoint.check_for_collision(waypoints_1, waypoints_2)
    for group in waypoint_groups:
        print(
            f"Collision detected between ({group.drone1_waypoints[0]}, {group.drone1_waypoints[1]}) and ({group.drone2_waypoints[0]}, {group.drone2_waypoints[1]})"
        )
