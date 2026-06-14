from typing import Any, NamedTuple
import json
import math

from numpy import random

# 32 inches in latitude degrees (1 inch ≈ 2.28e-7 degrees)
COLLISION_RADIUS: float = 0.000000228 * 32  # NOTE edit this as needed for more spacing


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

    def __repr__(self):
        return self.__str__()

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Waypoint":
        return cls(
            drone_id=d["drone_id"],
            lat=d["lat"],
            long=d["long"],
            waypoint_id=d["waypoint_id"],
            name=d.get("name", ""),
        )

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
    def _segment_min_distance(l1: Line, l2: Line) -> float:
        # Minimum Euclidean distance between two line segments in lat/lon space.
        d1x, d1y = l1.dx, l1.dy
        d2x, d2y = l2.dx, l2.dy
        rx = l1.start._get_longitude() - l2.start._get_longitude()
        ry = l1.start._get_latitude() - l2.start._get_latitude()

        a = d1x * d1x + d1y * d1y
        e = d2x * d2x + d2y * d2y
        f = d2x * rx + d2y * ry

        DEGEN_EPS = 1e-20  # for zero-length segment detection
        if a <= DEGEN_EPS and e <= DEGEN_EPS:
            # If a and e are both <= DEGEN_EPS, they are zero length (a point)
            # So, return distance between points (c^2=a^2+b^2)
            return math.sqrt(rx * rx + ry * ry)

        if a <= DEGEN_EPS:
            # l1 is a point; s=0, find closest point on l2 to l1.start
            s, t = 0.0, max(0.0, min(1.0, f / e))
        else:
            c = d1x * rx + d1y * ry
            if e <= DEGEN_EPS:
                # l2 is a point; t=0, find closest point on l1 to l2.start
                s, t = max(0.0, min(1.0, -c / a)), 0.0
            else:
                b = d1x * d2x + d1y * d2y
                denom = a * e - b * b
                # relative threshold: avoids misclassifying short non-parallel segments as parallel
                s = (
                    max(0.0, min(1.0, (b * f - c * e) / denom))
                    if abs(denom) > 1e-10 * a * e
                    else 0.0
                )
                t = (b * s + f) / e
                if t < 0.0:
                    t, s = 0.0, max(0.0, min(1.0, -c / a))
                elif t > 1.0:
                    t, s = 1.0, max(0.0, min(1.0, (b - c) / a))

        # s parameterizes l1, t parameterizes l2
        dx = (l1.start._get_longitude() + s * d1x) - (l2.start._get_longitude() + t * d2x)
        dy = (l1.start._get_latitude() + s * d1y) - (l2.start._get_latitude() + t * d2y)
        return math.sqrt(dx * dx + dy * dy)

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

        collision_waypoint_groups: list[WaypointGroups] = []

        # Check distance between segments for each line and return one's that could collide
        for i in range(len(drone1_waypoints) - 1):
            for j in range(len(drone2_waypoints) - 1):
                if (
                    Waypoint._segment_min_distance(drone1_lines[i], drone2_lines[j])
                    < COLLISION_RADIUS
                ):
                    collision_waypoint_groups.append(
                        WaypointGroups(
                            (drone1_waypoints[i], drone1_waypoints[i + 1]),
                            (drone2_waypoints[j], drone2_waypoints[j + 1]),
                        )
                    )
        return collision_waypoint_groups


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
