from __future__ import annotations

from typing import Any
import math
import random

EARTH_RADIUS_M: float = 6_378_137.0  # WGS-84 equatorial radius
_DEG_TO_M: float = math.pi * EARTH_RADIUS_M / 180.0  # meters per degree of latitude


def project_to_meters(
    lat: float, lon: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """Project a lat/lon onto a local east/north frame in meters around a reference."""
    lon_scale = _DEG_TO_M * math.cos(math.radians(ref_lat))
    return (lon - ref_lon) * lon_scale, (lat - ref_lat) * _DEG_TO_M


def _segment_distance_xy(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    cx: float,
    cy: float,
    dx_: float,
    dy_: float,
) -> float:
    """Minimum distance between segments a->b and c->d, all in the same meter frame."""
    d1x, d1y = bx - ax, by - ay
    d2x, d2y = dx_ - cx, dy_ - cy
    rx, ry = ax - cx, ay - cy

    a = d1x * d1x + d1y * d1y
    e = d2x * d2x + d2y * d2y
    f = d2x * rx + d2y * ry

    DEGEN_EPS = 1e-12  # squared meters; segments shorter than a micrometer are points
    if a <= DEGEN_EPS and e <= DEGEN_EPS:
        # Both segments are points, so this is just the distance between them.
        return math.sqrt(rx * rx + ry * ry)

    if a <= DEGEN_EPS:
        # First segment is a point; s=0, find the closest point on the second to it.
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = d1x * rx + d1y * ry
        if e <= DEGEN_EPS:
            # Second segment is a point; t=0, find the closest point on the first to it.
            s, t = max(0.0, min(1.0, -c / a)), 0.0
        else:
            b = d1x * d2x + d1y * d2y
            denom = a * e - b * b
            # Relative threshold: avoids misclassifying short non-parallel segments
            # as parallel.
            s = max(0.0, min(1.0, (b * f - c * e) / denom)) if abs(denom) > 1e-10 * a * e else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))

    # s parameterizes the first segment, t the second.
    dx = (ax + s * d1x) - (cx + t * d2x)
    dy = (ay + s * d1y) - (cy + t * d2y)
    return math.sqrt(dx * dx + dy * dy)


def segment_distance(
    a1: tuple[float, float],
    a2: tuple[float, float],
    b1: tuple[float, float],
    b2: tuple[float, float],
) -> float:
    """Minimum distance in meters between segments a1->a2 and b1->b2.

    Each point is a (latitude, longitude) pair in degrees. Degenerate segments
    (a1 == a2) are treated as points, so this also answers point-to-segment and
    point-to-point queries.
    """
    ref_lat, ref_lon = a1
    ax, ay = project_to_meters(a1[0], a1[1], ref_lat, ref_lon)
    bx, by = project_to_meters(a2[0], a2[1], ref_lat, ref_lon)
    cx, cy = project_to_meters(b1[0], b1[1], ref_lat, ref_lon)
    dx_, dy_ = project_to_meters(b2[0], b2[1], ref_lat, ref_lon)
    return _segment_distance_xy(ax, ay, bx, by, cx, cy, dx_, dy_)


class Waypoint:
    def __init__(
        self,
        drone_id: int,
        lat: float,
        long: float,
        waypoint_id: int = -1,
        name: str = "",
        lap: int | None = None,
        index: int | None = None,
    ):

        self.drone_id: int = drone_id
        self.lat: float = lat
        self.long: float = long
        if waypoint_id != -1:
            self.waypoint_id = waypoint_id
        else:
            self.waypoint_id = self.drone_id * 100000000 + random.randint(0, 999999)

        # Which lap of the circle this point belongs to and its position within that lap.
        self.lap: int | None = lap
        self.index: int | None = index

        self.name = name  # Optional name for the waypoint, can be used for easier identification

        self.has_visited = False
        self.has_to_wait = False
        self.waypoints_to_reach: list[Waypoint] = []

    def __str__(self) -> str:
        return (
            f"Waypoint(id={self.waypoint_id}, drone_id={self.drone_id}, lat={self.lat},"
            f" long={self.long}, lap={self.lap}, index={self.index}, name='{self.name}')"
        )

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
            "lap": self.lap,
            "index": self.index,
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
            lap=d.get("lap"),
            index=d.get("index"),
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
