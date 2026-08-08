"""
Generate latitude/longitude waypoints in a circle around a center point.

Uses the equirectangular approximation with proper Earth radius math.
For radii up to ~100km this is accurate to within a few meters.
"""

import math
from typing import List, Tuple
from flight.waypoint import Waypoint

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius in meters


def circle_waypoints(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    drone_id: int,
    num_points: int = 36,
    closed: bool = False,
    lap: int = 0,
    start_bearing_deg: float = 0.0,
) -> List[Waypoint]:
    """
    Generate evenly-spaced waypoints on a circle around a center coordinate.

    Args:
        center_lat: Center latitude in decimal degrees (-90 to 90).
        center_lon: Center longitude in decimal degrees (-180 to 180).
        radius_m:   Circle radius in meters (must be > 0).
        num_points: Number of waypoints around the circle (default 36 = every 10°).
        closed:     If True, repeats the first point at the end so the
                    path closes cleanly (useful for drawing/flying loops).
        lap:        Which lap of the circle these points belong to. Tags the
                    waypoints and makes their ids unique across laps, so a
                    point can be identified in a log without ambiguity.
        start_bearing_deg:
                    Compass bearing of the first waypoint from the center
                    (0 = due north, 90 = due east). Every drone in a formation
                    must use the same value or the circles stop being in phase
                    and the lockstep separation guarantee is lost. It matters
                    because it also decides the direction each drone flies to
                    *enter* its circle — see ENTRY_BEARING_DEG in poif_impl.

    Returns:
        List of Waypoints, starting at `start_bearing_deg` from the center and
        going clockwise.
    """
    if not -90.0 <= center_lat <= 90.0:
        raise ValueError(f"center_lat {center_lat} out of range [-90, 90]")
    if not -180.0 <= center_lon <= 180.0:
        raise ValueError(f"center_lon {center_lon} out of range [-180, 180]")
    if radius_m <= 0:
        raise ValueError(f"radius_m must be positive, got {radius_m}")
    if num_points < 3:
        raise ValueError(f"num_points must be >= 3, got {num_points}")

    lat_rad = math.radians(center_lat)

    # Degrees of latitude per meter (constant)
    deg_lat_per_m = 1.0 / (math.pi * EARTH_RADIUS_M / 180.0)
    # Degrees of longitude per meter (depends on latitude)
    deg_lon_per_m = 1.0 / (math.pi * EARTH_RADIUS_M * math.cos(lat_rad) / 180.0)

    waypoints: List[Tuple[float, float]] = []
    for i in range(num_points):
        # Bearing in radians, 0 = North, increasing clockwise
        bearing = math.radians(start_bearing_deg) + 2.0 * math.pi * i / num_points
        d_north = radius_m * math.cos(bearing)
        d_east = radius_m * math.sin(bearing)

        lat = center_lat + d_north * deg_lat_per_m
        lon = center_lon + d_east * deg_lon_per_m

        # Normalize longitude to [-180, 180]
        lon = ((lon + 180.0) % 360.0) - 180.0
        # Deterministic id: drone / lap / index all readable straight off the
        # number. The default random id makes it impossible to line a waypoint
        # up across two drones' logs.
        waypoints.append(
            Waypoint(
                drone_id=drone_id,
                lat=lat,
                long=lon,
                waypoint_id=drone_id * 1_000_000 + lap * 1_000 + i,
                name=f"d{drone_id}L{lap}P{i}",
                lap=lap,
                index=i,
            )
        )

    if closed:
        waypoints.append(waypoints[0])

    return waypoints


if __name__ == "__main__":
    # Demo: 12 waypoints in a 500 m circle around Rolla, MO
    center = (37.9485, -91.7715)
    radius = 500.0
    points = circle_waypoints(*center, radius_m=radius, drone_id=0, num_points=12)

    print(f"12 waypoints, {radius:.0f} m circle around {center}:")
    print(f"{'#':>3}  {'Latitude':>11}  {'Longitude':>12}")
    for i, waypoint in enumerate(points):
        print(f"{i:>3}  {waypoint.lat:>11.6f}  {waypoint.long:>12.6f}")
