"""
Generate latitude/longitude waypoints in a circle around a center point.

Uses the equirectangular approximation with proper Earth radius math.
For radii up to ~100km this is accurate to within a few meters.
"""

import math
from typing import List, Tuple
from waypoint import Waypoint

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius in meters


def circle_waypoints(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    drone_id: int,
    num_points: int = 36,
    closed: bool = False,
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

    Returns:
        List of (latitude, longitude) tuples in decimal degrees, starting
        due north of center and going clockwise.
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
        bearing = 2.0 * math.pi * i / num_points
        d_north = radius_m * math.cos(bearing)
        d_east = radius_m * math.sin(bearing)

        lat = center_lat + d_north * deg_lat_per_m
        lon = center_lon + d_east * deg_lon_per_m

        # Normalize longitude to [-180, 180]
        lon = ((lon + 180.0) % 360.0) - 180.0
        waypoints.append(Waypoint(drone_id=drone_id, lat=lat, long=lon))

    if closed:
        waypoints.append(waypoints[0])

    return waypoints


if __name__ == "__main__":
    # Demo: 12 waypoints in a 500 m circle around Rolla, MO
    center = (37.9485, -91.7715)
    radius = 500.0
    points = circle_waypoints(*center, radius_m=radius, num_points=12)

    print(f"12 waypoints, {radius:.0f} m circle around {center}:")
    print(f"{'#':>3}  {'Latitude':>11}  {'Longitude':>12}")
    for i, (lat, lon) in enumerate(points):
        print(f"{i:>3}  {lat:>11.6f}  {lon:>12.6f}")
