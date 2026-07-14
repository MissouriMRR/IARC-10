"""
Small geodesy helpers shared by path generation and LIDAR mapping.

All functions use the same equirectangular approximation as
flight/circlePath.py, which is accurate to within a few meters for
distances up to ~100 km.
"""

import math

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius in meters


def deg_per_meter(lat_deg: float) -> tuple[float, float]:
    """
    Degrees of latitude and longitude per meter at the given latitude.

    Parameters
    ----------
    lat_deg : float
        Latitude in decimal degrees.

    Returns
    -------
    tuple[float, float]
        (degrees latitude per meter, degrees longitude per meter).
    """
    deg_lat_per_m = 1.0 / (math.pi * EARTH_RADIUS_M / 180.0)
    deg_lon_per_m = 1.0 / (math.pi * EARTH_RADIUS_M * math.cos(math.radians(lat_deg)) / 180.0)
    return deg_lat_per_m, deg_lon_per_m


def offset_latlon(lat: float, lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
    """
    Project a point `dist_m` meters from (lat, lon) along a compass bearing.

    Parameters
    ----------
    lat, lon : float
        Starting coordinate in decimal degrees.
    bearing_deg : float
        Compass bearing in degrees, 0 = North, increasing clockwise.
    dist_m : float
        Distance to project in meters.

    Returns
    -------
    tuple[float, float]
        The projected (latitude, longitude) in decimal degrees.
    """
    bearing_rad = math.radians(bearing_deg)
    d_north = dist_m * math.cos(bearing_rad)
    d_east = dist_m * math.sin(bearing_rad)

    deg_lat_per_m, deg_lon_per_m = deg_per_meter(lat)
    new_lat = lat + d_north * deg_lat_per_m
    new_lon = lon + d_east * deg_lon_per_m

    # Normalize longitude to [-180, 180]
    new_lon = ((new_lon + 180.0) % 360.0) - 180.0
    return new_lat, new_lon


def bearing_deg_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compass bearing from point 1 to point 2.

    Returns
    -------
    float
        Bearing in degrees [0, 360), 0 = North, increasing clockwise.
    """
    deg_lat_per_m, deg_lon_per_m = deg_per_meter(lat1)
    d_north = (lat2 - lat1) / deg_lat_per_m
    d_east = (lon2 - lon1) / deg_lon_per_m
    return math.degrees(math.atan2(d_east, d_north)) % 360.0


def latlon_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Ground (2-D) distance in meters between two coordinates.
    """
    deg_lat_per_m, deg_lon_per_m = deg_per_meter(lat1)
    d_north = (lat2 - lat1) / deg_lat_per_m
    d_east = (lon2 - lon1) / deg_lon_per_m
    return math.hypot(d_north, d_east)


def point_segment_distance_m(
    point: tuple[float, float],
    seg_start: tuple[float, float],
    seg_end: tuple[float, float],
) -> float:
    """
    Minimum distance in meters from a (lat, lon) point to the segment
    between two (lat, lon) endpoints.
    """
    deg_lat_per_m, deg_lon_per_m = deg_per_meter(seg_start[0])

    # Local planar coordinates in meters, origin at seg_start
    def to_local(lat: float, lon: float) -> tuple[float, float]:
        return (
            (lon - seg_start[1]) / deg_lon_per_m,
            (lat - seg_start[0]) / deg_lat_per_m,
        )

    px, py = to_local(*point)
    ex, ey = to_local(*seg_end)

    seg_len_sq = ex * ex + ey * ey
    if seg_len_sq == 0.0:
        return math.hypot(px, py)

    t = max(0.0, min(1.0, (px * ex + py * ey) / seg_len_sq))
    return math.hypot(px - t * ex, py - t * ey)
