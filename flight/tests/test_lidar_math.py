"""
Unit tests for the pure-math pieces of the LIDAR mapping pipeline.
No drone or simulator needed.

Run with:  uv run python -m flight.tests.test_lidar_math
"""

import math
import time

from flight.circlePath import circle_waypoints
from flight.lidar import (
    LidarConfig,
    LidarController,
    RangeSample,
    filter_scan_samples,
    nearest_circle_point,
    rotate_to_nearest,
)
from flight.pathfinding.utils.coord_convert import FT_PER_M, field_transformer_ft
from flight.pathfinding.utils.geo import (
    bearing_deg_between,
    latlon_distance_m,
    offset_latlon,
    point_segment_distance_m,
)

# Football-field test corners from coord_convert.py's own test case
FIELD_CORNERS = [
    {"lat": 36.021683, "lon": -95.941831},  # 1
    {"lat": 36.020694, "lon": -95.941856},  # 2
    {"lat": 36.021694, "lon": -95.942372},  # 3 (origin)
    {"lat": 36.020703, "lon": -95.942397},  # 4
]

CENTER = (36.0212, -95.9421)


def make_sample(lat, lon, heading_deg, range_m, sensor_id=-1, timestamp=None):
    return RangeSample(
        range_m=range_m,
        timestamp=timestamp if timestamp is not None else time.monotonic(),
        lat=lat,
        lon=lon,
        alt=5.0,
        heading_deg=heading_deg,
        sensor_id=sensor_id,
    )


def test_offset_latlon_round_trip():
    for bearing in (0.0, 45.0, 137.0, 250.0, 359.0):
        for dist in (1.0, 8.0, 50.0):
            lat, lon = offset_latlon(*CENTER, bearing, dist)
            assert abs(latlon_distance_m(*CENTER, lat, lon) - dist) < 0.01
            assert abs(bearing_deg_between(*CENTER, lat, lon) - bearing) < 0.1


def test_offset_matches_circle_waypoints():
    circle = circle_waypoints(*CENTER, 10.0, drone_id=1, num_points=8)
    for i, waypoint in enumerate(circle):
        expected = offset_latlon(*CENTER, 360.0 * i / 8, 10.0)
        assert abs(waypoint.lat - expected[0]) < 1e-12
        assert abs(waypoint.long - expected[1]) < 1e-12


def test_field_transformer_ft():
    transformer = field_transformer_ft(FIELD_CORNERS)
    # Corner 3 is the origin; corner 4 is ~360 ft along the x axis
    corner3 = transformer.latlon_to_local(FIELD_CORNERS[2]["lat"], FIELD_CORNERS[2]["lon"])
    corner4 = transformer.latlon_to_local(FIELD_CORNERS[3]["lat"], FIELD_CORNERS[3]["lon"])
    assert math.hypot(*corner3) < 1.0
    assert abs(corner4[0] - 360.0) < 15.0  # field is "~360 ft", so loose bound
    assert abs(corner4[1]) < 1.0

    # 10 meters of ground distance must equal 10 * FT_PER_M local units
    point = offset_latlon(*CENTER, 90.0, 10.0)
    a = transformer.latlon_to_local(*CENTER)
    b = transformer.latlon_to_local(*point)
    assert abs(math.dist(a, b) - 10.0 * FT_PER_M) < 0.1


def test_point_segment_distance():
    a = CENTER
    b = offset_latlon(*CENTER, 90.0, 100.0)
    # Point 30 m north of the segment midpoint
    mid = offset_latlon(*CENTER, 90.0, 50.0)
    p = offset_latlon(*mid, 0.0, 30.0)
    assert abs(point_segment_distance_m(p, a, b) - 30.0) < 0.1
    # Point beyond the end of the segment measures to the endpoint
    beyond = offset_latlon(*CENTER, 90.0, 140.0)
    assert abs(point_segment_distance_m(beyond, a, b) - 40.0) < 0.1


def test_estimate_center():
    # Drone 5 m south of object, looking north at it
    drone_pos = offset_latlon(*CENTER, 180.0, 5.0)
    samples = [make_sample(*drone_pos, 0.0, 5.0) for _ in range(5)]
    estimate = LidarController.estimate_center(samples)
    assert latlon_distance_m(*CENTER, *estimate) < 0.1


def test_sensor_lateral_offset():
    drone_pos = offset_latlon(*CENTER, 180.0, 5.0)
    left = make_sample(*drone_pos, 0.0, 5.0, sensor_id=0).hit_latlon()
    right = make_sample(*drone_pos, 0.0, 5.0, sensor_id=1).hit_latlon()
    # Front-left and front-right sensors are 0.3 m apart laterally
    assert abs(latlon_distance_m(*left, *right) - 0.3) < 0.01
    assert bearing_deg_between(*left, *right) - 90.0 < 0.5  # right is east of left


def test_filter_scan_samples_rejects_other_obstacles():
    transformer = field_transformer_ft(FIELD_CORNERS)
    standoff = 4.0
    # Valid: drone on the circle, object return at the standoff distance
    drone_pos = offset_latlon(*CENTER, 180.0, standoff)
    good = make_sample(*drone_pos, 0.0, standoff)
    # Reject: return from beyond the circle (a different obstacle)
    far = make_sample(*drone_pos, 0.0, 7.5)
    # Reject: short return whose hit point lands far from the center
    # (looking away from the object at something else nearby)
    askew = make_sample(*drone_pos, 180.0, 3.0)

    vertices_ft, vertices_latlon = filter_scan_samples(
        [good, far, askew], CENTER, standoff, max_object_radius_ft=8.0, transformer=transformer
    )
    assert len(vertices_ft) == 1
    assert len(vertices_latlon) == 1
    center_ft = transformer.latlon_to_local(*CENTER)
    assert math.dist(vertices_ft[0], center_ft) < 1.0


def test_is_duplicate():
    transformer = field_transformer_ft(FIELD_CORNERS)
    config = LidarConfig(dedupe_radius_ft=10.0)
    controller = LidarController(None, transformer, config, None)
    assert not controller.is_duplicate(*CENTER)

    from flight.lidar import ScannedObject

    controller.scanned_objects.append(
        ScannedObject(
            center_latlon=CENTER,
            center_field_ft=tuple(transformer.latlon_to_local(*CENTER)),
            vertices_field_ft=[],
            vertices_latlon=[],
            scanned_at=time.time(),
        )
    )
    near = offset_latlon(*CENTER, 45.0, 5.0 / FT_PER_M)  # 5 ft away
    far = offset_latlon(*CENTER, 45.0, 20.0 / FT_PER_M)  # 20 ft away
    assert controller.is_duplicate(*near)
    assert not controller.is_duplicate(*far)


def test_rotate_to_nearest():
    circle = circle_waypoints(*CENTER, 10.0, drone_id=1, num_points=8, closed=True)
    # Drone approaching from the east: nearest vertex is the due-east one (index 2)
    drone_pos = offset_latlon(*CENTER, 90.0, 20.0)
    rotated = rotate_to_nearest(circle, *drone_pos)
    assert len(rotated) == len(circle)
    assert rotated[0] is circle[2]
    assert rotated[-1] is rotated[0]  # still closed
    # Order preserved
    assert rotated[1] is circle[3]


def test_nearest_circle_point():
    drone_pos = offset_latlon(*CENTER, 90.0, 20.0)
    entry = nearest_circle_point(CENTER, 4.0, *drone_pos)
    assert abs(latlon_distance_m(*CENTER, *entry) - 4.0) < 0.05
    assert abs(bearing_deg_between(*CENTER, *entry) - 90.0) < 0.5


def run_tests():
    tests = [value for name, value in globals().items() if name.startswith("test_")]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"\n{len(tests)} tests passed")


if __name__ == "__main__":
    run_tests()
