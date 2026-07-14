"""
LIDAR mapping pipeline simulation.

Simulates a drone circling a square obstacle with two forward-facing 1-D
rangefinders (TF Luna style), runs the returns through the real
filter/convert pipeline from flight/lidar.py, and plots the collected
vertices over the field.

A second obstacle sits just outside the scan circle to demonstrate that
its returns are filtered out ("ignore other obstacles").

Run with:   uv run python -m flight.pathfinding.test_lidar_vertices
"""

import math
import time

import matplotlib.pyplot as plt

from flight.lidar import RangeSample, filter_scan_samples
from flight.pathfinding.utils.coord_convert import FT_PER_M, field_transformer_ft
from flight.pathfinding.utils.geo import bearing_deg_between

"""
Config
"""
# Football-field test corners from coord_convert.py's test case
FIELD_CORNERS = [
    {"lat": 36.021683, "lon": -95.941831},  # 1
    {"lat": 36.020694, "lon": -95.941856},  # 2
    {"lat": 36.021694, "lon": -95.942372},  # 3 (origin)
    {"lat": 36.020703, "lon": -95.942397},  # 4
]

OBJECT_CENTER_FT = (120.0, 60.0)  # field-frame feet
OBJECT_HALF_SIZE_FT = 2.0  # 4 ft x 4 ft square obstacle
OTHER_OBSTACLE_FT = (150.0, 60.0)  # a different obstacle beyond the circle
OTHER_HALF_SIZE_FT = 3.0

STANDOFF_RADIUS_M = 4.0
CIRCLE_NUM_POINTS = 12
SAMPLES_PER_STOP = 10  # samples collected while dwelling at each stop
MAX_OBJECT_RADIUS_FT = 8.0
SENSOR_OFFSETS_FT = {0: -0.15 * FT_PER_M, 1: 0.15 * FT_PER_M}

"""
Synthetic ray casting (field-frame feet)
"""


def square_edges(center, half):
    x, y = center
    corners = [
        (x - half, y - half),
        (x + half, y - half),
        (x + half, y + half),
        (x - half, y + half),
    ]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


EDGES = square_edges(OBJECT_CENTER_FT, OBJECT_HALF_SIZE_FT) + square_edges(
    OTHER_OBSTACLE_FT, OTHER_HALF_SIZE_FT
)


def ray_cast(origin, direction):
    """Distance in feet from origin along direction to the nearest edge, or None."""
    ox, oy = origin
    dx, dy = direction
    best = None
    for (ax, ay), (bx, by) in EDGES:
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue
        t = ((ax - ox) * ey - (ay - oy) * ex) / denom
        s = ((ax - ox) * dy - (ay - oy) * dx) / denom
        if t > 0 and 0.0 <= s <= 1.0 and (best is None or t < best):
            best = t
    return best


"""
Simulation: fly the circle, point at the center, sample both sensors
"""

transformer = field_transformer_ft(FIELD_CORNERS)
center_latlon = transformer.local_to_latlon(*OBJECT_CENTER_FT)

standoff_ft = STANDOFF_RADIUS_M * FT_PER_M
samples: list[RangeSample] = []
drone_track = []

for i in range(CIRCLE_NUM_POINTS):
    angle = 2.0 * math.pi * i / CIRCLE_NUM_POINTS
    drone_ft = (
        OBJECT_CENTER_FT[0] + standoff_ft * math.cos(angle),
        OBJECT_CENTER_FT[1] + standoff_ft * math.sin(angle),
    )
    drone_track.append(drone_ft)
    drone_latlon = transformer.local_to_latlon(*drone_ft)
    heading = bearing_deg_between(*drone_latlon, *center_latlon)

    # Unit vector pointing at the object center, and its right-hand normal
    to_center = (OBJECT_CENTER_FT[0] - drone_ft[0], OBJECT_CENTER_FT[1] - drone_ft[1])
    norm = math.hypot(*to_center)
    look = (to_center[0] / norm, to_center[1] / norm)
    right = (look[1], -look[0])

    for j in range(SAMPLES_PER_STOP):
        # Heading jitter while hovering, like a real dwell; wide enough that
        # some rays miss the target square and hit the other obstacle
        jitter = math.radians((j - SAMPLES_PER_STOP / 2) * 5.0)
        direction = (
            look[0] * math.cos(jitter) - look[1] * math.sin(jitter),
            look[0] * math.sin(jitter) + look[1] * math.cos(jitter),
        )
        for sensor_id, lateral_ft in SENSOR_OFFSETS_FT.items():
            origin = (drone_ft[0] + right[0] * lateral_ft, drone_ft[1] + right[1] * lateral_ft)
            hit_ft = ray_cast(origin, direction)
            if hit_ft is None:
                continue
            range_m = hit_ft / FT_PER_M
            origin_latlon = transformer.local_to_latlon(*origin)
            samples.append(
                RangeSample(
                    range_m=range_m,
                    timestamp=time.monotonic(),
                    lat=origin_latlon[0],
                    lon=origin_latlon[1],
                    alt=5.0,
                    # Compass bearings increase clockwise; the field-frame
                    # rotation above is counterclockwise, hence the sign flip
                    heading_deg=heading - math.degrees(jitter),
                    # sensor offset already applied to the origin here, so the
                    # pipeline must not re-apply it; -1 has zero offset
                    sensor_id=-1,
                )
            )

vertices_ft, _ = filter_scan_samples(
    samples, tuple(center_latlon), STANDOFF_RADIUS_M, MAX_OBJECT_RADIUS_FT, transformer
)

print(f"Scan stops:          {CIRCLE_NUM_POINTS}")
print(f"Raw samples:         {len(samples)}")
print(f"Accepted vertices:   {len(vertices_ft)}")
rejected = len(samples) - len(vertices_ft)
print(f"Rejected (other obstacles / out of gate): {rejected}")

"""
Visualization
"""

fig, ax = plt.subplots(figsize=(8, 8))

for edges, color, label in (
    (square_edges(OBJECT_CENTER_FT, OBJECT_HALF_SIZE_FT), "black", "scanned object"),
    (square_edges(OTHER_OBSTACLE_FT, OTHER_HALF_SIZE_FT), "gray", "other obstacle"),
):
    for k, ((ax1, ay1), (bx1, by1)) in enumerate(edges):
        ax.plot([ax1, bx1], [ay1, by1], color=color, label=label if k == 0 else None)

track_x = [p[0] for p in drone_track] + [drone_track[0][0]]
track_y = [p[1] for p in drone_track] + [drone_track[0][1]]
ax.plot(track_x, track_y, "b--", alpha=0.5, label="scan circle")
ax.scatter(track_x[:-1], track_y[:-1], c="blue", s=25, zorder=3)

all_hits = [transformer.latlon_to_local(*s.hit_latlon()) for s in samples]
ax.scatter(
    [h[0] for h in all_hits],
    [h[1] for h in all_hits],
    c="lightcoral",
    s=12,
    label="rejected returns",
    zorder=2,
)
ax.scatter(
    [v[0] for v in vertices_ft],
    [v[1] for v in vertices_ft],
    c="green",
    s=25,
    label="accepted vertices",
    zorder=4,
)
ax.scatter(*OBJECT_CENTER_FT, c="black", marker="x", s=60, label="object center", zorder=5)

ax.set_aspect("equal")
ax.set_xlabel("field x (ft)")
ax.set_ylabel("field y (ft)")
ax.set_title("LIDAR mapping — collected vertices (field frame)")
ax.legend(loc="upper left", fontsize=8)

fig.savefig("latest_test_lidar_vertices.jpeg", dpi=120)
print("\nSaved latest_test_lidar_vertices.jpeg")
plt.show()
