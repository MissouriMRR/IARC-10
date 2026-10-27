"""
Generate latitude/longitude waypoints in a circle around a center point.

Uses the equirectangular approximation with proper Earth radius math.
For radii up to ~100km this is accurate to within a few meters.
"""

import math
from typing import List, Tuple
from flight.waypoint import POSITION_UNCERTAINTY_M, Waypoint

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius in meters


def _degrees_per_meter(center_lat: float) -> Tuple[float, float]:
    """Degrees of latitude and of longitude per meter at this latitude."""
    deg_lat_per_m = 1.0 / (math.pi * EARTH_RADIUS_M / 180.0)
    deg_lon_per_m = 1.0 / (math.pi * EARTH_RADIUS_M * math.cos(math.radians(center_lat)) / 180.0)
    return deg_lat_per_m, deg_lon_per_m


def point_at_phase(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    phase_rad: float,
    start_bearing_deg: float = 0.0,
) -> Tuple[float, float]:
    """The point on the circle `phase_rad` around from `start_bearing_deg`.

    Phase runs the same way the waypoint indices do — clockwise from the start
    bearing — so waypoint `i` of `num_points` sits at `2*pi*i / num_points`.
    Flying a circle continuously means commanding points *between* waypoints, so
    the index-based generator below is just this function sampled at those angles.

    Returns:
        (latitude, longitude) in decimal degrees, longitude normalized to
        [-180, 180].
    """
    deg_lat_per_m, deg_lon_per_m = _degrees_per_meter(center_lat)

    bearing = math.radians(start_bearing_deg) + phase_rad
    lat = center_lat + radius_m * math.cos(bearing) * deg_lat_per_m
    lon = center_lon + radius_m * math.sin(bearing) * deg_lon_per_m

    return lat, ((lon + 180.0) % 360.0) - 180.0


def phase_of(
    center_lat: float,
    center_lon: float,
    lat: float,
    lon: float,
    start_bearing_deg: float = 0.0,
) -> float:
    """Where a position sits around the circle, in radians in [0, 2*pi).

    The inverse of `point_at_phase`, and exact on its output: it undoes the same
    per-meter scaling rather than approximating it, so `phase_of` on waypoint `i`
    returns exactly `2*pi*i / num_points`. Radius is irrelevant — only the
    direction from the center matters, which is what makes this usable on a
    measured position that is never exactly on the circle.
    """
    deg_lat_per_m, deg_lon_per_m = _degrees_per_meter(center_lat)

    # Back out of degrees into the meter frame the bearing was built in.
    d_north = (lat - center_lat) / deg_lat_per_m
    # Normalize the longitude difference so a position across the antimeridian
    # from the center doesn't read as most of a lap away.
    d_east = (((lon - center_lon) + 180.0) % 360.0 - 180.0) / deg_lon_per_m

    # atan2(east, north) is the compass bearing from the center; subtracting the
    # start bearing turns it into phase.
    bearing = math.atan2(d_east, d_north)
    return (bearing - math.radians(start_bearing_deg)) % (2.0 * math.pi)


class CircleProgress:
    """Monotonic progress around a circle, unwrapped across laps.

    `phase_of` only ever answers in [0, 2*pi), so on its own it cannot tell lap 0
    index 23 from lap 9 index 23 — and the lockstep barrier is expressed in
    global indices spanning every lap. This adds the missing lap count.

    Progress is reported as `lap * 2*pi + measured phase`, **not** as a running
    sum of per-tick deltas. Only the integer lap count is accumulated state, so
    position noise cannot bank into it. Summing deltas and clamping the negative
    ones — the obvious implementation — is biased: it keeps every noise-positive
    step and discards the noise-negative ones, so reported progress creeps ahead
    of the truth and the drone broadcasts waypoints it has not reached.

    Noise gets one more guard. A sample further round than the drone could
    physically have flown since the last accepted one is a glitch, not flight,
    and is dropped without updating any state. The elapsed time is measured from
    the last *accepted* sample, not from the last tick — otherwise one rejection
    makes the next delta look twice as impossible and rejection cascades until it
    happens to recover.

    That guard has to budget for the noise as well as the motion, or it throws
    away the very data it exists to protect: a threshold covering travel alone
    sits about one standard deviation from the mean at real-GPS accuracy and
    discards 13% of perfectly good fixes at 0.25 m noise, 21% at 0.35 m. Each
    rejection freezes the reported phase for a tick, and a formation flying on
    stale phases falls apart. Rejection is cheap to be generous with, because a
    single wrong sample cannot bank anything — only the lap count persists, and
    that needs an error near half a lap to move.

    Progress is reported in fractional waypoint indices, so 3.5 means "half a leg
    past waypoint 3", which is what the governor and the crossing detector want.
    Noise makes it dither, so it is not monotonic; latching that into one-way
    progress is the crossing detector's job.
    """

    def __init__(
        self,
        center_lat: float,
        center_lon: float,
        radius_m: float,
        points_per_lap: int,
        start_bearing_deg: float = 0.0,
        max_speed_m_s: float = 2.0,
        start_index: int = 0,
        position_noise_m: float = POSITION_UNCERTAINTY_M,
    ) -> None:
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.radius_m = radius_m
        self.points_per_lap = points_per_lap
        self.start_bearing_deg = start_bearing_deg
        # Generous: this rejects teleports, not brisk flying. Too tight and a
        # legitimately fast drone stops making progress at all.
        self.max_speed_m_s = max_speed_m_s * 2.0
        # Two fixes go into every delta, so the allowance covers both, several
        # deviations out.
        self.noise_allowance_m = 4.0 * position_noise_m
        self.start_index = start_index

        self._lap: int = 0
        self._phase: float = 2.0 * math.pi * start_index / points_per_lap % (2.0 * math.pi)
        self._seeded: bool = False
        self._since_accepted: float = 0.0
        self.rejected: int = 0

    @property
    def phase(self) -> float:
        """Current angle around the circle, in [0, 2*pi), lap discarded."""
        return self._phase

    @property
    def radians(self) -> float:
        """Total angle travelled since the tracker was seeded."""
        return self._lap * 2.0 * math.pi + self._phase

    @property
    def index(self) -> float:
        """Progress as a fractional global waypoint index."""
        return self.radians * self.points_per_lap / (2.0 * math.pi)

    def update(self, lat: float, lon: float, dt: float) -> float:
        """Fold a measured position in, and return the new fractional index.

        `dt` is the time since the previous sample, used only to bound how far
        the drone could have travelled.
        """
        phase = phase_of(self.center_lat, self.center_lon, lat, lon, self.start_bearing_deg)

        if not self._seeded:
            # Seed onto whichever lap puts us nearest the index we were told we
            # are starting from. Without this a drone sitting on index 0 with a
            # sliver of noise on the wrong side of the seam reads as index 23.
            self._seeded = True
            self._phase = phase
            want = 2.0 * math.pi * self.start_index / self.points_per_lap
            self._lap = round((want - phase) / (2.0 * math.pi))
            return self.index

        self._since_accepted += max(dt, 0.0)

        # Shortest way round from the last accepted phase, so crossing the
        # 0/2*pi seam reads as a small step rather than a whole lap.
        delta = (phase - self._phase + math.pi) % (2.0 * math.pi) - math.pi

        plausible = (
            self.max_speed_m_s * self._since_accepted + self.noise_allowance_m
        ) / self.radius_m
        if abs(delta) > plausible:
            self.rejected += 1
            return self.index

        # A forward step across the seam lands on a phase smaller than the last;
        # the sign of delta says which way we went, so the lap count follows it.
        if delta > 0.0 and phase < self._phase:
            self._lap += 1
        elif delta < 0.0 and phase > self._phase:
            self._lap -= 1

        self._phase = phase
        self._since_accepted = 0.0
        return self.index

    def point_at(self, index_ahead: float) -> Tuple[float, float]:
        """The point on the circle `index_ahead` fractional indices past here."""
        phase = (self.index + index_ahead) * 2.0 * math.pi / self.points_per_lap
        return point_at_phase(
            self.center_lat,
            self.center_lon,
            self.radius_m,
            phase,
            self.start_bearing_deg,
        )


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

    waypoints: List[Tuple[float, float]] = []
    for i in range(num_points):
        lat, lon = point_at_phase(
            center_lat,
            center_lon,
            radius_m,
            2.0 * math.pi * i / num_points,
            start_bearing_deg,
        )
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
