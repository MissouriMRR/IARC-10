"""
LIDAR obstacle detection and mapping support.

The drone carries two TF Luna 1-D rangefinders (front-left and front-right,
both facing straight ahead) wired to the flight controller, so range data
arrives over MAVLink as DISTANCE_SENSOR messages. Because the sensors are
1-D, a "point cloud" is synthesized from (drone position, heading, range)
at each sample — this module never assumes a scanning lidar.

Pieces:
- MavlinkRangefinderBackend: reads DISTANCE_SENSOR via a dronekit message
  listener (with a vehicle.rangefinder polling fallback).
- LidarController: background proximity monitor that queues a scan
  (scan_pending flag) when an unscanned object comes within threshold,
  plus the store of scanned objects.
- Pure helpers (filtering, center estimation, circle rotation) used by the
  LidarMap state and unit-testable without a drone.
"""

import asyncio
import logging
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol, Sequence

from pymavlink import mavutil

from flight.pathfinding.utils.coord_convert import FT_PER_M, SimToLatLonTransformer
from flight.pathfinding.utils.geo import (
    bearing_deg_between,
    latlon_distance_m,
    offset_latlon,
    point_segment_distance_m,
)
from flight.waypoint import Waypoint

if TYPE_CHECKING:
    import dronekit

# TF Luna spec limits; returns outside this window are discarded
MIN_RANGE_M = 0.2
MAX_RANGE_M = 8.0

SAMPLE_RATE_HZ = 10
# Consecutive in-threshold samples required before a scan is queued (debounce)
TRIGGER_WINDOW = 5
# Time spent hovering at each circle vertex collecting samples
DWELL_S = 1.5
YAW_SPEED_DPS = 30.0
# Extra range beyond the standoff radius accepted during a scan; returns
# farther than this come from something other than the circled object
RANGE_GATE_MARGIN_M = 1.0
# Minimum altitude before the proximity monitor is active, so ground
# clutter during takeoff/landing can't queue scans
MONITOR_MIN_ALT_M = 1.0

# Lateral mounting offset of each sensor from the drone's center, in meters.
# Positive = right of the heading. Keyed by the DISTANCE_SENSOR `id` field.
# -1 is the synthetic id used by the vehicle.rangefinder polling fallback.
SENSOR_LATERAL_OFFSETS_M: dict[int, float] = {
    0: -0.15,  # front-left
    1: 0.15,  # front-right
    -1: 0.0,
}

# MAV_SENSOR_ROTATION_NONE: only forward-facing sensors are ours; a future
# downward altimeter rangefinder must not feed the obstacle detector
FORWARD_ORIENTATION = 0


@dataclass(frozen=True)
class RangeSample:
    """A single rangefinder return paired with the drone pose at sample time."""

    range_m: float
    timestamp: float  # time.monotonic()
    lat: float
    lon: float
    alt: float
    heading_deg: float
    sensor_id: int = -1

    def hit_latlon(self) -> tuple[float, float]:
        """The world coordinate this return came from, accounting for the
        sensor's lateral mounting offset."""
        lateral = SENSOR_LATERAL_OFFSETS_M.get(self.sensor_id, 0.0)
        lat, lon = self.lat, self.lon
        if lateral != 0.0:
            lat, lon = offset_latlon(lat, lon, self.heading_deg + 90.0, lateral)
        return offset_latlon(lat, lon, self.heading_deg, self.range_m)


@dataclass
class ScannedObject:
    """
    The result of circling one obstacle.

    kind distinguishes lidar-mapped obstacles from mines (which come from
    the vision pipeline) so the two are never conflated when this data is
    later merged into the field map.
    """

    center_latlon: tuple[float, float]
    center_field_ft: tuple[float, float]
    vertices_field_ft: list[tuple[float, float]]
    vertices_latlon: list[tuple[float, float]]  # kept for debugging/replay
    scanned_at: float
    kind: str = "obstacle"

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "center_latlon": self.center_latlon,
            "center_field_ft": self.center_field_ft,
            "vertices_field_ft": self.vertices_field_ft,
            "vertices_latlon": self.vertices_latlon,
            "scanned_at": self.scanned_at,
        }


@dataclass
class LidarConfig:
    """Tunable settings, loaded from mission_config's "lidar_config" block."""

    enabled: bool = False
    proximity_threshold_m: float = 6.0
    standoff_radius_m: float = 4.0
    circle_num_points: int = 12
    dedupe_radius_ft: float = 10.0
    max_object_radius_ft: float = 8.0

    @staticmethod
    def from_dict(raw: dict[str, Any] | None) -> "LidarConfig":
        if not raw:
            return LidarConfig()
        return LidarConfig(
            enabled=raw.get("enabled", False),
            proximity_threshold_m=raw.get("proximity_threshold_m", 6.0),
            standoff_radius_m=raw.get("standoff_radius_m", 4.0),
            circle_num_points=raw.get("circle_num_points", 12),
            dedupe_radius_ft=raw.get("dedupe_radius_ft", 10.0),
            max_object_radius_ft=raw.get("max_object_radius_ft", 8.0),
        )


class LidarBackend(Protocol):
    """Source of RangeSamples; swappable so the sim can use a different
    transport than the real TF Lunas if MAVLink forwarding fails."""

    def start(self) -> None: ...

    def latest_all(self) -> list[RangeSample]: ...


class MavlinkRangefinderBackend:
    """
    Reads forward-facing DISTANCE_SENSOR messages via a dronekit message
    listener. Both TF Lunas show up on the same message type with different
    `id` values. Falls back to polling vehicle.rangefinder if no
    DISTANCE_SENSOR message ever arrives (some ArduPilot builds only expose
    the first rangefinder that way).
    """

    # Listener data older than this means the message stream is dead and the
    # polling fallback should engage
    LISTENER_STALE_S = 1.0

    def __init__(self, vehicle: "dronekit.Vehicle") -> None:
        self._vehicle = vehicle
        # sensor id -> latest sample; written from dronekit's receive thread,
        # which is safe because each callback only reassigns one dict slot
        self._latest: dict[int, RangeSample] = {}
        self._last_listener_time: float = 0.0
        self._logged_live_path = False

    def start(self) -> None:
        self._vehicle.add_message_listener("DISTANCE_SENSOR", self._on_distance_sensor)

    def _capture_pose(self) -> tuple[Any, Any, Any, Any]:
        # dronekit reports None for these fields until the first GPS fix
        position = self._vehicle.location.global_relative_frame
        return position.lat, position.lon, position.alt, self._vehicle.heading

    def _on_distance_sensor(self, vehicle: "dronekit.Vehicle", name: str, message: Any) -> None:
        if message.orientation != FORWARD_ORIENTATION:
            return
        range_m = message.current_distance / 100.0  # message field is in cm
        if not MIN_RANGE_M <= range_m <= MAX_RANGE_M or math.isnan(range_m):
            return
        lat, lon, alt, heading = self._capture_pose()
        if lat is None or lon is None:
            return
        self._last_listener_time = time.monotonic()
        if not self._logged_live_path:
            self._logged_live_path = True
            logging.info("Lidar backend: receiving DISTANCE_SENSOR messages")
        self._latest[message.id] = RangeSample(
            range_m=range_m,
            timestamp=self._last_listener_time,
            lat=lat,
            lon=lon,
            alt=alt,
            heading_deg=heading,
            sensor_id=message.id,
        )

    def _poll_rangefinder_fallback(self) -> None:
        distance = getattr(self._vehicle.rangefinder, "distance", None)
        if distance is None or not MIN_RANGE_M <= distance <= MAX_RANGE_M:
            return
        lat, lon, alt, heading = self._capture_pose()
        if lat is None or lon is None:
            return
        if not self._logged_live_path:
            self._logged_live_path = True
            logging.warning(
                "Lidar backend: no DISTANCE_SENSOR messages; polling vehicle.rangefinder"
            )
        self._latest[-1] = RangeSample(
            range_m=distance,
            timestamp=time.monotonic(),
            lat=lat,
            lon=lon,
            alt=alt,
            heading_deg=heading,
            sensor_id=-1,
        )

    def latest_all(self) -> list[RangeSample]:
        if time.monotonic() - self._last_listener_time > self.LISTENER_STALE_S:
            self._poll_rangefinder_fallback()
        return list(self._latest.values())


class LidarController:
    """
    Owns the pending-scan flag, the dedupe list, and the collected objects.
    The background monitor() task only sets scan_pending — it never touches
    flight control. Traversal states consume the flag at the end of a
    waypoint leg and divert into the LidarMap state.
    """

    def __init__(
        self,
        backend: LidarBackend,
        transformer: SimToLatLonTransformer,
        config: LidarConfig,
        vehicle: "dronekit.Vehicle",
    ) -> None:
        self.backend = backend
        self.transformer = transformer
        self.config = config
        self._vehicle = vehicle

        self.scan_pending: bool = False
        self.scan_in_progress: bool = False
        self.pending_center_estimate: tuple[float, float] | None = None
        self.trigger_samples: deque[RangeSample] = deque(maxlen=TRIGGER_WINDOW)
        self.scanned_objects: list[ScannedObject] = []
        # timestamp of the newest sample already consumed, per sensor id
        self._consumed_ts: dict[int, float] = {}

    def new_samples(self) -> list[RangeSample]:
        """Samples that arrived since the last call (per sensor)."""
        fresh = []
        for sample in self.backend.latest_all():
            if sample.timestamp > self._consumed_ts.get(sample.sensor_id, 0.0):
                self._consumed_ts[sample.sensor_id] = sample.timestamp
                fresh.append(sample)
        return fresh

    def is_duplicate(self, lat: float, lon: float) -> bool:
        """Whether an object centered near (lat, lon) has already been scanned."""
        return any(
            latlon_distance_m(lat, lon, *obj.center_latlon) * FT_PER_M
            < self.config.dedupe_radius_ft
            for obj in self.scanned_objects
        )

    @staticmethod
    def estimate_center(samples: Sequence[RangeSample]) -> tuple[float, float]:
        """Mean hit point of the given samples, in lat/lon."""
        hits = [sample.hit_latlon() for sample in samples]
        return (
            sum(hit[0] for hit in hits) / len(hits),
            sum(hit[1] for hit in hits) / len(hits),
        )

    def register_scan(self, scanned: ScannedObject) -> None:
        self.scanned_objects.append(scanned)
        self.clear_pending()
        logging.info(
            "Registered scanned %s with %d vertices: %s",
            scanned.kind,
            len(scanned.vertices_field_ft),
            scanned.to_dict(),
        )

    def clear_pending(self) -> None:
        self.scan_pending = False
        self.pending_center_estimate = None
        self.trigger_samples.clear()

    async def collect(self, duration_s: float) -> list[RangeSample]:
        """Gather every sample that arrives over the next duration_s seconds."""
        collected: list[RangeSample] = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            collected.extend(self.new_samples())
            await asyncio.sleep(1.0 / SAMPLE_RATE_HZ)
        return collected

    async def monitor(self) -> None:
        """
        Background task: watch the range stream and queue a scan when an
        unscanned object comes within the proximity threshold.
        """
        logging.info("Lidar proximity monitor running")
        while True:
            await asyncio.sleep(1.0 / SAMPLE_RATE_HZ)

            if self.scan_in_progress or self.scan_pending:
                continue
            if not self._vehicle.armed:
                continue
            alt = self._vehicle.location.global_relative_frame.alt
            if alt is None or alt < MONITOR_MIN_ALT_M:
                continue

            close_samples = [
                sample
                for sample in self.new_samples()
                if sample.range_m <= self.config.proximity_threshold_m
            ]
            if not close_samples:
                self.trigger_samples.clear()
                continue
            self.trigger_samples.extend(close_samples)
            if len(self.trigger_samples) < TRIGGER_WINDOW:
                continue

            center = self.estimate_center(self.trigger_samples)
            if self.is_duplicate(*center):
                logging.info("Lidar: object near (%.6f, %.6f) already scanned; ignoring", *center)
                self.trigger_samples.clear()
                continue

            self.pending_center_estimate = center
            self.scan_pending = True
            logging.info("Lidar: queued scan of object near (%.6f, %.6f)", *center)


def filter_scan_samples(
    samples: Sequence[RangeSample],
    center_latlon: tuple[float, float],
    standoff_m: float,
    max_object_radius_ft: float,
    transformer: SimToLatLonTransformer,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """
    Reduce raw scan samples to vertices belonging to the circled object,
    rejecting returns from other obstacles.

    A sample is kept iff:
    - range gate: its range is at most standoff_m + RANGE_GATE_MARGIN_M,
      i.e. the return originates from inside the circle being flown;
    - center gate: its hit point is within max_object_radius_ft of the
      object center in field-frame feet.

    Returns
    -------
    tuple[list, list]
        (vertices in field-frame feet, the same vertices in lat/lon).
    """
    center_ft = transformer.latlon_to_local(*center_latlon)
    vertices_ft: list[tuple[float, float]] = []
    vertices_latlon: list[tuple[float, float]] = []
    for sample in samples:
        if sample.range_m > standoff_m + RANGE_GATE_MARGIN_M:
            continue
        hit = sample.hit_latlon()
        hit_ft = transformer.latlon_to_local(*hit)
        if math.dist(hit_ft, center_ft) > max_object_radius_ft:
            continue
        vertices_ft.append((hit_ft[0], hit_ft[1]))
        vertices_latlon.append(hit)
    return vertices_ft, vertices_latlon


def rotate_to_nearest(waypoints: list[Waypoint], lat: float, lon: float) -> list[Waypoint]:
    """
    Rotate a closed circle-waypoint list so it starts at the vertex nearest
    (lat, lon), preserving traversal order. Keeps the drone from flying
    across the object to reach the circle's due-north start point.
    """
    points = waypoints[:-1] if len(waypoints) > 1 and waypoints[0] is waypoints[-1] else waypoints
    nearest = min(
        range(len(points)),
        key=lambda i: latlon_distance_m(lat, lon, points[i].lat, points[i].long),
    )
    rotated = points[nearest:] + points[:nearest]
    if points is not waypoints:  # re-close the loop
        rotated.append(rotated[0])
    return rotated


def nearest_circle_point(
    center: tuple[float, float], radius_m: float, lat: float, lon: float
) -> tuple[float, float]:
    """The point on the circle around center closest to (lat, lon)."""
    bearing = bearing_deg_between(center[0], center[1], lat, lon)
    return offset_latlon(center[0], center[1], bearing, radius_m)


def lidar_approach_is_safe(controller: LidarController, vehicle: "dronekit.Vehicle") -> bool:
    """
    Whether flying from the current position to the pending object's circle
    is clear of everything we know about.

    Checks the straight-line approach segment against all previously scanned
    object centers (unknown obstacles cannot be checked — the standoff radius
    being well under the sensor range bounds that exposure), and requires the
    circle entry point to be inside the field boundary.
    """
    center = controller.pending_center_estimate
    if center is None:
        return False

    position = vehicle.location.global_relative_frame
    entry = nearest_circle_point(
        center, controller.config.standoff_radius_m, position.lat, position.lon
    )
    segment_start = (position.lat, position.lon)

    for scanned in controller.scanned_objects:
        clearance = point_segment_distance_m(scanned.center_latlon, segment_start, entry)
        if clearance < controller.config.standoff_radius_m:
            logging.info(
                "Lidar: approach passes within %.1f m of scanned object at %s; skipping",
                clearance,
                scanned.center_field_ft,
            )
            return False

    # The circle entry point must be inside the field boundary
    corners_local = controller.transformer.get_arb_corners()
    xs = [corner[0] for corner in corners_local]
    ys = [corner[1] for corner in corners_local]
    entry_x, entry_y = controller.transformer.latlon_to_local(*entry)
    if not (min(xs) <= entry_x <= max(xs) and min(ys) <= entry_y <= max(ys)):
        logging.info(
            "Lidar: circle entry point (%.1f, %.1f) ft is off-field; skipping", entry_x, entry_y
        )
        return False

    return True


def condition_yaw(vehicle: "dronekit.Vehicle", heading_deg: float) -> None:
    """
    Command the drone to yaw to an absolute compass heading (GUIDED mode).
    Requires WP_YAW_BEHAVIOR=0 for the heading to hold through subsequent
    simple_goto calls.
    """
    message = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading_deg,  # target heading
        YAW_SPEED_DPS,  # yaw speed
        0,  # direction: shortest
        0,  # 0 = absolute heading
        0,
        0,
        0,
    )
    vehicle.send_mavlink(message)
