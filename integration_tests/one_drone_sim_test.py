"""
Deep-analysis testbench: drives the REAL state machine (Start -> Takeoff ->
CalcScanPath -> Scan -> DroneShare -> ... -> ExpandNodes -> EndRun ->
AppShare -> Recall -> Land) through a simulated one-drone
mission, to answer "does a one-drone configuration actually work" by
running it, not just reading it.

Only the hardware boundary is faked:
  - dronekit/picamera2 don't exist in this dev environment at all (see
    _hardware_shims.py) -- a minimal shim lets the real modules import.
  - MockVehicle stands in for dronekit.Vehicle. Its position comes from a
    Motion model (vehicle_motion.py): TeleportMotion snaps to every goto
    (the default here: every polling loop converges on its first check),
    CruiseMotion flies a straight line over real time (the multi-drone test
    uses it, so peers see each other in transit). The goal is exercising the
    real flight-control CODE PATHS, not real flight dynamics.
  - MockCamera subclasses vision.Cameras.baseCamera.BaseCamera directly,
    so get_image_corner_coordinates/get_pixel_coordinate -- the real FOV/
    mount-offset/ray-casting geometry scan_impl.py and drone_share_impl.py
    actually run -- are the REAL implementation. Only capture_image/
    capture_and_detect_mines (the actual camera hardware boundary) are
    faked, checking a ground-truth minefield against the real computed
    footprint polygon.
  - MockInterdrone replaces the real Interdrone class outright (not just
    its transport) -- the real one spins up an actual networking thread
    (sockets, its own event loop) at construction, which is orthogonal to
    what this test validates and adds real-world fragility for zero one-
    drone-specific value (drone_states is empty either way for a solo
    mission). A background "autopilot" task drives it through the ARM ->
    MISSION handshake the way an operator/app normally would.

Run directly: `python integration_tests/one_drone_sim_test.py`
"""

import asyncio
import logging
import math
import sys
import time
import types

from integration_tests import _hardware_shims

_hardware_shims.install()

from PIL import Image as PILImage

import dronekit

from vision.Cameras.baseCamera import BaseCamera
from vision.common.detection import Detection
from vision.common.drone_coordinates import DronePose, GimbalPose
from vision.common.image import Image as VisionImage

from flight.pathfinding.iarc_report import format_iarc_path

from integration_tests.vehicle_motion import Fix, Motion, TeleportMotion

from state_machine.drone import Drone, RANGEFINDER3_MAVLINK_ID
from state_machine.flight_settings import FlightSettings, SimMode
from state_machine.interdrone import CMD_MSG
from state_machine.state_machine import StateMachine
from state_machine.states import Start
import state_machine.states.impl.takeoff_impl as takeoff_impl_module
import vision.Cameras.RPICamera.RPICamera as rpicamera_module

# ======================================================================
# Ground-truth field/mine setup -- same equirectangular-approximation
# pattern flight/pathfinding/tests/droneWorkflowTest.py's own
# _field_corners() uses, kept self-contained here rather than importing
# that (matplotlib-heavy) module for one helper.
# ======================================================================
_BASE_LAT, _BASE_LON = 36.0, -95.9
_M_PER_LAT = 111320.0
_M_PER_LON = 111320.0 * math.cos(math.radians(_BASE_LAT))
_FT_TO_M = 0.3048
# The simulated field, in feet: 80 x 300 (the IARC field).
WIDTHOFFIELD = 80.0
HEIGHTOFFIELD = 300.0


def _local_to_latlon(x_ft: float, y_ft: float) -> tuple[float, float]:
    lat = _BASE_LAT + (y_ft * _FT_TO_M) / _M_PER_LAT
    lon = _BASE_LON + (x_ft * _FT_TO_M) / _M_PER_LON
    return lat, lon


def _field_corners() -> tuple[tuple[float, float], ...]:
    c1 = _local_to_latlon(0, 0)
    c2 = _local_to_latlon(WIDTHOFFIELD, 0)
    c3 = _local_to_latlon(WIDTHOFFIELD, HEIGHTOFFIELD)
    c4 = _local_to_latlon(0, HEIGHTOFFIELD)
    return (c1, c2, c3, c4)


def _point_in_latlon_polygon(lat: float, lon: float, polygon: list[tuple[float, float]]) -> bool:
    """Crossing-number point-in-polygon test, operating directly on
    (lat, lon) as if they were planar (x, y) -- fine at the scale of a
    single camera footprint (a few meters across), where lat/lon's
    equirectangular distortion is negligible."""
    inside = False
    n = len(polygon)
    for i in range(n):
        lat1, lon1 = polygon[i]
        lat2, lon2 = polygon[(i + 1) % n]
        if (lat1 > lat) != (lat2 > lat):
            lon_intersect = lon1 + (lat - lat1) * (lon2 - lon1) / (lat2 - lat1 + 1e-15)
            if lon < lon_intersect:
                inside = not inside
    return inside


# ======================================================================
# MockVehicle -- see module docstring.
# ======================================================================
class _Location:
    def __init__(self, lat: float, lon: float, alt: float):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class _LocationBundle:
    def __init__(self, fix: Fix):
        self.global_relative_frame = _Location(fix.lat, fix.lon, fix.alt)
        self.global_frame = _Location(fix.lat, fix.lon, fix.alt)


class _Attitude:
    def __init__(self, pitch: float = 0.0, yaw: float = 0.0, roll: float = 0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class MockVehicle:
    """dronekit.Vehicle stand-in. Where it is comes from its Motion (vehicle_motion.py):
    TeleportMotion (the default; every goto lands at once) or CruiseMotion (flies there over
    time, so peers see it in transit). `location` is read fresh from the motion each time.

    Setting mode to RTL/LAND puts it on the ground and disarms at once -- Land.return_to_launch
    and EmergencyLand both poll for exactly that outcome, and this mock has no descent
    dynamics to produce it otherwise."""

    def __init__(self, start_lat: float, start_lon: float, motion: Motion | None = None):
        self.motion: Motion = motion or TeleportMotion(start_lat, start_lon)
        self.attitude = _Attitude()
        self.home_location = _Location(start_lat, start_lon, 0.0)
        self._mode = dronekit.VehicleMode("STABILIZE")
        self.armed = False
        self.is_armable = True
        self.airspeed = 0
        self.system_status = types.SimpleNamespace(state="ACTIVE")
        self.parameters: dict = {}
        self._listeners: dict[str, list] = {}

    @property
    def location(self) -> _LocationBundle:
        return _LocationBundle(self.motion.position())

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value) -> None:
        self._mode = value
        if value.name in ("RTL", "LAND"):
            self.motion.touch_down()
            self.armed = False

    def simple_goto(self, location, groundspeed: float | None = None) -> None:
        alt = location.alt if location.alt is not None else self.motion.position().alt
        self.motion.goto(location.lat, location.lon, alt, groundspeed)

    def simple_takeoff(self, alt: float) -> None:
        here = self.motion.position()
        self.motion.goto(here.lat, here.lon, alt, None)

    def add_message_listener(self, name: str, fn) -> None:
        self._listeners.setdefault(name, []).append(fn)

    def fire_message(self, name: str, message) -> None:
        for fn in self._listeners.get(name, []):
            fn(self, name, message)

    def close(self) -> None:
        pass


class _FakeDistanceSensorMessage:
    def __init__(self, current_distance_cm: float, sensor_id: int):
        self.current_distance = current_distance_cm
        self.id = sensor_id


async def _rangefinder_feed(vehicle: MockVehicle) -> None:
    """Keeps Drone.rangefinder_altitude_agl_m fresh (RANGEFINDER_STALE_S is
    1.0s) by periodically firing a synthetic DISTANCE_SENSOR message
    reflecting the mock vehicle's own current (teleported) altitude."""
    while True:
        alt_m = vehicle.location.global_relative_frame.alt
        vehicle.fire_message(
            "DISTANCE_SENSOR",
            _FakeDistanceSensorMessage(current_distance_cm=alt_m * 100.0, sensor_id=RANGEFINDER3_MAVLINK_ID),
        )
        await asyncio.sleep(0.3)


# ======================================================================
# MockCamera -- see module docstring.
# ======================================================================
_SIM_CONTEXT: dict = {}


class TruthCamera(BaseCamera):
    """A fake camera and mine detector over a ground-truth minefield (one- and multi-drone).

    - Capture: a blank image; the footprint is the real camera geometry (BaseCamera).
    - Detect: each mine inside the footprint is reported ONCE per camera, boxed at the
      pixel where it really lies in the frame.
    - Locate: get_pixel_coordinate gives such a pixel's mine its exact position (a perfect
      localiser), so every drone that sees a mine reports the same spot.
    """

    IMAGE_SIZE = (640, 480)

    def __init__(self, vision_config: dict, pose_provider, true_mines_latlon):
        mount = vision_config.get("cameraMountRotationDeg", {})
        super().__init__(
            h_fov_deg=vision_config.get("hFovDeg", 0.0),
            v_fov_deg=vision_config.get("vFovDeg", 0.0),
            offset=tuple(vision_config.get("cameraOffsetM", (0.0, 0.0, 0.0))),
            mount_rotation=GimbalPose(
                yaw=mount.get("yaw", 0.0), pitch=mount.get("pitch", 0.0), roll=mount.get("roll", 0.0)
            ),
        )
        self._pose_provider = pose_provider
        self._true_mines_latlon: list[tuple[float, float]] = true_mines_latlon
        self._reported: set[int] = set()
        self._located: dict[tuple[float, float], tuple[float, float]] = {}   # pixel -> mine
        self.capture_count = 0

    def initialize_camera(self) -> None:
        pass

    def capture_image(self, only_metadata: bool) -> VisionImage:
        self.capture_count += 1
        return VisionImage(PILImage.new("RGB", self.IMAGE_SIZE), {})

    def get_pixel_coordinate(self, px, py, image_width, image_height, drone):
        mine = self._located.get((px, py))
        if mine is not None:
            return mine
        return super().get_pixel_coordinate(px, py, image_width, image_height, drone)

    @staticmethod
    def _pixel_of(point, corners, width, height):
        """Pixel of a ground point inside the footprint (corners: top-left, top-right,
        bottom-left, bottom-right in lat/lon; a nadir footprint is a parallelogram)."""
        (tl_lat, tl_lon), (tr_lat, tr_lon), (bl_lat, bl_lon), _br = corners
        ax, ay = tr_lat - tl_lat, tr_lon - tl_lon
        bx, by = bl_lat - tl_lat, bl_lon - tl_lon
        dx, dy = point[0] - tl_lat, point[1] - tl_lon
        det = ax * by - ay * bx
        u, v = (dx * by - dy * bx) / det, (ax * dy - ay * dx) / det
        return u * (width - 1), v * (height - 1)

    def capture_and_detect_mines(self) -> list[Detection]:
        pose: DronePose = self._pose_provider()
        width, height = self.IMAGE_SIZE
        corners = self.get_image_corner_coordinates(width, height, pose)
        if any(c is None for c in corners):
            return []
        top_left, top_right, bottom_left, bottom_right = corners
        polygon = [top_left, top_right, bottom_right, bottom_left]

        detections = []
        for i, mine in enumerate(self._true_mines_latlon):
            if i in self._reported or not _point_in_latlon_polygon(*mine, polygon):
                continue
            self._reported.add(i)
            px, py = self._pixel_of(mine, corners, width, height)
            self._located[(px, py)] = mine
            detections.append(Detection(0.92, (px, py, 20, 20), (width, height)))
        return detections

    def capture_and_detect_apriltags(self) -> list[Detection]:
        return []


class MockCamera(TruthCamera):
    """The one-drone test's camera: pose and mines from the module-level _SIM_CONTEXT."""

    def __init__(self, vision_config: dict):
        super().__init__(vision_config, _SIM_CONTEXT["pose_provider"],
                         _SIM_CONTEXT["true_mines_latlon"])


def _make_pose_provider(vehicle: MockVehicle):
    def _pose() -> DronePose:
        loc = vehicle.location.global_relative_frame
        att = vehicle.attitude
        return DronePose(
            lat=loc.lat,
            lon=loc.lon,
            altitude=loc.alt,
            yaw=math.degrees(att.yaw),
            pitch=math.degrees(att.pitch),
            roll=math.degrees(att.roll),
        )

    return _pose


# ======================================================================
# MockInterdrone -- see module docstring.
# ======================================================================
class MockInterdrone:
    def __init__(self, flight_settings: FlightSettings, drone: Drone):
        self.flight_settings = flight_settings
        self.drone = drone
        self.drone_states: list = []
        self.cmd_msg: CMD_MSG = CMD_MSG.NONE
        self.current_state_name: str | None = None
        self._restart_callback = None

    def get_cmd_msg(self) -> CMD_MSG:
        return self.cmd_msg

    def set_cmd_msg(self, cmd_msg: CMD_MSG) -> None:
        self.cmd_msg = cmd_msg

    def register_state_machine(self, callback) -> None:
        self._restart_callback = callback

    def update_task(self, task) -> None:
        pass

    def update_state(self, state) -> None:
        self.current_state_name = type(state).__name__

    async def report_own_position(self) -> None:
        pass

    async def announce_landing(self) -> None:
        pass

    async def ping_drones(self, timeout_sec: float = 2.0) -> bool:
        return True

    async def all_armed(self) -> bool:
        return True

    async def all_takeoff(self) -> bool:
        return True

    async def all_demo_start(self) -> bool:
        return True

    async def all_mission_start(self) -> bool:
        return True

    async def send_ARM(self, ids) -> None:
        pass

    async def send_arm_ack(self) -> None:
        pass

    async def send_start_mission(self, ids) -> None:
        pass

    async def send_mission_ack(self) -> None:
        pass

    async def send_start_demo(self, ids) -> None:
        pass

    async def send_demo_ack(self) -> None:
        pass

    async def send_takeoff(self, ids) -> None:
        pass

    async def send_takeoff_ack(self) -> None:
        pass

    async def push_mission_path(self) -> None:
        # AppShare's own final push (Interdrone.push_mission_path, added
        # once SEND_PATHS_TO_APP became a real proactive push, not just a
        # REQUEST_MAP_DATA reply) -- a no-op here, same reasoning as every
        # other send_* stub on this class: this test validates state-
        # machine/pathfinder behavior, not interdrone wire traffic.
        pass


async def _autopilot(interdrone: MockInterdrone) -> None:
    """Stands in for an operator/app: arms immediately, then starts the
    mission once (and only once) Takeoff is actually polling for it --
    setting cmd_msg straight to MISSION up front would race Start's own
    level-triggered `while get_cmd_msg() != ARM` check, which could then
    never observe ARM at all if it happened to check after the flip."""
    interdrone.set_cmd_msg(CMD_MSG.ARM)
    while interdrone.current_state_name != "Takeoff":
        await asyncio.sleep(0.02)
    interdrone.set_cmd_msg(CMD_MSG.MISSION)


# ======================================================================
# Driver
# ======================================================================
async def run_simulation(
    true_mines_local: list[tuple[float, float]], timeout_s: float = 240.0
):
    corners = _field_corners()
    start_lat, start_lon = _local_to_latlon(WIDTHOFFIELD / 2, 0)
    true_mines_latlon = [_local_to_latlon(x, y) for x, y in true_mines_local]

    flight_settings = FlightSettings(
        drone_ID=1,
        drones_in_mission=[1],
        mission_corners=[{"lat": c[0], "lon": c[1]} for c in corners],
        max_height=30,
        start_coord={"lat": start_lat, "lon": start_lon},
        mission_type="Automatic",
        sim_mode=SimMode.SIM,
    )

    drone = Drone()
    drone.id = 1
    drone.use_settings(flight_settings.sim_mode)
    vehicle = MockVehicle(start_lat, start_lon)
    drone._vehicle = vehicle
    vehicle.add_message_listener("DISTANCE_SENSOR", drone._on_distance_sensor)

    _SIM_CONTEXT["pose_provider"] = _make_pose_provider(vehicle)
    _SIM_CONTEXT["true_mines_latlon"] = true_mines_latlon

    # Takeoff.run() now imports RPICamera locally (inside the MISSION
    # branch, not at module level -- see takeoff_impl.py) so that the real
    # picamera2 stack is only ever pulled in on hardware that has it.
    # Patching takeoff_impl_module.RPICamera would no-op against that local
    # import -- it re-resolves the name from RPICamera's own home module at
    # call time, so the patch has to land there instead.
    rpicamera_module.RPICamera = MockCamera

    interdrone = MockInterdrone(flight_settings, drone)
    machine = StateMachine(
        Start(drone, flight_settings, interdrone), drone, flight_settings, interdrone
    )

    autopilot_task = asyncio.ensure_future(_autopilot(interdrone))
    rangefinder_task = asyncio.ensure_future(_rangefinder_feed(vehicle))

    error = None
    started_at = time.monotonic()
    try:
        await asyncio.wait_for(machine.run(), timeout=timeout_s)
    except asyncio.TimeoutError:
        error = f"timed out after {timeout_s:.0f}s (last state: {interdrone.current_state_name})"
    except Exception as exc:  # noqa: BLE001 -- report whatever broke, don't hide it
        logging.exception("state machine raised")
        error = exc
    finally:
        elapsed = time.monotonic() - started_at
        autopilot_task.cancel()
        rangefinder_task.cancel()

    return {
        "drone": drone,
        "vehicle": vehicle,
        "interdrone": interdrone,
        "true_mines_latlon": true_mines_latlon,
        "error": error,
        "elapsed_s": elapsed,
    }


def check_reported_route(planner, mission_path) -> list[str]:
    """Problems with a reported route (empty if none): it must be continuous in the
    strict IARC format, run from one field edge to the other, and avoid every mine this
    planner knows about."""
    if not mission_path:
        return ["no route reported"]
    problems = []
    pts = [planner.latlon_to_local(lat, lon) for lat, lon in mission_path]
    try:
        format_iarc_path(pts, strict=True, field_w=planner.field_w, field_h=planner.field_h)
    except ValueError as exc:
        problems.append(f"not continuous: {exc}")
    ys = sorted((pts[0][1], pts[-1][1]))
    if ys[0] > 0.01 or abs(ys[1] - planner.field_h) > 0.01:
        problems.append(f"does not span the field: ends at y={pts[0][1]:.2f} and y={pts[-1][1]:.2f}")
    if planner.backend.crosses(pts):
        problems.append("crosses a known mine")
    return problems


def _report(result: dict) -> bool:
    """Prints the report. Returns True iff the run completed (and, path-node, the reported
    route checks out)."""
    print()
    print("=" * 70)
    print("ONE-DRONE STATE MACHINE SIMULATION -- REPORT")
    print("=" * 70)
    print(f"final state observed : {result['interdrone'].current_state_name}")
    print(f"elapsed (wall clock) : {result['elapsed_s']:.1f}s")
    if result["error"] is not None:
        print(f"ERROR                 : {result['error']}")
    else:
        print("ERROR                 : none -- ran to completion")

    ok = result["error"] is None
    drone = result["drone"]
    planner = drone.planner
    if planner is not None:
        true_count = len(result["true_mines_latlon"])
        print(f"mines: discovered={len(planner.backend.mines())} / true={true_count}")
        print(f"reported route        : {len(drone.mission_path)} points")
        try:
            print("iarc path (first 5 lines):")
            for line in drone.get_mission_iarc_path(buffer_width=1).splitlines()[:5]:
                print(f"    {line}")
        except Exception as exc:  # noqa: BLE001
            print(f"get_mission_iarc_path failed: {exc}")
        problems = check_reported_route(planner, drone.mission_path)
        print(f"route check           : {'; '.join(problems) if problems else 'ok'}")
        ok = ok and not problems
    else:
        print("no planner -- configureField never ran")
        ok = False

    camera = drone.camera
    if camera is not None:
        print(f"camera captures taken : {camera.capture_count}")
    print("=" * 70)
    return ok


def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    # A handful of true mines spread through the field so DroneShare's
    # detection/integration path gets exercised, not just the pure-coverage sweep.
    true_mines_local = [(20.0, 40.0), (55.0, 120.0), (35.0, 220.0)]
    result = asyncio.run(run_simulation(true_mines_local))
    if not _report(result):
        sys.exit(1)


if __name__ == "__main__":
    main()
