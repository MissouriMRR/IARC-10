"""
Multi-drone (2/3/4) async integration testbench: drives N REAL state
machines concurrently -- Start -> Takeoff -> CalcScanPath -> Scan ->
DroneShare -> ... -> ExpandNodes -> EndRun -> AppShare -> Recall -> Land
-- through a simulated GAMBLER/ASSISTANT (2 drones), GAMBLER/ASSISTANT +
SOLOGAMBLER (3 drones), or two GAMBLER/ASSISTANT pairs (4 drones) mission,
to answer "does the interdrone wiring (segment-B handoff, SHARE_PHOTOS,
cross-pair mine relay, point-A sync) actually work" by running it, not
just reading it. See integration_tests/one_drone_sim_test.py for the
one-drone (SOLOGAMBLER) equivalent -- this module reuses its MockVehicle/
field-geometry helpers, but everything interdrone-shaped is new, since a
one-drone mission never sends a single interdrone message.

Two problems the one-drone test never has to solve, both because it only
ever runs ONE drone's worth of state in ONE process:

  - Interdrone's real transport (state_machine/interdrone.py's own
    __init__) spawns a background NetworkingThread that binds real
    sockets and connects to peer IPs -- exactly the kind of real-world
    fragility one_drone_sim_test.py's own MockInterdrone avoided by
    replacing Interdrone outright. A multi-drone test can't do that here
    -- the whole point is exercising Interdrone's REAL message-handling
    logic (every case in its receive loop) between real peers. Instead,
    each drone gets a REAL Interdrone instance (bypassing only its
    thread-spawning __init__, the same technique tests/test_emergency_
    land.py already uses) wired to a shared in-process _MessageRouter --
    an asyncio.Queue per drone, routing by Message.drones_to_send_data,
    entirely single-threaded so there's no cross-thread queue/future
    dance to get right.

  - flight/pathfinder.py's Pathfinder.instance is a process-wide
    singleton ("Static variable for singleton access"), fine for a real
    deployment (one drone per process) but not when this testbench runs
    several GAMBLER/SOLOGAMBLER drones concurrently in ONE process: each
    one's own Takeoff.configureField() construction would silently
    stomp every other drone's Pathfinder.instance the moment any of them
    awaits. Solved narrowly -- not by patching flight/pathfinder.py --
    by (a) monkeypatching Pathfinder.__init__ to also record each new
    instance by its own droneID, and (b) wrapping each drone's own
    Interdrone.update_state (already called by StateMachine._run() right
    before every state's own run()) to re-pin Pathfinder.instance to
    THIS drone's own recorded instance immediately before dispatch. That
    covers every Pathfinder-mutating code path (CalcScanPath/Scan/
    DroneShare/EndRun/ExpandNodes all capture Pathfinder.instance into a
    local `pf` once at the top of their own run(), before any further
    await, so re-pinning right before dispatch is sufficient). It does
    NOT cover the two low-stakes, read-only Pathfinder.instance reads
    inside interdrone.py's own receive loop (FIELD_CHECKSUM's comparison,
    REQUEST_MAP_DATA's coordinate conversion) -- those run on a second,
    concurrently-scheduled task per drone (interdrone_loop(), separate
    from the state machine's own run loop) that this pinning hook doesn't
    reach. Worst case there is a spurious mismatch warning or a
    momentarily-wrong reported path, not corrupted mission state -- an
    accepted, documented gap, not a silent one.

Also fixed here (not testbench workarounds -- real, previously-latent
bugs this harness is what actually surfaced them, since a single real
drone process never exercises a paired peer at all):

  - MessageType.PING had no receive-side case at all, so ping_drones()
    could never succeed between two real Interdrone instances and every
    multi-drone Start state would hang forever. See the new
    `case MessageType.PING:` in state_machine/interdrone.py.

  - Drone.last_synced_point_a was read/written by calc_scan_path_impl.py's
    _send_point_a_sync_if_changed but never initialized in Drone.__init__
    -- an AttributeError on the very first CalcScanPath pass for any
    GAMBLER/SOLOGAMBLER with a cross_pair_partner_id (3- or 4-drone only,
    which is why the 2-drone config never caught it).

  - A GAMBLER that finishes its own segment A while its paired ASSISTANT
    still has segment B outstanding bounces CalcScanPath -> Scan ->
    CalcScanPath with an empty local queue and no other await in
    between -- across two drones sharing one event loop (not one drone
    per process, as in a real deployment), that busy loop never yields,
    so it both (a) resent -- and reset -- the ASSISTANT's own in-flight
    queue every single pass (fixed with Drone.last_sent_segment_b, a
    dedupe-on-unchanged-content guard) and (b) starved every other
    drone's task outright, including the interdrone_loop that would have
    delivered the ASSISTANT's own reply (fixed with an asyncio.sleep(0)
    in scan_impl.py's own bounce-back branch). Confirmed as the actual
    cause, not just theorized: a 2-drone run hung indefinitely sending
    the identical SEND_SEGMENT_B_WAYPOINTS message hundreds of times
    before both fixes; 2/3/4-drone runs are clean after them.

Run directly: `python integration_tests/multi_drone_sim_test.py [2|3|4]`
(omit the argument to run all three configurations in sequence).
"""

import asyncio
import contextvars
import logging
import math
import queue as sync_queue
import sys
import time

from integration_tests import _hardware_shims

_hardware_shims.install()

from PIL import Image as PILImage

from vision.Cameras.baseCamera import BaseCamera
from vision.common.detection import Detection
from vision.common.drone_coordinates import DronePose, GimbalPose
from vision.common.image import Image as VisionImage

from flight.pathfinder import Pathfinder, WIDTHOFFIELD, HEIGHTOFFIELD

from interdrone_communication.message_types import MessageType

from state_machine.drone import Drone, RANGEFINDER3_MAVLINK_ID
from state_machine.drone_state import DroneState
from state_machine.flight_settings import FlightSettings, Side, SimMode
from state_machine.interdrone import CMD_MSG, Interdrone
from state_machine.state_machine import StateMachine
from state_machine.states import Start
import vision.Cameras.RPICamera.RPICamera as rpicamera_module

from integration_tests.one_drone_sim_test import (
    MockVehicle,
    _field_corners,
    _local_to_latlon,
    _point_in_latlon_polygon,
    _rangefinder_feed,
)

# ======================================================================
# Pathfinder.instance singleton workaround -- see module docstring.
# ======================================================================
_pathfinder_by_drone_id: dict[int, Pathfinder] = {}
_original_pathfinder_init = Pathfinder.__init__


def _tracking_pathfinder_init(self, *args, **kwargs) -> None:
    _original_pathfinder_init(self, *args, **kwargs)
    _pathfinder_by_drone_id[self.droneID] = self


Pathfinder.__init__ = _tracking_pathfinder_init


def _install_pathfinder_pinning(interdrone: Interdrone) -> None:
    """Wraps this drone's own Interdrone.update_state -- called by
    StateMachine._run() immediately before every state's own run() -- to
    re-pin the process-wide Pathfinder.instance singleton to THIS drone's
    own Pathfinder first. A no-op (via .get() returning None) for an
    ASSISTANT, or before configureField has run for a GAMBLER/
    SOLOGAMBLER."""
    original_update_state = interdrone.update_state
    drone_id = interdrone.flight_settings.current_drone_ID

    def _pinning_update_state(state) -> None:
        # Unconditional, including the None case: an ASSISTANT has no
        # Pathfinder of its own (see configureField) and every state's own
        # code that checks `if pf is not None:` (ExpandNodes, EndRun, ...)
        # relies on genuinely seeing None, the same as a real one-drone-
        # per-process deployment always would. Only setting this when a pf
        # IS registered would let an ASSISTANT silently inherit whatever
        # the most recently active GAMBLER/SOLOGAMBLER last pinned --
        # confirmed happening (ExpandNodes firing nodes_expanded, and
        # mutating that OTHER drone's live Pathfinder, for a role that
        # should have been a pure no-op).
        Pathfinder.instance = _pathfinder_by_drone_id.get(drone_id)
        original_update_state(state)

    interdrone.update_state = _pinning_update_state


# ======================================================================
# Per-drone camera context -- a contextvar, not a single module-level
# dict (one_drone_sim_test.py's own _SIM_CONTEXT), since several drones'
# Takeoff.run() construct a MockCamera concurrently, each needing ITS OWN
# pose_provider/true_mines. Set once, as the first statement of each
# drone's own top-level coroutine (_run_one_drone below) -- asyncio.
# create_task copies the current context at task-creation time, and a
# contextvar set inside one task is invisible to sibling tasks, so this
# is naturally isolated per drone with no locking needed.
# ======================================================================
_sim_context_var: contextvars.ContextVar[dict] = contextvars.ContextVar("sim_context")


class MockCamera(BaseCamera):
    """See one_drone_sim_test.py's own MockCamera -- identical in every
    way except reading its (pose_provider, true_mines_latlon) from the
    current task's contextvar instead of a single shared global."""

    IMAGE_SIZE = (640, 480)

    def __init__(self, vision_config: dict):
        mount = vision_config.get("cameraMountRotationDeg", {})
        super().__init__(
            h_fov_deg=vision_config.get("hFovDeg", 0.0),
            v_fov_deg=vision_config.get("vFovDeg", 0.0),
            offset=tuple(vision_config.get("cameraOffsetM", (0.0, 0.0, 0.0))),
            mount_rotation=GimbalPose(
                yaw=mount.get("yaw", 0.0), pitch=mount.get("pitch", 0.0), roll=mount.get("roll", 0.0)
            ),
        )
        ctx = _sim_context_var.get()
        self._pose_provider = ctx["pose_provider"]
        self._true_mines_latlon: list[tuple[float, float]] = ctx["true_mines_latlon"]
        self._reported: set[int] = set()
        self.capture_count = 0

    def initialize_camera(self) -> None:
        pass

    def capture_image(self, only_metadata: bool) -> VisionImage:
        self.capture_count += 1
        return VisionImage(PILImage.new("RGB", self.IMAGE_SIZE), {})

    def capture_and_detect_mines(self) -> list[Detection]:
        pose: DronePose = self._pose_provider()
        width, height = self.IMAGE_SIZE
        corners = self.get_image_corner_coordinates(width, height, pose)
        if any(c is None for c in corners):
            return []
        top_left, top_right, bottom_left, bottom_right = corners
        polygon = [top_left, top_right, bottom_right, bottom_left]

        detections = []
        for i, (mine_lat, mine_lon) in enumerate(self._true_mines_latlon):
            if i in self._reported:
                continue
            if _point_in_latlon_polygon(mine_lat, mine_lon, polygon):
                self._reported.add(i)
                cx, cy = width / 2.0, height / 2.0
                detections.append(Detection(0.92, (cx, cy, 20, 20), (width, height)))
        return detections

    def capture_and_detect_apriltags(self) -> list[Detection]:
        return []


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
# In-process "network": one asyncio.Queue per drone, routing a sent
# Message to every id in its own drones_to_send_data. Deliberately non-
# blocking (get_nowait/put_nowait) -- unlike the real NetworkingInterface
# (interdrone_communication/networking_interface.py), which bridges the
# state machine's own thread with a SEPARATE networking thread via
# run_coroutine_threadsafe + a blocking future.result(timeout=...), there
# is no second thread here: every drone in this testbench runs on ONE
# event loop, so reproducing that blocking-with-timeout dance would just
# freeze every other drone's task for the duration of the wait.
# ======================================================================
class _MessageRouter:
    def __init__(self) -> None:
        self._queues: dict[int, "asyncio.Queue"] = {}

    def register(self, drone_id: int) -> "asyncio.Queue":
        q: asyncio.Queue = asyncio.Queue()
        self._queues[drone_id] = q
        return q

    def route(self, message) -> None:
        for target_id in message.drones_to_send_data:
            q = self._queues.get(target_id)
            if q is not None:
                q.put_nowait(message)


class _FakeNetworking:
    """NetworkingInterface stand-in -- see _MessageRouter's own docstring."""

    def __init__(self, drone_id: int, router: _MessageRouter) -> None:
        self._router = router
        self._inbox = router.register(drone_id)

    def queue_client_message(self, message, timeout: float | None = None) -> None:
        self._router.route(message)

    def is_client_in_empty(self) -> bool:
        return self._inbox.empty()

    def try_get_server_message(self, timeout: float = 0.0):
        try:
            return self._inbox.get_nowait()
        except asyncio.QueueEmpty:
            return None


def _make_interdrone(flight_settings: FlightSettings, drone: Drone, router: _MessageRouter) -> Interdrone:
    """A real Interdrone with no networking thread behind it -- the real
    constructor spawns one and blocks on it (see interdrone.py's own
    __init__), so every OTHER attribute it would set is assembled by hand
    instead, the same technique tests/test_emergency_land.py's own
    _interdrone() helper uses. Every actual message-handling code path in
    Interdrone (the full receive-loop match statement, every send_*
    method) runs for real from here on -- that is the entire point of
    this testbench."""
    interdrone: Interdrone = object.__new__(Interdrone)
    interdrone._current_task = None
    interdrone._current_state = None
    interdrone._restart_callback = None
    interdrone.flight_settings = flight_settings
    interdrone.drone = drone
    interdrone.drone_states = [
        DroneState(drone_id=did, drone_ip="127.0.0.1")
        for did in flight_settings.other_drones_in_mission
    ]
    interdrone.cmd_msg = CMD_MSG.NONE
    interdrone.max_messages_per_tick = 25
    interdrone._swarm_status_in_progress = False
    interdrone.interdrone_messages = {
        MessageType.PING_ACK: sync_queue.Queue(),
        MessageType.PING_NACK: sync_queue.Queue(),
        MessageType.SEND_GPS_OFFSET_ACK: sync_queue.Queue(),
        MessageType.SEND_DRONE_STATUS: sync_queue.Queue(),
    }
    interdrone.networking = _FakeNetworking(flight_settings.current_drone_ID, router)
    return interdrone


async def _leader_autopilot(interdrone: Interdrone) -> None:
    """Stands in for an operator/app -- ONLY on drone 1. Every other
    drone's cmd_msg is set for real, by actually receiving ARM/
    START_MISSION over interdrone (see interdrone.py's own case
    MessageType.ARM / case MessageType.START_MISSION), the same as a real
    deployment -- this is deliberately not per-drone the way
    one_drone_sim_test.py's own _autopilot is, since there IS only one
    real operator/app in a mission, talking only to drone 1."""
    interdrone.set_cmd_msg(CMD_MSG.ARM)
    while type(interdrone._current_state).__name__ != "Takeoff":
        await asyncio.sleep(0.02)
    interdrone.set_cmd_msg(CMD_MSG.MISSION)


def _start_coord_for(flight_settings: FlightSettings) -> dict:
    """Every drone starts at the same x (field center) -- retarget_
    approach_target is what's supposed to pull two pairs together, not
    starting position -- on whichever y edge its own start_side (auto-
    derived by FlightSettings.__init__ from drone count/id) puts it on."""
    x = WIDTHOFFIELD / 2
    y = 0.0 if flight_settings.start_side == Side.START else float(HEIGHTOFFIELD)
    lat, lon = _local_to_latlon(x, y)
    return {"lat": lat, "lon": lon}


async def _run_one_drone(
    drone_id: int,
    drones_in_mission: list[int],
    corners: tuple[tuple[float, float], ...],
    true_mines_latlon: list[tuple[float, float]],
    router: _MessageRouter,
    timeout_s: float,
) -> dict:
    flight_settings = FlightSettings(
        drone_ID=drone_id,
        drones_in_mission=drones_in_mission,
        mission_corners=[{"lat": c[0], "lon": c[1]} for c in corners],
        max_height=30,
        mission_type="Automatic",
        sim_mode=SimMode.SIM,
    )
    flight_settings.start_coord = _start_coord_for(flight_settings)
    start_lat, start_lon = flight_settings.start_coord["lat"], flight_settings.start_coord["lon"]

    drone = Drone(id=drone_id)
    drone.use_settings(flight_settings.sim_mode)
    vehicle = MockVehicle(start_lat, start_lon)
    drone._vehicle = vehicle
    vehicle.add_message_listener("DISTANCE_SENSOR", drone._on_distance_sensor)

    _sim_context_var.set(
        {"pose_provider": _make_pose_provider(vehicle), "true_mines_latlon": true_mines_latlon}
    )
    # Shared across every drone's own task -- Takeoff.run()'s local
    # `from vision.Cameras.RPICamera.RPICamera import RPICamera` import
    # (see takeoff_impl.py) re-resolves this name from RPICamera's own
    # home module at call time, so a single class-level patch here is
    # enough; MockCamera itself reads the calling task's own contextvar,
    # not anything set here.
    rpicamera_module.RPICamera = MockCamera

    interdrone = _make_interdrone(flight_settings, drone, router)
    _install_pathfinder_pinning(interdrone)
    machine = StateMachine(
        Start(drone, flight_settings, interdrone), drone, flight_settings, interdrone
    )

    background_tasks = [
        asyncio.ensure_future(interdrone.interdrone_loop()),
        asyncio.ensure_future(_rangefinder_feed(vehicle)),
    ]
    if drone_id == 1:
        background_tasks.append(asyncio.ensure_future(_leader_autopilot(interdrone)))

    error = None
    started_at = time.monotonic()
    try:
        await asyncio.wait_for(machine.run(), timeout=timeout_s)
    except asyncio.TimeoutError:
        last_state = type(interdrone._current_state).__name__ if interdrone._current_state else None
        error = f"timed out after {timeout_s:.0f}s (last state: {last_state})"
    except Exception as exc:  # noqa: BLE001 -- report whatever broke, don't hide it
        logging.exception("drone %d: state machine raised", drone_id)
        error = exc
    finally:
        elapsed = time.monotonic() - started_at
        for task in background_tasks:
            task.cancel()
        for task in background_tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:  # noqa: BLE001 -- a background task's own crash shouldn't hide `error` above
                logging.exception("drone %d: background task raised during teardown", drone_id)

    return {
        "drone_id": drone_id,
        "role": flight_settings.role,
        "drone": drone,
        "vehicle": vehicle,
        "interdrone": interdrone,
        "pathfinder": _pathfinder_by_drone_id.get(drone_id),
        "final_state": type(interdrone._current_state).__name__ if interdrone._current_state else None,
        "error": error,
        "elapsed_s": elapsed,
    }


# ======================================================================
# Driver
# ======================================================================
async def run_simulation(
    n_drones: int, true_mines_local: list[tuple[float, float]], timeout_s: float = 300.0
) -> list[dict]:
    if n_drones not in (2, 3, 4):
        raise ValueError(f"n_drones must be 2, 3, or 4 (got {n_drones})")

    # Fresh registry per run -- otherwise a later run's drone 1 would see
    # an earlier run's drone 1 Pathfinder still sitting in the dict.
    _pathfinder_by_drone_id.clear()
    Pathfinder.instance = None

    corners = _field_corners()
    true_mines_latlon = [_local_to_latlon(x, y) for x, y in true_mines_local]
    drones_in_mission = list(range(1, n_drones + 1))
    router = _MessageRouter()

    tasks = [
        asyncio.ensure_future(
            _run_one_drone(did, drones_in_mission, corners, true_mines_latlon, router, timeout_s)
        )
        for did in drones_in_mission
    ]
    results = await asyncio.gather(*tasks)
    for result in results:
        result["true_mines_latlon"] = true_mines_latlon
    return results


def _report(n_drones: int, results: list[dict]) -> bool:
    """Prints the per-drone + swarm-level report. Returns True iff every
    drone reached Land with no error."""
    print()
    print("=" * 70)
    print(f"{n_drones}-DRONE STATE MACHINE SIMULATION -- REPORT")
    print("=" * 70)
    all_ok = True
    for result in sorted(results, key=lambda r: r["drone_id"]):
        did = result["drone_id"]
        ok = result["error"] is None and result["final_state"] == "Land"
        all_ok = all_ok and ok
        print(f"-- drone {did} ({result['role'].name}) " + "-" * (52 - len(result["role"].name)))
        print(f"   final state : {result['final_state']}")
        print(f"   elapsed     : {result['elapsed_s']:.1f}s")
        print(f"   error       : {result['error'] if result['error'] is not None else 'none'}")
        pf = result["pathfinder"]
        if pf is not None:
            # protoMines is a raw discovery-EVENT log (Pathfinder.
            # add_discovered_mine appends to it unconditionally, even when
            # the detection merges into an already-known obstacle) -- not
            # a count of distinct mines, so it can legitimately exceed the
            # true count even with zero bugs (the same physical mine
            # photographed once by the GAMBLER and once by its ASSISTANT
            # near the segment A/B seam is two discovery events). Distinct
            # LIVE obstacles is what actually answers "how many different
            # mines does this drone's graph know about."
            live_obstacles = len(pf.nodeField.mines) + len(pf.nodeField.unionObstacles)
            print(
                f"   mines found : {live_obstacles} distinct / {len(result['true_mines_latlon'])} true"
                f"  ({len(pf.protoMines)} discovery events)"
            )
            print(f"   seen cells  : {pf.seen_tracker.count()}")
        camera = result["drone"].camera
        if camera is not None:
            print(f"   captures    : {camera.capture_count}")
    print("=" * 70)
    print(f"RESULT: {'PASS' if all_ok else 'FAIL'} -- all drones reached Land: {all_ok}")
    print("=" * 70)
    return all_ok


def _mines_for(n_drones: int) -> list[tuple[float, float]]:
    """A single column near the field's own x-center (matching
    _start_coord_for's start x), spaced every 40ft up the FULL height --
    close enough together that a GAMBLER's own segment-A sweep chains
    through several discoveries in a row before the paired ASSISTANT's
    SHARE_PHOTOS reports catch up (see Pathfinder.start_helper_node_
    detour's own docstring on chaining), which is what actually produces
    a non-trivial maze_b_path for the ASSISTANT to fly -- a handful of
    mines spaced 80-100ft apart (this module's own first-draft mine set)
    each got resolved by a single cheap helper-node detour with nothing
    left over to hand off, so 2-drone SHARE_PHOTOS never actually
    triggered. Spanning the full 0-300ft height (not just one end) means
    a 3/4-drone mission's END-side pair/solo (sweeping from the far edge
    inward, per _start_coord_for) runs into its own early local finds
    too, not just whatever the START side eventually relays over
    CROSS_PAIR_MINE_RELAY."""
    return [(40.0, y) for y in range(30, 271, 40)]


def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    requested = sys.argv[1:2]
    configs = [int(requested[0])] if requested else [2, 3, 4]

    all_ok = True
    for n_drones in configs:
        results = asyncio.run(run_simulation(n_drones, _mines_for(n_drones)))
        all_ok = _report(n_drones, results) and all_ok

    if not all_ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
