"""
Multi-drone (2/3/4) async integration testbench: drives N REAL state
machines concurrently -- Start -> Takeoff -> CalcScanPath -> Scan ->
DroneShare -> ... -> ExpandNodes -> EndRun -> AppShare -> Recall -> Land
-- through a simulated mission: one side of 2 drones, a side of 2 plus a
side of 1 (3 drones), or two sides of 2 (4 drones). Every drone keeps its
own copy of its side's planner (flight/pathfinding/side_agent.py). Answers
"does the interdrone wiring (SHARE_PHOTOS between side-mates, SIDE_JOB,
cross-side mine relay and sync, point A, the join) actually work" by
running it, not just reading it. See integration_tests/one_drone_sim_test.py
for the one-drone equivalent -- this module reuses its MockVehicle/
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

  - Messages cross the router as real JSON (the same encode/decode the
    networking thread does), so wire-format bugs show up here too.

Also fixed here (not testbench workarounds -- real, previously-latent
bugs this harness is what actually surfaced them, since a single real
drone process never exercises a paired peer at all):

  - MessageType.PING had no receive-side case at all, so ping_drones()
    could never succeed between two real Interdrone instances and every
    multi-drone Start state would hang forever. See the new
    `case MessageType.PING:` in state_machine/interdrone.py.

  - A drone with an empty queue (waiting on its side-mate's photos or the
    other side's point A) bounces CalcScanPath -> Scan -> CalcScanPath with
    no other await in between -- across drones sharing one event loop (not
    one drone per process, as in a real deployment), that busy loop never
    yields and starves every other drone's task, including the
    interdrone_loop that would deliver what it is waiting on (fixed with an
    asyncio.sleep(0) in scan_impl.py's bounce-back branch).

Run directly: `python integration_tests/multi_drone_sim_test.py [2|3|4] [dense]`
(omit the count to run all three configurations in sequence).
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

from vision.common.drone_coordinates import DronePose

from interdrone_communication.json_message_utilities import JsonMessageUtilities
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
    TruthCamera,
    _field_corners,
    _local_to_latlon,
    _rangefinder_feed,
    check_reported_route,
    HEIGHTOFFIELD,
    WIDTHOFFIELD,
    _Location,
    _LocationBundle,
)

# How fast the fake vehicles fly. Fast, so the harness stays quick, but not instant: a leg then
# takes real time, so the messages two drones send each other (position, photo, job) reach the
# other side while it is still planning the next leg -- as they do in flight. A teleport lets one
# drone race through a hundred photos before its mate hears the first.
FAKE_SPEED_M_S = 8.0
_M_PER_DEG_LAT = 111_320.0


class _MovingLocation(_Location):
    """A position that moves in a straight line at constant speed toward the last goto target."""

    def __init__(self, lat: float, lon: float, alt: float):
        self._from, self._to, self._since = (lat, lon), (lat, lon), 0.0
        self._len_m = 0.0
        self.alt = alt

    def go(self, lat: float, lon: float) -> None:
        """Head for (lat, lon) from wherever the vehicle is now."""
        here = (self.lat, self.lon)
        dlat_m = (lat - here[0]) * _M_PER_DEG_LAT
        dlon_m = (lon - here[1]) * _M_PER_DEG_LAT * math.cos(math.radians(here[0]))
        self._from, self._to, self._since = here, (lat, lon), time.monotonic()
        self._len_m = math.hypot(dlat_m, dlon_m)

    @property
    def _fraction(self) -> float:
        if self._len_m == 0.0:
            return 1.0
        return min(1.0, (time.monotonic() - self._since) * FAKE_SPEED_M_S / self._len_m)

    @property
    def lat(self) -> float:
        return self._from[0] + (self._to[0] - self._from[0]) * self._fraction

    @property
    def lon(self) -> float:
        return self._from[1] + (self._to[1] - self._from[1]) * self._fraction


class PacedVehicle(MockVehicle):
    """MockVehicle that flies its legs (FAKE_SPEED_M_S) instead of teleporting."""

    def __init__(self, start_lat: float, start_lon: float):
        super().__init__(start_lat, start_lon)
        self.location = _LocationBundle(start_lat, start_lon, 0.0)
        for frame in ("global_relative_frame", "global_frame"):
            setattr(self.location, frame, _MovingLocation(start_lat, start_lon, 0.0))

    def simple_goto(self, location, groundspeed: float | None = None) -> None:
        for frame in (self.location.global_relative_frame, self.location.global_frame):
            frame.go(location.lat, location.lon)
        if location.alt is not None:
            self.location.global_relative_frame.alt = location.alt

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


class MockCamera(TruthCamera):
    """The multi-drone test's camera: pose and mines from the current task's contextvar
    (see above), so each drone's camera sees its own vehicle."""

    def __init__(self, vision_config: dict):
        ctx = _sim_context_var.get()
        super().__init__(vision_config, ctx["pose_provider"], ctx["true_mines_latlon"])


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
        # Through real JSON, as the networking thread sends it.
        message = JsonMessageUtilities.message_from_json(JsonMessageUtilities.message_to_json(message))
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


LAUNCH_SPACING_FT = 10.0


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

    drone = Drone(id=drone_id)
    drone.use_settings(flight_settings.sim_mode)
    # Every drone of a side plans from the same entry (start_coord), but no two launch
    # from the same pad: the second drone of a pair starts LAUNCH_SPACING_FT to the side,
    # or the measured-separation check (live positions) would trip at the first leg.
    launch_x = WIDTHOFFIELD / 2 + (LAUNCH_SPACING_FT if flight_settings.paired_drone is not None
                                   and drone_id > flight_settings.paired_drone else 0.0)
    launch_y = 0.0 if flight_settings.start_side == Side.START else float(HEIGHTOFFIELD)
    vehicle = PacedVehicle(*_local_to_latlon(launch_x, launch_y))
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
        "side": "bottom" if drone_id in (1, 2) else "top",
        "drone": drone,
        "vehicle": vehicle,
        "interdrone": interdrone,
        "final_state": type(interdrone._current_state).__name__ if interdrone._current_state else None,
        "error": error,
        "elapsed_s": elapsed,
    }


# ======================================================================
# Driver
# ======================================================================
async def run_simulation(
    n_drones: int,
    true_mines_local: list[tuple[float, float]],
    timeout_s: float = 300.0,
) -> list[dict]:
    if n_drones not in (2, 3, 4):
        raise ValueError(f"n_drones must be 2, 3, or 4 (got {n_drones})")

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
        print(f"-- drone {did} ({result['side']} side) " + "-" * (47 - len(result["side"])))
        print(f"   final state : {result['final_state']}")
        print(f"   elapsed     : {result['elapsed_s']:.1f}s")
        print(f"   error       : {result['error'] if result['error'] is not None else 'none'}")
        planner = result["drone"].planner
        if planner is not None:
            print(
                f"   mines found : {len(planner.backend.mines())} / "
                f"{len(result['true_mines_latlon'])} true"
            )
            print(f"   seen cells  : {len(list(planner.backend.planner.seen.on_cells()))}")
        if did == 1 and planner is not None:
            drone = result["drone"]
            problems = check_reported_route(planner, drone.mission_path)
            if drone.reported_half_route:
                problems.append("reported only its own half")
            print(f"   reported    : {len(drone.mission_path)} points -- "
                  f"{'; '.join(problems) if problems else 'full field, continuous, clear'}")
            all_ok = all_ok and not problems
        camera = result["drone"].camera
        if camera is not None:
            print(f"   captures    : {camera.capture_count}")
    planners = [r["drone"].planner for r in results if r["drone"].planner is not None]
    if len(planners) > 1:
        # Every mine is relayed both ways, so both leads must end with the same set.
        agree = len({p.mine_hash() for p in planners}) == 1
        print(f"final mine sets agree : {agree}")
        all_ok = all_ok and agree
    print("=" * 70)
    print(f"RESULT: {'PASS' if all_ok else 'FAIL'} -- all drones reached Land: {all_ok}")
    print("=" * 70)
    return all_ok


def _mines_for(n_drones: int) -> list[tuple[float, float]]:
    """A single column near the field's own x-center (matching
    _start_coord_for's start x), spaced every 40ft up the FULL height --
    close enough together that one drone's sweep chains through several
    discoveries in a row before its side-mate's SHARE_PHOTOS reports catch
    up, which is what actually produces detours for the side-mate to fly --
    a handful of
    mines spaced 80-100ft apart (this module's own first-draft mine set)
    each got resolved by a single cheap helper-node detour with nothing
    left over to hand off, so 2-drone SHARE_PHOTOS never actually
    triggered. Spanning the full 0-300ft height (not just one end) means
    a 3/4-drone mission's END-side pair/solo (sweeping from the far edge
    inward, per _start_coord_for) runs into its own early local finds
    too, not just whatever the START side eventually relays over
    CROSS_PAIR_MINE_RELAY."""
    return [(40.0, y) for y in range(30, 271, 40)]


def _dense_mines(count: int = 60, seed: int = 3) -> list[tuple[float, float]]:
    """A random field (the path-node simulator's generator). With the column above, a
    route steps around the first mine and then runs clear of the rest, so a member
    gets little work."""
    from flight.pathfinding.pathnode.minefield import generate_true_minefield

    return list(generate_true_minefield(count, seed))


def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    args = sys.argv[1:]
    counts = [int(a) for a in args if a.isdigit()]
    configs = counts or [2, 3, 4]
    mines = _dense_mines if "dense" in args else _mines_for

    all_ok = True
    for n_drones in configs:
        results = asyncio.run(run_simulation(
                n_drones, mines(n_drones) if mines is _mines_for else mines()
            ))
        all_ok = _report(n_drones, results) and all_ok

    if not all_ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
