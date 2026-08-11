"""Preflight check that `run.py` can talk to a real flight controller.

Everything else in `tests/` runs against fakes, which is the only way to test
logic but says nothing about whether the wire works. This connects to an actual
autopilot through the same code path `run.py` uses -- `FlightSettings` ->
`Drone.use_settings` -> `Drone.connect_drone` -- and checks the things the
flight code assumes about it before anything spins a motor.

Nothing here arms, commands a mode, or moves the vehicle. It reads telemetry
and, in one test, closes and reopens the connection.

Skipped unless `FC_TEST=1`, so a normal `pytest tests/` on a laptop is
unaffected. On the Pi, with the autopilot wired up:

    FC_TEST=1 uv run --with pytest python -m pytest tests/test_flight_controller_connection.py -v -s

or just `python tests/test_flight_controller_connection.py`, which sets the
variable itself.

Environment
-----------
FC_TEST=1
    Required. Without it the whole module is skipped.
FC_SIM_MODE=real|sim|airsim, default real
    Which set of connection settings to test, exactly as `use_settings`
    computes them. Use `sim` to rehearse this against a SITL first.
FC_ADDRESS, FC_BAUD
    Override the address and baud rate `use_settings` chose. Rarely needed --
    testing the address the flight code picks on its own is the point.
FC_DRONE_ID, default 1
    Matters in airsim mode, where the port is derived from the ID.
FC_REQUIRE_GPS=1
    Enforce the outdoor readiness checks (3D fix, armable, home location)
    rather than reporting them and skipping. Set this for the real preflight;
    leave it off on a bench with no sky.
"""

import asyncio
import os
import sys
import time
from typing import Any, Awaitable
from unittest import mock

import pytest

# Runnable directly from the repo root as a script, where `tests/` is the
# script's directory and the package root is its parent.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import state_machine.drone as drone_module  # noqa: E402
from state_machine.drone import Drone  # noqa: E402
from state_machine.flight_settings import SimMode  # noqa: E402

# The link-recovery work (link_age/link_alive/ensure_link) lives on a branch
# that is not always checked out. Where it is present these tests exercise it;
# where it is not they fall back to DroneKit's own heartbeat clock, so this file
# is useful either way.
HAS_LINK_RECOVERY: bool = hasattr(Drone, "ensure_link")
needs_link_recovery = pytest.mark.skipif(
    not HAS_LINK_RECOVERY,
    reason="this checkout has no link-recovery API in state_machine.drone",
)

# How long the autopilot may go without a heartbeat before its telemetry stops
# being believed. Matches the flight code's own limit when it defines one;
# DroneKit warns at 5s regardless.
LINK_STALE_S: float = float(getattr(drone_module, "LINK_STALE_S", 5.0))

pytestmark = pytest.mark.skipif(
    os.environ.get("FC_TEST") != "1",
    reason="needs a flight controller on the other end; set FC_TEST=1 to run",
)

# How long to watch a connected link before believing it. Longer than
# LINK_STALE_S so a radio that drops enough heartbeats to trip the watchdog
# gets caught here instead of in the air.
HEARTBEAT_SAMPLE_S: float = 12.0

# Budget for one reconnect in the reconnect test. `ensure_link` retries, so
# this covers a couple of attempts on a serial port that needs a moment to
# become reusable.
RECONNECT_BUDGET_S: float = 60.0

# How long to wait for the autopilot to declare itself armable, once the GPS
# checks are being enforced. EKF convergence after boot is not instant.
ARMABLE_WAIT_S: float = 60.0

# ArduPilot's fix_type in GPS_RAW_INT: 3 is a 3D fix, the minimum for a
# position-controlled mode.
GPS_3D_FIX: int = 3

# The modes the flight code puts the vehicle into. GUIDED for the mission,
# RTL for the graceful exit and return_to_launch, LAND for emergency landing.
COMMANDED_MODES: tuple[str, ...] = ("GUIDED", "RTL", "LAND")

# The mode `FlightManager.kill_switch` waits for the pilot to select.
KILL_SWITCH_MODE: str = "POSITION"


def _run(coro: Awaitable[Any], timeout: float = 120.0) -> Any:
    return asyncio.run(asyncio.wait_for(coro, timeout))


def _sim_mode() -> SimMode:
    return SimMode(os.environ.get("FC_SIM_MODE", "real").lower())


def _report(label: str, value: Any) -> None:
    """Print a measurement. Visible under `-s`, which this is meant to run with."""
    print(f"    {label:<26} {value}")


def _link_age(connection: Drone) -> float:
    """Seconds since the autopilot last spoke.

    Prefers the flight code's own figure where it exists. DroneKit's
    `last_heartbeat` is the fallback: it is measured on the MAVLink input
    thread, so it freezes rather than ageing once the link truly dies -- fine
    for measuring gaps on a live link, which is all it is used for here.
    """
    if HAS_LINK_RECOVERY:
        return float(connection.link_age)
    return float(connection.vehicle.last_heartbeat)


def _close_quietly(vehicle: Any) -> None:
    closer = getattr(drone_module, "_close_quietly", None)
    if closer is not None:
        closer(vehicle)
        return
    try:
        vehicle.close()
    except Exception:  # pylint: disable=broad-except
        pass


@pytest.fixture(scope="module")
def drone() -> Drone:
    """One connection, opened the way `run.py` opens it, shared by every test.

    Connecting costs seconds -- DroneKit waits for the first heartbeat and
    then downloads the whole parameter table -- and reconnecting per test on a
    serial port invites the port-still-busy failure that has nothing to do with
    what is being measured.
    """
    sim_mode: SimMode = _sim_mode()
    connection = Drone(id=int(os.environ.get("FC_DRONE_ID", "1")))
    connection.use_settings(sim_mode)

    if "FC_ADDRESS" in os.environ:
        connection.address = os.environ["FC_ADDRESS"]
    if "FC_BAUD" in os.environ:
        connection.baud = int(os.environ["FC_BAUD"])

    print(f"\n  connecting: mode={sim_mode.name} address={connection.address} baud={connection.baud}")
    started: float = time.monotonic()
    try:
        # `connect_drone` blocks on `input()` in REAL mode, waiting for the
        # operator to confirm before the state machine starts. That prompt is
        # part of the path being tested, so stub the read rather than route
        # around the call.
        with mock.patch("builtins.input", return_value=""):
            _run(connection.connect_drone())
    except Exception as ex:  # pylint: disable=broad-except
        pytest.fail(
            f"could not connect to the autopilot at {connection.address}"
            f" (baud={connection.baud}) after {time.monotonic() - started:.0f}s: {ex}\n"
            "run.py will fail the same way here."
        )

    _report("connected in", f"{time.monotonic() - started:.1f}s")
    yield connection

    _close_quietly(connection.vehicle)


class TestConnection:
    """The connection itself, and the link health the flight code polls."""

    def test_connects_at_the_address_the_flight_code_chooses(self, drone: Drone) -> None:
        """`use_settings` picks the address; nothing later gets to disagree.

        A connection made to an address typed into this test would prove only
        that the hardware answers somewhere.
        """
        assert drone.is_connected
        assert drone.vehicle is not None

        if _sim_mode() is SimMode.REAL and "FC_ADDRESS" not in os.environ:
            assert drone.address == "/dev/serial0"
            assert drone.baud == 115200

        _report("address", drone.address)
        _report("autopilot", drone.vehicle.version)
        _report("vehicle type", drone.vehicle._vehicle_type)

    @needs_link_recovery
    def test_the_link_reads_as_alive(self, drone: Drone) -> None:
        """`link_alive` gates every command and every flight guard.

        If it reads false on a healthy bench connection, the watchdog will tear
        down a working link mid-flight and the states will land a fine vehicle.
        """
        assert drone.link_alive, (
            f"connected, but the last heartbeat is {drone.link_age:.1f}s old"
            f" (stale past {LINK_STALE_S}s)"
        )

    def test_heartbeats_keep_arriving(self, drone: Drone) -> None:
        """Watch the link for longer than it takes to go stale.

        A radio that drops the odd heartbeat still connects fine -- the failure
        shows up later, as the link watchdog rebuilding a connection that was
        never actually broken. Measuring the worst gap over a window says
        whether the real link has margin against LINK_STALE_S.
        """
        worst_age: float = 0.0
        deadline: float = time.monotonic() + HEARTBEAT_SAMPLE_S
        while time.monotonic() < deadline:
            worst_age = max(worst_age, _link_age(drone))
            time.sleep(0.2)

        _report("worst heartbeat gap", f"{worst_age:.2f}s of {LINK_STALE_S}s allowed")
        assert worst_age < LINK_STALE_S, (
            f"heartbeats went {worst_age:.1f}s apart over {HEARTBEAT_SAMPLE_S:.0f}s of"
            f" watching, past the {LINK_STALE_S}s staleness limit. Telemetry this old"
            " is a link the flight code cannot trust to command or observe the"
            " vehicle."
        )

    @needs_link_recovery
    def test_ensure_link_leaves_a_healthy_link_alone(self, drone: Drone) -> None:
        """Called from the watchdog every second and from the landing states.

        On a live link it must be a no-op: rebuilding here would mean closing a
        working connection on a schedule.
        """
        before: int = drone.link_reconnects
        assert _run(drone.ensure_link(deadline_s=0)) is True
        assert drone.link_reconnects == before

    @needs_link_recovery
    def test_a_closed_connection_can_be_rebuilt(self, drone: Drone) -> None:
        """The recovery path, exercised against the real port.

        This is the half of link recovery that fakes cannot cover. Reopening a
        serial port the same process just closed, or a TCP port the SITL only
        hands to one client, is where recovery actually fails -- and the flight
        code only finds out with the vehicle in the air.
        """
        if drone.vehicle.armed:
            pytest.skip("vehicle is armed; not touching the link")

        _close_quietly(drone.vehicle)
        # `ensure_link` reconnects on silence, not on the close, and DroneKit's
        # cached heartbeat timestamp survives the close. Age it past the
        # threshold so this asks the question the watchdog would ask.
        drone._last_heartbeat_at = time.monotonic() - 60.0
        assert not drone.link_alive

        before: int = drone.link_reconnects
        started: float = time.monotonic()
        recovered: bool = _run(
            drone.ensure_link(deadline_s=RECONNECT_BUDGET_S),
            timeout=RECONNECT_BUDGET_S + 30.0,
        )

        _report("reconnected in", f"{time.monotonic() - started:.1f}s")
        assert recovered, (
            f"could not reopen {drone.address} within {RECONNECT_BUDGET_S:.0f}s."
            " A link lost in flight would stay lost."
        )
        assert drone.link_alive
        assert drone.link_reconnects == before + 1


class TestTelemetry:
    """The attributes the states read. DroneKit fills these from the stream, so
    an attribute that is still None is a message the autopilot is not sending.
    """

    def test_position_telemetry_is_populated(self, drone: Drone) -> None:
        """`currentPosition`, `gotoWaypoint` and every arrival check read this.

        A None here does not raise where it is read -- it propagates into a
        distance calculation and fails somewhere unrelated.
        """
        position = drone.vehicle.location.global_relative_frame
        _report("position", f"{position.lat}, {position.lon} @ {position.alt}m rel")

        assert position.lat is not None and position.lon is not None, (
            "no GLOBAL_POSITION_INT from the autopilot: the mission cannot"
            " measure where it is or whether it has arrived anywhere"
        )
        assert position.alt is not None, "no relative altitude; takeoff has nothing to wait on"

    def test_mode_and_armed_state_are_readable(self, drone: Drone) -> None:
        """Read by the kill switch, `_check_still_ours`, and the graceful exit."""
        mode = drone.vehicle.mode
        assert mode is not None and mode.name is not None
        assert drone.vehicle.armed is not None
        assert drone.vehicle.system_status is not None

        _report("mode", mode.name)
        _report("armed", drone.vehicle.armed)
        _report("system status", drone.vehicle.system_status.state)

    def test_battery_and_heading_are_reported(self, drone: Drone) -> None:
        """Not flight-critical to the code, but a silent battery or compass is
        a wiring fault worth finding on the bench rather than in the log.
        """
        battery = drone.vehicle.battery
        heading = drone.vehicle.heading
        _report("battery", f"{battery.voltage}V, {battery.level}%")
        _report("heading", heading)

        assert battery is not None and battery.voltage is not None, "no SYS_STATUS battery data"
        assert heading is not None, "no heading; the compass is not reporting"

    def test_parameters_downloaded(self, drone: Drone) -> None:
        """`connect(wait_ready=True)` promises the parameter table, and
        `remove_arming_check` writes into it. A partial download is a slow or
        lossy link, and it shows up as a connect that took a very long time.
        """
        parameters = drone.vehicle.parameters
        count: int = len(parameters)
        _report("parameters", count)
        _report("ARMING_CHECK", parameters.get("ARMING_CHECK"))

        assert count > 50, (
            f"only {count} parameters downloaded; DroneKit's wait_ready did not"
            " finish the table, which usually means a lossy or very slow link"
        )


class TestModes:
    """Every mode the flight code names has to exist on this firmware. A mode
    name the autopilot does not know is accepted silently by DroneKit and
    simply never takes effect.
    """

    @pytest.mark.parametrize("mode", COMMANDED_MODES)
    def test_the_modes_the_flight_code_commands_exist(self, drone: Drone, mode: str) -> None:
        available = drone.vehicle._mode_mapping
        assert mode in available, (
            f"this firmware has no {mode} mode (it has: {sorted(available)}). The"
            " flight code sets it by name, so the command would be discarded"
            " with no error and the vehicle would keep doing what it was doing."
        )

    def test_the_kill_switch_mode_exists(self, drone: Drone) -> None:
        """`FlightManager.kill_switch` waits for `mode.name == "POSITION"`.

        Separate from the modes above because this one is a *trigger*, not a
        command: if the firmware calls the pilot's position-hold mode something
        else, the switch never fires and flipping it does nothing.
        """
        available = drone.vehicle._mode_mapping
        assert KILL_SWITCH_MODE in available, (
            f"this firmware has no {KILL_SWITCH_MODE} mode (it has:"
            f" {sorted(available)}). The kill switch waits for that exact name,"
            " so it can never trigger on this vehicle."
        )


class TestPreflightReadiness:
    """Whether this vehicle could actually fly the mission right now.

    Reported rather than enforced unless FC_REQUIRE_GPS=1, because indoors none
    of it can pass and the connection tests above still mean something there.
    """

    @staticmethod
    def _check(passed: bool, message: str) -> None:
        if os.environ.get("FC_REQUIRE_GPS") == "1":
            assert passed, message
        elif not passed:
            pytest.skip(f"{message} (set FC_REQUIRE_GPS=1 to make this a failure)")

    def test_gps_has_a_3d_fix(self, drone: Drone) -> None:
        gps = drone.vehicle.gps_0
        _report("gps", f"fix {gps.fix_type}, {gps.satellites_visible} sats")
        self._check(
            gps.fix_type is not None and gps.fix_type >= GPS_3D_FIX,
            f"GPS fix type is {gps.fix_type}, need {GPS_3D_FIX} for a position-controlled mode",
        )

    def test_the_vehicle_becomes_armable(self, drone: Drone) -> None:
        """`Drone.arm` waits on `is_armable` with no timeout -- a vehicle that
        never becomes armable hangs the Start state forever with nothing in the
        log after "Waiting for vehicle to intialize...".
        """
        deadline: float = time.monotonic() + ARMABLE_WAIT_S
        while not drone.vehicle.is_armable and time.monotonic() < deadline:
            time.sleep(1.0)

        armable: bool = bool(drone.vehicle.is_armable)
        _report("is_armable", armable)
        _report("ekf ok", drone.vehicle.ekf_ok)
        self._check(
            armable,
            f"not armable after {ARMABLE_WAIT_S:.0f}s (ekf_ok={drone.vehicle.ekf_ok});"
            " Drone.arm() would wait here indefinitely",
        )

    def test_home_location_is_set(self, drone: Drone) -> None:
        """`return_to_launch` reads `home_location.lat` directly, so an unset
        home is an AttributeError partway through a landing.
        """
        # DroneKit only populates home_location after the waypoints are read.
        if drone.vehicle.home_location is None:
            drone.vehicle.commands.download()
            drone.vehicle.commands.wait_ready()

        home = drone.vehicle.home_location
        _report("home", home)
        self._check(
            home is not None,
            "no home location; return_to_launch() reads home_location.lat and would"
            " raise AttributeError mid-landing",
        )


if __name__ == "__main__":
    os.environ.setdefault("FC_TEST", "1")
    sys.exit(pytest.main([os.path.abspath(__file__), "-v", "-s"]))
