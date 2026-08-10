"""Regression tests for the LAND and EMERGENCY_LAND commands.

Before this, both commands set `cmd_msg` and nothing else: a drone flying the
POIF circles that was told to land kept flying the circles. The tests here are
about the whole path from the message arriving to the vehicle being commanded,
because every interesting failure mode is in the seams between the interdrone
loop, the state machine, and the autopilot -- and every one of them fails
*silently*, which is the worst way for a land command to fail.

No pytest-asyncio: each async scenario is driven with `asyncio.run` so these
run against the project's own dependencies.
"""

import asyncio
from typing import Any, Awaitable, Callable

import pytest

from interdrone_communication.message_types import Message, MessageType
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import CMD_MSG, Interdrone
from state_machine.state_machine import StateMachine
from state_machine.states.emergency_land import EmergencyLand
from state_machine.states.impl import emergency_land_impl
from state_machine.states.state import State

# The real intervals are tuned for a vehicle; these tests only care about the
# ordering, so they run the state at a pace that keeps the suite quick.
FAST_POLL_S = 0.01
FAST_CONFIRM_S = 0.05


def _run(coro: Awaitable[Any], timeout: float = 10.0) -> Any:
    """Drive an async scenario to completion, failing rather than hanging."""
    return asyncio.run(asyncio.wait_for(coro, timeout))


class FakeMode:
    def __init__(self, name: str) -> None:
        self.name = name


class FakeVehicle:
    """Minimal stand-in for a DroneKit Vehicle.

    `drop_mode_sets` swallows the first N mode commands without applying them,
    which is what a mode set lost on the MAVLink link looks like from here.
    """

    def __init__(self, mode: str = "GUIDED", armed: bool = True, drop_mode_sets: int = 0) -> None:
        self._mode = FakeMode(mode)
        self.armed = armed
        self.drop_mode_sets = drop_mode_sets
        self.mode_commands: list[str] = []

    @property
    def mode(self) -> FakeMode:
        return self._mode

    @mode.setter
    def mode(self, value: Any) -> None:
        name: str = getattr(value, "name", str(value))
        self.mode_commands.append(name)
        if self.drop_mode_sets > 0:
            self.drop_mode_sets -= 1
            return
        self._mode = FakeMode(name)


class FakeDrone:
    """A Drone whose `vehicle` raises the same way the real one does when unset."""

    def __init__(self, vehicle: FakeVehicle | None = None, drone_id: int = 1) -> None:
        self._vehicle = vehicle
        self.id = drone_id
        self.address = ""
        self.odlc_scan = None

    @property
    def vehicle(self) -> FakeVehicle:
        if self._vehicle is None:
            raise RuntimeError("we haven't connected to the drone yet")
        return self._vehicle


class FakeNetworking:
    """Hands the interdrone loop a fixed script of messages, then nothing."""

    def __init__(self, incoming: list[Message] | None = None) -> None:
        self.incoming: list[Message] = list(incoming or [])
        self.sent: list[Message] = []

    def try_get_server_message(self, timeout: float = 0.0) -> Message | None:
        return self.incoming.pop(0) if self.incoming else None

    def queue_client_message(self, message: Message) -> None:
        self.sent.append(message)


class BusyState(State):
    """Holds its atomic lock briefly, then flies forever.

    Stands in for POIF: something long-running that has a section it must not
    be cancelled in the middle of.
    """

    hold_s: float = 0.1

    def run(self) -> Awaitable[None]:
        return self._fly()

    async def _fly(self) -> None:
        async with self.atomic:
            await asyncio.sleep(self.hold_s)
        await asyncio.sleep(3600)


def _settings(drones: list[int] | None = None) -> FlightSettings:
    return FlightSettings(drone_ID=1, drones_in_mission=drones or [1])


def _interdrone(drone: FakeDrone, settings: FlightSettings, **kwargs: Any) -> Interdrone:
    """An Interdrone with no networking thread behind it.

    The real constructor spawns a thread and blocks on it, so the parts these
    tests exercise are assembled directly.
    """
    interdrone: Interdrone = object.__new__(Interdrone)
    interdrone._current_task = None
    interdrone._current_state = None
    interdrone._restart_callback = None
    interdrone.flight_settings = settings
    interdrone.drone = drone  # type: ignore[assignment]
    interdrone.drone_states = []
    interdrone.cmd_msg = CMD_MSG.NONE
    interdrone.interdrone_messages = {}
    interdrone.networking = FakeNetworking(kwargs.get("incoming"))  # type: ignore[assignment]
    return interdrone


def _message(message_id: MessageType, sender_id: int = 0) -> Message:
    return Message.create(
        id=message_id,
        drones_to_send_data=(1,),
        sender_id=sender_id,
        data={},
    )


async def _until(predicate: Callable[[], bool], interval: float = 0.01) -> None:
    """Wait for a condition. The outer wait_for is what bounds this."""
    while not predicate():
        await asyncio.sleep(interval)


@pytest.fixture(autouse=True)
def fast_emergency_land(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(emergency_land_impl, "POLL_INTERVAL_S", FAST_POLL_S)
    monkeypatch.setattr(emergency_land_impl, "MODE_CONFIRM_S", FAST_CONFIRM_S)


class TestEmergencyLandState:
    def test_commands_land_and_returns_once_disarmed(self):
        """The whole point: LAND mode, in place, and don't return until it's down."""

        async def scenario() -> None:
            vehicle = FakeVehicle(mode="GUIDED")
            state = EmergencyLand(FakeDrone(vehicle), _settings(), None)  # type: ignore[arg-type]

            landing = asyncio.ensure_future(state.run())
            await _until(lambda: vehicle.mode.name == "LAND")
            assert not landing.done(), "returned before the vehicle disarmed"

            vehicle.armed = False
            await landing

            assert vehicle.mode_commands == ["LAND"]

        _run(scenario())

    def test_commands_land_from_a_mode_the_pilot_left_it_in(self):
        """Not being in GUIDED is a reason to command LAND, not a reason to skip it."""

        async def scenario() -> None:
            vehicle = FakeVehicle(mode="STABILIZE")
            state = EmergencyLand(FakeDrone(vehicle), _settings(), None)  # type: ignore[arg-type]

            landing = asyncio.ensure_future(state.run())
            await _until(lambda: vehicle.mode.name == "LAND")
            vehicle.armed = False
            await landing

            assert vehicle.mode_commands == ["LAND"]

        _run(scenario())

    def test_leaves_a_vehicle_that_is_already_coming_down_alone(self):
        """An autopilot failsafe already has it. Fighting that mid-descent helps nobody."""

        async def scenario() -> None:
            vehicle = FakeVehicle(mode="RTL")
            state = EmergencyLand(FakeDrone(vehicle), _settings(), None)  # type: ignore[arg-type]

            landing = asyncio.ensure_future(state.run())
            await asyncio.sleep(FAST_POLL_S * 5)
            vehicle.armed = False
            await landing

            assert vehicle.mode_commands == []

        _run(scenario())

    def test_resends_a_mode_command_that_did_not_take(self):
        """DroneKit mode sets are fire-and-forget. This one has to arrive."""

        async def scenario() -> None:
            vehicle = FakeVehicle(mode="AUTO", drop_mode_sets=1)
            state = EmergencyLand(FakeDrone(vehicle), _settings(), None)  # type: ignore[arg-type]

            landing = asyncio.ensure_future(state.run())
            await _until(lambda: vehicle.mode.name == "LAND")
            vehicle.armed = False
            await landing

            assert vehicle.mode_commands == ["LAND", "LAND"]

        _run(scenario())

    def test_does_not_command_a_disarmed_vehicle(self):
        """On the ground already: leave it in the mode its operator chose."""

        async def scenario() -> None:
            vehicle = FakeVehicle(mode="GUIDED", armed=False)
            state = EmergencyLand(FakeDrone(vehicle), _settings(), None)  # type: ignore[arg-type]

            await state.run()

            assert vehicle.mode_commands == []

        _run(scenario())


class TestInterrupt:
    def test_cancels_a_running_state_and_lands(self):
        """The path that matters: mid-flight command reaches the vehicle.

        BusyState holds its atomic lock on entry, so this also covers the
        cancel arriving while a state is in a section it must finish.
        """

        async def scenario() -> None:
            vehicle = FakeVehicle()
            drone = FakeDrone(vehicle)
            settings = _settings()
            interdrone = _interdrone(drone, settings)
            busy = BusyState(drone, settings, interdrone)  # type: ignore[arg-type]
            machine = StateMachine(busy, drone, settings, interdrone)  # type: ignore[arg-type]

            machine_task = asyncio.ensure_future(machine.run())
            await _until(lambda: interdrone._current_state is busy)

            landing = EmergencyLand(drone, settings, interdrone)  # type: ignore[arg-type]
            await interdrone.interrupt_into(
                lambda: landing, label="EMERGENCY_LAND", fallback_mode="LAND"
            )

            # The state machine is flying the landing, not the interrupt path's
            # direct-to-vehicle fallback.
            assert interdrone._current_state is landing

            await _until(lambda: vehicle.mode.name == "LAND")
            vehicle.armed = False
            await machine_task
            assert vehicle.mode_commands == ["LAND"]

        _run(scenario())

    def test_commands_the_vehicle_directly_when_no_state_is_running(self):
        """A command arriving before Start, or after the machine finished, still lands."""

        async def scenario() -> None:
            vehicle = FakeVehicle()
            interdrone = _interdrone(FakeDrone(vehicle), _settings())

            def should_not_be_built() -> State:
                raise AssertionError("built a state when there was nothing to interrupt")

            await interdrone.interrupt_into(
                should_not_be_built, label="EMERGENCY_LAND", fallback_mode="LAND"
            )

            assert vehicle.mode_commands == ["LAND"]

        _run(scenario())

    def test_direct_command_survives_having_no_vehicle(self):
        """No connection is not a reason to raise inside the interdrone loop."""

        async def scenario() -> None:
            interdrone = _interdrone(FakeDrone(None), _settings())
            interdrone.command_vehicle_mode("LAND")

        _run(scenario())


class TestStateMachineRestart:
    def test_run_waits_for_a_previous_run_to_finish(self):
        """The silent-failure regression.

        `cancel_state()` returns before the run() that owned the cancelled task
        has cleared `run_task`, so a restart lands here with `run_task` still
        set. Returning early instead of waiting drops the restart with no error
        anywhere -- and the restart being dropped is a drone that doesn't land.
        """

        async def scenario() -> None:
            vehicle = FakeVehicle()
            drone = FakeDrone(vehicle)
            settings = _settings()
            interdrone = _interdrone(drone, settings)
            landing = EmergencyLand(drone, settings, interdrone)  # type: ignore[arg-type]
            machine = StateMachine(landing, drone, settings, interdrone)  # type: ignore[arg-type]

            # Stand in for a previous run still tearing down.
            stale = asyncio.ensure_future(asyncio.sleep(3600))
            machine.run_task = stale

            async def finish_teardown() -> None:
                await asyncio.sleep(0.1)
                stale.cancel()
                machine.run_task = None

            teardown = asyncio.ensure_future(finish_teardown())
            run = asyncio.ensure_future(machine.run())

            await _until(lambda: vehicle.mode.name == "LAND")
            vehicle.armed = False
            await run
            await teardown

            assert vehicle.mode_commands == ["LAND"]

        _run(scenario())


class TestMessageHandling:
    def _drive_loop(self, interdrone: Interdrone, seconds: float = 0.3) -> None:
        """Run the interdrone loop long enough to drain its scripted messages."""

        async def scenario() -> None:
            loop_task = asyncio.ensure_future(interdrone.interdrone_loop())
            await asyncio.sleep(seconds)
            loop_task.cancel()
            try:
                await loop_task
            except asyncio.CancelledError:
                pass

        _run(scenario())

    def test_emergency_land_acts_once_on_duplicates(self):
        """Duplicates are routine here, and acting on them cancels the landing.

        Drone 1 fans the command out, the client loops sends-to-self back in,
        and EMERGENCY_LAND is in the resend-on-failure set.
        """
        drone = FakeDrone(FakeVehicle())
        interdrone = _interdrone(drone, _settings([1, 2]))
        interdrone.networking = FakeNetworking(  # type: ignore[assignment]
            [_message(MessageType.EMERGENCY_LAND), _message(MessageType.EMERGENCY_LAND)]
        )
        interrupts: list[int] = []
        interdrone.start_emergency_land = lambda: interrupts.append(1)  # type: ignore[method-assign]

        self._drive_loop(interdrone)

        assert interrupts == [1]
        assert interdrone.cmd_msg is CMD_MSG.EMERGENCY_LAND
        # And fanned out to the swarm exactly once, not once per copy.
        sent: list[Message] = interdrone.networking.sent  # type: ignore[attr-defined]
        assert [m.id for m in sent] == [MessageType.EMERGENCY_LAND]

    def test_land_starts_an_orderly_landing(self):
        drone = FakeDrone(FakeVehicle())
        interdrone = _interdrone(drone, _settings([1, 2]))
        interdrone.networking = FakeNetworking([_message(MessageType.LAND)])  # type: ignore[assignment]
        interrupts: list[int] = []
        interdrone.start_land = lambda: interrupts.append(1)  # type: ignore[method-assign]

        self._drive_loop(interdrone)

        assert interrupts == [1]
        assert interdrone.cmd_msg is CMD_MSG.LAND

    def test_land_does_not_override_an_emergency_land(self):
        """A descending drone must not be pulled back up into an RTL transit."""
        drone = FakeDrone(FakeVehicle())
        interdrone = _interdrone(drone, _settings([1, 2]))
        interdrone.cmd_msg = CMD_MSG.EMERGENCY_LAND
        interdrone.networking = FakeNetworking([_message(MessageType.LAND)])  # type: ignore[assignment]
        interrupts: list[int] = []
        interdrone.start_land = lambda: interrupts.append(1)  # type: ignore[method-assign]

        self._drive_loop(interdrone)

        assert interrupts == []
        assert interdrone.cmd_msg is CMD_MSG.EMERGENCY_LAND
        sent: list[Message] = interdrone.networking.sent  # type: ignore[attr-defined]
        assert sent == []
