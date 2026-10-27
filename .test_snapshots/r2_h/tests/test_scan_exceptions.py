"""Scan.run must not end the state machine with the drone airborne.

  - LegStalled (still GUIDED) -> Land.
  - FlightInterrupted (autopilot owns it) -> LAND mode, wait for disarm, no more commands.
  - Applies to gotoWaypoint and to the climb-retry move_to in _capture_and_mark_seen.
"""

import asyncio
from types import SimpleNamespace

import pytest

from flight.pathfinding.utils.goto import FlightInterrupted, LegStalled
from state_machine.states.impl import scan_impl
from state_machine.states.land import Land
from state_machine.states.scan import Scan


class FakeVehicle:
    def __init__(self, mode="GUIDED"):
        self.armed = True
        self.mode = SimpleNamespace(name=mode)
        self.commands = []

    def __setattr__(self, key, value):
        if key == "mode" and hasattr(self, "commands"):
            self.commands.append(value.name)
            value = SimpleNamespace(name=value.name)
        super().__setattr__(key, value)


class FakeDrone:
    id = 1
    replan_needed = None
    waypoints = [object()]
    planner = None

    def __init__(self, vehicle, exc):
        self.vehicle = vehicle
        self._exc = exc

    def time_exceeded(self, _):
        return False

    async def gotoWaypoint(self, *_, **__):
        if self._exc is not None:
            raise self._exc
        return SimpleNamespace(waypoint_id=1, lat=0.0, long=0.0)


def _scan(monkeypatch, drone):
    for name in ("update_state", "update_drone", "update_flight_settings"):
        monkeypatch.setattr(scan_impl, name, lambda *a, **k: None)
    settings = SimpleNamespace(max_flight_time=1e9, max_flight_height=100.0)
    async def _nothing():
        return None

    interdrone = SimpleNamespace(drone_states=[], report_own_position=_nothing,
                                 announce_landing=_nothing)
    return Scan(drone, settings, interdrone)


def test_leg_stalled_lands(monkeypatch):
    drone = FakeDrone(FakeVehicle(), LegStalled("stuck"))
    nxt = asyncio.run(scan_impl.run(_scan(monkeypatch, drone)))
    assert isinstance(nxt, Land)


def test_climb_retry_stall_lands(monkeypatch):
    drone = FakeDrone(FakeVehicle(), None)

    async def boom(*_a, **_k):
        raise LegStalled("climb stuck")

    monkeypatch.setattr(scan_impl, "_capture_and_mark_seen", boom)
    nxt = asyncio.run(scan_impl.run(_scan(monkeypatch, drone)))
    assert isinstance(nxt, Land)


@pytest.mark.parametrize("mode,expected", [("GUIDED", ["LAND"]), ("LAND", []), ("RTL", [])])
def test_flight_interrupted_lands_and_waits(monkeypatch, mode, expected):
    vehicle = FakeVehicle(mode)
    drone = FakeDrone(vehicle, FlightInterrupted("failsafe"))

    async def scenario():
        task = asyncio.ensure_future(scan_impl.run(_scan(monkeypatch, drone)))
        await asyncio.sleep(0.1)
        assert not task.done()  # still waiting for the autopilot to disarm
        vehicle.armed = False
        return await asyncio.wait_for(task, 5)

    assert asyncio.run(scenario()) is None
    assert vehicle.commands == expected
