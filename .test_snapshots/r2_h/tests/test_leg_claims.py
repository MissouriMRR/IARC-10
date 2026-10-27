"""Leg claims: two drones that choose crossing legs at the same moment must not both fly.

Found with the moving fake vehicle of the flight harness (integration_tests/vehicle_motion.py):
with legs that take time, the side-mates (and the two sides at the middle) flew onto each other
and aborted at the 1.2 m separation floor. The planner only knows where the others WERE when it
chose a leg. A drone now claims its leg in its position reports (Drone.claim_leg); the claims
are ordered by a Lamport clock, and the later claim waits (conflictsForLeg(my_claim=...)).
"""

import asyncio
from types import SimpleNamespace

import pytest

from flight.waypoint import Waypoint
from state_machine import drone as drone_module
from state_machine.drone import Drone, LegBlocked
from state_machine.drone_state import DroneState

LAT, LON = 37.0, -91.0
M = 1.0 / 111_320.0            # one metre of latitude, in degrees


def _at(north_m, east_m=0.0):
    return (LAT + north_m * M, LON + east_m * M * 1.25)


def _peer(drone_id, at, leg_to=None, clock=0):
    state = DroneState(drone_id, "127.0.0.1")
    state.touch()
    state.set_live_position(*at, 2.0)
    state.set_leg(leg_to, clock)
    return state


def test_a_claim_is_later_than_every_claim_heard_of():
    drone = Drone(id=2)
    assert drone.claim_leg(_at(5)) == (1, 2)
    drone.observe_leg_clock(7)
    assert drone.claim_leg(_at(5)) == (8, 2), "next after the 7 it heard of"
    drone.observe_leg_clock(3)
    assert drone.leg_clock == 8, "an older clock changes nothing"
    drone.release_leg()
    assert drone.leg_to is None and drone.leg_stamp == 8, "the stamp stays: it is the last claim"


def test_a_peer_with_a_leg_occupies_the_stretch_it_will_fly():
    here, there = _at(0), _at(20)
    assert _peer(1, here).believed_occupancy(12.0) == (here, here)
    assert _peer(1, here, leg_to=there, clock=4).believed_occupancy(12.0) == (here, there)


def test_the_later_claim_waits_the_earlier_one_flies():
    me = Drone(id=2)
    crossing_leg = (_at(10, -5), _at(10, 5))
    peer_flying_north = _peer(1, _at(0), leg_to=_at(30), clock=4)

    # No claim of mine: the peer's stretch is a conflict, as before.
    assert [c.peer_id for c in me.conflictsForLeg(*crossing_leg, [peer_flying_north])] == [1]
    # My claim is later than its (5 > 4): I wait.
    assert [c.peer_id for c in
            me.conflictsForLeg(*crossing_leg, [peer_flying_north], my_claim=(5, 2))] == [1]
    # My claim is earlier (3 < 4): the peer waits for me, not the other way round.
    assert me.conflictsForLeg(*crossing_leg, [peer_flying_north], my_claim=(3, 2)) == []
    # The same clock: the lower id goes first.
    peer_same_clock = _peer(3, _at(0), leg_to=_at(30), clock=4)
    assert me.conflictsForLeg(*crossing_leg, [peer_same_clock], my_claim=(4, 2)) == []
    assert [c.peer_id for c in
            me.conflictsForLeg(*crossing_leg, [_peer(1, _at(0), _at(30), 4)], my_claim=(4, 2))] == [1]


def test_a_parked_peer_blocks_whatever_its_clock():
    """No claim = nobody is waiting for me on its behalf: it is just in the way."""
    me = Drone(id=1)
    parked = _peer(2, _at(10), leg_to=None, clock=99)
    assert [c.peer_id for c in
            me.conflictsForLeg(_at(0), _at(20), [parked], my_claim=(1, 1))] == [2]


class _Vehicle:
    def __init__(self, at):
        self.location = SimpleNamespace(global_relative_frame=SimpleNamespace(lat=at[0], lon=at[1], alt=2.0))
        self.mode = SimpleNamespace(name="GUIDED")
        self.armed = True


def test_a_claimed_leg_held_too_long_is_dropped_for_a_replan(monkeypatch):
    """The drone in the way may itself be waiting for this leg: after HOLD_REPLAN_S the leg is
    given up (LegBlocked, claim released) instead of waiting out HOLD_ABORT_S."""
    monkeypatch.setattr(drone_module, "HOLD_REPLAN_S", 0.3)
    monkeypatch.setattr(drone_module, "LEG_CLAIM_SETTLE_S", 0.0)
    drone = Drone(id=2)
    drone._vehicle = _Vehicle(_at(0))
    target = _at(20)
    drone.waypoints = [Waypoint(2, *target, name="t")]
    parked = _peer(1, _at(10), leg_to=None)             # sits on the leg
    announced = []

    async def announce():
        announced.append(drone.leg_to)

    with pytest.raises(LegBlocked):
        asyncio.run(drone.gotoWaypoint([parked], announce=announce))
    assert announced[0] == target, "the claim was sent before the check"
    assert drone.leg_to is None, "and released"


def test_without_announce_nothing_is_claimed(monkeypatch):
    """POIF (and every caller that passes no announce) keeps its old behaviour."""
    monkeypatch.setattr(drone_module, "HOLD_ABORT_S", 0.3)
    drone = Drone(id=2)
    drone._vehicle = _Vehicle(_at(0))
    drone.waypoints = [Waypoint(2, *_at(20), name="t")]
    with pytest.raises(drone_module.FormationLost):
        asyncio.run(drone.gotoWaypoint([_peer(1, _at(10))]))
    assert drone.leg_clock == 0
