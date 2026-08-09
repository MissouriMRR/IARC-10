"""Regression tests for the POIF lockstep collision avoidance.

These are built around the geometry the competition actually asks for — four
drones on a line 3 m apart, each orbiting its own hover point on a 5 m radius
circle, all at the same altitude with deliberately intersecting paths — because
that geometry is what broke the previous implementation. The 2026-08-07 3 m run
deadlocked every drone at waypoint index 3 and never recovered.
"""

import math
import random

import pytest

from flight.circlePath import CircleProgress, circle_waypoints, phase_of, point_at_phase
from flight.waypoint import (
    COLLISION_RADIUS,
    FORMATION_TOLERANCE_M,
    MIN_SEPARATION_M,
    SEPARATION_ABORT_M,
    Waypoint,
    formation_floor,
    segment_distance,
)
from state_machine.drone import Drone, FormationLost
from state_machine.drone_state import DroneState
from state_machine.states.impl.poif_impl import (
    ENTRY_BEARING_DEG,
    LEAD_STOP_LEGS,
    LockstepGovernor,
    _peer_centre,
)

# The demo as the rules specify it: 10 ft of hover-line separation, a ~30 ft
# diameter circle, four vehicles.
SPACING_M = 3.0
RADIUS_M = 5.0
POINTS_PER_LAP = 24
NUM_DRONES = 4

# Somewhere on the Rolla test field, matching the coordinates in the flight logs.
BASE_LAT = 37.94894
BASE_LON = -91.7845611
_DEG_LAT_PER_M = 180.0 / (math.pi * 6_378_137.0)


def _center(drone_id: int) -> tuple[float, float]:
    """Hover point for a drone, `SPACING_M` north of the one before it."""
    return (BASE_LAT + (drone_id - 1) * SPACING_M * _DEG_LAT_PER_M, BASE_LON)


def _lap(drone_id: int) -> list[Waypoint]:
    """One lap for a drone, at the same phase the demo actually flies."""
    return circle_waypoints(
        *_center(drone_id),
        RADIUS_M,
        drone_id=drone_id,
        num_points=POINTS_PER_LAP,
        lap=0,
        start_bearing_deg=ENTRY_BEARING_DEG,
    )


def _peer(drone_id: int, reached_index: int, queued: list[Waypoint]) -> DroneState:
    """A peer that has reached `reached_index` and is flying the plan honestly."""
    state = DroneState(drone_id, "127.0.0.1")
    lap = _lap(drone_id)
    state.last_reached_waypoint = lap[reached_index]
    state.list_of_waypoints = queued
    # Sitting exactly on the point it says it reached.
    state.set_live_position(lap[reached_index].lat, lap[reached_index].long, 6.0)
    state.touch()
    return state


def _drone(drone_id: int) -> Drone:
    return Drone(id=drone_id)


class TestFormationGeometry:
    def test_floor_matches_brute_force_swept_distance(self):
        """formation_floor() is the real worst case, not a convenient formula."""
        a = _lap(1)
        b = _lap(2)
        worst = min(
            segment_distance(
                (a[k].lat, a[k].long),
                (a[(k + 1) % POINTS_PER_LAP].lat, a[(k + 1) % POINTS_PER_LAP].long),
                (b[k].lat, b[k].long),
                (b[(k + 1) % POINTS_PER_LAP].lat, b[(k + 1) % POINTS_PER_LAP].long),
            )
            for k in range(POINTS_PER_LAP)
        )
        assert formation_floor(SPACING_M, RADIUS_M, POINTS_PER_LAP) == pytest.approx(
            worst, abs=0.02
        )

    def test_three_metre_formation_clears_the_airframe(self):
        """The worst case the barrier permits still leaves the airframes clear."""
        floor = formation_floor(SPACING_M, RADIUS_M, POINTS_PER_LAP)
        assert floor > MIN_SEPARATION_M
        assert floor > SEPARATION_ABORT_M

    def test_swept_path_test_is_unsatisfiable_at_three_metres(self):
        """The premise of the fix: the old check could never pass at 3 m.

        If this ever stops being true the lockstep suppression is no longer
        load-bearing and this whole design should be revisited — so it is worth
        asserting rather than leaving as a comment.
        """
        assert formation_floor(SPACING_M, RADIUS_M, POINTS_PER_LAP) < COLLISION_RADIUS


class TestLockstepGate:
    def test_in_formation_swarm_never_gridlocks(self):
        """Every drone can depart at every index. This is the 3 m gridlock test.

        The previous implementation stopped the whole swarm at index 3 on a
        clearance that never changed, because it measured swept path overlap
        between drones the barrier was already keeping apart.
        """
        laps = {i: _lap(i) for i in range(1, NUM_DRONES + 1)}

        for me in range(1, NUM_DRONES + 1):
            drone = _drone(me)
            for departing_for in range(1, POINTS_PER_LAP):
                my_progress = departing_for - 1
                leg_from = (laps[me][my_progress].lat, laps[me][my_progress].long)
                leg_to = (laps[me][departing_for].lat, laps[me][departing_for].long)

                peers = [
                    _peer(other, my_progress, laps[other][my_progress + 1 :])
                    for other in range(1, NUM_DRONES + 1)
                    if other != me
                ]

                conflicts = drone.conflictsForLeg(
                    leg_from,
                    leg_to,
                    peers,
                    points_per_lap=POINTS_PER_LAP,
                    my_progress=my_progress,
                )
                assert conflicts == [], (
                    f"drone {me} blocked departing for index {departing_for}"
                    f" by {[(c.peer_id, c.reason, round(c.clearance_m, 2)) for c in conflicts]}"
                )

    def test_peer_one_leg_ahead_is_still_in_formation(self):
        """The barrier window is two indices wide, not one."""
        drone = _drone(1)
        mine, theirs = _lap(1), _lap(2)
        my_progress = 5

        peer = _peer(2, my_progress + 1, theirs[my_progress + 2 :])
        conflicts = drone.conflictsForLeg(
            (mine[my_progress].lat, mine[my_progress].long),
            (mine[my_progress + 1].lat, mine[my_progress + 1].long),
            [peer],
            points_per_lap=POINTS_PER_LAP,
            my_progress=my_progress,
        )
        assert conflicts == []

    def test_peer_outside_the_window_still_blocks(self):
        """A drone the barrier does not account for gets the conservative test."""
        drone = _drone(1)
        mine = _lap(1)
        my_progress = 5

        # No waypoint index at all — a drone that is not flying the formation.
        stray = DroneState(9, "127.0.0.1")
        parked = Waypoint(
            drone_id=9,
            lat=mine[my_progress + 1].lat,
            long=mine[my_progress + 1].long,
            waypoint_id=9_000_000,
        )
        stray.last_reached_waypoint = parked
        stray.list_of_waypoints = [parked]
        stray.set_live_position(parked.lat, parked.long, 6.0)
        stray.touch()

        conflicts = drone.conflictsForLeg(
            (mine[my_progress].lat, mine[my_progress].long),
            (mine[my_progress + 1].lat, mine[my_progress + 1].long),
            [stray],
            points_per_lap=POINTS_PER_LAP,
            my_progress=my_progress,
        )
        assert [c.peer_id for c in conflicts] == [9]
        assert conflicts[0].reason == "swept_path"

    def test_silent_peer_is_not_waited_for(self):
        """A peer that has stopped talking must not park the swarm forever."""
        drone = _drone(1)
        mine = _lap(1)
        my_progress = 5

        stray = DroneState(9, "127.0.0.1")
        parked = Waypoint(
            drone_id=9,
            lat=mine[my_progress + 1].lat,
            long=mine[my_progress + 1].long,
            waypoint_id=9_000_000,
        )
        stray.last_reached_waypoint = parked
        stray.list_of_waypoints = [parked]
        # Never touched, so it has been silent since forever.

        conflicts = drone.conflictsForLeg(
            (mine[my_progress].lat, mine[my_progress].long),
            (mine[my_progress + 1].lat, mine[my_progress + 1].long),
            [stray],
            points_per_lap=POINTS_PER_LAP,
            my_progress=my_progress,
        )
        assert conflicts == []


class TestFormationIntegrity:
    def _drifted_peer(self, drift_m: float, my_progress: int) -> DroneState:
        theirs = _lap(2)
        state = _peer(2, my_progress, theirs[my_progress + 1 :])
        # Push the reported position radially outward from its own circle
        # center. That is perpendicular to the chord it claims to be flying at
        # every index — displacing along a fixed compass bearing would slide it
        # *along* the leg at some indices and not register as drift at all —
        # and it is what being blown off the circle actually looks like.
        here = theirs[my_progress]
        center_lat, center_lon = _center(2)
        lon_scale = math.cos(math.radians(center_lat))
        north = (here.lat - center_lat) / _DEG_LAT_PER_M
        east = (here.long - center_lon) / _DEG_LAT_PER_M * lon_scale
        length = math.hypot(north, east)
        state.set_live_position(
            here.lat + (north / length) * drift_m * _DEG_LAT_PER_M,
            here.long + (east / length) * drift_m * _DEG_LAT_PER_M / lon_scale,
            6.0,
        )
        return state

    def test_peer_on_its_path_does_not_block(self):
        drone = _drone(1)
        mine = _lap(1)
        my_progress = 5
        peer = self._drifted_peer(0.0, my_progress)

        conflicts = drone.conflictsForLeg(
            (mine[my_progress].lat, mine[my_progress].long),
            (mine[my_progress + 1].lat, mine[my_progress + 1].long),
            [peer],
            points_per_lap=POINTS_PER_LAP,
            my_progress=my_progress,
        )
        assert conflicts == []

    def test_peer_off_its_path_blocks(self):
        """The check the barrier cannot do for itself: is the premise true?"""
        drone = _drone(1)
        mine = _lap(1)
        my_progress = 5
        peer = self._drifted_peer(FORMATION_TOLERANCE_M + 0.5, my_progress)

        conflicts = drone.conflictsForLeg(
            (mine[my_progress].lat, mine[my_progress].long),
            (mine[my_progress + 1].lat, mine[my_progress + 1].long),
            [peer],
            points_per_lap=POINTS_PER_LAP,
            my_progress=my_progress,
        )
        assert [c.reason for c in conflicts] == ["off_formation"]
        assert conflicts[0].peer_id == 2

    def test_missing_position_report_does_not_block(self):
        """Degrade to the barrier alone rather than re-inventing the deadlock."""
        drone = _drone(1)
        mine, theirs = _lap(1), _lap(2)
        my_progress = 5

        peer = DroneState(2, "127.0.0.1")
        peer.last_reached_waypoint = theirs[my_progress]
        peer.list_of_waypoints = theirs[my_progress + 1 :]
        peer.touch()  # talking, but never sent a position

        conflicts = drone.conflictsForLeg(
            (mine[my_progress].lat, mine[my_progress].long),
            (mine[my_progress + 1].lat, mine[my_progress + 1].long),
            [peer],
            points_per_lap=POINTS_PER_LAP,
            my_progress=my_progress,
        )
        assert conflicts == []


class TestSeparationAbort:
    def _drone_at(self, lat: float, lon: float) -> Drone:
        drone = _drone(1)
        drone.currentPosition = lambda: (lat, lon)  # type: ignore[method-assign]
        return drone

    def test_nominal_separation_does_not_abort(self):
        drone = self._drone_at(BASE_LAT, BASE_LON)
        peer = DroneState(2, "127.0.0.1")
        peer.set_live_position(BASE_LAT + SPACING_M * _DEG_LAT_PER_M, BASE_LON, 6.0)
        peer.touch()

        closest = drone.checkSeparation([peer])
        assert closest == pytest.approx(SPACING_M, abs=0.05)
        assert drone.formation_abort is None

    def test_breach_latches_and_raises(self):
        drone = self._drone_at(BASE_LAT, BASE_LON)
        peer = DroneState(2, "127.0.0.1")
        peer.set_live_position(
            BASE_LAT + (SEPARATION_ABORT_M - 0.3) * _DEG_LAT_PER_M, BASE_LON, 6.0
        )
        peer.touch()

        drone.checkSeparation([peer])
        assert drone.formation_abort is not None
        assert "drone 2" in drone.formation_abort

        # Latched: moving apart again does not clear it. Once two airframes have
        # been that close the demo is over.
        peer.set_live_position(BASE_LAT + 5.0 * _DEG_LAT_PER_M, BASE_LON, 6.0)
        drone.checkSeparation([peer])
        assert drone.formation_abort is not None

    def test_peer_without_position_reports_is_skipped(self):
        """An absent measurement is not a small distance."""
        drone = self._drone_at(BASE_LAT, BASE_LON)
        peer = DroneState(2, "127.0.0.1")
        peer.touch()

        assert drone.checkSeparation([peer]) is None
        assert drone.formation_abort is None


class TestEntryTransit:
    """Flying from the hover line onto the circles.

    Not covered by the lockstep barrier — no drone has reached an index yet, so
    there is nothing to synchronise on. It is governed by the entry bearing and
    by the geometric gate acting on live positions.
    """

    def _entry_legs(self, bearing: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        legs = []
        for d in range(1, NUM_DRONES + 1):
            center = _center(d)
            first = circle_waypoints(
                *center,
                RADIUS_M,
                drone_id=d,
                num_points=POINTS_PER_LAP,
                lap=0,
                start_bearing_deg=bearing,
            )[0]
            legs.append((center, (first.lat, first.long)))
        return legs

    def test_north_entry_crosses_the_neighbours_hover_point(self):
        """Documents the hazard the entry bearing exists to avoid."""
        legs = self._entry_legs(0.0)
        # Drone 1 flying to its first waypoint passes over drone 2's hover point.
        start, end = legs[0]
        neighbour = _center(2)
        assert segment_distance(start, end, neighbour, neighbour) < 0.1

    def test_east_entry_keeps_full_spacing(self):
        """Parallel tracks: every drone holds its spacing for the whole transit."""
        legs = self._entry_legs(ENTRY_BEARING_DEG)
        for i, (start, end) in enumerate(legs):
            for j, (other_start, other_end) in enumerate(legs):
                if i >= j:
                    continue
                gap = segment_distance(start, end, other_start, other_end)
                assert gap >= SPACING_M - 0.05, f"drones {i + 1} and {j + 1} only {gap:.2f}m apart"

    def test_entry_gate_sees_a_parked_neighbour(self):
        """A neighbour still sitting on its hover point must be avoided.

        This is the 0.12 m near-collision: the parked drone was modelled at the
        waypoint it had been sent to, five metres away, rather than where it was.
        """
        drone = _drone(1)
        first = circle_waypoints(
            *_center(1),
            RADIUS_M,
            drone_id=1,
            num_points=POINTS_PER_LAP,
            lap=0,
            start_bearing_deg=0.0,  # the unsafe bearing, to exercise the gate
        )[0]

        parked = DroneState(2, "127.0.0.1")
        theirs = circle_waypoints(
            *_center(2),
            RADIUS_M,
            drone_id=2,
            num_points=POINTS_PER_LAP,
            lap=0,
            start_bearing_deg=0.0,
        )
        parked.list_of_waypoints = list(theirs)  # told where to go
        parked.set_live_position(*_center(2), 6.0)  # but has not gone yet
        parked.touch()

        conflicts = drone.conflictsForLeg(
            _center(1),
            (first.lat, first.long),
            [parked],
            points_per_lap=POINTS_PER_LAP,
            my_progress=-1,
        )
        assert [c.peer_id for c in conflicts] == [2]

    def test_entry_gate_is_blind_to_a_parked_neighbour_without_reports(self):
        """Why the entry bearing, not the gate, is the fix for the entry.

        With no position report the only model of a peer is its waypoint list,
        which says it is already at its first waypoint 5 m north — 3 m clear of
        our leg. It is actually still on its hover point, squarely in the way.
        The gate cannot see that and reports no conflict, which is how two
        drones reached 0.12 m. Position reports close the gap; the eastward
        entry bearing removes the crossing altogether so nothing has to.
        """
        drone = _drone(1)
        theirs = circle_waypoints(
            *_center(2),
            RADIUS_M,
            drone_id=2,
            num_points=POINTS_PER_LAP,
            lap=0,
            start_bearing_deg=0.0,
        )
        first = circle_waypoints(
            *_center(1),
            RADIUS_M,
            drone_id=1,
            num_points=POINTS_PER_LAP,
            lap=0,
            start_bearing_deg=0.0,
        )[0]

        peer = DroneState(2, "127.0.0.1")
        peer.list_of_waypoints = list(theirs)
        peer.touch()  # no position report at all

        conflicts = drone.conflictsForLeg(
            _center(1),
            (first.lat, first.long),
            [peer],
            points_per_lap=POINTS_PER_LAP,
            my_progress=-1,
        )
        assert conflicts == []
        # And the position it is actually at *is* on our leg, which is the
        # discrepancy the whole POSITION_REPORT stream exists to expose.
        assert (
            segment_distance(_center(1), (first.lat, first.long), _center(2), _center(2))
            < MIN_SEPARATION_M
        )


class TestFormationLostIsRaised:
    def test_check_still_ours_raises_on_latched_abort(self):
        drone = _drone(1)

        class _Mode:
            name = "GUIDED"

        class _Vehicle:
            mode = _Mode()
            armed = True

        drone._vehicle = _Vehicle()  # type: ignore[assignment]
        drone._check_still_ours()  # no abort latched, returns quietly

        drone.formation_abort = "peer too close"
        with pytest.raises(FormationLost):
            drone._check_still_ours()


_M_PER_DEG_LAT = 1.0 / _DEG_LAT_PER_M


def _noisy(lat: float, lon: float, sigma: float, rng: random.Random) -> tuple[float, float]:
    """A position fix with `sigma` metres of Gaussian error on each axis."""
    scale = _DEG_LAT_PER_M
    return (
        lat + rng.gauss(0.0, sigma) * scale,
        lon + rng.gauss(0.0, sigma) * scale / math.cos(math.radians(lat)),
    )


class TestPhaseGeometry:
    """`phase_of` is the measurement the whole pursuit controller is built on."""

    def test_round_trips_with_point_at_phase(self):
        for drone_id in range(1, NUM_DRONES + 1):
            centre = _center(drone_id)
            for k in range(360):
                want = 2.0 * math.pi * k / 360.0
                lat, lon = point_at_phase(*centre, RADIUS_M, want, ENTRY_BEARING_DEG)
                got = phase_of(*centre, lat, lon, ENTRY_BEARING_DEG)
                assert abs((got - want + math.pi) % (2.0 * math.pi) - math.pi) < 1e-9

    def test_waypoint_index_lands_on_its_exact_phase(self):
        """Index i must read as exactly 2*pi*i/N, or crossings fire off-position."""
        for drone_id in range(1, NUM_DRONES + 1):
            lap = _lap(drone_id)
            for i, waypoint in enumerate(lap):
                got = phase_of(*_center(drone_id), waypoint.lat, waypoint.long, ENTRY_BEARING_DEG)
                want = 2.0 * math.pi * i / POINTS_PER_LAP
                assert abs((got - want + math.pi) % (2.0 * math.pi) - math.pi) < 1e-8

    def test_phase_ignores_radius(self):
        """A drone off its circle still has a well-defined phase."""
        centre = _center(1)
        for radius in (1.0, 3.0, RADIUS_M, 8.0):
            lat, lon = point_at_phase(*centre, radius, 1.234, ENTRY_BEARING_DEG)
            assert abs(phase_of(*centre, lat, lon, ENTRY_BEARING_DEG) - 1.234) < 1e-8

    def test_peer_centre_recovers_the_circle_from_a_waypoint(self):
        """The governor turns a peer's measured position into a phase this way."""
        for drone_id in range(1, NUM_DRONES + 1):
            lap = _lap(drone_id)
            for index in (0, 5, 12, 23):
                state = DroneState(drone_id, "127.0.0.1")
                state.last_reached_waypoint = lap[index]
                recovered = _peer_centre(state)
                assert recovered is not None
                assert (
                    segment_distance(recovered, recovered, _center(drone_id), _center(drone_id))
                    < 1e-6
                )


class TestCircleProgress:
    """Unwrapping laps, and surviving the position noise that comes with them."""

    def _tracker(self, drone_id: int = 1, start_index: int = 0) -> CircleProgress:
        return CircleProgress(
            *_center(drone_id),
            RADIUS_M,
            POINTS_PER_LAP,
            start_bearing_deg=ENTRY_BEARING_DEG,
            max_speed_m_s=2.0,
            start_index=start_index,
        )

    def _fly(self, tracker: CircleProgress, index: float, sigma=0.0, rng=None, dt=0.2):
        lat, lon = point_at_phase(
            *_center(1), RADIUS_M, index * 2.0 * math.pi / POINTS_PER_LAP, ENTRY_BEARING_DEG
        )
        if sigma:
            lat, lon = _noisy(lat, lon, sigma, rng)
        return tracker.update(lat, lon, dt)

    def test_unwraps_across_every_lap(self):
        """Phase alone cannot tell lap 0 index 23 from lap 9 index 23."""
        tracker = self._tracker()
        for step in range(1, 241):
            index = self._fly(tracker, step * 0.5)
            assert abs(index - step * 0.5) < 1e-6

    def test_seeding_near_the_seam_does_not_read_a_lap_ahead(self):
        """A drone a hair short of index 0 is at index 0, not index 24."""
        tracker = self._tracker(start_index=0)
        index = self._fly(tracker, -0.02)
        assert abs(index) < 0.5

    def test_rejects_an_impossible_jump_and_recovers(self):
        tracker = self._tracker()
        self._fly(tracker, 0.0)
        self._fly(tracker, 0.2)
        before = tracker.index

        self._fly(tracker, 6.0)  # 25 m in 0.2 s
        assert tracker.rejected == 1
        assert tracker.index == pytest.approx(before)

        assert self._fly(tracker, 0.4) == pytest.approx(0.4, abs=1e-6)

    def test_noise_does_not_bias_progress(self):
        """Summing clamped deltas would creep ahead of the truth; this must not.

        Reported progress running ahead of reality means broadcasting waypoints
        the drone has not reached, which moves every peer's barrier with it.
        """
        for sigma in (0.15, 0.25, 0.35):
            rng = random.Random(11)
            tracker = self._tracker()
            index = 0.0
            for step in range(1, 1200):
                index = self._fly(tracker, step * 0.2, sigma=sigma, rng=rng)
            assert abs(index - 1199 * 0.2) < 0.5, f"drifted at sigma={sigma}"

    def test_every_waypoint_crossing_fires_once_and_in_order(self):
        """What drives the peer messages, so a miss or a repeat desynchronises."""
        for sigma in (0.0, 0.15, 0.25, 0.35):
            rng = random.Random(5)
            tracker = self._tracker()
            crossed: list[int] = []
            last = 0
            true_index = 0.0
            while true_index < 240:
                true_index += 0.08
                index = self._fly(tracker, true_index, sigma=sigma, rng=rng)
                while index >= last + 1 + 0.05:
                    last += 1
                    crossed.append(last)
            assert crossed == list(range(1, len(crossed) + 1)), f"out of order at sigma={sigma}"
            assert len(crossed) >= 239, f"missed crossings at sigma={sigma}"

    def test_plausibility_gate_keeps_good_fixes_at_real_gps_noise(self):
        """A gate budgeting for travel but not noise throws away real data.

        At 0.25 m it discarded 13% of good fixes and at 0.35 m one in five, each
        one freezing the reported phase for a tick. A formation pacing itself on
        stale phases comes apart.
        """
        for sigma in (0.15, 0.25, 0.35):
            rng = random.Random(3)
            tracker = self._tracker()
            for step in range(1, 1000):
                self._fly(tracker, step * 0.08, sigma=sigma, rng=rng)
            assert tracker.rejected / 1000 < 0.02, f"{tracker.rejected} rejects at sigma={sigma}"


class _Swarm:
    """The peer list a governor reads, and nothing else."""

    def __init__(self, states):
        self.interdrone = type("_I", (), {"drone_states": states})()


class TestLockstepGovernor:
    """The barrier's replacement: same one-leg bound, expressed as a position."""

    def _peer_at(self, drone_id: int, index: float, reported: int | None = None) -> DroneState:
        state = DroneState(drone_id, "127.0.0.1")
        lap = _lap(drone_id)
        state.last_reached_waypoint = lap[
            int(index) % POINTS_PER_LAP if reported is None else reported
        ]
        lat, lon = point_at_phase(
            *_center(drone_id),
            RADIUS_M,
            index * 2.0 * math.pi / POINTS_PER_LAP,
            ENTRY_BEARING_DEG,
        )
        state.set_live_position(lat, lon, 6.0)
        state.touch()
        return state

    def _limit(self, peers, my_index: float, ticks: int = 40) -> float:
        """Settle the smoothing filter, then read the limit."""
        governor = LockstepGovernor(_Swarm(peers))
        my_phase = my_index * 2.0 * math.pi / POINTS_PER_LAP % (2.0 * math.pi)
        limit = 0.0
        for tick in range(ticks):
            limit, _ = governor.limit(my_phase, my_index, tick * 0.2)
        return limit

    def test_level_swarm_may_fly_a_full_leg_ahead(self):
        peers = [self._peer_at(i, 6.0) for i in (2, 3, 4)]
        assert self._limit(peers, 6.0) == pytest.approx(6.0 + LEAD_STOP_LEGS, abs=0.05)

    def test_limit_tracks_the_slowest_peer_not_the_average(self):
        peers = [self._peer_at(2, 6.0), self._peer_at(3, 5.0), self._peer_at(4, 7.0)]
        assert self._limit(peers, 6.0) == pytest.approx(5.0 + LEAD_STOP_LEGS, abs=0.05)

    def test_a_full_leg_ahead_stops_us(self):
        """The bound formation_floor assumes. Being at the limit means no carrot."""
        peers = [self._peer_at(i, 6.0) for i in (2, 3, 4)]
        limit = self._limit(peers, 6.0 + LEAD_STOP_LEGS)
        assert limit <= 6.0 + LEAD_STOP_LEGS + 0.05

    def test_stale_peer_is_ignored(self):
        """A crashed drone must not park the swarm forever."""
        stale = self._peer_at(2, 0.0)
        stale._last_heard_monotonic = None  # never heard from
        assert self._limit([stale], 6.0) == math.inf

    def test_flying_alone_is_unconstrained(self):
        assert self._limit([], 6.0) == math.inf

    def test_live_peer_that_has_never_reported_stops_us(self):
        """It could be anywhere on its circle, including alongside us."""
        blank = DroneState(2, "127.0.0.1")
        blank.touch()
        assert self._limit([blank], 6.0) == -math.inf

    def test_limit_is_smooth_under_position_noise(self):
        """The defect this design replaced: a limit that chatters is chased.

        Scaling speed by the headroom left made the commanded point snap between
        "here" and a full carrot ahead at the tick rate whenever the lead sat
        near the threshold, and a drone chasing that swings wide of its circle.
        Radial error, not index skew, is what then eats the separation.
        """
        rng = random.Random(17)
        governor = LockstepGovernor(_Swarm([self._peer_at(i, 6.0) for i in (2, 3, 4)]))
        my_phase = 6.0 * 2.0 * math.pi / POINTS_PER_LAP

        limits = []
        for tick in range(200):
            for state in governor.state.interdrone.drone_states:
                lat, lon = point_at_phase(
                    *_center(state.drone_id),
                    RADIUS_M,
                    my_phase,
                    ENTRY_BEARING_DEG,
                )
                state.set_live_position(*_noisy(lat, lon, 0.25, rng), 6.0)
                state.touch()
            limit, _ = governor.limit(my_phase, 6.0, tick * 0.2)
            limits.append(limit)

        steady = limits[40:]
        jitter = max(steady) - min(steady)
        assert jitter < 0.6, f"limit swung {jitter:.2f} legs on a stationary formation"
