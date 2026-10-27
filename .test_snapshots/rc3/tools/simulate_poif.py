"""Fly the POIF demo offline, to size the carrot without burning an AirSim run.

A real 4-drone AirSim run of the 10 laps takes as long as the demo does — 15.6
minutes at the stop-and-go pace that prompted this — which is far too slow a loop
to tune a pace parameter on. This runs the same laps as point masses in a few
seconds, driving the *real* `LockstepGovernor` and the real
`CircleProgress` tracker with real `DroneState` objects, so what is being
measured is the actual flight code and not a paraphrase of it.

What is modelled: a jerk-free position controller with ArduCopter's stock
acceleration and a commanded speed cap, Gaussian position noise on every
measurement, and message latency between drones. What is not: wind, attitude,
EKF behaviour, or packet loss. It answers "how fast do the laps go and how close
do the drones get", which is what picking `carrot_arc_m` needs.

The vehicle model is *fitted*, not assumed. `--stopgo` flies each drone
independently through the old stop-at-every-waypoint loop, and STOPGO_SETTLE_S is
set so its median leg matches the 2.78 s measured in `Logs/run_3m_seperation_2`.
An ideal double integrator covers that 1.31 m leg in 1.45 s — twice as quick as
the real copter — so an unfitted model would overstate every result here.

`--stopgo` is a **timing calibration only**, and deliberately runs no barrier: the
separation baseline is already known from the real flight (2.59 m minimum), and
reproducing it faithfully would mean modelling the autopilot's settling behaviour
far more carefully than sizing a carrot requires.

    python tools/simulate_poif.py --stopgo            # check the model's pace
    python tools/simulate_poif.py                     # the change, at the default
    python tools/simulate_poif.py --sweep             # pick carrot_arc_m
"""

from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Run as `python tools/simulate_poif.py` from anywhere in the repo.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flight.circlePath import CircleProgress, circle_waypoints  # noqa: E402
from flight.waypoint import MIN_SEPARATION_M, SEPARATION_ABORT_M, formation_floor  # noqa: E402
from state_machine.drone_state import DroneState  # noqa: E402
from state_machine.states.impl.poif_impl import (  # noqa: E402
    CARROT_ARC_M,
    CARROT_RETREAT_LEGS_PER_TICK,
    CIRCLE_RADIUS_M,
    CROSS_EPSILON,
    ENTRY_BEARING_DEG,
    NUM_LAPS,
    POINTS_PER_LAP,
    LockstepGovernor,
)

# The demo as the rules specify it.
SPACING_M = 3.0
NUM_DRONES = 4
BASE_LAT = 37.94894
BASE_LON = -91.7845611

EARTH_RADIUS_M = 6_378_137.0
_DEG_LAT_PER_M = 180.0 / (math.pi * EARTH_RADIUS_M)
_DEG_LON_PER_M = _DEG_LAT_PER_M / math.cos(math.radians(BASE_LAT))

# ArduCopter stock, since the plan leaves autopilot tuning out of scope.
# WPNAV_ACCEL is 250 cm/s^2.
WPNAV_ACCEL_M_S2 = 2.5
LEG_GROUNDSPEED_M_S = 2.0

# Velocity-tracking lag and jerk limit. Kept tight and near ArduCopter's own
# PSC_JERK_XY, because the deceleration ramp has to actually land the drone on
# the waypoint: a sluggish model overshoots, and an overshooting drone sails
# straight through the arrival tolerance without ever registering it.
VEL_TAU_S = 0.15
JERK_M_S3 = 5.0

# What move_to required before it called a waypoint reached; only --stopgo uses
# it, since the pursuit controller passes waypoints rather than arriving at them.
LEG_TOLERANCE_M = 0.35

# move_to's arrival test is pure distance, but the real vehicle is decelerating
# onto the point by then — the flight logs show a 0.35 m/s median groundspeed and
# 30% of samples under 0.1 m/s. Without this the model can "arrive" at 1.3 m/s,
# which splits leg times into a fast and a slow mode with the real 2.78 s falling
# in the gap between them, and no settle value can then match the flight.
ARRIVAL_SPEED_M_S = 0.4

# The rest of the old per-leg cost, which is software and not dynamics: this
# model flies the 1.31 m leg in 1.6 s, and the flight logs measure 2.78 s. The
# difference is the settling tail (0.73 s median spent inside the last 0.6 m) and
# up to 0.25 s of move_to's polling interval. Charging it as a slower vehicle
# would be wrong — it is the cost of *stopping*, which is exactly what the
# pursuit controller removes — so it is charged only in --stopgo mode.
#
# Fitted on the *median* leg (2.80 s here against 2.78 s flown). The real
# distribution has a long right tail this model has no mechanism for — a 13.4 s
# worst leg, mean 3.36 s — so totals here run about 15% optimistic in both modes.
# The ratio between them is the trustworthy output, not the absolute seconds.
STOPGO_SETTLE_S = 0.8

# The controller ticks at CARROT_UPDATE_HZ; the physics wants finer steps than
# that to integrate cleanly.
CONTROL_HZ = 5.0
PHYSICS_HZ = 50.0

# Roughly what the flight logs showed between a broadcast and a peer acting on
# it, once the old 0.25 s barrier poll is taken out.
DEFAULT_LATENCY_S = 0.1


def _to_latlon(east_m: float, north_m: float) -> tuple[float, float]:
    return BASE_LAT + north_m * _DEG_LAT_PER_M, BASE_LON + east_m * _DEG_LON_PER_M


def _to_metres(lat: float, lon: float) -> tuple[float, float]:
    return (lon - BASE_LON) / _DEG_LON_PER_M, (lat - BASE_LAT) / _DEG_LAT_PER_M


class _Interdrone:
    """Just enough of the interdrone object for the governor to read peers."""

    def __init__(self, drone_states: list[DroneState]) -> None:
        self.drone_states = drone_states


class _Self:
    """Stands in for the POIF state, which the governor only reads peers from."""

    def __init__(self, drone_states: list[DroneState]) -> None:
        self.interdrone = _Interdrone(drone_states)


@dataclass
class Vehicle:
    """One drone: point mass, its own progress tracker, its own view of peers."""

    drone_id: int
    centre_e: float
    centre_n: float
    carrot_arc_m: float
    rng: random.Random

    east: float = 0.0
    north: float = 0.0
    vel_e: float = 0.0
    vel_n: float = 0.0
    acc_e: float = 0.0
    acc_n: float = 0.0
    crossed: int = 0
    carrot_index: float = 0.0
    done_at: float | None = None
    settle_until: float = 0.0
    stopped_ticks: int = 0
    leg_times: list[float] = field(default_factory=list)
    _last_crossing_at: float = 0.0

    def __post_init__(self) -> None:
        centre_lat, centre_lon = _to_latlon(self.centre_e, self.centre_n)
        self.centre = (centre_lat, centre_lon)
        self.waypoints = [
            wp
            # The extra lap contributes only its index 0: the closing point that
            # makes this ten whole revolutions rather than 9.96.
            for lap in range(NUM_LAPS + 1)
            for wp in circle_waypoints(
                centre_lat,
                centre_lon,
                CIRCLE_RADIUS_M,
                drone_id=self.drone_id,
                num_points=POINTS_PER_LAP,
                lap=lap,
                start_bearing_deg=ENTRY_BEARING_DEG,
            )[: POINTS_PER_LAP if lap < NUM_LAPS else 1]
        ]
        # Start on waypoint index 0, i.e. after the entry transit, which the real
        # run flies with the old per-waypoint code and is not what is being
        # measured here.
        start = self.waypoints[0]
        self.east, self.north = _to_metres(start.lat, start.long)
        self.progress = CircleProgress(
            centre_lat,
            centre_lon,
            CIRCLE_RADIUS_M,
            POINTS_PER_LAP,
            start_bearing_deg=ENTRY_BEARING_DEG,
            max_speed_m_s=LEG_GROUNDSPEED_M_S,
            start_index=0,
        )
        self.target_e, self.target_n = self.east, self.north

    def measure(self, sigma: float) -> tuple[float, float]:
        """A noisy fix, as the EKF would hand it to us."""
        return (
            self.east + self.rng.gauss(0.0, sigma),
            self.north + self.rng.gauss(0.0, sigma),
        )

    def step_physics(self, dt: float) -> None:
        """Close on the commanded point under speed, accel and jerk limits.

        The speed cap falls off as `sqrt(2*a*d)` near the target, which is what
        makes a target only one waypoint away produce the stop-and-go profile the
        flight logs show, and a target several metres ahead produce cruise. The
        velocity lag and jerk limit are what stop it being twice as quick as the
        real thing over a short leg.
        """
        to_e = self.target_e - self.east
        to_n = self.target_n - self.north
        distance = math.hypot(to_e, to_n)

        if distance < 1e-9:
            desired_e = desired_n = 0.0
        else:
            capped = min(LEG_GROUNDSPEED_M_S, math.sqrt(2.0 * WPNAV_ACCEL_M_S2 * distance))
            desired_e = to_e / distance * capped
            desired_n = to_n / distance * capped

        want_e = (desired_e - self.vel_e) / VEL_TAU_S
        want_n = (desired_n - self.vel_n) / VEL_TAU_S
        want = math.hypot(want_e, want_n)
        if want > WPNAV_ACCEL_M_S2:
            want_e *= WPNAV_ACCEL_M_S2 / want
            want_n *= WPNAV_ACCEL_M_S2 / want

        da_e = want_e - self.acc_e
        da_n = want_n - self.acc_n
        da = math.hypot(da_e, da_n)
        budget = JERK_M_S3 * dt
        if da > budget:
            da_e *= budget / da
            da_n *= budget / da

        self.acc_e += da_e
        self.acc_n += da_n
        self.vel_e += self.acc_e * dt
        self.vel_n += self.acc_n * dt
        self.east += self.vel_e * dt
        self.north += self.vel_n * dt

    @property
    def speed(self) -> float:
        return math.hypot(self.vel_e, self.vel_n)


def simulate(
    carrot_arc_m: float,
    sigma: float,
    seed: int = 7,
    latency_s: float = DEFAULT_LATENCY_S,
    time_limit_s: float = 3600.0,
    stop_and_go: bool = False,
    trace=None,
) -> dict:
    """Fly all four drones through the laps and report what happened."""
    rng = random.Random(seed)
    final_index = NUM_LAPS * POINTS_PER_LAP

    vehicles = [
        Vehicle(
            drone_id=i + 1,
            centre_e=0.0,
            centre_n=i * SPACING_M,
            carrot_arc_m=carrot_arc_m,
            rng=random.Random(rng.randrange(1 << 30)),
        )
        for i in range(NUM_DRONES)
    ]

    # views[i][j] is drone i's DroneState for drone j.
    views: dict[int, dict[int, DroneState]] = {
        v.drone_id: {
            w.drone_id: DroneState(w.drone_id, "127.0.0.1")
            for w in vehicles
            if w.drone_id != v.drone_id
        }
        for v in vehicles
    }
    governors = {
        v.drone_id: LockstepGovernor(_Self(list(views[v.drone_id].values()))) for v in vehicles
    }

    # Seed every view with the start, as the entry transit's reached-broadcast
    # would have.
    for v in vehicles:
        for i in views:
            if i == v.drone_id:
                continue
            state = views[i][v.drone_id]
            state.last_reached_waypoint = v.waypoints[0]
            state.set_live_position(v.waypoints[0].lat, v.waypoints[0].long, 6.0)
            state.touch()

    inbox: list[tuple[float, int, int, str, object]] = []
    index_per_metre = POINTS_PER_LAP / (2.0 * math.pi * CIRCLE_RADIUS_M)
    physics_dt = 1.0 / PHYSICS_HZ
    control_every = round(PHYSICS_HZ / CONTROL_HZ)

    separations: list[float] = []
    min_separation = math.inf
    closest_pair = None
    aborts: list[str] = []
    carrots: list[float] = []
    # Once measured separation reaches SEPARATION_ABORT_M the real swarm latches
    # formation_abort and lands, so nothing past that point is a real approach —
    # tracking the minimum through it would report a collision that would never
    # have been flown.
    aborted_at: float | None = None

    now = 0.0
    tick = 0
    while now < time_limit_s and any(v.done_at is None for v in vehicles):
        if tick % control_every == 0:
            control_dt = control_every * physics_dt

            # Deliver anything whose latency has elapsed.
            due = [m for m in inbox if m[0] <= now]
            inbox = [m for m in inbox if m[0] > now]
            for _, to_id, from_id, kind, payload in due:
                state = views[to_id][from_id]
                if kind == "position":
                    state.set_live_position(payload[0], payload[1], 6.0)
                else:
                    state.last_reached_waypoint = payload
                state.touch()

            for v in vehicles:
                if v.done_at is not None:
                    continue

                lat, lon = _to_latlon(*v.measure(sigma))
                index = v.progress.update(lat, lon, control_dt)

                # Position reports go out at the control rate, as POIF's
                # _broadcast_position does.
                for other in vehicles:
                    if other.drone_id != v.drone_id:
                        inbox.append(
                            (now + latency_s, other.drone_id, v.drone_id, "position", (lat, lon))
                        )

                def _cross_to(target_index: int, v: Vehicle = v, now: float = now) -> None:
                    v.crossed = target_index
                    v.leg_times.append(now - v._last_crossing_at)
                    v._last_crossing_at = now
                    reached = v.waypoints[target_index]
                    for other in vehicles:
                        if other.drone_id != v.drone_id:
                            inbox.append(
                                (now + latency_s, other.drone_id, v.drone_id, "reached", reached)
                            )

                if stop_and_go:
                    # Arrive at the fixed next waypoint, settle, then step to the
                    # one after it — what move_to plus the blocking barrier did.
                    nxt = v.waypoints[v.crossed + 1]
                    nxt_e, nxt_n = _to_metres(nxt.lat, nxt.long)
                    # Within tolerance, or already past it. Distance alone is a
                    # trap at this sample rate: a drone doing 1.7 m/s covers a
                    # third of a metre between ticks and can step clean over the
                    # 0.35 m ball, after which it chases a waypoint behind it and
                    # falls out of the formation. The real move_to gets away with
                    # a pure distance test only because the vehicle is stopping.
                    arrived = (
                        math.hypot(nxt_e - v.east, nxt_n - v.north) <= LEG_TOLERANCE_M
                        and v.speed <= ARRIVAL_SPEED_M_S
                    ) or index >= v.crossed + 1 + CROSS_EPSILON
                    if now >= v.settle_until and arrived:
                        _cross_to(v.crossed + 1)
                        v.settle_until = now + STOPGO_SETTLE_S
                        if v.crossed >= final_index:
                            v.done_at = now
                            continue
                        nxt = v.waypoints[v.crossed + 1]
                        nxt_e, nxt_n = _to_metres(nxt.lat, nxt.long)

                    if now < v.settle_until:
                        v.stopped_ticks += 1
                        # Hold the waypoint just reached, not wherever inertia has
                        # carried us to — a target of "here" gives the controller
                        # nothing to correct back to.
                        held = v.waypoints[v.crossed]
                        v.target_e, v.target_n = _to_metres(held.lat, held.long)
                    else:
                        v.target_e, v.target_n = nxt_e, nxt_n
                    carrots.append(0.0)
                    continue

                while v.crossed < final_index and index >= v.crossed + 1 + CROSS_EPSILON:
                    _cross_to(v.crossed + 1)

                if v.crossed >= final_index:
                    v.done_at = now
                    continue

                limit, _ = governors[v.drone_id].limit(v.progress.phase, index, now)
                wanted = min(index + v.carrot_arc_m * index_per_metre, limit, float(final_index))
                v.carrot_index = max(wanted, v.carrot_index - CARROT_RETREAT_LEGS_PER_TICK, index)
                ahead = max(v.carrot_index - index, 0.0)
                carrots.append(ahead / index_per_metre)
                if ahead <= 1e-9:
                    v.stopped_ticks += 1

                target_lat, target_lon = v.progress.point_at(ahead)
                v.target_e, v.target_n = _to_metres(target_lat, target_lon)

        for v in vehicles:
            if v.done_at is None:
                v.step_physics(physics_dt)

        # True separation, not what anyone believes it to be.
        for i in range(NUM_DRONES):
            for j in range(i + 1, NUM_DRONES):
                a, b = vehicles[i], vehicles[j]
                distance = math.hypot(a.east - b.east, a.north - b.north)
                if aborted_at is not None:
                    continue
                separations.append(distance)
                if distance < min_separation:
                    min_separation = distance
                    closest_pair = (a.drone_id, b.drone_id)
                if distance < SEPARATION_ABORT_M:
                    aborted_at = now
                    aborts.append(f"d{a.drone_id}-d{b.drone_id} at {distance:.2f}m, t={now:.1f}s")
                    if trace is not None:
                        trace(now, distance, a, b, vehicles)

        now += physics_dt
        tick += 1

    legs = [t for v in vehicles for t in v.leg_times]
    control_ticks = max(sum(1 for _ in carrots), 1)
    return {
        "carrot_arc_m": carrot_arc_m,
        "sigma": sigma,
        "total_s": max((v.done_at for v in vehicles if v.done_at is not None), default=None),
        "finished": all(v.done_at is not None for v in vehicles),
        "laps_flown": min(v.crossed for v in vehicles) / POINTS_PER_LAP,
        "min_separation_m": min_separation,
        "mean_separation_m": statistics.fmean(separations),
        "closest_pair": closest_pair,
        "aborts": aborts,
        "aborted_at": aborted_at,
        "leg_median_s": statistics.median(legs) if legs else None,
        "leg_max_s": max(legs) if legs else None,
        "stopped_frac": sum(v.stopped_ticks for v in vehicles) / control_ticks,
        "mean_carrot_m": statistics.fmean(carrots) if carrots else 0.0,
    }


def _print(result: dict, stop_and_go: bool = False) -> None:
    total = result["total_s"]
    head = (
        f"stop-and-go     sigma {result['sigma']:.2f} m  |  "
        if stop_and_go
        else f"carrot {result['carrot_arc_m']:>5.2f} m  sigma {result['sigma']:.2f} m  |  "
    )
    line = (
        f"{head}total {total:>6.1f}s ({total / 60:.1f} min)  "
        f"leg med {result['leg_median_s']:.2f}s"
    )
    if not stop_and_go:
        line += (
            f"  |  min sep {result['min_separation_m']:.2f} m "
            f"(pair {result['closest_pair']})  "
            f"stopped {100 * result['stopped_frac']:.0f}%"
        )
    print(line)
    if stop_and_go:
        # No barrier runs in this mode, so the drones drift apart freely and
        # their separation means nothing. Only the pace is being measured.
        return
    if result["aborted_at"] is not None:
        print(
            f"    ABORTED at t={result['aborted_at']:.0f}s after"
            f" {result['laps_flown']:.2f} laps — {result['aborts'][0]}"
        )
    elif not result["finished"]:
        print(f"    DID NOT FINISH — only {result['laps_flown']:.2f} laps")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carrot", type=float, default=CARROT_ARC_M, help="carrot arc, metres")
    parser.add_argument(
        "--sigma",
        default="0.15,0.25,0.35",
        help="comma-separated position noise standard deviations, metres",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--latency", type=float, default=DEFAULT_LATENCY_S)
    parser.add_argument(
        "--stopgo",
        action="store_true",
        help="fly the old stop-at-every-waypoint loop instead, to check the model",
    )
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="sweep the carrot from short and cautious to long and quick",
    )
    args = parser.parse_args()

    sigmas = [float(s) for s in args.sigma.split(",")]
    spacing_floor = formation_floor(SPACING_M, CIRCLE_RADIUS_M, POINTS_PER_LAP)
    print(
        f"{NUM_DRONES} drones, {SPACING_M} m apart, {CIRCLE_RADIUS_M} m radius,"
        f" {POINTS_PER_LAP} pts/lap, {NUM_LAPS} laps"
    )
    print(
        f"formation floor {spacing_floor:.2f} m | abort {SEPARATION_ABORT_M:.2f} m |"
        f" airframe {MIN_SEPARATION_M:.2f} m\n"
    )

    carrots = [1.3, 2.0, 2.6, 3.5, 5.0, 6.5] if args.sweep else [args.carrot]
    for carrot in carrots:
        for sigma in sigmas:
            _print(
                simulate(
                    carrot,
                    sigma,
                    seed=args.seed,
                    latency_s=args.latency,
                    stop_and_go=args.stopgo,
                ),
                stop_and_go=args.stopgo,
            )
        if len(carrots) > 1:
            print()


if __name__ == "__main__":
    main()
