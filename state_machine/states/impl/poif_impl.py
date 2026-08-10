import asyncio
import logging
import math
import time
from collections import deque

from flight.circlePath import CircleProgress, circle_waypoints, phase_of, point_at_phase
import flight.flight_log as flight_log
from flight.pathfinding.utils.goto import (
    ALTITUDE_TOLERANCE_M,
    STALL_EPSILON_M,
    STALL_TIMEOUT_S,
    FlightInterrupted,
    LegStalled,
)
from flight.waypoint import (
    MIN_SEPARATION_M,
    SEPARATION_ABORT_M,
    Waypoint,
    formation_floor,
    segment_distance,
)

# state_machine.drone patches the collections aliases dronekit needs on
# import, so it must come before dronekit.
from state_machine.drone import (
    HOLD_ABORT_S,
    LEG_ALTITUDE_M,
    LEG_GROUNDSPEED_M_S,
    LEG_MAX_TOLERANCE_M,
    PEER_STALE_S,
    FormationLost,
    _describe_conflict,
)
import dronekit
from state_machine.state_tracker import (
    update_state,
    update_drone,
    update_flight_settings,
)
from state_machine.states.land import Land
from state_machine.states.poif import POIF

# POIF requires a circle *diameter* of ~30 ft (~10 m). Altitude lives in
# state_machine/drone.py as LEG_ALTITUDE_M.
CIRCLE_RADIUS_M: float = 5.0
NUM_LAPS: int = 10
# ~1.3 m between waypoints: well above the 0.5 m arrival tolerance, short enough
# that a conflict can't develop mid-leg.
POINTS_PER_LAP: int = 24
# How far ahead we publish, and so how far ahead peers can avoid us.
LOOKAHEAD: int = 5

# Compass bearing of every drone's first leg off its hover point. The drones
# hover on a north-south line, so an eastward entry sends all four off on
# parallel tracks a full hover-spacing apart.
#
# Due north (the old value) ran each entry leg straight over its northern
# neighbour's hover point — drones 2 and 3 closed to 0.12 m that way on
# 2026-08-07. Entry only: once on the circles the phase is common to all of
# them, so the formation is identical whichever bearing it started from.
ENTRY_BEARING_DEG: float = 90.0

# How far ahead of the drone the commanded point is held, in metres of arc along
# the circle. This is the pace control for the whole demo.
#
# `simple_goto` commands a point the autopilot *stops* at. Aiming it at the next
# waypoint 1.31 m away means the copter accelerates and decelerates 240 times and
# never exceeds ~1.3 m/s: the 2026-08-08 run averaged 0.47 m/s and took 15.6
# minutes, with 84% of that inside the legs and only 16% at the barrier. Holding
# the target a few metres ahead instead gives the autopilot a distant stop point,
# so it reaches and holds LEG_GROUNDSPEED_M_S and never stops at all.
#
# Larger is faster but cuts the corner harder: a straight line to a point `a`
# metres of arc ahead pulls the path inside the circle by
# R*(1-cos(a/R)) — 0.17 m at 2.6 m. Every drone cuts identically, so the
# formation is unaffected; only the circle's effective diameter shrinks.
CARROT_ARC_M: float = 2.6

# Control rate for the pursuit loop. Matches POSITION_REPORT_HZ: there is no new
# information about the swarm between position reports.
CARROT_UPDATE_HZ: float = 5.0

# How far ahead of the furthest-behind peer this drone may get, in waypoint
# indices, before the carrot is fully collapsed and it stops. This is the same
# one-leg skew bound the old blocking barrier enforced, and it is what
# `formation_floor` assumes — do not raise it without redoing that arithmetic.
LEAD_STOP_LEGS: float = 1.0

# Exponential smoothing on each peer's measured lead, per control tick. One 5 Hz
# fix carries roughly a fifth of a leg of noise at real-GPS accuracy; acting on it
# unsmoothed makes the limit jitter and the drone chase it.
LEAD_SMOOTHING: float = 0.4

# How old a peer's position report is by the time we compare against it: one
# POSITION_REPORT_HZ interval of sampling plus the trip over the link.
#
# It matters because the comparison is otherwise dishonest. Our own position is
# measured now and the peer's is a third of a second stale, so at 2 m/s every
# drone reads itself as a third of a leg ahead of every peer — simulation shows a
# mean lead of 0.285 legs in a formation flying perfectly in step. That is in the
# safe direction (everyone throttles) but it spends a third of the one-leg margin
# before noise is accounted for at all, and the margin left is then thin enough
# for noise to push drones into the stop and destabilise the formation. So we
# compare the peer's fix against where *we* were when it was taken.
PEER_REPORT_LAG_S: float = 0.3

# How far the commanded point may slide back along the circle in one tick. It
# still stops the drone promptly — a leg and a half a second — while turning a
# discontinuous "stop now" into a deceleration the vehicle can actually fly.
CARROT_RETREAT_LEGS_PER_TICK: float = 0.3

# How far past a waypoint the drone must be before its crossing is broadcast.
# Guards against position noise re-triggering the same index.
CROSS_EPSILON: float = 0.05

# How often a drone stopped by the governor re-broadcasts its last reached
# waypoint. Must stay well under PEER_STALE_S or a waiting drone looks dead.
SYNC_HEARTBEAT_S: float = 4.0

# How often we tell the swarm where we actually are. Fast enough that a drone at
# full airspeed moves well under the separation margin between reports.
POSITION_REPORT_HZ: float = 5.0

# Startup wait for the peer waypoints the feasibility check needs. They arrive in
# tens of milliseconds on a healthy network.
PEER_PLAN_TIMEOUT_S: float = 5.0


async def _assert_formation_feasible(self: POIF) -> None:
    """Refuse to start the circles if the formation cannot be flown safely.

    A separation threshold larger than the clearance the formation can produce
    is unsatisfiable by any amount of holding, and it's a property of the
    numbers, not the flight — so catch it in a stable hover rather than 20
    seconds into the laps.

    All four circles are congruent and same-phase, so the distance between two
    drones' index-0 waypoints is the spacing between their circle centers.
    """
    deadline = time.monotonic() + PEER_PLAN_TIMEOUT_S
    while time.monotonic() < deadline:
        if all(
            state.list_of_waypoints or state.is_stale(PEER_STALE_S)
            for state in self.interdrone.drone_states
        ):
            break
        await asyncio.sleep(0.1)

    mine = self.drone.waypoints[0]
    spacings: list[float] = []
    for state in self.interdrone.drone_states:
        if state.is_stale(PEER_STALE_S) or not state.list_of_waypoints:
            continue
        theirs = state.list_of_waypoints[0]
        spacings.append(
            segment_distance(
                (mine.lat, mine.long),
                (mine.lat, mine.long),
                (theirs.lat, theirs.long),
                (theirs.lat, theirs.long),
            )
        )

    if not spacings:
        # Flying alone, or nobody answered — no formation to be infeasible.
        flight_log.event("formation_check", spacing_m=None, peers=0)
        return

    spacing = min(spacings)
    floor = formation_floor(spacing, CIRCLE_RADIUS_M, POINTS_PER_LAP)
    budget = floor - 2 * LEG_MAX_TOLERANCE_M
    flight_log.event(
        "formation_check",
        spacing_m=spacing,
        floor_m=floor,
        budget_m=budget,
        abort_m=SEPARATION_ABORT_M,
        airframe_m=MIN_SEPARATION_M,
        peers=len(spacings),
    )

    if floor <= SEPARATION_ABORT_M:
        raise FormationLost(
            f"formation is not flyable: circle centers are {spacing:.2f}m apart, so"
            f" the worst case the lockstep barrier allows is {floor:.2f}m, which is"
            f" already under the {SEPARATION_ABORT_M:.2f}m separation abort."
            " Increase the spacing, increase POINTS_PER_LAP, or lower"
            " SEPARATION_ABORT_M — but the last one only if the airframe allows it."
        )
    if floor <= MIN_SEPARATION_M:
        raise FormationLost(
            f"formation is not flyable: worst-case separation {floor:.2f}m at"
            f" {spacing:.2f}m spacing is inside the {MIN_SEPARATION_M:.2f}m airframe."
        )
    if budget < MIN_SEPARATION_M:
        # True at the competition's 3 m spacing: 1.69 m worst case minus two
        # 0.6 m arrival allowances leaves 0.49 m, under the 0.81 m airframe. It
        # takes both drones a full leg out of step *and* both at the edge of
        # tolerance, so the measured-separation abort covers it instead.
        logging.warning(
            "Formation is tight: %.2fm spacing gives a %.2fm worst case, and two"
            " arrival allowances of %.2fm leave %.2fm against a %.2fm airframe."
            " Nominal separation is %.2fm; the %.2fm measured-separation abort is"
            " what covers the difference.",
            spacing,
            floor,
            LEG_MAX_TOLERANCE_M,
            budget,
            MIN_SEPARATION_M,
            spacing,
            SEPARATION_ABORT_M,
        )


async def _broadcast_position(self: POIF, peers: tuple[int, ...]) -> None:
    """Stream this drone's measured position to the swarm until cancelled.

    Runs for the whole demo, independent of the mission loop. Without it, peers
    avoid us using only our waypoint messages — which say where we intend to be,
    and keep saying it after that stops being true.
    """
    interval = 1.0 / POSITION_REPORT_HZ
    while True:
        try:
            position = self.drone.vehicle.location.global_relative_frame
            await self.interdrone.report_position(peers, position.lat, position.lon, position.alt)
            # Also checked in the mission loop; here a breach is caught within a
            # report interval no matter what the loop is doing.
            self.drone.checkSeparation(self.interdrone.drone_states)
        except asyncio.CancelledError:
            raise
        except Exception as ex:
            # A dropped packet must not end the stream — peers drop us from
            # their avoidance after PEER_STALE_S. Log it and keep reporting.
            logging.warning("Position report failed, continuing: %s", ex)
        await asyncio.sleep(interval)


def _peer_centre(state) -> tuple[float, float] | None:
    """Where this peer's circle is centred, from any waypoint it has reported.

    Every drone flies the same congruent, same-phase circle, so a waypoint at
    index `i` sits `CIRCLE_RADIUS_M` from its centre on bearing
    `ENTRY_BEARING_DEG + 2*pi*i/POINTS_PER_LAP`. Stepping half a turn back along
    that bearing recovers the centre — which is what turns a peer's *measured*
    position into a phase we can compare against our own.
    """
    waypoint = state.last_reached_waypoint
    if waypoint is None or waypoint.index is None:
        return None
    phase = 2.0 * math.pi * waypoint.index / POINTS_PER_LAP
    return point_at_phase(
        waypoint.lat,
        waypoint.long,
        CIRCLE_RADIUS_M,
        phase + math.pi,
        ENTRY_BEARING_DEG,
    )


def _peer_lead_legs(my_phase: float, my_index: float, state) -> float | None:
    """How many waypoint indices ahead of this peer we are.

    Prefers the peer's *measured* position, which gives a continuous answer good
    to position noise. Falls back to the waypoint index it last reported
    reaching, which lags its true progress by up to a full leg and so overstates
    our lead — the conservative direction, and exactly the bound the old blocking
    barrier used.

    Returns None when the peer has told us nothing we can locate it with.
    """
    centre = _peer_centre(state)
    position = state.live_position

    if centre is not None and position is not None and not state.position_is_stale(PEER_STALE_S):
        their_phase = phase_of(*centre, *position, ENTRY_BEARING_DEG)
        # Shortest way round. The barrier holds skew far under half a lap, so
        # there is no ambiguity about who is ahead.
        gap = (my_phase - their_phase + math.pi) % (2.0 * math.pi) - math.pi
        return gap * POINTS_PER_LAP / (2.0 * math.pi)

    reached = state.last_reached_global_index(POINTS_PER_LAP)
    if reached is None:
        return None
    return my_index - reached


class LockstepGovernor:
    """Bounds how far round the circle this drone may fly, given the swarm.

    This is the demo's actual collision-avoidance strategy, and it enforces the
    same guarantee the old blocking barrier did: no drone gets more than
    LEAD_STOP_LEGS ahead of the furthest-behind live peer. All drones fly
    congruent, same-phase circles offset by their hover-line spacing, so while
    everyone is at the same phase the vector between any two of them equals the
    offset between their circle centres — the paths intersect, but the drones are
    never at the intersection together.

    It answers with a **position limit**, not a speed. Scaling speed by how much
    headroom is left is the obvious formulation and it chatters badly: the lead
    is measured from noisy positions, so whenever the true lead sits near the
    threshold the estimate flips either side of it and the commanded point snaps
    between "here" and "a full carrot ahead" at the tick rate. Chasing a target
    that jumps metres backwards and forwards throws the drone off its circle, and
    it is *radial* error, not index skew, that then eats the separation — 1.7 m
    of it in simulation at 0.25 m position noise, worse than the barrier this
    replaces. A limit degrades gracefully instead: the carrot shortens as the
    limit approaches, so the drone decelerates smoothly and stops exactly on it,
    and noise moves the limit by centimetres rather than switching a decision.

    Per-peer leads are smoothed as well, because a single 5 Hz fix is a poor
    estimate of anything. Smoothing each peer separately and taking the worst
    afterwards also avoids the bias in taking a maximum over noisy values.

    Stale peers are excluded so a crashed drone can't stall the formation.
    """

    def __init__(self, state: POIF) -> None:
        self.state = state
        self._smoothed: dict[int, float] = {}
        # Our own recent phases, so a peer's stale fix can be compared against
        # where we were at the time rather than where we are now.
        self._history: deque[tuple[float, float]] = deque()

    def _phase_when_peer_measured(self, now: float, my_phase: float) -> float:
        """Our own phase at about the time the peers' latest fixes were taken."""
        self._history.append((now, my_phase))
        while len(self._history) > 1 and now - self._history[0][0] > 2 * PEER_REPORT_LAG_S:
            self._history.popleft()

        wanted = now - PEER_REPORT_LAG_S
        for timestamp, phase in self._history:
            if timestamp >= wanted:
                return phase
        return my_phase

    def limit(self, my_phase: float, my_index: float, now: float) -> tuple[float, list[dict]]:
        """The furthest waypoint index we may currently fly to.

        Returns that limit and a per-peer record for the log. `math.inf` means
        nothing is holding us back.
        """
        worst: float | None = None
        detail: list[dict] = []
        then = self._phase_when_peer_measured(now, my_phase)

        for peer in self.state.interdrone.drone_states:
            if peer.is_stale(PEER_STALE_S):
                self._smoothed.pop(peer.drone_id, None)
                detail.append({"peer": peer.drone_id, "stale": True, "lead": None})
                continue

            raw = _peer_lead_legs(then, my_index, peer)
            if raw is None:
                # Live, but it has never told us where it is. It could be
                # anywhere on its circle, including alongside us.
                detail.append({"peer": peer.drone_id, "stale": False, "lead": None})
                worst = math.inf
                continue

            previous = self._smoothed.get(peer.drone_id)
            lead = raw if previous is None else previous + LEAD_SMOOTHING * (raw - previous)
            self._smoothed[peer.drone_id] = lead
            detail.append({"peer": peer.drone_id, "stale": False, "lead": lead, "lead_raw": raw})

            if worst is None or lead > worst:
                worst = lead

        if worst is None:
            # Flying alone. Nothing to stay in step with.
            return math.inf, detail
        if worst == math.inf:
            return -math.inf, detail

        # worst = my_index - slowest_index, so this is slowest_index + the slack.
        return my_index - worst + LEAD_STOP_LEGS, detail


async def _extend_window(self: POIF, remaining: list[Waypoint], peers: tuple[int, ...]) -> None:
    """Publish one more waypoint, keeping the lookahead window full.

    Peers avoid us using the path we have advertised, so the window has to refill
    as we consume it or their picture of where we are going runs out.
    """
    if not remaining:
        return
    nextWaypoint = remaining.pop(0)
    self.drone.updateWaypoints([nextWaypoint])
    # Announce the waypoint we just added, not the one we just left, and send it
    # once — send_new_waypoints already fans out to every peer.
    await self.interdrone.send_new_waypoints(peers, [nextWaypoint])
    flight_log.event("waypoint_broadcast", waypoint=flight_log.waypoint_brief(nextWaypoint))
    for drone in self.interdrone.drone_states:
        self.drone.checkForCollision(drone.list_of_waypoints)


async def _fly_circles(
    self: POIF,
    centre: tuple[float, float],
    remaining: list[Waypoint],
    peers: tuple[int, ...],
    heartbeat,
    report_reached,
) -> None:
    """Fly the laps continuously, chasing a point held ahead on the circle.

    The vehicle is never commanded to a waypoint. It is commanded to a point
    further round the circle, recomputed every tick from where the drone actually
    is, so the autopilot always has a distant target to hold speed towards and
    never decelerates into a stop. Waypoints are *passed*, not arrived at:
    crossing one is what triggers the same peer messages the
    stop-at-every-waypoint loop used to send on arrival.

    Everything that could stop the drone does it by pulling that point back — the
    lockstep governor, a leg conflict, a stall. `carrot_arc_m` is only the cap;
    in formation the governor's one-leg bound is usually what decides, so it is
    the lockstep slack rather than the carrot that sets the pace.

    Note the commanded point always sits *on the circle*, at the drone's own
    phase when everything is pulling it back. Targeting the drone's literal
    position instead would let it stop wherever inertia left it, and a drone
    parked off its circle has no separation guarantee at all — the whole argument
    is that congruent circles at a common phase stay a fixed distance apart.
    """
    carrot_arc_m = self.flight_settings.carrot_arc_m
    if carrot_arc_m is None:
        carrot_arc_m = CARROT_ARC_M
    progress = CircleProgress(
        *centre,
        CIRCLE_RADIUS_M,
        POINTS_PER_LAP,
        start_bearing_deg=ENTRY_BEARING_DEG,
        max_speed_m_s=LEG_GROUNDSPEED_M_S,
        start_index=0,
    )
    # Arc length and waypoint index are the same measurement in different units.
    index_per_metre = POINTS_PER_LAP / (2.0 * math.pi * CIRCLE_RADIUS_M)
    interval = 1.0 / CARROT_UPDATE_HZ
    # The closing waypoint, one full lap-count round from index 0.
    final_index = NUM_LAPS * POINTS_PER_LAP

    governor = LockstepGovernor(self)
    carrot_index = 0.0  # where the commanded point currently sits, in indices
    crossed = 0  # index of the last waypoint we have passed and broadcast
    now = time.monotonic()
    last_tick = now
    last_crossing_at = now
    last_heartbeat = now
    stopped_since: float | None = None
    stop_reason: str | None = None
    blocking: set[int] = set()
    stall_best = 0.0
    stall_since = now
    recommanded = False

    # The last waypoint is flown by gotoWaypoint, which stops on it properly —
    # you cannot *pass* the point you are meant to finish on.
    while len(self.drone.waypoints) > 1:
        await asyncio.sleep(interval)
        now = time.monotonic()
        dt, last_tick = now - last_tick, now

        self.drone._check_still_ours()
        self.drone.checkSeparation(self.interdrone.drone_states)

        position = self.drone.vehicle.location.global_relative_frame
        index = progress.update(position.lat, position.lon, dt)
        altitude_error = abs(position.alt - LEG_ALTITUDE_M)

        crossing = False
        while crossed < final_index and index >= crossed + 1 + CROSS_EPSILON:
            crossed += 1
            crossing = True
            reached = self.drone.waypoints.pop(0)
            reached.has_visited = True
            at = self.drone.currentPosition()
            flight_log.event(
                "wp_reached",
                target=flight_log.waypoint_brief(reached),
                at=at,
                # How far off the circle we were as we went past — the continuous
                # equivalent of the old arrival error.
                error_m=segment_distance(
                    at, at, (reached.lat, reached.long), (reached.lat, reached.long)
                ),
                leg_seconds=now - last_crossing_at,
                alt_error_m=altitude_error,
            )
            last_crossing_at = now
            logging.info(f"Passed waypoint {reached.waypoint_id}")
            await report_reached(reached)
            await _extend_window(self, remaining, peers)
            if len(self.drone.waypoints) <= 1:
                break

        if len(self.drone.waypoints) <= 1:
            break

        # Where the carrot would go unimpeded, then everything that may pull it
        # back: the swarm, and the end of the plan.
        limit, peer_detail = governor.limit(progress.phase, index, now)
        wanted = min(index + carrot_arc_m * index_per_metre, limit, float(final_index))
        # The commanded point may run forward freely but only ease back: a target
        # that snaps backwards along the circle is a command to turn round, and
        # the drone answers it by swinging wide of the circle it is supposed to
        # be flying. Radial error is what costs separation here, not index skew.
        carrot_index = max(wanted, carrot_index - CARROT_RETREAT_LEGS_PER_TICK, index)
        allowed = max(carrot_index - index, 0.0) / index_per_metre

        heartbeat_due = stopped_since is not None and now - last_heartbeat >= SYNC_HEARTBEAT_S
        nextWaypoint = self.drone.waypoints[0]
        conflicts = self.drone.conflictsForLeg(
            self.drone.currentPosition(),
            (nextWaypoint.lat, nextWaypoint.long),
            self.interdrone.drone_states,
            log_reason="pre_leg" if crossing else ("hold_heartbeat" if heartbeat_due else None),
            points_per_lap=POINTS_PER_LAP,
            my_progress=crossed,
        )
        if conflicts:
            # A conflict is a safety stop, so it is not eased into like the
            # governor's limit. Move the commanded point itself rather than just
            # the distance, or it stays parked out ahead and springs back the
            # moment the conflict clears.
            carrot_index = index
            allowed = 0.0

        if allowed <= 0.0:
            reason = "conflict" if conflicts else "lockstep"
            if stopped_since is None:
                stopped_since = now
                stop_reason = reason
                blocking = {c.peer_id for c in conflicts}
                # Two causes, two event names, so the flight report can still
                # tell "waiting for the formation" from "the leg is not clear".
                if conflicts:
                    flight_log.event(
                        "hold_start",
                        target=flight_log.waypoint_brief(nextWaypoint),
                        conflicts=[c._asdict() for c in conflicts],
                    )
                else:
                    flight_log.event(
                        "sync_wait",
                        target=flight_log.waypoint_brief(nextWaypoint),
                        behind=peer_detail,
                    )
            elif now - stopped_since >= HOLD_ABORT_S:
                held = now - stopped_since
                flight_log.event(
                    "hold_abort",
                    held_for=held,
                    reason=reason,
                    target=flight_log.waypoint_brief(nextWaypoint),
                    conflicts=[c._asdict() for c in conflicts],
                    peers=peer_detail,
                )
                raise FormationLost(
                    f"stopped {held:.0f}s short of waypoint {nextWaypoint.waypoint_id}"
                    f" ({reason}): "
                    + (
                        "; ".join(_describe_conflict(c) for c in conflicts)
                        if conflicts
                        else f"peers {peer_detail}"
                    )
                )

            still_blocking = {c.peer_id for c in conflicts}
            if conflicts and still_blocking != blocking:
                flight_log.event(
                    "hold_blockers_changed",
                    was=sorted(blocking),
                    now=sorted(still_blocking),
                    held_for=now - stopped_since,
                )
                blocking = still_blocking

            if heartbeat_due:
                last_heartbeat = now
                # Stay visibly alive while stopped, or peers drop us as stale and
                # fly through our position.
                await heartbeat()
                flight_log.event(
                    "hold_tick",
                    held_for=now - stopped_since,
                    reason=reason,
                    conflicts=[c._asdict() for c in conflicts],
                    peers=peer_detail,
                )
        elif stopped_since is not None:
            flight_log.event(
                "hold_end" if stop_reason == "conflict" else "sync_clear",
                held_for=now - stopped_since,
                waited=now - stopped_since,
                target=flight_log.waypoint_brief(nextWaypoint),
            )
            stopped_since = None
            stop_reason = None
            blocking = set()

        # Not making progress while nothing is holding us back means the vehicle
        # is not tracking the command. Same re-command-once logic as move_to.
        making_progress = index > stall_best + STALL_EPSILON_M * index_per_metre
        if allowed <= 0.0:
            stall_since = now
            stall_best = index
            recommanded = False
        elif making_progress and altitude_error <= ALTITUDE_TOLERANCE_M:
            stall_best = index
            stall_since = now
            recommanded = False
        elif now - stall_since >= STALL_TIMEOUT_S:
            if not recommanded:
                recommanded = True
                logging.warning(
                    "Stalled at index %.2f (%.2fm off altitude) with %.2fm of carrot;"
                    " re-commanding",
                    index,
                    altitude_error,
                    allowed,
                )
            else:
                raise LegStalled(
                    f"stopped advancing at waypoint index {index:.2f} with"
                    f" {allowed:.2f}m of carrot available and {altitude_error:.2f}m of"
                    f" altitude error, and did not resume after re-commanding"
                )

        target_lat, target_lon = progress.point_at(allowed * index_per_metre)
        self.drone.commandPoint(
            target_lat,
            target_lon,
            LEG_ALTITUDE_M,
            groundspeed=LEG_GROUNDSPEED_M_S,
            force=recommanded and not making_progress,
        )


async def run(self: POIF) -> None:
    """
    Implements the run method for the POIF state.

    Flies NUM_LAPS laps of a CIRCLE_RADIUS_M circle around this drone's hover
    position, in waypoint-index lockstep with the rest of the swarm, then
    transitions to Land. If the autopilot takes over mid-demo (EKF failsafe,
    pilot takeover), the mission loop stops immediately so peers aren't fed
    phantom progress, and the state machine moves on to Land.
    """
    telemetry: flight_log.PositionLogger | None = None
    position_broadcast: asyncio.Task | None = None
    try:
        update_state("POIF")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("POIF state running")

        location = (
            self.drone.vehicle.location.global_relative_frame.lat,
            self.drone.vehicle.location.global_relative_frame.lon,
        )

        telemetry = flight_log.PositionLogger(self.drone, self.interdrone.drone_states)
        telemetry.start()

        circleWaypoints = []
        for lap in range(NUM_LAPS):
            circleWaypoints.extend(
                circle_waypoints(
                    *location,
                    CIRCLE_RADIUS_M,
                    drone_id=self.drone.id,
                    num_points=POINTS_PER_LAP,
                    lap=lap,
                    start_bearing_deg=ENTRY_BEARING_DEG,
                )
            )
        # NUM_LAPS * POINTS_PER_LAP points span 0 to 358.75 degrees, so flying all
        # of them is 9.96 revolutions, not the 10 the rules require. Close the
        # circle with the point the drones started from.
        circleWaypoints.extend(
            circle_waypoints(
                *location,
                CIRCLE_RADIUS_M,
                drone_id=self.drone.id,
                num_points=POINTS_PER_LAP,
                lap=NUM_LAPS,
                start_bearing_deg=ENTRY_BEARING_DEG,
            )[:1]
        )

        flight_log.event(
            "poif_plan",
            center=location,
            radius_m=CIRCLE_RADIUS_M,
            laps=NUM_LAPS,
            points_per_lap=POINTS_PER_LAP,
            lookahead=LOOKAHEAD,
            peers=[state.drone_id for state in self.interdrone.drone_states],
            waypoints=flight_log.waypoints_brief(circleWaypoints),
        )

        peers = tuple(self.flight_settings.other_drones_in_mission)

        # Latest waypoint we reported reaching, re-broadcast as a heartbeat
        # while parked. Peers drop drones silent for PEER_STALE_S.
        last_reached: Waypoint | None = None

        async def heartbeat() -> None:
            if last_reached is not None:
                await self.interdrone.reached_waypoint(peers, last_reached)
                flight_log.event(
                    "liveness_heartbeat",
                    waypoint=flight_log.waypoint_brief(last_reached),
                )

        async def report_reached(waypoint: Waypoint) -> None:
            nonlocal last_reached
            last_reached = waypoint
            await self.interdrone.reached_waypoint(peers, waypoint)
            flight_log.event("reached_broadcast", waypoint=flight_log.waypoint_brief(waypoint))

        self.drone.updateWaypoints(circleWaypoints[:LOOKAHEAD])

        await self.interdrone.send_new_waypoints(peers, circleWaypoints[:LOOKAHEAD])
        circleWaypoints = circleWaypoints[LOOKAHEAD:]
        for state in self.interdrone.drone_states:
            self.drone.checkForCollision(state.list_of_waypoints)

        # Before flying anywhere: peers' avoidance needs a position from us, and
        # the feasibility check below needs theirs.
        position_broadcast = asyncio.create_task(_broadcast_position(self, peers))
        await _assert_formation_feasible(self)

        # Entry transit. The drones hover at their circle *centres*, so the first
        # leg is a 5 m radial run out to index 0 rather than an arc, and phase is
        # meaningless until it is done. It is also the one leg the lockstep
        # barrier never covered: what makes it safe is ENTRY_BEARING_DEG putting
        # all four on parallel tracks a full spacing apart, with the geometric
        # gate in gotoWaypoint watching live positions. So it keeps flying the
        # per-waypoint way.
        curWaypoint = await self.drone.gotoWaypoint(
            self.interdrone.drone_states,
            heartbeat=heartbeat,
            points_per_lap=POINTS_PER_LAP,
        )
        await report_reached(curWaypoint)
        await _extend_window(self, circleWaypoints, peers)

        # The laps: one continuous pursuit loop, down to the final waypoint.
        await _fly_circles(self, location, circleWaypoints, peers, heartbeat, report_reached)

        # And stop on the last point. The old loop exited as soon as it ran out
        # of *unpublished* waypoints, abandoning the LOOKAHEAD still queued — so
        # it finished on lap 9 index 19, four short, flying 9.79 of the 10
        # revolutions the rules require.
        curWaypoint = await self.drone.gotoWaypoint(
            self.interdrone.drone_states,
            heartbeat=heartbeat,
            points_per_lap=POINTS_PER_LAP,
        )
        await report_reached(curWaypoint)

        flight_log.event("poif_complete")
        return Land(self.drone, self.flight_settings, self.interdrone)

    except FormationLost as ex:
        # The vehicle is fine, but the swarm isn't in the shape the avoidance
        # strategy assumes, so its separation guarantee no longer holds. Land.
        # Peers time us out after PEER_STALE_S and decide for themselves.
        flight_log.event("poif_formation_lost", reason=str(ex))
        logging.critical("POIF aborted, formation lost: %s", ex)
        return Land(self.drone, self.flight_settings, self.interdrone)

    except LegStalled as ex:
        # Still in GUIDED, just not tracking to the waypoint. Land rather than
        # park the swarm at the barrier waiting on progress that isn't coming.
        flight_log.event("poif_leg_stalled", reason=str(ex))
        logging.error("POIF aborted, drone is not reaching its waypoints: %s", ex)
        return Land(self.drone, self.flight_settings, self.interdrone)

    except FlightInterrupted as ex:
        flight_log.event("poif_failsafe", reason=str(ex))
        logging.critical("POIF aborted, autopilot took over: %s", ex)
        # The autopilot owns the vehicle now, so don't command it further — the
        # Land state's return-to-launch would wait forever for GUIDED movement.
        # Make sure it's coming down and wait for the autopilot to finish.
        vehicle = self.drone.vehicle
        if vehicle.armed and vehicle.mode.name not in ("LAND", "RTL"):
            vehicle.mode = dronekit.VehicleMode("LAND")
        while vehicle.armed:
            await asyncio.sleep(0.5)
        flight_log.event("poif_failsafe_landed")
        logging.info("Failsafe landing complete, motors disarmed")
        return

    except asyncio.CancelledError as ex:
        flight_log.event("poif_cancelled")
        logging.error("Land state canceled")
        raise ex
    finally:
        # Stop talking first: past here our reports describe a drone that is
        # landing, not one flying the formation. Not awaited — this also runs
        # under cancellation, where awaiting would swallow that cancellation.
        if position_broadcast is not None:
            position_broadcast.cancel()
        if telemetry is not None:
            telemetry.stop()


# Setting the run_callable attribute of the Land class to the run function
POIF.run_callable = run
