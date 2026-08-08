import asyncio
import logging
import time

from flight.circlePath import circle_waypoints
import flight.flight_log as flight_log
from flight.pathfinding.utils.goto import FlightInterrupted, LegStalled
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
    LEG_MAX_TOLERANCE_M,
    PEER_STALE_S,
    FormationLost,
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

SYNC_POLL_S: float = 0.25
# How often a drone at the lockstep barrier re-broadcasts its last reached
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
            await self.interdrone.report_position(
                peers, position.lat, position.lon, position.alt
            )
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


async def _hold_for_lockstep(self: POIF, target: Waypoint, heartbeat) -> None:
    """Wait until every live peer has reached the waypoint index before ours.

    This is the demo's actual collision-avoidance strategy. All drones fly
    congruent, same-phase circles offset by their hover-line spacing, so while
    everyone is on the same waypoint index the vector between any two drones
    equals the offset between their circle centers: the paths intersect, but the
    drones are never at the intersection together. The geometric leg check in
    gotoWaypoint is the safety net for whatever this assumption misses.

    Stale peers are excluded so a crashed drone can't stall the formation.
    """
    if target.lap is None or target.index is None:
        return
    step = target.lap * POINTS_PER_LAP + target.index
    if step == 0:
        # No drone has reached anything yet, so the barrier has nothing to
        # synchronise on. The entry leg is safe for a different reason:
        # ENTRY_BEARING_DEG puts all four on parallel tracks a full spacing
        # apart, with the geometric gate in gotoWaypoint watching live positions.
        return

    started = time.monotonic()
    last_heartbeat = started
    waiting_logged = False
    while True:
        self.drone._check_still_ours()
        self.drone.checkSeparation(self.interdrone.drone_states)
        behind = []
        for state in self.interdrone.drone_states:
            if state.is_stale(PEER_STALE_S):
                continue
            reached = state.last_reached_global_index(POINTS_PER_LAP)
            if reached is None or reached < step - 1:
                behind.append({"peer": state.drone_id, "reached": reached})

        if not behind:
            if waiting_logged:
                flight_log.event(
                    "sync_clear",
                    target=flight_log.waypoint_brief(target),
                    waited=time.monotonic() - started,
                )
            return

        if not waiting_logged:
            waiting_logged = True
            flight_log.event(
                "sync_wait", target=flight_log.waypoint_brief(target), behind=behind
            )

        now = time.monotonic()
        if now - last_heartbeat >= SYNC_HEARTBEAT_S:
            last_heartbeat = now
            await heartbeat()

        await asyncio.sleep(SYNC_POLL_S)


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

        self.drone.updateWaypoints(circleWaypoints[:LOOKAHEAD])

        await self.interdrone.send_new_waypoints(peers, circleWaypoints[:LOOKAHEAD])
        circleWaypoints = circleWaypoints[LOOKAHEAD:]
        for state in self.interdrone.drone_states:
            self.drone.checkForCollision(state.list_of_waypoints)

        # Before flying anywhere: peers' avoidance needs a position from us, and
        # the feasibility check below needs theirs.
        position_broadcast = asyncio.create_task(_broadcast_position(self, peers))
        await _assert_formation_feasible(self)

        while True:
            await _hold_for_lockstep(self, self.drone.waypoints[0], heartbeat)
            curWaypoint = await self.drone.gotoWaypoint(
                self.interdrone.drone_states,
                heartbeat=heartbeat,
                points_per_lap=POINTS_PER_LAP,
            )
            last_reached = curWaypoint
            await self.interdrone.reached_waypoint(peers, curWaypoint)
            flight_log.event(
                "reached_broadcast", waypoint=flight_log.waypoint_brief(curWaypoint)
            )
            if len(circleWaypoints) == 0:
                break

            nextWaypoint = circleWaypoints.pop(0)
            self.drone.updateWaypoints([nextWaypoint])
            # Announce the waypoint we just added, not the one we just left, and
            # send it once — send_new_waypoints already fans out to every peer.
            await self.interdrone.send_new_waypoints(peers, [nextWaypoint])
            flight_log.event(
                "waypoint_broadcast", waypoint=flight_log.waypoint_brief(nextWaypoint)
            )
            for drone in self.interdrone.drone_states:
                self.drone.checkForCollision(drone.list_of_waypoints)

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
