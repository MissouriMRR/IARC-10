"""Implements the behavior of the EndRun state."""

import asyncio
import logging
import time

import flight.flight_log as flight_log
from state_machine.flight_settings import Side
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.app_share import AppShare
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.end_run import EndRun
from state_machine.states.state import State

# Two-sided missions (see _join_with_partner):
# - how often the top side's speaker re-sends its route until drone 1 acks it;
CHAIN_RESEND_S = 1.0
# - once the flight time budget is gone, how long either still waits for the other before
#   giving up (drone 1 then reports only its own half, flagged);
JOIN_GRACE_S = 10.0
# - how often the wait loop polls;
JOIN_POLL_S = 0.2
# - how often each side's speaker sends its full mine list while waiting (CROSS_PAIR_MINE_SYNC);
MINE_SYNC_S = 1.0
# - drone 1: how long a join must stay valid (same mine hash on both sides) before it is
#   reported -- a mine still in flight from the partner's pair lands inside this window.
JOIN_CONFIRM_S = 3.0


async def _join_with_partner(self: EndRun, planner) -> State | None:
    """
    Two-sided mission: turn the two sides' halves into one report. Drone 1 (the reporter: it
    holds the app link) and the top side's speaker (its lowest id still heard from) do the
    joining; every other drone keeps its own route but, like the speaker, leaves only on the
    reporter's ack whose mine hash matches its own (so all copies end with the same mines).
    Nobody leaves while its side-mate is still working: the mate's photos can still add mines.

    - Each side's speaker sends its full mine list every MINE_SYNC_S (CROSS_PAIR_MINE_SYNC);
      the other side takes the union, so a lost relay can't keep the mine hashes apart.
    - Top speaker: sends its route to the bottom side (CROSS_PAIR_CHAIN) -- "complete" when
      the route is verified (whole and checked), even with optional work left -- and leaves
      only on an ack whose mine hash matches its own current one.
    - Other drones: send nothing; wait for that same ack (the reporter sends it to the
      top side and to its own side-mate).
    - Drone 1: once its own side is done (or time is up), joins a complete partner route
      planned around exactly the mines it knows (matching mine hash). If the halves don't
      connect (both sides met crosswise), the planner re-opens this side toward the
      partner's route end and the scan resumes. A join is reported only after it has stayed
      valid for JOIN_CONFIRM_S -- a mine arriving meanwhile drops it. If the partner's route
      never arrives or doesn't connect in time, reports its own half and sets
      Drone.reported_half_route.
    - Both keep applying relayed mines while they wait. A late one can reopen this side
      (not done any more): returns CalcScanPath then, to finish scanning first.

    Waits until joined, or until JOIN_GRACE_S after the flight time budget runs out.
    Returns CalcScanPath if this side reopened, else None (mission_path is set).
    """
    drone, fs, side = self.drone, self.flight_settings, self.drone.side
    partners = tuple(fs.partner_side_drones)
    is_reporter = fs.current_drone_ID == 1
    sends_chain = fs.start_side == Side.END and side.speaker
    ack_to = partners + ((fs.paired_drone,) if fs.paired_drone is not None else ())
    timed_out_at = None
    last_chain_sent = None
    joined = None
    joined_at = None
    while True:
        side.apply_inbox()
        await side.send_heartbeats(MINE_SYNC_S)
        if drone.time_exceeded(fs.max_flight_time):
            timed_out_at = timed_out_at or time.monotonic()
        elif not planner.failure and not planner.done():
            drone.route_joined = False
            flight_log.event("end_run_reopened", drone=fs.current_drone_ID)
            return CalcScanPath(drone, fs, self.interdrone)

        if is_reporter:
            mine_hash = planner.mine_hash()
            if joined is not None and drone.partner_route_mine_hash != drone.joined_mine_hash:
                # The partner has a mine the join didn't: wait for its route around it (it
                # can't leave -- our ack no longer matches its hash).
                flight_log.event("join_dropped", reason="partner mine hash changed")
                joined, joined_at, drone.route_joined = None, None, False
            elif joined is not None and drone.joined_mine_hash != mine_hash:
                # A mine reached this side after the join. Keep the join only if it still
                # clears every mine known here (the partner may already have left).
                joined = planner.join_latlon(drone.partner_route)
                if joined is None:
                    flight_log.event("join_dropped", reason="a new mine is on the route")
                    joined_at, drone.route_joined = None, False
                else:
                    drone.joined_mine_hash = mine_hash
                    await self.interdrone.send_cross_pair_chain_ack(mine_hash, ack_to)
            if joined is None and (
                drone.partner_route
                and drone.partner_route_complete
                and drone.partner_route_mine_hash == mine_hash
                and ((planner.done() and not side.mate_busy) or timed_out_at is not None)
            ):
                # Join only a complete partner route planned around exactly the mines this
                # side knows: otherwise a mine still in flight to either side could lie on it.
                if timed_out_at is None:
                    joined = planner.check_join_latlon(drone.partner_route)
                    if joined is None and not planner.done():
                        flight_log.event("end_run_reopened", drone=fs.current_drone_ID,
                                         reason="halves don't connect")
                        return CalcScanPath(drone, fs, self.interdrone)
                else:
                    joined = planner.join_latlon(drone.partner_route)
                if joined is not None:
                    joined_at = time.monotonic()
                    drone.route_joined = True
                    drone.joined_mine_hash = mine_hash
                    await self.interdrone.send_cross_pair_chain_ack(mine_hash, ack_to)
            if joined is not None and time.monotonic() - joined_at >= JOIN_CONFIRM_S:
                break
        else:
            if drone.partner_route_ack_hash == planner.mine_hash() and not side.mate_busy:
                break
            now = time.monotonic()
            if sends_chain and (last_chain_sent is None
                                or now - last_chain_sent >= CHAIN_RESEND_S):
                await self.interdrone.send_cross_pair_chain(
                    planner.route_latlon(), planner.route_verified(), planner.mine_hash(), partners
                )
                last_chain_sent = now

        if timed_out_at is not None and time.monotonic() - timed_out_at >= JOIN_GRACE_S:
            break
        await asyncio.sleep(JOIN_POLL_S)

    if is_reporter:
        # Ack even when the join failed: the partner can't fix it, and should go home.
        drone.route_joined = True
        drone.joined_mine_hash = planner.mine_hash()
        await self.interdrone.send_cross_pair_chain_ack(drone.joined_mine_hash, ack_to)
        if joined is not None:
            drone.mission_path = joined
        else:
            drone.mission_path = planner.route_latlon()
            drone.reported_half_route = True
            flight_log.event(
                "reported_half_route",
                partner_route_received=drone.partner_route is not None,
                partner_route_complete=drone.partner_route_complete,
            )
            logging.error(
                "drone 1: could not join the partner's route (%s) -- reporting only this"
                " side's half of the field",
                "it never arrived complete" if not drone.partner_route_complete
                else "the two halves don't connect",
            )
    else:
        drone.mission_path = planner.route_latlon()
    return None


async def run(self: EndRun) -> State:
    """
    Implements the run method for the EndRun state.

    The mission is over -- either the scan finished and ExpandNodes
    hardened its route, or CalcScanPath/Scan cut the scan short because
    FlightSettings.max_flight_time was exceeded (see Drone.
    time_exceeded) -- either way, there's nothing more for this drone to
    search for. Sets Drone.mission_path to this drone's final route and
    transitions to AppShare to report it before heading home.

    Two-sided mission: drone 1 and the top side's speaker join the halves first
    (_join_with_partner); drone 1 reports the full-field route.

    Returns
    -------
    AppShare : State
        The mission's final results still need to reach the app before
        Recall/Land actually bring the drone down.
    CalcScanPath : State
        A relayed mine reopened this side while it waited to join (see
        _join_with_partner).

    Raises
    ------
    asyncio.CancelledError
        If the execution of the EndRun state is canceled.
    """
    try:
        update_state("EndRun")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("EndRun state running")

        planner = self.drone.planner
        if planner is not None:
            if self.flight_settings.partner_side_drones:
                reopened = await _join_with_partner(self, planner)
                if reopened is not None:
                    return reopened
            else:
                # A wholesale REPLACE: CalcScanPath's mission_path updates are progress
                # snapshots, and shortcuts can change the route after them.
                self.drone.mission_path = planner.route_latlon()

        flight_log.event(
            "end_run",
            mission_path_len=len(self.drone.mission_path),
            reported_half_route=self.drone.reported_half_route,
            time_exceeded=self.drone.time_exceeded(self.flight_settings.max_flight_time),
        )

        return AppShare(self.drone, self.flight_settings, self.interdrone)
    except asyncio.CancelledError as ex:
        logging.error("EndRun state canceled")
        raise ex
    finally:
        pass


# Setting the run_callable attribute of the EndRun class to the run function
EndRun.run_callable = run
