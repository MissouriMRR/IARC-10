"""One drone's part in its side: its copy of the side's planner, plus everything it says to
and hears from the other drones. The flight states call this; nothing here knows about
states.

Sides (FlightSettings.side_drones): the one or two drones working from one field edge. Every
drone keeps its own copy of its side's planner (MissionPlanner -> pathnode/session.py) and
works out the whole side's claims itself, from where both drones are and the job its
side-mate last announced. There are no roles:
- Photos: every drone applies its own photos, and sends each to its side-mate (SHARE_PHOTOS),
  so both copies see the same coverage and mines.
- Jobs: every drone announces the job it is working on (SIDE_JOB); the side-mate's copy never
  hands that job to anyone else (claims.py). Ties go to the lower id.
- The speaker -- the lowest-id drone of the side still heard from -- speaks for the side to
  the other side: point A, the full mine list. Mines and photo footprints go to the other
  side from whichever drone found or took them.
- Messages to the other side go to all of its drones, so each copy there hears them.
- The two sides keep apart the same way the side-mates do (planner.plan): each drone hears
  the other side's POSITION_REPORTs and announced jobs. Lower ids keep their ground, so the
  sides cannot block each other.
- A silent drone is never assumed gone at once (a starved event loop or a radio dropout
  leaves a drone flying while it says nothing):
  - side-mate silent MATE_LOST_S: "lost". Its work is taken over, but its last known position
    and last announced job keep blocking (same keep-apart distances) and the sequence of that
    job stays reserved, since the drone is probably still flying it. Heard again: an
    ordinary mate at once, its announced job wins by the usual tie rules.
  - side-mate silent MATE_GONE_S: gone (crashed or landed): ignored entirely, or its last
    position would block the work around it for good.
  - other side's drone: its last known position and job keep blocking until it has been
    silent for MATE_GONE_S; a position report only ever gets older, so it is used as is.

Drone fields this reads (staged by the interdrone receive task, which never touches the
planner): pending_cross_pair_mines, pending_photo_reports, pending_partner_coverage,
pending_partner_point_a, peer_jobs.
"""

import logging
import time

import flight.flight_log as flight_log

POINT_A_HEARTBEAT_S = 1.0      # point A is resent at least this often (a lossy stream)
JOB_HEARTBEAT_S = 1.0          # this drone's job is re-announced at least this often
MINE_SYNC_S = 5.0              # the speaker sends its full mine list this often
MATE_POSITION_MAX_AGE_S = 2.0  # a position report older than this isn't trusted
MATE_LOST_S = 20.0             # a side-mate silent this long is lost (its work comes here)
MATE_GONE_S = 120.0            # silent this long: gone -- its last position stops blocking


class SideAgent:
    def __init__(self, planner, drone, flight_settings, interdrone):
        """planner: this drone's MissionPlanner (its copy of the side's planner)."""
        self.planner = planner
        self.drone = drone
        self.fs = flight_settings
        self.interdrone = interdrone
        self.me = flight_settings.current_drone_ID
        self.mate = flight_settings.paired_drone            # None: working the side alone
        self.partners = tuple(flight_settings.partner_side_drones)
        self.mate_lost = False
        self.mate_gone = False
        self._mate_photo_at = None      # (lat, lon) of the mate's last photo report
        self._point_a_sent = None       # ((lat, lon), time)
        self._job_sent = None           # (job, time)
        self._mines_synced_at = None

    # -- who is who ------------------------------------------------------------------------
    def _mate_state(self):
        return None if self.mate is None else self.interdrone.get_drone_state_from_id(self.mate)

    def _check_mate(self):
        """Update mate_lost: silent for MATE_LOST_S once the mission is under way."""
        state = self._mate_state()
        started = (self.drone.startTime is not None
                   and time.monotonic() - self.drone.startTime > MATE_LOST_S)
        lost = state is not None and started and state.is_stale(MATE_LOST_S)
        gone = lost and state.is_stale(MATE_GONE_S)
        if lost and not self.mate_lost:
            flight_log.event("mate_lost", peer=self.mate)
            logging.error("drone %d: side-mate %d silent for %.0fs -- taking over its work "
                          "(its last position and job stay blocked)",
                          self.me, self.mate, MATE_LOST_S)
        elif self.mate_lost and not lost:
            flight_log.event("mate_back", peer=self.mate)
            logging.warning("drone %d: side-mate %d heard again", self.me, self.mate)
        if gone and not self.mate_gone:
            flight_log.event("mate_gone", peer=self.mate)
            logging.error("drone %d: side-mate %d silent for %.0fs -- no longer blocking its "
                          "last position", self.me, self.mate, MATE_GONE_S)
        self.mate_lost, self.mate_gone = lost, gone

    @property
    def speaker(self):
        """Whether this drone speaks for its side: the lowest id still heard from."""
        return self.mate is None or self.mate_lost or self.me < self.mate

    def lost_mates(self):
        """The side-mates to pass as `lost` to the planner: silent, but not yet gone."""
        return (self.mate,) if self.mate_lost and not self.mate_gone else ()

    def positions(self):
        """Where the side's drones are (drone id -> (lat, lon)). The mate: a fresh position
        report, else its last photo report, else the side's start point. Lost: its last
        known position (the newest report, however old), which keeps blocking; None once
        gone."""
        positions = {self.me: self.drone.currentPosition()}
        if self.mate is None:
            return positions
        state = self._mate_state()
        if self.mate_gone:
            positions[self.mate] = None
            return positions
        live = state is not None and state.live_position is not None
        if live and (self.mate_lost or not state.position_is_stale(MATE_POSITION_MAX_AGE_S)):
            positions[self.mate] = state.live_position
        elif self._mate_photo_at is not None:
            positions[self.mate] = self._mate_photo_at
        else:
            start = self.fs.start_coord
            positions[self.mate] = (start["lat"], start["lon"])
        return positions

    def other_side_positions(self):
        """Where the other side's drones are, for keeping clear of them (drone id -> (lat, lon)):
        the last reported position, however old, until the drone has been silent for
        MATE_GONE_S -- a stale report is the best guess, and ignoring it lets a drone that
        only went quiet be flown into. Never heard from: left out."""
        out = {}
        for d in self.partners:
            state = self.interdrone.get_drone_state_from_id(d)
            if (state is not None and state.live_position is not None
                    and not state.is_stale(MATE_GONE_S)):
                out[d] = state.live_position
        return out

    # -- in --------------------------------------------------------------------------------
    def apply_inbox(self):
        """Everything staged since the last pass, in a fixed order: the other side's mines
        first (no path node may be laid over ground about to turn out unsafe), then the mate's
        photos, then the other side's coverage and point A."""
        planner, drone = self.planner, self.drone
        mines, drone.pending_cross_pair_mines = drone.pending_cross_pair_mines, []
        for report in mines:
            planner.partner_mine(report.lat, report.lon)
        photos, drone.pending_photo_reports = drone.pending_photo_reports, []
        for report in photos:
            if report.failed_at is not None:
                planner.failed_photo(*report.failed_at)
                continue
            planner.mate_photo(report.corner_coordinates, report.mines)
            corners = report.corner_coordinates
            self._mate_photo_at = (sum(lat for lat, _ in corners) / len(corners),
                                   sum(lon for _, lon in corners) / len(corners))
        coverage, drone.pending_partner_coverage = drone.pending_partner_coverage, []
        for corners in coverage:
            planner.partner_coverage(corners)
        point_a, drone.pending_partner_point_a = drone.pending_partner_point_a, None
        if point_a is not None:
            planner.partner_point_a(*point_a)

    # -- out -------------------------------------------------------------------------------
    @property
    def mate_busy(self):
        """The side-mate last announced a job (and is not lost): it is still working, and its
        photos may still change this copy."""
        return (self.mate is not None and not self.mate_lost
                and self.drone.peer_jobs.get(self.mate) is not None)

    async def send_heartbeats(self, mine_sync_s=MINE_SYNC_S):
        """Everyone's job announcement (resent every JOB_HEARTBEAT_S, so a lost "no job" cannot
        keep the mate waiting), then the speaker's streams to the other side: point A (on
        change, and every POINT_A_HEARTBEAT_S) and the full mine list (every mine_sync_s)."""
        self._check_mate()
        await self._announce_job()
        if not self.partners or not self.speaker or self.planner.failure:
            return
        now = time.monotonic()
        point_a = self.planner.point_a_latlon()
        last = self._point_a_sent
        if last is None or last[0] != point_a or now - last[1] >= POINT_A_HEARTBEAT_S:
            await self.interdrone.send_cross_pair_point_a(*point_a, self.partners)
            self._point_a_sent = (point_a, now)
        if self._mines_synced_at is None or now - self._mines_synced_at >= mine_sync_s:
            await self.interdrone.send_cross_pair_mine_sync(self.planner.mines_latlon(),
                                                            self.partners)
            self._mines_synced_at = now

    async def _announce_job(self):
        """Tell the mate and the other side which job this drone holds (on change, and every
        JOB_HEARTBEAT_S)."""
        to = ((() if self.mate is None or self.mate_lost else (self.mate,))
              + tuple(self.partners))
        if not to:
            return
        job = self.planner.current_job(self.me)
        now = time.monotonic()
        last = self._job_sent
        if last is None or last[0] != job or now - last[1] >= JOB_HEARTBEAT_S:
            await self.interdrone.send_side_job(job, to)
            self._job_sent = (job, now)

    # -- a planning pass -------------------------------------------------------------------
    async def plan(self):
        """Apply the inbox, send the heartbeats, and work out the side's claims. Returns this
        drone's queue, [(lat, lon)] in flying order."""
        self._check_mate()
        self.apply_inbox()
        await self.send_heartbeats()
        peer_jobs = self.drone.peer_jobs
        # A mate not yet heard from is left out (unknown), not held as "no job" (idle). A lost
        # one's last announced job is kept: the planner reserves it and keeps clear of it.
        held = ({self.mate: peer_jobs[self.mate]}
                if self.mate is not None and not self.mate_gone and self.mate in peer_jobs
                else {})
        others = self.other_side_positions()
        held.update({d: peer_jobs[d] for d in others if d in peer_jobs})
        plan = self.planner.assignments(self.positions(), held, self.me, others,
                                        self.lost_mates())
        await self._announce_job()
        return plan[self.me]

    # -- this drone's photos ---------------------------------------------------------------
    def own_footprint(self, corners):
        self.planner.own_footprint(corners)

    async def own_photo(self, corners, mines):
        """This drone's photo is done (its footprint already recorded; corners is the last
        one, or None if it had none) with these mines in it: apply it, relay new mines and
        the footprint to the other side, and send the photo to the mate."""
        for lat, lon, relay_id in self.planner.own_photo(corners, mines):
            if self.partners:
                await self.interdrone.send_cross_pair_mine_relay(lat, lon, relay_id, self.partners)
        if corners is not None and self.partners:
            await self.interdrone.send_cross_pair_coverage(corners, self.partners)
        if self.mate is not None and not self.mate_lost:
            if corners is not None:
                photo = {"cornerCoordinates": corners, "mines": list(mines)}
            else:
                photo = {"cornerCoordinates": [], "mines": []}
            await self.interdrone.share_photos([photo], (self.mate,))
        await self.send_heartbeats()

    async def failed_photo(self, lat, lon):
        """A planned photo that produced no footprint: a miss here and in the mate's copy."""
        self.planner.failed_photo(lat, lon)
        if self.mate is not None and not self.mate_lost:
            await self.interdrone.share_photos(
                [{"cornerCoordinates": [], "mines": [], "failedAt": (lat, lon)}], (self.mate,))
