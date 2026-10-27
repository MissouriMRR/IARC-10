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

Parts (each one job; SideAgent composes them):
- MateWatch (mate_watch.py): side-mate lost / gone / landed, and who speaks.
- PartnerWatch (partner_watch.py): the other side gone (landed or silent).
- PeerPositions (peer_positions.py): where the side's and the other side's drones are.
- InboxApplier (inbox.py): applies the staged messages in a fixed order.
- Heartbeats (heartbeats.py): job, point A and mine-sync streams with their timers.
- SideAgent itself: the planning pass and this drone's photo relays.

Drone fields this reads (staged by the interdrone receive task, which never touches the
planner): pending_cross_pair_mines, pending_photo_reports, pending_partner_coverage,
pending_partner_point_a, peer_jobs.
"""

import hashlib
import logging

import flight.flight_log as flight_log

from flight.pathfinding.heartbeats import MINE_SYNC_S, Heartbeats
from flight.pathfinding.inbox import InboxApplier
from flight.pathfinding.mate_watch import MateWatch
from flight.pathfinding.partner_watch import PartnerWatch
from flight.pathfinding.peer_positions import PeerPositions


def relay_id(lat, lon):
    """A short id for a mine relay, from its position (echoed back in the ack)."""
    return hashlib.sha256(f"{lat:.7f},{lon:.7f}".encode("utf-8")).hexdigest()[:16]


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
        self.watch = MateWatch(self.me, self.mate, interdrone, drone)
        self.partner_watch = PartnerWatch(self.partners, interdrone)
        self._partner_lost_told = False
        self.peers = PeerPositions(self.me, self.mate, self.partners, flight_settings,
                                   interdrone, drone, self.watch)
        self.inbox = InboxApplier(planner, drone, self.peers)
        self.heartbeats = Heartbeats(planner, interdrone, self.watch, self.me, self.mate,
                                     self.partners)

    # -- who is who ------------------------------------------------------------------------
    @property
    def mate_lost(self):
        return self.watch.lost

    @property
    def mate_gone(self):
        return self.watch.gone

    @property
    def partner_gone(self):
        """Every drone of the other side has landed or gone silent: its half is not coming."""
        return self.partner_watch.gone

    def partner_lost(self):
        """Tell the planner the other side is gone (once): it turns toward the lost side's last
        point A and on to its edge, so this side's own photos check that half too."""
        if not self._partner_lost_told:
            self._partner_lost_told = True
            flight_log.event("partner_side_gone", partners=list(self.partners))
            logging.error("drone %d: the other side has landed or gone silent -- carrying on "
                          "without it", self.me)
            self.planner.partner_lost()

    @property
    def speaker(self):
        """Whether this drone speaks for its side: the lowest id still heard from."""
        return self.watch.speaker

    @property
    def mate_busy(self):
        """The side-mate is still working, and its photos may still change this copy: it last
        announced a job, or has not announced anything yet (a late starter), and is not lost."""
        return (self.mate is not None and not self.watch.lost
                and (self.mate not in self.drone.peer_jobs
                     or self.drone.peer_jobs[self.mate] is not None))

    def lost_mates(self):
        """The side-mates to pass as `lost` to the planner: silent, but not yet gone."""
        return self.watch.lost_mates()

    def positions(self):
        """Where the side's drones are (drone id -> (lat, lon)); see PeerPositions.side."""
        return self.peers.side()

    def other_side_positions(self):
        """Where the other side's drones are; see PeerPositions.other_side."""
        return self.peers.other_side()

    # -- in / out --------------------------------------------------------------------------
    def apply_inbox(self):
        """Everything staged since the last pass, in the fixed order of InboxApplier."""
        self.inbox.apply()

    async def send_heartbeats(self, mine_sync_s=MINE_SYNC_S):
        """Job announcement, then the speaker's point A and mine sync; see Heartbeats."""
        await self.heartbeats.send(mine_sync_s)

    # -- a planning pass -------------------------------------------------------------------
    async def plan(self):
        """Apply the inbox, send the heartbeats, and work out the side's claims. Returns this
        drone's queue, [(lat, lon)] in flying order."""
        self.watch.check()
        self.apply_inbox()
        await self.send_heartbeats()
        if self.peers.mate_unseen():
            # A drone that starts after its mate would plan against a guessed position (the
            # side's start point) and fly onto the mate's real one. Wait to hear it.
            return []
        if self.mate is not None and self.me > self.mate and self.mate not in self.drone.peer_jobs                 and not self.watch.lost:
            # Both drones start with the same idea of the best job and would fly it together
            # (each copy has to hear the other's claim first). The lower id plans at once;
            # the higher waits for its announcement and then plans around it.
            return []
        peer_jobs = self.drone.peer_jobs
        # A mate not yet heard from is left out (unknown), not held as "no job" (idle). A lost
        # one's last announced job is kept: the planner reserves it and keeps clear of it.
        held = ({self.mate: peer_jobs[self.mate]}
                if self.mate is not None and not self.watch.gone and self.mate in peer_jobs
                else {})
        others = self.other_side_positions()
        held.update({d: peer_jobs[d] for d in others if d in peer_jobs})
        plan = self.planner.assignments(self.positions(), held, self.me, others,
                                        self.lost_mates())
        await self.heartbeats.announce_job()
        return plan[self.me]

    # -- this drone's photos ---------------------------------------------------------------
    def own_footprint(self, corners):
        self.planner.own_footprint(corners)

    async def own_photo(self, corners, mines):
        """This drone's photo is done (its footprint already recorded; corners is the last
        one, or None if it had none) with these mines in it: apply it, relay new mines and
        the footprint to the other side, and send the photo to the mate."""
        for lat, lon in self.planner.own_photo(corners, mines):
            if self.partners:
                await self.interdrone.send_cross_pair_mine_relay(
                    lat, lon, relay_id(lat, lon), self.partners)
        if corners is not None and self.partners:
            await self.interdrone.send_cross_pair_coverage(corners, self.partners)
        if self.mate is not None and not self.watch.lost:
            if corners is not None:
                photo = {"cornerCoordinates": corners, "mines": list(mines)}
            else:
                photo = {"cornerCoordinates": [], "mines": []}
            await self.interdrone.share_photos([photo], (self.mate,))
        await self.send_heartbeats()

    async def failed_photo(self, lat, lon):
        """A planned photo that produced no footprint: a miss here and in the mate's copy."""
        self.planner.failed_photo(lat, lon)
        if self.mate is not None and not self.watch.lost:
            await self.interdrone.share_photos(
                [{"cornerCoordinates": [], "mines": [], "failedAt": (lat, lon)}], (self.mate,))
