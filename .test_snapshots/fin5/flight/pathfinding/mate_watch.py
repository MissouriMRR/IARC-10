"""MateWatch: is the side-mate heard from, silent (lost) or gone? Plus who speaks for the side.

Contract:
- Input: the mate's interdrone state (is_stale), this drone's start time, a clock.
- Output: `lost`, `gone`, `speaker`, `lost_mates()`, `state()`. `check()` updates them and logs
  each transition once.
- Levels (silence of the mate, once the mission has run MATE_LOST_S):
  - lost (silent MATE_LOST_S): its work is taken over, but its last position and job keep
    blocking, since it is probably still flying. Heard again: an ordinary mate at once.
  - gone (silent MATE_GONE_S): crashed or landed; ignored entirely.
- No mate (flying the side alone): never lost, never gone, always the speaker.
"""

import logging
import time

import flight.flight_log as flight_log

MATE_LOST_S = 20.0             # a side-mate silent this long is lost (its work comes here)
MATE_GONE_S = 120.0            # silent this long: gone -- its last position stops blocking


class MateWatch:
    def __init__(self, me, mate, interdrone, drone, clock=time.monotonic):
        """me, mate: drone ids (mate None: working alone). drone: for `startTime`."""
        self.me = me
        self.mate = mate
        self.interdrone = interdrone
        self.drone = drone
        self.clock = clock
        self.lost = False
        self.gone = False

    def state(self):
        """The mate's interdrone state, or None (no mate, or never heard from)."""
        return None if self.mate is None else self.interdrone.get_drone_state_from_id(self.mate)

    def check(self):
        """Update lost/gone from how long the mate has been silent; log each change."""
        state = self.state()
        started = (self.drone.startTime is not None
                   and self.clock() - self.drone.startTime > MATE_LOST_S)
        lost = state is not None and started and state.is_stale(MATE_LOST_S)
        gone = lost and state.is_stale(MATE_GONE_S)
        if lost and not self.lost:
            flight_log.event("mate_lost", peer=self.mate)
            logging.error("drone %d: side-mate %d silent for %.0fs -- taking over its work "
                          "(its last position and job stay blocked)",
                          self.me, self.mate, MATE_LOST_S)
        elif self.lost and not lost:
            flight_log.event("mate_back", peer=self.mate)
            logging.warning("drone %d: side-mate %d heard again", self.me, self.mate)
        if gone and not self.gone:
            flight_log.event("mate_gone", peer=self.mate)
            logging.error("drone %d: side-mate %d silent for %.0fs -- no longer blocking its "
                          "last position", self.me, self.mate, MATE_GONE_S)
        self.lost, self.gone = lost, gone

    @property
    def speaker(self):
        """Whether this drone speaks for its side: the lowest id still heard from."""
        return self.mate is None or self.lost or self.me < self.mate

    def lost_mates(self):
        """The side-mates to pass as `lost` to the planner: silent, but not yet gone."""
        return (self.mate,) if self.lost and not self.gone else ()
