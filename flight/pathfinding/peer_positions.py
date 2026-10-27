"""PeerPositions: where the other drones are, for the planner's keep-apart rules.

Contract:
- `side()`: this side's drones, id -> (lat, lon). The mate's entry is the best guess:
  - gone: None.
  - lost: its last known position (the newest report, however old), which keeps blocking.
  - else a position report no older than MATE_POSITION_MAX_AGE_S, else its last photo report,
    else the side's start point.
- `other_side()`: the other side's drones, id -> (lat, lon): the last report, however old, until
  the drone has been silent MATE_GONE_S or announced it is landing (a stale report is the best guess; ignoring it lets a
  drone that only went quiet be flown into). Never heard from: left out.
- `mate_unseen()`: the mate has never reported a position (and is not lost): where it is, is
  unknown -- the start point `side()` falls back to would be a guess the planner acts on.
- `mate_photographed(corners)`: remember where the mate's latest photo was (set by the inbox).
"""

from flight.pathfinding.mate_watch import MATE_GONE_S

MATE_POSITION_MAX_AGE_S = 2.0  # a position report older than this isn't trusted


class PeerPositions:
    def __init__(self, me, mate, partners, flight_settings, interdrone, drone, watch):
        """watch: this side's MateWatch. partners: the other side's drone ids."""
        self.me = me
        self.mate = mate
        self.partners = partners
        self.fs = flight_settings
        self.interdrone = interdrone
        self.drone = drone
        self.watch = watch
        self._mate_photo_at = None      # (lat, lon) of the mate's last photo report

    def mate_photographed(self, corners):
        """Note the mate's photo with these corner coordinates (its centre is its position)."""
        self._mate_photo_at = (sum(lat for lat, _ in corners) / len(corners),
                               sum(lon for _, lon in corners) / len(corners))

    def mate_unseen(self):
        """Whether the mate is expected (not alone, not lost) but has not yet reported where it is."""
        if self.mate is None or self.watch.lost:
            return False
        state = self.watch.state()
        return state is None or state.live_position is None

    def side(self):
        positions = {self.me: self.drone.currentPosition()}
        if self.mate is None:
            return positions
        state = self.watch.state()
        if self.watch.gone:
            positions[self.mate] = None
            return positions
        live = state is not None and state.live_position is not None
        if live and (self.watch.lost or not state.position_is_stale(MATE_POSITION_MAX_AGE_S)):
            positions[self.mate] = state.live_position
        elif self._mate_photo_at is not None:
            positions[self.mate] = self._mate_photo_at
        else:
            start = self.fs.start_coord
            positions[self.mate] = (start["lat"], start["lon"])
        return positions

    def other_side(self):
        out = {}
        for d in self.partners:
            state = self.interdrone.get_drone_state_from_id(d)
            if (state is not None and state.live_position is not None
                    and not state.mission_over and not state.is_stale(MATE_GONE_S)):
                out[d] = state.live_position
        return out
