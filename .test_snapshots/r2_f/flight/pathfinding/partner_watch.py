"""PartnerWatch: is the other side gone?

Contract:
- Input: the other side's drone ids, the interdrone (their states).
- Output: `gone` -- every drone of the other side has landed (MISSION_END) or, having been
  heard from, has been silent MATE_GONE_S. Never true with no other side, nor for a drone never
  heard from (it may not have started).
- `gone` is what EndRun waits on: with it set nobody is coming with the other half of the route.
"""

from flight.pathfinding.mate_watch import MATE_GONE_S


class PartnerWatch:
    def __init__(self, partners, interdrone):
        """partners: the other side's drone ids (empty: one-sided mission)."""
        self.partners = tuple(partners)
        self.interdrone = interdrone

    def _drone_gone(self, drone_id):
        state = self.interdrone.get_drone_state_from_id(drone_id)
        if state is None:
            return False
        return state.mission_over or (state.live_position is not None
                                      and state.is_stale(MATE_GONE_S))

    @property
    def gone(self):
        return bool(self.partners) and all(self._drone_gone(d) for d in self.partners)
