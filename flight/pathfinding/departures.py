"""Departures: which drones have left the mission (landed, or gone silent)?

Contract:
- Input: the interdrone (the other drones' states).
- `gone(drone_id)`: the drone announced it is landing (MISSION_END, `state.mission_over`), or,
  having been heard from, has been silent MATE_GONE_S. A drone never heard from is not gone
  (it may not have started); neither is one we have no state for.
- `all_gone(drone_ids)`: a non-empty set of drones, every one gone.
- Used by EndRun to stop waiting: for the other side's route (`all_gone(partners)`), and for
  the reporter's ack (`gone(1)`).
"""

from flight.pathfinding.mate_watch import MATE_GONE_S


class Departures:
    def __init__(self, interdrone):
        self.interdrone = interdrone

    def gone(self, drone_id):
        state = self.interdrone.get_drone_state_from_id(drone_id)
        if state is None:
            return False
        return state.mission_over or (state.live_position is not None
                                      and state.is_stale(MATE_GONE_S))

    def all_gone(self, drone_ids):
        drone_ids = tuple(drone_ids)
        return bool(drone_ids) and all(self.gone(d) for d in drone_ids)
