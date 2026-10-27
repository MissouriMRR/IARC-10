"""The expanding-mines loop for the simulator (Mission(..., levels=True)).

LevelDriver sits between Mission and the backends' level API (search_backend.py), the way the
flight states will:
- every pass, each copy calls try_advance_level (snapshot when ready, grow when the lockstep
  rule says go) and announces its LevelState over the simulated bus to its side-mate and the
  other side (a lossy stream, re-sent every pass, like point A);
- a growth step keeps the drone busy for `expand_ticks` (the rebuild), and none is started
  when the time left is under TIME_MARGIN x that;
- time is the tick budget: when it runs out the mission is over and the reported route is the
  latest valid snapshot, not the live chain;
- the reported route: one side, its latest valid snapshot; two sides, the sides' snapshots at
  the highest level both hold, joined (join_at).
"""

from flight.pathfinding.levels import TIME_MARGIN, Step, cell_path, clearance


class LevelDriver:
    def __init__(self, mission, max_levels=None, tick_budget=None, expand_ticks=3):
        """
        - max_levels: never grow past this level (a test knob; None = until a cap or the time).
        - tick_budget: the mission's time limit in ticks (None = the mission's max_ticks).
        - expand_ticks: ticks a growth step keeps a drone busy.
        """
        self.m = mission
        self.max_levels = max_levels
        self.budget = tick_budget if tick_budget is not None else mission.max_ticks
        self.expand_ticks = expand_ticks
        self.timed_out = False
        self.busy_until = {}
        self.last = {}              # drone -> its latest LevelStep
        self.growth = []            # (tick, drone, level reached, seconds) of every growth
        self.reported_level = None

    # -- per pass ---------------------------------------------------------------------------
    def before_photo(self, drone):
        """Advance this copy's level if it may and announce its state. Returns True when the
        drone is busy with the rebuild and takes no photo this tick."""
        m = self.m
        b = m.backends[drone]
        if m.tick >= self.busy_until.get(drone, 0):
            limit = self.max_levels
            if self.budget - m.tick < TIME_MARGIN * self.expand_ticks:
                limit = b.level              # no time for another growth step
            step = b.try_advance_level(drone, now=m.tick, max_level=limit)
            self.last[drone] = step
            if step.step is Step.GREW:
                self.busy_until[drone] = m.tick + self.expand_ticks
                self.growth.append((m.tick, drone, step.level, step.seconds))
        self._announce(drone)
        return m.tick < self.busy_until.get(drone, 0)

    def _announce(self, drone):
        m = self.m
        state = m.backends[drone].level_state(drone)
        for to in m._mates(drone) + m._partner_drones(drone):
            m.bus.send(to, ("level", state), m.tick)

    def hear(self, to, state):
        self.m.backends[to].hear_level_state(state)

    # -- finished ---------------------------------------------------------------------------
    def done(self):
        """The time is up, or every live copy is ready and will not grow again."""
        if self.timed_out:
            return True
        drones = self.m._alive_drones()
        return all(d in self.last and self.last[d].step.final for d in drones)

    # -- the report ---------------------------------------------------------------------------
    def report(self):
        """(route, level) drone 1 reports; route None when nothing can be reported yet."""
        m = self.m
        b1 = m.backends[1]
        if not m._two_sided():
            snaps = b1.valid_snapshots()
            return b1.report_route(), (snaps[-1].level if snaps and b1.report_valid() else None)
        b3 = m.backends[3]
        mine = {s.level: s for s in b1.valid_snapshots()}
        theirs = {s.level: s for s in b3.valid_snapshots()}
        if not mine or not theirs:                  # no snapshot yet on a side: the live join
            return b1.join(b3.route()), None
        for level in sorted(set(mine) & set(theirs), reverse=True):
            joined = b1.join_at(level, theirs[level].route, theirs[level].mine_hash)
            if joined is not None:
                return joined, level
        return None, None

    def summary(self):
        """Level metrics for the sweep: levels reached per side, snapshots, buffer_width of the
        reported route, the reported level, mean growth seconds."""
        m = self.m
        route, level = self.report()
        speakers = [sp for sp in m.sides if sp not in m.dead]
        secs = [s for *_rest, s in self.growth]
        width = None
        if route:
            width = clearance(cell_path(route, m.field_w, m.field_h), list(m.discovered),
                              round(m.field_w / 2) + round(m.field_h / 2))
        return {"levels": {sp: m.backends[sp].level for sp in speakers},
                "snapshots": {sp: len(m.backends[sp].valid_snapshots()) for sp in speakers},
                "reported_level": level,
                "buffer_width": width,
                "growth_s": round(sum(secs) / len(secs), 3) if secs else 0.0,
                "growths": len(secs), "timed_out": self.timed_out}
