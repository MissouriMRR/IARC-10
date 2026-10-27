"""The planner's counters and a bounded event log (a debugging aid; bounded so a long flight
can't grow it without limit)."""

from collections import deque

from flight.pathfinding.pathnode.config import EVENT_LOG_LEN

STATS = ("jumps", "nodes_in_mine", "orphans", "bridges", "replans", "entry_shifts",
         "shortcuts", "big_shortcuts", "instant_shortcuts", "shortcuts_given_up",
         "blocked_squares", "reopens")


class PlannerLog:
    def __init__(self):
        self.stats = {name: 0 for name in STATS}
        self.events = deque(maxlen=EVENT_LOG_LEN)

    def count(self, name, n=1):
        self.stats[name] += n

    def event(self, kind, *details):
        self.events.append((kind, *details))
