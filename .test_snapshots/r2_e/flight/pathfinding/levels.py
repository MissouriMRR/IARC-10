"""Levels, snapshots and the lockstep rule of the expanding-mines phase (pure; no planner here).

Level k = growth steps done: every mine's danger zone is 2 ft wider per level (the zone shapes
are the path engine's job; see pathnode/engines/base.py). A copy that has finished all its
important work at a level SNAPSHOTS its route, then grows one level and works again. Both
sides climb in lockstep. Design: docs/design/expanding_mines.md (section 10 = decisions).

What is here (all plain values and small pure classes, backend-neutral):
- Step, LevelStep   what one try at advancing did
- LevelState        the small fact a copy announces to its side-mate(s) and the other side
- Snapshot          a frozen record of a complete route at one level
- SnapshotBook      stores snapshots, checks them against the known mines, picks what to report
- LevelGate         the lockstep rule over announced facts
- cell_path, clearance   cell helpers (the reported cell path; city-block distance to mines)

The planner-side owner of all this is pathnode/level_control.py (LevelController).
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.iarc_report import CELL_FT

__all__ = ["Step", "LevelStep", "LevelState", "Snapshot", "SnapshotBook", "Verdict", "LevelGate",
           "cell_path", "clearance", "mine_squares", "TIME_MARGIN"]

# Skip a growth step when the time left is below this multiple of the last measured
# growth + recheck time.
TIME_MARGIN = 1.5


# -- what happened ----------------------------------------------------------------------------
class Step(str, Enum):
    """What one try at advancing a level did."""
    NOT_READY = "not_ready"         # this copy still has important work at its level
    WAITING = "waiting"             # ready, but the side-mate or the other side is not
    GREW = "grew"                   # one level up
    CAPPED = "capped"               # growth was infeasible: stays at its level for good
    AT_CAP = "at_cap"               # will not grow: capped earlier, or at the caller's limit
    PEER_CAPPED = "peer_capped"     # a peer capped at this level or below: no point going up

    @property
    def final(self):
        """Whether this copy will not grow again (it may still re-snapshot after a late mine)."""
        return self in (Step.CAPPED, Step.AT_CAP, Step.PEER_CAPPED)


@dataclass(frozen=True)
class LevelStep:
    """The result of try_advance_level.

    - step: a Step.
    - level: the level after the try.
    - seconds, cut, bridged: for GREW and CAPPED, the growth + recheck time and the growth
      rule's counts (see mine_rule.GrowthResult); else 0.
    - reason: why a level was infeasible, else None.
    """
    step: Step
    level: int
    seconds: float = 0.0
    cut: int = 0
    bridged: int = 0
    reason: str | None = None


# -- announced facts --------------------------------------------------------------------------
@dataclass(frozen=True)
class LevelState:
    """What a copy announces about itself (the flight stage carries it in one message).

    - side: the side's entry edge, "bottom" or "top".
    - drone: the announcing drone's id.
    - level: the level it works at now.
    - ready: it holds a snapshot of its complete, checked route at `level`.
    - capped: it will not grow past `level`.
    - seq: rises with every change of (level, ready, capped); a receiver ignores older ones.
    """
    side: str
    drone: int
    level: int
    ready: bool
    capped: bool = False
    seq: int = 0

    def to_dict(self):
        return {"side": self.side, "drone": self.drone, "level": self.level,
                "ready": self.ready, "capped": self.capped, "seq": self.seq}

    @classmethod
    def from_dict(cls, d):
        return cls(str(d["side"]), int(d["drone"]), int(d["level"]), bool(d["ready"]),
                   bool(d.get("capped", False)), int(d.get("seq", 0)))


# -- cells -------------------------------------------------------------------------------------
def cell_path(points, field_w, field_h):
    """The ordered cell path (col, row) of a route, one cardinal step at a time (the cells the
    IARC report is made of)."""
    cols, rows = round(field_w / CELL_FT), round(field_h / CELL_FT)
    grid = CellField(cols, rows, max_corner=(cols * CELL_FT, rows * CELL_FT))
    return grid.path_cells([tuple(p) for p in points])


def mine_squares(mines):
    """The set of squares (col, row) holding the mines [(x, y)]."""
    return {(int(x // CELL_FT), int(y // CELL_FT)) for x, y in mines}


def clearance(cells, mines, none_known):
    """
    Takes: a cell path, the known mines [(x, y)], and what to return when there are none.

    Returns: the city-block (Manhattan) distance, in squares, within which the path has no
    known mine: (smallest distance from a mine's square to the nearest path square) - 1.
    A mine on the path gives -1; a mine next to it, 0.
    """
    squares = mine_squares(mines)
    if not squares or not cells:
        return none_known
    path = np.array(cells, dtype=int)
    best = None
    for c, r in squares:
        d = int((np.abs(path[:, 0] - c) + np.abs(path[:, 1] - r)).min())
        best = d if best is None else min(best, d)
    return best - 1


# -- snapshots ---------------------------------------------------------------------------------
@dataclass(frozen=True)
class Snapshot:
    """A complete route, frozen the moment its copy was ready at a level. Never changed.

    - level: the zone level the route was checked at.
    - seq: 0, 1, 2... per copy, in the order taken (a late mine can force a second one at a
      level).
    - route: the points from the entry to the target (local feet).
    - cells: the ordered cell path of the route.
    - mine_hash: the copy's mine-and-level hash (mine_hash()) when frozen.
    - t: the caller's clock when frozen.
    - spans: the route runs from its entry edge to the far edge (a half route ends at the
      partner's point A instead).
    """
    level: int
    seq: int
    route: tuple
    cells: tuple
    mine_hash: str
    t: float = 0.0
    spans: bool = False


class SnapshotBook:
    """The snapshots a copy has taken, plus what changes after the fact (outside the frozen
    snapshots): how many of each one's squares have since become unsafe, i.e. fallen inside a
    zone that did not hold them when the snapshot was taken.

    Valid = no known mine lies directly on the snapshot's cell path; mines in the buffer
    around it are fine (buffer_width then reports the smaller clearance).
    """

    def __init__(self, cols, rows):
        """cols, rows: the field in squares (bounds the clearance when no mine is known)."""
        self.none_known = cols + rows
        self._snaps = []
        self._inside = {}               # seq -> [squares inside a zone at the snapshot, now]

    def __len__(self):
        return len(self._snaps)

    @property
    def snapshots(self):
        return list(self._snaps)

    def latest(self):
        return self._snaps[-1] if self._snaps else None

    def add(self, snap, in_zone=None):
        """Store a snapshot (it must be newer than the last). in_zone: see refresh."""
        assert not self._snaps or snap.seq > self._snaps[-1].seq
        self._snaps.append(snap)
        n = self._count(snap, in_zone) if in_zone else 0
        self._inside[snap.seq] = [n, n]

    # -- the unsafe-squares counter ---------------------------------------------------------
    @staticmethod
    def _count(snap, in_zone):
        return sum(1 for c, r in snap.cells if in_zone(c, r))

    def refresh(self, in_zone):
        """Recount every snapshot's squares that now lie inside a zone (call after the zones
        changed: growth, rollback, a new mine). in_zone: (col, row) -> bool."""
        for snap in self._snaps:
            self._inside[snap.seq][1] = self._count(snap, in_zone)

    def unsafe(self, snap):
        """How many of the snapshot's squares became unsafe since it was taken: inside a zone
        now, beyond those that were already (a route may hug a zone's edge)."""
        then, now = self._inside.get(snap.seq, (0, 0))
        return max(0, now - then)

    # -- validity and choice ----------------------------------------------------------------
    @staticmethod
    def valid(snap, mines):
        """No known mine on the snapshot's squares."""
        return not (mine_squares(mines) & set(snap.cells))

    def valid_snapshots(self, mines):
        """The valid snapshots, oldest first."""
        squares = mine_squares(mines)
        return [s for s in self._snaps if not squares & set(s.cells)]

    def latest_valid(self, mines, level=None, spans=None):
        """The newest valid snapshot, or None. level: only that level; spans: only snapshots
        that do (True) or do not (False) run edge to edge."""
        for s in reversed(self.valid_snapshots(mines)):
            if (level is None or s.level == level) and (spans is None or s.spans == spans):
                return s
        return None

    def buffer_width(self, snap, mines):
        """The snapshot's clearance in squares (see clearance)."""
        return clearance(snap.cells, mines, self.none_known)


# -- the lockstep rule -------------------------------------------------------------------------
class Verdict(str, Enum):
    GO = "go"
    NOT_READY = "not_ready"
    AT_CAP = "at_cap"
    PEER_CAPPED = "peer_capped"
    WAIT_MATE = "wait_mate"
    WAIT_OTHER = "wait_other"


class LevelGate:
    """Whether a copy may go from level L to L + 1. No decider: every copy applies this rule to
    the facts announced to it, and the same facts give the same answer on every copy.

    Go when this copy is ready at L and
    - every side-mate (not lost) is ready at L or already past it, and
    - the other side is: some drone of it is ready at L or past it (or the other side is gone).
    A peer that capped at L or below ends the climb (PEER_CAPPED). A mate or other side not yet
    heard from blocks: unknown is never taken as ready.
    """

    def __init__(self, side_drones, has_partner):
        self.side_drones = tuple(side_drones)
        self.has_partner = has_partner
        self.partner_gone = False       # the other side is lost: nobody to wait for
        self.heard = {}                 # drone -> its newest LevelState

    def hear(self, state):
        """Take in an announcement. Returns False for one older than what is known (ignored)."""
        old = self.heard.get(state.drone)
        if old is not None and state.seq <= old.seq:
            return False
        self.heard[state.drone] = state
        return True

    def partner_lost(self):
        """The other side is gone for good: it no longer gates, and its facts are dropped."""
        self.partner_gone = True
        self.has_partner = False

    @staticmethod
    def _past(state, level):
        return state.level > level or (state.level == level and state.ready)

    def check(self, own, lost=()):
        """
        Takes: own, this copy's LevelState; lost, side-mate ids gone quiet (they do not gate).

        Returns: a Verdict. Checks in this order: NOT_READY, AT_CAP, PEER_CAPPED, WAIT_MATE,
        WAIT_OTHER, else GO.
        """
        if not own.ready:
            return Verdict.NOT_READY
        if own.capped:
            return Verdict.AT_CAP
        mates = [d for d in self.side_drones if d != own.drone and d not in lost]
        others = [s for s in self.heard.values() if s.side != own.side]
        peers = [s for s in self.heard.values() if s.drone != own.drone and s.drone not in lost
                 and not (self.partner_gone and s.side != own.side)]
        if any(s.capped and s.level <= own.level for s in peers):
            return Verdict.PEER_CAPPED
        for d in mates:
            state = self.heard.get(d)
            if state is None or not self._past(state, own.level):
                return Verdict.WAIT_MATE
        if self.has_partner and not any(self._past(s, own.level) for s in others):
            return Verdict.WAIT_OTHER
        return Verdict.GO
