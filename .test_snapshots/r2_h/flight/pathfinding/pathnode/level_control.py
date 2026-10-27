"""LevelController: one planner's expanding-mines sequence (levels, snapshots, growth).

Owned by PathNodePlanner (planner.levels). The loop it runs, one pass at a time
(try_advance), is:

    ready? -> snapshot -> the LevelGate says go? -> grow one level -> recheck -> work -> ready?

Pieces (the pure parts are in flight/pathfinding/levels.py):
- ready()      groups 0-3 (PATH, BIG_SHORTCUT, ADJACENT, SHORTCUT) are empty, the route is met,
               nothing is pending and no shortcut trial is live. OUTER is not required.
- snapshot()   freezes the complete route (Snapshot); never changed afterwards.
- grow()       routes.set_level(L + 1), then mine_rule.apply_growth: every cut gap is re-bridged,
               and the bridges become the work. Infeasible: the zones go back to L and the copy
               is capped there (rollback).
- recover()    a late mine at level > 0 left no route, or the two sides cannot meet any more
               (stuck): step down level by level, rebuilding the chain from the latest
               snapshot, until some level keeps a route.
- reporting    report_route / join_at / buffer_width over the book of snapshots.

Time: grow() measures its own duration (last_seconds); growth_fits(remaining) is the skip rule
(remaining >= TIME_MARGIN x last_seconds). The clock itself belongs to the caller.
"""

import math
import time

from flight.pathfinding.levels import (TIME_MARGIN, LevelGate, LevelState, LevelStep, Snapshot,
                                       SnapshotBook, Step, Verdict, cell_path, clearance)
from flight.pathfinding.pathnode import join as join_mod
from flight.pathfinding.pathnode import mine_rule
from flight.pathfinding.pathnode.chain import Chain, Link, PathNode
from flight.pathfinding.pathnode.config import ROUTE_MATCH_FT, Group, LinkKind, Placer
from flight.pathfinding.pathnode.engines import make_engine

# Passes of try_advance with no way forward (see LevelController.stuck) before stepping down a level.
STUCK_PASSES = 20

# Groups whose photos must all be taken before a snapshot (OUTER is not among them).
IMPORTANT = (Group.PATH, Group.BIG_SHORTCUT, Group.ADJACENT, Group.SHORTCUT)

_STEP_OF = {Verdict.NOT_READY: Step.NOT_READY, Verdict.AT_CAP: Step.AT_CAP,
            Verdict.PEER_CAPPED: Step.PEER_CAPPED, Verdict.WAIT_MATE: Step.WAITING,
            Verdict.WAIT_OTHER: Step.WAITING}


class LevelController:
    def __init__(self, pl):
        """pl: the PathNodePlanner this controller belongs to."""
        self.pl = pl
        cols, rows = round(pl.field_w / 2.0), round(pl.field_h / 2.0)
        self.book = SnapshotBook(cols, rows)
        self.gate = LevelGate(pl.drones, pl.target.has_partner)
        self.cap = None                 # the level it will not grow past, once known
        self.last_seconds = None        # duration of the latest growth + recheck
        self.growth_log = []            # (level reached, seconds) of every successful growth
        self._seq = 0                   # LevelState.seq counter
        self._announced = None          # (level, ready, capped) of the last LevelState
        self._stuck = 0                 # consecutive passes with no way forward
        self._scratch = None            # (key, engine) for clears_at

    # -- state ------------------------------------------------------------------------------
    @property
    def level(self):
        return self.pl.routes.level

    @property
    def capped(self):
        return self.cap is not None and self.cap <= self.level

    def partner_lost(self):
        """The other side is gone: it no longer gates the climb."""
        self.gate.partner_lost()

    # -- ready and snapshot -------------------------------------------------------------------
    def ready(self):
        """Whether every important job at this level is done (see the module docstring)."""
        pl = self.pl
        if pl.failure or not pl.met or pl.chain.has_pending():
            return False
        if any(seq for g in IMPORTANT for _key, seq in pl.groups()[g]):
            return False
        return all(t is None for t in pl.shortcuts.trials.values())

    def _key(self):
        return (self.level, tuple(self.pl.chain.route()), self.pl.level_hash())

    def snapshot(self, now=0.0):
        """Freeze the route if this copy is ready and has not already frozen this very one.

        Returns: the Snapshot that now describes the route, or None when not ready.
        """
        if not self.ready():
            return None
        pl = self.pl
        level, route, mine_hash = self._key()
        last = self.book.latest()
        if last is not None and (last.level, last.route, last.mine_hash) == (level, route, mine_hash):
            return last
        far_y = pl.start_edge.opposite.y(pl.field_h)
        spans = abs(route[0][1] - pl.start_edge.y(pl.field_h)) < 1e-6 and abs(route[-1][1] - far_y) < 1e-6
        snap = Snapshot(level, len(self.book), route, tuple(cell_path(route, pl.field_w, pl.field_h)),
                        mine_hash, float(now), spans)
        self.book.add(snap, pl.routes.in_zone)
        return snap

    def is_snapshotted(self):
        """Ready, and the newest snapshot is exactly the current route at the current level."""
        if not self.ready():
            return False
        last = self.book.latest()
        return last is not None and (last.level, last.route, last.mine_hash) == self._key()

    # -- announcing and hearing -----------------------------------------------------------------
    def state(self, me):
        """The LevelState drone `me` announces now (seq rises when anything in it changes)."""
        now = (self.level, self.is_snapshotted(), self.capped)
        if now != self._announced:
            self._seq += 1
            self._announced = now
        return LevelState(self.pl.start_edge.value, me, *now, self._seq)

    def hear(self, state):
        """Take in a peer's announcement. Returns False for an old one."""
        return self.gate.hear(state)

    # -- advancing --------------------------------------------------------------------------
    def try_advance(self, me, now=0.0, lost=(), max_level=None):
        """One pass: snapshot if ready, then grow if the lockstep rule allows.

        Takes: me (this copy's drone), now (caller's clock, stamped on a new snapshot), lost
        (side-mates gone quiet), max_level (never grow to a level above it; the caller's mission
        limit or "no time left").
        Returns: a LevelStep.
        """
        if self.stuck():
            self._stuck += 1
            if self._stuck >= STUCK_PASSES and self.recover():
                self._stuck = 0
        else:
            self._stuck = 0
        self.snapshot(now)
        own = self.state(me)
        verdict = self.gate.check(own, lost)
        if verdict is Verdict.GO and max_level is not None and self.level >= max_level:
            return LevelStep(Step.AT_CAP, self.level)
        if verdict is not Verdict.GO:
            return LevelStep(_STEP_OF[verdict], self.level)
        return self.grow()

    def grow(self):
        """Grow one level and recheck. Takes no notice of the gate: call it when it said go.

        Does: set_level(L + 1), apply_growth, advance. Infeasible: rollback to L and cap there.
        Returns: a LevelStep (GREW or CAPPED); NOT_READY / AT_CAP when it was not allowed.
        """
        pl = self.pl
        if not self.ready():
            return LevelStep(Step.NOT_READY, self.level)
        if self.capped:
            return LevelStep(Step.AT_CAP, self.level)
        level = self.level
        t0 = time.perf_counter()
        pl.routes.set_level(level + 1)
        res = mine_rule.apply_growth(pl)
        if res.feasible:
            pl.advance()
            self.book.refresh(pl.routes.in_zone)
        else:
            self.rollback(level)
        self.last_seconds = time.perf_counter() - t0
        pl.log.event("grow" if res.feasible else "grow-capped", level + 1, res.reason)
        if res.feasible:
            self.growth_log.append((level + 1, self.last_seconds))
            return LevelStep(Step.GREW, level + 1, self.last_seconds, res.cut, res.bridged)
        return LevelStep(Step.CAPPED, level, self.last_seconds, res.cut, res.bridged, res.reason)

    def rollback(self, level):
        """Put the zones back at `level` and cap there for good. The chain must already be
        valid at `level` (zones only shrank)."""
        self.pl.routes.set_level(level)
        self.cap = level
        self.book.refresh(self.pl.routes.in_zone)

    def growth_fits(self, remaining):
        """Whether `remaining` (the caller's time left, in seconds) allows another growth step:
        at least TIME_MARGIN x the last measured growth + recheck time (always true before the
        first measurement)."""
        return self.last_seconds is None or remaining >= TIME_MARGIN * self.last_seconds

    def after_obstacle(self):
        """A mine or blocked square was just added (the mine rule has run). With no route left
        at a grown level: recover. Otherwise recount the snapshots' unsafe squares."""
        if self.pl.failure and self.level > 0:
            self.recover()
        elif len(self.book):
            self.book.refresh(self.pl.routes.in_zone)

    def stuck(self):
        """At a grown level, not met, aiming at a partner's point A that this side cannot close
        on (a stand-in, or no route), with nothing at all left to photograph: growth walled the
        two sides apart. Waiting cannot fix it; stepping down a level can."""
        pl = self.pl
        return (self.level > 0 and not pl.failure and not pl.met and pl.target.has_partner
                and not pl.target.may_close()
                and not any(seq for group in pl.groups() for _key, seq in group))

    # -- recovering from a late mine ------------------------------------------------------------
    def recover(self):
        """A mine at level > 0 left the planner without a route (pl.failure). Step down level
        by level, each time rebuilding the chain from the latest snapshot at or below that level
        and rechecking it against the current zones, until a route survives.

        Returns: whether the planner has a route again (cap = that level). Otherwise the failure
        stands (level 0 has no lower level to fall back to).
        """
        pl = self.pl
        reason = pl.failure
        for level in range(self.level - 1, -1, -1):
            pl.failure = None
            pl.routes.set_level(level)
            if self._rebuild_from_snapshot(level):
                self.cap = level
                self.book.refresh(pl.routes.in_zone)
                pl.log.event("recovered", level)
                return True
        pl.failure = reason
        return False

    def _rebuild_from_snapshot(self, level):
        pl = self.pl
        snap = next((s for s in reversed(self.book.snapshots) if s.level <= level), None)
        if snap is None:
            return False
        pts = [tuple(p) for p in snap.route]
        nodes = [PathNode(*pts[0], Placer.ENTRY)]
        nodes += [PathNode(*p, Placer.FORWARD) for p in pts[1:-1]] + [PathNode(*pts[-1], Placer.TARGET)]
        links = [Link(LinkKind.CHECKED, [pts[i], pts[i + 1]]) for i in range(len(pts) - 1)]
        if pl.target.has_partner and math.dist(pts[-1], pl.target.partner_a) > ROUTE_MATCH_FT:
            route, _ = pl.target.route_from(pts[-1])
            if route is not None:                     # the partner's point A moved on since
                nodes[-1] = PathNode(*pts[-1], Placer.FORWARD)
                nodes.append(PathNode(*route[-1], Placer.TARGET))
                links.append(Link(LinkKind.FORWARD, route))
        pl.use_chain(Chain(nodes, links))
        if not mine_rule.apply_growth(pl).feasible:
            return False
        pl.advance()
        return True

    # -- reporting ----------------------------------------------------------------------------
    def valid_snapshots(self):
        """The snapshots with no known mine on their squares, oldest first."""
        return self.book.valid_snapshots(self.pl.discovered)

    def report_snapshot(self, level=None):
        """The snapshot that would be reported: the newest valid one (at `level` if given). Once
        the partner is lost, one that runs edge to edge is preferred. None if there is none."""
        mines = self.pl.discovered
        if level is None and self.gate.partner_gone:
            snap = self.book.latest_valid(mines, spans=True)
            if snap is not None:
                return snap
        return self.book.latest_valid(mines, level=level)

    def report_route(self, level=None):
        """The route to report: the newest valid snapshot's. With no valid one: the live route if
        it is whole and checked, else the newest snapshot, else the live route. With `level`:
        that level's newest valid snapshot's route, or None."""
        snap = self.report_snapshot(level)
        if snap is not None:
            return list(snap.route)
        if level is not None:
            return None
        if self.pl.route_verified() or self.book.latest() is None:
            return self.pl.route()
        return list(self.book.latest().route)

    def report_valid(self):
        """Whether report_route() is a valid snapshot."""
        return self.report_snapshot() is not None

    def buffer_width(self):
        """The reported route's clearance: the city-block distance, in squares, from its cell
        path within which there is no known mine = (smallest distance from a known mine's square
        to the nearest path square) - 1. 0 = a mine next to the path; -1 = one on it. With no
        known mine: the largest distance in the field."""
        snap = self.report_snapshot()
        if snap is not None:
            return self.book.buffer_width(snap, self.pl.discovered)
        route = self.report_route()
        cells = cell_path(route, self.pl.field_w, self.pl.field_h) if route else []
        return clearance(cells, self.pl.discovered, self.book.none_known)

    # -- joining at a level -------------------------------------------------------------------
    def join_at(self, level, partner_route, partner_hash=None):
        """This copy's snapshot route at `level` joined with the partner's route at the same
        level, or None.

        Same hash as the partner's snapshot (same mines, same level): the join is two stretches
        of snapshots that each clear those zones, so only the join itself is needed. Otherwise
        the joined route must also clear the zones at `level` of every mine this copy knows.
        """
        snap = self.book.latest_valid(self.pl.discovered, level=level)
        if snap is None:
            return None
        pl = self.pl
        joined = join_mod.join(list(snap.route), [tuple(p) for p in partner_route],
                               pl.routes, pl.coverage, pl.log)
        if joined is None:
            return None
        if partner_hash is not None and partner_hash == snap.mine_hash:
            return joined
        return joined if self.clears_at(level, joined) else None

    def clears_at(self, level, route):
        """Whether `route` stays out of every zone at `level` (built from all mines and blocked
        squares known now; the live engine when `level` is the current one)."""
        pl = self.pl
        if level == self.level:
            return not pl.engine.crosses(route)
        key = (level, len(pl.discovered), len(pl.blocked))
        if self._scratch is None or self._scratch[0] != key:
            eng = make_engine(pl.config.engine, pl.field_w, pl.field_h)
            for m in pl.discovered:
                eng.add_mine(*m)
            for c, r in sorted(pl.blocked):
                eng.add_square(c, r)
            eng.set_level(level)
            self._scratch = (key, eng)
        return not self._scratch[1].crosses(route)
