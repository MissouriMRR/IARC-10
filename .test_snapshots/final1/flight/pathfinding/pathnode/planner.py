"""One side's path-node planner: the core shared by the flight code and the simulator.

A side is one or two drones working from one edge. Every drone keeps its own copy of the
side's planner and plans for every drone of the side; which drone flies what is decided by
claims (claims.py) plus the jobs the drones announce, not by role. Everything is in local
feet; the flight code wraps it in lat/lon (flight/pathfinding/mission_planner.py).

What it is made of (one module each):
- chain.py      the route so far: path nodes joined by checked / pending / forward links
- routes.py     known obstacles (mine zones, blocked squares) and shortest paths around them
- target.py     what the forward gap aims at: the partner side's point A, or the far edge
- coverage.py   photographed squares, what a stretch needs, misses, photo placement
- mine_rule.py  repairing the chain when an obstacle appears
- shortcuts.py  replacing checked sections with shorter paths
- work.py       the photos still to take, in priority groups
- join.py       joining the two sides' routes
- claims.py     which drone takes which work

Order of events for each photo (session.py does this for both the simulator and the flight
code): record_footprint, then add_mine for its mines, then advance() -- path nodes are laid
only after the photo's mines are in.
"""

import dataclasses
import hashlib
import math

from flight.pathfinding.pathnode import join as join_mod
from flight.pathfinding.pathnode import mine_rule
from flight.pathfinding.pathnode.chain import Chain, Link, PathNode
from flight.pathfinding.pathnode.claims import MATCH_FT, ClaimBook, _tupled
from flight.pathfinding.pathnode.config import (GROUPS, Edge, Group, LinkKind, Placer,
                                                PlannerConfig, Source)
from flight.pathfinding.pathnode.coverage import Coverage, cell_centre
from flight.pathfinding.pathnode.engines import make_engine
from flight.pathfinding.pathnode.geometry import CELL, snap_to_cell_center
from flight.pathfinding.pathnode.log import PlannerLog
from flight.pathfinding.pathnode.routes import Routes
from flight.pathfinding.pathnode.shortcuts import Shortcuts
from flight.pathfinding.pathnode.target import Target
from flight.pathfinding.pathnode.work import Work

__all__ = ["PathNodePlanner", "GROUPS", "Group", "Source"]


class PathNodePlanner:
    def __init__(self, field_w, field_h, entry_x, start_edge=Edge.BOTTOM, partner_entry_x=None,
                 drones=(1,), config=None, **options):
        """
        Takes:
        - field_w, field_h: field size in feet (local frame, origin at the bottom-left).
        - entry_x, start_edge: where this side enters (Edge.BOTTOM: y = 0; Edge.TOP: y = h).
        - partner_entry_x: set when a partner side works from the other edge (3-4 drones).
          The target is then the partner's point A, at its entry until the first relay.
          None aims at the far edge.
        - drones: the side's drone ids (claim keys), in tie-break order -- ascending id, so the
          lower id claims first.
        - config: a PlannerConfig; or pass its fields as keyword options.
        """
        self.config = config or PlannerConfig(**options)
        self.field_w, self.field_h = float(field_w), float(field_h)
        self.start_edge = Edge(start_edge)
        self.drones = tuple(drones)
        self.claims = ClaimBook(self.drones)
        self.last_plan = {d: [] for d in self.drones}
        self.discovered = []            # every known mine, (x, y)
        self.blocked = set()            # squares given up on (see coverage.py)
        self.partner_route = []         # the partner's latest route (check_join)
        self.failure = None
        self.log = PlannerLog()
        self.routes = Routes(make_engine(self.config.engine, self.field_w, self.field_h))
        self.target = Target(self.field_w, self.field_h, self.start_edge, partner_entry_x,
                             self.routes)
        self.coverage = Coverage(self.field_w, self.field_h, self.config)
        y0 = 0.0 if self.start_edge == Edge.BOTTOM else self.field_h
        entry = PathNode(float(entry_x), y0, Placer.ENTRY)
        self.chain = Chain([entry], [])
        route, _gids = self.target.route_from(entry.xy)
        if route is None:
            self.failure = "no route from the entry to the target"
        else:
            self.chain = Chain([entry, PathNode(*route[-1], Placer.TARGET)],
                               [Link(LinkKind.FORWARD, route)])
        self.shortcuts = Shortcuts(self.chain, self.routes, self.coverage, self.config, self.log)
        self.work = Work(self.chain, self.routes, self.coverage, self.shortcuts, self.config)

    def use_chain(self, chain):
        """Replace the chain's contents (a hand-built Chain, e.g. Chain.build) -- for tests and
        hand-built cases. Every component keeps its reference to the same Chain object."""
        chain.check()
        self.chain.nodes, self.chain.links = chain.nodes, chain.links

    def reconfigure(self, **changes):
        """Change PlannerConfig fields mid-mission (tests, tuning)."""
        self.config = dataclasses.replace(self.config, **changes)
        for part in (self.coverage, self.shortcuts, self.work):
            part.config = self.config

    # -- state -----------------------------------------------------------------------------
    @property
    def nodes(self):
        return self.chain.nodes

    @property
    def links(self):
        return self.chain.links

    @property
    def engine(self):
        return self.routes.engine

    @property
    def seen(self):
        return self.coverage.seen

    @property
    def stats(self):
        return self.log.stats

    @property
    def events(self):
        return self.log.events

    @property
    def met(self):
        """The forward gap is closed: the chain reaches the target."""
        return self.failure is None and self.chain.forward_index() is None

    def done(self):
        """Nothing left to do on this side: met, nothing pending, rings photographed, no
        shortcut left to try."""
        return (self.met and not self.chain.has_pending()
                and not self.work.adjacent() and not self.work.outer()
                and not self.shortcuts.left_to_try(self.met))

    def route_verified(self):
        """The route is whole and every link of it checked -- safe to report, even with
        optional work (rings, shortcuts) left."""
        return self.met and self.chain.all_checked()

    def point_a(self):
        """This side's point A -- the frontier. Once met, the meeting point."""
        return self.chain.point_a()

    def route(self):
        return self.chain.route()

    def mine_hash(self):
        """Order-independent hash of the known mines (each snapped to its square)."""
        cells = sorted(snap_to_cell_center(x, y) for x, y in self.discovered)
        return hashlib.sha256(repr(cells).encode("utf-8")).hexdigest()

    def footprint_cells(self, x, y):
        return self.coverage.footprint_cells(x, y)

    def cell_seen_at(self, x, y):
        return self.coverage.cell_seen_at(x, y)

    # -- photos ----------------------------------------------------------------------------
    def record_footprint(self, corners):
        """A photo's ground footprint (any quadrilateral, corners in order) as coverage; a miss
        for any needed square it was aimed at and didn't cover (see coverage.py)."""
        needed = self.work.needed_squares() if not self.failure else set()
        cx, cy = self.coverage.record(corners)
        self._misses(cx, cy, needed)

    def record_failed_photo(self, x, y):
        """A photo planned at (x, y) that produced no footprint (camera failure): a miss for
        every needed square it was aimed at."""
        if self.failure:
            return
        self._misses(x, y, self.work.needed_squares())

    def _misses(self, x, y, needed):
        for c, r in self.coverage.count_misses(x, y, needed):
            self._block_square(c, r)

    def _block_square(self, c, r):
        """Give up on square (c, r): make it an obstacle and apply the mine rule to it."""
        if (c, r) in self.blocked or self.failure:
            return
        self.blocked.add((c, r))
        self.log.count("blocked_squares")
        self.log.event("blocked", c, r)
        mine_rule.apply(self, self.routes.add_square(c, r), Source.PARTNER)

    # -- mines -----------------------------------------------------------------------------
    def add_mine(self, x, y, source):
        """Apply the mine rule (mine_rule.py). source: Source.PAIR (found by this side's
        drones) or Source.PARTNER (relayed). Returns whether the mine was new -- a mine in a
        square that already has one is the same mine."""
        source = Source(source)
        if self.failure:
            return False
        cell = snap_to_cell_center(x, y)
        if any(snap_to_cell_center(*m) == cell for m in self.discovered):
            return False
        self.discovered.append((float(x), float(y)))
        mine_rule.apply(self, self.routes.add_mine(x, y), source)
        return True

    # -- the partner side ------------------------------------------------------------------
    def set_partner_point_a(self, x, y):
        """Retarget the forward gap at the partner's latest point A. Once met, only stored."""
        target = self.target
        if not target.partner or self.failure:
            return
        if (float(x), float(y)) == target.partner_a:
            return      # the heartbeat resends an unchanged point A; new mines replan on their own
        target.partner_a = (float(x), float(y))
        f = self.chain.forward_index()
        if f is None:
            return
        route, _ = target.route_from(self.chain.nodes[f].xy)
        if route is None:
            self.log.event("stale-target", target.partner_a)
            return
        self.chain.nodes[f + 1] = PathNode(*route[-1], Placer.TARGET)
        self.chain.links[f].poly = route
        # The new target can land on ground that is already checked -- or exactly on our own
        # point A once the partner has reached it. Nothing else would advance the chain then
        # (no photo is left to take), so the gap would never close.
        self.advance()

    def join(self, partner_route):
        """This side's route joined with the partner's (join.py), or None."""
        return join_mod.join(self.route(), partner_route, self.routes, self.coverage, self.log)

    def check_join(self, partner_route):
        """
        Takes: the partner side's latest route.

        Does: once met, checks this side's route joins it. Under relay loss both sides can
        meet crosswise -- each closing on a point A the other has since abandoned -- and then
        neither route contains the other's end. The bottom side yields: it re-opens its gap
        toward the partner's route end, which is on the partner's route by definition. The top
        side only waits, so the two never chase each other.

        Returns: the joined route, or None.
        """
        self.partner_route = [tuple(p) for p in partner_route]
        if not self.met or len(self.partner_route) < 2:
            return None
        joined = self.join(self.partner_route)
        if joined is None and self.start_edge == Edge.BOTTOM:
            self._reopen_toward(self.partner_route[-1])
        return joined

    def _reopen_toward(self, point):
        """Turn the met target into an ordinary node and open a forward gap to `point`."""
        self.target.partner_a = (float(point[0]), float(point[1]))
        end = self.chain.nodes[-1]
        route, _gids = self.target.route_from(end.xy)
        if route is None:
            self.log.event("reopen-failed", self.target.partner_a)
            return
        self.chain.nodes[-1] = PathNode(end.x, end.y, Placer.FORWARD)
        self.chain.nodes.append(PathNode(*route[-1], Placer.TARGET))
        self.chain.links.append(Link(LinkKind.FORWARD, route))
        self.log.count("reopens")
        self.log.event("reopen", end.xy, self.target.partner_a)
        self.advance()

    # -- planning --------------------------------------------------------------------------
    def advance(self):
        """Lay path nodes over everything now checked -- the forward gap from point A and
        every pending span from its start -- then update the shortcuts."""
        if self.failure:
            return
        chain = self.chain
        seen = self.coverage.all_seen
        f = chain.forward_index()
        # Never close the gap against a stand-in target (see target.py).
        if f is not None and (not self.target.partner or self.target.is_partner):
            chain.advance_link(chain.links[f], Placer.FORWARD, seen)
        for link in [k for k in chain.links if k.kind == LinkKind.PENDING]:
            chain.advance_link(link, Placer.SPAN, seen)
        self.shortcuts.update(self.met)

    def groups(self):
        """The photos still to take, one list per Group (work.py)."""
        if self.failure:
            return [[] for _ in Group]
        return self.work.groups()

    def plan(self, positions, held=None, me=None, others=None, lost=()):
        """
        Takes:
        - positions: drone id -> (x, y), where each of the side's drones is; None for a drone
          that is gone (ignored entirely: no claims, no keep-apart).
        - held: drone id -> the job it announced (claims.ClaimBook.update). The side-mates'
          jobs are held from them; the other side's only steer the drones apart.
        - me: the drone this copy of the planner belongs to, if any. If this copy gives it
          nothing to fly -- no work, or only work it must keep clear of, while its side-mates
          announced they have no job -- it takes work the others hold in their queues, except
          every sequence a side-mate announced (that drone is flying it: any photo of it only
          sends this drone along the same path). The side-mate's own copy may not have the
          other work at all (copies can differ while messages are in flight), so nobody else
          would do it.
        - others: drone id -> (x, y) for the other side's drones -- the last known position
          if one has gone quiet (the caller drops it only after a long silence); None or
          missing is ignored. Its work is not ours (its planner is its own) -- we only keep
          clear of it, by the same rule as within the side.
        - lost: side drone ids that went quiet. Their LAST KNOWN position is in positions and
          last announced job in held. Only their work is handed to the others, and even then
          the sequence of the job a lost drone last announced stays reserved (it is probably
          still flying it), and its last position and job keep blocking (keep-apart, the same
          distances as a live drone). The caller drops it from `lost` (position None) after a
          long silence, and when it is heard again it is an ordinary drone (its announced
          job wins as usual).

        Does: advance(), then updates the claims (claims.py), keeps the drones apart
        (keep_away_ft, keep_clear_ft).

        Returns: drone id -> [(x, y, group), ...] in flying order. Also kept as last_plan;
        each drone's current job is claims.heads[drone].
        """
        if self.failure:
            self.last_plan = {d: [] for d in self.drones}
            return self.last_plan
        self.advance()      # anything already checked must close before we plan more photos
        others = {d: tuple(p) for d, p in (others or {}).items()
                  if p is not None and d not in self.drones}
        held = held or {}
        lost = {d for d in lost if d in self.drones and d != me and positions.get(d) is not None}
        side_held = {d: job for d, job in held.items() if d in self.drones}
        other_held = {d: job for d, job in held.items() if d in others}
        reserved = [_tupled(job) for d, job in side_held.items() if d in lost and job is not None]
        claim_pos = {d: (None if d in lost else p) for d, p in positions.items()}
        plan = self.claims.update(self._without(self.groups(), reserved), claim_pos, side_held)
        idle = {d for d, job in side_held.items() if job is None and d not in lost}
        kept = self._keep_apart(plan, positions, idle, others, other_held, lost, side_held)
        if me is not None and (not plan.get(me) or (not kept.get(me)
                                                    and idle >= set(self.drones) - {me})):
            announced = [_tupled(job) for d, job in side_held.items()
                         if job is not None and d != me and d not in lost]
            plan[me] = plan.get(me, []) + [q for d in self.drones if d != me for q in plan[d]
                                           if not self._is_announced(q, announced)]
            # Work taken over is no longer the other drone's next photo: left there, it would
            # keep this drone away from its own new queue (keep-apart) and nobody would fly it.
            taken = {tuple(map(float, q[:2])) for q in plan[me]}
            for d in self.drones:
                if d != me:
                    plan[d] = [q for q in plan[d] if tuple(map(float, q[:2])) not in taken]
            kept = self._keep_apart(plan, positions, idle, others, other_held, lost, side_held)
        self.last_plan = kept
        if me is not None:
            self._make_way(me, plan, positions, {**side_held, **other_held}, others)
        # Each drone's job is what it will actually fly first, so a job it had to give up here
        # isn't announced -- and held from the other drone -- for nothing. The exception is a
        # drone held back by a drone it must not fly near (it has work but nothing it may fly,
        # or only a spare photo _make_way sent it to): it announces the job it waits for, so
        # the other drone (whose copy may not know the job) makes way for it.
        for d, queue in self.last_plan.items():
            if d == me and plan.get(me) and (not queue or self.claims.job_at(queue[0]) is None):
                queue = plan[me]
            self.claims.heads[d] = self.claims.job_at(queue[0]) if queue else None
        return self.last_plan

    @staticmethod
    def _same_sequence(job, g, key, pt):
        """Whether an announced job (group, key, (x, y)) is the sequence with this group, key
        (None: a free-standing photo) and first/next photo pt."""
        if job[0] != g:
            return False
        if key is not None:
            return job[1] == key
        return job[1] is None and math.dist(job[2], pt) <= MATCH_FT

    def _without(self, groups, jobs):
        """groups minus every sequence one of `jobs` belongs to."""
        if not jobs:
            return groups
        return [[(key, seq) for key, seq in seqs
                 if not seq or not any(self._same_sequence(j, g, key, seq[0])
                                      for j in jobs)]
                for g, seqs in enumerate(groups)]

    def _is_announced(self, entry, jobs):
        """A queued photo belongs to a sequence one of `jobs` announced (same path key, or the
        same free-standing photo)."""
        mine = self.claims.job_at(entry)
        return mine is not None and any(self._same_sequence(j, *mine) for j in jobs)

    def _make_way(self, me, plan, positions, held, others):
        """Break a keep-apart standoff by moving out of the way. The drone is sent to a spare
        photo on ground where none is needed, clear of the job and of the other drones (the
        last group, so it is not a job). Whoever must give way:
        - the higher-id drone, when a lower-id drone announced a job within keep_away_ft of
          it (a lower id keeps its ground: ties go that way); or
        - a drone that is not the side's lowest id, when every drone's queue was emptied by
          _keep_apart while work is left; or
        - any drone with nothing to do itself, when a higher-id drone announced a job within
          keep_clear_ft of it -- that drone cannot fly a photo next to it, and no other move
          would free the job."""
        gap, clear = self.config.keep_away_ft, self.config.keep_clear_ft
        spot_gap = max(gap, clear) + 0.5
        if (not self.config.keep_away_ft or positions.get(me) is None
                or self.last_plan.get(me)):
            return
        here = positions[me]
        # A lower drone's next photo is kept keep_away_ft clear of this drone (_keep_apart);
        # a higher drone's only keep_clear_ft from an idle one.
        jobs = [tuple(map(float, job[2])) for d, job in held.items()
                if job is not None and d < me and math.dist(job[2], here) < gap]
        if not jobs and not plan.get(me):
            jobs = [tuple(map(float, job[2])) for d, job in held.items()
                    if job is not None and d > me and math.dist(job[2], here) < clear]
        if (not jobs and self.drones.index(me) != 0
                and not any(self.last_plan.values())):
            jobs = [tuple(q[:2]) for queue in plan.values() for q in queue]
            jobs = [min(jobs, key=lambda j: (math.dist(j, here), j))] if jobs else []
        if not jobs:
            return
        job = jobs[0]
        around = [p for d, p in positions.items() if d != me and p is not None]
        around += list(others.values())
        needed = self.work.needed_squares()
        reach = self.coverage.aim_radius() + CELL
        margin = max(self.config.shape_size_ft) / 2
        for radius in (spot_gap + 1.0, spot_gap + 4.0, spot_gap + 8.0):
            spots = []
            for k in range(16):
                a = k * math.pi / 8
                x, y = job[0] + radius * math.cos(a), job[1] + radius * math.sin(a)
                if not (margin <= x <= self.field_w - margin and margin <= y <= self.field_h - margin):
                    continue
                if any(math.dist((x, y), p) < spot_gap for p in around + jobs):
                    continue
                if any(math.dist(cell_centre(c, r), (x, y)) <= reach for c, r in needed):
                    continue
                spots.append((math.dist((x, y), here), x, y))
            if spots:
                _d, x, y = min(spots)
                self.last_plan[me] = [(x, y, len(GROUPS) - 1)]
                return

    def _keep_apart(self, plan, positions, idle=(), others=None, other_held=None, lost=(),
                    side_held=None):
        """Keep the drones physically apart; the photos dropped here stay work for later.
        Ties go to the lower id (every drone of the bottom side is lower than any of the
        top side), so two drones can never block each other:
        - a higher-id drone skips photos within keep_away_ft of a lower-id drone's position
          and of its next photo (of its position only within keep_clear_ft if that drone has
          nothing queued, or announced it has no job -- its own copy has no work, so the
          queue this copy gives it is never flown). A lower drone of the other side has
          no queue here: its announced job stands in for its next photo, and it counts as
          working until it announces none. A lost side-mate counts as working, at its last
          known position and last announced job;
        - a lower-id drone skips only photos within keep_clear_ft of a higher-id drone's
          position (just over the flight code's 1.2 m separation floor)."""
        gap, clear = self.config.keep_away_ft, self.config.keep_clear_ft
        others, other_held, side_held = others or {}, other_held or {}, side_held or {}
        if not gap or (len(self.drones) < 2 and not others):
            return plan
        out = {}
        for d, queue in plan.items():
            near = []           # (point, distance to keep from it)
            for o in self.drones:
                if o == d or positions.get(o) is None:
                    continue
                if self.drones.index(o) < self.drones.index(d):
                    # An idle lower drone stays put, so only the clear distance is needed:
                    # else a job within gap of it, and of the higher drone, is never flown.
                    working = o in lost or (bool(plan.get(o)) and o not in idle)
                    near.append((positions[o], gap if working else min(gap, clear)))
                    if o in lost:
                        if side_held.get(o) is not None:
                            near.append((side_held[o][2], gap))
                    elif working:
                        near.append((plan[o][0][:2], gap))
                else:
                    near.append((positions[o], min(gap, clear)))
            for o, pos in others.items():
                if o < d:
                    job = other_held.get(o)
                    working = o not in other_held or job is not None
                    near.append((pos, gap if working else min(gap, clear)))
                    if job is not None:
                        near.append((job[2], gap))
                else:
                    near.append((pos, min(gap, clear)))
            out[d] = [q for q in queue if all(math.dist(q[:2], p) >= r for p, r in near)]
        return out
