"""Side coordination: who flies what, and keeping the drones apart.

SideCoordinator turns the planner's priority groups of work into one photo queue per drone of
the side. It owns the claims book (claims.py) and the keep-apart policy; the planner owns the
mission state it reads (work, coverage, config).
"""

import math

from flight.pathfinding.pathnode.claims import MATCH_FT, ClaimBook, _tupled
from flight.pathfinding.pathnode.config import GROUPS
from flight.pathfinding.pathnode.coverage import cell_centre
from flight.pathfinding.pathnode.geometry import CELL, dist_to_segment


LANE_PHOTOS = 3     # a lower drone's lane: where it is, then its next this many photos


def _polyline_gap(p, points):
    """Distance from point p to the polyline through `points` (one point: to that point)."""
    if len(points) == 1:
        return math.dist(p, points[0])
    return min(dist_to_segment(p, a, b) for a, b in zip(points, points[1:]))


def _segment_gap(a, b, c, d):
    """Closest distance between segment a-b and segment c-d (0 when they cross)."""
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    if side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0:
        return 0.0
    return min(dist_to_segment(a, c, d), dist_to_segment(b, c, d),
               dist_to_segment(c, a, b), dist_to_segment(d, a, b))


def _lane_gap(a, b, points):
    """Closest distance between segment a-b and the polyline through `points`."""
    if len(points) == 1:
        return dist_to_segment(points[0], a, b)
    return min(_segment_gap(a, b, c, d) for c, d in zip(points, points[1:]))


class SideCoordinator:
    def __init__(self, drones, config, work, coverage, field_w, field_h):
        """
        Takes:
        - drones: the side's drone ids (claim keys), in tie-break order -- ascending id, so the
          lower id claims first.
        - config: a PlannerConfig (keep_away_ft, keep_clear_ft, shape_size_ft); the planner
          re-assigns `config` when it is reconfigured.
        - work, coverage: the planner's Work and Coverage (needed squares, aim radius).
        - field_w, field_h: field size in feet.
        """
        self.drones = tuple(drones)
        self.config = config
        self.work = work
        self.coverage = coverage
        self.field_w, self.field_h = float(field_w), float(field_h)
        self.claims = ClaimBook(self.drones)
        self.last_plan = {d: [] for d in self.drones}

    def clear(self):
        """Nothing to fly (the planner has failed): empty queues for every drone."""
        self.last_plan = {d: [] for d in self.drones}
        return self.last_plan

    def plan(self, groups, positions, held=None, me=None, others=None, lost=()):
        """
        Takes:
        - groups: the photos still to take, one list per Group (PathNodePlanner.groups).
        - positions, held, me, others, lost: as PathNodePlanner.plan.

        Does: updates the claims (claims.py), keeps the drones apart (keep_away_ft,
        keep_clear_ft), lets `me` take over idle work, and sets each drone's announced job.

        Returns: drone id -> [(x, y, group), ...] in flying order. Also kept as last_plan;
        each drone's current job is claims.heads[drone].
        """
        others = {d: tuple(p) for d, p in (others or {}).items()
                  if p is not None and d not in self.drones}
        held = held or {}
        lost = {d for d in lost if d in self.drones and d != me and positions.get(d) is not None}
        side_held = {d: job for d, job in held.items() if d in self.drones}
        other_held = {d: job for d, job in held.items() if d in others}
        reserved = [_tupled(job) for d, job in side_held.items() if d in lost and job is not None]
        claim_pos = {d: (None if d in lost else p) for d, p in positions.items()}
        plan = self.claims.update(self._without(groups, reserved), claim_pos, side_held)
        idle = {d for d, job in side_held.items() if job is None and d not in lost}
        kept = self._keep_apart(plan, positions, idle, others, other_held, lost, side_held)
        if me is not None and (not plan.get(me) or (not kept.get(me)
                                                    and idle >= set(self.drones) - {me})):
            self._take_over(me, plan, side_held, lost)
            kept = self._keep_apart(plan, positions, idle, others, other_held, lost, side_held)
        self.last_plan = kept
        if me is not None:
            self._make_way(me, plan, positions, {**side_held, **other_held}, others)
        self._announce_heads(me, plan)
        return self.last_plan

    # -- taking over the side-mates' queued work ----------------------------------------------
    def _take_over(self, me, plan, side_held, lost):
        """`me` has nothing to fly: move the other drones' queued photos into its queue, except
        every sequence a side-mate announced (that drone is flying it: any photo of it only
        sends this drone along the same path). The side-mate's own copy may not have the other
        work at all (copies can differ while messages are in flight), so nobody else would do
        it."""
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

    def _announce_heads(self, me, plan):
        """Each drone's job is what it will actually fly first, so a job it had to give up
        isn't announced -- and held from the other drone -- for nothing. The exception is a
        drone held back by a drone it must not fly near (it has work but nothing it may fly,
        or only a spare photo _make_way sent it to): it announces the job it waits for, so
        the other drone (whose copy may not know the job) makes way for it."""
        for d, queue in self.last_plan.items():
            if d == me and plan.get(me) and (not queue or self.claims.job_at(queue[0]) is None):
                queue = plan[me]
            self.claims.heads[d] = self.claims.job_at(queue[0]) if queue else None

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

    # -- keeping the drones apart -------------------------------------------------------------
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
          would free the job; or
        - any drone with nothing to do itself, when it stands within keep_clear_ft of the leg
          to the next photo of a side-mate held back by _keep_apart, or to the announced job of
          any other drone (the flight code would hold that leg until the idle drone moved: never)."""
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
        legs = self._blocked_legs(me, here, plan, positions, held, others, clear)
        jobs = jobs or [leg[1] for leg in legs]
        if not jobs:
            return
        around = [p for d, p in positions.items() if d != me and p is not None]
        around += list(others.values())
        if self._is_spare(here, around + jobs, spot_gap, legs):
            return      # already out of the way: stay (a hop to the nearest ring point can flip-flop)
        spot = self._spare_spot(jobs[0], here, around + jobs, spot_gap, legs)
        if spot is not None:
            self.last_plan[me] = [(spot[0], spot[1], len(GROUPS) - 1)]

    def _blocked_legs(self, me, here, plan, positions, held, others, clear):
        """The legs (from, to) of other drones that pass within `clear` of `here`:
        - a side-mate's leg to the first photo this copy dropped for it (_keep_apart);
        - any other drone's leg to the job it announced -- its own copy may hold work this one
          doesn't, so the announcement is all this copy can know."""
        legs = [(tuple(positions[d]), tuple(queue[0][:2])) for d, queue in plan.items()
                if d != me and queue and not self.last_plan.get(d) and positions.get(d) is not None]
        where = {**positions, **others}
        legs += [(tuple(where[d]), tuple(map(float, job[2]))) for d, job in held.items()
                 if d != me and job is not None and where.get(d) is not None]
        return [leg for leg in legs if dist_to_segment(here, *leg) < clear]

    def _is_spare(self, p, avoid, spot_gap, legs=()):
        """Whether p is a spare spot: inside the field (a photo's half-footprint from the edge),
        at least spot_gap from every point in `avoid` and from every (from, to) leg in `legs`,
        and out of reach of every needed square."""
        margin = max(self.config.shape_size_ft) / 2
        if not (margin <= p[0] <= self.field_w - margin and margin <= p[1] <= self.field_h - margin):
            return False
        if any(math.dist(p, q) < spot_gap for q in avoid):
            return False
        if any(dist_to_segment(p, *leg) < spot_gap for leg in legs):
            return False
        reach = self.coverage.aim_radius() + CELL
        return not any(math.dist(cell_centre(c, r), p) <= reach
                       for c, r in self.work.needed_squares())

    def _spare_spot(self, job, here, avoid, spot_gap, legs=()):
        """The nearest point to `here` on a ring around `job` that is a spare spot
        (_is_spare) -- (x, y), or None."""
        for radius in (spot_gap + 1.0, spot_gap + 4.0, spot_gap + 8.0):
            spots = [(math.dist(p, here), p) for p in (
                (job[0] + radius * math.cos(k * math.pi / 8),
                 job[1] + radius * math.sin(k * math.pi / 8)) for k in range(16))
                if self._is_spare(p, avoid, spot_gap, legs)]
            if spots:
                return min(spots)[1]
        return None

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
        # Lower ids first: a higher drone keeps clear of the lower drones' kept queues too.
        for d, queue in sorted(plan.items(), key=lambda item: self.drones.index(item[0])):
            near = []           # (point, distance to keep from it)
            lanes = []          # (polyline a lower drone will fly, distance to keep from it)
            peers = [p for o, p in list(positions.items()) + list(others.items())
                     if o != d and p is not None]       # where the other drones are now
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
                            lanes.append(([tuple(positions[o]), tuple(side_held[o][2])], gap))
                    elif working:
                        near.append((plan[o][0][:2], gap))
                        lanes.append(self._lane(positions[o], out.get(o, plan[o]), gap))
                else:
                    near.append((positions[o], min(gap, clear)))
            for o, pos in others.items():
                if o < d:
                    job = other_held.get(o)
                    working = o not in other_held or job is not None
                    near.append((pos, gap if working else min(gap, clear)))
                    if job is not None:
                        near.append((job[2], gap))
                        lanes.append(([tuple(pos), tuple(job[2])], gap))
                else:
                    near.append((pos, min(gap, clear)))
            out[d] = self._flyable(positions.get(d), queue, near, peers, clear, lanes)
        return out

    @staticmethod
    def _lane(position, queue, gap):
        """The path a drone is about to fly: where it is, then its next LANE_PHOTOS photos."""
        return ([tuple(position)] + [tuple(q[:2]) for q in queue[:LANE_PHOTOS]], gap)

    @staticmethod
    def _flyable(start, queue, near, peers, leg_clear, lanes=()):
        """The photos of `queue` the drone may fly, in order, from `start`:
        - each at least r from every (point, r) in `near`;
        - each reached by a leg at least r from every (polyline, r) in `lanes` -- the paths lower
          drones are flying: those drones are moving, so where they stand now says little about
          where they will be when this leg is flown. A leg that only moves away from a lane the
          drone is already inside r of is fine;
        - each reached by a leg that stays leg_clear from every peer position, unless the leg
          only leaves a peer it is already that near (the flight code holds a leg that comes
          within its radius of a peer, and an idle peer never moves: the hold only ends in an
          abort)."""
        kept, here = [], start
        for q in queue:
            pt = tuple(q[:2])
            if any(math.dist(pt, p) < r for p, r in near):
                continue
            if here is not None and any(
                    dist_to_segment(p, here, pt) < min(leg_clear, math.dist(here, p)) for p in peers):
                continue
            if here is not None and any(
                    _lane_gap(here, pt, points) < (r if _polyline_gap(here, points) >= r
                                                   else _polyline_gap(here, points) - 0.25)
                    for points, r in lanes):
                continue
            kept.append(q)
            here = pt
        return kept
