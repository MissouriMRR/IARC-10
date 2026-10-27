"""Side coordination: who flies what, and keeping the drones apart.

SideCoordinator turns the planner's priority groups of work into one photo queue per drone of
the side. It owns the claims book (claims.py) and the keep-apart policy; the planner owns the
mission state it reads (work, coverage, config).
"""

import math

from flight.pathfinding.pathnode.claims import MATCH_FT, ClaimBook, _tupled
from flight.pathfinding.pathnode.config import GROUPS
from flight.pathfinding.pathnode.coverage import cell_centre
from flight.pathfinding.pathnode.geometry import CELL


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
        around = [p for d, p in positions.items() if d != me and p is not None]
        around += list(others.values())
        spot = self._spare_spot(jobs[0], here, around + jobs, spot_gap)
        if spot is not None:
            self.last_plan[me] = [(spot[0], spot[1], len(GROUPS) - 1)]

    def _spare_spot(self, job, here, avoid, spot_gap):
        """The nearest point to `here` on a ring around `job` that is inside the field, at
        least spot_gap from every point in `avoid`, and out of reach of every needed square --
        (x, y), or None."""
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
                if any(math.dist((x, y), p) < spot_gap for p in avoid):
                    continue
                if any(math.dist(cell_centre(c, r), (x, y)) <= reach for c, r in needed):
                    continue
                spots.append((math.dist((x, y), here), x, y))
            if spots:
                _d, x, y = min(spots)
                return x, y
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
