"""Who photographs what: priority groups of waypoints, claimed by the drones of one side.

Groups
- A list of groups, highest priority first (see PathNodePlanner.groups). Each group is a list
  of (key, sequence); a sequence is the photos along one path (the forward gap, a
  redirection), in flying order from its start. A free-standing photo is a sequence of one.
- key names the path the sequence belongs to -- ("span", end-node xy), "forward" or
  ("shortcut", a, b) -- or is None for a free-standing photo.
- update() deals every group on each pass, in priority order (a lower group is dealt with
  the higher groups' claims already counted into each drone's route), and each drone flies
  everything it holds in a higher group before anything in a lower one. A drone with nothing
  left in a higher group takes from a lower one even while the higher is still being flown.
- Within a group a drone flies its paths whole, nearest start first -- except the forward
  gap, which is open-ended and always goes last (short redirections near point A are
  cheaper to clear on the way than to come back for). Free-standing photos go nearest first.
- A sequence is claimed whole: splitting one path between two drones only sends the second
  one ahead to ground the first is about to cover anyway -- and the far end of a path is the
  part most likely to be rerouted before anyone gets there.

Claiming (ClaimBook.update) -- a growing path radius
- Picture a radius r growing from 0. A drone may claim a sequence once its route to the
  sequence's start -- from where it is, through everything it already holds -- is no longer
  than r. Its route then runs on through the sequence, so its next claim is measured from
  the sequence's end.
- Implemented without stepping r: repeatedly, the drone that can start an unclaimed
  sequence soonest takes it.
- So a drone's current claims raise its threshold for new ones, and nothing is sent across
  the field while another drone is nearer. Whoever is at point A holds the whole path ahead
  of it; a redirection behind it tends to go to the other drone. No drone is special: either
  pioneers whenever it is the better placed.
- Deterministic: ties go by drone order, then by sequence start. Any drone with the same
  inputs computes the same claims -- which drone will pick up what is known exactly.

A drone whose position is None is absent (gone): it holds nothing and is handed nothing,
so its work goes back to the others.

Claims between updates: only what each drone is working on now is kept -- the sequence its
next photo belongs to. Everything else is handed out afresh from where the drones are now,
since a claim made from across the field goes stale as drones move.
- A kept path keeps its owner however much its photos move (a replan shifts every photo
  along a path; that shouldn't hand it to whoever happens to be a foot nearer its new start).
- A kept free-standing photo is matched within MATCH_FT, since photos are regenerated as
  coverage changes.
"""

import math

MATCH_FT = 2.0      # a regenerated waypoint this close to a claimed one keeps its claim


def _tupled(value):
    """value with every list turned into a tuple -- a job announced over the network comes
    back from JSON as lists."""
    if isinstance(value, (list, tuple)):
        return tuple(_tupled(v) for v in value)
    return value


def _nearest(frm, pts):
    """The point of pts nearest frm (ties by coordinates)."""
    return min(pts, key=lambda p: (math.dist(frm, p), p))


def path_length(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _sid(key, seq):
    """A sequence's id within its group: its key, or ("loose", point) for a free-standing
    photo. Claims are held per sequence, never per point: two paths can share a photo point
    (a redirection starting where the forward gap does), and must not share an owner by it."""
    return key if key is not None else ("loose", seq[0])


class ClaimBook:
    def __init__(self, drones):
        """drones: the side's drone ids, in tie-break order (ascending)."""
        self.drones = tuple(drones)
        self.claims = {}            # (group index, sequence id) -> drone key
        self.heads = {}             # drone key -> (group index, path key, (x, y)) of its next photo
        self._groups = []           # this update's groups: [[(key, sequence), ...], ...]

    def _inherit(self, groups, active):
        """Keep each active drone's current job: the path (by key) or free-standing photo (by
        proximity) its next photo belonged to."""
        kept = {}
        for d in active:
            head = self.heads.get(d)
            if head is None:
                continue
            g, key, pt = head
            if g >= len(groups):
                continue
            if key is not None:
                if any(k == key for k, _seq in groups[g]) and (g, key) not in kept:
                    kept[(g, key)] = d
            else:
                loose = [seq[0] for k, seq in groups[g]
                         if k is None and (g, _sid(k, seq)) not in kept]
                if loose:
                    p = _nearest(pt, loose)
                    if math.dist(p, pt) <= MATCH_FT:
                        kept[(g, ("loose", p))] = d
        self.claims = kept

    def job_at(self, entry):
        """The job -- (group, path key, (x, y)) -- a queued photo (x, y, group) of the last
        update belongs to; None if it is in none of this update's groups."""
        x, y, g = entry
        for key, seq in self._groups[g] if g < len(self._groups) else []:
            if (x, y) in seq:
                return (g, key, (x, y))
        return None

    def _queue(self, drone, start, upto):
        """This drone's claims in groups 0..upto, in flying order (see module doc).
        Returns (route [((x, y), group, key), ...], length)."""
        route, cur, length = [], start, 0.0
        for g in range(upto + 1):
            units, last = [], []
            for key, seq in self._groups[g]:
                if self.claims.get((g, _sid(key, seq))) != drone:
                    continue
                (last if key == "forward" else units).append((key, seq))
            while units:
                key, seq = min(units, key=lambda u: (math.dist(cur, u[1][0]), u[1][0]))
                units.remove((key, seq))
                length += math.dist(cur, seq[0]) + path_length(seq)
                route.extend((p, g, key) for p in seq)
                cur = seq[-1]
            for key, seq in last:
                length += math.dist(cur, seq[0]) + path_length(seq)
                route.extend((p, g, key) for p in seq)
                cur = seq[-1]
        return route, length

    def update(self, groups, positions, held=None):
        """
        Takes:
        - groups: list of groups, highest priority first; each a list of
          (key, [(x, y), ...]) (see module doc).
        - positions: drone key -> (x, y), where each drone is now; None for a drone that is
          absent (gone). Every drone of the side must have an entry.
        - held: drone -> (group, key, (x, y)), the job another drone has announced it is
          working on. It stands in for this book's own record of that drone's job, so every
          copy honours it; if two drones hold the same job, the first in drone order keeps it.

        Does: keeps valid claims, then hands out every unclaimed sequence (see module doc).

        Returns: drone key -> [(x, y, group index), ...] in flying order.
        """
        norm = []
        for seqs in groups:
            out, loose_starts = [], set()
            for key, seq in seqs:
                seq = list(dict.fromkeys(seq))
                if not seq:
                    continue
                if key is None:
                    if seq[0] in loose_starts:
                        continue            # the same free-standing photo twice
                    loose_starts.add(seq[0])
                out.append((key, seq))
            norm.append(out)
        groups = norm
        self._groups = groups
        active = [d for d in self.drones if positions[d] is not None]
        for d, job in (held or {}).items():
            if d in self.drones:
                self.heads[d] = None if job is None else _tupled(job)
        self._inherit(groups, active)
        for g, seqs in enumerate(groups):
            open_seqs = sorted(((key, seq) for key, seq in seqs
                                if (g, _sid(key, seq)) not in self.claims),
                               key=lambda ks: ks[1][0])
            if not open_seqs:
                continue
            if not active:
                break
            tail, cost = {}, {}
            for d in active:
                route, cost[d] = self._queue(d, positions[d], g)
                tail[d] = route[-1][0] if route else positions[d]
            while open_seqs:
                best = None
                for order, d in enumerate(active):
                    for i, (_key, seq) in enumerate(open_seqs):
                        rank = (cost[d] + math.dist(tail[d], seq[0]), order, seq[0])
                        if best is None or rank < best[0]:
                            best = (rank, d, i)
                (start, _order, _p), d, i = best
                key, seq = open_seqs.pop(i)
                self.claims[(g, _sid(key, seq))] = d
                cost[d], tail[d] = start + path_length(seq), seq[-1]
        out = {}
        last = len(groups) - 1
        for d in self.drones:
            if positions[d] is None:
                out[d], self.heads[d] = [], None
                continue
            route, _length = self._queue(d, positions[d], last)
            out[d] = [(x, y, g) for (x, y), g, _key in route]
            if route:
                p, g, key = route[0]
                self.heads[d] = (g, key, p)
            else:
                self.heads[d] = None
        return out
