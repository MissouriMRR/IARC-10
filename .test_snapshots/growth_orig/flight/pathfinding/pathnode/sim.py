"""Multi-drone simulation of the path-node planner, driven the way the flight code drives it.

A side is the one or two drones working from one edge. Every drone keeps its own copy of the
side's SearchBackend (search_backend.py; made by make_backend, "pathnode" by default -- the same
class the flight code uses), and works out the whole side's claims itself. The sim talks only
to that contract, so any registered backend runs here (Mission(backend="name")). The
drones only share what the flight code relays between them:
- side speaker (the side's lower id) -> the other side's drones: point A, and (top side, once
  met) its route (CROSS_PAIR_POINT_A / CROSS_PAIR_CHAIN);
- every drone -> the other side's drones: the photo coverage and new mines it produced
  (CROSS_PAIR_COVERAGE / CROSS_PAIR_MINE_RELAY);
- every drone -> its side-mate: each photo's footprint and mines found (SHARE_PHOTOS);
- every drone -> the side-mate and the other side: the job it is working on (SIDE_JOB); the
  other side's drones are also heard from by position, to keep clear of them.
Relays can be dropped or delayed to check the mission still completes. The sim never loses
a drone (the flight harness does).

Sides by drone count match FlightSettings:
  1: drone 1 alone (bottom), target = far edge
  2: drones 1 + 2 (bottom), target = far edge
  3: drones 1 + 2 (bottom), drone 3 alone (top); the two sides target each other
  4: drones 1 + 2 (bottom), drones 3 + 4 (top); the two sides target each other
Drone 1 reports: in a two-sided mission, its route joined with its partner's.

Pathnode-only parts (a backend without `.planner` loses them): `Mission.planners` and the
record_frames snapshots, which read the PathNodePlanner's chain, claims and shortcuts. The
run, summary(), evaluate() and report_route() work on the contract alone; summary() adds
backend.stats() (empty by default).
"""

import math
import random

import shapely
from shapely.ops import unary_union

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.pathnode.geometry import CELL, FIELD_H, FIELD_W, mine_zone, unsafe_cells
from flight.pathfinding.pathnode.config import GROUPS, SHORTCUT_RATIO
from flight.pathfinding.pathnode.polyline import poly_len
from flight.pathfinding.search_backend import make_backend


class _Bus:
    """Relays between drones, with optional loss and delay (in ticks).

    Only the streaming relays -- point A and coverage -- are ever dropped. Mine relays and
    the member's photo reports are acknowledged and retried in the flight code, and a
    dropped mine is not safe to lose: the side that never hears of it routes through it.
    They are delayed like everything else but always delivered."""

    def __init__(self, drop, delay, rng):
        self.drop, self.delay, self.rng = drop, delay, rng
        self.queue = []
        self.sent = self.dropped = 0

    def send(self, to, msg, now):
        self.sent += 1
        lossy = msg[0] in ("point_a", "coverage")
        if lossy and self.drop and self.rng.random() < self.drop:
            self.dropped += 1
            return
        self.queue.append((now + self.delay, to, msg))

    def due(self, now):
        ready = [(to, m) for t, to, m in self.queue if t <= now]
        self.queue = [(t, to, m) for t, to, m in self.queue if t > now]
        return ready


class Mission:
    def __init__(self, true_mines, drones=4, field_w=FIELD_W, field_h=FIELD_H,
                 entry_x=FIELD_W / 2, coverage="corridor", discovery="square",
                 shape_size_ft=(6.0, 4.0), overlap=0.1, drop=0.0, delay=0, seed=0,
                 record_frames=False, max_ticks=4000, adjacent_ring=1, outer_ring=2,
                 shortcut_ratio=SHORTCUT_RATIO, planner_options=None, photo_yaw_deg=0.0, backend="pathnode"):
        """photo_yaw_deg: rotate every photo's footprint by this many degrees, or "random" for
        a random heading per photo -- the real footprint turns with the drone's heading."""
        if drones not in (1, 2, 3, 4):
            raise ValueError("drones must be 1-4")
        if discovery not in ("square", "point"):
            raise ValueError(f"discovery must be 'square' or 'point', got {discovery!r}")
        self.true_mines = list(true_mines)
        self.field_w, self.field_h = field_w, field_h
        self.discovery = discovery
        self.shape = shape_size_ft
        self.max_ticks = max_ticks
        two_sided = drones >= 3
        # planner_options: any other backend settings (pathnode: PlannerConfig fields, e.g.
        # shortcut_max_waypoints).
        kw = dict(shape_size_ft=shape_size_ft, overlap=overlap, coverage=coverage,
                  adjacent_ring=adjacent_ring, outer_ring=outer_ring,
                  shortcut_ratio=shortcut_ratio,
                  **(planner_options or {}))
        # The sides, each keyed by its speaker (lowest id -- the sim never loses a drone): the
        # drone that sends the side's point A and route to the other side.
        self.sides = {1: (1, 2) if drones >= 2 else (1,)}
        if two_sided:
            self.sides[3] = (3, 4) if drones == 4 else (3,)
        self.side_of = {d: sp for sp, side in self.sides.items() for d in side}
        # Every drone keeps its own copy of its side's backend (search_backend.py).
        self.backends = {}
        for sp, side in self.sides.items():
            edge = "bottom" if sp == 1 else "top"
            for d in side:
                self.backends[d] = make_backend(backend, field_w, field_h, entry_x, edge, side,
                                                two_sided, **kw)
        self.partner = {1: 3, 3: 1} if two_sided else {}
        # receiver -> sender -> the job the sender last announced (ClaimBook held jobs)
        self.known_jobs = {d: {} for d in self.backends}
        start = {1: (entry_x, 0.0), 3: (entry_x, field_h)}
        self.pos = {d: start[sp] for d, sp in self.side_of.items()}
        self.photos = {d: [] for d in self.side_of}
        self.group_photos = {d: [0] * len(GROUPS) for d in self.side_of}  # photos per group
        cols, rows = math.ceil(field_w / CELL - 1e-9), math.ceil(field_h / CELL - 1e-9)
        self.seen_all = CellField(cols, rows, max_corner=(cols * CELL, rows * CELL))
        self.discovered = {}
        self.bus = _Bus(drop, delay, random.Random(seed))
        self.photo_yaw_deg = photo_yaw_deg
        self._yaw_rng = random.Random(seed + 1)
        self.tick = 0
        self.failure = None
        self.record_frames = record_frames
        self.frames = []

    @property
    def planners(self):
        """drone -> its PathNodePlanner. Pathnode backend only (reads backend.planner)."""
        return {d: b.planner for d, b in self.backends.items()}

    # -- relays ------------------------------------------------------------------------
    def _mates(self, drone):
        return [d for d in self.sides[self.side_of[drone]] if d != drone]

    def _partner_drones(self, drone):
        sp = self.side_of[drone]
        return list(self.sides[self.partner[sp]]) if sp in self.partner else []

    def _deliver(self):
        for to, msg in self.bus.due(self.tick):
            side = self.backends[to]
            kind = msg[0]
            if kind == "point_a":                       # CROSS_PAIR_POINT_A
                side.partner_point_a(*msg[1])
            elif kind == "coverage":                    # CROSS_PAIR_COVERAGE
                side.partner_coverage(msg[1])
            elif kind == "route":                       # CROSS_PAIR_CHAIN
                side.partner_route(msg[1])
            elif kind == "mine":                        # CROSS_PAIR_MINE_RELAY
                side.partner_mine(*msg[1])
            elif kind == "photo":                       # SHARE_PHOTOS from a side-mate
                side.mate_photo(msg[1], msg[2])
            elif kind == "job":                         # SIDE_JOB
                self.known_jobs[to][msg[1]] = msg[2]

    def _send_outbox(self, drone, out):
        """Relay this drone's Outbox to every drone of the partner side, in the flight code's
        order. Only the speaker streams point A."""
        for to in self._partner_drones(drone):
            for m in out.mines:
                self.bus.send(to, ("mine", m), self.tick)
            if out.coverage is not None:
                self.bus.send(to, ("coverage", out.coverage), self.tick)
            if out.point_a is not None and drone == self.side_of[drone]:
                self.bus.send(to, ("point_a", out.point_a), self.tick)

    def _relay_point_a(self, speaker):
        for to in self._partner_drones(speaker):
            self.bus.send(to, ("point_a", self.backends[speaker].point_a()), self.tick)

    # -- photos --------------------------------------------------------------------------
    def _found(self, x, y, corners=None):
        along, across = self.shape
        hx, hy = across / 2, along / 2
        if self.discovery == "square" and corners is not None:
            # A mine is found when its whole square is inside the (possibly rotated) photo.
            photo = shapely.Polygon(corners).buffer(1e-9)
            hit = lambda m: photo.contains(shapely.box(  # noqa: E731
                math.floor(m[0] / CELL) * CELL, math.floor(m[1] / CELL) * CELL,
                (math.floor(m[0] / CELL) + 1) * CELL, (math.floor(m[1] / CELL) + 1) * CELL))
        elif self.discovery == "square":
            covered = self._footprint_cells(x, y)
            hit = lambda m: (math.floor(m[0] / CELL), math.floor(m[1] / CELL)) in covered  # noqa: E731
        else:
            hit = lambda m: abs(m[0] - x) <= hx and abs(m[1] - y) <= hy  # noqa: E731
        out = [m for m in self.true_mines if m not in self.discovered and hit(m)]
        for m in out:
            self.discovered[m] = self.tick
        return out

    def _footprint_cells(self, x, y):
        """Squares fully inside the axis-aligned photo centred at (x, y)."""
        along, across = self.shape
        x0, x1, y0, y1 = x - across / 2, x + across / 2, y - along / 2, y + along / 2
        eps = 1e-9
        cols = range(math.ceil((x0 - eps) / CELL), math.floor((x1 + eps) / CELL))
        rows = range(math.ceil((y0 - eps) / CELL), math.floor((y1 + eps) / CELL))
        return {(c, r) for c in cols for r in rows}

    def _photo(self, drone, x, y):
        self.photos[drone].append((x, y))
        self.pos[drone] = (x, y)
        along, across = self.shape
        hx, hy = across / 2, along / 2
        corners = [(x - hx, y - hy), (x + hx, y - hy), (x + hx, y + hy), (x - hx, y + hy)]
        yaw = self.photo_yaw_deg
        if yaw == "random":
            yaw = self._yaw_rng.uniform(0.0, 180.0)
        if yaw:
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            corners = [(x + (px - x) * c - (py - y) * s, y + (px - x) * s + (py - y) * c)
                       for px, py in corners]
            self.seen_all.fill_polygon_covered(corners)
            return corners, self._found(x, y, corners)
        self.seen_all.fill_polygon_covered(corners)
        return corners, self._found(x, y)

    def _step(self, drone):
        side = self.backends[drone]
        if side.failure:
            return False
        positions = {d: self.pos[d] for d in side.drones}
        # The other side's drones are always heard from here (POSITION_REPORT in flight).
        others = {d: self.pos[d] for d in self._partner_drones(drone)}
        queue = side.assignments(positions, self.known_jobs[drone], drone, others)[drone]
        for to in self._mates(drone) + self._partner_drones(drone):
            self.bus.send(to, ("job", drone, side.current_job(drone)), self.tick)
        if not queue:
            return False
        x, y, group = queue[0]
        self.group_photos[drone][group] += 1
        corners, mines = self._photo(drone, x, y)
        side.own_footprint(corners)
        out = side.own_photo(corners, mines)
        for mate in self._mates(drone):
            self.bus.send(mate, ("photo", corners, mines), self.tick)
        self._send_outbox(drone, out)
        return True

    # -- run -------------------------------------------------------------------------------
    def done(self):
        return (all(b.done() for b in self.backends.values())
                and (3 not in self.backends or self.report_route() is not None))

    def run(self):
        for sp in self.sides:
            self._relay_point_a(sp)
        if self.record_frames:
            self.frames.append(self._snapshot())
        idle = 0
        while self.tick < self.max_ticks:
            self.tick += 1
            self._deliver()
            if self.done() and not self._pending_relays():
                break
            acted = False
            for drone in self.side_of:
                acted = self._step(drone) or acted
                self._deliver()
            # Point A is a lossy stream, so it is re-sent every pass rather than only when
            # it changes: with send-on-change, one dropped update leaves the partner stale
            # for good once this side stops moving.
            for sp in self.sides:
                self._relay_point_a(sp)
            # The top side's speaker streams its route once met (CROSS_PAIR_CHAIN in flight);
            # the bottom side checks the two join.
            if 3 in self.sides and self.backends[3].met:
                for to in self.sides[1]:
                    self.bus.send(to, ("route", self.backends[3].route()), self.tick)
            if self.record_frames:
                self.frames.append(self._snapshot())
            failed = [g for g, b in self.backends.items() if b.failure]
            if failed:
                self.failure = f"drone {failed[0]}: {self.backends[failed[0]].failure}"
                break
            # A side can legitimately idle for a few ticks waiting on a delayed point-A
            # update, so a stall is a sustained idle streak, not one quiet tick.
            idle = 0 if (acted or self._pending_relays() or self.done()) else idle + 1
            if idle > self.bus.delay + 5:
                self.failure = "stalled"
                break
        if not self.failure and self.tick >= self.max_ticks:
            self.failure = "hit tick cap"
        return self

    def _pending_relays(self):
        """Anything other than the every-pass point-A heartbeat still in flight."""
        return any(m[0] not in ("point_a", "route", "job") for _t, _to, m in self.bus.queue)

    def report_route(self):
        """What drone 1 reports: its route, joined with its partner's when two-sided."""
        if 3 not in self.sides:
            return self.backends[1].route()
        return self.backends[1].join(self.backends[3].route())

    def _snapshot(self):
        """One frame for render.py. Pathnode backend only (reads backend.planner)."""
        zones = []
        if self.discovered:
            u = unary_union([mine_zone(*m) for m in self.discovered])
            for part in (u.geoms if hasattr(u, "geoms") else [u]):
                zones.append(list(part.exterior.coords))
        return {"tick": self.tick, "seen": self.seen_all.copy(), "zones": zones,
                "sides": {g: {"links": [(k.kind, list(k.poly)) for k in p.links],
                              "nodes": [(n.x, n.y, n.placed_by) for n in p.nodes],
                              "claims": [(x, y, grp, d) for d in p.drones
                                         for x, y, grp in self.planners[d].last_plan.get(d, [])],
                              "shortcuts": [list(t.poly) for t in p.shortcuts.trials.values()
                                            if t is not None]}
                          for g, p in ((sp, self.planners[sp]) for sp in self.sides)},
                "photos": {d: list(v) for d, v in self.photos.items()},
                "side_of": dict(self.side_of), "discovered": set(self.discovered)}

    # -- results -------------------------------------------------------------------------
    def summary(self):
        dist = {d: poly_len(p) if len(p) > 1 else 0.0 for d, p in self.photos.items()}
        stats = {}
        for b in (self.backends[sp] for sp in self.sides):     # one copy per side
            for k, v in b.stats().items():
                stats[k] = stats.get(k, 0) + v
        return {"makespan": max(dist.values()), "photos": sum(len(p) for p in self.photos.values()),
                "ticks": self.tick, "failure": self.failure, "done": self.done(),
                "per_drone": {d: (len(self.photos[d]), round(dist[d])) for d in self.side_of},
                "group_photos": {d: list(v) for d, v in self.group_photos.items()},
                "sent": self.bus.sent, "dropped": self.bus.dropped, **stats}


def evaluate(mission):
    """Independent checks on drone 1's reported route."""
    route = mission.report_route()
    if not route:
        return {"complete": False, "joined": False}
    start_ok = abs(route[0][1]) < 1e-6
    end_ok = abs(route[-1][1] - mission.field_h) < 1e-6
    segs = [shapely.LineString([route[k], route[k + 1]]) for k in range(len(route) - 1)
            if math.dist(route[k], route[k + 1]) > 1e-9]

    def crossings(mines):
        if not mines:
            return 0
        zone = unary_union([mine_zone(*m) for m in mines]).buffer(-1e-6)
        return int(sum(shapely.intersects(s, zone) for s in segs))

    scratch = mission.seen_all.copy()
    scratch.clear_all()
    scratch.mark_path(route)
    cells = set(scratch.on_cells())
    unseen = sum(1 for c in cells if not mission.seen_all.get(*c))
    unsafe = set()
    for m in mission.true_mines:
        unsafe.update(unsafe_cells(*m))
    hidden = [m for m in mission.true_mines if m not in mission.discovered]
    # The safety line: the route may pass near a mine nobody found, never over its square.
    on_mine = sum(1 for m in mission.true_mines
                  if (math.floor(m[0] / CELL), math.floor(m[1] / CELL)) in cells)
    return {"complete": mission.done() and mission.failure is None, "joined": True,
            "route_mine_squares": on_mine,
            "spans_field": start_ok and end_ok,
            "bad_edges_known": crossings(list(mission.discovered)),
            "bad_edges_hidden": crossings(hidden), "unseen_cells": unseen,
            "route_cells_in_danger": len(cells & unsafe)}
