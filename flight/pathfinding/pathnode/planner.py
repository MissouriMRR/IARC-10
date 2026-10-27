"""One side's path-node planner: the core shared by the flight code and the simulator.

A side is one or two drones working from one edge. Every drone keeps its own copy of the
side's planner and plans for every drone of the side; which drone flies what is decided by
claims (claims.py) plus the jobs the drones announce, not by role. Everything is in local
feet; the flight code wraps it in lat/lon (flight/pathfinding/mission_planner.py).

What it is made of (one module each; PathNodePlanner is the side's mission state and wires
the rest):
- config.py        names, enums (Edge, LinkKind, ...) and PlannerConfig
- geometry.py      field constants and cell helpers
- polyline.py      helpers on polylines
- log.py           counters and the bounded event log
- engines/         PathEngine (base.py, the shortest-path seam), ZoneMap (zonemap.py);
                   ENGINES and make_engine in __init__.py
- routes.py        known obstacles (mine zones, blocked squares) over an engine; shortest paths
- target.py        Target (seam): what the forward gap aims at -- FarEdgeTarget (1-2 drones) or
                   PartnerTarget, the partner side's point A (3-4 drones)
- chain.py         the route so far: path nodes joined by checked / pending / forward links
- coverage.py      photographed squares, misses; asks its placement for the rest
- placement.py     PhotoPlacement (seam): what a stretch needs and where its photos go, one
                   subclass per CoverageMode
- mine_rule.py     MineRule: repairing the chain when an obstacle appears (apply is the entry)
- shortcuts.py     replacing checked sections with shorter paths
- work.py          the photos still to take, in priority groups
- work_sources.py  WorkSource (seam): one producer of work per group
- claims.py        ClaimBook: which drone takes which work
- coordination.py  SideCoordinator: claims, takeover, keep-apart and make-way -> one queue per
                   drone
- join.py          joining the two sides' routes
- level_control.py LevelController (planner.levels): the expanding-mines sequence -- ready,
                   snapshots, growth by level, rollback (pure parts: pathfinding/levels.py)
- session.py       PathNodeBackend: the SearchBackend (search_backend.py) over this planner

The seams, what a new subclass must implement, and how-to recipes: docs/architecture.md.

Order of events for each photo (session.py does this for both the simulator and the flight
code): record_footprint, then add_mine for its mines, then advance() -- path nodes are laid
only after the photo's mines are in.
"""

import dataclasses
import hashlib

from flight.pathfinding.pathnode import join as join_mod
from flight.pathfinding.pathnode import mine_rule
from flight.pathfinding.pathnode.chain import Chain, Link, PathNode
from flight.pathfinding.pathnode.config import (GROUPS, Edge, Group, LinkKind, Placer,
                                                PlannerConfig, Source)
from flight.pathfinding.pathnode.coordination import SideCoordinator
from flight.pathfinding.pathnode.coverage import Coverage
from flight.pathfinding.pathnode.engines import make_engine
from flight.pathfinding.pathnode.geometry import snap_to_cell_center
from flight.pathfinding.pathnode.level_control import LevelController
from flight.pathfinding.pathnode.log import PlannerLog
from flight.pathfinding.pathnode.routes import Routes
from flight.pathfinding.pathnode.shortcuts import Shortcuts
from flight.pathfinding.pathnode.target import make_target
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
        self.discovered = []            # every known mine, (x, y)
        self.blocked = set()            # squares given up on (see coverage.py)
        self.partner_route = []         # the partner's latest route (check_join)
        self.failure = None
        self.log = PlannerLog()
        self.routes = Routes(make_engine(self.config.engine, self.field_w, self.field_h))
        self.target = make_target(self.field_w, self.field_h, self.start_edge, partner_entry_x,
                                  self.routes)
        self.coverage = Coverage(self.field_w, self.field_h, self.config)
        entry = PathNode(float(entry_x), self.start_edge.y(self.field_h), Placer.ENTRY)
        self.chain = Chain([entry], [])
        route, _gids = self.target.route_from(entry.xy)
        if route is None:
            self.failure = "no route from the entry to the target"
        else:
            self.chain = Chain([entry, PathNode(*route[-1], Placer.TARGET)],
                               [Link(LinkKind.FORWARD, route)])
        self.shortcuts = Shortcuts(self.chain, self.routes, self.coverage, self.config, self.log)
        self.work = Work(self.chain, self.routes, self.coverage, self.shortcuts, self.config)
        self.coordinator = SideCoordinator(drones, self.config, self.work, self.coverage,
                                           self.field_w, self.field_h)
        self.levels = LevelController(self)

    def use_chain(self, chain):
        """Replace the chain's contents (a hand-built Chain, e.g. Chain.build) -- for tests and
        hand-built cases. Every component keeps its reference to the same Chain object."""
        chain.check()
        self.chain.nodes, self.chain.links = chain.nodes, chain.links

    def reconfigure(self, **changes):
        """Change PlannerConfig fields mid-mission (tests, tuning)."""
        self.config = dataclasses.replace(self.config, **changes)
        for part in (self.coverage, self.shortcuts, self.work, self.coordinator):
            part.config = self.config

    # -- state -----------------------------------------------------------------------------
    @property
    def nodes(self):
        return self.chain.nodes

    @property
    def links(self):
        return self.chain.links

    @property
    def drones(self):
        return self.coordinator.drones

    @property
    def claims(self):
        return self.coordinator.claims

    @property
    def last_plan(self):
        return self.coordinator.last_plan

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

    @property
    def level(self):
        """The growth level the zones are at (0 = the original safety radius)."""
        return self.routes.level

    def mine_hash(self):
        """Order-independent hash of the known mines (each snapped to its square)."""
        cells = sorted(snap_to_cell_center(x, y) for x, y in self.discovered)
        return hashlib.sha256(repr(cells).encode("utf-8")).hexdigest()

    def level_hash(self):
        """mine_hash() and the level together: equal on two copies iff same mines, same level."""
        return hashlib.sha256(repr((self.level, self.mine_hash())).encode("utf-8")).hexdigest()

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
        self.levels.after_obstacle()

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
        self.levels.after_obstacle()
        return True

    # -- the partner side ------------------------------------------------------------------
    def set_partner_point_a(self, x, y):
        """Retarget the forward gap at the partner's latest point A. Once met, only stored."""
        target = self.target
        if not target.has_partner or self.failure:
            return
        if not target.retarget((x, y)):
            return      # the heartbeat resends an unchanged point A; new mines replan on their own
        f = self.chain.forward_index()
        if f is None:
            return
        route, _ = target.route_from(self.chain.nodes[f].xy)
        if route is None:
            self.log.event("stale-target", (float(x), float(y)))
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
        if joined is None and self.start_edge.yields_in_crosswise_join:
            self._reopen_toward(self.partner_route[-1])
        return joined

    def _reopen_toward(self, point):
        """Turn the met target into an ordinary node and open a forward gap to `point`."""
        point = (float(point[0]), float(point[1]))
        self.target.retarget(point)
        if self._open_gap():
            self.log.event("reopen", self.chain.nodes[-2].xy, point)

    def _open_gap(self):
        """Turn the met target into an ordinary node and open a forward gap to the (new) target.
        Returns whether a route was found."""
        end = self.chain.nodes[-1]
        route, _gids = self.target.route_from(end.xy)
        if route is None:
            self.log.event("reopen-failed", end.xy)
            return False
        self.chain.nodes[-1] = PathNode(end.x, end.y, Placer.FORWARD)
        self.chain.nodes.append(PathNode(*route[-1], Placer.TARGET))
        self.chain.links.append(Link(LinkKind.FORWARD, route))
        self.log.count("reopens")
        self.advance()
        return True

    def partner_lost(self):
        """The partner side is gone for good: aim at its last point A, then carry on to the far
        edge, so this side's own photos check the lost side's half too."""
        self.target = self.target.lost()
        self.levels.partner_lost()
        self._follow_target()

    def _follow_target(self):
        """Once the chain has reached its target, move on to the Target it hands over to
        (target.after_met; most targets hand over to themselves) and open a gap toward it."""
        nxt = self.target.after_met()
        if nxt is self.target or not self.met:
            return
        old, self.target = self.target, nxt
        if not self._open_gap():
            self.target = old

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
        if f is not None and self.target.may_close():
            chain.advance_link(chain.links[f], Placer.FORWARD, seen)
        for link in chain.pending_links():
            chain.advance_link(link, Placer.SPAN, seen)
        self.shortcuts.update(self.met)
        self._follow_target()

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
        - me: the drone this copy of the planner belongs to, if any (coordination.py: it takes
          over work the others hold in their queues when this copy gives it nothing to fly).
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

        Does: advance(), then hands the work to the SideCoordinator (coordination.py): claims,
        keep-apart.

        Returns: drone id -> [(x, y, group), ...] in flying order. Also kept as last_plan;
        each drone's current job is claims.heads[drone].
        """
        if self.failure:
            return self.coordinator.clear()
        self.advance()      # anything already checked must close before we plan more photos
        return self.coordinator.plan(self.groups(), positions, held, me, others, lost)
