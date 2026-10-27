"""SearchBackend: the one contract between the flight code / simulator and a route search.

A backend is ONE drone's copy of ONE side's search, in local feet (frame.py: origin at the
bottom-left corner, x across, y from the bottom edge towards the top). It takes events in
(photos, mines, partner messages) and hands out work and results. Everything above it
(mission_planner.py in flight, pathnode/sim.py in the simulator) talks only to this parent;
neither knows how routes are found.

Layers:
- MissionPlanner (mission_planner.py): lat/lon <-> feet, then calls a backend.
- SearchBackend (this file): the contract + `BACKENDS` + `make_backend`.
- PathNodeBackend (pathnode/session.py): the path-node planner, the default.

Rules every backend follows (the conformance suite test_search_backend.py checks them):
- Deterministic: two copies given the same events in the same order agree on `mine_hash()`,
  `route()` and `assignments(...)`. No clocks, no randomness, no dependence on object ids.
- Mines before routes: a photo's mines are applied before anything is laid or routed over
  its footprint, so no route is ever built over ground that is about to turn out unsafe.
- Safe: `route()` never crosses a known mine; `join(...)` and `partner_route(...)` only
  return routes that cross no known obstacle.
- Idempotent events: repeating a footprint, a mine (same square) or a partner message is
  harmless.

How to add a backend: see "Add a search backend" in docs/architecture.md.
"""

import importlib
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class Outbox:
    """What to relay to the partner side after a photo (relaying is the caller's job: in
    flight, messages; in the simulator, its bus).

    - mines: mines this side just learned, (x, y), each new to this side.
    - coverage: the photo's footprint corners, or None (none to relay).
    - point_a: this side's point A, only when it has a partner.
    """
    mines: list = field(default_factory=list)
    coverage: list | None = None
    point_a: tuple | None = None


class SearchBackend(ABC):
    """One drone's copy of one side's route search, in local feet. Read the module docstring
    for the rules every backend follows; each method's docstring is its contract.

    Types: a point is (x, y); corners is a list of 4 points in order; a route is a list of
    points from this side's entry; a job is opaque to the caller (the backend's own value,
    JSON-safe, compared with ==).
    """

    # -- who ------------------------------------------------------------------------------
    @property
    @abstractmethod
    def drones(self):
        """This side's drone ids, ascending (the claim keys; ties go to the first)."""

    @property
    @abstractmethod
    def has_partner(self):
        """Whether a partner side works from the other edge."""

    # -- events: this side's photos -------------------------------------------------------
    @abstractmethod
    def own_footprint(self, corners):
        """
        Takes: the ground footprint of a photo this drone just took.

        Does: records it as covered. May repeat for one waypoint (climb retries); repeated
        footprints must count as ONE attempt. Lays nothing: that waits for own_photo.
        """

    @abstractmethod
    def own_photo(self, corners, mines):
        """
        Takes: corners, the last footprint (None if there was none); mines, the (x, y) found
        in this photo.

        Does: applies the mines, THEN lays nodes / updates routes (mines before routes).

        Returns: an Outbox. outbox.mines holds only mines new to this side (a second mine in
        an already-known square is the same mine).
        """

    @abstractmethod
    def mate_photo(self, corners, mines):
        """
        Takes: a photo report from this side's other drone (footprint + mines).

        Does: same as own_footprint + own_photo, so both drones' copies learn the same facts.

        Returns: an Outbox (for completeness; the finder already relayed its mines).
        """

    @abstractmethod
    def failed_photo(self, x, y):
        """
        Takes: a photo planned at (x, y) that produced no footprint (camera failure).

        Does: counts a miss for the ground it was aimed at; the backend may give that ground
        up as unsafe after repeated misses.
        """

    # -- events: the partner side ---------------------------------------------------------
    @abstractmethod
    def partner_mine(self, x, y):
        """A mine the partner side found. Applied like an own mine; nothing is relayed."""

    @abstractmethod
    def partner_coverage(self, corners):
        """A photo footprint the partner side took: its ground counts as covered."""

    @abstractmethod
    def partner_point_a(self, x, y):
        """The partner's latest point A: this side now heads for it. Repeats are harmless."""

    @abstractmethod
    def partner_route(self, route):
        """
        Takes: the partner's latest route (it is met).

        Does: remembers it. If the halves do not connect, the side whose edge is the bottom
        may re-open its gap toward the partner's route end (done() then goes False); the
        other side only waits, so the two never chase each other.

        Returns: the joined route, or None (not met, no connection, or the joined route
        crosses a known obstacle).
        """

    # -- work -----------------------------------------------------------------------------
    @abstractmethod
    def assignments(self, positions, held=None, me=None, others=None, lost=()):
        """
        Takes:
        - positions: drone id -> (x, y) for this side's drones; None for a drone that is gone
          (ignored entirely).
        - held: drone id -> the job it announced (current_job on its own copy), for the
          side-mates and the other side's drones.
        - me: the drone this copy belongs to (it takes others' work when it has none).
        - others: drone id -> (x, y) for the other side's drones; the plan keeps clear.
        - lost: side drones gone quiet: their work goes to the others, but their last known
          position and last announced job keep blocking.

        Returns: drone id -> [(x, y, group)] in flying order, for every drone of this side
        (a drone with nothing to do gets []). group is a small int (priority, 0 first).
        Remembered: current_job(d) reports each drone's head job afterwards.
        """

    @abstractmethod
    def current_job(self, drone):
        """The job `drone` holds per the last assignments, to announce to the side-mate and
        pass back as `held`; None if it has none. Opaque, JSON-safe, compared with ==. Its
        head always matches the first entry of that drone's assignments."""

    # -- results --------------------------------------------------------------------------
    @property
    @abstractmethod
    def failure(self):
        """Why the search stopped for good (no route exists), or None."""

    @property
    @abstractmethod
    def met(self):
        """The route reaches its target (the far edge, or the partner's point A); it may
        still have unchecked stretches. False once `failure` is set."""

    @abstractmethod
    def done(self):
        """Nothing left to do on this side: met, every stretch checked, no optional work
        (rings, shortcuts) left."""

    @abstractmethod
    def route_verified(self):
        """The route is whole and every stretch of it checked: safe to report, even with
        optional work left."""

    @abstractmethod
    def point_a(self):
        """This side's frontier (the end of its checked route); once met, the meeting
        point. (x, y)."""

    @abstractmethod
    def route(self):
        """This side's current route from its entry, [(x, y)]. Never crosses a known mine."""

    @abstractmethod
    def join(self, partner_route):
        """
        Takes: the partner side's route.

        Returns: this side's route joined with it, or None if they do not connect or the
        joined route crosses a known obstacle. Changes nothing (unlike partner_route).
        """

    @abstractmethod
    def crosses(self, route):
        """Whether `route` passes through a known obstacle (a mine's unsafe zone)."""

    @abstractmethod
    def mines(self):
        """Every known mine, [(x, y)], in the order learned (for the mine-list sync)."""

    @abstractmethod
    def mine_hash(self):
        """A hash of the known mines AND the growth level, equal on two copies iff they know the
        same mines at the same level (order-independent; each mine snapped to its square).
        Compared across drones."""

    @abstractmethod
    def cell_seen_at(self, x, y):
        """Whether the ground at (x, y) is covered by a photo; None if off the field."""

    # -- expanding mines: levels, snapshots, lockstep -----------------------------------------
    # Level k = growth steps done: every mine's danger zone is 2 ft wider per level. A copy
    # that has finished all its important work at a level takes a SNAPSHOT of its complete
    # route, then grows one level and works again; both sides climb in lockstep (levels.py).
    # The caller (flight code, simulator) owns the clock and the messages; the backend owns the
    # rules. Types: LevelState, LevelStep, Snapshot are in flight/pathfinding/levels.py.
    @property
    @abstractmethod
    def level(self):
        """The growth level this copy works at (0 = the original safety radius)."""

    @abstractmethod
    def level_state(self, me):
        """
        Takes: me, the drone this copy belongs to.

        Returns: the LevelState to announce to the side-mate(s) and the other side: level,
        whether this copy holds a snapshot of its complete route at that level, whether it is
        capped. Cheap and repeatable; the announcement's seq rises only when something changed.
        """

    @abstractmethod
    def hear_level_state(self, state):
        """A LevelState announced by a side-mate or the other side. Older or repeated ones are
        ignored (by seq). Never changes the route."""

    @abstractmethod
    def try_advance_level(self, me, now=0.0, lost=(), max_level=None):
        """
        Takes:
        - me: the drone this copy belongs to.
        - now: the caller's clock, stamped on a snapshot taken now.
        - lost: side-mate ids gone quiet (they stop gating).
        - max_level: never grow to a level above it (the caller's mission limit, or "no time
          left": see growth_fits). None = no limit.

        Does: if every important job at this level is done, snapshots the route (once per
        distinct route); then, if this copy, its side-mate(s) and the other side are all ready
        at this level or already past it, grows one level and rechecks the route against the
        wider zones. A growth step with no route (or a blocked entry) is undone and caps the
        copy at its level for good. Call it once per pass, then announce level_state.

        Returns: a LevelStep (levels.Step says what happened; GREW means level went up and the
        route has new work).
        """

    @abstractmethod
    def growth_fits(self, remaining):
        """Whether `remaining` seconds of the caller's time allow another growth step: at least
        1.5 x the last measured growth + recheck time (True before the first). The caller
        applies it, e.g. by passing max_level=level when it is False."""

    @abstractmethod
    def valid_snapshots(self):
        """The snapshots taken so far with no known mine on their squares, oldest first
        (immutable; each carries level, route, cells, mine_hash, t)."""

    @abstractmethod
    def report_route(self, level=None):
        """
        The route to report: the newest VALID snapshot's (none of its squares holds a known
        mine; mines in the buffer around it are fine). With `level`, that level's newest valid
        snapshot, or None. With no valid snapshot: the live route if whole and checked, else the
        newest snapshot, else the live route -- so there is always a route to report once the
        first snapshot exists (report_valid() says whether it is a valid snapshot).
        """

    @abstractmethod
    def report_valid(self):
        """Whether report_route() is a valid snapshot."""

    @abstractmethod
    def join_at(self, level, partner_route, partner_hash=None):
        """
        Takes: the partner side's snapshot route at `level`, and optionally its mine_hash.

        Returns: this copy's snapshot route at `level` joined with it, or None (no valid
        snapshot at that level, no connection, or the join crosses a known zone). Equal hashes
        (same mines, same level) need no further check; otherwise the joined route must clear
        the zones at `level` of every mine this copy knows. Changes nothing.
        """

    @abstractmethod
    def buffer_width(self):
        """The reported route's clearance in squares: the city-block distance from its cell path
        within which there is no known mine = (smallest distance from a known mine's square to
        the nearest path square) - 1. 0 = a mine next to the path. With no known mine: the
        field's largest distance."""

    @abstractmethod
    def partner_lost(self):
        """
        The partner side is gone for good (detected by the caller). Does not stop: this side
        heads for the partner's last point A, then carries on to its own far edge, checking the
        lost side's half with its own photos; the lost side no longer gates the level climb.
        No-op without a partner.
        """

    # -- optional -------------------------------------------------------------------------
    def stats(self):
        """Counters for tools and the simulator's summary: name -> number. Default: none."""
        return {}


# name -> SearchBackend class, or "module:Class" imported on first use (a backend nobody
# selects is never loaded).
BACKENDS = {"pathnode": "flight.pathfinding.pathnode.session:PathNodeBackend"}


def backend_class(name):
    """The SearchBackend class registered as `name`."""
    if name not in BACKENDS:
        raise ValueError(f"unknown search backend {name!r}; choose from {sorted(BACKENDS)}")
    cls = BACKENDS[name]
    if isinstance(cls, str):
        module, attr = cls.split(":")
        cls = BACKENDS[name] = getattr(importlib.import_module(module), attr)
    return cls


def make_backend(name, field_w, field_h, entry_x, edge, side_drones, has_partner, **options):
    """
    Takes:
    - name: a key of BACKENDS.
    - field_w, field_h: field size in feet.
    - entry_x, edge: where this side enters, and on which edge ("bottom": y = 0; "top").
    - side_drones: this side's drone ids, ascending.
    - has_partner: whether a partner side works from the other edge.
    - options: the camera footprint (`shape_size_ft`, `overlap`) plus the backend's own
      settings; an unknown option is an error.

    Returns: a fresh backend of that kind.
    """
    return backend_class(name)(field_w, field_h, entry_x, edge, tuple(side_drones),
                               has_partner, **options)
