"""PathEngine: what a shortest-path implementation must provide for the path-node planner.

To add an engine: subclass PathEngine, implement every abstract method below, register it in
engines/__init__.py (ENGINES), run `python -m flight.pathfinding.pathnode.test_path_engine`.
The planner core (routes.py and everything above it) talks to an engine ONLY through this
interface; nothing else of an engine is visible to it.

Units and grid:
- All coordinates are field feet, origin bottom-left, x across (0..field_w), y along (0..field_h).
- Grid squares are CELL (2 ft) wide; a square is (col, row), col = x // CELL, row = y // CELL.
- A point is (x, y); a polyline is a list of points.
- An obstacle is a mine's no-go zone (the squares its safety circle touches) or a blocked
  square. Touching / overlapping obstacles merge into one group with one id (gid).
- "Inside" and "crosses" are tested against obstacles grown by a small margin, so a path may
  hug an obstacle corner but never cut through it or squeeze between two that touch.

Methods the planner calls:
- __init__(field_w, field_h)
    Takes: field size in ft. Does: starts with no obstacles.
- add_mine(x, y) -> gid
    Takes: the mine position (it is snapped to its square's centre). Does: adds its no-go zone.
    Returns: the id of the group it now belongs to (a NEW id when it merged groups).
- add_square(c, r) -> gid
    Does: blocks grid square (c, r) (a square that could not be photographed). Returns: gid, as above.
- resolve(gid) -> gid
    Returns: the group an older id belongs to now (itself when it never merged). Total over
    every id this engine ever returned.
- contains(x, y) -> bool
    True when the point is inside an obstacle (not a valid place for a path node).
- crosses(poly) -> bool
    True when the polyline enters an obstacle. False for fewer than 2 points / no obstacles.
- clear_line(p, q) -> bool
    Default: not crosses([p, q]). Override only for speed.
- shortest_path(p, q) -> (points, gids) | None
    Returns: the shortest obstacle-free polyline. points[0] == p and points[-1] == q EXACTLY
    (as floats); the points between are obstacle corners the path bends around.
    gids[i] is the group points[i] bends around (as returned by add_*; the caller resolves it
    with resolve()), None at both ends; len(gids) == len(points).
    A clear straight line gives ([p, q], [None, None]).
    None when there is no route or p or q is inside an obstacle.
- corners() -> [(x, y)]
    Every obstacle corner paths can bend around (used as candidate anchors by target.py and
    shortcuts.py). Never inside an obstacle.
- last_added (property) -> shapely geometry
    What the most recent add_mine / add_square made off limits (the new obstacle grown by the
    margin). Routes drops only the cached routes that intersect it. None before any add.
    After set_level: the area that became off limits (new minus old; empty when zones shrank).
    Routes does NOT use it for a level change: it clears every cache (simplest correct option,
    since a shrink also frees space and group ids change).
- set_level(k) -> None, level (property) -> int
    Growth. Level k = zones as they are after k growth steps of STEP_FT (2 ft) each:
    - a mine's zone is its ORIGINAL shape grown k steps in a DIAMOND (city-block) way: it
      extends 2k ft further along each axis, and no further than that on a diagonal
    - blocked squares (add_square) never grow
    - touching / overlapping grown zones merge into one group (exact union outline)
    - absolute, not cumulative: set_level(k) is a function of (mines, squares, k) alone, so
      set_level(k) then set_level(j<k) equals a fresh engine set to j (rollback); any k >= 0
      is allowed (no cap); set_level(level) changes nothing
    - mines and squares added later are built at the current level
    - Group ids: merges / splits change ids. resolve(old_gid) after a level change is the live
      group holding the first mine or square the old group held (total over every id ever
      returned), so a stale gid is always safe to resolve.
- obstacle_shapes() -> {gid: shapely geometry}
    Every live (unmerged) group's outline, for drawing and tests.

Invariants the planner's caches depend on:
- Monotone: adding an obstacle, or raising the level, can only lengthen or block a shortest
  path, never shorten one (lowering the level can only shorten).
  So a cached route the new obstacle does not cross is still the shortest, and a cached "no
  route" stays no route.
- Deterministic: the same sequence of adds / set_levels gives the same answers (two drones each hold a
  copy and must agree). No randomness, no dependence on call history of the queries.
- Queries never change the obstacles; they may be asked in any order, any number of times.
"""

from abc import ABC, abstractmethod

STEP_FT = 2.0  # one growth step (one grid square); level k grows every mine's zone k * STEP_FT


class PathEngine(ABC):
    """Obstacles plus shortest paths around them (see the module docstring for the contract)."""

    @abstractmethod
    def __init__(self, field_w, field_h):
        """Takes: field width and height in ft. Does: starts with no obstacles."""

    @abstractmethod
    def add_mine(self, x, y):
        """Add a mine's no-go zone. Returns the id of the group it belongs to."""

    @abstractmethod
    def add_square(self, c, r):
        """Block grid square (c, r). Returns the id of the group it belongs to."""

    @abstractmethod
    def resolve(self, gid):
        """The group an older gid belongs to now."""

    @abstractmethod
    def contains(self, x, y):
        """Whether the point is inside an obstacle."""

    @abstractmethod
    def crosses(self, poly):
        """Whether the polyline enters an obstacle."""

    def clear_line(self, p, q):
        """Whether the straight segment p-q stays out of every obstacle."""
        return not self.crosses([p, q])

    @abstractmethod
    def shortest_path(self, p, q):
        """Shortest obstacle-free (points, gids) from p to q, or None."""

    @abstractmethod
    def corners(self):
        """Every obstacle corner paths can bend around, as (x, y)."""

    @property
    @abstractmethod
    def last_added(self):
        """Shapely geometry of what the most recent add made off limits (None before any)."""

    @abstractmethod
    def set_level(self, k):
        """Rebuild the zones at growth level k (absolute; see the module docstring)."""

    @property
    @abstractmethod
    def level(self):
        """The current growth level (0 at the start)."""

    @abstractmethod
    def obstacle_shapes(self):
        """{gid: shapely geometry} of every live group."""
