"""Every name and tunable the path-node planner uses: enums for its string-like values, and
one frozen PlannerConfig for the per-mission settings.

The enums subclass str (or int) so they compare equal to the plain values they replace --
"checked", "big", group 2 -- which is what the renderer and the tests print."""

from dataclasses import dataclass
from enum import Enum, IntEnum

from flight.pathfinding.pathnode.geometry import CELL, unsafe_cells


class LinkKind(str, Enum):
    CHECKED = "checked"     # photographed ground (segment C)
    PENDING = "pending"     # a planned, unchecked detour around a mine (segment B)
    FORWARD = "forward"     # point A -> target: the gap still to explore (at most one)


class Placer(str, Enum):
    """Who laid a path node."""
    ENTRY = "entry"
    FORWARD = "forward"     # laid over the forward gap
    SPAN = "span"           # laid over a pending span
    JUMP = "jump"           # a new point A past a mine this pair found
    SHORTCUT = "shortcut"   # spliced in by a shortcut
    TARGET = "target"       # the virtual end node


class Source(str, Enum):
    """Who found a mine, as far as the mine rule cares."""
    PAIR = "pair"           # this side's own drones: may jump point A past it
    PARTNER = "partner"     # relayed by the other side: only re-bridges


class Edge(str, Enum):
    BOTTOM = "bottom"       # y = 0
    TOP = "top"             # y = field_h


class CoverageMode(str, Enum):
    CORRIDOR = "corridor"   # a link is checked once the squares it touches are photographed
    BAND = "band"           # ...and every square a mine could endanger it from


class Tier(str, Enum):
    BIG = "big"
    ORDINARY = "ordinary"


class Group(IntEnum):
    """Priority groups of work, highest first (see work.py)."""
    PATH = 0            # one valid path: the forward gap and every pending span
    BIG_SHORTCUT = 1    # the big shortcut being tried
    ADJACENT = 2        # squares within adjacent_ring of the checked route
    SHORTCUT = 3        # the ordinary shortcut being tried
    OUTER = 4           # the next layer out, within outer_ring


GROUPS = tuple(g.name.lower() for g in Group)          # ("path", "big_shortcut", ...)
TIER_GROUP = {Tier.BIG: Group.BIG_SHORTCUT, Tier.ORDINARY: Group.SHORTCUT}

# -- fixed tunables -----------------------------------------------------------------------
SAMPLE_FT = 0.5             # resolution for deciding how far along a link is checked
TILE_MARGIN_FT = 0.5        # a tiled photo keeps its target square this far inside it
MISS_CENTRE = 1             # misses before a square is photographed centred on itself
MISS_BLOCK = 3              # misses before a square is blocked (see coverage.py)
FOOTPRINT_SLACK_FT = 1e-3   # grows recorded footprints past lat/lon round-trip noise
ROUTE_MATCH_FT = 0.01       # two route points closer than this are the same point
MAX_BRIDGE_FT = 30.0        # longest straight bridge a join may add (join.py)
SHORTCUT_MIN_SAVING_FT = 2 * CELL   # an ordinary shortcut saves at least this much
CACHE_CAP = 50_000          # entries before a geometric cache is cleared
EVENT_LOG_LEN = 500         # planner events kept (a debugging aid, bounded for flight)
# Squares from which a mine could make square (0, 0) unsafe (band coverage).
DANGER_OFFSETS = unsafe_cells(CELL / 2, CELL / 2)


@dataclass(frozen=True)
class PlannerConfig:
    """
    Per-mission planner settings.
    - shape_size_ft: planning photo footprint (along = y, across = x).
    - overlap: overlap between consecutive photos along a path.
    - coverage: what must be photographed for a link to count as checked (CoverageMode).
    - adjacent_ring: how many squares out from the checked route the ADJACENT group reaches
      (0 = no such group).
    - outer_ring: how far out the OUTER group reaches, beyond adjacent_ring (at most
      adjacent_ring = no such group).
    - shortcut_ratio: try ordinary shortcuts shorter than this fraction of the section they
      would replace (None = no shortcuts at all).
    - big_shortcut_ratio, big_shortcut_saving_ft: what counts as a big shortcut (None = no
      big tier).
    - shortcut_max_waypoints, shortcut_ft_per_photo: a shortcut's photo allowance -- the
      larger of shortcut_max_waypoints and one photo per shortcut_ft_per_photo it saves.
    - keep_away_ft: a drone never flies a photo this close to a lower-id drone's position or
      next photo (0 = off). The claims keep the side-mates on different work, not physically
      apart; the separation check aborts a flight at 1.2 m. Applies to the other side's
      drones too (planner.plan).
    - keep_clear_ft: a drone never flies a photo this close to a higher-id drone's position
      -- and the most it keeps from any drone it is not yielding to. Must stay above the
      flight separation floor (drone.SEPARATION_ABORT_M = 1.2 m = 3.94 ft) and the radius
      the flight code holds a leg at (waypoint.COLLISION_RADIUS = 2.01 m = 6.6 ft): a photo
      nearer than that to a peer is held until the peer moves, and aborts after 15 s.
      keep_away_ft stays above it, so a tie has a winner.
    """
    shape_size_ft: tuple = (6.0, 4.0)
    overlap: float = 0.1
    coverage: CoverageMode = CoverageMode.CORRIDOR
    adjacent_ring: int = 1
    outer_ring: int = 2
    shortcut_ratio: float | None = 0.9
    big_shortcut_ratio: float | None = 0.8
    big_shortcut_saving_ft: float = 20.0
    shortcut_max_waypoints: int = 8
    shortcut_ft_per_photo: float = 2.0
    keep_away_ft: float = 9.0
    keep_clear_ft: float = 7.0

    def __post_init__(self):
        object.__setattr__(self, "coverage", CoverageMode(self.coverage))
        object.__setattr__(self, "shape_size_ft", tuple(float(v) for v in self.shape_size_ft))

    def tier_ratio(self, tier):
        return self.big_shortcut_ratio if tier == Tier.BIG else self.shortcut_ratio

    def tier_min_saving(self, tier):
        return self.big_shortcut_saving_ft if tier == Tier.BIG else SHORTCUT_MIN_SAVING_FT


SHORTCUT_RATIO = PlannerConfig.shortcut_ratio
