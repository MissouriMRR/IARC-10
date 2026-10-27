"""Known obstacles and the shortest paths around them, with the caches the planner leans on.

Obstacles are mine zones and blocked squares (coverage.py), all held in one PathEngine
(engines/base.py; ZoneMap by default). This class uses ONLY that interface. The planner asks the same geometric questions after every photo, so:
- shortest paths are cached by (from, to);
- "is this square's centre inside a zone?" is cached per square.
A new obstacle can only lengthen or block a shortest path, so a cached route it doesn't cross
is still the shortest, and a cached "no route" stays so: only the routes it crosses are
dropped (and the "inside" answers near it). `version` counts obstacles, for other caches.
"""

import numpy as np
import shapely

from flight.pathfinding.pathnode.config import CACHE_CAP
from flight.pathfinding.pathnode.geometry import CELL


class Routes:
    def __init__(self, engine):
        """Takes: a PathEngine instance (see engines.make_engine)."""
        self.engine = engine
        self.version = 0
        self._paths = {}        # ((ax, ay), (bx, by)) -> (points, gids, LineString) or None
        self._inside = {}       # square -> whether its centre is inside a zone

    # -- obstacles -------------------------------------------------------------------------
    def add_mine(self, x, y):
        """Add a mine's zone. Returns its group id."""
        return self._added(self.engine.add_mine(x, y))

    def add_square(self, c, r):
        """Add square (c, r) as an obstacle. Returns its group id."""
        return self._added(self.engine.add_square(c, r))

    def _added(self, gid):
        self.version += 1
        new = self.engine.last_added
        shapely.prepare(new)
        keys = [k for k, v in self._paths.items() if v is not None]
        if keys:
            lines = np.array([self._paths[k][2] for k in keys], dtype=object)
            for k, hit in zip(keys, shapely.intersects(lines, new)):
                if hit:
                    del self._paths[k]
        x0, y0, x1, y1 = new.bounds
        for (c, r), hit in list(self._inside.items()):
            if not hit and x0 <= (c + 0.5) * CELL <= x1 and y0 <= (r + 0.5) * CELL <= y1:
                del self._inside[(c, r)]
        return gid

    # -- queries ---------------------------------------------------------------------------
    def contains(self, x, y):
        return self.engine.contains(x, y)

    def crosses(self, poly):
        return self.engine.crosses(poly)

    def clear_line(self, p, q):
        return self.engine.clear_line(p, q)

    def shortest(self, a, b):
        """Shortest path a -> b around every obstacle, as (points, gids), or None. Returns
        copies, with gids resolved to the groups they have since merged into."""
        key = ((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
        if key not in self._paths:
            if len(self._paths) > CACHE_CAP:
                self._paths.clear()
            route = self.engine.shortest_path(*key)
            self._paths[key] = (None if route is None else
                                (route[0], route[1], shapely.LineString(route[0])))
        route = self._paths[key]
        if route is None:
            return None
        return list(route[0]), [None if g is None else self.engine.resolve(g) for g in route[1]]

    def route(self, a, b):
        """shortest(a, b), or (None, None) when there is none."""
        r = self.shortest(a, b)
        return r if r is not None else (None, None)

    def cached_pairs(self):
        return list(self._paths)

    def in_zone(self, c, r):
        """Whether square (c, r)'s centre is inside an obstacle."""
        hit = self._inside.get((c, r))
        if hit is None:
            hit = self._inside[(c, r)] = self.engine.contains((c + 0.5) * CELL, (r + 0.5) * CELL)
        return hit

    def corners(self):
        """Every obstacle corner the paths bend around, as (x, y)."""
        return self.engine.corners()
