"""What the forward gap aims at: the partner side's latest point A (3-4 drones), or the far
edge (1-2 drones)."""

import math

from flight.pathfinding.pathnode.config import Edge
from flight.pathfinding.pathnode.geometry import CELL
from flight.pathfinding.pathnode.polyline import poly_len


class Target:
    def __init__(self, field_w, field_h, start_edge, partner_entry_x, routes):
        """
        - start_edge: this side's entry edge; the target is on or towards the other one.
        - partner_entry_x: set when there is a partner side. Its point A starts at its entry
          until the first relay arrives. None aims at the far edge.
        """
        self.field_w = field_w
        self.far_y = field_h if Edge(start_edge) == Edge.BOTTOM else 0.0
        self.partner = partner_entry_x is not None
        self.partner_a = (float(partner_entry_x), self.far_y) if self.partner else None
        # False while heading for a stand-in (see route_from): the gap must not be declared
        # closed against it.
        self.is_partner = False
        self._routes = routes

    def route_from(self, frm):
        """Shortest route from `frm` to the target, as (points, gids) -- (None, None) if none.

        Far edge: straight ahead when that is clear, else the far-edge point with the
        shortest route.

        Partner: its point A. If that is inside an obstacle this side knows about, head for
        the obstacle corner nearest it -- the partner applies the same rule and re-places
        its point A just outside -- and is_partner goes False meanwhile."""
        routes = self._routes
        self.is_partner = False
        if self.partner:
            if not routes.contains(*self.partner_a):
                r = routes.route(frm, self.partner_a)
                if r[0] is not None:
                    self.is_partner = True
                    return r
            corners = sorted(routes.corners(), key=lambda v: math.dist(v, self.partner_a))
            for v in corners[:8]:
                r = routes.route(frm, v)
                if r[0] is not None:
                    return r
        straight = (min(max(frm[0], 0.0), self.field_w), self.far_y)
        if not routes.contains(*straight) and routes.zones.clear_line(frm, straight):
            return [tuple(frm), straight], [None, None]
        best = (None, None)
        best_len = math.inf
        for k in range(int(self.field_w // CELL)):
            cand = ((k + 0.5) * CELL, self.far_y)
            if routes.contains(*cand):
                continue
            r = routes.shortest(frm, cand)
            if r is not None and poly_len(r[0]) < best_len:
                best, best_len = r, poly_len(r[0])
        return best
