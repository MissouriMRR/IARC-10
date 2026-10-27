"""What the forward gap aims at. Kinds, chosen by make_target (and handed over by lost / after_met):
- FarEdgeTarget       the far edge (1-2 drones, no partner side)
- PartnerTarget       the partner side's latest point A (3-4 drones)
- LostPartnerTarget   the partner side is gone: its last point A, then on to the far edge
"""

import math
from abc import ABC, abstractmethod

from flight.pathfinding.pathnode.config import Edge
from flight.pathfinding.pathnode.geometry import CELL
from flight.pathfinding.pathnode.polyline import poly_len


class Target(ABC):
    """
    What the planner's forward gap closes on.

    Contract:
    - has_partner: whether a partner side exists (class attribute).
    - route_from(frm): shortest route from `frm` to the target.
    - may_close(): whether the gap may be declared closed against the target as it stands.
    - retarget(point): move the target to the partner's point A; False when nothing changed.
    - lost(): the Target to use once the partner side is gone (default: this one).
    - after_met(): the Target to carry on with once this one is met (default: this one).
    """
    has_partner = False

    def __init__(self, field_w, field_h, start_edge, routes):
        """
        - start_edge: this side's entry edge; the target is on or towards the other one.
        """
        self.field_w, self.field_h, self.start_edge = field_w, field_h, Edge(start_edge)
        self.far_y = self.start_edge.opposite.y(field_h)
        self._routes = routes

    @abstractmethod
    def route_from(self, frm):
        """Shortest route from `frm` to the target, as (points, gids) -- (None, None) if none."""

    @abstractmethod
    def may_close(self):
        """Whether the gap may close against the target as it stands: False while it is only a
        stand-in (see PartnerTarget.route_from). Valid after the latest route_from."""

    @abstractmethod
    def retarget(self, point):
        """Aim at the partner's point A. Returns whether the target changed."""

    def lost(self):
        """The Target for a side whose partner is gone."""
        return self

    def after_met(self):
        """The Target to carry on with once the chain has reached this one."""
        return self

    def _far_edge_route(self, frm):
        """Straight ahead to the far edge when that is clear, else the far-edge point with the
        shortest route -- (points, gids), (None, None) if none."""
        routes = self._routes
        straight = (min(max(frm[0], 0.0), self.field_w), self.far_y)
        if not routes.contains(*straight) and routes.clear_line(frm, straight):
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


class FarEdgeTarget(Target):
    """The far edge: there is no partner side, so there is nothing to retarget."""

    def route_from(self, frm):
        return self._far_edge_route(frm)

    def may_close(self):
        return True

    def retarget(self, point):
        return False


class PartnerTarget(Target):
    """The partner side's latest point A; at its entry until the first relay arrives."""
    has_partner = True

    def __init__(self, field_w, field_h, start_edge, partner_entry_x, routes):
        super().__init__(field_w, field_h, start_edge, routes)
        self.partner_a = (float(partner_entry_x), self.far_y)
        # False while heading for a stand-in (see route_from): the gap must not be declared
        # closed against it.
        self._on_partner = False

    def route_from(self, frm):
        """Its point A. If that is inside an obstacle this side knows about, head for the
        obstacle corner nearest it -- the partner applies the same rule and re-places its
        point A just outside -- and may_close() goes False meanwhile. With no route to either,
        the far edge."""
        routes = self._routes
        self._on_partner = False
        if not routes.contains(*self.partner_a):
            r = routes.route(frm, self.partner_a)
            if r[0] is not None:
                self._on_partner = True
                return r
        corners = sorted(routes.corners(), key=lambda v: math.dist(v, self.partner_a))
        for v in corners[:8]:
            r = routes.route(frm, v)
            if r[0] is not None:
                return r
        return self._far_edge_route(frm)

    def may_close(self):
        return self._on_partner

    def retarget(self, point):
        point = (float(point[0]), float(point[1]))
        if point == self.partner_a:
            return False        # the heartbeat resends an unchanged point A
        self.partner_a = point
        return True

    def lost(self):
        return LostPartnerTarget(self)


class LostPartnerTarget(PartnerTarget):
    """The partner side is gone. Heads for its last point A (the ground its photos covered is
    checked, so that stretch is cheap), ignores any later point A, and once there hands over to
    FarEdgeTarget: this side's own photos then check the lost side's half to the far edge."""

    def __init__(self, was):
        """was: the PartnerTarget it replaces (point A and stand-in state carry over)."""
        super().__init__(was.field_w, was.field_h, was.start_edge, 0.0, was._routes)
        self.partner_a, self._on_partner = was.partner_a, was._on_partner
        self._at_far_edge = False       # route_from fell back to the far edge

    def route_from(self, frm):
        """Its last point A when that is clear and reachable, else the far edge (it will not
        move: nobody is left to re-place it)."""
        routes = self._routes
        self._on_partner = self._at_far_edge = False
        if not routes.contains(*self.partner_a):
            r = routes.route(frm, self.partner_a)
            if r[0] is not None:
                self._on_partner = True
                return r
        self._on_partner = self._at_far_edge = True     # the far edge is a real target now
        return self._far_edge_route(frm)

    def retarget(self, point):
        return False

    def lost(self):
        return self

    def after_met(self):
        if self._at_far_edge:
            return self
        return FarEdgeTarget(self.field_w, self.field_h, self.start_edge, self._routes)


def make_target(field_w, field_h, start_edge, partner_entry_x, routes):
    """The Target for a side: PartnerTarget when partner_entry_x is set, else FarEdgeTarget."""
    if partner_entry_x is None:
        return FarEdgeTarget(field_w, field_h, start_edge, routes)
    return PartnerTarget(field_w, field_h, start_edge, partner_entry_x, routes)
