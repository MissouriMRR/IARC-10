"""One drone's copy of its side's planner, in lat/lon.

Every drone owns one (Drone.planner): a SideSession (pathnode/session.py -- the same event
handling the simulator uses) behind a lat/lon frame (frame.py). flight/pathfinding/
side_agent.py drives it and does the messaging. Every coordinate crossing this interface is
(lat, lon); claim jobs stay in local feet, which every drone computes identically.
"""

import hashlib
import logging

from flight.pathfinding.frame import field_transformer, validate_field_corners
from flight.pathfinding.iarc_report import format_iarc_path
from flight.pathfinding.pathnode.config import Edge, PlannerConfig
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.session import SideSession


def relay_id(lat, lon):
    """A short id for a mine relay, from its position (echoed back in the ack)."""
    return hashlib.sha256(f"{lat:.7f},{lon:.7f}".encode("utf-8")).hexdigest()[:16]


class MissionPlanner:
    def __init__(self, corners_latlon, start_latlon, start_edge, side_drones, has_partner,
                 shape_size_ft=(6.0, 4.0), overlap=0.1):
        """
        - corners_latlon: the field corners, counter-clockwise from corner 1 (frame.py).
        - start_latlon, start_edge: where this side enters, and on which edge.
        - side_drones: this side's drone ids, ascending (the claim keys; ties go to the first).
        - has_partner: whether a partner side works from the other edge.
        """
        validate_field_corners(corners_latlon)
        self._t, self.field_w = field_transformer(corners_latlon)
        pts = [self._t.latlon_to_local(lat, lon) for lat, lon in corners_latlon]
        self.field_h = max(y for _x, y in pts)
        entry_x = min(max(self._t.latlon_to_local(*start_latlon)[0], 0.0), self.field_w)
        self.drones = tuple(side_drones)
        self.planner = PathNodePlanner(
            self.field_w, self.field_h, entry_x, Edge(start_edge),
            partner_entry_x=entry_x if has_partner else None, drones=self.drones,
            config=PlannerConfig(shape_size_ft=shape_size_ft, overlap=overlap))
        self.session = SideSession(self.planner)
        self._started = False

    # -- the frame -------------------------------------------------------------------------
    def latlon_to_local(self, lat, lon):
        x, y = self._t.latlon_to_local(lat, lon)
        return float(x), float(y)

    def local_to_latlon(self, x, y):
        lat, lon = self._t.local_to_latlon(x, y)
        return float(lat), float(lon)

    def _local(self, pts):
        return [self.latlon_to_local(lat, lon) for lat, lon in pts]

    def _latlon(self, pts):
        return [self.local_to_latlon(x, y) for x, y in pts]

    def _relays(self, mines_local):
        out = []
        for m in mines_local:
            lat, lon = self.local_to_latlon(*m)
            out.append((lat, lon, relay_id(lat, lon)))
        return out

    # -- lifecycle -------------------------------------------------------------------------
    def start_if_needed(self):
        """None if already started; else True/False: whether any route exists at all."""
        if self._started:
            return None
        self._started = True
        if self.planner.failure:
            logging.error("path-node planner: %s", self.planner.failure)
        return self.planner.failure is None

    @property
    def failure(self):
        """Why planning stopped (no route left), or None."""
        return self.planner.failure

    def done(self):
        """Nothing left to do on this side."""
        return self.planner.done()

    def route_verified(self):
        """The route is whole and fully checked (optional work may remain)."""
        return self.planner.route_verified()

    # -- this side's photos ----------------------------------------------------------------
    def own_footprint(self, corners_latlon):
        """A photo this drone just took (Scan; may repeat while it climbs to retry)."""
        self.session.own_footprint(self._local(corners_latlon))

    def own_photo(self, corners_latlon, mines_latlon):
        """This drone's photo is done, with these mines found in it. Returns the mines to relay
        to the partner, as [(lat, lon, relay_id)] -- only ones new to this side."""
        out = self.session.own_photo(None, self._local(mines_latlon))
        return self._relays(out.mines)

    def mate_photo(self, corners_latlon, mines_latlon):
        """A SHARE_PHOTOS report from the side-mate (corners may be empty: mines only)."""
        if corners_latlon:
            self.session.mate_photo(self._local(corners_latlon), self._local(mines_latlon))
        else:
            self.session.own_photo(None, self._local(mines_latlon))

    def failed_photo(self, lat, lon):
        """A photo planned at (lat, lon) that produced no footprint (a miss; see coverage.py)."""
        self.session.failed_photo(*self.latlon_to_local(lat, lon))

    def cell_seen_at(self, lat, lon):
        """Whether the square at (lat, lon) is photographed; None if it's off the field."""
        return self.planner.cell_seen_at(*self.latlon_to_local(lat, lon))

    # -- the partner side ------------------------------------------------------------------
    def partner_mine(self, lat, lon):
        self.session.partner_mine(*self.latlon_to_local(lat, lon))

    def partner_coverage(self, corners_latlon):
        self.session.partner_coverage(self._local(corners_latlon))

    def partner_point_a(self, lat, lon):
        self.session.partner_point_a(*self.latlon_to_local(lat, lon))

    def point_a_latlon(self):
        return self.local_to_latlon(*self.planner.point_a())

    def mine_hash(self):
        return self.planner.mine_hash()

    def mines_latlon(self):
        """Every known mine, for CROSS_PAIR_MINE_SYNC."""
        return self._latlon(self.planner.discovered)

    # -- work ------------------------------------------------------------------------------
    def assignments(self, positions, held=None, me=None, others=None, lost=()):
        """
        Takes:
        - positions: drone id -> (lat, lon) for this side's drones; None for a drone that is
          gone (ignored). A drone left out is taken to be at point A.
        - held: drone id -> the job it announced (current_job on its own copy); the other
          side's drones too.
        - me: the drone this copy belongs to (it takes others' work when it has none).
        - others: drone id -> (lat, lon) for the other side's drones (last known if quiet);
          the plan keeps clear of them (planner.plan). Left out: ignored.
        - lost: side drones gone quiet: their work goes to the others, but their last known
          position (in positions) and last announced job (in held) keep blocking (planner.plan).

        Returns: drone id -> [(lat, lon)] in flying order (claims.py).
        """
        local = {}
        for d in self.drones:
            if d in positions and positions[d] is None:
                local[d] = None
            elif positions.get(d):
                local[d] = self.latlon_to_local(*positions[d])
            else:
                local[d] = self.planner.point_a()
        far = {d: self.latlon_to_local(*p) for d, p in (others or {}).items() if p is not None}
        plan = self.session.assignments(local, held, me, far, lost)
        return {d: self._latlon([(x, y) for x, y, _g in plan.get(d, [])]) for d in self.drones}

    def current_job(self, drone):
        """The job `drone` holds per the last assignments -- (group, key, (x, y)) in local
        feet, or None -- to announce to the side-mate."""
        return self.session.current_job(drone)

    def expand_margin(self, ft):
        """ExpandNodes. Returns a dict for the flight log, or None if it did nothing."""
        logging.info("path-node planner: ExpandNodes margin not applied (next phase)")
        return None

    # -- results ---------------------------------------------------------------------------
    def route_latlon(self):
        return self._latlon(self.planner.route())

    def join_latlon(self, partner_route_latlon):
        """This side's route joined with the partner's, or None if they don't connect."""
        joined = self.planner.join(self._local(partner_route_latlon))
        if joined is None or self.planner.engine.crosses(joined):
            return None
        return self._latlon(joined)

    def check_join_latlon(self, partner_route_latlon):
        """join_latlon, but if the halves don't connect the bottom side re-opens its gap toward
        the partner's route end (session.partner_route) -- done() goes False."""
        joined = self.session.partner_route(self._local(partner_route_latlon))
        if joined is None or self.planner.engine.crosses(joined):
            return None
        return self._latlon(joined)

    def iarc_path(self, route_latlon, buffer_width=0):
        return format_iarc_path(self._local(route_latlon), buffer_width, strict=False,
                                field_w=self.field_w, field_h=self.field_h)


def make_planner(flight_settings, shape_size_ft, overlap):
    """This drone's copy of its side's planner."""
    from state_machine.flight_settings import Side   # local: avoids an import cycle
    corners = [(c["lat"], c["lon"]) for c in flight_settings.mission_field_corners]
    start = (flight_settings.start_coord["lat"], flight_settings.start_coord["lon"])
    edge = Edge.BOTTOM if flight_settings.start_side == Side.START else Edge.TOP
    return MissionPlanner(corners, start, edge, tuple(flight_settings.side_drones),
                          bool(flight_settings.partner_side_drones),
                          shape_size_ft=shape_size_ft, overlap=overlap)
