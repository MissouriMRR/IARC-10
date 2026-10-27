"""One drone's copy of its side's planner, in lat/lon.

Every drone owns one (Drone.planner). It is only an adapter: a FieldFrame (frame.py, lat/lon <->
local feet) in front of a SearchBackend (search_backend.py, local feet; "pathnode" by default).
Every coordinate crossing this interface is (lat, lon); claim jobs stay in local feet, which
every drone computes identically. flight/pathfinding/side_agent.py drives it and does the
messaging; this class never looks inside the backend.
"""

import logging

from flight.pathfinding.frame import FieldFrame
from flight.pathfinding.iarc_report import format_iarc_path
from flight.pathfinding.search_backend import backend_class, make_backend

# Load the default backend when this module is imported, not on first use: a lazy import would
# land inside the first drone's Takeoff (~0.5 s), delaying it behind its side-mate.
backend_class("pathnode")


class MissionPlanner:
    def __init__(self, corners_latlon, start_latlon, start_edge, side_drones, has_partner,
                 shape_size_ft=(6.0, 4.0), overlap=0.1, backend="pathnode", **backend_options):
        """
        Takes:
        - corners_latlon: the field corners, counter-clockwise from corner 1 (frame.py).
        - start_latlon, start_edge: where this side enters, and on which edge ("bottom"/"top").
        - side_drones: this side's drone ids, ascending (the claim keys; ties go to the first).
        - has_partner: whether a partner side works from the other edge.
        - shape_size_ft, overlap: the camera footprint (along y, across x) and photo overlap.
        - backend: which SearchBackend (a key of search_backend.BACKENDS); backend_options
          are its own settings.
        """
        self.frame = FieldFrame(corners_latlon)
        self.field_w, self.field_h = self.frame.width, self.frame.height
        entry_x = min(max(self.frame.to_local(*start_latlon)[0], 0.0), self.field_w)
        self.drones = tuple(side_drones)
        self.backend = make_backend(backend, self.field_w, self.field_h, entry_x, start_edge,
                                    self.drones, has_partner, shape_size_ft=shape_size_ft,
                                    overlap=overlap, **backend_options)
        self._started = False

    # -- the frame -------------------------------------------------------------------------
    def latlon_to_local(self, lat, lon):
        return self.frame.to_local(lat, lon)

    def local_to_latlon(self, x, y):
        return self.frame.to_latlon(x, y)

    # -- lifecycle -------------------------------------------------------------------------
    def start_if_needed(self):
        """None if already started; else True/False: whether any route exists at all."""
        if self._started:
            return None
        self._started = True
        if self.backend.failure:
            logging.error("search backend: %s", self.backend.failure)
        return self.backend.failure is None

    @property
    def failure(self):
        """Why planning stopped (no route left), or None."""
        return self.backend.failure

    def done(self):
        """Nothing left to do on this side."""
        return self.backend.done()

    def route_verified(self):
        """The route is whole and fully checked (optional work may remain)."""
        return self.backend.route_verified()

    # -- this side's photos ----------------------------------------------------------------
    def own_footprint(self, corners_latlon):
        """A photo this drone just took (Scan; may repeat while it climbs to retry)."""
        self.backend.own_footprint(self.frame.points_to_local(corners_latlon))

    def own_photo(self, corners_latlon, mines_latlon):
        """This drone's photo is done, with these mines found in it. Returns the mines to relay
        to the partner, as [(lat, lon)] -- only ones new to this side."""
        out = self.backend.own_photo(None, self.frame.points_to_local(mines_latlon))
        return self.frame.points_to_latlon(out.mines)

    def mate_photo(self, corners_latlon, mines_latlon):
        """A SHARE_PHOTOS report from the side-mate (corners may be empty: mines only)."""
        mines = self.frame.points_to_local(mines_latlon)
        if corners_latlon:
            self.backend.mate_photo(self.frame.points_to_local(corners_latlon), mines)
        else:
            self.backend.own_photo(None, mines)

    def failed_photo(self, lat, lon):
        """A photo planned at (lat, lon) that produced no footprint (a miss)."""
        self.backend.failed_photo(*self.frame.to_local(lat, lon))

    def cell_seen_at(self, lat, lon):
        """Whether the square at (lat, lon) is photographed; None if it's off the field."""
        return self.backend.cell_seen_at(*self.frame.to_local(lat, lon))

    # -- the partner side ------------------------------------------------------------------
    def partner_mine(self, lat, lon):
        self.backend.partner_mine(*self.frame.to_local(lat, lon))

    def partner_coverage(self, corners_latlon):
        self.backend.partner_coverage(self.frame.points_to_local(corners_latlon))

    def partner_point_a(self, lat, lon):
        self.backend.partner_point_a(*self.frame.to_local(lat, lon))

    def point_a_latlon(self):
        return self.frame.to_latlon(*self.backend.point_a())

    def mine_hash(self):
        return self.backend.mine_hash()

    def mines_latlon(self):
        """Every known mine, for CROSS_PAIR_MINE_SYNC."""
        return self.frame.points_to_latlon(self.backend.mines())

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
          the plan keeps clear of them. Left out: ignored.
        - lost: side drones gone quiet: their work goes to the others, but their last known
          position (in positions) and last announced job (in held) keep blocking.

        Returns: drone id -> [(lat, lon)] in flying order.
        """
        local = {}
        for d in self.drones:
            if d in positions and positions[d] is None:
                local[d] = None
            elif positions.get(d):
                local[d] = self.frame.to_local(*positions[d])
            else:
                local[d] = self.backend.point_a()
        far = {d: self.frame.to_local(*p) for d, p in (others or {}).items() if p is not None}
        plan = self.backend.assignments(local, held, me, far, lost)
        return {d: self.frame.points_to_latlon([(x, y) for x, y, _g in plan.get(d, [])])
                for d in self.drones}

    def current_job(self, drone):
        """The job `drone` holds per the last assignments (opaque; local feet inside), or None,
        to announce to the side-mate."""
        return self.backend.current_job(drone)

    # -- expanding mines (levels, snapshots, lockstep): see search_backend.py ----------------
    @property
    def level(self):
        """The growth level this copy works at."""
        return self.backend.level

    def level_state(self, me):
        """The LevelState (levels.py) to announce to the side-mate and the other side."""
        return self.backend.level_state(me)

    def hear_level_state(self, state):
        """A LevelState announced by a side-mate or the other side."""
        self.backend.hear_level_state(state)

    def try_advance_level(self, me, now=0.0, lost=(), max_level=None):
        """Snapshot if ready, grow one level if the lockstep rule allows. Returns a LevelStep."""
        return self.backend.try_advance_level(me, now, lost, max_level)

    def growth_fits(self, remaining_s):
        """Whether `remaining_s` seconds allow another growth step (1.5 x the last one's time)."""
        return self.backend.growth_fits(remaining_s)

    def valid_snapshots_latlon(self):
        """The valid snapshots, oldest first, as [(level, mine_hash, route [(lat, lon)])]."""
        return [(s.level, s.mine_hash, self.frame.points_to_latlon(s.route))
                for s in self.backend.valid_snapshots()]

    def report_route_latlon(self, level=None):
        """The route to report (newest valid snapshot's; at `level` if given), or None."""
        route = self.backend.report_route(level)
        return None if route is None else self.frame.points_to_latlon(route)

    def report_valid(self):
        """Whether report_route_latlon() is a valid snapshot."""
        return self.backend.report_valid()

    def join_at_latlon(self, level, partner_route_latlon, partner_hash=None):
        """The snapshot at `level` joined with the partner's snapshot route, or None."""
        joined = self.backend.join_at(level, self.frame.points_to_local(partner_route_latlon),
                                      partner_hash)
        return None if joined is None else self.frame.points_to_latlon(joined)

    def buffer_width(self):
        """The reported route's clearance in squares (for the IARC report's S line)."""
        return self.backend.buffer_width()

    def partner_lost(self):
        """The partner side is gone: head for its last point A, then on to the far edge."""
        self.backend.partner_lost()

    def expand_margin(self, ft):
        """Kept so ExpandNodes (state_machine/states/impl/expand_nodes_impl.py) still runs: it
        does nothing. The next stage replaces that call with try_advance_level."""
        logging.info("MissionPlanner.expand_margin: replaced by try_advance_level (next stage)")
        return None

    # -- results ---------------------------------------------------------------------------
    def route_latlon(self):
        return self.frame.points_to_latlon(self.backend.route())

    def join_latlon(self, partner_route_latlon):
        """This side's route joined with the partner's, or None if they don't connect."""
        joined = self.backend.join(self.frame.points_to_local(partner_route_latlon))
        return None if joined is None else self.frame.points_to_latlon(joined)

    def check_join_latlon(self, partner_route_latlon):
        """join_latlon, but if the halves don't connect the bottom side re-opens its gap toward
        the partner's route end (backend.partner_route) -- done() goes False."""
        joined = self.backend.partner_route(self.frame.points_to_local(partner_route_latlon))
        return None if joined is None else self.frame.points_to_latlon(joined)

    def iarc_path(self, route_latlon, buffer_width=0):
        return format_iarc_path(self.frame.points_to_local(route_latlon), buffer_width,
                                strict=False, field_w=self.field_w, field_h=self.field_h)


def make_planner(flight_settings, shape_size_ft, overlap, backend="pathnode"):
    """This drone's copy of its side's planner, on the named SearchBackend."""
    from state_machine.flight_settings import Side   # local: avoids an import cycle
    corners = [(c["lat"], c["lon"]) for c in flight_settings.mission_field_corners]
    start = (flight_settings.start_coord["lat"], flight_settings.start_coord["lon"])
    edge = "bottom" if flight_settings.start_side == Side.START else "top"
    return MissionPlanner(corners, start, edge, tuple(flight_settings.side_drones),
                          bool(flight_settings.partner_side_drones),
                          shape_size_ft=shape_size_ft, overlap=overlap, backend=backend)
