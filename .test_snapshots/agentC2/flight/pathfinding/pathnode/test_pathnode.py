"""Tests for the path-node planner. Real assertions -- runs under pytest or as a script.

Chains are built by hand for the mine-rule cases: in ordinary missions a convex mine on a
near-straight chain deletes one contiguous run of nodes, so the orphan rule rarely fires.
"""
import math
import random
import subprocess
import sys

from flight.pathfinding.cellField.cellField import CellField
from flight.pathfinding.pathnode.chain import Chain
from flight.pathfinding.pathnode.claims import ClaimBook
from flight.pathfinding.pathnode.config import GROUPS, MISS_BLOCK, Edge, Placer, Source, Tier
from flight.pathfinding.pathnode.join import join_routes
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.polyline import poly_len
from flight.pathfinding.pathnode.sim import Mission, evaluate

PAIR, PARTNER = Source.PAIR, Source.PARTNER


def _planner(points, kinds, partner=True, **options):
    """Planner whose chain is `points` (last point = target) joined by links of `kinds`."""
    pl = PathNodePlanner(80, 300, points[0][0], Edge.BOTTOM,
                         partner_entry_x=points[-1][0] if partner else None, **options)
    pl.use_chain(Chain.build(points, kinds))
    if partner:
        pl.target.partner_a = tuple(float(v) for v in points[-1])
    return pl


def _rect_photo(pl, x, y):
    """A photo of the planning footprint centred at (x, y)."""
    along, across = pl.config.shape_size_ft
    hx, hy = across / 2, along / 2
    pl.record_footprint([(x - hx, y - hy), (x + hx, y - hy), (x + hx, y + hy), (x - hx, y + hy)])


def _consistent(pl):
    return all(math.dist(k.poly[0], pl.nodes[i].xy) < 1e-6
               and math.dist(k.poly[-1], pl.nodes[i + 1].xy) < 1e-6
               for i, k in enumerate(pl.links))


def _clear(pl):
    return all(not pl.engine.crosses(k.poly) for k in pl.links)


def test_footprint_model_matches_fill_polygon_covered():
    pl = PathNodePlanner(80, 300, 40)
    rng = random.Random(0)
    for _ in range(500):
        x, y = rng.uniform(3, 77), rng.uniform(4, 296)
        f = CellField(40, 150, max_corner=(80, 300))
        f.fill_polygon_covered([(x - 2, y - 3), (x + 2, y - 3), (x + 2, y + 3), (x - 2, y + 3)])
        assert set(f.on_cells()) == pl.footprint_cells(x, y)


def test_quadrilateral_footprint_marks_only_fully_covered_squares():
    """The real camera footprint is a rotated quadrilateral, not the planning rectangle."""
    pl = PathNodePlanner(80, 300, 40)
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    quad = [(40 + dx * c - dy * s, 100 + dx * s + dy * c)
            for dx, dy in ((-4, -4), (4, -4), (4, 4), (-4, 4))]
    pl.record_footprint(quad)
    from shapely.geometry import Polygon, box
    shape = Polygon(quad)
    for col in range(15, 26):
        for row in range(44, 57):
            inside = shape.buffer(1e-9).contains(box(col * 2, row * 2, col * 2 + 2, row * 2 + 2))
            assert pl.seen.get(col, row) == inside, (col, row)


def test_mid_chain_cut_is_bridged_not_orphaned():
    """A mine cuts the middle of the chain: the far fragment still reaches the target, so it
    is kept and re-bridged by a pending span."""
    pts = [(40, 0), (40, 10), (40, 20), (40, 30), (40, 40), (40, 300)]
    pl = _planner(pts, ["checked"] * 4 + ["forward"])
    n1, n3, n4 = pl.nodes[1], pl.nodes[3], pl.nodes[4]
    pl.add_mine(40.0, 20.0, PAIR)
    assert pl.failure is None and pl.stats["orphans"] == 0
    assert n3 in pl.nodes and n4 in pl.nodes
    i = pl.nodes.index(n1)
    assert pl.nodes[i + 1] is n3 and pl.links[i].kind == "pending"
    assert _clear(pl) and _consistent(pl)


def test_frontier_orphan_deleted_and_gambler_jumps():
    """The mine takes point A and a node behind it, stranding a node between them that is
    outside the mine. It is orphaned and deleted anyway; the gambler jumps past the mine."""
    pts = [(40, 0), (40, 20), (48, 24), (40, 28), (40, 300)]
    pl = _planner(pts, ["checked", "checked", "checked", "forward"])
    stranded = pl.nodes[2]
    pl.add_mine(40.0, 24.0, PAIR)
    assert pl.failure is None
    assert not pl.engine.contains(*stranded.xy), "the orphan is outside the mine..."
    assert stranded not in pl.nodes, "...and is deleted anyway"
    assert pl.stats["orphans"] == 1 and pl.stats["jumps"] == 1
    assert [k.kind for k in pl.links] == ["pending", "forward"]
    assert pl.nodes[1].placed_by == "jump"
    assert _clear(pl) and _consistent(pl)


def test_assistant_finds_jump_but_relayed_mines_only_rebridge():
    """Either drone of the pair may be pioneering, so a find by either jumps; a mine relayed
    from the other pair only re-bridges."""
    for source, jumps in ((PAIR, 1), (PARTNER, 0)):
        pts = [(40, 0), (40, 20), (48, 24), (40, 28), (40, 300)]
        pl = _planner(pts, ["checked", "checked", "checked", "forward"])
        pl.add_mine(40.0, 24.0, source)
        assert pl.failure is None and pl.stats["jumps"] == jumps, source
        kinds = [k.kind for k in pl.links]
        assert kinds == (["pending", "forward"] if jumps else ["forward"]), (source, kinds)
        assert _clear(pl) and _consistent(pl)


def test_island_between_two_cuts_is_orphaned_after_meeting():
    """After meeting (everything checked to the target), a route passing the same mine twice:
    the stretch between the cuts reaches no root, so it goes."""
    pts = [(40, 0), (40, 90), (40, 112), (52, 112), (52, 90), (44, 90), (44, 112), (60, 112),
           (60, 300)]
    pl = _planner(pts, ["checked"] * 8)
    island = pl.nodes[2:6]
    pl.add_mine(41.0, 101.0, PAIR)
    assert pl.failure is None
    assert all(n not in pl.nodes for n in island)
    assert all(not pl.engine.contains(*n.xy) for n in island)
    assert pl.stats["orphans"] == 4
    assert [k.kind for k in pl.links].count("pending") == 1
    assert _clear(pl) and _consistent(pl)


def test_cut_into_target_after_meeting_reopens_the_gap():
    pts = [(40, 0), (40, 100), (40, 110)]
    pl = _planner(pts, ["checked", "checked"])
    assert pl.met
    pl.add_mine(40.0, 106.0, PARTNER)
    assert pl.failure is None and not pl.met
    assert pl.links[-1].kind == "forward"
    assert _clear(pl) and _consistent(pl)


def test_duplicate_mine_is_ignored():
    pl = PathNodePlanner(80, 300, 40)
    pl.add_mine(40.3, 150.2, PAIR)
    before = (len(pl.discovered), dict(pl.stats))
    pl.add_mine(40.9, 150.8, PARTNER)       # same square
    assert (len(pl.discovered), dict(pl.stats)) == before


def test_far_edge_target_routes_around_a_mine():
    pl = PathNodePlanner(80, 300, 40)
    assert pl.nodes[-1].xy == (40.0, 300.0)
    pl.add_mine(40.0, 150.0, PARTNER)
    assert pl.failure is None
    assert abs(pl.nodes[-1].y - 300.0) < 1e-9
    assert _clear(pl) and _consistent(pl)


def test_join_routes_splices_at_first_shared_point():
    own = [(40, 0), (40, 100), (40, 150), (40, 160)]
    partner = [(40, 300), (40, 200), (40, 160), (40, 150)]
    assert join_routes(own, partner) == [(40, 0), (40, 100), (40, 150), (40, 160), (40, 200),
                                         (40, 300)]
    assert join_routes([(0, 0), (1, 1)], [(5, 5), (6, 6)]) is None


def _line(x, y0, y1, step=4.0):
    return [(x, y0 + i * step) for i in range(int((y1 - y0) / step) + 1)]


def test_claims_are_held_per_path_not_per_point():
    """Two paths sharing a photo point each get an owner of their own."""
    book = ClaimBook((1, 2))
    a = [(10.0, 10.0), (20.0, 20.0)]
    b = [(40.0, 20.0), (20.0, 20.0)]              # ends where a ends
    positions = {1: (10.0, 9.0), 2: (40.0, 21.0)}
    for _ in range(2):                             # and they stay that way (stickiness)
        plan = book.update([[("p1", a), ("p2", b)]], positions)
        got = {d: [(x, y) for x, y, _g in v] for d, v in plan.items()}
        assert got[1] == a and got[2] == b, got


def test_a_lost_drone_is_handed_nothing_and_its_work_goes_to_the_other():
    book = ClaimBook((1, 2))
    groups = [[("p1", _line(10, 0, 20)), ("p2", _line(60, 0, 20))]]
    book.update(groups, {1: (10.0, 0.0), 2: (60.0, 0.0)})
    plan = book.update(groups, {1: (10.0, 0.0), 2: None})
    assert plan[2] == [] and len(plan[1]) == 12, plan


def test_claims_are_deterministic():
    groups = [[("forward", _line(40, 10, 80)), (7, _line(30, 4, 20))],
              [(None, [(p, q)]) for p, q in ((20, 10), (60, 50), (44, 70), (10, 90))]]
    pos = {1: (40, 8), 2: (30, 0)}
    a = ClaimBook((1, 2)).update(groups, pos)
    b = ClaimBook((1, 2)).update(groups, pos)
    assert a == b
    every = sorted((x, y, g) for q in a.values() for x, y, g in q)
    assert every == sorted({(x, y, g) for g, seqs in enumerate(groups) for _k, seq in seqs
                            for x, y in seq}), "every waypoint is claimed exactly once"


def test_drone_at_point_a_claims_the_whole_path_and_the_other_takes_the_detour():
    forward = _line(40, 30, 290)
    detour = _line(36, 6, 26)
    book = ClaimBook((1, 2))
    q = book.update([[("forward", forward), (3, detour)], []],
                    {1: (40, 28), 2: (36, 2)})
    assert [p[:2] for p in q[1]] == forward
    assert [p[:2] for p in q[2]] == detour


def test_a_path_is_claimed_whole():
    book = ClaimBook((1, 2))
    path = _line(40, 0, 200)
    q = book.update([[("forward", path)], []], {1: (40, 0), 2: (40, 200)})
    held = [d for d in q if q[d]]
    assert len(held) == 1 and len(q[held[0]]) == len(path)


def test_idle_drone_takes_lower_group_while_higher_is_claimed():
    book = ClaimBook((1, 2))
    q = book.update([[("forward", _line(40, 30, 100))], [(None, [(40, 10)]), (None, [(44, 14)])]],
                    {1: (40, 28), 2: (40, 6)})
    assert all(g == 0 for _x, _y, g in q[1])
    assert {(x, y) for x, y, g in q[2] if g == 1} == {(40, 10), (44, 14)}


def test_only_the_current_job_stays_claimed():
    """A replan moves every photo on a path; each drone keeps the path it is working on, even
    though the assistant is now nearer the forward path."""
    book = ClaimBook((1, 2))
    book.update([[("forward", _line(40, 30, 100)), (5, _line(20, 30, 60))], []],
                {1: (40, 28), 2: (20, 0)})
    # Both paths shift 3ft sideways; the assistant has flown next to the forward path.
    q = book.update([[("forward", _line(43, 30, 100)), (5, _line(23, 30, 60))], []],
                    {1: (40, 28), 2: (41, 29)})
    assert q[1] and all(x == 43 for x, _y, _g in q[1]), "kept its current path"
    assert q[2] and all(x == 23 for x, _y, _g in q[2])


def _detour_planner():
    """Chain: entry -> jump (40,20) -> a detour out to x=60 -> jump (40,100) -> forward gap.
    The section between the anchors is ~117ft; straight it would be 100ft."""
    pts = [(40, 0), (40, 20), (60, 40), (60, 80), (40, 100), (40, 300)]
    # these tests aren't about the photo cap
    pl = _planner(pts, ["checked"] * 4 + ["forward"], partner=False, shortcut_max_waypoints=1000)
    pl.nodes[1].placed_by = pl.nodes[4].placed_by = Placer.JUMP
    return pl


def _active(pl):
    """The shortcut being tried, in whichever tier (tests have at most one)."""
    return next((t for t in pl.shortcuts.trials.values() if t is not None), None)


def _photograph(pl, poly):
    """Photograph every square along poly."""
    for x, y in pl.coverage.places_along(poly):
        _rect_photo(pl, x, y)


def test_shortcut_is_tried_and_spliced_only_once_fully_checked():
    pl = _detour_planner()
    before = pl.route()
    pl.advance()
    sc = _active(pl)
    assert sc is not None and pl.shortcuts.trials[Tier.ORDINARY] is sc, "17ft saved: ordinary"
    assert pl.groups()[GROUPS.index("shortcut")], "a shortcut is offered"
    assert pl.route() == before, "the valid chain is untouched while the shortcut is tried"
    (x0, y0), (x1, y1) = sc.poly[0], sc.poly[1]
    _photograph(pl, [(x0, y0), ((x0 + x1) / 2, (y0 + y1) / 2)])     # half of it
    pl.advance()
    assert pl.route() == before and _active(pl) is not None
    _photograph(pl, sc.poly)
    pl.advance()
    assert pl.stats["shortcuts"] == 1
    assert poly_len(pl.route()) < poly_len(before) - 10
    assert _clear(pl) and _consistent(pl)


def test_shortcut_is_given_up_when_a_mine_makes_it_too_long():
    pl = _detour_planner()
    pl.advance()
    before = pl.route()
    assert _active(pl) is not None
    for x in range(16, 53, 4):          # a wall across the shortcut, clear of the chain
        pl.add_mine(float(x), 60.0, PARTNER)
    pl.advance()
    assert pl.stats["shortcuts_given_up"] >= 1 and _active(pl) is None
    assert pl.route() == before, "the valid chain is untouched"
    assert _clear(pl) and _consistent(pl)


def test_shortcut_never_replaces_the_whole_path():
    pts = [(40, 0), (70, 100), (70, 200), (40, 300)]
    pl = _planner(pts, ["checked"] * 3, partner=False)     # met; anchors: entry, target
    pl.advance()
    assert _active(pl) is None
    assert all(pl.shortcuts.best(t, pl.met) is None for t in Tier)


def _big_detour_planner(**kw):
    """jump (40,20) -> out to x=70 -> jump (40,70): a 93ft section a 50ft straight line
    could replace."""
    pts = [(40, 0), (40, 20), (70, 30), (70, 60), (40, 70), (40, 300)]
    options = dict(shortcut_max_waypoints=1000)     # unless a test is about the cap
    options.update(kw)
    pl = _planner(pts, ["checked"] * 4 + ["forward"], partner=False, **options)
    pl.nodes[1].placed_by = pl.nodes[4].placed_by = Placer.JUMP
    return pl


def test_big_shortcut_comes_before_the_adjacent_check():
    pl = _big_detour_planner()
    pl.advance()
    assert pl.shortcuts.trials[Tier.BIG] is not None
    assert GROUPS.index("big_shortcut") < GROUPS.index("adjacent")
    assert pl.groups()[GROUPS.index("big_shortcut")]
    _photograph(pl, pl.shortcuts.trials[Tier.BIG].poly)
    pl.advance()
    assert pl.stats["big_shortcuts"] == 1 and _clear(pl) and _consistent(pl)


def test_shortcuts_needing_too_many_photos_are_never_tried():
    pl = _big_detour_planner()
    need = pl.coverage.photos_needed([(40, 20), (40, 70)])
    # only the flat allowance counts
    pl.reconfigure(shortcut_ft_per_photo=1000.0, shortcut_max_waypoints=need - 1)
    pl.advance()
    assert _active(pl) is None
    pl.reconfigure(shortcut_max_waypoints=need)
    pl.advance()
    assert _active(pl) is not None


def test_a_big_saving_earns_more_photos():
    """A shortcut past the flat cap is still tried if it saves enough per photo."""
    pl = _big_detour_planner()
    need = pl.coverage.photos_needed([(40, 20), (40, 70)])
    saving = pl.chain.section(1, 4) - 50.0
    # just pays for itself
    pl.reconfigure(shortcut_max_waypoints=need - 1, shortcut_ft_per_photo=saving / (need + 0.5))
    pl.advance()
    assert _active(pl) is not None


def test_shortcut_over_photographed_ground_is_applied_at_once():
    """A long shortcut costs nothing to check if its ground is already photographed."""
    pl = _big_detour_planner(shortcut_max_waypoints=0)
    _photograph(pl, [(40, 20), (40, 70)])
    pl.advance()
    assert pl.stats["instant_shortcuts"] == 1 and _active(pl) is None
    assert [n.xy for n in pl.nodes[:3]] == [(40, 0), (40, 20), (40, 70)]
    assert _clear(pl) and _consistent(pl)


def test_outer_layer_is_the_next_ring_out_and_comes_last():
    """A finished straight path along x=40 (it touches column 20): the adjacent layer is
    columns 19-21, the outer layer the next ones out, 18 and 22."""
    pl = _planner([(40, 0), (40, 300)], ["checked"], partner=False, shortcut_ratio=None)
    assert GROUPS[-1] == "outer"
    assert {c for c, _r in pl.work.route_cells()} == {20}
    pl.record_footprint([(38, 0), (44, 0), (44, 300), (38, 300)])      # columns 19-21
    assert not pl.work.adjacent()
    outer = pl.work.outer()
    assert {c for c, _r in outer} == {18, 22}
    assert pl.groups()[GROUPS.index("outer")] and not pl.done()
    pl.record_footprint([(36, 0), (46, 0), (46, 300), (36, 300)])      # columns 18-22
    assert not pl.work.outer() and pl.done()


def test_every_drone_count_completes_with_a_valid_route():
    for drones in (1, 2, 3, 4):
        for seed in range(1, 7):
            for n in (70, 90):
                m = Mission(generate_true_minefield(n, seed), drones=drones).run()
                e = evaluate(m)
                assert e["complete"] and e["joined"] and e["spans_field"], (drones, seed, n, m.failure)
                assert e["unseen_cells"] == 0 and e["bad_edges_known"] == 0, (drones, seed, n, e)
                # The route may pass near a mine nobody found, never over its square.
                assert e["route_mine_squares"] == 0, (drones, seed, n, e)


def test_completes_despite_dropped_and_delayed_relays():
    for seed in range(1, 9):
        m = Mission(generate_true_minefield(90, seed), drones=4, drop=0.5, delay=3,
                    seed=seed).run()
        e = evaluate(m)
        assert m.bus.dropped > 0
        assert e["complete"] and e["spans_field"], (seed, m.failure)
        assert e["unseen_cells"] == 0 and e["bad_edges_known"] == 0, (seed, e)


def test_band_coverage_leaves_no_route_square_in_danger():
    for seed in range(1, 5):
        m = Mission(generate_true_minefield(70, seed), drones=4, coverage="band").run()
        assert evaluate(m)["route_cells_in_danger"] == 0, seed


def _corners_latlon(order):
    """Field corners (lat, lon) for an 80 x 300 ft field, listed in `order` (indices into
    bottom-left, bottom-right, top-right, top-left)."""
    base_lat, base_lon, ft = 36.0, -95.9, 0.3048
    m_lon = 111320.0 * math.cos(math.radians(base_lat))
    pts = [(0, 0), (80, 0), (80, 300), (0, 300)]
    ll = [(base_lat + y * ft / 111320.0, base_lon + x * ft / m_lon) for x, y in pts]
    return [ll[i] for i in order]


def test_field_corners_must_go_around_the_edge():
    from flight.pathfinding.mission_planner import validate_field_corners
    validate_field_corners(_corners_latlon([0, 1, 2, 3]))
    try:
        validate_field_corners(_corners_latlon([0, 1, 3, 2]))     # a "Z": bow-tie
    except ValueError as exc:
        assert "swapped" in str(exc), exc
    else:
        raise AssertionError("bow-tie corners were accepted")
    try:
        validate_field_corners(_corners_latlon([0, 3, 2, 1]))     # clockwise
    except ValueError as exc:
        assert "clockwise" in str(exc), exc
    else:
        raise AssertionError("clockwise corners were accepted")


def test_local_frame_is_in_true_feet():
    """A 60 ft wide field is planned 60 ft wide, not stretched to 80."""
    from flight.pathfinding.mission_planner import MissionPlanner
    base_lat, base_lon, ft = 36.0, -95.9, 0.3048
    m_lon = 111320.0 * math.cos(math.radians(base_lat))
    corners = [(base_lat + y * ft / 111320.0, base_lon + x * ft / m_lon)
               for x, y in [(0, 0), (60, 0), (60, 200), (0, 200)]]
    mp = MissionPlanner(corners, corners[0], "bottom", (1,), False)
    assert abs(mp.field_w - 60) < 0.01 and abs(mp.field_h - 200) < 0.01, (mp.field_w, mp.field_h)


def test_latlon_planner_matches_the_local_planner():
    """The flight-code wrapper: lat/lon in, the same side session inside, a full two-sided
    mission through it (one copy per side here, fed both drones' photos), and a mine/join
    round trip."""
    from flight.pathfinding.mission_planner import MissionPlanner
    corners = _corners_latlon([0, 1, 2, 3])
    t = MissionPlanner(corners, corners[0], "bottom", (1,), False)._t
    bottom = MissionPlanner(corners, t.local_to_latlon(40, 0), "bottom", (1, 2), True)
    top = MissionPlanner(corners, t.local_to_latlon(40, 300), "top", (3, 4), True)
    assert abs(bottom.field_h - 300) < 0.5 and bottom.start_if_needed() is True
    assert bottom.start_if_needed() is None
    relays = bottom.own_photo(None, [t.local_to_latlon(41, 101)])
    assert len(relays) == 1 and bottom.own_photo(None, [t.local_to_latlon(41.4, 101.3)]) == []
    top.partner_mine(*relays[0][:2])
    assert bottom.mine_hash() == top.mine_hash()
    pos = {bottom: {1: t.local_to_latlon(40, 0), 2: t.local_to_latlon(40, 0)},
           top: {3: t.local_to_latlon(40, 300), 4: t.local_to_latlon(40, 300)}}
    for _ in range(2000):
        if bottom.done() and top.done():
            break
        bottom.partner_point_a(*top.point_a_latlon())
        top.partner_point_a(*bottom.point_a_latlon())
        for pl, other in ((bottom, top), (top, bottom)):
            first, second = pl.drones
            plan = pl.assignments(pos[pl])
            for who in (first, second):
                for lat, lon in plan[who][:1]:
                    pos[pl][who] = (lat, lon)
                    x, y = pl.latlon_to_local(lat, lon)
                    quad = [pl.local_to_latlon(x + dx, y + dy)
                            for dx, dy in ((-2, -3), (2, -3), (2, 3), (-2, 3))]
                    if who == first:
                        pl.own_footprint(quad)
                        pl.own_photo(quad, [])
                    else:
                        pl.mate_photo(quad, [])
                    other.partner_coverage(quad)
    assert bottom.done() and top.done()
    joined = bottom.join_latlon(top.route_latlon())
    assert joined is not None
    path = [bottom.latlon_to_local(lat, lon) for lat, lon in joined]
    assert path[0][1] < 0.01 and abs(path[-1][1] - bottom.field_h) < 0.01
    assert not bottom.planner.engine.crosses(path)
    bottom.iarc_path(joined)


def test_an_announced_job_is_honoured_by_the_other_copy():
    """Drone 2 announced it holds the forward path. Drone 1's copy, planning with drone 1
    right at its start, still leaves it to drone 2 -- and ties go to the lower id."""
    groups = [[("forward", _line(40, 30, 100)), (("span", (20.0, 60.0)), _line(20, 30, 60))], []]
    pos = {1: (40, 28), 2: (20, 0)}
    job = (0, "forward", (40, 30))
    q = ClaimBook((1, 2)).update(groups, pos, held={2: job})
    assert q[2] and all(x == 40 for x, _y, _g in q[2]), q
    assert q[1] and all(x == 20 for x, _y, _g in q[1]), q
    # The same job arriving as JSON (lists) matches too.
    q = ClaimBook((1, 2)).update(groups, pos, held={2: [0, "forward", [40, 30]]})
    assert all(x == 40 for x, _y, _g in q[2])
    # Both copies think they hold it: the lower id keeps it.
    book = ClaimBook((1, 2))
    book.update(groups, pos)                  # drone 1's own head: the forward path
    q = book.update(groups, pos, held={2: job})
    assert all(x == 40 for x, _y, _g in q[1]), q


def test_work_keys_are_the_same_in_every_copy():
    """Two copies of a side's planner fed the same events give every path the same key --
    what lets a job announced by one drone be honoured by the other's copy."""
    a, b = (_planner([(40, 0), (40, 20), (48, 24), (40, 28), (40, 300)],
                     ["checked", "checked", "checked", "forward"], partner=False)
            for _ in range(2))
    for pl in (a, b):
        pl.add_mine(40.0, 24.0, PAIR)
    keys = [[k for k, _seq in grp] for grp in a.groups()]
    assert keys == [[k for k, _seq in grp] for grp in b.groups()]
    assert any(isinstance(k, tuple) and k[0] == "span" for k in keys[0]), keys


def test_never_loads_the_old_planner():
    """Run missions in a fresh interpreter and prove the path-node planner imports nothing
    from the archived legacy planner (archive/legacy_planner) or the flight state machine."""
    code = (
        "import sys\n"
        "from flight.pathfinding.pathnode.minefield import generate_true_minefield\n"
        "from flight.pathfinding.pathnode.sim import Mission, evaluate\n"
        "for d in (1, 4): evaluate(Mission(generate_true_minefield(90, 3), drones=d).run())\n"
        "names = (k for k in sys.modules if k.startswith(('flight', 'archive', 'state_machine')))\n"
        "print('\\n'.join(sorted(names)))\n"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         check=True).stdout.split()
    banned = ("flight.pathfinder", "flight.pathfinding.nodeField", "archive", "state_machine")
    loaded = [mod for mod in out if mod.startswith(banned)]
    assert not loaded, loaded


def test_walled_in_entry_moves_along_the_edge():
    """Mines ringing the entry without covering it: the entry shifts to where a way out is."""
    pl = PathNodePlanner(80, 300, 41.0)
    for a in range(0, 181, 15):
        pl.add_mine(41 + 9 * math.cos(math.radians(a)), 9 * math.sin(math.radians(a)), PARTNER)
    assert pl.failure is None, pl.failure
    assert pl.nodes[0].x != 41.0 and pl.stats["entry_shifts"] >= 1
    assert _consistent(pl) and _clear(pl)


def test_a_square_that_keeps_being_missed_is_blocked_not_reissued():
    """A photo that never covers its square (dead camera, bad footprint): after MISS_CENTRE
    misses it is retaken centred; after MISS_BLOCK the square becomes an obstacle."""
    pl = PathNodePlanner(80, 300, 41.0)
    x, y, _g = pl.plan({1: pl.point_a()})[1][0]
    square = pl.seen.real_to_cell(x, y)
    for _ in range(MISS_BLOCK):
        assert square not in pl.blocked
        pl.record_failed_photo(x, y)
    assert square in pl.blocked and pl.failure is None
    # Nothing plans over it any more, and the route goes around it.
    assert all(pl.seen.real_to_cell(px, py) != square
               for px, py, _g in pl.plan({1: pl.point_a()})[1])
    assert all(square not in pl.coverage.touched(k.poly) for k in pl.links)


def test_rotated_footprints_still_finish():
    """The real footprint turns with the drone's heading; tiles must still cover their
    squares (the 45 degree case used to re-fly the same tiles forever)."""
    for yaw in (45.0, "random"):
        m = Mission(generate_true_minefield(30, 0), drones=2, photo_yaw_deg=yaw,
                    max_ticks=1200).run()
        e = evaluate(m)
        assert m.failure is None and e["complete"] and e["unseen_cells"] == 0, (yaw, m.failure, e)


def test_crosswise_meeting_is_repaired_by_the_bottom_side():
    """Under heavy loss both sides met points the other had abandoned (seed 100): the halves
    didn't connect. The bottom side re-opens toward the partner's route end."""
    m = Mission(generate_true_minefield(90, 100), drones=4, drop=0.5, delay=3, seed=100).run()
    e = evaluate(m)
    assert m.failure is None and e["complete"] and e["spans_field"], (m.failure, e)
    assert e["bad_edges_known"] == 0 and e["unseen_cells"] == 0, e


def test_check_join_reopens_only_the_bottom_side():
    bottom = _planner([(40, 0), (40, 100), (40, 150)], ["checked", "checked"])
    top = PathNodePlanner(80, 300, 40, Edge.TOP, partner_entry_x=40)
    partner_route = [(40.0, 300.0), (40.0, 160.0)]     # ends short of the bottom's route
    assert bottom.check_join(partner_route) is None and not bottom.met
    assert bottom.nodes[-1].xy == (40.0, 160.0) and _consistent(bottom)
    top.check_join([(40.0, 0.0), (40.0, 50.0)])
    assert top.stats["reopens"] == 0


def test_groups_cache_matches_a_fresh_computation():
    m = Mission(generate_true_minefield(60, 5), drones=2, max_ticks=60).run()
    pl = m.planners[1]
    cached = pl.groups()
    pl.work._groups, pl.work._bands = None, {}       # drop the caches
    assert cached == pl.groups()
    pl.plan({1: (40.0, 50.0), 2: (30.0, 20.0)})
    again = pl.plan({1: (40.0, 50.0), 2: (30.0, 20.0)})
    assert again == pl.plan({1: (40.0, 50.0), 2: (30.0, 20.0)})


def test_kept_routes_are_still_shortest_after_new_mines():
    """The route cache keeps what a new zone doesn't cross (routes.py): those must equal a
    fresh search."""
    m = Mission(generate_true_minefield(90, 7), drones=1).run()
    pl = m.planners[1]
    rng = random.Random(1)
    for _ in range(40):
        a = (rng.uniform(0, 80), rng.uniform(0, 300))
        b = (rng.uniform(0, 80), rng.uniform(0, 300))
        pl.routes.shortest(a, b)
    for k in pl.routes.cached_pairs():
        cached, fresh = pl.routes.shortest(*k), pl.engine.shortest_path(*k)
        assert (cached is None) == (fresh is None), k
        if cached is not None:
            assert abs(poly_len(cached[0]) - poly_len(fresh[0])) < 1e-6, k


def _first_photo(drones):
    pl = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=drones)
    at = pl.point_a()
    return pl, at, pl.plan({d: at for d in drones})[drones[0]][0]


def test_the_sides_keep_apart_by_id():
    """The other side's drones are avoided like a side-mate: the higher id keeps
    keep_away_ft from the lower's position, the lower only keep_clear_ft from the
    higher's; a drone that is stale (left out, or None) is ignored."""
    pl, at, (x, y, _g) = _first_photo((3,))
    cfg = pl.config
    for others in ({}, {1: None}):
        plan = pl.plan({3: at}, others=others)[3]
        assert plan and (plan[0][0], plan[0][1]) == (x, y)
    near = {1: (x + 9.0, y)}        # inside keep_away_ft, outside keep_clear_ft
    plan = pl.plan({3: at}, others=near)[3]
    assert all(math.dist(q[:2], near[1]) >= cfg.keep_away_ft for q in plan)
    assert len(plan) < len(pl.plan({3: at})[3])
    # The lower id (drone 1) is kept away only within keep_clear_ft of the higher one.
    low = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1,))
    full = low.plan({1: at})[1]
    plan = low.plan({1: at}, others={3: (x + 9.0, y)})[1]
    assert plan == full, "keeps only keep_clear_ft from a higher id"
    plan = low.plan({1: at}, others={3: (x + 2.0, y)})[1]
    assert all(math.dist(q[:2], (x + 2.0, y)) >= cfg.keep_clear_ft for q in plan)


def test_a_higher_id_makes_way_for_the_other_sides_job_on_top_of_it():
    """Drone 3 has no photo it may fly and sits within keep_clear_ft of the job drone 1
    announced: it moves off to a spare photo clear of it -- else drone 1 could never fly its
    job. (A drone that still has photos to fly just flies them.)"""
    m = Mission(generate_true_minefield(20, 3), drones=1).run()
    pl, here = m.planners[1], m.pos[1]
    pl.coordinator.drones, pl.coordinator.claims = (3,), ClaimBook((3,))   # this copy is drone 3's
    job = (0, None, (here[0] + 2.0, here[1]))
    others = {1: (here[0] + 12.0, here[1])}
    pl.plan({3: here}, held={1: job}, me=3, others=others)
    spare = pl.last_plan[3]
    assert len(spare) == 1 and spare[0][2] == len(GROUPS) - 1, spare
    assert math.dist(spare[0][:2], job[2]) >= pl.config.keep_clear_ft
    assert math.dist(spare[0][:2], others[1]) >= pl.config.keep_clear_ft


def test_an_idle_lower_id_makes_way_for_the_other_sides_job_on_top_of_it():
    """Drone 1 has nothing to do and drone 3 announced a job within keep_clear_ft of it: drone 3
    can't fly a photo next to it, so drone 1 moves away."""
    m = Mission(generate_true_minefield(20, 3), drones=1).run()
    pl = m.planners[1]
    here = m.pos[1]
    assert pl.done()
    job = (0, None, (here[0] + 2.0, here[1]))
    pl.plan({1: here}, held={3: job}, me=1, others={3: (here[0] + 8.0, here[1])})
    spare = pl.last_plan[1]
    assert len(spare) == 1 and math.dist(spare[0][:2], job[2]) >= pl.config.keep_clear_ft, spare
    pl.plan({1: here}, me=1, others={3: (here[0] + 8.0, here[1])})
    assert pl.last_plan[1] == [], "no announced job on top of it: it stays put"


def _two_copies():
    """Drone 1 at point A on a fresh field with its announced job (the forward path), and a
    fresh copy of the planner for drone 2 (as if it had just started)."""
    one = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1, 2))
    at = one.point_a()
    far = (at[0], at[1] + 60.0)
    one.plan({1: at, 2: far}, me=1)
    job = one.claims.heads[1]
    assert job is not None and job[1] == "forward", job
    two = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1, 2))
    path = [tuple(p) for k, seq in two.groups()[0] if k == "forward" for p in seq]
    assert len(path) > 10
    return two, at, far, job, path


def test_idle_drone_takes_no_photo_of_a_sequence_the_other_announced():
    """Drone 1 announced the forward path: drone 2 (nothing else to do) must not fly any photo
    of it -- both drones flying the same path is the waste the takeover must not cause."""
    two, at, far, job, path = _two_copies()
    plan = two.plan({1: at, 2: far}, held={1: list(job[:2]) + [list(job[2])]}, me=2)
    assert not [q for q in plan[2] if q[:2] in path], plan[2][:3]
    # Nothing announced (the other copy may not have the work): it is taken over.
    two2, at, far, job, path = _two_copies()
    plan = two2.plan({1: at, 2: far}, held={1: None}, me=2)
    assert any(q[:2] in path for q in plan[2])


def test_a_lost_mate_keeps_blocking_its_last_position_and_job():
    """Drone 2 goes silent while flying: drone 1 takes over its work but never plans a photo
    within keep_away/keep_clear of its last position or job, nor any photo of the sequence it
    announced (it is probably still flying it). Drone 2 the same for a lost drone 1."""
    cfg = PathNodePlanner(80, 300, 40).config
    two, at, far, job, path = _two_copies()
    held = {1: [job[0], job[1], list(job[2])]}
    plan = two.plan({1: at, 2: far}, held=held, me=2, lost={1})
    assert not [q for q in plan[2] if q[:2] in path]
    assert all(math.dist(q[:2], at) >= cfg.keep_away_ft and math.dist(q[:2], job[2]) >= cfg.keep_away_ft
               for q in plan[2])
    # Gone (a long silence): ignored entirely, its work is taken over.
    plan = two.plan({1: None, 2: far}, held={}, me=2)
    assert any(q[:2] in path for q in plan[2])
    # Higher-id mate lost: drone 1 keeps keep_clear from its last position.
    one = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1, 2))
    a1 = one.point_a()
    last2 = (a1[0] + 5.0, a1[1] + 4.0)
    j2 = (0, "forward", (a1[0] + 5.0, a1[1] + 8.0))
    plan = one.plan({1: a1, 2: last2}, held={2: j2}, me=1, lost={2})
    assert all(math.dist(q[:2], last2) >= cfg.keep_clear_ft for q in plan[1])
    assert not [q for q in plan[1] if q[2] == 0], "the announced forward path stays reserved"


def test_a_mate_heard_again_causes_no_duplicate_work():
    """Drone 2 was lost while drone 1 took over its queue; once it is heard again both copies
    agree on who holds what, and no photo is queued for both."""
    one = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1, 2))
    two = PathNodePlanner(80, 300, 40, Edge.BOTTOM, drones=(1, 2))
    at = one.point_a()
    pos = {1: at, 2: (at[0], at[1] + 60.0)}
    q2 = two.plan(pos, me=2)
    job2 = two.claims.heads[2]
    lost = one.plan(pos, held={2: job2}, me=1, lost={2})
    pos = {1: lost[1][0][:2] if lost[1] else at, 2: pos[2]}
    for _ in range(3):
        p1 = one.plan(pos, held={2: job2}, me=1)
        p2 = two.plan(pos, held={1: one.claims.heads[1]}, me=2)
    a = {q[:2] for q in p1[1]}
    b = {q[:2] for q in p2[2]}
    assert not (a & b), sorted(a & b)[:3]


def test_keep_clear_is_above_the_flight_separation_floor():
    from flight.waypoint import COLLISION_RADIUS, SEPARATION_ABORT_M
    cfg = PathNodePlanner(80, 300, 40).config
    assert cfg.keep_clear_ft > max(SEPARATION_ABORT_M, COLLISION_RADIUS) / 0.3048
    assert cfg.keep_away_ft > cfg.keep_clear_ft


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}", flush=True)
    print(f"\n{len(tests)}/{len(tests)} passed")
