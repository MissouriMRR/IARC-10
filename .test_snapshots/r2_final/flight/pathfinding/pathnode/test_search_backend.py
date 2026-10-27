"""Conformance suite for SearchBackend implementations. Plain asserts -- runs as a script:

    PYTHONPATH="$PWD" python -m flight.pathfinding.pathnode.test_search_backend

Every test runs against EVERY entry of search_backend.BACKENDS, in local feet. It checks the
contract written in flight/pathfinding/search_backend.py, not equality between backends. A
genuine gap goes in KNOWN_GAPS ((test name, backend name) -> message): it is reported as
KNOWN GAP when it fails, never skipped.
"""
import json

from flight.pathfinding.search_backend import BACKENDS, Outbox, make_backend

W, H = 40.0, 60.0
ENTRY = 20.0
KNOWN_GAPS = {}
HALF_X, HALF_Y = 2.0, 3.0           # the photo footprint: shape_size_ft (along y 6, across x 4)


def _make(name, edge="bottom", drones=(1,), partner=False, entry=ENTRY):
    return make_backend(name, W, H, entry, edge, drones, partner,
                        shape_size_ft=(2 * HALF_Y, 2 * HALF_X), overlap=0.1)


def _footprint(x, y):
    return [(x - HALF_X, y - HALF_Y), (x + HALF_X, y - HALF_Y),
            (x + HALF_X, y + HALF_Y), (x - HALF_X, y + HALF_Y)]


def _found(x, y, truth):
    return [m for m in truth if abs(m[0] - x) <= HALF_X and abs(m[1] - y) <= HALF_Y]


def _fly_one(b, truth=(), limit=400):
    """One drone, alone: photograph what the backend asks for until it is done."""
    pos = b.point_a()
    for _ in range(limit):
        if b.done() or b.failure:
            return
        queue = b.assignments({1: pos}, me=1)[1]
        assert queue, "not done, yet nothing to photograph"
        x, y, _g = queue[0]
        pos = (x, y)
        quad = _footprint(x, y)
        b.own_footprint(quad)
        b.own_photo(quad, _found(x, y, truth))
    raise AssertionError("did not finish")


def test_empty_field_gives_a_done_verified_straight_route(name):
    b = _make(name)
    assert b.failure is None and b.drones == (1,) and not b.has_partner
    assert not b.done() and not b.route_verified()
    assert b.point_a() == (ENTRY, 0.0)
    _fly_one(b)
    assert b.done() and b.route_verified() and b.met and b.failure is None
    route = b.route()
    assert route[0] == (ENTRY, 0.0) and abs(route[-1][1] - H) < 1e-6, route
    assert all(abs(x - ENTRY) < 1e-6 for x, _y in route), route
    assert b.mines() == [] and not b.crosses(route)
    assert b.cell_seen_at(ENTRY, 10.0) is True and b.cell_seen_at(-5.0, 10.0) is None


def test_a_mine_found_by_own_photo_is_relayed_once_and_known(name):
    b = _make(name, partner=True)
    x, y, _g = b.assignments({1: b.point_a()}, me=1)[1][0]
    mine = (x, y + 0.5)
    quad = _footprint(x, y)
    b.own_footprint(quad)
    out = b.own_photo(quad, [mine])
    assert isinstance(out, Outbox) and out.mines == [mine], out
    assert out.point_a is not None                    # it has a partner: point A goes along
    assert b.mines() == [mine] and b.mine_hash()
    again = b.own_photo(None, [(mine[0] + 0.3, mine[1] - 0.2)])      # the same square
    assert again.mines == [] and b.mines() == [mine]
    assert not b.crosses(b.route())


def test_the_mine_hash_tracks_the_known_mines(name):
    a, b = _make(name), _make(name)
    empty = a.mine_hash()
    assert empty == b.mine_hash()
    a.partner_mine(12.0, 31.0)
    assert a.mine_hash() != empty and a.mine_hash() != b.mine_hash()
    b.partner_mine(12.4, 31.3)                       # same square: the same mine
    assert a.mine_hash() == b.mine_hash() and len(b.mines()) == 1


def _drive_pair(name, truth, rounds=600):
    """Two side-mates, each with its own copy, driven like the flight code: each plans for
    itself (me=...) with the job the other announced, flies its own head photo, and every
    photo reaches both copies (own -> own_photo, the mate's -> mate_photo).
    Returns (copy 1, copy 2, the photos taken as [(quad, mines)])."""
    c1, c2 = _make(name, drones=(1, 2)), _make(name, drones=(1, 2))
    pos = {1: c1.point_a(), 2: c2.point_a()}
    photos = []
    for _ in range(rounds):
        if c1.done() and c2.done():
            break
        p1 = c1.assignments(dict(pos), {2: c2.current_job(2)}, 1)
        p2 = c2.assignments(dict(pos), {1: c1.current_job(1)}, 2)
        acted = False
        for d, own, mate, plan in ((1, c1, c2, p1), (2, c2, c1, p2)):
            if not plan[d]:
                continue
            x, y, _g = plan[d][0]
            pos[d] = (x, y)
            quad, mines = _footprint(x, y), _found(x, y, truth)
            own.own_footprint(quad)
            own.own_photo(quad, mines)
            mate.mate_photo(quad, mines)
            photos.append((quad, mines))
            acted = True
        assert acted or (c1.done() and c2.done()), "stalled"
    assert c1.done() and c2.done()
    return c1, c2, photos


def test_two_copies_fed_the_same_events_agree(name):
    c1, c2, photos = _drive_pair(name, [(19.0, 20.5), (21.5, 41.0)])
    assert c1.mine_hash() == c2.mine_hash() and len(c1.mines()) >= 1
    assert c1.route() == c2.route() and c1.route_verified() and not c1.crosses(c1.route())
    # Two fresh copies given the same photos (one as its own, one as the mate's) plan alike,
    # part-way through and at the end.
    for upto in (len(photos) // 2, len(photos)):
        a, b = _make(name, drones=(1, 2)), _make(name, drones=(1, 2))
        for quad, mines in photos[:upto]:
            a.own_footprint(quad)
            a.own_photo(quad, mines)
            b.mate_photo(quad, mines)
        pos = {1: a.point_a(), 2: a.point_a()}
        assert a.mine_hash() == b.mine_hash() and a.route() == b.route()
        assert a.assignments(dict(pos)) == b.assignments(dict(pos))
        assert a.current_job(1) == b.current_job(1) and a.current_job(2) == b.current_job(2)


def test_current_job_matches_the_head_of_assignments(name):
    c1, c2 = _make(name, drones=(1, 2)), _make(name, drones=(1, 2))
    pos = {1: c1.point_a(), 2: c1.point_a()}
    checked = False
    for _ in range(600):
        if c1.done():
            break
        plan = c1.assignments(dict(pos))
        for d in (1, 2):
            assert (c1.current_job(d) is not None) == bool(plan[d]), (d, plan)
        if plan[1] and plan[2] and not checked:
            # A job survives the wire (JSON) and, held by its owner, leaves the plan unchanged.
            job2 = json.loads(json.dumps(c1.current_job(2)))
            again = c2_copy_plan(c2, pos, job2)
            assert again == plan, (again, plan)
            gone = c1.assignments({1: pos[1], 2: None})
            assert gone[1] and not gone[2]       # a drone that is gone gets no work
            checked = True
        for d, own, mate in ((1, c1, c2), (2, c2, c1)):
            if plan[d]:
                x, y, _g = plan[d][0]
                pos[d] = (x, y)
                quad = _footprint(x, y)
                own.own_footprint(quad)
                own.own_photo(quad, [])
                mate.mate_photo(quad, [])
        c2.assignments(dict(pos))                # keep the mate's copy in step
    assert checked, "the two drones never both had work"


def c2_copy_plan(c2, pos, job2):
    """What a copy plans when drone 2 announces `job2` (a copy that has seen the same photos)."""
    return c2.assignments(dict(pos), held={2: job2})


def _walled_off(name, partner_mines=False):
    b = _make(name)
    wall = [(x + 0.5, 6.5) for x in range(0, int(W), 2)]
    if partner_mines:
        for m in wall:
            b.partner_mine(*m)
    else:
        b.own_photo(None, wall)
    return b


def test_a_walled_off_entry_sets_failure(name):
    for partner_mines in (False, True):
        b = _walled_off(name, partner_mines)
        assert b.failure, "a wall across the field left a route"
        assert not b.met and not b.route_verified()
        assert b.assignments({1: (ENTRY, 0.0)}, me=1)[1] == []


def _drive_two_sided(name, truth, rounds=800):
    """A bottom side (drone 1) and a top side (drone 3), each copy told what the flight code
    relays: point A, coverage, mines. Returns (bottom, top)."""
    bottom = _make(name, "bottom", (1,), True)
    top = _make(name, "top", (3,), True)
    pos = {1: bottom.point_a(), 3: top.point_a()}
    for _ in range(rounds):
        if bottom.done() and top.done():
            break
        bottom.partner_point_a(*top.point_a())
        top.partner_point_a(*bottom.point_a())
        for me, own, other in ((1, bottom, top), (3, top, bottom)):
            queue = own.assignments({me: pos[me]}, me=me)[me]
            if not queue:
                continue
            x, y, _g = queue[0]
            pos[me] = (x, y)
            quad, mines = _footprint(x, y), _found(x, y, truth)
            own.own_footprint(quad)
            out = own.own_photo(quad, mines)
            for m in out.mines:
                other.partner_mine(*m)
            other.partner_coverage(quad)
    assert bottom.done() and top.done(), (bottom.failure, top.failure)
    return bottom, top


def test_join_never_returns_a_route_crossing_a_known_mine(name):
    bottom, top = _drive_two_sided(name, [(18.0, 30.5), (26.0, 22.0)])
    joined = bottom.join(top.route())
    assert joined is not None and not bottom.crosses(joined)
    assert abs(joined[0][1]) < 1e-6 and abs(joined[-1][1] - H) < 1e-6
    assert bottom.mine_hash() == top.mine_hash() and len(bottom.mines()) == 2
    # A partner route straight through a known mine is never handed back as a join.
    mx, my = bottom.mines()[0]
    bad = [bottom.point_a(), (mx, my), (mx, H)]
    for route in (bad, list(reversed(bad)), top.route()[:2]):
        j = bottom.join(route)
        assert j is None or not bottom.crosses(j), j
        j = bottom.partner_route(route)
        assert j is None or not bottom.crosses(j), j
    j = bottom.partner_route(top.route())
    assert j is not None and not bottom.crosses(j)


def test_stats_have_the_documented_shape(name):
    assert isinstance(_make(name).stats(), dict)


# -- expanding mines: levels, snapshots, lockstep -----------------------------------------------
def test_a_fresh_backend_is_at_level_zero_with_nothing_to_report(name):
    b = _make(name)
    assert b.level == 0 and b.valid_snapshots() == [] and not b.report_valid()
    state = b.level_state(1)
    assert (state.level, state.ready, state.capped, state.drone) == (0, False, False, 1)
    assert b.level_state(1) == state                       # repeatable, same seq
    assert b.report_route() == b.route() and b.report_route(0) is None
    assert isinstance(b.buffer_width(), int) and b.growth_fits(0.0)
    b.hear_level_state(state)                              # harmless, even one's own
    b.partner_lost()                                       # harmless without a partner
    assert b.try_advance_level(1).step.value == "not_ready"


def test_the_mine_hash_includes_the_level(name):
    a, b = _make(name), _make(name)
    _fly_one(a)
    _fly_one(b)
    assert a.mine_hash() == b.mine_hash()
    a.try_advance_level(1)
    assert a.level == 1 and a.mine_hash() != b.mine_hash()


def test_a_finished_lone_copy_snapshots_grows_and_reports_the_latest_valid_one(name):
    b = _make(name)
    truth = [(18.0, 21.0), (23.0, 35.0)]
    widths = []
    for k in range(3):
        _fly_one(b, truth)
        step = b.try_advance_level(1, now=float(k))
        assert step.step.value == "grew" and step.level == b.level == k + 1, step
        snaps = b.valid_snapshots()
        assert len(snaps) == k + 1 and snaps[-1].level == k and snaps[-1].t == float(k)
        assert b.report_valid() and b.report_route() == list(snaps[-1].route)
        assert b.report_route(k) == list(snaps[-1].route)
        widths.append(b.buffer_width())
        assert not b.level_state(1).ready                  # new level, new work
    assert widths == sorted(widths) and widths[-1] > widths[0]   # more levels, more clearance


def test_the_two_sides_climb_in_lockstep_and_join_at_a_common_level(name):
    bottom, top = _drive_two_sided(name, [(18.0, 30.5), (26.0, 22.0)])
    s1 = bottom.try_advance_level(1, now=1.0)
    assert s1.step.value == "waiting"                      # nothing heard from the other side
    s3 = top.try_advance_level(3, now=1.0)
    assert s3.step.value == "waiting"
    bottom.hear_level_state(top.level_state(3))
    top.hear_level_state(bottom.level_state(1))
    assert bottom.try_advance_level(1, now=2.0).step.value == "grew"
    assert top.try_advance_level(3, now=2.0).step.value == "grew"
    assert bottom.level == top.level == 1
    old = top.level_state(3)
    top.hear_level_state(bottom.level_state(1))
    bottom.hear_level_state(old)                           # a repeat or older one: ignored
    base0 = bottom.valid_snapshots()[0]
    top0 = top.valid_snapshots()[0]
    joined = bottom.join_at(0, top0.route, top0.mine_hash)
    assert joined is not None and joined[0] == base0.route[0]
    assert abs(joined[-1][1] - H) < 1e-6 or abs(joined[-1][1]) < 1e-6
    assert bottom.join_at(5, top0.route) is None           # no snapshot at that level


def test_a_lost_partner_is_not_waited_for(name):
    bottom, _top = _drive_two_sided(name, [])
    assert bottom.try_advance_level(1, now=1.0).step.value == "waiting"
    bottom.partner_lost()
    assert bottom.try_advance_level(1, now=2.0).step.value == "grew"


def test_the_growth_time_guard_follows_the_last_measured_growth(name):
    b = _make(name)
    _fly_one(b)
    assert b.growth_fits(0.0)                              # nothing measured yet
    step = b.try_advance_level(1)
    assert step.step.value == "grew" and step.seconds > 0
    assert b.growth_fits(step.seconds * 1.5 + 1e-9) and not b.growth_fits(step.seconds)
    _fly_one(b)
    held = b.try_advance_level(1, max_level=b.level)       # the caller says "no more growth"
    assert held.step.value == "at_cap" and b.level == 1


def _run():
    tests = [(k, v) for k, v in dict(globals()).items() if k.startswith("test_")]
    passed, gaps, failed = 0, 0, 0
    for name in BACKENDS:
        for tname, t in tests:
            try:
                t(name)
                passed += 1
                print(f"PASS {name:10s} {tname}", flush=True)
            except Exception as err:       # noqa: BLE001 -- report every backend's every check
                if (tname, name) in KNOWN_GAPS:
                    gaps += 1
                    print(f"KNOWN GAP {name:10s} {tname}: {KNOWN_GAPS[(tname, name)]}"
                          f" ({err!r})", flush=True)
                else:
                    failed += 1
                    print(f"FAIL {name:10s} {tname}: {err!r}", flush=True)
    print(f"\n{passed} passed, {gaps} known gaps, {failed} failed "
          f"({len(BACKENDS)} backends x {len(tests)} checks)")
    return failed


if __name__ == "__main__":
    raise SystemExit(1 if _run() else 0)
