"""Tests for the expanding-mines phase: snapshots, the lockstep gate, planner growth, the sim.

Plain asserts -- runs under pytest or as a script:
    PYTHONPATH="$PWD" python -m flight.pathfinding.pathnode.test_levels
Planner cases use a small field (40 x 60 ft) and both engines where it is cheap.
"""
import dataclasses

from flight.pathfinding.levels import (LevelGate, LevelState, Snapshot, SnapshotBook, Step,
                                       Verdict, clearance)
from flight.pathfinding.pathnode.config import Edge, Source
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.sim import Mission, evaluate
from flight.pathfinding.pathnode.target import FarEdgeTarget, LostPartnerTarget

ENGINES = ("zonemap", "field")
FW, FH = 40.0, 60.0


def _sq(c, r):
    """The centre (x, y) of square (c, r), where a mine on that square is placed."""
    return (c * 2.0 + 1.0, r * 2.0 + 1.0)


def _snap(level, seq, cells, hash_="h", spans=True):
    pts = tuple((c * 2.0 + 1.0, r * 2.0 + 1.0) for c, r in cells)
    return Snapshot(level, seq, pts, tuple(cells), f"{hash_}{level}", float(seq), spans)


COLUMN = [(5, r) for r in range(10)]            # a path straight up column 5


# -- Snapshot, SnapshotBook ---------------------------------------------------------------------
def test_a_snapshot_is_immutable():
    snap = _snap(0, 0, COLUMN)
    for field in ("level", "route", "cells", "mine_hash"):
        try:
            setattr(snap, field, 1)
        except dataclasses.FrozenInstanceError:
            continue
        raise AssertionError(f"{field} was changed")
    assert isinstance(snap.route, tuple) and isinstance(snap.cells, tuple)
    book = SnapshotBook(20, 30)
    book.add(snap)
    book.refresh(lambda c, r: r < 4)                # the counter moves; the snapshot does not
    assert book.unsafe(snap) == 4 and book.snapshots[0] is snap
    book.refresh(lambda c, r: False)                # zones shrank again
    assert book.unsafe(snap) == 0
    assert snap == _snap(0, 0, COLUMN)


def test_a_mine_on_the_path_makes_a_snapshot_invalid_but_one_in_the_buffer_does_not():
    book = SnapshotBook(20, 30)
    snap = _snap(0, 0, COLUMN)
    book.add(snap)
    assert book.valid(snap, [])
    assert not book.valid(snap, [_sq(5, 4)])        # on a path square
    assert not book.valid(snap, [(11.9, 8.1)])      # the same square, anywhere inside it
    assert book.valid(snap, [_sq(6, 4)])            # right beside it: in the buffer
    assert book.valid(snap, [_sq(8, 4), _sq(2, 7)])


def test_the_latest_valid_snapshot_is_chosen():
    book = SnapshotBook(20, 30)
    shifted = [(7, r) for r in range(10)]
    a, b, c = _snap(0, 0, COLUMN), _snap(1, 1, shifted), _snap(1, 2, COLUMN, spans=False)
    for s in (a, b, c):
        book.add(s)
    assert book.latest_valid([]) is c
    assert book.latest_valid([_sq(5, 3)]) is b      # c and a are hit; b is not
    assert book.latest_valid([_sq(5, 3), _sq(7, 3)]) is None
    assert book.latest_valid([], level=0) is a and book.latest_valid([], level=2) is None
    assert book.latest_valid([], spans=True) is b
    assert book.valid_snapshots([_sq(7, 3)]) == [a, c]


def test_buffer_width_on_hand_made_cases():
    book = SnapshotBook(20, 30)
    snap = _snap(0, 0, COLUMN)
    assert book.buffer_width(snap, []) == 50                      # nothing known: the field
    assert book.buffer_width(snap, [_sq(6, 4)]) == 0              # next to the path
    assert book.buffer_width(snap, [_sq(8, 4)]) == 2              # three squares away
    assert book.buffer_width(snap, [_sq(7, 12)]) == 4             # past the end: 2 across + 3 up = 5, minus 1
    assert book.buffer_width(snap, [_sq(5, 4)]) == -1             # on it
    assert book.buffer_width(snap, [_sq(9, 4), _sq(5, 14)]) == 3  # the nearest mine counts
    assert clearance([(0, 0), (1, 0)], [_sq(1, 2)], 99) == 1      # a bend: city block, not straight


# -- LevelState, LevelGate ----------------------------------------------------------------------
def _st(drone, level, ready=True, capped=False, side="bottom", seq=1):
    return LevelState(side, drone, level, ready, capped, seq)


def test_a_level_state_survives_the_wire():
    s = _st(3, 2, side="top", seq=7)
    assert LevelState.from_dict(s.to_dict()) == s


def _verdict(own, heard=(), side=(1, 2), partner=True, lost=(), partner_gone=False):
    gate = LevelGate(side, partner)
    for h in heard:
        assert gate.hear(h)
    if partner_gone:
        gate.partner_lost()
    return gate.check(own, lost)


def test_the_gate_truth_table():
    me = _st(1, 0)
    mate_ready, mate_not = _st(2, 0), _st(2, 0, ready=False)
    mate_past = _st(2, 1, ready=False)
    other_ready, other_not = _st(3, 0, side="top"), _st(3, 0, ready=False, side="top")
    other_past = _st(3, 1, ready=False, side="top")
    go = Verdict.GO
    assert _verdict(me, [mate_ready, other_ready]) is go
    assert _verdict(_st(1, 0, ready=False), [mate_ready, other_ready]) is Verdict.NOT_READY
    assert _verdict(_st(1, 0, capped=True), [mate_ready, other_ready]) is Verdict.AT_CAP
    assert _verdict(me, [mate_not, other_ready]) is Verdict.WAIT_MATE
    assert _verdict(me, [other_ready]) is Verdict.WAIT_MATE             # mate not heard: unknown
    assert _verdict(me, [mate_ready, other_not]) is Verdict.WAIT_OTHER
    assert _verdict(me, [mate_ready]) is Verdict.WAIT_OTHER
    assert _verdict(me, [mate_past, other_past]) is go                  # already past
    assert _verdict(me, [mate_ready, other_past]) is go
    assert _verdict(_st(1, 1), [mate_ready, other_ready]) is Verdict.WAIT_MATE   # they are behind
    assert _verdict(me, [mate_ready, _st(3, 0, capped=True, side="top")]) is Verdict.PEER_CAPPED
    assert _verdict(me, [_st(2, 0, capped=True), other_ready]) is Verdict.PEER_CAPPED


def test_the_gate_with_a_lost_mate_a_lost_partner_and_no_partner():
    me = _st(1, 0)
    other_ready = _st(3, 0, side="top")
    assert _verdict(me, [other_ready], lost=(2,)) is Verdict.GO         # the mate is silent
    assert _verdict(me, [_st(2, 0, ready=False)], partner_gone=True) is Verdict.WAIT_MATE
    assert _verdict(me, [_st(2, 0)], partner_gone=True) is Verdict.GO   # no one to wait for
    assert _verdict(me, [_st(2, 0), _st(3, 0, capped=True, side="top")],
                    partner_gone=True) is Verdict.GO                    # a lost side's cap is stale
    assert _verdict(me, [], side=(1,), partner=False) is Verdict.GO     # a lone drone
    assert _verdict(me, [_st(2, 0)], partner=False) is Verdict.GO       # two drones, no partner
    two = LevelGate((1, 2), True)
    assert two.hear(_st(3, 0, seq=5)) and not two.hear(_st(3, 1, seq=5)) and not two.hear(_st(3, 1, seq=4))
    assert two.hear(_st(3, 1, seq=6)) and two.heard[3].level == 1       # newest by seq wins


def test_the_gate_is_deterministic_and_order_free():
    facts = [_st(2, 0), _st(3, 0, side="top"), _st(4, 0, ready=False, side="top")]
    first = _verdict(_st(1, 0), facts)
    assert first is Verdict.GO
    assert all(_verdict(_st(1, 0), list(reversed(facts))) is first for _ in range(3))


# -- the planner --------------------------------------------------------------------------------
def _planner(engine, edge=Edge.BOTTOM, partner=False, drones=(1,), entry=20.0):
    return PathNodePlanner(FW, FH, entry, edge, partner_entry_x=entry if partner else None,
                           drones=drones, engine=engine)


def _photo(pl, x, y, truth):
    """Photograph the planning footprint at (x, y); add the true mines in it."""
    hx, hy = pl.config.shape_size_ft[1] / 2, pl.config.shape_size_ft[0] / 2
    quad = [(x - hx, y - hy), (x + hx, y - hy), (x + hx, y + hy), (x - hx, y + hy)]
    pl.record_footprint(quad)
    for m in truth:
        if abs(m[0] - x) <= hx and abs(m[1] - y) <= hy:
            pl.add_mine(*m, Source.PAIR)
    pl.advance()


def _work(pl, truth=(), limit=3000):
    """One drone alone does the planner's work until it is ready (or has nothing to fly)."""
    pos = pl.point_a()
    for _ in range(limit):
        if pl.levels.ready():
            return
        queue = pl.plan({1: pos}, me=1)[1]
        assert queue, "not ready, yet nothing to photograph"
        pos = queue[0][:2]
        _photo(pl, *pos, truth)
    raise AssertionError("never became ready")


def _wall(x_centres, row=15):
    return [_sq(c, row) for c in x_centres]


def test_ready_takes_a_snapshot_once_and_only_when_ready():
    for engine in ENGINES:
        pl = _planner(engine)
        assert not pl.levels.ready() and pl.levels.snapshot(0.0) is None
        _work(pl)
        snap = pl.levels.snapshot(3.0)
        assert snap is not None and snap.level == 0 and snap.t == 3.0 and snap.spans
        assert snap.route == tuple(pl.route()) and snap.mine_hash == pl.level_hash()
        assert pl.levels.snapshot(9.0) is snap and len(pl.levels.book) == 1
        assert pl.levels.is_snapshotted() and pl.levels.state(1).ready


def test_not_ready_while_important_work_is_left_but_outer_alone_does_not_matter():
    pl = _planner("zonemap")
    _work(pl)
    assert pl.work.groups()[4], "the demo needs OUTER work left"   # OUTER photos remain
    assert pl.levels.ready()
    x, y = pl.chain.nodes[3].xy
    pl.add_mine(x + 4.0, y + 2.0, Source.PARTNER)                # new pending work near the route
    assert not pl.levels.ready() or pl.levels.snapshot(1.0) is not None


def test_growth_rebridges_and_the_next_snapshot_is_one_level_up():
    truth = [(18.0, 21.0), (23.0, 37.0), (14.0, 45.0)]
    for engine in ENGINES:
        pl = _planner(engine)
        _work(pl, truth)
        first = pl.levels.snapshot(1.0)
        step = pl.levels.grow()
        assert step.step is Step.GREW and pl.level == 1 and step.seconds > 0
        assert not pl.levels.ready() or pl.levels.snapshot(2.0).level == 1
        assert not pl.engine.crosses(pl.route()) and pl.levels.last_seconds == step.seconds
        pl.chain.check()
        _work(pl, truth)
        second = pl.levels.snapshot(2.0)
        assert second.level == 1 and second is not first and second.seq == first.seq + 1
        assert second.mine_hash != first.mine_hash
        assert pl.levels.book.snapshots[0] is first and first.level == 0   # the old one stands
        assert pl.levels.report_route() == list(second.route)
        assert not pl.engine.crosses(second.route)


def test_the_mine_hash_differs_by_level_and_not_by_the_order_mines_arrive():
    a, b = _planner("zonemap"), _planner("zonemap")
    mines = [(12.0, 31.0), (25.0, 40.0), (30.0, 20.0)]
    for m in mines:
        a.add_mine(*m, Source.PARTNER)
    for m in reversed(mines):
        b.add_mine(*m, Source.PARTNER)
    assert a.level_hash() == b.level_hash() and a.mine_hash() == b.mine_hash()
    before = a.level_hash()
    a.routes.set_level(1)
    assert a.level_hash() != before and a.mine_hash() == b.mine_hash()
    b.routes.set_level(1)
    assert a.level_hash() == b.level_hash()


def _walled(engine, gap_mines):
    """Known mines at row 15 (squares given by column)."""
    pl = _planner(engine)
    for c in gap_mines:
        pl.add_mine(*_sq(c, 15), Source.PARTNER)
    return pl


def test_an_infeasible_level_caps_and_keeps_reporting_the_old_snapshot():
    for engine in ENGINES:
        # mines at x = 3, 17, 31, 37 (squares 1, 8, 15, 18): a wall with 4 ft gaps at level 0
        # that the zones of level 1 (7 ft each way) close
        pl = _walled(engine, (1, 8, 15, 18))
        assert pl.failure is None
        _work(pl)
        first = pl.levels.snapshot(1.0)
        assert first is not None
        step = pl.levels.grow()
        assert step.step is Step.CAPPED and step.reason and pl.level == 0
        assert pl.failure is None and pl.levels.cap == 0 and pl.levels.capped
        assert pl.route() == list(first.route) and not pl.engine.crosses(pl.route())
        assert len(pl.levels.book) == 1 and pl.levels.report_route() == list(first.route)
        assert pl.levels.grow().step is Step.AT_CAP
        assert pl.levels.is_snapshotted() and pl.levels.state(1).capped


def test_a_late_mine_on_the_route_invalidates_the_snapshot_and_the_route_is_redone():
    for engine in ENGINES:
        pl = _planner(engine)
        _work(pl)
        first = pl.levels.snapshot(1.0)
        x, y = first.route[len(first.route) // 2]
        late = (x, 30.5)
        assert pl.add_mine(*late, Source.PARTNER)
        assert pl.levels.valid_snapshots() == [] and not pl.levels.report_valid()
        assert pl.levels.book.unsafe(first) == 0 or pl.levels.book.unsafe(first) > 0
        assert not pl.levels.ready()
        assert pl.levels.report_route() != list(first.route) or not pl.levels.report_valid()
        assert pl.levels.buffer_width() >= -1
        _work(pl, [late])
        second = pl.levels.snapshot(2.0)
        assert second.seq == 1 and second.level == 0
        assert pl.levels.valid_snapshots() == [second] and pl.levels.report_valid()
        assert pl.levels.report_route() == list(second.route) and pl.levels.buffer_width() >= 0


def test_a_mine_in_the_buffer_keeps_the_snapshot_valid_and_narrows_the_buffer():
    pl = _planner("zonemap")
    _work(pl)
    snap = pl.levels.snapshot(1.0)
    wide = pl.levels.buffer_width()
    x, y = snap.route[len(snap.route) // 2]
    pl.add_mine(x + 6.0, 31.0, Source.PARTNER)
    assert pl.levels.valid_snapshots() == [snap] and pl.levels.report_valid()
    assert pl.levels.buffer_width() < wide and pl.levels.buffer_width() >= 0


def test_the_unsafe_counter_counts_squares_the_growth_put_in_a_zone():
    pl = _planner("field")
    pl.add_mine(*_sq(10, 15), Source.PARTNER)
    _work(pl)
    snap = pl.levels.snapshot(1.0)
    assert pl.levels.book.unsafe(snap) == 0
    pl.levels.grow()
    assert pl.levels.book.unsafe(snap) > 0           # the old route now touches the wider zone
    pl.levels.rollback(0)
    assert pl.levels.book.unsafe(snap) == 0


def _at_level_one(engine):
    pl = _walled(engine, (1, 7, 19))                # a wide gap in the middle at both levels
    _work(pl)
    assert pl.levels.snapshot(1.0).level == 0
    assert pl.levels.grow().step is Step.GREW
    _work(pl)
    assert pl.levels.snapshot(2.0).level == 1
    return pl


def test_a_late_mine_that_leaves_no_route_at_level_one_falls_back_to_level_zero():
    for engine in ENGINES:
        recovered = 0
        for col in range(9, 19):                    # a mine somewhere in the gap
            pl = _at_level_one(engine)
            pl.add_mine(*_sq(col, 15), Source.PARTNER)
            if pl.level == 1:
                assert pl.failure is None           # the gap is still open at level 1
                continue
            if pl.failure:
                continue                            # shut at level 0 too: a real failure
            recovered += 1
            assert pl.level == 0 and pl.levels.cap == 0 and pl.met
            assert not pl.engine.crosses(pl.route())
            pl.chain.check()
            _work(pl, [])
            assert pl.levels.snapshot(3.0).level == 0
        assert recovered, engine


def test_a_failure_at_level_zero_is_still_a_failure():
    pl = _planner("zonemap")
    for c in range(0, 20, 2):
        pl.add_mine(*_sq(c, 15), Source.PARTNER)
    assert pl.failure


def test_the_time_guard_uses_the_last_measured_growth():
    pl = _planner("field")
    _work(pl)
    assert pl.levels.growth_fits(0.0)
    pl.levels.grow()
    last = pl.levels.last_seconds
    assert pl.levels.growth_fits(1.5 * last) and not pl.levels.growth_fits(1.4 * last)


# -- a lost partner -----------------------------------------------------------------------------
def test_a_lost_partner_target_goes_to_point_a_then_to_the_far_edge():
    pl = _planner("zonemap", partner=True)
    pl.set_partner_point_a(20.0, 30.0)
    assert pl.target.has_partner
    pl.partner_lost()
    assert isinstance(pl.target, LostPartnerTarget) and pl.target.partner_a == (20.0, 30.0)
    assert not pl.target.retarget((22.0, 10.0)) and pl.target.partner_a == (20.0, 30.0)
    truth = []
    pos = pl.point_a()
    for _ in range(600):
        if pl.levels.ready() and isinstance(pl.target, FarEdgeTarget):
            break
        queue = pl.plan({1: pos}, me=1)[1]
        assert queue
        pos = queue[0][:2]
        _photo(pl, *pos, truth)
    assert isinstance(pl.target, FarEdgeTarget) and pl.met and not pl.target.has_partner
    route = pl.route()
    assert abs(route[-1][1] - FH) < 1e-6 and (20.0, 30.0) in [tuple(p) for p in route]
    snap = pl.levels.snapshot(1.0)
    assert snap.spans and pl.levels.report_snapshot() is snap


def test_a_lost_partner_already_met_opens_a_gap_to_the_far_edge():
    pl = _planner("zonemap", partner=True)
    pl.set_partner_point_a(20.0, 20.0)
    pos = pl.point_a()
    for _ in range(300):
        if pl.met:
            break
        queue = pl.plan({1: pos}, me=1)[1]
        pos = queue[0][:2]
        _photo(pl, *pos, [])
    assert pl.met and abs(pl.route()[-1][1] - 20.0) < 1e-6
    pl.partner_lost()
    assert not pl.met and pl.chain.forward_index() is not None
    assert abs(pl.chain.links[-1].poly[-1][1] - FH) < 1e-6


# -- the simulator ------------------------------------------------------------------------------
def _mission(drones, mines=30, seed=3, **kw):
    options = dict(levels=True, max_levels=2, planner_options={"engine": "field"})
    options.update(kw)
    return Mission(generate_true_minefield(mines, seed), drones=drones, **options).run()


def _assert_reports_a_valid_snapshot(m, min_level=1):
    assert m.failure is None, m.failure
    s = m.summary()
    e = evaluate(m)
    assert e["complete"] and e["spans_field"] and e["route_mine_squares"] == 0, e
    assert e["bad_edges_known"] == 0 and e["unseen_cells"] == 0, e
    assert s["reported_level"] is not None and s["reported_level"] >= min_level, s
    assert min(s["levels"].values()) >= min_level and all(n >= 1 for n in s["snapshots"].values())
    assert s["buffer_width"] >= 0, s
    return s


def test_one_two_and_four_drone_missions_with_levels_report_a_valid_snapshot():
    for drones in (1, 2, 4):
        m = _mission(drones)
        s = _assert_reports_a_valid_snapshot(m)
        levels = list(s["levels"].values())
        assert max(levels) - min(levels) <= 1
        assert s["growths"] >= 2 * len(set(m.side_of.values())) and s["reported_level"] == 2


def test_levels_off_leaves_the_mission_as_it_was():
    plain = Mission(generate_true_minefield(30, 3), drones=2).run()
    assert plain.driver is None and "levels" not in plain.summary()
    assert plain.report_route() == plain.backends[1].route()


def test_with_message_loss_both_sides_end_within_one_level():
    for seed in (4, 9):
        m = _mission(4, mines=30, seed=seed, drop=0.5, delay=3, max_levels=3)
        s = _assert_reports_a_valid_snapshot(m)
        a, b = s["levels"][1], s["levels"][3]
        assert abs(a - b) <= 1, s


def test_a_sim_with_a_zonemap_engine_also_climbs():
    m = _mission(2, mines=20, seed=2, planner_options={"engine": "zonemap"}, max_levels=1)
    _assert_reports_a_valid_snapshot(m)


def test_out_of_time_reports_the_latest_valid_snapshot_not_the_live_chain():
    m = _mission(1, mines=30, seed=3, max_levels=None, tick_budget=330)
    assert m.driver.timed_out and m.failure is None
    b = m.backends[1]
    snaps = b.valid_snapshots()
    assert snaps and b.report_valid()
    assert m.report_route() == list(snaps[-1].route) and snaps[-1].level >= 1
    assert not b.level_state(1).ready or snaps[-1].level == b.level


def test_no_growth_is_started_when_less_than_one_and_a_half_rebuilds_remain():
    m = _mission(1, mines=10, seed=1, max_levels=None, expand_ticks=40, tick_budget=140)
    assert m.driver.timed_out or m.done()
    for tick, _drone, _level, _secs in m.driver.growth:
        assert 140 - tick >= 60, tick


def test_a_lost_partner_side_is_left_behind_and_the_mission_goes_on_to_the_edge():
    m = _mission(4, mines=25, seed=5, partner_lost_at=(3, 25))
    assert m.dead == {3} and m.failure is None
    b = m.backends[1]
    assert not b.has_partner
    s = m.summary()
    e = evaluate(m)
    assert e["complete"] and e["spans_field"] and e["route_mine_squares"] == 0, e
    route = m.report_route()
    assert abs(route[-1][1] - m.field_h) < 1e-6 and abs(route[0][1]) < 1e-6
    assert s["levels"][1] >= 1 and s["reported_level"] >= 1


def test_a_lost_partner_before_the_first_point_a_still_finishes():
    m = _mission(3, mines=25, seed=6, partner_lost_at=(3, 2), max_levels=1)
    assert m.failure is None
    route = m.report_route()
    assert abs(route[-1][1] - m.field_h) < 1e-6


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}", flush=True)
    print(f"\n{len(tests)}/{len(tests)} passed")
