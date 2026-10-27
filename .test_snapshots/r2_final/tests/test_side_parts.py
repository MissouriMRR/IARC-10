"""MateWatch, Resend and PeerPositions transitions, with fake clocks and fake peers."""

from types import SimpleNamespace

import asyncio

from flight.pathfinding.heartbeats import Heartbeats, Resend
from flight.pathfinding.side_agent import SideAgent
from flight.pathfinding.mate_watch import MATE_GONE_S, MATE_LOST_S, MateWatch
from flight.pathfinding.peer_positions import PeerPositions


class FakeState:
    def __init__(self, silent_for=0.0, position=None):
        self.silent_for = silent_for
        self.live_position = position
        self.mission_over = False

    def is_stale(self, seconds):
        return self.silent_for >= seconds

    def position_is_stale(self, seconds):
        return self.silent_for >= seconds


class FakeInterdrone:
    def __init__(self):
        self.states = {}

    def get_drone_state_from_id(self, drone_id):
        return self.states.get(drone_id)


def _watch(mate=2, me=1, start=0.0):
    clock = SimpleNamespace(now=1000.0)
    inter = FakeInterdrone()
    drone = SimpleNamespace(startTime=start, currentPosition=lambda: (1.0, 1.0))
    watch = MateWatch(me, mate, inter, drone, clock=lambda: clock.now)
    return watch, inter, clock


def test_mate_goes_lost_then_gone_then_back():
    watch, inter, _ = _watch()
    inter.states[2] = FakeState(silent_for=1.0)
    watch.check()
    assert (watch.lost, watch.gone) == (False, False) and watch.lost_mates() == ()
    inter.states[2].silent_for = MATE_LOST_S
    watch.check()
    assert (watch.lost, watch.gone) == (True, False) and watch.lost_mates() == (2,)
    inter.states[2].silent_for = MATE_GONE_S
    watch.check()
    assert (watch.lost, watch.gone) == (True, True) and watch.lost_mates() == ()
    inter.states[2].silent_for = 0.0
    watch.check()
    assert (watch.lost, watch.gone) == (False, False)


def test_not_lost_before_mission_has_run():
    watch, inter, _ = _watch(start=990.0)
    inter.states[2] = FakeState(silent_for=500.0)
    watch.check()
    assert not watch.lost


def test_speaker_is_lowest_id_still_heard():
    watch, inter, _ = _watch(me=1, mate=2)
    assert watch.speaker
    watch, inter, _ = _watch(me=2, mate=1)
    inter.states[1] = FakeState()
    watch.check()
    assert not watch.speaker
    inter.states[1].silent_for = MATE_LOST_S
    watch.check()
    assert watch.speaker


def test_alone_is_always_speaker():
    watch, _, _ = _watch(mate=None)
    watch.check()
    assert watch.speaker and watch.lost_mates() == ()


def test_resend_on_change_or_age():
    r = Resend(1.0)
    assert r.due("a", 0.0)
    r.mark("a", 0.0)
    assert not r.due("a", 0.5)
    assert r.due("b", 0.5)
    assert r.due("a", 1.0)
    assert not r.due("a", 4.0, period=5.0)
    assert r.due("a", 5.0, period=5.0)


def test_mate_position_fallbacks():
    watch, inter, _ = _watch()
    fs = SimpleNamespace(start_coord={"lat": 5.0, "lon": 6.0})
    drone = SimpleNamespace(currentPosition=lambda: (1.0, 1.0))
    peers = PeerPositions(1, 2, (), fs, inter, drone, watch)
    assert peers.side() == {1: (1.0, 1.0), 2: (5.0, 6.0)}
    peers.mate_photographed([(0.0, 0.0), (2.0, 4.0)])
    assert peers.side()[2] == (1.0, 2.0)
    inter.states[2] = FakeState(silent_for=0.5, position=(9.0, 9.0))
    assert peers.side()[2] == (9.0, 9.0)
    inter.states[2].silent_for = 5.0          # stale report: photo position again
    assert peers.side()[2] == (1.0, 2.0)
    inter.states[2].silent_for = MATE_GONE_S
    watch.check()
    assert peers.side()[2] is None


class _Planner:
    failure = None

    def __init__(self):
        self.plans = 0

    def current_job(self, _me):
        return None

    def assignments(self, *_a, **_k):
        self.plans += 1
        return {1: [(1.0, 2.0)], 2: [(1.0, 2.0)]}


class _Inter:
    def __init__(self):
        self.jobs = []

    async def send_side_job(self, job, to):
        self.jobs.append((job, to))


def test_no_job_is_announced_before_the_first_plan():
    """An early "no job" would tell a higher-id mate this drone is idle: both start the same work."""
    watch, _, _ = _watch(me=1, mate=2)
    inter = _Inter()
    beats = Heartbeats(_Planner(), inter, watch, 1, 2, ())
    asyncio.run(beats.send())
    assert inter.jobs == [], "idle only because it has not planned yet"
    asyncio.run(beats.announce_job())                  # what a plan ends with
    assert len(inter.jobs) == 1
    asyncio.run(beats.send(5.0))
    assert len(inter.jobs) == 1, "unchanged, not due"


def _agent(me, mate, peer_jobs):
    agent = object.__new__(SideAgent)
    watch, inter, _ = _watch(me=me, mate=mate)
    inter.states[mate] = FakeState(silent_for=0.0, position=(1.0, 1.0))
    agent.me, agent.mate, agent.partners = me, mate, ()
    agent.fs = SimpleNamespace(start_coord={"lat": 0.0, "lon": 0.0})
    agent.drone = SimpleNamespace(peer_jobs=peer_jobs, currentPosition=lambda: (1.0, 1.0))
    agent.planner = _Planner()
    agent.watch = watch
    agent.peers = PeerPositions(me, mate, (), agent.fs, inter, agent.drone, watch)
    agent.inbox = SimpleNamespace(apply=lambda: None)
    agent.heartbeats = SimpleNamespace(send=lambda _s=None: _noop(), announce_job=lambda: _noop())
    return agent


async def _noop():
    return None


def test_the_higher_id_waits_for_the_lower_mates_first_job_announcement():
    """Both copies would otherwise pick the same first job and fly it together."""
    higher = _agent(me=2, mate=1, peer_jobs={})
    assert asyncio.run(higher.plan()) == [] and higher.planner.plans == 0
    higher.drone.peer_jobs[1] = None                    # announced (even: no job)
    asyncio.run(higher.plan())
    assert higher.planner.plans == 1
    lower = _agent(me=1, mate=2, peer_jobs={})
    asyncio.run(lower.plan())
    assert lower.planner.plans == 1, "the lower id never waits for the higher"
