"""A drone that lands (an abort, or the end) tells the others, and they stop waiting for it.

Found in the flight harness: when one drone aborted into Land, every drone waiting on it sat in
EndRun until the 7-minute flight budget plus JOIN_GRACE_S ran out (the harness timed out at 300 s),
because nothing told them the drone was gone but 120 s of silence.
"""

import asyncio
from types import SimpleNamespace

from flight.pathfinding.departures import Departures
from flight.pathfinding.mate_watch import MATE_GONE_S, MateWatch
from flight.pathfinding.peer_positions import PeerPositions
from interdrone_communication.message_types import Message, MessageType
from state_machine.drone_state import DroneState
from state_machine.flight_settings import FlightSettings
from state_machine.interdrone import CMD_MSG, Interdrone
from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.impl import end_run_impl


class FakeInterdrone:
    def __init__(self, **states):
        self.states = {int(k[1:]): v for k, v in states.items()}

    def get_drone_state_from_id(self, drone_id):
        return self.states.get(drone_id)


def _state(**kw):
    state = SimpleNamespace(mission_over=False, live_position=(1.0, 1.0), silent_for=0.0)
    state.is_stale = lambda s: state.silent_for >= s
    state.position_is_stale = lambda s: state.silent_for >= s
    for k, v in kw.items():
        setattr(state, k, v)
    return state


# -- who counts as gone ----------------------------------------------------------------------
def test_a_landing_mate_is_lost_and_gone_at_once_and_stays_so():
    state = _state()
    drone = SimpleNamespace(startTime=0.0, currentPosition=lambda: (1.0, 1.0))
    watch = MateWatch(1, 2, FakeInterdrone(d2=state), drone, clock=lambda: 5.0)   # mission just began
    watch.check()
    assert (watch.lost, watch.gone, watch.landed) == (False, False, False)
    state.mission_over = True
    watch.check()
    assert (watch.lost, watch.gone, watch.landed) == (True, True, True)
    assert watch.speaker and watch.lost_mates() == ()


def test_departures_landed_or_silent_after_being_heard_but_not_unknown():
    inter = FakeInterdrone(d3=_state(mission_over=True), d4=_state(silent_for=MATE_GONE_S),
                           d5=_state(live_position=None, silent_for=1e9), d6=_state())
    out = Departures(inter)
    assert out.gone(3) and out.gone(4)
    assert not out.gone(5), "never heard from: it may not have started"
    assert not out.gone(6) and not out.gone(7), "alive; unknown"
    assert out.all_gone([3, 4]) and not out.all_gone([3, 6]) and not out.all_gone([])


def test_a_landed_drone_no_longer_blocks_the_other_sides_positions():
    inter = FakeInterdrone(d3=_state(mission_over=True), d4=_state())
    drone = SimpleNamespace(currentPosition=lambda: (0.0, 0.0))
    watch = SimpleNamespace(state=lambda: None, gone=False, lost=False)
    peers = PeerPositions(1, None, (3, 4), SimpleNamespace(start_coord=None), inter, drone, watch)
    assert set(peers.other_side()) == {4}


# -- the message -----------------------------------------------------------------------------
class _Net:
    def __init__(self, incoming=()):
        self.incoming, self.sent = list(incoming), []

    def try_get_server_message(self, timeout=0.0):
        return self.incoming.pop(0) if self.incoming else None

    def queue_client_message(self, message, timeout=None):
        self.sent.append(message)


def _interdrone(me, others, incoming=()):
    interdrone = object.__new__(Interdrone)
    interdrone._current_task = interdrone._current_state = interdrone._restart_callback = None
    interdrone.flight_settings = FlightSettings(drone_ID=me, drones_in_mission=[me, *others])
    interdrone.drone = SimpleNamespace(id=me, observe_leg_clock=lambda c: None)
    interdrone.drone_states = [DroneState(d, "127.0.0.1") for d in others]
    interdrone.cmd_msg = CMD_MSG.NONE
    interdrone.max_messages_per_tick = 25
    interdrone.interdrone_messages = {}
    interdrone.networking = _Net(incoming)
    return interdrone


def test_mission_end_marks_the_sender_as_over():
    end = Message.create(id=MessageType.MISSION_END, drones_to_send_data=(2,), sender_id=1, data={})
    interdrone = _interdrone(2, [1], incoming=[end])
    state = interdrone.get_drone_state_from_id(1)
    assert not state.mission_over

    async def run():
        loop = asyncio.ensure_future(interdrone.interdrone_loop())
        for _ in range(100):
            if state.mission_over:
                break
            await asyncio.sleep(0.01)
        loop.cancel()

    asyncio.run(run())
    assert state.mission_over


def test_announce_landing_sends_to_every_other_drone_more_than_once():
    interdrone = _interdrone(2, [1, 3, 4])
    asyncio.run(interdrone.announce_landing(repeats=3, gap_s=0.0))
    sent = interdrone.networking.sent
    assert [m.id for m in sent] == [MessageType.MISSION_END] * 3
    assert set(sent[0].drones_to_send_data) == {1, 3, 4}


# -- EndRun stops waiting ---------------------------------------------------------------------
class _Side:
    """The parts of SideAgent that EndRun's join loop touches."""

    def __init__(self, partner_gone=False, reporter_gone=False):
        self.partner_gone, self.reporter_gone = partner_gone, reporter_gone
        self.mate_busy = False
        self.speaker = True
        self.lost = []

    def apply_inbox(self):
        pass

    async def send_heartbeats(self, _s):
        pass

    def partner_lost(self):
        self.lost.append(True)


def _end_run(side, drone_id, **drone_kw):
    drone = SimpleNamespace(
        side=side, time_exceeded=lambda _s: False, route_joined=False, partner_route_ack_hash=None,
        partner_route=None, partner_route_complete=False, partner_route_mine_hash=None,
        joined_mine_hash=None, mission_path=[], reported_half_route=False, **drone_kw)
    fs = SimpleNamespace(max_flight_time=1000, current_drone_ID=drone_id, partner_side_drones=(3, 4),
                         paired_drone=2 if drone_id == 1 else 1, start_side=None)

    async def nothing(*_a, **_k):
        pass

    interdrone = SimpleNamespace(send_cross_pair_chain=nothing, send_cross_pair_chain_ack=nothing)
    return SimpleNamespace(drone=drone, flight_settings=fs, interdrone=interdrone)


def _planner(done=True):
    return SimpleNamespace(failure=None, done=lambda: done, mine_hash=lambda: "h",
                           route_latlon=lambda: [(1.0, 1.0), (2.0, 2.0)])


def _join(end_run, planner, monkeypatch):
    monkeypatch.setattr(end_run_impl, "JOIN_POLL_S", 0.001)
    return asyncio.run(asyncio.wait_for(end_run_impl._join_with_partner(end_run, planner), 5))


def test_the_reporter_stops_waiting_when_the_other_side_has_landed(monkeypatch):
    side = _Side(partner_gone=True)
    end_run = _end_run(side, 1)
    assert _join(end_run, _planner(done=True), monkeypatch) is None
    assert side.lost == [True], "the planner hook was told"
    assert end_run.drone.reported_half_route and end_run.drone.mission_path == [(1.0, 1.0), (2.0, 2.0)]


def test_work_left_after_the_other_side_is_lost_is_carried_on(monkeypatch):
    side = _Side(partner_gone=True)
    assert isinstance(_join(_end_run(side, 1), _planner(done=False), monkeypatch), CalcScanPath)
    assert side.lost == [True]


def test_a_partner_that_joined_and_went_home_is_not_lost(monkeypatch):
    side = _Side(partner_gone=True)
    end_run = _end_run(side, 2)                      # waits for the reporter's ack, which came
    end_run.drone.partner_route_ack_hash = "h"
    assert _join(end_run, _planner(), monkeypatch) is None
    assert side.lost == [] and not end_run.drone.reported_half_route


def test_a_drone_waiting_for_the_reporters_ack_leaves_when_the_reporter_landed(monkeypatch):
    side = _Side(reporter_gone=True)
    end_run = _end_run(side, 2)
    assert _join(end_run, _planner(), monkeypatch) is None
    assert end_run.drone.mission_path == [(1.0, 1.0), (2.0, 2.0)]
