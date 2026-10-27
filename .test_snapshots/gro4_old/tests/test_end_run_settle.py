"""EndRun of a one-sided mission must wait for the side-mate to finish before it leaves.

It used to leave the moment its own copy was done: the mate's last photo (and a mine in it) was
still on its way, so the two copies ended with different mine sets (flight harness `2 dense`).
"""

import asyncio
from types import SimpleNamespace

from state_machine.states.calc_scan_path import CalcScanPath
from state_machine.states.impl import end_run_impl


class _Side:
    """A side agent whose mate is busy for the first `busy_polls` checks."""

    def __init__(self, busy_polls):
        self.busy_polls = busy_polls
        self.applied = 0

    @property
    def mate_busy(self):
        self.busy_polls -= 1
        return self.busy_polls >= 0

    def apply_inbox(self):
        self.applied += 1

    async def send_heartbeats(self, _mine_sync_s):
        pass


def _end_run(side):
    drone = SimpleNamespace(side=side, time_exceeded=lambda _s: False)
    fs = SimpleNamespace(max_flight_time=1000, current_drone_ID=2)
    return SimpleNamespace(drone=drone, flight_settings=fs, interdrone=None)


def _planner(done):
    return SimpleNamespace(failure=None, done=lambda: done)


def _settle(side, done, monkeypatch):
    monkeypatch.setattr(end_run_impl, "JOIN_POLL_S", 0.001)
    return asyncio.run(end_run_impl._settle_with_mate(_end_run(side), _planner(done)))


def test_waits_while_the_mate_is_busy_then_leaves(monkeypatch):
    side = _Side(busy_polls=3)
    assert _settle(side, True, monkeypatch) is None
    assert side.applied >= 3, "kept applying what the mate sent while it waited"


def test_leaves_at_once_when_the_mate_is_idle(monkeypatch):
    assert _settle(_Side(busy_polls=0), True, monkeypatch) is None


def test_a_late_photo_that_reopens_the_side_goes_back_to_scanning(monkeypatch):
    assert isinstance(_settle(_Side(busy_polls=2), False, monkeypatch), CalcScanPath)
    assert isinstance(_settle(_Side(busy_polls=0), False, monkeypatch), CalcScanPath)
