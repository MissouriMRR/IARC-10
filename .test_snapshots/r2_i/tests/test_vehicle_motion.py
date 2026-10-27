"""The flight harness's motion models (integration_tests/vehicle_motion.py).

TeleportMotion lands every goto at once, which hid every crossing of two drones from the flight
code (legs took no time). CruiseMotion flies there over time.
"""

import math

import pytest

from integration_tests.vehicle_motion import CruiseMotion, TeleportMotion

LAT, LON = 36.0, -95.9
M = 1.0 / 111_320.0


class _Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def _metres(a, b):
    return math.hypot((b.lat - a.lat) / M, (b.lon - a.lon) / M * math.cos(math.radians(LAT)))


def test_teleport_is_there_at_once_and_touch_down_zeroes_altitude():
    m = TeleportMotion(LAT, LON)
    m.goto(LAT + 10 * M, LON, 3.0, 2.0)
    here = m.position()
    assert (here.lat, here.alt) == (LAT + 10 * M, 3.0)
    m.touch_down()
    assert m.position().alt == 0.0 and m.position().lat == here.lat


def test_cruise_flies_a_straight_line_at_speedup_times_the_groundspeed():
    clock = _Clock()
    m = CruiseMotion(LAT, LON, alt=2.0, speedup=2.0, clock=clock)
    m.goto(LAT + 40 * M, LON, 2.0, 2.0)             # 40 m at 2 m/s x 2 = 10 s
    start = m.position()
    clock.now += 5.0
    half = m.position()
    assert _metres(start, half) == pytest.approx(20.0, abs=0.01)
    assert half.lon == pytest.approx(LON)
    clock.now += 100.0
    assert m.position().lat == pytest.approx(LAT + 40 * M), "stops at the target"


def test_cruise_regoto_starts_from_where_it_is():
    clock = _Clock()
    m = CruiseMotion(LAT, LON, speedup=1.0, clock=clock)
    m.goto(LAT + 20 * M, LON, 0.0, 2.0)
    clock.now += 5.0                                  # 10 m on
    m.goto(LAT + 10 * M, LON + 10 * M, 0.0, None)     # keeps the speed
    assert _metres(m.position(), m.position()) == 0.0
    mid = m.position()
    assert (mid.lat - LAT) / M == pytest.approx(10.0, abs=0.01)
    clock.now += 100.0
    assert m.position().lat == pytest.approx(LAT + 10 * M)


def test_cruise_climb_takes_time_too():
    clock = _Clock()
    m = CruiseMotion(LAT, LON, speedup=1.0, clock=clock)
    m.goto(LAT, LON, 4.0, None)                       # 4 m at 1 m/s
    clock.now += 2.0
    assert m.position().alt == pytest.approx(2.0)
    m.touch_down()
    assert m.position().alt == 0.0
