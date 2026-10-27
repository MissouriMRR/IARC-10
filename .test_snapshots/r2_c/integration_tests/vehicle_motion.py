"""Motion models for the fake vehicle (`MockVehicle`): where it is, given the goto commands so far.

Contract:
- `Motion.position()`: the vehicle's fix NOW (lat, lon, alt in metres above the pad).
- `Motion.goto(lat, lon, alt, groundspeed)`: head for a point; replaces any goto in progress.
- `Motion.touch_down()`: on the ground at once (what RTL/LAND ends in).
- Two implementations (pick one per vehicle, no flags):
  - `TeleportMotion`: a goto lands at once. Legs take no time, so the flight code never sees
    another drone in transit.
  - `CruiseMotion`: a straight line at constant speed over real time. Reads of `position()`
    in between give the points on the way, as a real autopilot's position does.
- Speed: `CruiseMotion` flies `speedup` times the commanded groundspeed (a 300 s harness run
  cannot afford 2 m/s). The flight code's own timers stay real, so a leg that takes seconds
  on the field takes seconds / speedup here.
"""

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

_M_PER_DEG_LAT = 111320.0


@dataclass(frozen=True)
class Fix:
    """A position: lat, lon in degrees, alt in metres."""

    lat: float
    lon: float
    alt: float


def _metres_between(a: Fix, b: Fix) -> tuple[float, float, float]:
    """(north, east, up) from a to b, in metres (flat earth: legs are tens of metres)."""
    north = (b.lat - a.lat) * _M_PER_DEG_LAT
    east = (b.lon - a.lon) * _M_PER_DEG_LAT * math.cos(math.radians(a.lat))
    return north, east, b.alt - a.alt


class Motion(ABC):
    """Where a vehicle is over time; see the module docstring."""

    @abstractmethod
    def position(self) -> Fix:
        """The vehicle's fix now."""

    @abstractmethod
    def goto(self, lat: float, lon: float, alt: float, groundspeed: float | None) -> None:
        """Head for this point (groundspeed in m/s, None: keep the last one)."""

    @abstractmethod
    def touch_down(self) -> None:
        """On the ground (altitude 0) where it is."""


class TeleportMotion(Motion):
    """A goto lands at once."""

    def __init__(self, lat: float, lon: float, alt: float = 0.0):
        self._fix = Fix(lat, lon, alt)

    def position(self) -> Fix:
        return self._fix

    def goto(self, lat, lon, alt, groundspeed=None) -> None:
        self._fix = Fix(lat, lon, alt)

    def touch_down(self) -> None:
        self._fix = Fix(self._fix.lat, self._fix.lon, 0.0)


class CruiseMotion(Motion):
    """A straight line at constant speed.

    - Horizontal speed: the commanded groundspeed x `speedup`. Vertical: CLIMB_M_S x `speedup`.
    - The leg lasts as long as the slower of the two needs; both axes move in step.
    - A new goto starts from wherever the vehicle is at that moment.
    """

    DEFAULT_GROUNDSPEED_M_S = 2.0
    CLIMB_M_S = 1.0

    def __init__(self, lat: float, lon: float, alt: float = 0.0, speedup: float = 5.0,
                 clock: Callable[[], float] = time.monotonic):
        self._speedup = speedup
        self._clock = clock
        self._groundspeed = self.DEFAULT_GROUNDSPEED_M_S
        self._from = Fix(lat, lon, alt)
        self._to = self._from
        self._t0 = clock()
        self._duration = 0.0

    def position(self) -> Fix:
        if self._duration <= 0.0:
            return self._to
        f = (self._clock() - self._t0) / self._duration
        if f >= 1.0:
            return self._to
        a, b = self._from, self._to
        return Fix(a.lat + (b.lat - a.lat) * f, a.lon + (b.lon - a.lon) * f,
                   a.alt + (b.alt - a.alt) * f)

    def goto(self, lat, lon, alt, groundspeed=None) -> None:
        if groundspeed is not None:
            self._groundspeed = groundspeed
        here = self.position()
        target = Fix(lat, lon, alt)
        north, east, up = _metres_between(here, target)
        horizontal_s = math.hypot(north, east) / (self._groundspeed * self._speedup)
        vertical_s = abs(up) / (self.CLIMB_M_S * self._speedup)
        self._from, self._to = here, target
        self._t0 = self._clock()
        self._duration = max(horizontal_s, vertical_s)

    def touch_down(self) -> None:
        here = self.position()
        self._from = self._to = Fix(here.lat, here.lon, 0.0)
        self._duration = 0.0
