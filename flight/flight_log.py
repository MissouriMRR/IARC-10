"""Structured flight logging for collision-avoidance diagnosis.

Each drone runs in its own process, so each writes its own pair of files into a
shared run directory:

    Logs/<run_id>/drone_<id>.jsonl   machine-readable event stream
    Logs/<run_id>/drone_<id>.log     human-readable text log

`tools/analyze_flight.py` merges the JSONL streams from every drone in a run and
produces the plots and the analysis report. Everything is timestamped with
`time.time()` (unix epoch) rather than a monotonic clock so events from separate
processes can be placed on one timeline.

Set FLIGHT_LOG_RUN to the same value in every drone's environment so they land
in one directory. Without it each process invents its own run id and the
analyzer has to be pointed at the files by hand.
"""

from __future__ import annotations

import asyncio
import atexit
import json
import logging
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

# Rate the background telemetry task samples vehicle position at.
POSITION_SAMPLE_HZ: float = 5.0

_LOG_ROOT = Path(os.environ.get("FLIGHT_LOG_DIR", "Logs"))

_run_dir: Path | None = None
_drone_id: int = -1
_stream: Any = None
_stream_lock = threading.Lock()


def run_id() -> str:
    """The id of the current run, shared across drones via FLIGHT_LOG_RUN."""
    from_env = os.environ.get("FLIGHT_LOG_RUN")
    if from_env:
        return from_env
    return "run_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def run_dir() -> Path:
    """Directory holding this run's logs. None until `configure()` is called."""
    if _run_dir is None:
        raise RuntimeError("flight_log.configure() has not been called")
    return _run_dir


def configure(drone_id: int, console_level: int = logging.WARNING) -> Path:
    """Point logging at files and open this drone's event stream.

    Collision-avoidance output stops going to the console entirely: the
    "collision" logger is detached from the root logger and given its own file
    handler at DEBUG. Everything else still reaches the console, but only at
    `console_level` and above, so the terminal stays readable while the file
    keeps the full INFO-level record.

    Returns the run directory.
    """
    global _run_dir, _drone_id, _stream

    if _stream is not None:
        return run_dir()

    _drone_id = drone_id
    _run_dir = _LOG_ROOT / run_id()
    _run_dir.mkdir(parents=True, exist_ok=True)

    text_path = _run_dir / f"drone_{drone_id}.log"
    file_handler = logging.FileHandler(text_path, mode="w", encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(
        logging.Formatter(
            fmt=f"%(asctime)s.%(msecs)03d d{drone_id} %(levelname)-7s %(name)s: %(message)s",
            datefmt="%H:%M:%S",
        )
    )

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    for handler in list(root.handlers):
        # basicConfig() may already have installed a console handler; keep it but
        # raise its threshold so routine INFO chatter only lands in the file.
        if isinstance(handler, logging.StreamHandler) and not isinstance(
            handler, logging.FileHandler
        ):
            handler.setLevel(console_level)
    root.addHandler(file_handler)

    # Collision avoidance: file only. This is the data being collected, and at
    # 5 Hz hold heartbeats it would bury anything else printed to the terminal.
    collision = logging.getLogger("collision")
    collision.setLevel(logging.DEBUG)
    collision.propagate = False
    collision_handler = logging.FileHandler(
        _run_dir / f"drone_{drone_id}_collision.log", mode="w", encoding="utf-8"
    )
    collision_handler.setLevel(logging.DEBUG)
    collision_handler.setFormatter(
        logging.Formatter(
            fmt=f"%(asctime)s.%(msecs)03d d{drone_id} %(levelname)-7s %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    collision.addHandler(collision_handler)

    _stream = open(_run_dir / f"drone_{drone_id}.jsonl", "w", encoding="utf-8")
    atexit.register(close)

    # Recorded so the analyzer reports the thresholds this flight actually ran
    # with, rather than whatever is checked out when the logs are read. Imported
    # here rather than at module scope to keep this module importable on its own.
    from flight.waypoint import COLLISION_RADIUS, MIN_SEPARATION_M

    event(
        "run_start",
        pid=os.getpid(),
        run=run_id(),
        collision_radius=COLLISION_RADIUS,
        min_separation=MIN_SEPARATION_M,
        sample_hz=POSITION_SAMPLE_HZ,
    )
    return _run_dir


def close() -> None:
    """Flush and close the event stream."""
    global _stream
    with _stream_lock:
        if _stream is not None:
            _stream.flush()
            _stream.close()
            _stream = None


def event(kind: str, **fields: Any) -> None:
    """Append one event to this drone's JSONL stream.

    Silently does nothing if `configure()` was never called, so instrumented
    code paths stay safe in unit tests and one-off scripts.
    """
    if _stream is None:
        return

    record: dict[str, Any] = {"t": time.time(), "drone": _drone_id, "kind": kind}
    record.update(fields)
    line = json.dumps(record, default=_encode)
    with _stream_lock:
        if _stream is None:
            return
        _stream.write(line + "\n")
        # Flushed per event: a run that ends in a crash or a Ctrl-C is exactly
        # the run whose tail matters most, and the event rate is low enough
        # (~10/s/drone) that this costs nothing.
        _stream.flush()


def _encode(obj: Any) -> Any:
    to_dict = getattr(obj, "to_dict", None)
    if callable(to_dict):
        return to_dict()
    return str(obj)


def waypoint_brief(waypoint: Any) -> dict[str, Any] | None:
    """The fields of a waypoint worth repeating on every event."""
    if waypoint is None:
        return None
    return {
        "id": waypoint.waypoint_id,
        "lat": waypoint.lat,
        "lon": waypoint.long,
        "lap": getattr(waypoint, "lap", None),
        "idx": getattr(waypoint, "index", None),
    }


def waypoints_brief(waypoints: Iterable[Any]) -> list[dict[str, Any] | None]:
    return [waypoint_brief(waypoint) for waypoint in waypoints]


class PositionLogger:
    """Background task sampling live vehicle state at `POSITION_SAMPLE_HZ`.

    The sample also captures what this drone currently *believes* about every
    peer, so the analyzer can compare the avoidance logic's view of the world
    against where the peers actually were. A stale or wrong belief looks
    identical to a correct one from inside the algorithm, and telling those
    apart is the whole point of collecting this.
    """

    def __init__(self, drone: Any, peer_states: Iterable[Any] = ()) -> None:
        self.drone = drone
        self.peer_states = list(peer_states)
        self._task: asyncio.Task[None] | None = None

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.ensure_future(self._run())

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None

    async def _run(self) -> None:
        interval = 1.0 / POSITION_SAMPLE_HZ
        while True:
            try:
                self.sample()
            except Exception as ex:  # never let telemetry kill the flight
                logging.debug("position sample failed: %s", ex)
            await asyncio.sleep(interval)

    def sample(self, **extra: Any) -> None:
        """Emit one `pos` event. Also called directly at moments of interest."""
        vehicle = self.drone.vehicle
        position = vehicle.location.global_relative_frame

        beliefs = []
        for state in self.peer_states:
            occupied = state.occupied_segment()
            beliefs.append(
                {
                    "peer": state.drone_id,
                    "queued": len(state.list_of_waypoints),
                    "last_reached": waypoint_brief(state.last_reached_waypoint),
                    "target": waypoint_brief(
                        state.list_of_waypoints[0] if state.list_of_waypoints else None
                    ),
                    "occupied": occupied,
                    # What the peer says it measured, next to what its waypoints
                    # imply. The gap between these two is the whole question:
                    # avoidance is only as good as the belief it acts on, and a
                    # wrong belief is indistinguishable from a right one from
                    # inside the algorithm.
                    "reported": getattr(state, "live_position", None),
                    "reported_alt": getattr(state, "live_altitude", None),
                    "drift": (
                        state.formation_drift()
                        if hasattr(state, "formation_drift")
                        else None
                    ),
                }
            )

        event(
            "pos",
            lat=position.lat,
            lon=position.lon,
            alt=position.alt,
            groundspeed=getattr(vehicle, "groundspeed", None),
            heading=getattr(vehicle, "heading", None),
            mode=getattr(getattr(vehicle, "mode", None), "name", None),
            armed=getattr(vehicle, "armed", None),
            beliefs=beliefs,
            **extra,
        )
