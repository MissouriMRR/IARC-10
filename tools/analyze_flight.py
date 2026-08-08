#!/usr/bin/env python3
"""Merge per-drone flight logs into path plots and a diagnosis report.

    python tools/analyze_flight.py Logs/run_20260806_143000

Reads every drone_*.jsonl in a run directory, puts all the drones on one
timeline, and writes:

    paths_overview.png      every drone's whole flight, one color each
    paths_lap_<n>.png       one panel per lap, planned circle vs flown track
    separation.png          true inter-drone distance over time vs the radius
    report.md               the numbers: separation minima, breaches, holds,
                            tracking error, stalls, belief staleness

The report is the part that diagnoses the problem. The plots are for looking at.
True separation is computed from the raw position streams, so it does not depend
on the avoidance logic being correct -- that is the point. Everything the
algorithm believed is reported separately and compared against it.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

# Deliberately self-contained: this reads JSONL files and needs nothing from the
# flight package. Importing flight.waypoint would drag in the whole flight
# import chain, so a broken dronekit or numpy in whichever interpreter you
# happen to run this with would stop you analyzing logs that are already on
# disk. matplotlib is the only third-party import, and it is optional.

EARTH_RADIUS_M: float = 6_378_137.0
_DEG_TO_M: float = math.pi * EARTH_RADIUS_M / 180.0

# Fallbacks only. The real values are read from each run's `run_start` event, so
# the report reflects the constants the flight actually used rather than
# whatever happens to be checked out now. Kept in sync with flight/waypoint.py.
DEFAULT_MIN_SEPARATION_M: float = 32 * 0.0254
DEFAULT_COLLISION_RADIUS: float = DEFAULT_MIN_SEPARATION_M + 2 * 2.0

# Overwritten by load_run() from the log's run_start event when present.
COLLISION_RADIUS: float = DEFAULT_COLLISION_RADIUS
MIN_SEPARATION_M: float = DEFAULT_MIN_SEPARATION_M

# Sampling step for the true-separation sweep, in seconds. Finer than the 5 Hz
# telemetry so interpolation, not sampling, sets the resolution.
SEPARATION_STEP_S: float = 0.1


def project_to_meters(
    lat: float, lon: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """Project a lat/lon onto a local east/north frame in meters around a reference."""
    lon_scale = _DEG_TO_M * math.cos(math.radians(ref_lat))
    return (lon - ref_lon) * lon_scale, (lat - ref_lat) * _DEG_TO_M


# Distinct in both light and dark, and distinguishable in grayscale print.
DRONE_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b"]


def color_for(drone_id: int) -> str:
    return DRONE_COLORS[(drone_id - 1) % len(DRONE_COLORS)]


@dataclass
class DroneLog:
    """One drone's events, split into the series the report needs."""

    drone_id: int
    events: list[dict[str, Any]] = field(default_factory=list)
    times: list[float] = field(default_factory=list)  # position sample times
    lats: list[float] = field(default_factory=list)
    lons: list[float] = field(default_factory=list)
    alts: list[float] = field(default_factory=list)
    speeds: list[float] = field(default_factory=list)

    def of_kind(self, *kinds: str) -> list[dict[str, Any]]:
        return [e for e in self.events if e["kind"] in kinds]

    def position_at(self, t: float) -> tuple[float, float] | None:
        """Linearly interpolated position, or None outside the sampled window."""
        if not self.times or t < self.times[0] or t > self.times[-1]:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == 0:
            return (self.lats[0], self.lons[0])
        t0, t1 = self.times[i - 1], self.times[i]
        if t1 == t0:
            return (self.lats[i], self.lons[i])
        f = (t - t0) / (t1 - t0)
        return (
            self.lats[i - 1] + f * (self.lats[i] - self.lats[i - 1]),
            self.lons[i - 1] + f * (self.lons[i] - self.lons[i - 1]),
        )


def load_run(run_dir: Path) -> dict[int, DroneLog]:
    """Read every drone_<id>.jsonl in a run directory."""
    drones: dict[int, DroneLog] = {}
    paths = sorted(run_dir.glob("drone_*.jsonl"))
    if not paths:
        raise SystemExit(f"no drone_*.jsonl files in {run_dir}")

    for path in paths:
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                # A run killed mid-write leaves a torn final line. Everything
                # before it is still good, so keep it and move on.
                print(f"  skipping malformed line {path.name}:{line_no}")
                continue

            drone_id = record["drone"]
            log = drones.setdefault(drone_id, DroneLog(drone_id))
            log.events.append(record)

            if record["kind"] == "pos" and record.get("lat") is not None:
                log.times.append(record["t"])
                log.lats.append(record["lat"])
                log.lons.append(record["lon"])
                log.alts.append(record.get("alt") or 0.0)
                log.speeds.append(record.get("groundspeed") or 0.0)

    for log in drones.values():
        log.events.sort(key=lambda e: e["t"])

    _adopt_run_constants(drones)
    return drones


def _adopt_run_constants(drones: dict[int, DroneLog]) -> None:
    """Use the thresholds the flight actually ran with, if it recorded them."""
    global COLLISION_RADIUS, MIN_SEPARATION_M
    for log in drones.values():
        for event in log.of_kind("run_start"):
            if event.get("collision_radius") is not None:
                COLLISION_RADIUS = event["collision_radius"]
                MIN_SEPARATION_M = event["min_separation"]
                return
    print(
        f"  note: logs predate constant recording; assuming COLLISION_RADIUS="
        f"{COLLISION_RADIUS:.2f}m, MIN_SEPARATION_M={MIN_SEPARATION_M:.2f}m"
    )


def meters_between(a: tuple[float, float], b: tuple[float, float]) -> float:
    dx, dy = project_to_meters(b[0], b[1], a[0], a[1])
    return math.hypot(dx, dy)


# --------------------------------------------------------------------------
# Analysis
# --------------------------------------------------------------------------


def true_separation(
    drones: dict[int, DroneLog],
) -> tuple[list[float], dict[tuple[int, int], list[float]]]:
    """Actual distance between every pair of drones, on a common time base."""
    covered = [log for log in drones.values() if log.times]
    if len(covered) < 2:
        return [], {}

    start = max(log.times[0] for log in covered)
    end = min(log.times[-1] for log in covered)
    if end <= start:
        return [], {}

    steps = int((end - start) / SEPARATION_STEP_S) + 1
    timeline = [start + i * SEPARATION_STEP_S for i in range(steps)]

    ids = sorted(log.drone_id for log in covered)
    series: dict[tuple[int, int], list[float]] = {}
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            distances: list[float] = []
            for t in timeline:
                pa, pb = drones[a].position_at(t), drones[b].position_at(t)
                distances.append(meters_between(pa, pb) if pa and pb else float("nan"))
            series[(a, b)] = distances
    return timeline, series


@dataclass
class Breach:
    pair: tuple[int, int]
    start: float
    end: float
    closest: float
    at: float


def find_breaches(
    timeline: list[float],
    series: dict[tuple[int, int], list[float]],
    threshold: float,
) -> list[Breach]:
    """Contiguous stretches where a pair was closer than `threshold`."""
    breaches: list[Breach] = []
    for pair, distances in series.items():
        run_start: int | None = None
        for i, d in enumerate(distances):
            inside = not math.isnan(d) and d < threshold
            if inside and run_start is None:
                run_start = i
            elif not inside and run_start is not None:
                window = distances[run_start:i]
                closest = min(window)
                breaches.append(
                    Breach(
                        pair,
                        timeline[run_start],
                        timeline[i - 1],
                        closest,
                        timeline[run_start + window.index(closest)],
                    )
                )
                run_start = None
        if run_start is not None:
            window = distances[run_start:]
            closest = min(window)
            breaches.append(
                Breach(
                    pair,
                    timeline[run_start],
                    timeline[-1],
                    closest,
                    timeline[run_start + window.index(closest)],
                )
            )
    breaches.sort(key=lambda b: b.closest)
    return breaches


_LAP_TRACK_CACHE: dict[int, dict[int, list[tuple[float, float]]]] = {}


def lap_tracks(log: DroneLog) -> dict[int, list[tuple[float, float]]]:
    """The flown track split by lap, using waypoint arrivals as lap boundaries.

    A position sample belongs to the lap of the next waypoint the drone went on
    to reach, which keeps the time spent holding attached to the lap it was
    holding for.
    """
    if log.drone_id in _LAP_TRACK_CACHE:
        return _LAP_TRACK_CACHE[log.drone_id]

    arrivals = [
        e for e in log.of_kind("wp_reached") if (e.get("target") or {}).get("lap") is not None
    ]
    if not arrivals:
        _LAP_TRACK_CACHE[log.drone_id] = {}
        return {}

    boundary_times = [e["t"] for e in arrivals]
    boundary_laps = [e["target"]["lap"] for e in arrivals]

    tracks: dict[int, list[tuple[float, float]]] = defaultdict(list)
    for t, lat, lon in zip(log.times, log.lats, log.lons):
        i = min(bisect.bisect_left(boundary_times, t), len(boundary_laps) - 1)
        tracks[boundary_laps[i]].append((lat, lon))

    result = dict(tracks)
    _LAP_TRACK_CACHE[log.drone_id] = result
    return result


def planned_circle(log: DroneLog) -> tuple[tuple[float, float], float, list[dict[str, Any]]]:
    """Center, radius and full waypoint list from this drone's POIF plan."""
    plans = log.of_kind("poif_plan")
    if not plans:
        return ((0.0, 0.0), 0.0, [])
    plan = plans[-1]
    return (tuple(plan["center"]), plan["radius_m"], plan.get("waypoints") or [])


# --------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------


def make_plots(drones: dict[int, DroneLog], out_dir: Path, timeline, series) -> list[str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plots (report is unaffected)")
        return []

    written: list[str] = []
    ref = next(((log.lats[0], log.lons[0]) for log in drones.values() if log.times), (0.0, 0.0))

    def to_xy(points: Iterable[tuple[float, float]]) -> tuple[list[float], list[float]]:
        xs, ys = [], []
        for lat, lon in points:
            x, y = project_to_meters(lat, lon, ref[0], ref[1])
            xs.append(x)
            ys.append(y)
        return xs, ys

    # --- overview: every drone's whole flight -----------------------------
    fig, ax = plt.subplots(figsize=(11, 11))
    for drone_id in sorted(drones):
        log = drones[drone_id]
        if not log.times:
            continue
        xs, ys = to_xy(zip(log.lats, log.lons))
        ax.plot(xs, ys, color=color_for(drone_id), lw=1.2, label=f"drone {drone_id} flown")

        center, radius, waypoints = planned_circle(log)
        if waypoints:
            wx, wy = to_xy([(w["lat"], w["lon"]) for w in waypoints])
            ax.scatter(wx, wy, color=color_for(drone_id), s=8, alpha=0.35, marker="x")

        # Lap number where each lap's first waypoint was actually reached.
        for event in log.of_kind("wp_reached"):
            target = event.get("target") or {}
            if target.get("idx") == 0 and target.get("lap") is not None:
                px, py = to_xy([tuple(event["at"])])
                ax.annotate(
                    str(target["lap"]),
                    (px[0], py[0]),
                    color=color_for(drone_id),
                    fontsize=9,
                    fontweight="bold",
                )

    for breach in find_breaches(timeline, series, COLLISION_RADIUS)[:40]:
        a, b = breach.pair
        pa = drones[a].position_at(breach.at)
        if pa:
            bx, by = to_xy([pa])
            ax.scatter(bx, by, s=110, facecolors="none", edgecolors="red", lw=1.5, zorder=5)

    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(
        "Flown paths (solid) vs planned waypoints (x). "
        "Numbers mark lap starts; red rings mark separation breaches."
    )
    ax.grid(alpha=0.3)
    ax.legend()
    path = out_dir / "paths_overview.png"
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    written.append(path.name)

    # --- per-lap panels ---------------------------------------------------
    all_laps = sorted({lap for log in drones.values() for lap in lap_tracks(log)})
    for lap in all_laps:
        fig, ax = plt.subplots(figsize=(9, 9))
        for drone_id in sorted(drones):
            log = drones[drone_id]
            track = lap_tracks(log).get(lap)
            if track:
                xs, ys = to_xy(track)
                ax.plot(xs, ys, color=color_for(drone_id), lw=1.6, label=f"drone {drone_id}")

            _, _, waypoints = planned_circle(log)
            lap_wps = [w for w in waypoints if w.get("lap") == lap]
            if lap_wps:
                wx, wy = to_xy([(w["lat"], w["lon"]) for w in lap_wps])
                ax.plot(
                    wx + wx[:1],
                    wy + wy[:1],
                    color=color_for(drone_id),
                    ls="--",
                    lw=0.8,
                    alpha=0.5,
                )
                for w, x, y in zip(lap_wps, wx, wy):
                    if w["idx"] % 9 == 0:
                        ax.annotate(str(w["idx"]), (x, y), fontsize=7, color=color_for(drone_id))
        ax.set_aspect("equal")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        ax.set_title(f"Lap {lap} — flown (solid) vs planned circle (dashed)")
        ax.grid(alpha=0.3)
        ax.legend()
        path = out_dir / f"paths_lap_{lap}.png"
        fig.savefig(path, dpi=140, bbox_inches="tight")
        plt.close(fig)
        written.append(path.name)

    # --- separation over time --------------------------------------------
    if timeline:
        fig, ax = plt.subplots(figsize=(13, 5))
        t0 = timeline[0]
        for pair, distances in sorted(series.items()):
            ax.plot([t - t0 for t in timeline], distances, lw=1.0, label=f"{pair[0]}-{pair[1]}")
        ax.axhline(COLLISION_RADIUS, color="orange", ls="--", label="COLLISION_RADIUS")
        ax.axhline(MIN_SEPARATION_M, color="red", ls="--", label="airframe separation")
        ax.set_xlabel("seconds since first common sample")
        ax.set_ylabel("distance (m)")
        ax.set_title("True inter-drone separation")
        ax.grid(alpha=0.3)
        ax.legend(ncol=4, fontsize=8)
        path = out_dir / "separation.png"
        fig.savefig(path, dpi=140, bbox_inches="tight")
        plt.close(fig)
        written.append(path.name)

    return written


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------


def write_report(
    drones: dict[int, DroneLog],
    out_dir: Path,
    timeline: list[float],
    series: dict[tuple[int, int], list[float]],
    plots: list[str],
) -> Path:
    lines: list[str] = []
    add = lines.append

    add(f"# Flight analysis — {out_dir.name}\n")
    add(f"- drones: {sorted(drones)}")
    add(f"- COLLISION_RADIUS: {COLLISION_RADIUS:.2f} m")
    add(f"- airframe separation: {MIN_SEPARATION_M:.2f} m")
    if plots:
        add(f"- plots: {', '.join(plots)}")
    add("")

    # --- geometry sanity --------------------------------------------------
    add("## Planned geometry\n")
    add("| drone | center lat | center lon | radius m | pts/lap | spacing m | laps |")
    add("|---|---|---|---|---|---|---|")
    spacings: list[float] = []
    for drone_id in sorted(drones):
        plans = drones[drone_id].of_kind("poif_plan")
        if not plans:
            continue
        p = plans[-1]
        per_lap = p["points_per_lap"]
        spacing = 2 * p["radius_m"] * math.sin(math.pi / per_lap)
        spacings.append(spacing)
        add(
            f"| {drone_id} | {p['center'][0]:.7f} | {p['center'][1]:.7f} |"
            f" {p['radius_m']:.1f} | {per_lap} | {spacing:.2f} | {p['laps']} |"
        )
    add("")
    if spacings and min(spacings) < COLLISION_RADIUS:
        add(
            f"> Waypoint spacing ({min(spacings):.2f} m) is below COLLISION_RADIUS"
            f" ({COLLISION_RADIUS:.2f} m), so a swept-path test between two drones on"
            " the same circle can never pass. Expected: the drones fly deliberately"
            " intersecting circles, and separation is guaranteed by the lockstep"
            " barrier instead. COLLISION_RADIUS applies only to drones outside that"
            " barrier's window. A gridlock here means the barrier is not being"
            " applied, not that the spacing is wrong.\n"
        )

    centers = [
        tuple(drones[d].of_kind("poif_plan")[-1]["center"])
        for d in sorted(drones)
        if drones[d].of_kind("poif_plan")
    ]
    if len(centers) > 1:
        spread = max(meters_between(a, b) for i, a in enumerate(centers) for b in centers[i + 1 :])
        add(f"Max distance between circle centers: **{spread:.2f} m**\n")

    # --- true separation --------------------------------------------------
    add("## Actual separation (from raw position telemetry)\n")
    if not series:
        add("No overlapping position data — separation could not be computed.\n")
    else:
        add("| pair | min m | mean m | time under radius | time under airframe |")
        add("|---|---|---|---|---|")
        for pair, distances in sorted(series.items()):
            valid = [d for d in distances if not math.isnan(d)]
            if not valid:
                continue
            under_r = sum(1 for d in valid if d < COLLISION_RADIUS) * SEPARATION_STEP_S
            under_a = sum(1 for d in valid if d < MIN_SEPARATION_M) * SEPARATION_STEP_S
            add(
                f"| {pair[0]}-{pair[1]} | **{min(valid):.2f}** |"
                f" {sum(valid) / len(valid):.2f} | {under_r:.1f}s | {under_a:.1f}s |"
            )
        add("")

        breaches = find_breaches(timeline, series, COLLISION_RADIUS)
        add(f"### Breach episodes: {len(breaches)}\n")
        if breaches:
            t0 = timeline[0]
            add("| pair | t+start | duration | closest m | actual collision |")
            add("|---|---|---|---|---|")
            for b in breaches[:30]:
                add(
                    f"| {b.pair[0]}-{b.pair[1]} | {b.start - t0:.1f}s |"
                    f" {b.end - b.start:.1f}s | **{b.closest:.2f}** |"
                    f" {'YES' if b.closest < MIN_SEPARATION_M else 'no'} |"
                )
            add("")

    # --- per drone --------------------------------------------------------
    add("## Per-drone behaviour\n")
    for drone_id in sorted(drones):
        log = drones[drone_id]
        add(f"### Drone {drone_id}\n")

        arrivals = log.of_kind("wp_reached")
        holds = log.of_kind("hold_start")
        ends = log.of_kind("hold_end")
        add(f"- waypoints reached: {len(arrivals)}")
        add(f"- holds entered: {len(holds)}, holds released: {len(ends)}")

        if len(ends) < len(holds):
            add(
                f"- **{len(holds) - len(ends)} hold(s) never released** — this drone"
                " was still waiting when the log ended (deadlock candidate)"
            )

        if ends:
            durations = [e["held_for"] for e in ends]
            durations.sort()
            add(
                f"- hold duration: total {sum(durations):.1f}s,"
                f" median {durations[len(durations) // 2]:.2f}s,"
                f" max {durations[-1]:.2f}s"
            )

        if arrivals:
            errors = sorted(e["error_m"] for e in arrivals)
            legs = sorted(e["leg_seconds"] for e in arrivals)
            add(
                f"- arrival error vs waypoint: median {errors[len(errors) // 2]:.2f} m,"
                f" max {errors[-1]:.2f} m"
            )
            add(f"- leg flight time: median {legs[len(legs) // 2]:.2f}s," f" max {legs[-1]:.2f}s")
            last = arrivals[-1].get("target") or {}
            add(f"- last waypoint reached: lap {last.get('lap')} index {last.get('idx')}")

        # Reports about waypoints we were never told about leave the peer's
        # believed position frozen, which silently defeats avoidance.
        unmatched = [e for e in log.of_kind("reached_recv") if not e.get("matched")]
        if unmatched:
            by_peer: dict[int, int] = defaultdict(int)
            for e in unmatched:
                by_peer[e["peer"]] += 1
            add(
                f"- **{len(unmatched)} unmatched 'reached' reports**"
                f" (per peer: {dict(by_peer)}) — believed peer position went stale"
            )

        mismatched = [
            e
            for e in log.of_kind("waypoints_recv")
            if e.get("checksum") != e.get("sender_checksum")
        ]
        if mismatched:
            add(f"- **{len(mismatched)} waypoint checksum mismatches** with peers")

        # Conflicts deliberately not acted on. Old logs mark peers ignored by
        # the (since removed) ID yielding rule with yielded_to=False; new logs
        # mark peers dropped for silence with stale=True.
        ignored_yield = 0
        ignored_stale = 0
        for e in log.of_kind("leg_check"):
            for peer in e.get("peers") or []:
                clearance = peer.get("clearance")
                if clearance is None or clearance >= e["radius"]:
                    continue
                if peer.get("stale"):
                    ignored_stale += 1
                elif "yielded_to" in peer and not peer["yielded_to"]:
                    ignored_yield += 1
        if ignored_yield:
            add(
                f"- **{ignored_yield} conflicts ignored by the yielding rule** (peer had a"
                " lower ID, so this drone did not hold for it)"
            )
        if ignored_stale:
            add(
                f"- **{ignored_stale} conflicts ignored because the peer was stale**"
                " (silent past the staleness timeout, so presumed landed/crashed)"
            )
        add("")

    # --- believed vs actual ----------------------------------------------
    add("## Believed peer position vs actual\n")
    add(
        "How wrong each drone's model of its peers was. Avoidance acts on the"
        " belief, so a large error here means the algorithm was solving the"
        " wrong problem regardless of whether its own logic is sound.\n"
    )
    add("| drone | peer | samples | median err m | p95 err m | max err m |")
    add("|---|---|---|---|---|---|")
    for drone_id in sorted(drones):
        errors: dict[int, list[float]] = defaultdict(list)
        for event in drones[drone_id].of_kind("pos"):
            for belief in event.get("beliefs") or []:
                peer = belief["peer"]
                occupied = belief.get("occupied")
                if not occupied or peer not in drones:
                    continue
                actual = drones[peer].position_at(event["t"])
                if actual is None:
                    continue
                # Believed location is a segment; the error is the distance from
                # the peer's true position to the nearest point on it.
                errors[peer].append(
                    min(
                        meters_between(actual, tuple(occupied[0])),
                        meters_between(actual, tuple(occupied[1])),
                    )
                )
        for peer, values in sorted(errors.items()):
            if not values:
                continue
            values.sort()
            add(
                f"| {drone_id} | {peer} | {len(values)} |"
                f" {values[len(values) // 2]:.2f} |"
                f" {values[int(len(values) * 0.95)]:.2f} | {values[-1]:.2f} |"
            )
    add("")

    # --- timeline of decisions -------------------------------------------
    add("## Merged event timeline (holds, arrivals, breaches)\n")
    add("```")
    merged: list[tuple[float, str]] = []
    for drone_id, log in drones.items():
        base = log.times[0] if log.times else 0.0
        for e in log.of_kind("hold_start", "hold_end", "hold_blockers_changed", "poif_complete"):
            if e["kind"] == "hold_start":
                blockers = ",".join(str(c["peer_id"]) for c in e["conflicts"])
                closest = min(c["clearance_m"] for c in e["conflicts"])
                target = e.get("target") or {}
                merged.append(
                    (
                        e["t"],
                        f"d{drone_id} HOLD  for [{blockers}] closest={closest:.2f}m"
                        f" heading to L{target.get('lap')}P{target.get('idx')}",
                    )
                )
            elif e["kind"] == "hold_end":
                target = e.get("target") or {}
                merged.append(
                    (
                        e["t"],
                        f"d{drone_id} CLEAR after {e['held_for']:.1f}s"
                        f" -> L{target.get('lap')}P{target.get('idx')}",
                    )
                )
            elif e["kind"] == "hold_blockers_changed":
                merged.append((e["t"], f"d{drone_id} blockers {e['was']} -> {e['now']}"))
            else:
                merged.append((e["t"], f"d{drone_id} POIF COMPLETE"))

    for b in find_breaches(timeline, series, COLLISION_RADIUS):
        merged.append(
            (
                b.at,
                f"!! BREACH {b.pair[0]}-{b.pair[1]} closest {b.closest:.2f}m"
                f" for {b.end - b.start:.1f}s",
            )
        )

    merged.sort()
    # Same origin as separation.png so the two can be read side by side.
    origin = timeline[0] if timeline else (merged[0][0] if merged else 0.0)
    for t, text in merged[:400]:
        add(f"{t - origin:8.2f}  {text}")
    if len(merged) > 400:
        add(f"... {len(merged) - 400} more events")
    add("```")

    path = out_dir / "report.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Logs/<run_id> directory to analyze")
    parser.add_argument(
        "--out", type=Path, default=None, help="Where to write output (default: run_dir)"
    )
    args = parser.parse_args()

    out_dir = args.out or args.run_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    drones = load_run(args.run_dir)
    print(f"loaded {len(drones)} drone(s): {sorted(drones)}")
    for drone_id, log in sorted(drones.items()):
        print(f"  drone {drone_id}: {len(log.events)} events, {len(log.times)} position samples")

    timeline, series = true_separation(drones)
    plots = make_plots(drones, out_dir, timeline, series)
    report = write_report(drones, out_dir, timeline, series, plots)
    print(f"\nwrote {report}")
    for name in plots:
        print(f"wrote {out_dir / name}")


if __name__ == "__main__":
    main()
