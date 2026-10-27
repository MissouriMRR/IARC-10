"""Multi-seed mission sweep: does the planner finish safely on many random minefields?

Takes: --standard (the 90-seed set), or --mines N --seeds A B [--drop P --delay T --drones D].
       --levels adds the expanding-mines loop (levels, snapshots, lockstep) to every mission:
       [--engine NAME --max-levels N --tick-budget T].
Does:  runs one simulated mission per seed (flight/pathfinding/pathnode/sim.py), in parallel,
       and prints one line per seed: ok or BAD, plus the numbers behind it. With --levels the
       line also shows the levels reached per side, snapshots, the reported level, buffer_width
       and growth time, and a table of their means follows.
Returns: exit code 0 if every seed is ok, 1 if any is BAD.

A seed is BAD when: the mission failed or did not finish, the route does not span the field,
it crosses a known mine, runs over a true mine square, or covers ground nobody photographed.
With --levels also: no valid snapshot was reported, or the sides ended more than one level apart.

Examples:
  python tools/testing/sweep.py --standard
  python tools/testing/sweep.py --mines 90 --seeds 0 2
  python tools/testing/sweep.py --mines 90 --seeds 100 103 --drop 0.5 --delay 3
  python tools/testing/sweep.py --levels --engine field --max-levels 4 --mines 90 --seeds 0 6
"""
import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402,F401  (puts the repo root on sys.path)

# tag: (mines, first seed, last seed + 1, drop, delay). Six configs x 15 seeds = 90 missions.
STANDARD = {
    "a": (90, 0, 15, 0.0, 0),
    "b": (90, 15, 30, 0.0, 0),
    "c": (120, 0, 15, 0.0, 0),        # denser field
    "d": (90, 100, 115, 0.5, 3),      # lossy: half the relays dropped, up to 3 ticks late
    "e": (90, 115, 130, 0.5, 3),
    "f": (60, 200, 215, 0.3, 5),
}

_MIN = {"cross": math.inf, "side": math.inf}
_patched = False


def _track_separation():
    """Wrap Mission._step so every tick records the closest approach between drones."""
    global _patched
    if _patched:
        return
    from flight.pathfinding.pathnode.sim import Mission
    orig = Mission._step

    def step(self, drone):
        result = orig(self, drone)
        for other in self.pos:
            if other != drone:
                kind = "side" if self.side_of[other] == self.side_of[drone] else "cross"
                _MIN[kind] = min(_MIN[kind], math.dist(self.pos[other], self.pos[drone]))
        return result

    Mission._step = step
    _patched = True


def run_seed(job):
    """Takes: (tag, mines, seed, drop, delay, drones[, levels]) where levels is None or a dict of
    Mission keywords (levels=True, engine, max_levels, tick_budget).
    Returns: (bad, printable line, the level metrics or None)."""
    tag, mines, seed, drop, delay, drones, *rest = job
    levels = rest[0] if rest else None
    from flight.pathfinding.pathnode.minefield import generate_true_minefield
    from flight.pathfinding.pathnode.sim import Mission, evaluate
    _track_separation()
    _MIN.update(cross=math.inf, side=math.inf)
    kw = dict(drop=drop, delay=delay, seed=seed) if drop or delay else {}
    if levels:
        kw.update(levels=True, max_levels=levels["max_levels"], tick_budget=levels["tick_budget"],
                  planner_options={"engine": levels["engine"]})
    m = Mission(generate_true_minefield(mines, seed), drones=drones, **kw).run()
    e = evaluate(m)
    bad = bool(m.failure or not e.get("complete") or not e.get("spans_field")
               or e.get("unseen_cells") or e.get("bad_edges_known") or e.get("route_mine_squares"))
    metrics = _level_metrics(m) if levels else None
    if metrics:
        bad = bad or not metrics["valid"] or metrics["skew"] > 1
    line = (f"{tag} seed={seed} {'BAD' if bad else 'ok'} failure={m.failure} "
            f"complete={e.get('complete')} spans={e.get('spans_field')} "
            f"unseen={e.get('unseen_cells')} known={e.get('bad_edges_known')} "
            f"on_mine={e.get('route_mine_squares')} min_cross={_MIN['cross']:.2f} "
            f"min_side={_MIN['side']:.2f} ticks={m.tick}")
    if metrics:
        line += (f" levels={metrics['levels']} snaps={metrics['snaps']}"
                 f" reported={metrics['reported']} buffer={metrics['buffer']}"
                 f" valid={metrics['valid']} growth_s={metrics['growth_s']}"
                 f" growths={metrics['growths']}")
    return bad, line, metrics


def _level_metrics(m):
    """The expanding-mines numbers of a finished Mission (sim_levels.LevelDriver.summary)."""
    s = m.summary()
    levels = s["levels"]
    return {"levels": "/".join(str(v) for v in levels.values()), "skew": max(levels.values()) - min(levels.values()),
            "level_min": min(levels.values()), "snaps": "/".join(str(v) for v in s["snapshots"].values()),
            "reported": s["reported_level"], "buffer": s["buffer_width"],
            "valid": all(b.report_valid() for sp, b in m.backends.items() if sp in m.sides),
            "growth_s": s["growth_s"], "growths": s["growths"]}


def _print_level_table(rows):
    """Means over the missions that ran with levels (rows: their metric dicts)."""
    n = len(rows)
    if not n:
        return

    def mean(key):
        vals = [r[key] for r in rows if r[key] is not None]
        return sum(vals) / len(vals) if vals else float("nan")

    growths = sum(r["growths"] for r in rows)
    print(f"\nlevels table ({n} missions): level reached (min over sides) mean {mean('level_min'):.2f}"
          f" | reported level mean {mean('reported'):.2f} | buffer_width mean {mean('buffer'):.2f}"
          f" | valid snapshot reported {sum(r['valid'] for r in rows)}/{n}"
          f" | growth seconds per level mean {mean('growth_s'):.3f}"
          f" max {max(r['growth_s'] for r in rows):.3f} ({growths} growths)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--standard", action="store_true", help="run the 90-seed standard set")
    ap.add_argument("--mines", type=int, default=90, help="mines per field (default 90)")
    ap.add_argument("--seeds", type=int, nargs=2, metavar=("FIRST", "END"), default=[0, 5],
                    help="seeds FIRST..END-1 (default 0 5)")
    ap.add_argument("--drop", type=float, default=0.0, help="fraction of relays dropped")
    ap.add_argument("--delay", type=int, default=0, help="max ticks a relay is delayed")
    ap.add_argument("--drones", type=int, default=4, choices=(1, 2, 3, 4))
    ap.add_argument("--tag", default="x", help="label printed on each line")
    ap.add_argument("--levels", action="store_true",
                    help="run the expanding-mines loop and report its metrics")
    ap.add_argument("--engine", default="zonemap", help="with --levels: the path engine")
    ap.add_argument("--max-levels", type=int, default=3,
                    help="with --levels: never grow past this level (default 3)")
    ap.add_argument("--tick-budget", type=int, default=None,
                    help="with --levels: the mission's time limit in ticks (default: max_ticks)")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2,
                    help="parallel processes (default: CPU count)")
    args = ap.parse_args()

    extra = ([dict(engine=args.engine, max_levels=args.max_levels, tick_budget=args.tick_budget)]
             if args.levels else [])
    if args.standard:
        jobs = [(t, n, s, d, dl, 4, *extra) for t, (n, s0, s1, d, dl) in STANDARD.items()
                for s in range(s0, s1)]
    else:
        jobs = [(args.tag, args.mines, s, args.drop, args.delay, args.drones, *extra)
                for s in range(*args.seeds)]
    bad_count = 0
    rows = []
    with ProcessPoolExecutor(max_workers=max(1, min(args.jobs, len(jobs)))) as pool:
        for bad, line, metrics in pool.map(run_seed, jobs):
            bad_count += bad
            print(line, flush=True)
            if metrics:
                rows.append(metrics)
    print(f"\n{len(jobs) - bad_count}/{len(jobs)} ok, {bad_count} BAD")
    _print_level_table(rows)
    return 1 if bad_count else 0


if __name__ == "__main__":
    sys.exit(main())
