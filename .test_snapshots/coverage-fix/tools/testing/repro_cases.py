"""Named hard cases for the planner (each once broke it).

Takes: optional case names (default: all). --list shows them.
Does:  runs each case as a simulated mission and prints one line of results.
Returns: exit code 0 if every case finished with a clean route, else 1.

Cases:
  crosswise, crosswise_loss  the two sides meet crosswise (also with 50% relay loss)
  walled_entry               1 drone, entry point boxed in by mines
  rotated45, rotated_random  photo footprints turned by the drone's heading

Examples:
  python tools/testing/repro_cases.py
  python tools/testing/repro_cases.py walled_entry rotated45
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402,F401

from flight.pathfinding.pathnode.minefield import generate_true_minefield as g  # noqa: E402
from flight.pathfinding.pathnode.polyline import poly_len  # noqa: E402
from flight.pathfinding.pathnode.sim import Mission, evaluate  # noqa: E402

CASES = {
    "crosswise": dict(mines=90, seed=22, drones=4, kw={}),
    "crosswise_loss": dict(mines=90, seed=220, drones=4, kw=dict(drop=0.5, delay=3, seed=220)),
    "walled_entry": dict(mines=120, seed=13, drones=1, kw={}),
    "rotated45": dict(mines=30, seed=0, drones=2, kw=dict(photo_yaw_deg=45.0, max_ticks=1200)),
    "rotated_random": dict(mines=30, seed=1, drones=2,
                           kw=dict(photo_yaw_deg="random", max_ticks=1200)),
}


def run_case(name):
    """Takes: a case name. Returns: (ok, printable line)."""
    c = CASES[name]
    t0 = time.process_time()
    m = Mission(g(c["mines"], c["seed"]), drones=c["drones"], **c["kw"]).run()
    e, s, route = evaluate(m), m.summary(), m.report_route()
    ok = bool(not m.failure and e.get("complete") and e.get("spans_field")
              and not e.get("unseen_cells") and not e.get("bad_edges_known")
              and not e.get("route_mine_squares"))
    line = (f"{name}: {'ok' if ok else 'BAD'} failure={m.failure} ticks={m.tick} "
            f"makespan={s['makespan']:.0f} photos={s['photos']} "
            f"route={'none' if not route else round(poly_len(route))} "
            f"complete={e.get('complete')} joined={e.get('joined')} spans={e.get('spans_field')} "
            f"unseen={e.get('unseen_cells')} known_crossings={e.get('bad_edges_known')} "
            f"on_mine={e.get('route_mine_squares')} cpu={time.process_time() - t0:.1f}s")
    return ok, line


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("cases", nargs="*", help="case names (default: all)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    if args.list:
        print("\n".join(CASES))
        return 0
    unknown = [c for c in args.cases if c not in CASES]
    if unknown:
        ap.error(f"unknown case(s) {unknown}; see --list")
    all_ok = True
    for name in args.cases or CASES:
        ok, line = run_case(name)
        all_ok = all_ok and ok
        print(line, flush=True)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
