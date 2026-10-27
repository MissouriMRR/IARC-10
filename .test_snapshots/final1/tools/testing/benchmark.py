"""Seed-76 benchmark: how good and how fast is a mission on one fixed field?

Takes: --mines N (90), --seed S (76), --drones D (4).
Does:  runs the mission for two photo shapes (planning 6x4 ft, real 8.52x6.44 ft footprint)
       x two outer-ring settings (0, 2) and prints, per run: makespan (longest single-drone
       flight, ft), photos, route length (ft), route cells in danger, route squares on a
       mine, blocked squares, complete, CPU seconds.
Returns: nothing; compare the numbers before and after a change.

Examples:
  python tools/testing/benchmark.py
  python tools/testing/benchmark.py --drones 2 --seed 3
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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--mines", type=int, default=90)
    ap.add_argument("--seed", type=int, default=76)
    ap.add_argument("--drones", type=int, default=4, choices=(1, 2, 3, 4))
    args = ap.parse_args()
    for shape in ((6.0, 4.0), (8.52, 6.44)):
        for outer in (0, 2):
            t = time.process_time()
            m = Mission(g(args.mines, args.seed), drones=args.drones, outer_ring=outer,
                        shape_size_ft=shape).run()
            e, s = evaluate(m), m.summary()
            route = m.report_route()
            print(f"shape={shape} outer={outer}: failure={m.failure} makespan={s['makespan']:.0f} "
                  f"photos={s['photos']} route={poly_len(route) if route else 0:.1f} "
                  f"danger={e.get('route_cells_in_danger')} on_mine={e.get('route_mine_squares')} "
                  f"blocked={s.get('blocked_squares')} complete={e.get('complete')} "
                  f"cpu={time.process_time() - t:.1f}s", flush=True)


if __name__ == "__main__":
    main()
