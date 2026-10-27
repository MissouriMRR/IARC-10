"""Run test_every_drone_count_completes_with_a_valid_route's 48 missions in parallel."""
import sys
from multiprocessing import Pool

from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.sim import Mission, evaluate


def one(args):
    drones, seed, n = args
    m = Mission(generate_true_minefield(n, seed), drones=drones).run()
    e = evaluate(m)
    ok = (e["complete"] and e["joined"] and e["spans_field"] and e["unseen_cells"] == 0
          and e["bad_edges_known"] == 0 and e["route_mine_squares"] == 0)
    return args, ok, m.failure


if __name__ == "__main__":
    jobs = [(d, s, n) for d in (1, 2, 3, 4) for s in range(1, 7) for n in (70, 90)]
    bad = 0
    with Pool(6) as pool:
        for args, ok, failure in pool.imap_unordered(one, jobs):
            if not ok:
                bad += 1
                print("FAIL", args, failure, flush=True)
    print("done, failures:", bad)
