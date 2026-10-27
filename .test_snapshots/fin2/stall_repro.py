"""Reproduce a planner-sim stall and show who is blocked (drones, seed, n from argv)."""
import sys

from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.sim import Mission

drones, seed, n = 4, int(sys.argv[1]), 90
m = Mission(generate_true_minefield(n, seed), drones=drones, drop=0.5, delay=3).run()
print("failure:", m.failure, "tick", m.tick)
for d, pl in sorted(m.planners.items()):
    print("drone", d, "pos", m.pos[d], "last_plan", pl.coordinator.last_plan, "heads", pl.claims.heads,
          "done", pl.done(), "met", pl.met)
    print("   groups", [[(str(k)[:30], [(round(a, 1), round(b, 1)) for a, b in seq][:3]) for k, seq in g][:3]
                        for g in pl.groups()])
