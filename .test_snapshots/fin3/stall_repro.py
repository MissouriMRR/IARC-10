"""Reproduce a sweep case: stall_repro.py TAG SEED  (TAG d/e: 90 mines drop .5 delay 3; f: 60, .3, 5)."""
import sys

from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.sim import Mission

tag, seed = sys.argv[1], int(sys.argv[2])
mines, drop, delay = {"d": (90, .5, 3), "e": (90, .5, 3), "f": (60, .3, 5)}[tag]
m = Mission(generate_true_minefield(mines, seed), drones=4, drop=drop, delay=delay, seed=seed).run()
print("failure:", m.failure, "tick", m.tick)
for d, pl in sorted(m.planners.items()):
    print("drone", d, "pos", m.pos[d], "last_plan", pl.coordinator.last_plan, "heads", pl.claims.heads,
          "done", pl.done(), "met", pl.met)
    print("   groups", [[(str(k)[:30], [(round(a, 1), round(b, 1)) for a, b in seq][:3]) for k, seq in g][:3]
                        for g in pl.groups()])
