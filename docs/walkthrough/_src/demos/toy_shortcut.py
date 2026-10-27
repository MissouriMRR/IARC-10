"""Toy run: one side, two drones, 14 seeded random mines on a 40 ft x 120 ft field. Prints the
shortcut events drone 1's planner logged (see pathnode/shortcuts.py)."""
import random
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.sim import Mission

rng = random.Random(1)
mines = [(rng.uniform(6, 34), rng.uniform(6, 114)) for _ in range(14)]
m = Mission(mines, drones=2, field_w=40.0, field_h=120.0, entry_x=20.0,
            shape_size_ft=(8.52, 6.44), max_ticks=3000).run()
p = m.planners[1]
print("failure:", m.failure, "| stats:", {k: v for k, v in p.stats.items() if v})
for e in p.events:
    if e[0] == "shortcut":
        kind, tier, a, b, section, new = e
        print(f"{kind}: tier={tier} anchors {tuple(round(v, 1) for v in a)} -> "
              f"{tuple(round(v, 1) for v in b)}  section {section:.1f} ft replaced by {new:.1f} ft")
