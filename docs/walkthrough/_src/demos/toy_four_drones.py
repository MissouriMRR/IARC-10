"""Toy run: the headless simulator (pathnode/sim.py) with 4 drones on a 24 ft x 80 ft field and
six mines. Each side ends up with the same route; drone 1 reports the joined one."""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.sim import Mission

mines = [(12, 20), (9, 26), (16, 31), (12, 44), (14, 52), (10, 60)]
m = Mission(mines, drones=4, field_w=24.0, field_h=80.0, entry_x=12.0,
            shape_size_ft=(8.52, 6.44), max_ticks=1500).run()
s = m.summary()
print("failure:", s["failure"], "| done:", s["done"], "| photos:", s["photos"], "| ticks:", s["ticks"])
for d in (1, 2, 3, 4):
    p = m.planners[d]
    print(f"drone {d}: met={p.met} done={p.done()} stats={ {k: v for k, v in p.stats.items() if v} }")
    print("   events:", [(e[0], tuple(round(v, 1) for v in e[1])) for e in p.events if e[0] == 'jump'])
print("route drone 1 reports:", [(round(x, 1), round(y, 1)) for x, y in m.report_route()])
