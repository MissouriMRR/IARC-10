"""Toy run: a shortcut trial from start to finish, on drone 1's planner copy.

Headless two-drone mission (pathnode/sim.py), 40 ft x 120 ft field, 16 seeded random mines. A hook
on Shortcuts.update prints every time drone 1's trials or the shortcut events change. 'photos'
is coverage.photos_needed(poly): how many photos the trial still needs."""
import random
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode import shortcuts as sc
from flight.pathfinding.pathnode.polyline import poly_len
from flight.pathfinding.pathnode.sim import Mission

rng = random.Random(16)
mines = [(rng.uniform(6, 34), rng.uniform(6, 114)) for _ in range(16)]
m = Mission(mines, drones=2, field_w=40.0, field_h=120.0, entry_x=20.0,
            shape_size_ft=(8.52, 6.44), max_ticks=3000)
watched = m.planners[1]
state = {"last": None, "events": 0}
orig = sc.Shortcuts.update


def hooked(self, met):
    orig(self, met)
    if self is not watched.shortcuts:
        return
    now = []
    for tier, tr in self.trials.items():
        if tr is not None:
            now.append((tier.value, tuple(round(v, 1) for v in tr.a.xy), tuple(round(v, 1) for v in tr.b.xy),
                        f"section {tr.section:.1f} -> {poly_len(tr.poly):.1f} ft",
                        f"photos {self.coverage.photos_needed(tr.poly)}"))
    new_events = [e for e in watched.events if e[0].startswith("shortcut")][state["events"]:]
    state["events"] += len(new_events)
    if now != state["last"] or new_events:
        print(f"tick {m.tick:>2}  trials={now}")
        for e in new_events:
            print("         event:", e[0], *[tuple(round(v, 1) for v in x) if isinstance(x, tuple) else
                                             (round(x, 1) if isinstance(x, float) else x) for x in e[1:]])
        state["last"] = now


sc.Shortcuts.update = hooked
m.run()
print("stats:", {k: v for k, v in watched.stats.items() if v})
