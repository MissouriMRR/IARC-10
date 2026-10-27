"""Toy run: the SIDE-MATE finds a mine while drone 1 is in the middle of a leg.

Two planner copies on one side (drones 1 and 2), field 20 ft x 60 ft, no partner side.
  - drone 2 is pioneering the forward gap (it announced that job); drone 1 is behind it, at
    (11.0, 9.9), flying ring photos;
  - drone 2 photographs (11.0, 21.5) and finds a mine at (12.0, 22.5).
What drone 1's NEXT planning pass does depends on whether SHARE_PHOTOS has been applied yet.
"""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.session import PathNodeBackend
from flight.pathfinding.pathnode.config import Edge, PlannerConfig, GROUPS

ALONG, ACROSS = 8.52, 6.44


def footprint(x, y):
    return [(x - ACROSS / 2, y - ALONG / 2), (x + ACROSS / 2, y - ALONG / 2),
            (x + ACROSS / 2, y + ALONG / 2), (x - ACROSS / 2, y + ALONG / 2)]


def make():
    return PathNodePlanner(20.0, 60.0, 10.0, Edge.BOTTOM, drones=(1, 2),
                           config=PlannerConfig(shape_size_ft=(ALONG, ACROSS), overlap=0.1))


def chain(p):
    out = f"{p.nodes[0].placed_by.value}({p.nodes[0].x:.1f},{p.nodes[0].y:.1f})"
    for k, n in zip(p.links, p.nodes[1:]):
        out += f" -{k.kind.value}- {n.placed_by.value}({n.x:.1f},{n.y:.1f})"
    return out


def queue(plan):
    return [(round(x, 1), round(y, 1)) for x, y, _g in plan][:6]


A, B = make(), make()                    # A = drone 1's copy, B = drone 2's copy
sa, sb = PathNodeBackend.from_planner(A), PathNodeBackend.from_planner(B)
# both copies have seen drone 1's photos up to (11.0, 9.9) (drone 1 shared them earlier)
for y in (2.7, 9.9):
    for s in (sa, sb):
        s.planner.record_footprint(footprint(11.0, y)); s.planner.advance()
pos = {1: (11.0, 9.9), 2: (16.0, 22.5)}
mate_job = {2: (0, 'forward', (11.0, 21.5))}      # drone 2 announced: SIDE_JOB (path 'forward', next photo (11.0, 21.5))

plan = sa.assignments(pos, mate_job, 1)
print("drone 1's copy, before anything arrives")
print("  chain:", chain(A))
print("  drone 1 queue:", queue(plan[1]), "| drone 2 queue:", queue(plan[2]))

print("\ndrone 2 photographs (11.0, 21.5) and finds a mine at (12.0, 22.5)")
sb.own_footprint(footprint(11.0, 21.5))
out = sb.own_photo(None, [(12.0, 22.5)])
print("  drone 2's own copy: ", chain(B))
print("  drone 2 stages for drone 1: SHARE_PHOTOS(corners, mines=[(12.0, 22.5)])")

print("\n(drone 1 finishes its leg, photographs, then CalcScanPath runs apply_inbox -> mate_photo)")
sa.mate_photo(footprint(11.0, 21.5), [(12.0, 22.5)])
print("  drone 1's copy:     ", chain(A))
print("  same mine set on both copies:", A.mine_hash() == B.mine_hash())
plan = sa.assignments(pos, mate_job, 1)
print("  drone 1 queue now:  ", queue(plan[1]), "| drone 2 queue:", queue(plan[2]))
for i, g in enumerate(A.groups()):
    for key, seq in g:
        if i == 0:
            print(f"  group {GROUPS[i]}: key={key!s:<26} {len(seq)} photo(s), first {tuple(round(v, 1) for v in seq[0])}")
print("  drone 1 stats:", {k: v for k, v in A.stats.items() if v})
