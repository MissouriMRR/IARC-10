"""Toy run: the FIRST planning pass of two side-mates, each on its own planner copy, and what
happens over the next few passes as each hears the other's announced job (SIDE_JOB).

Field 20 ft x 60 ft; drone 1 at the entry (10, 0), drone 2 nine and a half feet to its right. Each copy
plans exactly as SideAgent.plan does: planner.plan(positions, held=<mate's announced job>, me=...).
"""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.config import Edge, PlannerConfig

cfg = PlannerConfig(shape_size_ft=(8.52, 6.44), overlap=0.1)
A = PathNodePlanner(20.0, 60.0, 10.0, Edge.BOTTOM, drones=(1, 2), config=cfg)   # drone 1's copy
B = PathNodePlanner(20.0, 60.0, 10.0, Edge.BOTTOM, drones=(1, 2), config=cfg)   # drone 2's copy
pos = {1: (10.0, 0.0), 2: (19.5, 0.0)}


def q(queue):
    return [(round(x, 1), round(y, 1)) for x, y, _g in queue][:5]


print("groups (same on both copies): ", {i: [(k, len(s)) for k, s in g] for i, g in enumerate(A.groups()) if g})
print("first photos of the forward gap:", [(round(x, 1), round(y, 1)) for x, y in A.groups()[0][0][1]][:6], "...")
job1 = job2 = None
for n in range(1, 4):
    held_a = {2: job2} if n > 1 else {}
    held_b = {1: job1} if n > 1 else {}
    pa = A.plan(pos, held=held_a, me=1)
    pb = B.plan(pos, held=held_b, me=2)
    job1, job2 = A.claims.heads.get(1), B.claims.heads.get(2)
    print(f"pass {n}: drone 1's copy -> drone 1 flies {q(pa[1])}   (drone 2 in that copy: {q(pa[2])})")
    print(f"        drone 2's copy -> drone 2 flies {q(pb[2])}   (drone 1 in that copy: {q(pb[1])})")
    print(f"        announced jobs: drone 1 {job1 and job1[:2]}  drone 2 {job2 and job2[:2]}")
