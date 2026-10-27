"""Toy run: ONE side (drones 1 and 2, bottom edge) on a 20 ft x 60 ft field, no partner side.

Drives two PathNodePlanner copies through PathNodeBackend, the same calls SideAgent makes in
flight. Drone 1's copy is A, drone 2's copy is B. Numbers are feet in the local frame.
"""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.session import PathNodeBackend
from flight.pathfinding.pathnode.config import Edge, PlannerConfig

ALONG, ACROSS = 8.52, 6.44          # the planning footprint calc_scan_path_impl computes at 2 m


def footprint(x, y):                 # what the camera "saw": a box centred on the photo point
    return [(x - ACROSS / 2, y - ALONG / 2), (x + ACROSS / 2, y - ALONG / 2),
            (x + ACROSS / 2, y + ALONG / 2), (x - ACROSS / 2, y + ALONG / 2)]


def make():
    return PathNodePlanner(20.0, 60.0, 10.0, Edge.BOTTOM, drones=(1, 2),
                           config=PlannerConfig(shape_size_ft=(ALONG, ACROSS), overlap=0.1))


def q(queue):
    return [(round(x, 1), round(y, 1), g) for x, y, g in queue][:4]


def chain(p):
    return [(n.placed_by.value, round(n.x, 1), round(n.y, 1)) for n in p.nodes], \
           [k.kind.value for k in p.links]


A, B = make(), make()
sa, sb = PathNodeBackend.from_planner(A), PathNodeBackend.from_planner(B)
pos = {1: (10.0, 0.0), 2: (16.0, 0.0)}

print("1. start: chain =", chain(A))
pa = A.plan(pos, me=1)
pb = B.plan(pos, me=2)
print("2. A (drone 1's copy) queues:", {d: q(v) for d, v in pa.items()})
print("   B (drone 2's copy) queues:", {d: q(v) for d, v in pb.items()})
print("   A announces heads:", A.claims.heads)
print("   B announces heads:", B.claims.heads)
print("   (the copies disagree: neither has heard the other's job yet)")

x, y, _g = pa[1][0]
sa.own_footprint(footprint(x, y))
sa.own_photo(None, [])
pos[1] = (x, y)
print("3. drone 1 photographs", (round(x, 1), round(y, 1)), "-> A's chain:", chain(A))
pa = A.plan(pos, held={2: B.claims.heads[2]}, me=1)
print("   A replans (holding drone 2's announced job):", {d: q(v) for d, v in pa.items()})

x, y, _g = pa[1][0]
mine = (x + 1.0, y + 1.0)
sa.own_footprint(footprint(x, y))
out = sa.own_photo(None, [mine])
pos[1] = (x, y)
print("4. drone 1 photographs", (round(x, 1), round(y, 1)), "and finds a mine at",
      (round(mine[0], 1), round(mine[1], 1)))
print("   Outbox.mines (to relay to the other side):", [(round(a, 1), round(b, 1)) for a, b in out.mines])
print("   A's chain:", chain(A))
print("   A's stats:", {k: v for k, v in A.stats.items() if v})
for i, grp in enumerate(A.groups()):
    for key, seq in grp:
        print("   A group", i, key, q([(a, b, 0) for a, b in seq]))

sb.mate_photo(footprint(x, y), [mine])
print("5. drone 2's copy hears SHARE_PHOTOS (mate_photo) -> B's chain:", chain(B))
print("   same mine set on both copies:", A.mine_hash() == B.mine_hash())
print("   (B's chain differs from A's: B never heard photo 1, so it has no checked start yet)")
pb = B.plan({1: (x, y), 2: (16.0, 0.0)}, held={1: A.claims.heads.get(1)}, me=2)
print("   B replans:", {d: q(v) for d, v in pb.items()})
