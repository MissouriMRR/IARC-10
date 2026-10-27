"""Toy run: what the NEXT planning pass gives drone 1 after it finds a mine, and after the photo
that follows it. One side (drones 1 and 2), field 20 ft x 60 ft. Drone 2 is at (19.5, 0) throughout
and is 'idle' (it has announced no job)."""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.session import PathNodeBackend
from flight.pathfinding.pathnode.config import Edge, PlannerConfig, GROUPS

ALONG, ACROSS = 8.52, 6.44


def footprint(x, y):
    return [(x - ACROSS / 2, y - ALONG / 2), (x + ACROSS / 2, y - ALONG / 2),
            (x + ACROSS / 2, y + ALONG / 2), (x - ACROSS / 2, y + ALONG / 2)]


P = PathNodePlanner(20.0, 60.0, 10.0, Edge.BOTTOM, drones=(1, 2),
                    config=PlannerConfig(shape_size_ft=(ALONG, ACROSS), overlap=0.1))
S = PathNodeBackend.from_planner(P)  # drone 1's copy
pos = {1: (10.0, 0.0), 2: (19.5, 0.0)}
mate_idle = {2: None}                     # drone 2 announced "no job"


def chain():
    out = f"{P.nodes[0].placed_by.value}({P.nodes[0].x:.1f},{P.nodes[0].y:.1f})"
    for k, n in zip(P.links, P.nodes[1:]):
        out += f" -{k.kind.value}- {n.placed_by.value}({n.x:.1f},{n.y:.1f})"
    return out


def report(title):
    print(f"--- {title}")
    print("chain :", chain())
    for i, g in enumerate(P.groups()):
        for key, seq in g:
            head = f"{len(seq)} photo(s), first {tuple(round(v, 1) for v in seq[0])}"
            print(f"group {i} {GROUPS[i]:<12} key={key!s:<28} {head}")
    plan = S.assignments(pos, mate_idle, 1)
    for d in (1, 2):
        print(f"drone {d} queue:", [(round(x, 1), round(y, 1), GROUPS[g]) for x, y, g in plan[d]][:5])
    print("claims (paths only):", {(GROUPS[g], sid[0] if isinstance(sid, tuple) else sid): d for (g, sid), d in P.claims.claims.items() if not (isinstance(sid, tuple) and sid[0] == "loose")})
    print("drone 1 head (announced):", P.claims.heads[1] and (GROUPS[P.claims.heads[1][0]], P.claims.heads[1][1]))
    return plan


plan = report("pass 1: start")
x, y, _ = plan[1][0]
S.own_footprint(footprint(x, y)); S.own_photo(None, []); pos[1] = (x, y)
plan = report(f"pass 2: after a photo at ({x:.1f}, {y:.1f}), no mine")
x, y, _ = plan[1][0]
S.own_footprint(footprint(x, y)); out = S.own_photo(None, [(x + 1.0, y + 1.0)]); pos[1] = (x, y)
print(f"\n*** photo at ({x:.1f}, {y:.1f}) contains a mine at ({x + 1.0:.1f}, {y + 1.0:.1f}); new mines for the other side: {len(out.mines)}\n")
plan = report("pass 3: right after the mine")
x, y, g = plan[1][0]
print(f"\n*** drone 1 flies to ({x:.1f}, {y:.1f}) [{GROUPS[g]}], photo has no mine\n")
S.own_footprint(footprint(x, y)); S.own_photo(None, []); pos[1] = (x, y)
plan = report("pass 4: after the span's photo")
