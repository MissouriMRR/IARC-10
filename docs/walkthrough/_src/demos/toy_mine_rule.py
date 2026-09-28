"""Toy run: the mine rule (pathnode/mine_rule.py) on one PathNodePlanner, three ways. Field 20 ft x
60 ft, one drone side, no partner. Numbers are feet in the local frame."""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.planner import PathNodePlanner
from flight.pathfinding.pathnode.config import Edge, PlannerConfig, Source

ALONG, ACROSS = 8.52, 6.44


def footprint(x, y):
    return [(x - ACROSS / 2, y - ALONG / 2), (x + ACROSS / 2, y - ALONG / 2),
            (x + ACROSS / 2, y + ALONG / 2), (x - ACROSS / 2, y + ALONG / 2)]


def make():
    return PathNodePlanner(20.0, 60.0, 10.0, Edge.BOTTOM, drones=(1,),
                           config=PlannerConfig(shape_size_ft=(ALONG, ACROSS), overlap=0.1))


def show(label, p):
    nodes = [f"{n.placed_by.value}({n.x:.1f},{n.y:.1f})" for n in p.nodes]
    kinds = [k.kind.value for k in p.links]
    out = nodes[0]
    for k, n in zip(kinds, nodes[1:]):
        out += f" -{k}- {n}"
    print(f"{label}: {out}")
    print(f"      stats: { {k: v for k, v in p.stats.items() if v} }  failure={p.failure}")


def fly(p, n):
    """Photograph the next n photo spots the planner asks for (drone 1 alone)."""
    for _ in range(n):
        plan = p.plan({1: p.point_a()})
        if not plan[1]:
            break
        x, y, _g = plan[1][0]
        p.record_footprint(footprint(x, y))
        p.advance()


print("=== A. a mine found IN FRONT of point A by this side (Source.PAIR): point A jumps past it")
p = make()
fly(p, 2)
show("before   ", p)
print("      mine at (10.5, 22.0); the forward gap runs straight through it")
p.add_mine(10.5, 22.0, Source.PAIR)
show("after    ", p)
print("      obstacle corners the routes bend around:", [(round(x, 1), round(y, 1)) for x, y in p.routes.corners()][:6], "...")

print()
print("=== B. a mine relayed by the OTHER side (Source.PARTNER) lands on ground already CHECKED")
p = make()
fly(p, 6)
show("before   ", p)
print("      route is checked up to y =", round(p.point_a()[1], 1), "; mine at (10.0, 8.0) on the checked stretch")
p.add_mine(10.0, 8.0, Source.PARTNER)
show("after    ", p)

print()
print("=== C. a mine on the ENTRY square")
p = make()
show("before   ", p)
p.add_mine(10.0, 1.0, Source.PAIR)
show("after    ", p)
