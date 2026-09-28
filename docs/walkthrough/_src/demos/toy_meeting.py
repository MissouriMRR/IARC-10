"""Toy run: two sides closing on each other. 4 drones, 24 ft x 80 ft field, 4 mines.

Prints, per simulator tick, each side's point A and the length of its forward gap (the stretch
it still has to explore toward the other side's point A), then who has 'met' and the joined
route. Side 1 = drones 1+2 (bottom), side 3 = drones 3+4 (top); each side's speaker is 1 / 3."""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.pathnode.config import LinkKind
from flight.pathfinding.pathnode.polyline import poly_len
from flight.pathfinding.pathnode.sim import Mission

mines = [(12, 20), (9, 26), (14, 52), (10, 60)]
m = Mission(mines, drones=4, field_w=24.0, field_h=80.0, entry_x=12.0,
            shape_size_ft=(8.52, 6.44), max_ticks=1500, record_frames=True).run()


def side_line(side):
    links = side["links"]
    nodes = side["nodes"]
    fwd = [poly for kind, poly in links if kind == LinkKind.FORWARD]
    gap = f"{poly_len(fwd[0]):5.1f} ft" if fwd else " closed"
    f = next((i for i, (kind, _p) in enumerate(links) if kind == LinkKind.FORWARD), None)
    a = nodes[f] if f is not None else nodes[-1]
    return f"A=({a[0]:5.1f},{a[1]:5.1f}) gap={gap}"


last = None
for fr in m.frames:
    line = f"bottom side {side_line(fr['sides'][1])} | top side {side_line(fr['sides'][3])}"
    if line != last:
        print(f"tick {fr['tick']:>2}: {line}")
        last = line
p1, p3 = m.planners[1], m.planners[3]
print("bottom met:", p1.met, "| top met:", p3.met)
print("top route as sent in CROSS_PAIR_CHAIN:", [(round(x, 1), round(y, 1)) for x, y in p3.route()][:4], "... to", [round(v, 1) for v in p3.route()[-1]])
print("bottom route:", [(round(x, 1), round(y, 1)) for x, y in p1.route()][:4], "... to", [round(v, 1) for v in p1.route()[-1]])
joined = p1.join(p3.route())
print("joined (bottom entry -> top entry):", [(round(x, 1), round(y, 1)) for x, y in joined])
