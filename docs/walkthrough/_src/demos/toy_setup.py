"""Toy run: what MissionPlanner/PathNodePlanner look like right after construction, for drone 1 of
the 4-drone mission on the golf field (corners of mission_config.json, re-ordered so they go
counter-clockwise: 1, 2, 4, 3 -- see chapter 2)."""
import json
import sys
sys.path.insert(0, ".")
from flight.pathfinding.mission_planner import MissionPlanner

cfg = json.load(open("mission_config.json"))
c = [(p["lat"], p["lon"]) for p in cfg["mission_field_corners"]]
corners = [c[0], c[1], c[3], c[2]]
mid = ((corners[0][0] + corners[1][0]) / 2, (corners[0][1] + corners[1][1]) / 2)   # middle of the bottom edge

for label, side_drones, edge, partner in (("drone 1 (bottom side, has a partner side)", (1, 2), "bottom", True),
                                          ("drone 3 (top side, has a partner side)", (3, 4), "top", True),
                                          ("solo/2-drone mission (no partner side)", (1, 2), "bottom", False)):
    mp = MissionPlanner(corners, mid, edge, side_drones, partner, shape_size_ft=(8.52, 6.44))
    p = mp.planner
    print(f"--- {label}")
    print(f"field {mp.field_w:.1f} x {mp.field_h:.1f} ft, cells {p.coverage.seen.width} x {p.coverage.seen.height}")
    print("nodes :", [(n.placed_by.value, round(n.x, 1), round(n.y, 1)) for n in p.nodes])
    print("links :", [(k.kind.value, [(round(x, 1), round(y, 1)) for x, y in k.poly]) for k in p.links])
    print("target: partner =", p.target.partner, "partner_a =", p.target.partner_a, "is_partner =", p.target.is_partner)
    print("claims:", p.claims.drones, "| failure:", p.failure)
