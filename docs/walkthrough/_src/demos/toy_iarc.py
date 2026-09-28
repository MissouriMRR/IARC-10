"""Toy run: the route drone 1 reports for the 4-drone toy mission (24 ft x 80 ft, 4 mines), as the
IARC text format that AppShare pushes and writes (iarc_report.format_iarc_path)."""
import sys
sys.path.insert(0, ".")
from flight.pathfinding.iarc_report import format_iarc_path
from flight.pathfinding.pathnode.sim import Mission

mines = [(12, 20), (9, 26), (14, 52), (10, 60)]
m = Mission(mines, drones=4, field_w=24.0, field_h=80.0, entry_x=12.0,
            shape_size_ft=(8.52, 6.44), max_ticks=1500).run()
joined = m.report_route()
print("joined route has", len(joined), "points, from", tuple(round(v, 1) for v in joined[0]),
      "to", tuple(round(v, 1) for v in joined[-1]))
text = format_iarc_path(joined, 0, strict=False, field_w=24.0, field_h=80.0)
lines = text.splitlines()
print("--- format_iarc_path (each line: direction, number of 2 ft squares) ---")
print("\n".join(lines[:9]), "\n...", f"({len(lines)} lines in all)")
