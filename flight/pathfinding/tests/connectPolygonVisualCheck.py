"""
End-to-end visual check for the new group/arc-based connectPolygon: builds a
deliberately clustered field (some obstacles placed so one sits between two
others), then uses the real Field.plotField() debug renderer to visually
confirm no accepted connection cuts through a third obstacle.
"""

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot

from flight.pathfinding.nodeField.field import Field

arbCorners = [[0, 100], [100, 100], [0, 0], [100, 0]]
sim_field_size = [100, 100]
field = Field(sim_field_size, arbCorners)


def square(cx, cy, half):
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]


# Deliberately clustered layout: B sits directly between A and C, D sits
# between B and E, F is off to the side to give a non-trivial group shape.
obstacles = [
    ("A", square(20, 50, 3)),
    ("B", square(35, 50, 2.5)),  # between A and C
    ("C", square(50, 50, 3)),
    ("D", square(50, 62, 2)),  # between B/C and E, roughly
    ("E", square(50, 75, 3)),
    ("F", square(65, 40, 3)),
]

add_order = [2, 0, 4, 1, 5, 3]  # C, A, E, B, F, D -- not a trivial left-to-right order
for idx in add_order:
    name, verts = obstacles[idx]
    field.createPolygonObstacle(verts)

field.plotField(
    labeled=False,
    title="Clustered layout: visual check for connections cutting through a third obstacle",
)
fig = pyplot.gcf()
fig.set_size_inches(12, 12)
pyplot.xlim(5, 80)
pyplot.ylim(25, 90)
out_path = r"C:\Users\harpe\AppData\Local\Temp\claude\c--Users-harpe-Multirotor-IARC-10\2567942b-0ada-473d-af3b-214803e7410d\scratchpad\connectPolygon_visual_check.png"
pyplot.savefig(out_path, dpi=150)
print("saved to", out_path)
