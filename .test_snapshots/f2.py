import shapely, traceback
from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.pathnode.minefield import generate_true_minefield
e = Field(80.0, 300.0)
for i,(x, y) in enumerate(generate_true_minefield(25, 3)):
    try:
        e.add_mine(x, y)
    except Exception as ex:
        print("fail at", i, x, y, repr(ex)); 
        b = e.blocker; print(type(b), b.is_valid, b.geom_type, shapely.is_prepared(b)); break
print("ok", i)
