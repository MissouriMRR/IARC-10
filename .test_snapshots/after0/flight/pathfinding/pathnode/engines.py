"""The registry of path engines (see path_engine.py). PlannerConfig.engine names one."""

from flight.pathfinding.nodeField.field import Field
from flight.pathfinding.pathnode.geometry import ZoneMap

ENGINES = {"zonemap": ZoneMap, "field": Field}


def make_engine(name, field_w, field_h):
    """A fresh PathEngine of the named kind for a field_w x field_h ft field."""
    if name not in ENGINES:
        raise ValueError(f"unknown path engine {name!r}; choose from {sorted(ENGINES)}")
    return ENGINES[name](field_w, field_h)
