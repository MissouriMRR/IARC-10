"""The registry of path engines (see path_engine.py). PlannerConfig.engine names one."""

import importlib

from flight.pathfinding.pathnode.geometry import ZoneMap


class _Engines(dict):
    """name -> PathEngine class. A "module:Class" string is imported on first use, so an engine
    nobody selects is never loaded (the default planner must not import nodeField)."""

    def __getitem__(self, name):
        cls = super().__getitem__(name)
        if isinstance(cls, str):
            module, attr = cls.split(":")
            cls = getattr(importlib.import_module(module), attr)
            self[name] = cls
        return cls


ENGINES = _Engines({"zonemap": ZoneMap, "field": "flight.pathfinding.nodeField.field:Field"})


def make_engine(name, field_w, field_h):
    """A fresh PathEngine of the named kind for a field_w x field_h ft field."""
    if name not in ENGINES:
        raise ValueError(f"unknown path engine {name!r}; choose from {sorted(ENGINES)}")
    return ENGINES[name](field_w, field_h)
