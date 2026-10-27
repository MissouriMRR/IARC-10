"""Path engines: the shortest-path-around-mines implementations the planner can run on.

Layout:
- base.py: PathEngine, the contract every engine meets (read its docstring first).
- zonemap.py: ZoneMap, the default engine (convex-hull zones + visibility graph).
- nodeField/field.py: Field, the diamond-corner engine; registered here, imported lazily.
- this file: ENGINES (name -> class) and make_engine(name, w, h). PlannerConfig.engine is a name.

How to add an engine:
1. Subclass PathEngine (engines/base.py) in a new module and implement every abstract method.
2. Register it in ENGINES below: `"name": MyEngine`, or `"name": "package.module:MyEngine"` so
   it is imported only when selected (the default planner must not import heavy engines).
3. Run `python -m flight.pathfinding.pathnode.test_path_engine`; it checks every registered
   engine against the contract. A genuine gap goes in its KNOWN_GAPS with a message.
4. Select it with PlannerConfig(engine="name").
"""

import importlib

from flight.pathfinding.pathnode.engines.base import PathEngine
from flight.pathfinding.pathnode.engines.zonemap import ZoneMap


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


__all__ = ["PathEngine", "ENGINES", "make_engine"]
