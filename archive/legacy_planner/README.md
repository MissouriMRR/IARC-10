# Legacy maze planner (archived)

The search algorithm the drones used before the path-node planner
(`flight/pathfinding/pathnode/`) replaced it. It was taken out of the flight code's main path
and kept here for reference.

## What's here

| Path | What it was |
|---|---|
| `flight/pathfinder.py` | `Pathfinder`: the maze planner (segments A/B/C, helper-node detours, cross-pair patched spans, mine-radius expansion) |
| `flight/pathfinding/nodeField/` | The node graph, polygon and union obstacles, and the field it ran on |
| `flight/pathfinding/protoMine.py`, `blockField/`, `mineCellField.py`, `common.py` | Mine models and grids used only by the node field |
| `flight/pathfinding/path_calculation.py`, `path_subdivision.py` | Shortest-path and route subdivision over the node graph |
| `flight/pathfinding/sim/` | The full-detail GIF renderer for the old planner's simulations |
| `legacy_mission_planner.py` | `LegacyMissionPlanner`: the adapter that ran `Pathfinder` behind the flight code's planner interface, while `FlightSettings.planner` could pick `"legacy"` |

The layout mirrors the old `flight/` package, so each file sits at the path it used to have.

## It does not run from here

Nothing imports this folder, and its imports still name `flight.pathfinder`,
`flight.pathfinding.nodeField` and so on, which no longer exist in the package. To run it
again, restore it from git history.

It was archived from the working tree on top of commit `4b88b73` ("Baseline before path-node
flight integration"), and it includes that session's uncommitted edits (for example,
`pathfinder.py` re-exporting `format_iarc_path` from `flight/pathfinding/iarc_report.py`).
The last commit containing it in its original place is `4b88b73`.

The old planner's tests (`flight/pathfinding/tests/`, apart from the CellField tests) and its
design docs (`ALGORITHM.md`, `REMEDIATION_PLAN.md`, `nodeField/notes.txt`) were deleted
rather than archived. They are in git history at the same commit.
