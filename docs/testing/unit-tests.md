# Unit and geometry tests

Four suites that need no drone hardware. Run from the repo root. Git Bash forms shown; in WSL
use the same lines (WSL has pytest; the Windows Python here does not, so use the script form
on Windows).

| Suite | Command | Time |
|---|---|---|
| Planner (path-node) | `PYTHONPATH=$PWD python -m flight.pathfinding.pathnode.test_pathnode` | 15-20 min |
| Geometry (nodeField) | `PYTHONPATH=$PWD python -m flight.pathfinding.nodeField.test_nodefield` | ~25 min full, ~3 min with `NODEFIELD_LAYOUTS=2` |
| CellField bit grid | `PYTHONPATH=$PWD python -m flight.pathfinding.tests.cellFieldTest` | ~30 s |
| pytest suite (`tests/`) | see [pytest suite](#pytest-suite-tests) | ~10 s |

PowerShell: `$env:PYTHONPATH = (Get-Location).Path; python -m ...`.

All of them are plain `assert`s in `test_*` functions (except cellFieldTest, see below). They
run either as a script or under pytest (`python -m pytest -q flight/pathfinding/pathnode/test_pathnode.py`
in WSL, or `-k name` to run one test).

## Planner suite: flight/pathfinding/pathnode/test_pathnode.py

Run as a script, it prints `PASS <test name>` per test and ends with `N/N passed`. It runs the
tests in file order and STOPS at the first failure (traceback, no summary); use pytest if you
want every failure listed.

```
PYTHONPATH=$PWD python -m flight.pathfinding.pathnode.test_pathnode
```

Measured on 2026-09-28: `45/45 passed`, about 20 minutes. Real output (start; slow tests are the whole-mission ones near the middle):

```
PASS test_a_big_saving_earns_more_photos
PASS test_shortcut_over_photographed_ground_is_applied_at_once
PASS test_outer_layer_is_the_next_ring_out_and_comes_last
PASS test_every_drone_count_completes_with_a_valid_route
PASS test_completes_despite_dropped_and_delayed_relays
```

Look at the file for the current list (`grep -n "^def test_" flight/pathfinding/pathnode/test_pathnode.py`);
other people are editing it, so do not rely on a list copied here. The test groups, in file order:

| Group | What it checks |
|---|---|
| Footprint | the planner's photo footprint model matches `CellField.fill_polygon_covered`; only fully covered squares count |
| Mine rule / chain repair | a mine cutting a chain is bridged, orphaned nodes are deleted, "gambler" jumps and assistant nodes; duplicate mines ignored. Chains are built by hand (`_planner(points, kinds)`) because real missions rarely trigger the orphan rule |
| Routes and joining | routing around a mine; `join_routes` splices at the first shared point |
| Claims (`ClaimBook`) | work is claimed per path, deterministic, a lost drone is handed nothing, idle drones take lower priority groups |
| Shortcuts | a shortcut is tried, spliced once fully checked, dropped if a mine makes it too long, never replaces the whole path; photo budget scales with saving |
| Whole missions | every drone count finishes with a valid route; still completes with dropped and delayed relays; band coverage leaves nothing in danger; field-corner cases |
| Frames | local frame is in true feet; the lat/lon planner matches the local one |
| Side sessions / work keys | an announced job is honoured by the other copy of the planner; work keys are identical in every copy; the old planner is never imported |
| Regressions | walled-in entry, a square that keeps being missed gets blocked, rotated footprints finish, crosswise meeting is repaired |
| Caches | cached groups equal a fresh computation; kept routes stay shortest after new mines |
| Separation | sides keep apart by drone id; a higher id makes way for the other side's job; `keep_clear_ft` is above the flight abort distance (`SEPARATION_ABORT_M` in `flight/waypoint.py`) |

### Adding a test

- Add `def test_<what_it_guarantees>():` with a one-line docstring and plain `assert`s (give the
  assert a message). The `__main__` block picks up every top-level function whose name starts
  with `test_`, so no registration is needed.
- For a whole-mission check use `Mission(generate_true_minefield(n, seed), drones=d).run()` and
  `evaluate(m)` (see [simulator.md](simulator.md)). Keep `n` small; the slow tests are slow
  because they run many missions.
- For a chain/mine-rule check build the chain by hand with `_planner(points, kinds)`.
- A bug fix should come with a regression test that fails before the fix. If the trigger is a
  particular seed, also add it to `tools/testing/repro_cases.py`.

## Geometry suite: flight/pathfinding/nodeField/test_nodefield.py

Tests the mine-shape and shortest-path geometry package `nodeField`. Prints `ok <test> (<secs>)`
per test and `N tests passed`; stops at the first failure.

```
NODEFIELD_LAYOUTS=2 PYTHONPATH=$PWD python -m flight.pathfinding.nodeField.test_nodefield
```

Real output (2 layouts; 12 tests; the completeness test dominates):

```
completeness: 2 layouts, 100 routes (74 reachable), 0 mismatches, 96s
ok test_completeness_vs_bruteforce (95.6s)
expand: 3 steps, squares inside, paths equal brute force
ok test_expand (14.0s)
ok test_expand_matches_fresh_geometry (0.0s)
incremental == rebuilt after 40 adds + expand (621 edges)
ok test_incremental_graph_equals_rebuild (5.0s)
ok test_merge_exact_union_resolve_and_hash (3.8s)
ok test_mine_hash_formula_matches_planner (0.7s)
ok test_no_query_point_in_graph (1.0s)
ok test_outline_covers_unsafe_squares_only (1.7s)
ok test_plot_smoke (1.1s)
ok test_protomine_squares_match_unsafe_cells (0.5s)
tangents-only graph: 38/150 routes differ from the complete graph
ok test_tangents_alone_and_subset (28.2s)
ok test_zonemap_surface (0.0s)
12 tests passed
```

What it checks:
- `protoMine` squares equal `unsafe_cells` (the planner's notion of a mine's danger squares);
  the outline covers only unsafe squares.
- Merging overlapping mines gives the exact union; the mine hash formula equals the planner's.
- `ZoneMap`-compatible surface (`zonemap_surface`): the methods the planner calls exist and agree.
- Completeness against brute force: 30 random layouts x 50 routes, each compared with a graph
  over every corner pair. The full run is the ~18-25 minute one. `NODEFIELD_LAYOUTS=<n>` sets the
  layout count (default 30); use 2 for a quick check, the default before trusting a geometry
  change. The line `tangents-only graph: 38/150 routes differ` is informational (a tangent-only
  graph is deliberately not complete).
- The incrementally built graph equals a full rebuild; no query point leaks into the graph;
  `expand` (grow mines by a margin) matches a fresh mitre offset; a plot smoke test (Agg backend).

Run it whenever you change `flight/pathfinding/nodeField/`, `protoMine.py`, or
`pathnode/geometry.py`.

## CellField: flight/pathfinding/tests/cellFieldTest.py

Checks `CellField`, the bit-packed grid (one bit per cell in one Python int): get/set, vertical
slice, `cover_with_shape`, shift/expand, bit ops, fill rect/disk/polygon, `mark_path`, block
commands, and some speed budgets.

Unlike the others its `main()` calls every test by hand (each test prints its own
`-> PASS` line and returns True). New tests must be added to the list in `main()` or they will not run as a script (pytest
would still find them).

```
PYTHONPATH=$PWD python -m flight.pathfinding.tests.cellFieldTest
```

Real output (tail):

```
test_out_of_bounds_path_stays_fast: elapsed=0.031ms (budget=1000ms) -> PASS
test_block_commands_simple: -> PASS
...
58/58 passed
```

The "faster than" tests compare timings, so a heavily loaded machine can make one flaky: re-run
before assuming a real regression.

## pytest suite: tests/

Flight-code unit tests (fakes only): `test_emergency_land.py` (LAND / EMERGENCY_LAND commands),
`test_poif_avoidance.py` (POIF lockstep collision avoidance), `test_flight_controller_connection.py`
(preflight link check; needs a real autopilot and `FC_TEST=1`, otherwise skipped).

WSL:
```
cd /mnt/c/Users/harpe/Multirotor/IARC-10
~/.local/bin/uv run --no-sync --with pytest python -m pytest -q tests
```
Git Bash:
```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/harpe/Multirotor/IARC-10 && ~/.local/bin/uv run --no-sync --with pytest python -m pytest -q tests"
```

Expected baseline (real): `3 failed, 44 passed, 16 skipped in 8.32s`.

Known failures (3), all in `tests/test_emergency_land.py::TestMessageHandling`:
- `test_emergency_land_acts_once_on_duplicates`
- `test_land_starts_an_orderly_landing`
- `test_land_does_not_override_an_emergency_land`

Cause: `AttributeError: 'Interdrone' object has no attribute 'max_messages_per_tick'`
(`state_machine/interdrone.py:1474`). The test builds an `Interdrone` without its `__init__`
(where `max_messages_per_tick = 25` is set) and never sets that attribute. The bug is in the
test helper, not in `Interdrone` (`integration_tests/multi_drone_sim_test.py` sets the
attribute in its own equivalent helper). The fix is one line in the test's `_interdrone()`
helper. Until then, "3 failed" here is the baseline; anything else is new.

The 16 skips are all in `test_flight_controller_connection.py`: most need a real flight
controller (`FC_TEST=1`), a few need a link-recovery API that `state_machine.drone` in this
checkout lacks. `pytest -rs` prints the reason for each.
