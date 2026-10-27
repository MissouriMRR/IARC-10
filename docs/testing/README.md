# Testing the drone mine-field project

Start here. There are no drones needed for any of this: everything runs on a laptop with a
simulated field. Pages:

- [unit-tests.md](unit-tests.md): the planner suite, coverage suite, path-engine conformance suite, geometry (nodeField) suite, CellField suite, pytest suite.
- [../architecture.md](../architecture.md): how the planner is built and which test to run per extension point.
- [simulator.md](simulator.md): `Mission` / `evaluate()`, what each metric means, making GIFs.
- [flight-harness.md](flight-harness.md): the full state machine on fake hardware (needs WSL).
- [tools.md](tools.md): `tools/testing/` scripts (verify, sweep, golden, benchmark, repro cases, snapshots).

## Quick start: I changed X, run Y

Commands are in Git Bash from the repo root. `$PWD` is the repo. "WSL" commands: see
[flight-harness.md](flight-harness.md) and [unit-tests.md](unit-tests.md) for the exact lines.

| I changed | Run | Expect | Time |
|---|---|---|---|
| everything (before merging, after a big change) | `python tools/testing/verify.py` | `ALL PASSED` (3 known pytest failures shown as `known`) | ~7 min idle, ~17 min under load |
| planner code (`flight/pathfinding/pathnode/`) | `PYTHONPATH=$PWD python -m flight.pathfinding.pathnode.test_pathnode` | `N/N passed` | 15-20 min |
| planner code, quick look | `python tools/testing/sweep.py --mines 90 --seeds 0 4` | `4/4 ok, 0 BAD` | ~1 min |
| planner code, before merging | `python tools/testing/sweep.py --standard --jobs 6` | `90/90 ok, 0 BAD` | ~6 min |
| anything about lost / late messages | the lossy configs d, e, f in `--standard` (or `--drop 0.5 --delay 3`) | `ok` on every seed | included above |
| a bug on one seed | `python tools/testing/repro_cases.py` (or `sweep.py --seeds S S+1`) | `ok` per case | 1-5 min |
| refactor, no behaviour change intended | `golden.py` before and after, then `golden_compare.py` | `17 identical, 0 different` | ~10 min per recording |
| geometry (`nodeField/`, `protoMine.py`, `geometry.py`) | `PYTHONPATH=$PWD python -m flight.pathfinding.nodeField.test_nodefield` | `12 tests passed` | ~25 min (`NODEFIELD_LAYOUTS=2`: ~3 min) |
| `CellField` | `PYTHONPATH=$PWD python -m flight.pathfinding.tests.cellFieldTest` | `58/58 passed` | 30 s |
| state machine, interdrone messages, flight glue | flight harness: default, `4 dense`, one-drone (WSL) | `RESULT: PASS`; one-drone `route check : ok` | ~8-20 min |
| interdrone / emergency land / POIF | pytest suite (WSL) | `3 failed, 44 passed, 16 skipped` (3 known failures) | ~10 s |
| efficiency ("is it faster?") | `python tools/testing/benchmark.py` | `complete=True`, lower `makespan`/`photos` | ~1 min |
| I want to keep editing while tests run | `bash tools/testing/snapshot.sh mytry`, then test inside the copy | same as above | seconds |

## Where the tests live

| Path | What |
|---|---|
| `flight/pathfinding/pathnode/test_pathnode.py` | planner unit + whole-mission tests |
| `flight/pathfinding/pathnode/sim.py`, `minefield.py`, `render.py` | the simulator, field generator, GIF renderer |
| `flight/pathfinding/nodeField/test_nodefield.py` | geometry tests |
| `flight/pathfinding/tests/cellFieldTest.py` | CellField tests |
| `integration_tests/` | flight-code harness (`multi_drone_sim_test.py`, `one_drone_sim_test.py`) |
| `tests/` | pytest suite for flight code |
| `tools/testing/` | verify (all suites at once), sweep, golden, benchmark, repro cases, snapshot scripts |

## The whole ladder in one command: verify.py

`python tools/testing/verify.py` freezes the tree (a snapshot under `.test_snapshots/`, path
printed) and runs every step of the ladder below on all cores: unit + geometry + CellField
tests (one process per test), the 90-seed sweep (one process per seed), pytest and pyflakes
(WSL), then the flight harness (WSL, every configuration its own process). Takes about
7-17 minutes instead of ~40, prints a line per finished job and one summary table, exits 1 on
any real failure. The flight harness is timing sensitive, so it runs AFTER the CPU-heavy
suites, not alongside them (see [tools.md](tools.md#verifypy-the-whole-ladder-on-all-cores)
for options and the measurements). Use the manual ladder below when you only need one step.

## The verification ladder (run in this order after a change)

Cheap checks first, so you fail fast. Stop at the first failure and fix it before going on.

1. Unit suite: `PYTHONPATH=$PWD python -m flight.pathfinding.pathnode.test_pathnode`
   (15-20 min). Expect `N/N passed`. It stops at the first failing test.
2. Sweep, all 90 seeds including the lossy configs d/e/f:
   `python tools/testing/sweep.py --standard --jobs 6` (about 6 min measured on 22 cores with 6 processes, while other jobs were running). Expect `90/90 ok, 0 BAD`.
3. Flight harness (WSL), all three:
   - `... -m integration_tests.multi_drone_sim_test`  (2, 3, 4 drones; about 1 min)
   - `... -m integration_tests.multi_drone_sim_test 4 dense`  (several minutes)
   - `... -m integration_tests.one_drone_sim_test`  (about 6 s)
   Expect `RESULT: PASS` for each drone count and `route check : ok` for one-drone.
   (`4 dense` currently fails, see below.)
4. pytest suite (WSL, ~10 s): expect `3 failed, 44 passed, 16 skipped`, with the 3 failures
   being the known ones in `tests/test_emergency_land.py`. Any other failure is new.

Add when relevant:
- Pure refactor (no intended behaviour change): record `golden.py` on the OLD code first, then
  after the change run `golden_compare.py`; expect `17 identical, 0 different`. This replaces
  eyeballing the sweep for "did anything move".
- Geometry change: also run the nodeField suite (full 30 layouts before trusting it).
- Performance-motivated change: run `benchmark.py` before and after.

Shortcut for a first look at a planner change: run the sweep on a few seeds
(`--seeds 0 4`) before the 20-minute unit suite.

### Test a snapshot while you keep editing

The slow steps take 15-20 minutes each. Freeze a copy and test that:

```
bash tools/testing/snapshot.sh try1          # -> .test_snapshots/try1
cd .test_snapshots/try1
PYTHONPATH=$PWD python -m flight.pathfinding.pathnode.test_pathnode > unit.log 2>&1 &
python tools/testing/sweep.py --standard --jobs 6 > sweep.log 2>&1 &
# flight harness on the copy, from Git Bash (from the repo root, not the snapshot):
cd ../..
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/harpe/Multirotor/IARC-10/tools/testing/flight_snapshot.sh try1 > flight.log 2>&1 &
```

Come back later and read `unit.log`, `sweep.log`, `.test_snapshots/try1/flight_results.txt`.
Details and the `.gitignore` note are in [tools.md](tools.md).

## Rules of thumb

- Run the WHOLE ladder after a fix, not just the case you fixed. A fix for one seed has broken
  others before.
- Prefer a short new test plus a `repro_cases.py` entry for every bug fixed.
- Long commands (unit suite, full sweep, flight harness): run them in the background with output to a
  file and read it afterwards (`cmd > out.log 2>&1 &`).
- Tests use fixed seeds, so results are reproducible: the same command on the same code gives the
  same output. A different result means the code (or the machine load, for the timing tests in
  `cellFieldTest`) changed.

## Currently failing (2026-09-28)

- Flight harness `multi_drone_sim_test 4 dense`: fails run to run in different ways (mine sets
  disagree / no route reported). Details in [flight-harness.md](flight-harness.md#known-problem-2026-09-28-4-dense-fails).
- pytest: the 3 known `test_emergency_land.py` failures.

## Things that look broken but are not

- pytest: 3 failures in `tests/test_emergency_land.py` (`max_messages_per_tick`), see
  [unit-tests.md](unit-tests.md).
- Flight harness noise: `Drone N failed to arm. Resending message.`, `start mission recieved`.
- One-drone report: `mines: discovered=0 / true=3` and no "PASS" line.
- The flight harness reports `mines found : 4 / 7 true`: only mines near the route need finding.
- Windows Python has no `pytest`: use the script forms, or WSL.
