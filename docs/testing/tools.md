# Test tools (tools/testing/)

Small scripts that run the simulator ([simulator.md](simulator.md)) many ways. All of them:
- work from any folder (they find the repo root from their own location, so no PYTHONPATH
  needed, and they test whichever copy of the repo they live in, including a snapshot);
- have `--help`;
- exit 0 on success and 1 on failure, so they can sit in a script.

Commands below are for Git Bash from the repo root (they also work in WSL and PowerShell,
with `python` replaced by your interpreter).

| Script | Question it answers | Typical time |
|---|---|---|
| `sweep.py` | does the planner finish safely on many random fields? | 90 seeds: ~6 min on 6 processes |
| `golden.py` + `golden_compare.py` | did my refactor change ANY behaviour? | recording all 17 cases: several minutes |
| `repro_cases.py` | do the named hard cases still work? | ~1 min for two cases, a few minutes for all 5 |
| `benchmark.py` | how efficient / how slow is a mission on the seed-76 field? | about a minute |
| `verify.py` | is everything still green? (all suites at once, on a frozen copy) | ~7 min on an idle 22-core box, ~17 min with other agents running (was ~40 min serial) |
| `snapshot.sh`, `flight_snapshot.sh` | test a frozen copy while I keep editing | seconds / 15-20 min |

## sweep.py

Runs one 4-drone mission per seed, in parallel, and prints `ok` or `BAD` per seed.

```
python tools/testing/sweep.py --standard                      # the 90-seed set below
python tools/testing/sweep.py --mines 90 --seeds 0 2 --tag a  # seeds 0 and 1 only
python tools/testing/sweep.py --mines 90 --seeds 100 103 --drop 0.5 --delay 3   # lossy
```

Real output (2 seeds):

```
a seed=0 ok failure=None complete=True spans=True unseen=0 known=0 on_mine=0 min_cross=4.57 min_side=4.00 ticks=133
a seed=1 ok failure=None complete=True spans=True unseen=0 known=0 on_mine=0 min_cross=4.40 min_side=4.00 ticks=171

2/2 ok, 0 BAD
```

Reading a line:
- `failure` / `complete` / `spans` / `unseen` / `known` / `on_mine`: the `evaluate()` metrics
  ([simulator.md](simulator.md)). Any bad one makes the line `BAD`.
- `min_cross`: closest two drones of different sides ever got (ft). `min_side`: same for
  two drones of one side. Watch these if you change separation or keep-clear code: the flight
  code aborts below `SEPARATION_ABORT_M` (`flight/waypoint.py`), and the planner's
  `keep_clear_ft` must stay above it. Around 4 ft is normal here; a drop toward 0 is a real
  problem.
- `ticks`: how long the mission took (simulation steps).

### The standard 90-seed set (`--standard`)

| Tag | Mines | Seeds | Relay drop | Max delay (ticks) | What it stresses |
|---|---|---|---|---|---|
| a | 90 | 0-14 | 0 | 0 | normal field |
| b | 90 | 15-29 | 0 | 0 | more normal fields |
| c | 120 | 0-14 | 0 | 0 | dense field (boxed-in routes) |
| d | 90 | 100-114 | 0.5 | 3 | lossy radio |
| e | 90 | 115-129 | 0.5 | 3 | lossy radio |
| f | 60 | 200-214 | 0.3 | 5 | lossy, long delays |

The seeds are fixed so a failure is reproducible: re-run just that seed with
`--mines N --seeds S S+1 --drop P --delay T` (same config values as its row), or use
`repro_cases.py`. `--jobs 6` runs six processes (the default is your CPU count).

Real result of the full set (`--standard --jobs 6`, 5m47s wall clock):

```
...
f seed=214 ok ...

90/90 ok, 0 BAD
```

Run it in the background and read the file afterwards if your shell has a time limit:

```
python tools/testing/sweep.py --standard --jobs 6 > sweep.log 2>&1 &
```

Always re-run the whole set after any planner fix, not just the seed you were fixing: a fix
for one seed has broken others before.

## golden.py and golden_compare.py (behaviour fingerprint)

Use for pure refactors ("moved code, renamed things, should behave identically").

`golden.py` runs 17 fixed missions (1-4 drones x seeds 3, 11, 76; two lossy cases; random
photo rotation; a walled-in entry; band coverage) and saves everything observable: every
drone's exact photo sequence, the reported route, discovered mines, tick count and planner
counters.

```
python tools/testing/golden.py --list                 # the 17 case names
python tools/testing/golden.py before.json            # all 17
python tools/testing/golden.py before.json d1_s3 walled   # just two
python tools/testing/golden_compare.py before.json after.json
```

Real output (two cases, recorded twice, then compared):

```
d1_s3 f6d1fe3b65cd935b None 445
walled f5aa1569c5f59c49 None 693
2 identical, 0 different
```

(Columns: case, hash, failure, total photos.) The simulator is deterministic, so any DIFF is a
real behaviour change: `DIFF ['photos', 'stats.jumps=1->2']` tells you what moved. Extra
arguments to `golden_compare.py` ignore keys (e.g. `tick`) when you expect that to change.
To record a baseline of the OLD code while you keep editing, take a snapshot first (below)
and run `golden.py` inside the snapshot.

## repro_cases.py

Five named cases that each broke the planner once: `crosswise`, `crosswise_loss`,
`walled_entry`, `rotated45`, `rotated_random`.

```
python tools/testing/repro_cases.py --list
python tools/testing/repro_cases.py walled_entry rotated45
```

Real output:

```
walled_entry: ok failure=None ticks=694 makespan=3483 photos=693 route=339 complete=True joined=True spans=True unseen=0 known_crossings=0 on_mine=0 cpu=36.7s
rotated45: ok failure=None ticks=410 makespan=1662 photos=816 route=316 complete=True joined=True spans=True unseen=0 known_crossings=0 on_mine=0 cpu=15.5s
```

Add a case by adding an entry to `CASES` at the top of the file (`mines`, `seed`, `drones`,
and `kw` = extra `Mission` options).

## benchmark.py

Runs one field (default seed 76, 90 mines, 4 drones) four ways (planning footprint 6x4 ft
vs real 8.52x6.44 ft footprint, `outer_ring` 0 vs 2) and prints the efficiency numbers.
Compare before and after a change: smaller `makespan`/`photos` is better as long as
`complete=True` and `on_mine=0`.

```
python tools/testing/benchmark.py --drones 2 --mines 30 --seed 3
```

Real output (small case):

```
shape=(6.0, 4.0) outer=0: failure=None makespan=728 photos=244 route=305.1 danger=1 on_mine=0 blocked=0 complete=True cpu=1.6s
shape=(6.0, 4.0) outer=2: failure=None makespan=1059 photos=399 route=305.2 danger=0 on_mine=0 blocked=0 complete=True cpu=7.3s
shape=(8.52, 6.44) outer=0: failure=None makespan=300 photos=46 route=302.8 danger=1 on_mine=0 blocked=0 complete=True cpu=0.4s
shape=(8.52, 6.44) outer=2: failure=None makespan=746 photos=142 route=302.9 danger=0 on_mine=0 blocked=0 complete=True cpu=1.5s
```

`danger=1` with `outer=0` is expected (no safety ring); `outer=2` (the default) gives 0.

## verify.py (the whole ladder on all cores)

Freezes the tree, splits every suite into small jobs and runs them in one process pool.
Replaces running the unit suite, sweep, flight harness and pytest one after another (~40 min).

```
python tools/testing/verify.py                              # snapshot + everything
python tools/testing/verify.py --only unit,sweep,lint       # some suites
python tools/testing/verify.py --tree .test_snapshots/x     # an existing copy, no new snapshot
python tools/testing/verify.py --serial-flight              # flight configs one at a time
```

Options: `--snapshot LABEL` (name the copy, default `verify-<time>`), `--tree PATH` (test that
folder as is), `--only` (unit, nodefield, cellfield, sweep, flight, pytest, lint), `--jobs N`
(default cores - 2), `--flight-repeat N`, `--flight-jobs N`, `--serial-flight`,
`--flight-alongside`, `--lint-baseline FILE`. Exit 0 = everything passed (the 3 known pytest
failures and pyflakes warnings do not count), 1 = a real failure or timeout, 130 = Ctrl-C.

How it splits the work:

| Suite | Jobs |
|---|---|
| unit (`test_pathnode.py`) | one process per `test_*`; the 3 slow loop tests (`every_drone_count`, `band_coverage`, `dropped_and_delayed`) are cut into 12 / 4 / 8 shards of their innermost `for` loop |
| nodefield | one process per test; `completeness_vs_bruteforce` in 10 shards of layouts |
| cellfield | whole file, one retry (it has timing asserts) |
| sweep | one process per seed of `--standard` (90) |
| flight (WSL) | multi 2, 3, 4, 2 dense, 3 dense, 4 dense (x3), one-drone, each its own process |
| pytest (WSL) | `tests/`; exactly the 3 known `test_emergency_land.py` failures are tolerated, 44 must pass |
| lint (WSL) | pyflakes; reported only. `verify_logs/<run>/lint_warnings.txt` (no line numbers) can be passed back as `--lint-baseline` to list only new warnings |

Scheduling: longest job first, using durations cached in `.test_snapshots/verify_durations.json`
(the first run uses guesses; later runs are better balanced). Timeouts: 30 min per unit job,
20 min per flight config. Ctrl-C kills all child processes (and leftover harness processes in WSL).

Output: one line per finished job, then a table (suite, jobs, passed, failed, known, wall,
cpu-sum) and the failing jobs' last 15 output lines. Logs: `<tree>/verify_logs/<timestamp>/<job>.log`
and `summary.txt`; every run is appended to `<tree>/verify_logs/summary_all.txt`.

### Flight harness under load: run it on a quiet CPU

Earlier flight failures (drones at 0.00 m, "side-mate silent for 20s") were blamed on CPU
starvation from running many things at once. Measured on the same frozen tree (v5), flight
harness only, 9 configs per round (4 dense x3), background load sampled every ~11 s
(total CPU 6-30 % from other agents while the rounds ran):

| Round | Mode | Result (fails of 9) | Wall | Note |
|---|---|---|---|---|
| 1 | parallel | 5 (3 dense, 4, 4 dense x3) | 5 min | clean |
| 1 | serial | 3 (4 dense x3) | 17 min | clean |
| 2 | parallel | 3 (4 dense x3) | 5 min | clean |
| 2 | serial | 5 (3 dense, 4, 4 dense x3) | 24 min | 51 s machine stall: not counted |
| 3 | parallel | 3 (3 dense, 4 dense x2) | 9 min | 245 s machine stall: not counted |
| 3 | serial | 0 | 10 min | clean |

Reading it:
- Failures happen in BOTH modes on the same code, with the machine mostly idle (serial round 1:
  one harness process, CPU 12 % average). Clean rounds: parallel 8 failed of 18 configs, serial
  3 of 18 (Fisher p = 0.15, not significant). `4 dense` fails in most rounds either way, and
  serial round 3 passed all 9: the harness is non-deterministic on this tree, so this is a
  real problem for the flight code, not a CPU-starvation artefact.
- The machine really does stall or sleep during long runs: `verify.py` saw a 51 s and a 245 s
  freeze, and one earlier serial run lost 53 minutes (laptop suspended; a 20-minute timeout
  never fired because the clock was frozen). Any job overlapping a freeze is unreliable:
  `verify.py` now prints `!! machine stalled/slept ...`, marks those jobs
  `(overlaps a N s machine stall)` and adds a WARNING line to the summary. It also asks Windows not to
  idle-sleep while it runs (a closed lid can still suspend the laptop).
- Running ~170 processes at once (flight alongside the sweep) did make things worse
  (drones at 0.00 m, 20 s silences), so the default is: heavy suites first, harness after.

Default: the 9 flight configs run all at once, AFTER the CPU-heavy suites (`--flight-alongside`
overlaps them; `--serial-flight` runs one at a time, ~3x slower). Parallel was not
distinguishably less reliable than serial on a mostly quiet machine and saves 5-15 minutes.
If a flight config fails, re-run it before calling it a bug:
`python tools/testing/verify.py --tree <snapshot> --only flight --serial-flight`.
A failure that shows up in both modes is real.

### Memory

Default workers = min(cores - 2, free memory / 150 MB), printed at start with the reason (a
Windows job peaks at 10-65 MB; the other agents' runs can leave only ~1 GB free, which used
to crash with `MemoryError`). A job whose output contains `MemoryError`, `Unable to allocate`
and similar is treated as an infrastructure error: it is retried once, alone, after 10 s, and
if it still runs out of memory it is reported as `INFRA: out of memory, not a test failure`.
Pass `--jobs N` to override.

## snapshot.sh and flight_snapshot.sh (test a frozen copy)

Problem: the full tests take 15-20 minutes and you want to keep editing. Solution: copy the
tree, test the copy.

```
bash tools/testing/snapshot.sh mytry                 # copy to .test_snapshots/mytry
cd .test_snapshots/mytry
PYTHONPATH=$PWD python -m flight.pathfinding.pathnode.test_pathnode > unit.log 2>&1 &
python tools/testing/sweep.py --standard --jobs 6 > sweep.log 2>&1 &
```

Real output of the snapshot step, and what is inside:

```
/c/Users/harpe/Multirotor/IARC-10/.test_snapshots/doc_demo

flight/ state_machine/ interdrone_communication/ integration_tests/ tests/ tools/ vision/
main.py run.py mission_config*.json ...  SNAPSHOT_TIME.txt SNAPSHOT_DIFFSTAT.txt
```

Importing from inside the snapshot picked up the copy, not the live tree
(`sim.__file__` = `...\.test_snapshots\doc_demo\flight\pathfinding\pathnode\sim.py`), and
`tools/testing/sweep.py` run from inside it ran fine on the copy.

- `snapshot.sh LABEL [BASE_DIR]`: base defaults to `<repo>/.test_snapshots`. Copies code and
  small configs only (no model files, no `.venv`, no git history).
- `flight_snapshot.sh LABEL [BASE_DIR]`: runs the flight harness on the copy, inside WSL,
  using the repo's `.venv` (set `PY=...` to override). Writes `flight_results.txt`, plus
  `md_default.log`, `md_dense.log`, `one.log`, into the snapshot.

```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/harpe/Multirotor/IARC-10/tools/testing/flight_snapshot.sh mytry > flight.log 2>&1 &
```

- Add `.test_snapshots/` to `.gitignore` (it is not there yet) so copies are never committed.
- Delete old snapshots when done (`rm -rf .test_snapshots/mytry`).
- The copy holds the working tree as it was, uncommitted edits included. Edits made after the
  snapshot are not tested; that is the point.
