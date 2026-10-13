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
