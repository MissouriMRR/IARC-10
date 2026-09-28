# Flight-code integration tests (integration_tests/)

These run the REAL flight code (the state machine: Start, Takeoff, CalcScanPath, Scan,
DroneShare, ..., EndRun, AppShare, Recall, Land) against fake hardware. They answer "does the
whole mission actually work when the pieces are wired together", which the planner unit tests
cannot.

## What is real and what is faked

Real:
- every state in `state_machine/`, `Drone`, `FlightSettings`, the planner and `SideSession`;
- `Interdrone`'s message handling: every case of its receive loop and every `send_*` method;
- the wire format: messages cross an in-process router as real JSON, so encode/decode bugs show;
- the camera geometry (`vision/Cameras/baseCamera.py`: footprint, mount offset, ray casting).

Faked:
- `dronekit` vehicle: `MockVehicle` TELEPORTS on `simple_goto`/`simple_takeoff`; every wait loop
  ends on its first check. No flight dynamics, no battery, no wind.
- camera capture: `MockCamera` checks the ground-truth minefield against the real footprint.
- network: `_MessageRouter` (an `asyncio.Queue` per drone) replaces sockets. `Interdrone` is built
  without its normal `__init__` (which would start a socket thread).
- `dronekit` / `picamera2` imports: `integration_tests/_hardware_shims.py`.

NOT simulated (a pass here says nothing about these):
- **Drone rotation.** Photo footprints are always axis-aligned. The planner sim
  (`Mission(photo_yaw_deg=...)`, see [simulator.md](simulator.md)) covers rotated footprints; the
  flight harness does not.
- **Message drops or delays.** The router delivers everything, in order. Lossy behaviour is only
  tested in the planner sweep (`sweep.py`, configs d/e/f).
- Real GPS error, real timing between drones, collision avoidance in the air (POIF).
- Launch positions are fixed: all drones of a side plan from the field-centre entry, but the
  second drone of a pair starts `LAUNCH_SPACING_FT` = 10 ft to the side (in
  `multi_drone_sim_test.py`), otherwise the live-separation check would trip on the first leg.
  There is no other launch-timing model.

## Where to run: WSL

They need `dronekit`, which is installed in the repo's `.venv` inside WSL Ubuntu-24.04
(not in your Windows Python). Working commands:

Git Bash (note the `MSYS_NO_PATHCONV=1` prefix, or Git Bash rewrites the paths):
```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/harpe/Multirotor/IARC-10 && PYTHONPATH=\$PWD .venv/bin/python -m integration_tests.multi_drone_sim_test 4 dense"
```
Inside a WSL shell already:
```
cd /mnt/c/Users/harpe/Multirotor/IARC-10
PYTHONPATH=$PWD .venv/bin/python -m integration_tests.multi_drone_sim_test 4 dense
```

## multi_drone_sim_test.py

```
... -m integration_tests.multi_drone_sim_test            # 2, 3 and 4 drones, in turn
... -m integration_tests.multi_drone_sim_test 4          # only 4 drones
... -m integration_tests.multi_drone_sim_test 4 dense    # 4 drones on a random 60-mine field
```

- Drone counts: 2 = one side of two; 3 = a pair plus one alone on the far edge; 4 = two pairs.
- Default field: one column of 7 mines up the middle (every 40 ft). "dense": 60 random mines
  (`generate_true_minefield(60, 3)`) which gives the planner real detours to hand out. Run both;
  the default is quick, dense is the harder one.
- Exit code 0 on PASS, 1 on FAIL. Per drone timeout: 300 s of wall clock. Not deterministic: drones share one asyncio loop, so timing (and sometimes the result) varies between runs.
- Time (measured): default (2+3+4 drones) about 1 minute; `dense` (2+3+4) about 6 minutes, more if a
  drone hits its 300 s timeout; one-drone about 13 s. Whole step: ~8 min when everything passes.

### What PASS means

Real output (one configuration of the default run):

```
======================================================================
4-DRONE STATE MACHINE SIMULATION -- REPORT
======================================================================
-- drone 1 (bottom side) -----------------------------------------
   final state : Land
   elapsed     : 13.0s
   error       : none
   mines found : 4 / 7 true
   seen cells  : 1594
   reported    : 42 points -- full field, continuous, clear
   captures    : 48
-- drone 2 (bottom side) ...          (same fields for drones 2-4)
final mine sets agree : True
======================================================================
RESULT: PASS -- all drones reached Land: True
======================================================================
```

PASS needs ALL of:
1. Every drone: `final state : Land` and `error : none`.
2. Drone 1's reported route reads `full field, continuous, clear`. That is
   `check_reported_route`: the route is continuous in the strict IARC format, runs from one
   field edge to the other, and crosses no mine the planner knows about. Drone 1 also must not
   have "reported only its own half".
3. `final mine sets agree : True` (only printed for 2+ drones): every drone's planner ended with
   the same mine set (`mine_hash`), i.e. mine relays worked both ways.

`mines found : 4 / 7 true` is not a failure: a route only needs the mines that lie near it.
Lines like `Drone 3 failed to arm. Resending message.` and `start mission recieved` are normal
noise from the fake operator, not errors.

### Reading a failure

The report line tells you where to look:

| You see | Likely cause |
|---|---|
| `error : timed out after 300s (last state: X)` | a drone hung in state X: usually waiting for a message that never came (a send/receive bug in `Interdrone`) or a busy loop with no `await` starving the others |
| `error : <exception text>` | a real exception in flight code; the traceback is above the report |
| `final state` not `Land` | the mission ended early (abort, emergency land) |
| `reported : ... -- not continuous: ...` | route format broken (`format_iarc_path`) |
| `... does not span the field: ends at y=...` | route stops short of the far edge |
| `... crosses a known mine` | route passes through a known mine zone (planner or join bug) |
| `reported only its own half` | the two-sided join failed |
| `final mine sets agree : False` | a mine relay (CROSS_PAIR_MINE_RELAY / SHARE_PHOTOS) was lost or mishandled |

Then re-run just the failing configuration (`... 3 dense`) and read the traceback lines above
the report. The planner sim (`repro_cases.py`, `sweep.py`) is quicker for planner-only bugs.

## Known problem (2026-09-28): `4 dense` fails

On the working tree of 2026-09-28 (uncommitted flight-code edits in progress), everything passes
EXCEPT `multi_drone_sim_test 4 dense`, which failed on all three runs, each time differently:

| Run | What happened |
|---|---|
| snapshot run | drone 1 `reported : 4 points -- does not span the field: ends at y=0.00 and y=43.90`; drones 2 and 4 still in `EndRun` after the 300 s timeout; `final mine sets agree : False` |
| live tree, run 1 | all four reached Land but `final mine sets agree : False` (drone 2 knew 23 mines, the others 24; drone 4 took only 6 photos) |
| live tree, run 2 | `reported : 0 points -- no route reported`; drones knew 5, 7, 15 and 2 of 60 mines; `agree : False` |

It looks like a race: drones reach Land (or a drone leaves EndRun) before the last mine relays
and the route hand-over have arrived. The result changes from run to run because the drones
share one asyncio loop with wall-clock waits, so the harness is not deterministic (the planner
sim is). Default field (2, 3, 4 drones), `2 dense`, `3 dense` and one-drone passed.
Treat `4 dense` as a real, currently open failure, not a harness quirk, until someone fixes
it; re-check whether this note is still true before relying on it.

## one_drone_sim_test.py

```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/harpe/Multirotor/IARC-10 && PYTHONPATH=\$PWD .venv/bin/python -m integration_tests.one_drone_sim_test"
```

The single-drone version (3 fixed mines, `MockInterdrone` instead of the real one, since a solo
drone sends no messages). Takes about 6 seconds. Real output:

```
======================================================================
ONE-DRONE STATE MACHINE SIMULATION -- REPORT
======================================================================
final state observed : Land
elapsed (wall clock) : 6.0s
ERROR                 : none -- ran to completion
mines: discovered=0 / true=3
reported route        : 39 points
iarc path (first 5 lines):
    S,20,1
    U,150
route check           : ok
camera captures taken : 38
======================================================================
```

There is no "PASS" line here. Pass = `ERROR : none -- ran to completion` AND
`route check : ok` AND exit code 0 (`echo $?` in WSL). Note `discovered=0 / true=3`: the route
never came near the three mines, so none were found; that is fine.

## Shared code

`multi_drone_sim_test.py` imports `MockVehicle`, camera and geometry helpers, and
`check_reported_route` from `one_drone_sim_test.py`. If you change one, run both.
`integration_tests/` is being edited by other people at the same time as this doc was written:
check `git log -- integration_tests` if a description here looks stale.
