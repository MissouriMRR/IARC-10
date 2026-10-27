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
- `dronekit` vehicle: `MockVehicle` takes its position from a motion model
  (`integration_tests/vehicle_motion.py`): `CruiseMotion` (multi-drone default: a straight line at
  `MOTION_SPEEDUP` = 2 x the commanded groundspeed, over real time, so other drones see it in
  transit) or `TeleportMotion` (one-drone test; `teleport` on the multi command line: legs take
  no time). No flight dynamics beyond that, no battery, no wind. The speedup keeps a run inside
  the 300 s timeout; it also scales the age error of position reports (5 Hz at 4 m/s = 0.8 m),
  so do not raise it much (at 5 about half the dense runs aborted on report age alone).
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
- Exit code 0 on PASS, 1 on FAIL. Per drone timeout: 300 s of wall clock. Extra words: `teleport`, `speedup=N`, `abort=ID@SEC`. Not deterministic: drones share one asyncio loop, so timing (and sometimes the result) varies between runs.
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

## Known problems (2026-10-26)

Root causes found and fixed (each has a regression test):

| Symptom | Root cause | Fix |
|---|---|---|
| `4`: `mines found : 13 / 7 true`, mine sets disagree, 200 s+ | The fake camera boxed every detection at the image centre, so each drone put the same mine at the middle of its own photo: up to 4 spots per mine | `TruthCamera` (one_drone_sim_test.py) boxes the mine where it lies and locates it exactly; `tests/test_truth_camera.py` |
| A drone with 12,000+ captures, others waiting in `EndRun` | Standoff: `_make_way` re-picked its spare spot from the nearest ring point of the nearest job, so two jobs made it flip between two spots; Scan re-photographed them 200 times a second and the traffic backed up every inbox | `coordination.py`: a drone already on a spare spot stays put (`_is_spare`); `test_pathnode.py` |
| `held 15 s short of waypoint` then `Land` | The planner dropped photos near a peer but not the *leg* to a photo that passes an idle peer; the flight code holds such a leg until the peer moves | `coordination.py`: `_flyable` drops photos whose leg closes in on a peer, `_blocked_legs` makes an idle drone step out of a side-mate's or other side's announced leg; `test_pathnode.py` |
| `2 dense`: mine sets differ by one mine | One-sided `EndRun` left as soon as its own copy was done, before the mate's last photo arrived | `end_run_impl._settle_with_mate`, `SideAgent.mate_busy` counts a mate that has announced nothing as busy; `tests/test_end_run_settle.py` |
| A late-starting drone plans onto its mate | Until the mate's first POSITION_REPORT the planner assumes it is at the side's start point | `SideAgent.plan` waits while `PeerPositions.mate_unseen()` |

Round 2 (moving fake vehicle; the teleport race is gone, replaced by real crossings it exposed):

| Symptom | Root cause | Fix |
|---|---|---|
| `separation ... fell to 1.1m` at ~8 s, both drones of a side | Both copies start with the same best job and each flies it (the claim is only heard after the first plan); also a "no job" sent before the first plan told the mate it was idle | `Heartbeats` announce nothing before the first plan; `SideAgent.plan`: the higher id waits for the lower mate's first announcement |
| Breach mid-run, side-mates | The planner kept a higher drone off the lower drone's position and next photo only; its leg could cross the lower drone's route a photo later | `coordination._keep_apart`: lanes (position + next 3 photos) of lower drones; `_flyable` rejects legs closing in on a lane; `tests/test_lanes.py` |
| Breach, legs started at the same moment, or onto a drone flying a long leg | A queued leg is flown without replanning; the hold only looked at peers' positions | Leg claims: position reports carry the leg end + Lamport clock (`Drone.claim_leg`, `conflictsForLeg(my_claim=)`); the later claim waits, only the spot a waiting drone stands on blocks; a claimed leg held HOLD_REPLAN_S (2 s) is dropped (`LegBlocked`, Scan replans); `tests/test_leg_claims.py` |
| Other drones in `EndRun` for 300 s after one aborted | Nothing told them the drone was gone (silence rule: 120 s + budget) | `Land`/Scan failsafe send MISSION_END (3x); `DroneState.mission_over`; `MateWatch` (landed = lost+gone), `Departures` (`partner_gone`, `reporter_gone`); EndRun stops waiting; `tests/test_landing_notice.py` |

Fault injection: `abort=ID@SECONDS` on the multi command line latches an abort in that drone (real
path: FormationLost -> Land). The survivors finish alone (150-180 s); the PASS line then fails (mine
sets / route), which is expected. "Partner side gone" calls `SideAgent.partner_lost()` ->
`MissionPlanner.partner_lost()` (design Q8); EndRun then returns CalcScanPath while work is left,
else reports its own half (`reported_half_route`).

Still open: occasional breaches at 0.8-1.1 m when the shared event loop stalls (a CPU-bound plan
holds every drone's report for 0.3 s: not possible with one CPU per drone); see the pass counts in
the round-2 report. The "all four in EndRun at 300 s" runs were not reproduced after the landing
notice; the only 300 s hangs seen were a drone waiting on an aborted reporter.

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
