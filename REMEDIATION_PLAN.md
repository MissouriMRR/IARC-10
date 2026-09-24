# Remediation plan — findings from the 2026-09-21 audit

Companion to [`interdrone_communication/TECHNICAL_DEBT.md`](interdrone_communication/TECHNICAL_DEBT.md).
This plan sequences the fixes; that register holds the messaging-layer detail.

## Context

A read-only audit of the state machine, the messaging layer, and the mission core turned up
roughly fifty findings. They are not evenly weighted: one blocks flight outright, a cluster of
them share a single root cause, and the rest is ordinary debt.

**The systemic finding is silent failure.** In five independent places this system cannot
distinguish success from failure, and in three of those it keeps flying as if nothing happened:

- a mission-state exception leaves the drone hovering until battery failsafe, unlogged
- one malformed message silently kills all inter-drone comms, including `EMERGENCY_LAND`
- a failed camera capture reports the *previous* waypoint's footprint as covered
- `repair_path_after_expansion` returns the same value whether it converged or gave up
- a fully mine-blocked field reported as a completed scan (already fixed — `_check_no_valid_path`)

That pattern, not any single bug, is what this plan is mainly built to close. A system that
fails loudly can be debugged at competition; this one cannot.

**Ordering principle.** Flight-blocking first, then the silent-failure class, then correctness,
then ownership and structure. Within each phase, cheapest-and-highest-value first.

**Relationship to the existing plan.** The pathfinder debt plan
(`.claude/plans/enumerated-humming-fiddle.md`) is partly done: mine-grid alignment (Step 1),
the `mine_safety_radius_ft` rename (Step 2), and the test gate (Step 4) are complete and
verified. Its Step 3 is absorbed into **A3** below. Its Phase 3 structural work resumes only
after Phase B here, and **one item in it is now wrong**: it said `Pathfinder.instance` should
stay on the façade. It should not — see **D1**.

**Status markers.** `VERIFIED HERE` = re-checked directly against the source, with the result
recorded. `REPORTED` = from the audit, not independently re-run; confirm before acting.

---

# Phase A — Blocks flight

Nothing should be flown until these three are done.

## A1. Field corner ordering and validation

**`VERIFIED HERE`, and this is the most severe finding in the audit.**

With the real `mission_config.json`, `Field.withinField` rejects 6 of 7 interior field points,
including the exact centre:

```
real config    (40,150)→False  (10,50)→False  (70,250)→False  (5,150)→False   1/7 inside
sim harness    (40,150)→True   (10,50)→True   (70,250)→True   (5,150)→True    7/7 inside
```

Three conventions are in play and none is enforced:

| Where | Expects |
|---|---|
| `flight/pathfinding/utils/coord_convert.py:6-33` | cyclic `[origin, +x, diagonal, +y]` |
| `flight/pathfinder.py:307-312` | converts cyclic → `Field`'s Z-order `[TL, TR, BL, BR]` |
| `flight/pathfinding/nodeField/field.py:44-49` | Z-order, and warns any other order "builds the wrong edges, so `withinField` rejects the whole field" |
| `README.md` + all four `mission_config*.json` | Z-order |

So the configs feed Z-order into a function expecting cyclic. Swapping the last two corners
takes `withinField` from **1/7 to 7/7** — verified directly.

There is a second, separate defect in the committed file: it places the whole field at negative
y (`y ∈ [-302, 0]`) while every grid, the ending-node row at `HEIGHTOFFIELD+1`, and
`format_iarc_path` assume `[0, 300]`. The current uncommitted reorder fixes the sign and should
be kept — it is a real fix, not scratch — but it does not fix the ordering.

**Do:**
1. Correct the corner order in `mission_config.json` (swap the last two entries so the order
   runs around the perimeter, not in a Z). Same check for the other three config files.
2. Add a loader in `flight/pathfinder.py` (or a new `mission_setup` module) that ingests corners,
   **validates the quad** — convexity, near-perpendicularity, plausible aspect ratio — and emits
   both orderings from one source. Raise on a bad quad rather than silently building a wedge.
3. Assert at `Pathfinder` construction that all four local corners satisfy
   `0 <= y <= HEIGHTOFFIELD` and `|x| <= WIDTHOFFIELD`, and raise with the computed corners in
   the message.
4. Delete the rival convention from `README.md` (it also documents a config schema that no
   longer exists — see E3).

**Verify:** a regression test asserting `withinField` is True for a grid of interior points and
False outside, run against **the real config file**, not synthetic corners. This is the specific
gap that let the bug survive — see the note on sim/production divergence below.

## A2. Config validation

`REPORTED`, low risk to fix.

`get_mission_config` (`state_machine/mission_config.py:99-110`) is `json.load` cast to a
TypedDict with zero runtime validation. Consequences found:

- `mission_config2.json`, `mission_config_full.json`, `mission_config_solo.json` all lack
  `app_info.latitude`/`longitude`, which `flight_settings.py:359-360` indexes directly →
  `KeyError` during startup, before any logging that would explain it. **Three of the four
  configs cannot start.**
- `mission_type: "Competition"` in `mission_config_full.json` is invalid; the validating setter
  (`flight_settings.py:398-410`) is bypassed because `__init__` assigns the private field
  directly (`:191`). It then behaves as "not Prompted" and waits silently for an app command a
  console operator will never send.
- `mission_data_path`'s setter (`:842-853`) has a docstring and signature but **no body** — it
  silently discards its argument.
- Nothing validates that `start_coord` lies inside `mission_field_corners`. The current file
  demonstrates the failure: a Tulsa start against Huntsville corners maps to local
  `(2_239_383, -1_704_483)`, which degrades into "no path found" rather than failing at startup.

**Do:** one `validate(config)` pass that reports *every* missing or mistyped key at once, parses
`mission_type` into an Enum, checks `start_coord` against the polygon, and cross-checks
`drones_in_mission ⊆ {d["id"] for d in drone_info}`. Fix the empty setter.

## A3. Wire the cross-pair boundary into production

`VERIFIED HERE` (independently confirmed by two separate reviews).

`set_cross_pair_home_boundary` is called only from `flight/pathfinding/tests/droneWorkflowTest.py`.
Production still calls `retarget_approach_target` (`calc_scan_path_impl.py:232`) — the design
whose own docstring records that it drove a pair deep into the other's half on **5 of 5 dense
seeds**. Every real 3- or 4-drone mission runs the known-bad path.

This is the absorbed Step 3 of the pathfinder plan; the deletion set is enumerated in
`interdrone_communication/TECHNICAL_DEBT.md` finding 18. Note the boundary call must use the
field height from **A1**'s validated geometry, not the `HEIGHTOFFIELD` constant — see D3.

**Verify:** `python integration_tests/multi_drone_sim_test.py`, plus a grep gate that no deleted
symbol survives.

---

# Phase B — Make failures loud

This phase is the point of the plan. Each item converts a silent failure into a loud one.

## B1. A mission-state exception must land the drone, not hover it

`VERIFIED HERE` — the most dangerous behaviour found.

Every mission impl catches only `asyncio.CancelledError`. An exception from `State.run()`
propagates through `StateMachine._run` to `await run_task` at
`state_machine/state_machine.py:124`, which also catches only `CancelledError` — so it skips
`self.run_task = None` at `:128-129`. `FlightManager.run_manager`
(`state_machine/flight_manager.py:87`) then loops forever on
`if state_machine.run_task is not None ... continue`. The exception sits in a task nobody
awaits, so nothing is logged. The vehicle holds GUIDED at its last commanded point until the
battery failsafe.

Named reachable triggers, all mid-flight:
- `drone_share_impl.py:60` `camera.capture_and_detect_mines()` and `scan_impl.py:144`
  `camera.capture_image()` — no try/except at all
- `drone_share_impl.py:164-167` uses `Pathfinder.instance` with no `None` guard, where
  `end_run_impl.py:48` and `expand_nodes_impl.py:63` both do guard
- `app_share_impl.py:99/104` `path_file.write_text(...)` — an `OSError` on a *debug log file*
  (full SD card on a Pi) prevents the drone reaching Recall/Land

**Do:** give `StateMachine.run` an `except Exception` that logs with `logging.exception` and
transitions into `EmergencyLand`; fix the `run_task` leak so the manager cannot hang; wrap
camera I/O per call. POIF already has the right shape (`poif_impl.py:767-795` maps domain
errors → Land) — mirror it.

## B2. The receive loop must survive a bad message

`VERIFIED HERE`. `state_machine/interdrone.py:1482` wraps the whole loop in
`except KeyboardInterrupt`, which asyncio never delivers to a task. There is no
`except Exception`, and the task is spawned fire-and-forget with no done-callback — though the
file already defines `_log_task_failure` (`:118`) and uses it elsewhere. One `KeyError`
permanently ends all comms: no position reports for collision avoidance, no `EMERGENCY_LAND`,
no `LAND`, while the state machine keeps flying.

**Do:** wrap per-message dispatch in `try/except Exception: logging.exception(...)`; attach
`_log_task_failure` at both spawn sites (`flight_manager.py:79-81`, `main.py:25`).

## B3. Message validation must reject, not hollow out

`VERIFIED HERE`. `Message.__post_init__`
(`interdrone_communication/message_types.py:702-717`) warns and sets `data = {}` on any schema
mismatch, then delivers the message anyway. ~25 handlers index `message.data["key"]` directly.
Combined with B2 this is the most likely route to a dead comms layer.

**Do:** make validation failure reject the message (raise, or return a sentinel the loop drops
with a log). That removes the need for defensive `.get` at 25 sites rather than adding it.

## B4. Clear stale photo corners

`REPORTED`. One line, prevents permanent silent data corruption.

`_capture_for_assistant` returns early at `scan_impl.py:57` (no camera) and `:62` (capture
failed) without clearing `drone.last_photo_corners_latlon`; only the corner-ray failure path at
`:97` clears it. `drone_share_impl.py:198-206` then reads the *previous* waypoint's corners and
sends them as this waypoint's `SHARE_PHOTOS` footprint. The paired GAMBLER calls
`accept_image_corner_coord` on them, permanently marking as covered a cell that was never
photographed — and `get_places_to_check_maze` will never queue it again.

**Do:** set `last_photo_corners_latlon = None` at the top of `_capture_for_assistant`, before
any early return.

## B5. Distinguish "repaired" from "gave up"

`REPORTED`. Same class as the already-fixed `_check_no_valid_path`, and higher stakes: this is
the route the competition scores.

`repair_path_after_expansion` (`flight/pathfinder.py:588-595`) returns `pass_num` on convergence
**and** returns `MAX_EXPANSION_REPAIR_PASSES` when it exhausts the cap without converging.
`expand_nodes_impl.py:65` only logs it. `end_run_impl.py:71` then writes the possibly-invalid
route into `mission_path` as the authoritative answer, and `AppShare` reports it.

**Do:** mirror `_check_no_valid_path` — set a flag on `Drone` when the cap is hit and have
`AppShare` say so explicitly. Better: return an explicit `(converged: bool, passes: int)`.

## B6. Bound the camera-failure livelock

`REPORTED`. If capture fails, `_capture_and_mark_seen` returns at `scan_impl.py:133`/`:147`
without marking `seen_tracker`. `CalcScanPath` recomputes, gets the same cell back, and
`Scan → DroneShare → Scan` re-flies the same waypoint until `max_flight_time` (420 s) expires.
The flight budget is the only thing ending it.

**Do:** count consecutive no-coverage captures; fail the state (or flag the drone) after a
threshold instead of relying on the time budget.

---

# Phase C — Correctness

Independent fixes; each verifiable on its own. Ordered by severity.

| # | Finding | Location | Status |
|---|---|---|---|
| C1 | `DISARM` fan-out addresses `message.sender_id` instead of `other_drones_in_mission` — drone 1 disarms itself and tells the app; drones 2..N never hear it. `EMERGENCY_LAND`/`LAND` do it correctly. | `interdrone.py:1590` | `VERIFIED HERE` |
| C2 | `touch()` called on 6 of ~25 receive cases, so a demonstrably alive peer can be judged stale and have its route ignored by collision avoidance. Move it into the loop, before dispatch. | `interdrone.py` | `REPORTED` |
| C3 | `recall()` considers only the `+x`/`+y` edges: at `x=5, y=150` it compares `80-5=75` vs `300-150=150` and flies the full field width to `x=80` instead of the edge 5 units away. Docstring claims "nearest field edge". Wait loop has no timeout, no abort check, and never re-commands. | `drone.py:1171-1179` | `VERIFIED HERE` |
| C4 | `SEND_GROUND_TRUTH_COORDS` blocks the receive loop 5 s inline, never stores the offset it computes (leaves `0.0`), then distributes `0.0, 0.0` swarm-wide and acks success. Two `TODO`s confirm the placeholder is knowing — the misleading ack is the bug. | `interdrone.py:2098-2118` | `VERIFIED HERE` |
| C5 | `resetWaypoints` called from the interdrone receive task races `gotoWaypoint`, which holds `waypoints[0]` across many awaits then `pop(0)`s. The popped waypoint may not be the one flown → wrong photo label, dropped coverage, `IndexError` on an empty list. Every other cross-drone message stages-and-flags; this one mutates live flight state. | `interdrone.py:1778`, `drone.py:998/1145` | `REPORTED` |
| C6 | Verification waypoints are drained into `resetWaypoints` (a *replace*), so a replan before they are flown discards them permanently — after the sender was already acked "queued". Nothing re-sends. | `calc_scan_path_impl.py:254-267` | `REPORTED` |
| C7 | ASSISTANT idle wait is a hot spin: `CalcScanPath → Scan → CalcScanPath` with one `sleep(0)` per iteration, at 100% event-loop CPU for up to 30 s — starving the very interdrone loop that would deliver the work it is waiting for. | `calc_scan_path_impl.py:149`, `scan_impl.py:308` | `REPORTED` |
| C8 | `last_sent_segment_b` is recorded even when the send was a silent no-op (unknown target id), so the dedupe guard suppresses all retries: an ASSISTANT that never gets work and a GAMBLER that thinks it delegated. | `interdrone.py:562-585` | `REPORTED` |
| C9 | Role/pairing derivation keys off `len(drones_in_mission)` and hardcoded IDs. A roster of `[2,3,4]` pairs drone 2 with drone 1 — which is not flying — and drone 4 falls through to SOLOGAMBLER. A drone absent from `drones_in_mission` is invisible to collision avoidance. | `flight_settings.py:200-239` | `REPORTED` |
| C10 | `state_tracker.update_drone` reads `drone.odlc_scan`, which `Drone` does not define (SUAS-era leftover). Harmless only because `state.json` is gitignored and absent; restoring that file makes **every** state transition raise. | `state_tracker.py:119-120` | `REPORTED` |
| C11 | The `atomic` lock is never taken by any production state, yet `emergency_land_impl.py:50` asserts "every other state uses one". An EMERGENCY_LAND during `_drain_cross_pair_mines` can cancel between the pop and the await, losing relayed mine positions and leaving the graph half-mutated. | `state.py:76` | `REPORTED` |
| C12 | Drained pending-report batches are popped wholesale into a local before awaiting per item — a raise or cancellation loses the remainder. For relayed *mine* positions this is silent loss of safety data. | `calc_scan_path_impl.py:185-195` | `REPORTED` |

---

# Phase D — Ownership and structure

Only after Phase B. These are large moves and need the loud-failure behaviour in place first.

## D1. Give `Pathfinder` an owner — correcting the earlier plan

`REPORTED`, and it **supersedes** the pathfinder plan's Step 8, which said the singleton should
stay on the façade.

`Pathfinder.instance` (`flight/pathfinder.py:246`, assigned `:410-411`) is set **only if
currently `None`** and never cleared. In a single-process multi-drone run, every drone after the
first builds a `Pathfinder` that is unreachable, and all its states silently operate on drone
1's graph — `ExpandNodes.increase_radius` applied N times to one field, `EndRun` overwriting
another drone's `mission_path`. `configureField` stores the object nowhere, relying entirely on
the singleton. The only mitigation is a monkeypatch in `integration_tests/multi_drone_sim_test.py`;
production has none. `takeoff_impl.py:223`'s comment that `__init__` "already sets
`Pathfinder.instance`" is wrong for every drone but the first.

**Do:** `configureField` assigns `drone.pathfinder`; readers take it from the `Drone`; keep
`Pathfinder.instance` only as a deprecated alias, or delete it. Two production readers currently
have no `None` guard (`calc_scan_path_impl.py:274`, `:310`) while five others do — make the
discipline uniform.

## D2. Extract the inbox off `Drone`

`REPORTED`. `Drone` (1179 lines) owns six unrelated concerns. The highest-value seam is the
interdrone staging area — `replan_needed`, `pending_cross_pair_mines`,
`pending_verification_waypoints`, `pending_point_a_sync`, `pending_photo_reports`,
`last_sent_segment_b`, `last_synced_point_a`, `assistant_idle_since` — eight public attributes
with a cross-task contract, currently mutated from four places in the receive loop and cleared
in a fifth. Replace with a `MissionInbox` exposing `stage()`/`drain()`.

Also delete the dead weight: `field_size`, `seen_tracker`, `field`, `start_node`, `end_nodes`,
`tasks`, `mine_radius`, `setFieldSize` (which would `TypeError` if called), `updateWaypoints`,
`getWaypointChecksum`. Note `seen_tracker` allocates a 3600×960 numpy array (~3.4 MB) on every
`Drone()` and is read nowhere.

## D3. Field dimensions from validated geometry

Folds together A1 and the pathfinder plan's Step 6. `WIDTHOFFIELD`/`HEIGHTOFFIELD` are module
constants imported **by value** into five modules including production `drone.py:1171`, while
production derives real geometry from config. After A1 there is a validated source of truth;
make `width_ft`/`height_ft` instance attributes derived from it, and keep the constants as
sim defaults only, clearly labelled.

## D4. Split the two oversized files

- `interdrone.py` (2219 lines, four responsibilities) — seams and order in
  `interdrone_communication/TECHNICAL_DEBT.md` P3. The dispatch table (finding 8) also closes
  C2 structurally, by making `touch()` and `try/except` uniform rather than per-case.
- `droneWorkflowTest.py` (2671 lines) — drivers / assertions / renderers, per the pathfinder
  plan's Step 7.

Then resume the pathfinder plan's Phase 3 (collaborator extraction), with D1 replacing its
singleton assumption.

---

# Phase E — Dead code and documentation

Low risk, do opportunistically.

- **E1.** Eight message types have a full send helper, no receive case, and no caller
  (`SURVEY_*`, `MISSION_END*`, `SEND_SEEN*`) — ~120 lines. Nine more are schema'd but never sent
  or handled. Two are handled but unreachable. Detail in the register, findings 13–15.
- **E2.** Dead attributes and imports across the state impls: `last_captured_image` (written
  twice, read nowhere), `extract_gps` imported unused in three files, an unreachable `return`
  after `while True:` at `takeoff_impl.py:180`, `waypoint_up_to_date` written in six places and
  read nowhere.
- **E3.** `README.md` documents a config schema that no longer exists (`current_drone_ID`,
  `app_IP`, `number_of_total_drones`, `other_drone_info` vs the current `self_id`, `app_info`,
  `drones_in_mission`, `drone_info`) and carries the wrong corner diagram from A1.
- **E4.** Entry points: `run.py` is real; `main.py` and `networking_main.py` are two
  near-identical networking smoke tests, one of which is half-migrated to a changed `Waypoint`
  signature and passes *lat = drone id* positionally. Keep `run.py`, fold the others into
  `integration_tests/`, document the invocation in the README.
- **E5.** Stale docstrings, ~12 of them, including `states/poif.py` being a wholesale copy-paste
  from `land.py` (its module docstring, class docstring and type annotations all say "Land"), and
  `poif_impl.py:799` logging "Land state canceled" from POIF's cancellation handler.

---

# Verification strategy

Three of this session's bugs — the cross-pair retarget, the mine-grid misalignment, and the
corner ordering — share one shape: **the simulation and the real configuration disagree, and only
the simulation is tested.** 25 seeds × 2 drivers pass cleanly while production is broken, because
`build_empty_pathfinder` constructs its own correctly-ordered corners.

So the verification rule for this plan is: **test against the real config, not synthetic inputs.**

Per phase:

- **A:** a `withinField` regression test driven from the actual `mission_config*.json` files; a
  config-validation test asserting each file either loads or fails with a named key. Then
  `integration_tests/multi_drone_sim_test.py`.
- **B:** fault injection, since these are error paths — raise from a camera stub, deliver a
  malformed message, make `write_text` raise. Each must produce a log line and a safe
  transition, not a hang. Without injection these fixes are untested by construction.
- **C:** unit-level where possible (C1, C3 are pure logic); C5/C6/C7 need the two-drone harness.
- **D:** the existing gate — `droneWorkflowTest.py main()` plus
  `sweep_two_pairs(n_seeds=150+)` in both driver modes, run after **each** extraction, not at
  the end. A pure move should change no counter; any drift means the move was not pure.

Standing discipline, from prior experience on this code: one change at a time, re-run the full
ladder after each, and never trust "0 failures across N seeds" alone — cross-check a denser
minefield before concluding a path is unreachable. A fix here has already been observed to
introduce a different regression elsewhere.

## Commit hygiene

Nine files currently carry uncommitted work and nothing is committed, so there is no
known-good point to roll back to. Commit the verified work (mine-grid alignment, the
`mine_safety_radius_ft` rename, the test gate) **before** starting Phase A, so each phase below
has a clean base and a rollback target.
