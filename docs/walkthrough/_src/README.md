# Execution walkthrough: a drone swarm's state machine and its path-finding

This is a guided tour of the flight code, told by following one run as it executes.

## The mission

A swarm of drones maps a **minefield**. Each drone flies over the field at 2 m taking photos. A photo covers some 2 ft squares; if it contains a mine, that mine becomes an obstacle. The result the team reports (to the app, in the IARC text format `S,col,buffer / U,n / L,n ...`) is **one safe path from one edge of the field to the other**, made only of ground that was actually photographed and that keeps clear of every mine found.

We follow **a 4-drone mission**. Drones 1 and 2 (the *bottom side*) start on the bottom edge; drones 3 and 4 (the *top side*) start on the top edge. Each side builds a route from its own edge toward the *other side's frontier* ("point A"), so the two halves meet in the middle and are joined. It is told mostly from **drone 1's point of view**, with side notes for drones 2, 3 and 4. (There are no roles: the drones of a side are equals, and every drone keeps its own copy of its side's planner.)

## How to read this

- Chapters follow the execution order. Each step has a short "what happens" line, the **actual code** (copied from the repository, with a `path:line` header), a bulleted explanation of the lines that matter, and a "goes next to ->" pointer.
- The **same code is revisited** when the inputs change: chapter 3 is `CalcScanPath` with nothing known, chapter 6 is the same functions after a mine is found, chapter 7 is how a message from another drone changes the next pass.
- Chapters 3, 5 and 8 to 10 include **toy runs**: small scripts (in `_src/demos/`) that drive the real planner classes and print real output. Their numbers are exact for the scenario shown, not measurements from a flight.
- Plain words first, code names second; every term is defined where it first appears and again in the [glossary](11-glossary.md).
- Sections marked **Status** say what is still in flux (see below).

## Chapters

| # | Chapter | What it explains |
|---|---|---|
| 1 | [Process start](01-process-start.md) | `run.py`, `FlightManager`, the state machine loop and the receive loop running side by side, `FlightSettings` and sides, the `Start` state (connect, arm, ping) |
| 2 | [Takeoff and planner setup](02-takeoff-and-planner-setup.md) | `Takeoff` mission branch; `configureField` -> `MissionPlanner` -> `PathNodePlanner` (config, chain, routes, target, coverage, shortcuts, work, claims); the field frame; `SideAgent`; camera; position stream |
| 3 | [The first CalcScanPath pass](03-first-calcscanpath.md) | `side.plan()`: inbox, heartbeats, `plan()`: `advance`, `groups`, `ClaimBook.update`, keep-apart, idle takeover, job announcement; the waypoint queue |
| 4 | [Scan](04-scan.md) | `gotoWaypoint`: holds, separation abort, identity-based removal; photo, real footprint, climb-retry; coverage and misses |
| 5 | [DroneShare and the mine rule](05-droneshare-and-mine-rule.md) | Mine detection, `own_photo`, `add_mine`, **the mine rule** (delete, cut, replan, orphans, bridge, jump), relays and sharing |
| 6 | [CalcScanPath after a mine](06-calcscanpath-after-mine.md) | The same functions on the changed chain: new groups, claims, keep-apart; the pass after a photo with no mine |
| 7 | [Messages from other drones](07-messages-from-other-drones.md) | Receive loop staging, `apply_inbox` order, a concrete "side-mate finds a mine mid-leg" case, `replan_needed` |
| 8 | [Shortcuts](08-shortcuts.md) | Anchors, candidates, trials, splice, with real runs |
| 9 | [Two sides meeting](09-two-sides-meeting.md) | Target and stand-in, point A exchange, closing the gap, join, reopening, cross-side keep-apart |
| 10 | [Finishing](10-finishing.md) | `done()`, `ExpandNodes` (a no-op), `EndRun` join protocol, `AppShare`, `Recall`, `Land`, the time limit |
| 11 | [Glossary](11-glossary.md) | Every term |
| 12 | [Notes and oddities](12-notes-and-oddities.md) | Bugs, dead code and stale comments noticed on the way |

## Status of the code being described

- **Obstacle geometry (being replaced).** The planner routes with `ZoneMap` (`pathnode/geometry.py`, wrapped by `pathnode/routes.py`). A rebuilt geometry (`flight/pathfinding/nodeField/`: `Field`, `FieldConnections.nodeGraph`, `MineNode`, `PolygonObstacle`, `unionObstacle`, `BlockMine`; and `flight/pathfinding/protoMine.py`) is finished but **not wired in yet**. The walkthrough describes path-finding through `Routes` as "shortest path around known obstacles" and does not go into `ZoneMap`. It will be extended when `Field` is wired in.
- **Per-drone planner copies, keep-apart (including cross-side keep-apart) and claims** are in final verification (`planner.py`, `claims.py`, `session.py`, `sim.py`, `side_agent.py`); the text follows the code as read. The geometry is still `ZoneMap`; the new `Field` is not wired in.
- **Expanding mines (`ExpandNodes`) is not implemented**: the state is a no-op today (chapter 10).

## How these files are produced (for maintainers)

The chapter files are **generated**. Do not edit `docs/walkthrough/*.md` directly; edit the templates in `docs/walkthrough/_src/` and rebuild:

```text
python docs/walkthrough/_src/build.py
```

- A template line `{{snip: path :: start => end :: start2 => end2 +N}}` inserts a code block copied verbatim from the repository, with a `# path:line-line` header. Ranges are found by **anchor text** (first line containing `start`, then the first line at or after it containing `end`), so a rebuild picks up moved code and new line numbers automatically. If an anchor no longer exists the build **stops and names it**: that chapter needs updating.
- A template line `{{run: demos/name.py}}` runs a toy script from the repo root and inserts its output.
- Toy scripts import the real planner classes; if the planner API changes they fail loudly and print the error in the chapter.

## Coverage table: file:function -> chapter

| File : function or class | Chapter |
|---|---|
| `run.py` (entry point) | 1 |
| `state_machine/flight_manager.py` : `FlightManager.run_manager`, `kill_switch` | 1 |
| `state_machine/state_machine.py` : `StateMachine._run`, `run` | 1 |
| `state_machine/states/state.py`, `states/*.py` (state classes), `state_tracker.py` | 1 |
| `state_machine/flight_settings.py` : `FlightSettings.__init__`, `mission_sides`, `side_drones`, `partner_side_drones`, `paired_drone`, `start_side`, `from_mission_config` | 1 |
| `state_machine/mission_config.py` : `MissionConfig` | 1 |
| `state_machine/states/impl/start_impl.py` : `run` | 1 |
| `state_machine/interdrone.py` : `ping_drones`, `send_ARM`, `all_armed`, `cancel_state`, `restart_state_machine`, `interrupt_into`, `start_emergency_land`, `start_land` | 1 |
| `state_machine/interdrone.py` : `interdrone_loop` (structure, `ARM`/`PING` cases) | 1, 7 |
| `interdrone_communication/networking_interface.py` : `queue_client_message`, `try_get_server_message` | 1 |
| `state_machine/states/impl/takeoff_impl.py` : `run` (mission branch), `configureField` | 2 |
| `state_machine/interdrone.py` : `stream_positions`, `report_position` | 2, 7 |
| `state_machine/states/impl/calc_scan_path_impl.py` : `_camera_footprint_ft`, `SHAPE_SIZE_FT`, `OVERLAP` | 2 |
| `flight/pathfinding/mission_planner.py` : `make_planner`, `MissionPlanner.__init__`, `latlon_to_local` | 2 |
| `flight/pathfinding/frame.py` : `field_transformer`, `validate_field_corners` | 2 |
| `flight/pathfinding/pathnode/planner.py` : `PathNodePlanner.__init__` | 2 |
| `flight/pathfinding/pathnode/config.py` : `PlannerConfig`, enums, constants | 2 |
| `flight/pathfinding/pathnode/chain.py` : `Chain`, `PathNode`, `Link` | 2 |
| `flight/pathfinding/pathnode/routes.py` : `Routes` (interface only), `geometry.py` (not detailed; being replaced) | 2 |
| `flight/pathfinding/side_agent.py` : `SideAgent.__init__`, `speaker` | 2 |
| `state_machine/states/impl/calc_scan_path_impl.py` : `run`, `_build_waypoints`, `_flag_no_valid_path` | 3 |
| `flight/pathfinding/side_agent.py` : `plan`, `apply_inbox`, `send_heartbeats`, `_announce_job`, `positions`, `other_side_positions`, `_check_mate` | 3, 7 |
| `flight/pathfinding/mission_planner.py` : `assignments`, `current_job` | 3 |
| `flight/pathfinding/pathnode/session.py` : `assignments` | 3 |
| `flight/pathfinding/pathnode/planner.py` : `plan`, `advance`, `groups`, `_keep_apart`, `_make_way` | 3 |
| `flight/pathfinding/pathnode/chain.py` : `advance_link` | 3 |
| `flight/pathfinding/pathnode/work.py` : `Work.groups`, `band`, `needed_squares` | 3, 6 |
| `flight/pathfinding/pathnode/coverage.py` : `places_along`, `need`, `touched`, `tile` | 3, 6 |
| `flight/pathfinding/path_cover.py` : `path_cover_unseen` (used, not detailed) | 3 |
| `flight/pathfinding/pathnode/claims.py` : `ClaimBook.update`, `_inherit`, `_queue` | 3, 6 |
| `state_machine/interdrone.py` : `send_side_job`, `send_cross_pair_point_a`, `send_cross_pair_mine_sync` | 3, 9 |
| `state_machine/states/impl/scan_impl.py` : `run`, `_capture_and_mark_seen` | 4 |
| `state_machine/drone.py` : `gotoWaypoint`, `conflictsForLeg`, `checkSeparation`, `_check_still_ours`, `resetWaypoints` | 4 |
| `state_machine/drone_state.py` : `believed_occupancy`, `occupied_segment`, `position_is_stale`, `is_stale` | 4 |
| `flight/waypoint.py` : `COLLISION_RADIUS`, `SEPARATION_ABORT_M` | 4 |
| `flight/pathfinding/utils/goto.py` : `move_to` | 4 |
| `flight/pathfinding/mission_planner.py` : `own_footprint`, `cell_seen_at`, `failed_photo` | 4, 5 |
| `flight/pathfinding/pathnode/planner.py` : `record_footprint`, `record_failed_photo`, `_misses`, `_block_square` | 4 |
| `flight/pathfinding/pathnode/coverage.py` : `record`, `count_misses` | 4 |
| `state_machine/states/impl/drone_share_impl.py` : `run`, `_check_photo_latlon` | 5 |
| `flight/pathfinding/side_agent.py` : `own_photo`, `failed_photo` | 5 |
| `flight/pathfinding/mission_planner.py` : `own_photo`, `relay_id`, `mate_photo`, `partner_*` | 5, 7 |
| `flight/pathfinding/pathnode/session.py` : `own_photo`, `mate_photo`, `_add_pair_mines`, `partner_mine`, `partner_coverage`, `partner_point_a`, `partner_route` | 5, 7, 9 |
| `flight/pathfinding/pathnode/planner.py` : `add_mine` | 5 |
| `flight/pathfinding/pathnode/routes.py` : `_added`, `add_mine`, `add_square` | 5 |
| `flight/pathfinding/pathnode/mine_rule.py` : `apply`, `_frontier_links`, `_shift_entry`, `_reroute_from_entry` | 5 |
| `state_machine/interdrone.py` : `send_cross_pair_mine_relay`, `send_cross_pair_coverage`, `share_photos` | 5, 7 |
| `state_machine/interdrone.py` : receive cases `SHARE_PHOTOS`, `SIDE_JOB`, `CROSS_PAIR_*`, `POSITION_REPORT`, `FIELD_CHECKSUM` | 7 |
| `state_machine/drone.py` : `Drone.__init__` staging fields, `CrossPairMineReport`, `SharedPhotoReport` | 7 |
| `flight/pathfinding/pathnode/shortcuts.py` : `Shortcuts` (all) | 8 |
| `flight/pathfinding/pathnode/chain.py` : `Chain.replace` | 8 |
| `flight/pathfinding/pathnode/target.py` : `Target.route_from` | 9 |
| `flight/pathfinding/pathnode/planner.py` : `set_partner_point_a`, `met`, `point_a`, `check_join`, `_reopen_toward`, `join` | 9 |
| `flight/pathfinding/pathnode/join.py` : `join_routes`, `join` | 9 |
| `flight/pathfinding/mission_planner.py` : `join_latlon`, `check_join_latlon` | 9, 10 |
| `flight/pathfinding/pathnode/sim.py` (headless driver used by toy runs) | 8, 9, 10 |
| `flight/pathfinding/pathnode/planner.py` : `done`, `route_verified`, `mine_hash` | 10 |
| `state_machine/states/impl/calc_scan_path_impl.py` : `_finish_scan` | 10 |
| `state_machine/states/impl/expand_nodes_impl.py` : `run`; `mission_planner.py` : `expand_margin` | 10 |
| `state_machine/states/impl/end_run_impl.py` : `run`, `_join_with_partner` | 10 |
| `state_machine/states/impl/app_share_impl.py` : `run`, `_write_path_file` | 10 |
| `state_machine/interdrone.py` : `push_mission_path`, `send_paths_to_app`, `send_cross_pair_chain`, `send_cross_pair_chain_ack`, `send_checksum` | 9, 10 |
| `flight/pathfinding/iarc_report.py` : `format_iarc_path`; `Drone.get_mission_iarc_path` | 10 |
| `state_machine/states/impl/recall_impl.py`, `Drone.recall` | 10 |
| `state_machine/states/impl/land_impl.py`, `Drone.return_to_launch` | 10 |
| `state_machine/states/impl/emergency_land_impl.py` | 1 (interrupt path only) |
| `state_machine/states/impl/poif_impl.py` (the POIF demo) | not covered |
| `flight/pathfinding/nodeField/*`, `protoMine.py` (new geometry, not wired in) | not covered yet (to be added when wired) |

Chapters 11 and 12 (glossary, notes) cover no code directly; when a change lands, update the notes list too.
