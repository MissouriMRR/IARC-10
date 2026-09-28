# The mine-field search: how it works and how it plugs in

This document describes the **path-node planner**: the search that finds and verifies a
safe route across the mine field, and how it is wired into the flight state machine. It is
organised around three questions:

1. **What happens, in order?** The mission process ([§2](#2-the-mission-process)).
2. **What data is kept, and where?** The data structures ([§3](#3-data-structures-and-where-they-live))
   and the path data ([§5](#5-path-data-storage-formats-and-conversions)).
3. **Who calls whom?** The interface layers, from the state machine down to the geometry,
   and the messages between drones ([§4](#4-interfaces-layer-by-layer), [§6](#6-messages-between-drones)).

The old maze planner (`flight/pathfinder.py`) is archived under `archive/legacy_planner/`;
nothing here uses it.

> This describes the code as of 2026-09-28. The design is still moving (generic drone roles,
> a data-structure redesign, the expanding-mines phase are all pending); update this file
> with each change. Known problems and open work are listed in [TODO.md](TODO.md).

---

## 1. Where the code is

| Layer | File | What it holds |
|---|---|---|
| Entry | `run.py` | Builds `FlightSettings`, starts `FlightManager`. |
| Process | `state_machine/flight_manager.py` | Runs the state machine and the interdrone loop as two concurrent asyncio tasks. |
| State machine | `state_machine/state_machine.py`, `state_machine/states/*.py`, `state_machine/states/impl/*_impl.py` | The states. Each `*_impl.py` holds one state's `run()`. |
| Drone | `state_machine/drone.py` | `Drone`: the vehicle, the waypoint queue, the planner, and every staged message field (§3.6). |
| Settings | `state_machine/flight_settings.py`, `mission_config.json` | Drone id, role, pairing, field corners, start coordinate. |
| Messages | `state_machine/interdrone.py`, `interdrone_communication/` | Send helpers, the receive loop, message schemas, transport. |
| Planner facade | `flight/pathfinding/mission_planner.py` | `MissionPlanner` / `PathNodeMissionPlanner`: everything in lat/lon; the only planner API the states use. |
| Planner core | `flight/pathfinding/pathnode/planner.py` | `PathNodePlanner`: the chain, the mine rule, coverage, priority groups, shortcuts. Local feet. |
| Claims | `flight/pathfinding/pathnode/claims.py` | `ClaimBook`: which drone of a pair photographs what. |
| Geometry | `flight/pathfinding/pathnode/geometry.py` | `ZoneMap`: known mine zones and the shortest-path graph around them. |
| Coverage grid | `flight/pathfinding/cellField/` | `CellField`: a bit grid of 2 ft squares; "photographed" is one of these. |
| Photo placement | `flight/pathfinding/path_cover.py` | `path_cover_unseen`: photo centres along a polyline's unphotographed stretches. |
| Report format | `flight/pathfinding/iarc_report.py` | `format_iarc_path`: route -> the IARC `S/U/D/L/R` text. |
| Coordinates | `flight/pathfinding/utils/coord_convert.py` | `SimToLatLonTransformer`: lat/lon <-> local feet. |
| Simulator | `flight/pathfinding/pathnode/sim.py`, `render.py`, `minefield.py` | Multi-drone simulation driving the same planner core; GIF renderer. |
| Tests | `flight/pathfinding/pathnode/test_pathnode.py`, `integration_tests/*_sim_test.py` | Planner unit tests; full state-machine runs with mock vehicles. |

---

## 2. The mission process

### 2.1 Roles and pairs

Roles come from the drone count and id (`FlightSettings.__init__`):

| Drones | Drone 1 | Drone 2 | Drone 3 | Drone 4 |
|---|---|---|---|---|
| 1 | SOLOGAMBLER, start edge | | | |
| 2 | GAMBLER, start edge | ASSISTANT of 1 | | |
| 3 | GAMBLER, start edge | ASSISTANT of 1 | SOLOGAMBLER, end edge | |
| 4 | GAMBLER, start edge | ASSISTANT of 1 | GAMBLER, end edge | ASSISTANT of 3 |

- A **pair** is a gambler and its assistant (or a solo gambler alone) working one side.
- With 3–4 drones the two sides work towards each other; the two gamblers are
  **cross-pair partners** (`FlightSettings.cross_pair_partner_id`: 1 <-> 3).
- **Only a gambler owns a planner.** It plans for its whole pair, and hands the assistant
  its share of the work over messages. The assistant flies what it is sent and reports
  back its photos.
- Drone 1 is the reporter: it joins the two sides' routes and pushes the result to the app.

### 2.2 State flow (mapping mission)

```
Start --> Takeoff --> CalcScanPath --> Scan --> DroneShare
                        ^    |           |         |
                        |    |           |         +-- gambler: after every photo --> CalcScanPath
                        |    |           |         +-- assistant: --> Scan (next queued waypoint)
                        |    |           +-- queue empty or replan_needed --> CalcScanPath
                        |    |
                        |    +-- nothing left to check --> ExpandNodes --> EndRun --> AppShare --> Recall --> Land
                        |                                                    |
                        +---------- a relayed mine reopened this side -------+

Flight time budget exceeded (Drone.time_exceeded): CalcScanPath or Scan --> EndRun directly.
```

| State | File | What it does for the search |
|---|---|---|
| Start | `start_impl.py` | Connect, arm, ping the swarm. |
| Takeoff | `takeoff_impl.py` | `configureField`: `drone.planner = make_planner(...)` (gamblers only). Validates the field corners. |
| CalcScanPath | `calc_scan_path_impl.py` | **The planning step.** Applies everything relayed in, asks the planner for the next photos, queues this drone's, sends the assistant its share. Finishes the scan when the planner is done. |
| Scan | `scan_impl.py` | Flies the next queued waypoint, takes the photo, records its real footprint as coverage. |
| DroneShare | `drone_share_impl.py` | Mine detection on that photo. Gambler: applies the mine, relays it, relays coverage and point A. Assistant: reports the photo to its gambler. |
| ExpandNodes | `expand_nodes_impl.py` | Final mine-margin growth. **Currently a no-op** (the expanding-mines phase). |
| EndRun | `end_run_impl.py` | Two-sided mission: joins the two routes. Releases the assistant. Sets the final `mission_path`. |
| AppShare | `app_share_impl.py` | Drone 1 pushes the route to the app in IARC format. |
| Recall | `recall_impl.py` | Leaves the field by the nearest edge (`Drone.recall`, uses the planner's frame). Does nothing on an assistant (no planner). |

The loop runs one photo at a time: **CalcScanPath -> Scan -> DroneShare -> CalcScanPath**.
A gambler replans after *every* photo, because its forward target is the partner's point A,
which keeps moving.

### 2.3 One planning pass (a gambler's CalcScanPath)

`_run_gambler` in `calc_scan_path_impl.py`:

1. `planner.start_if_needed()` the first time (flags a fully blocked field).
2. `drain_relays()`, in a fixed order:
   1. relayed mines from the partner (`pending_cross_pair_mines` -> `planner.add_remote_mine`);
   2. the assistant's photo reports (`pending_photo_reports` -> `planner.record_assistant_photo`),
      relaying any new mines to the partner;
   3. the partner's coverage and latest point A (`pending_partner_coverage`,
      `pending_partner_point_a`);
   4. `planner.advance()` (lay path nodes over what is now checked);
   5. send this side's point A to the partner (on change, and at least every
      `POINT_A_HEARTBEAT_S`).
3. `planner.next_batch(overlap, shape, positions)`: returns this gambler's photos, the
   assistant's photos, and whether the side is done. `positions` is where both drones are
   (`_pair_positions`).
4. If done: `_finish_scan` -> `mission_path = planner.complete(...)`, checksum to the partner,
   -> ExpandNodes.
5. Otherwise: send the assistant its list if it changed (`SEND_SEGMENT_B_WAYPOINTS`), queue
   this drone's own waypoints, -> Scan.

### 2.4 One photo (Scan -> DroneShare)

1. **Scan** pops the next waypoint, flies to it, captures, projects the four image corners
   to the ground, and calls `planner.record_footprint(corners)`. If the waypoint's own
   square still isn't covered, it climbs and retries. (An assistant has no planner: it only
   captures and keeps the corners; it does not climb and retry.)
2. **DroneShare** (gambler) runs mine detection. A find goes to `planner.add_mine(lat, lon)`
   (the mine rule, §3.4), and is relayed to the partner if new. Then `planner.advance()`,
   then coverage and point A to the partner. -> CalcScanPath.
3. **DroneShare** (assistant) sends `SHARE_PHOTOS` (footprint, any mine, a running photo
   number) to its gambler. -> Scan.

### 2.5 Finishing

- **One side (1–2 drones).** When the planner is done, `mission_path` is its route, entry to
  far edge.
- **Two sides (3–4 drones).** Each gambler aims at the other's point A until the gaps close.
  In EndRun the end-side gambler sends its route (`CROSS_PAIR_CHAIN`) until drone 1 acks.
  Drone 1 joins the two routes only if both know exactly the same mines (`mine_hash`); if the
  routes never arrive or don't connect, it reports its own half and sets
  `reported_half_route`. Both keep applying relayed mines while they wait, and go back to
  CalcScanPath if one reopens their side.
- Each gambler then sends its assistant `PAIR_SCAN_DONE`; the assistant finishes.

---

## 3. Data structures and where they live

All planner geometry is in **local feet**: x across the field (0..80), y along it (0 at the
start edge, `field_h` at the end edge). Squares are `CELL` = 2 ft.

### 3.1 The chain (`PathNodePlanner.nodes`, `.links`) -- `pathnode/planner.py`

One gambler's view of its side's route: a list of nodes and the links between them.

```
nodes:  [entry] --link0-- [n1] --link1-- [n2] ... [point A] --forward-- [target]
```

- **Invariant:** `links[i]` joins `nodes[i]` to `nodes[i+1]`; `len(links) == len(nodes) - 1`.
- `PathNode(x, y, placed_by)`: `placed_by` is `"entry"`, `"self"` (laid over the forward
  gap), `"span"` (over a pending span), `"jump"` (a new point A past a mine), `"shortcut"`
  (spliced in by a shortcut) or `"target"`.
- `Link(kind, poly, uid)`: `poly` is the polyline between its two nodes (a detour has
  several points). `kind` is:
  - `"checked"`: every square along it is photographed (segment C);
  - `"pending"`: a planned detour around a mine that still has to be photographed (segment B);
  - `"forward"`: point A -> target, the gap still to explore. At most one.
- **Point A** is the node before the forward link. When there is no forward link, the side
  has met its target.
- **Target** is virtual: the partner's latest point A (3–4 drones) or a far-edge point
  (1–2 drones). Never laid, never deleted by a mine.

### 3.2 Coverage (`PathNodePlanner.seen`) -- `cellField/`

- `seen` is a `CellField`: one bit per 2 ft square, set when a photo **fully** covers the
  square (`record_footprint` -> `fill_polygon_covered`, footprint grown by
  `FOOTPRINT_SLACK_FT` against coordinate round-trip noise).
- `_touched(poly)`: the squares a polyline passes through (traced both directions, cached by
  polyline). A link is checked when all of them are photographed (`corridor` coverage;
  `band` also requires the mine-danger ring around them).
- `advance()` walks each forward/pending link from its start and converts the photographed
  prefix to checked links, laying nodes.

### 3.3 Mines (`PathNodePlanner.zones`, `.discovered`) -- `pathnode/geometry.py`

- `discovered`: every known mine, as (x, y) in local feet.
- `ZoneMap` (`zones`):
  - `groups`: gid -> the merged unsafe area of mines that touch (each mine's zone is the
    hull of the squares within `SAFETY_RADIUS_FT`);
  - `blocker`: all zones, grown slightly -- "may not go here";
  - `vertices` + `_adj`: the visibility graph (zone corners, and which corners see each
    other) used by `shortest_path(p, q)`.
- `mine_hash()`: an order-independent hash of the known mines, used to check two drones agree.

### 3.4 The mine rule (`PathNodePlanner.add_mine`)

When a mine is added (`source` = `"self"`, `"assistant"` or `"remote"`):

0. A mine is ignored if one is already known in the same square. If the entry is now inside
   a zone, the entry shifts sideways along its edge to the nearest clear square.
1. Delete nodes inside the mine's zone; cut checked links that cross it.
2. Replan pending/forward links that cross it.
3. Delete **orphans**: fragments of the chain no longer connected to the entry or the target.
4. Re-bridge the gap with a shortest path.
5. **Jump** if this pair found the mine (`self` or `assistant`) and the frontier was affected --
   either the gap reaches the target, or the forward link was simply replanned: a new point A
   at the mine's far corner, and the detour behind it becomes a pending span. A mine relayed
   from the other pair only re-bridges.

### 3.5 Priority groups and claims -- `planner.py` `groups()`, `claims.py`

`PathNodePlanner.groups()` lists the photos still to take, in priority order (`GROUPS`):

| # | Group | What |
|---|---|---|
| 0 | `path` | The forward gap and every pending span: what it takes to have one valid route. |
| 1 | `big_shortcut` | The big shortcut being tried (§3.7). |
| 2 | `adjacent` | Unphotographed squares within `adjacent_ring` (1) of the checked route, outside mine zones. |
| 3 | `shortcut` | The ordinary shortcut being tried. |
| 4 | `outer` | The next layer out: beyond `adjacent_ring`, within `outer_ring` (2). |

Each group is a list of `(key, sequence)`: the photos along one path in flying order (keyed
by the path: `"forward"`, a pending span's `uid`, a shortcut id), or one free-standing photo
(key `None`).

`ClaimBook.update(groups, positions)` decides who takes what:

- A path is claimed **whole**. Groups are handed out in priority order.
- **Growing radius:** repeatedly, the drone that can *start* an unclaimed sequence soonest
  (its current route length + distance to the start) takes it. A drone's existing claims
  raise its cost, so nobody is sent across the field while another drone is nearer.
- **Only the current job is sticky:** the sequence each drone's next photo belongs to.
  Everything else is re-handed out every update from where the drones are now.
- A drone flies its claims group by group; within a group, paths nearest-start first,
  the forward gap last, free-standing photos nearest first.
- **Deterministic:** ties go by drone order, then coordinates.

`PathNodePlanner.plan(positions)` runs `advance()`, then `ClaimBook.update(groups(),
positions)`, and keeps the result in `last_plan`: `{"self": [...], "assistant": [...]}`.

### 3.6 What the Drone holds -- `state_machine/drone.py`

| Field | Who writes it | Who reads it | What |
|---|---|---|---|
| `planner` | Takeoff | CalcScanPath, Scan, DroneShare, EndRun, ExpandNodes, interdrone (checksum) | `PathNodeMissionPlanner`, or None (assistant). |
| `waypoints` | CalcScanPath (own), interdrone (assistant's batch) | Scan | The flight queue. |
| `last_reached_waypoint`, `last_photo_corners_latlon` | Scan | DroneShare | The photo just taken; its corners are also relayed to the partner as `CROSS_PAIR_COVERAGE`. |
| `pending_cross_pair_mines` | interdrone (`CROSS_PAIR_MINE_RELAY`) | CalcScanPath | Staged relayed mines (`CrossPairMineReport`; only `lat`/`lon` are used). |
| `pending_photo_reports` | interdrone (`SHARE_PHOTOS`) | CalcScanPath | Staged assistant photos. |
| `pending_partner_point_a`, `pending_partner_coverage` | interdrone | CalcScanPath | Staged partner stream. |
| `assistant_position`, `assistant_photos_applied` | CalcScanPath | CalcScanPath | For the claims; for the assistant's skip-flown filter. |
| `photo_seq`, `flown_unapplied` | DroneShare (assistant), interdrone | interdrone | The assistant skips waypoints it already photographed. |
| `last_sent_segment_b`, `last_point_a_sent` | CalcScanPath | CalcScanPath | Dedupe / heartbeat. |
| `partner_route`, `partner_route_complete`, `partner_route_mine_hash`, `route_joined`, `partner_route_acked` | interdrone, EndRun | EndRun, interdrone | The join handshake. |
| `pair_scan_done` | interdrone | CalcScanPath (assistant) | The assistant may finish. |
| `mission_path`, `reported_half_route`, `no_valid_path_found` | CalcScanPath, EndRun | AppShare, interdrone | The reported route and its flags. |
| `replan_needed` | interdrone | Scan, CalcScanPath | Break the flight loop to replan. |

**Rule:** the interdrone receive task never touches the planner (except `mine_hash()`, a pure
read). It only **stages** into these fields; CalcScanPath applies them synchronously.

### 3.7 Shortcuts -- `planner.py` `_update_shortcut`

A shortcut replaces a section of the checked chain with a shorter path around the known mines.

- **Anchors:** the entry, jump nodes, nodes on mine-zone corners, and (once met) the target.
  Never entry -> target (the whole path).
- **Candidates:** for each anchor pair whose chain between them is fully checked, the
  shortest path around the known mines (`_shortest_path`, cached until the next mine).
  - big: shorter than `BIG_SHORTCUT_RATIO` x the section and saving at least `BIG_SHORTCUT_SAVING_FT`;
  - ordinary: shorter than `SHORTCUT_RATIO` x the section, saving at least `SHORTCUT_MIN_SAVING_FT`.
  - **Photo allowance:** the photos it needs (only unphotographed squares count) must be at
    most `max(SHORTCUT_MAX_WAYPOINTS, saving / SHORTCUT_FT_PER_PHOTO)`.
- One shortcut is tried per tier at a time (`shortcuts["big"]`, `shortcuts["ordinary"]`).
  **The chain is untouched until the shortcut is fully photographed**; then it is spliced in.
- **Free shortcuts:** any candidate passing the *ordinary* test whose ground is already all
  photographed is spliced in at once, biggest saving first (`_apply_free_shortcuts`). It is
  credited to a tier only if it was the one that tier was trying; otherwise it counts as an
  ordinary, instant shortcut.
- **Greedy:** after each new mine it is recomputed; a big one that stops qualifying drops to
  ordinary; one that fails the ordinary test or the allowance is given up for good.

### 3.8 Caches -- `planner.py`

The planner is re-asked the same geometric questions after every photo, so it caches the
answers that only change when their inputs do:

| Cache | Key | Invalidated | Saves |
|---|---|---|---|
| `_touched_cache` | a polyline | never (capped at 50k entries) | tracing a path's squares on the grid |
| `_route_cache` (`_shortest_path`) | (from, to) | on every new mine | shortest paths around the mines: shortcut candidates, forward-gap replans, re-bridges |
| `_photos_cache` (`_photos_needed`) | polyline + its unphotographed squares | never (capped) | placing photos to count a shortcut's cost |
| `_zone_cells` (`_in_zone`) | a square | on every new mine | "is this square inside a mine zone?" for the adjacent/outer rings |
| `_candidate_cache` | mines, anchor nodes, section lengths, given-up count | when any of those change | the list of shortcut candidates |

`set_partner_point_a` also returns at once when the partner's point A hasn't changed (the
heartbeat resends it every second). None of this changes what the planner decides.

---

## 4. Interfaces, layer by layer

```
 state machine states (impl/*.py)
        |  lat/lon, via Drone.planner
        v
 PathNodeMissionPlanner            flight/pathfinding/mission_planner.py
        |  converts lat/lon <-> local ft (SimToLatLonTransformer)
        v
 PathNodePlanner                   flight/pathfinding/pathnode/planner.py
   |-- ZoneMap   (mines, shortest paths)      pathnode/geometry.py
   |-- CellField (seen squares)               cellField/
   |-- ClaimBook (who flies what)             pathnode/claims.py
   `-- path_cover_unseen (photo centres)      path_cover.py

 interdrone receive task --stages--> Drone fields --drained by--> CalcScanPath --> planner
```

### 4.1 States -> Drone -> MissionPlanner (lat/lon)

The states only ever call `drone.planner`, a `PathNodeMissionPlanner`
(`flight/pathfinding/mission_planner.py`, built by `make_planner(flight_settings, shape,
overlap)` in Takeoff). Every coordinate crossing this line is `(lat, lon)`.

| Method | Called from | Does |
|---|---|---|
| `start_if_needed()` | CalcScanPath | First call: True/False whether any route exists; later None. |
| `record_footprint(corners)` | Scan | A photo's 4 ground corners as coverage. |
| `cell_seen_at(lat, lon)` | Scan (climb retry) | Is that square photographed? |
| `add_mine(lat, lon)` | DroneShare | A mine this drone found. Returns a relay id if new. |
| `add_remote_mine(lat, lon)` | CalcScanPath | A mine relayed by the partner. |
| `record_assistant_photo(corners, mines)` | CalcScanPath | An assistant photo report. Returns new mines to relay. |
| `record_partner_coverage(corners)`, `set_partner_point_a(lat, lon)` | CalcScanPath | The partner's stream. |
| `advance()` | CalcScanPath, DroneShare | Lay path nodes over what is now checked. |
| `next_batch(overlap, shape, positions)` | CalcScanPath | `(own, assistant, done)`: photo lists for both drones. `overlap` and `shape` are ignored: the footprint is fixed when the planner is built. |
| `point_a_latlon()` | CalcScanPath | This side's frontier, to stream to the partner. |
| `done()`, `failure` | CalcScanPath, EndRun | Finished / cannot finish. |
| `complete(mission_path)` | CalcScanPath | The route when finished. |
| `route_latlon()`, `join_latlon(partner_route)` | EndRun | This side's route; the joined full-field route. |
| `mine_hash()` | CalcScanPath, EndRun, interdrone | Mine agreement. |
| `expand_margin(ft)` | ExpandNodes | No-op for now. |
| `iarc_path(route, buffer_width)` | Drone.get_mission_iarc_path | The IARC text report. |
| `latlon_to_local`, `local_to_latlon`, `field_w`, `field_h` | Drone.recall, harnesses | The local frame. |

The facade also validates the field corners at construction (`validate_field_corners`:
they must go around the field, not in a "Z").

### 4.2 MissionPlanner -> PathNodePlanner (local feet)

`PathNodeMissionPlanner` holds one `PathNodePlanner` (`.planner`) and a transformer, and
converts each call. The core's API:

| Method | Does |
|---|---|
| `record_footprint(corners)` | Coverage. |
| `add_mine(x, y, source)` | The mine rule (§3.4). |
| `set_partner_point_a(x, y)` | Retarget the forward gap. |
| `advance()` | Lay path nodes; update shortcuts. |
| `plan(positions)` | Claims for both drones (§3.5). |
| `groups()` | The photos still to take, by priority. |
| `done()`, `met`, `point_a()`, `route()`, `join(partner_route)` | State and results. |
| `mine_hash()`, `cell_seen_at(x, y)` | Queries. |

### 4.3 PathNodePlanner -> geometry, coverage, claims

- `ZoneMap.add_mine`, `.contains`, `.crosses`, `.shortest_path`: all mine geometry.
- `CellField.fill_polygon_covered`, `.mark_path`, `.get`: coverage.
- `path_cover_unseen(poly, seen, shape, overlap)`: photo centres along a link.
- `ClaimBook.update(groups, positions)`: the claims.

### 4.4 The simulator drives the same core

`pathnode/sim.py`'s `Mission` owns one `PathNodePlanner` per gambler and relays the same
things the flight code does (point A, coverage, mines, assistant photos), with optional loss
and delay. It skips the facade (no lat/lon) and the state machine. **Planner behaviour
changes belong in the core so the simulator and the drones run the same thing.**

---

## 5. Path data: storage, formats and conversions

| Form | Where | Frame | Built by | Used by |
|---|---|---|---|---|
| The chain (nodes + links) | `PathNodePlanner.nodes/.links` | local ft | the planner | the planner |
| Route polyline | `PathNodePlanner.route()` | local ft | entry -> ... -> target, all link points | `route_latlon`, `join` |
| Route lat/lon | `MissionPlanner.route_latlon()` | lat/lon | facade conversion | EndRun, `CROSS_PAIR_CHAIN` |
| Joined route | `MissionPlanner.join_latlon(partner)` | lat/lon | `join_routes` splice at the shared point; verified bridge if needed; rejected if it crosses a mine | EndRun (drone 1) |
| Reported route | `Drone.mission_path` | lat/lon | CalcScanPath `_finish_scan`, replaced by EndRun | AppShare, `push_mission_path` |
| IARC report | `Drone.get_mission_iarc_path()` -> `format_iarc_path` | 2 ft squares | route -> cells -> `S,col,buf` then `U/D/L/R,n` runs | the app (`SEND_PATHS_TO_APP`) |
| Photo waypoints | `Drone.waypoints` (`Waypoint`) | lat/lon | CalcScanPath / interdrone | Scan |

**Coordinate frame.** `SimToLatLonTransformer(corners, 80.0)`: corner 1 is the origin, the
corner-1 -> corner-2 edge is x (scaled to 80 ft), y runs towards corners 3/4. `field_h` is the
largest corner y. Each drone builds the same transformer from the same corners, so local
coordinates agree across drones; everything sent between drones is lat/lon.

---

## 6. Messages between drones

`state_machine/interdrone.py` (send helpers + receive loop); schemas in
`interdrone_communication/message_types.py`. Every message is JSON on the wire (tuples come
back as lists and are restored by the decoder).

| Message (id) | From -> to | Carries | Receiver stages into | Applied by |
|---|---|---|---|---|
| `SEND_SEGMENT_B_WAYPOINTS` (595) | gambler -> its assistant | the assistant's claimed photos, `photos_applied` | `drone.waypoints` (minus photos already flown) | Scan |
| `SHARE_PHOTOS` (575, ack 576) | assistant -> its gambler | footprint corners, mines, `seq` | `pending_photo_reports` | CalcScanPath |
| `PAIR_SCAN_DONE` (608) | gambler -> its assistant | -- | `pair_scan_done` | CalcScanPath (assistant) |
| `CROSS_PAIR_MINE_RELAY` (600, ack 601) | gambler -> partner gambler | a mine (lat/lon) and its id | `pending_cross_pair_mines` | CalcScanPath |
| `CROSS_PAIR_POINT_A` (604) | gambler -> partner | this side's point A (streamed, lossy) | `pending_partner_point_a` (latest wins) | CalcScanPath |
| `CROSS_PAIR_COVERAGE` (605) | gambler -> partner | one photo footprint (lossy) | `pending_partner_coverage` | CalcScanPath |
| `CROSS_PAIR_CHAIN` (606, ack 607) | end-side gambler -> drone 1 | its route, complete flag, `mine_hash` | `partner_route*` | EndRun |
| `FIELD_CHECKSUM` (580, ack 581) | gambler -> partner | `mine_hash` | (answered directly: pure read) | interdrone |

Reliability: `SHARE_PHOTOS`, `CROSS_PAIR_MINE_RELAY` and `PAIR_SCAN_DONE` are requeued by the
transport **only when the socket send itself fails** (`interdrone_communication/client.py`).
The acks are logged but nothing resends on a missing ack, so a message the network accepts
but loses is gone. (The simulator is stronger: it never drops these.) `CROSS_PAIR_CHAIN` is
resent every second until drone 1 acks. Point A and coverage are streams where a loss only
costs a re-photo or a briefly stale target. Ids 602–603 are retired.

---

## 7. Tunables

| Constant | File | Value | Meaning |
|---|---|---|---|
| `CELL` | `pathnode/geometry.py` | 2 ft | Square size. |
| `SAFETY_RADIUS_FT` | `pathnode/geometry.py` | 3 ft | A mine's unsafe radius. |
| `SHAPE_SIZE_FT`, `OVERLAP` | `calc_scan_path_impl.py` | from `vision/config.json`, 0.1 | Planning photo footprint. |
| `adjacent_ring`, `outer_ring` | `PathNodePlanner(...)` | 1, 2 | Reach of the `adjacent` / `outer` groups. |
| `SHORTCUT_RATIO`, `SHORTCUT_MIN_SAVING_FT` | `planner.py` | 0.9, 4 ft | Ordinary shortcut. |
| `BIG_SHORTCUT_RATIO`, `BIG_SHORTCUT_SAVING_FT` | `planner.py` | 0.8, 20 ft | Big shortcut. |
| `SHORTCUT_MAX_WAYPOINTS`, `SHORTCUT_FT_PER_PHOTO` | `planner.py` | 8, 2 ft | Shortcut photo allowance. |
| `TILE_MARGIN_FT` | `planner.py` | 0.5 ft | Tiled photos keep their target this far inside. |
| `MATCH_FT` | `pathnode/claims.py` | 2 ft | A regenerated photo inherits a claim within this. |
| `POINT_A_HEARTBEAT_S`, `IDLE_POLL_S` | `calc_scan_path_impl.py` | 1 s, 0.1 s | Point-A stream; idle polling. |
| `CHAIN_RESEND_S`, `JOIN_GRACE_S` | `end_run_impl.py` | 1 s, 10 s | Join handshake. |
| `MINE_RADIUS_EXPANSION_FT` | `expand_nodes_impl.py` | 2 ft | Final margin (not applied yet). |

---

## 8. Running and testing

- Planner unit tests: `python -m flight.pathfinding.pathnode.test_pathnode`.
- CellField tests: `python flight/pathfinding/tests/cellFieldTest.py`.
- Full state-machine runs (need `dronekit`, i.e. the project venv):
  `python -m integration_tests.multi_drone_sim_test [2|3|4] [dense]` and
  `python -m integration_tests.one_drone_sim_test`.
- Simulator + GIF: `Mission(mines, drones=4, record_frames=True).run()` then
  `render_mission_gif(mission, "out.gif")` (`pathnode/sim.py`, `pathnode/render.py`).
