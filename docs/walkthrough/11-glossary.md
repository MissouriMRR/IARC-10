# 11. Glossary

> Part of the [execution walkthrough](README.md). Previous: [10. Finishing](10-finishing.md). Next: [12. Notes and oddities](12-notes-and-oddities.md).

Terms are grouped by topic. "Ch." points to the chapter that explains it in context.

## The mission and the drones

| Term | Meaning |
|---|---|
| **Mission (mapping mission)** | Find a safe path through a field with hidden mines and report it. Drones fly over the field at 2 m taking photos; a mine found in a photo is an obstacle the route must avoid. |
| **Field** | The rectangle (four lat/lon corners) being mapped. About 82 x 300 ft for the golf-field config. |
| **Local frame** | The planner's own x/y coordinates in feet: corner 1 is the origin, corner 1 to 2 is +x, +y toward corners 3/4. Every drone builds the same frame (Ch. 2). |
| **Cell / square** | A 2 ft x 2 ft grid square (`CELL`), `(column, row)` from the field's bottom-left. Coverage and obstacles are counted in squares. |
| **Side** | The one or two drones that work from one field edge. `FlightSettings.side_drones`. 1 to 2 drones: one side; 3 to 4: two sides (Ch. 1). |
| **Side-mate / paired drone** | The other drone in your side (`FlightSettings.paired_drone`), or none. |
| **Partner side / other side** | The opposite side (`partner_side_drones`). |
| **Speaker** | The lowest-id drone of a side still heard from. It speaks for the side to the other side: point A and the full mine list (Ch. 2, 3, 9). |
| **Reporter** | Drone 1: holds the app link and reports the final (joined) route (Ch. 10). |
| **Role** | There are none. Drones of a side are equals; each keeps its own planner copy. |
| **Planner copy** | Each drone's own `PathNodePlanner` for its side (`Drone.planner`). Copies see the same events but may apply them at different moments. |

## The state machine

| Term | Meaning |
|---|---|
| **State** | One phase of the mission (`Start`, `Takeoff`, `CalcScanPath`, ...). A state's `run()` returns the next state (Ch. 1). |
| **`Drone`** | The per-drone object: flight controller connection plus all mission data shared between states (queue, planner, staged messages). |
| **`Interdrone`** | Sends messages and runs the receive loop; also owns the `cmd_msg` mailbox and per-peer `DroneState`s. |
| **`cmd_msg` / `CMD_MSG`** | Mailbox for commands (`ARM`, `MISSION`, `LAND`...) written by the receive loop, read by states. |
| **`DroneState`** | What this drone knows about a *peer*: last position report, last heard time, waypoints. |
| **Staging** | The receive loop only stores incoming data on the `Drone` (`pending_*`); `CalcScanPath` applies it (Ch. 7). |
| **`replan_needed`** | A flag set by the receive loop for some messages. Read by `Scan` at entry; effectively redundant today (Ch. 7). |
| **Waypoint** | A lat/lon photo spot in the drone's queue (`Drone.waypoints`). Rebuilt from scratch on every `CalcScanPath` pass. |
| **Leg** | One flight from where the drone is to the next waypoint (`gotoWaypoint`). |
| **Hold** | The drone waits in the air because a peer is within `COLLISION_RADIUS` of the leg (Ch. 4). |
| **`FormationLost`** | Raised when a hold lasts 15 s or measured separation falls below 1.2 m; `Scan` lands. |
| **Stale / lost** | A peer with no message (or position report) for too long is ignored (stale, 12 s; position 2 s) and a side-mate silent for 20 s is *lost* (its work goes to the others). |
| **POIF** | The other mission ("demo"): drones fly formation circles. Not covered here. |

## The route (chain)

| Term | Meaning |
|---|---|
| **Chain** | The route so far: path nodes joined by links (`chain.py`, Ch. 2). |
| **Path node** | A corner point of the route, tagged with who placed it (`ENTRY`, `FORWARD`, `SPAN`, `JUMP`, `SHORTCUT`, `TARGET`). |
| **Link** | The stretch between two nodes: **CHECKED** (photographed), **PENDING** (planned detour, not yet photographed) or **FORWARD** (the one gap still to explore). |
| **Entry** | The first node, on this side's edge (bottom `y = 0`, top `y = field_h`). |
| **Point A** | The frontier: the node before the forward link. With no forward link, the target itself. |
| **Target** | The virtual last node the forward gap aims at: the partner's point A, or a far-edge point. |
| **Stand-in target** | A corner of an obstacle used while the partner's real point A lies inside an obstacle this side knows (`Target.may_close() = False`; Ch. 9). |
| **Jump node / jump** | New point A created past a mine this side found (Ch. 5). |
| **Span** | A pending link (a detour behind point A); keyed by its far end. |
| **Met** | The forward gap is closed: the chain reaches the target (`PathNodePlanner.met`). |
| **Done** | Met, nothing pending, rings photographed, no shortcut left (`done()`). |
| **Route verified** | Met and every link checked (`route_verified()`). |
| **Join** | Splicing the two sides' routes into one (`join.py`, Ch. 9, 10). |
| **Reopen** | Turning a met target back into a forward gap because the two halves did not connect (`check_join`; Ch. 9). |
| **`mission_path`** | The route (lat/lon) the drone reports. |
| **IARC path** | The text format `S,col,buffer / U,n / L,n ...` pushed to the app (Ch. 10). |

## Obstacles and the mine rule

| Term | Meaning |
|---|---|
| **Mine** | A detected mine, snapped to its square's centre. |
| **Danger zone / obstacle** | Every whole square within 3 ft of a mine's square; overlapping zones merge into a **group** with an id. |
| **Blocked square** | A square given up after 3 misses; treated as a tiny mine zone (Ch. 4). |
| **`Routes`** | The planner's interface to obstacles: `add_mine`, `contains`, `crosses`, `shortest`, `corners`. It wraps a swappable `PathEngine` (`engines/`, Ch. 2). |
| **Shortest path** | Shortest route around every known obstacle, bending only at zone corners. |
| **Mine rule** | `mine_rule.apply`: repair the chain when an obstacle appears (Ch. 5). |
| **Fragment / orphan / bridge** | Pieces of the chain left after the cut; orphans connect to neither root and are deleted; the bridge is the new pending link joining the roots (Ch. 5). |
| **Source `PAIR` / `PARTNER`** | Who found the mine: this side's drones (may cause a jump) or relayed by the other side (only re-bridges). |
| **Mine hash** | Order-independent hash of the known mine squares, for checking two copies agree (Ch. 10). |

## Photos and coverage

| Term | Meaning |
|---|---|
| **Footprint** | The four ground corners of what a photo covered, ray-cast from the camera geometry (Ch. 4). |
| **Planning footprint (`SHAPE_SIZE_FT`)** | The fixed rectangle assumed when placing photos: (8.52, 6.44) ft at 2 m (Ch. 2). |
| **Seen** | A square fully inside some recorded footprint. |
| **Corridor / band coverage** | What a link needs photographed: squares it touches, or also every square a mine could endanger it from. Default: corridor. |
| **Miss** | A needed square a photo was aimed at but did not cover. 1 miss: centre a photo on it; 3: block it. |
| **Ring (adjacent / outer)** | Extra photos 1 (then 2) squares around the checked route, for safety margin. |
| **Tile** | A free-standing photo placed to cover scattered squares. |

## Work, claims and keep-apart

| Term | Meaning |
|---|---|
| **Group** | Priority class of photos: PATH (0), BIG_SHORTCUT (1), ADJACENT (2), SHORTCUT (3), OUTER (4). |
| **Sequence / key** | The photos along one path in flying order; the key names the path (`"forward"`, `("span", end)`, `("shortcut", a, b)`, or `None` for a free-standing photo). |
| **Claim** | The decision that one drone takes a whole sequence. Computed by each drone's `ClaimBook` (Ch. 3). |
| **Job / head** | The `(group, key, (x, y))` of a drone's next photo. Announced to others as `SIDE_JOB`. |
| **Held job** | A job another drone announced; it overrides this copy's own record of that drone (Ch. 3, 7). |
| **Idle takeover** | A drone with nothing to fly takes its mate's queued work except the announced next photo (Ch. 3). |
| **Keep-apart** | Dropping photos too close to another drone: `keep_away_ft` 10 ft, `keep_clear_ft` 7.5 ft (both above the flight code's 6.6 ft hold radius). Lower id keeps its ground. Includes the other side's drones (Ch. 3, 9). |
| **Make way** | Sending a blocked drone to a spare photo out of the way (`SideCoordinator._make_way`); the higher id yields to a lower id's announced job within `keep_away_ft`. |

## Shortcuts

| Term | Meaning |
|---|---|
| **Anchor** | Entry, jump nodes, nodes on obstacle corners, and the target once met (Ch. 8). |
| **Section** | The chain between two anchors; must be fully checked. |
| **Ordinary / big shortcut** | Path shorter than 0.9 (or 0.8) of the section, saving at least 4 ft (or 20 ft). |
| **Trial** | A shortcut being tried; one per tier. Its photos are a work group. |
| **Allowance** | Most photos a shortcut may need: `max(8, saving / 2 ft)`. |
| **Splice** | Replacing the section with the shortcut once fully photographed (`Chain.replace`). |
