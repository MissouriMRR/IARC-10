# 5. DroneShare: finding a mine, and the mine rule

> Part of the [execution walkthrough](README.md). Previous: [4. Scan](04-scan.md). Next: [6. CalcScanPath after a mine](06-calcscanpath-after-mine.md).

**What this chapter covers.** After `Scan` photographs a spot, `DroneShare` does "the actual check": it looks for a mine in that spot, tells the planner, and tells everyone else. When the planner learns of a mine it runs **the mine rule** (`pathnode/mine_rule.py`), which repairs the route around the new obstacle. This is the most intricate logic in the mission.

> **Status.** The per-drone code is in final verification. The obstacle geometry under `routes.py` is still `ZoneMap` and is due to be replaced (chapter 2); the mine rule only asks `Routes` the questions listed there (`contains`, `crosses`, `route`, `corners`, `add_mine`, `add_square`).

## 5.1 DroneShare.run

{{snip: state_machine/states/impl/drone_share_impl.py :: waypoint = self.drone.last_reached_waypoint => return CalcScanPath(}}

Three outcomes for the photo just taken at `last_reached_waypoint`:

1. **A footprint and maybe a mine.** `_check_photo_latlon` runs mine detection (5.1a) and returns at most **one** mine. `side.own_photo(corners, mines)` applies it (5.2).
2. **No footprint and no mine.** The photo failed (capture error, ray never hit the ground). `side.failed_photo(lat, lon)` records a *miss* for the squares this spot was aimed at (chapter 4), here **and** in the side-mate's copy, so an untakeable photo is not handed out forever.
3. Whatever happens, `DroneShare` always returns `CalcScanPath`. (The `Scan` docstring talks about `Scan -> DroneShare -> Scan`; that describes an older loop. In the current code every photo goes back through `CalcScanPath`.)

### 5.1a Mine detection

{{snip: state_machine/states/impl/drone_share_impl.py :: detections = camera.capture_and_detect_mines() => px, py, _w, _h = best.box :: ground = camera.get_pixel_coordinate( => return mine_lat, mine_lon}}

- A **fresh** frame is captured (the IMX500 camera's on-chip detector is tied to its own capture); the drone has not moved since `Scan`'s photo. Detection is single-shot: the voting machinery in the vision config is bypassed.
- `best = max(detections, key=score)`: only the highest-confidence detection is used. A photo with two mines in view reports **one** of them; the other gets found when a later photo lands on it.
- The pixel `(px, py)` is ray-cast to the ground with the same `DronePose` idea as in chapter 4, giving the mine's **lat/lon**.

**Goes next to ->** `SideAgent.own_photo`.

## 5.2 side.own_photo: apply, relay, share

{{snip: flight/pathfinding/side_agent.py :: async def own_photo => await self.send_heartbeats()}}

The three jobs, in order:

1. **`planner.own_photo(corners, mines)`** applies the photo to this drone's copy of the planner and returns the *new* mines as `[(lat, lon, relay_id)]`:

{{snip: flight/pathfinding/mission_planner.py :: def relay_id => [:16] :: def own_photo => return self._relays(out.mines)}}

   `MissionPlanner.own_photo` -> `SideSession.own_photo`:

{{snip: flight/pathfinding/pathnode/session.py :: def _add_pair_mines => return [m for m :: def own_photo(self, corners, mines): => return self._outbox(}}

   - The footprint was **already** recorded by `Scan` (`own_footprint`); this call only adds the mines.
   - `_add_pair_mines` calls `planner.add_mine(x, y, Source.PAIR)` for each and keeps the ones that were **new** (a second mine in the same 2 ft square is the same mine).
   - `planner.advance()` runs **after** the mines are in: "path nodes only after the photo's mines are in", so nothing is laid over ground that has just turned out unsafe.
   - `Outbox` (a small record) carries the new mines, this photo's footprint and this side's point A.
2. **Relay to the other side** (only if `partners` is non-empty): each new mine as `CROSS_PAIR_MINE_RELAY` (with the position-derived `relay_id`, which the receiver echoes in its ack), and this photo's footprint as `CROSS_PAIR_COVERAGE`. Both go to **all** of the other side's drones. `CROSS_PAIR_MINE_RELAY` is one of the two message types resent if the socket fails ("a drone that never hears of a mine plans through it"). The receive side is chapter 7.
3. **Share with the side-mate** as `SHARE_PHOTOS`: `{"cornerCoordinates": corners, "mines": mines}`. The mate's copy applies it with `mate_photo` (chapter 7), so both copies see the same coverage and mines. The mate does **not** relay the mines to the other side again: the finder already did.
4. **`send_heartbeats()`** again: point A may just have moved (see the jump below), and the speaker sends the new point A at once instead of waiting for the next pass.

**Goes next to ->** `PathNodePlanner.add_mine`.

## 5.3 add_mine: a new obstacle

{{snip: flight/pathfinding/pathnode/planner.py :: def add_mine => return True}}

- **Deduplication.** The mine is snapped to the centre of its 2 ft square; a mine in a square that already has one is ignored (returns `False`, so nothing is relayed).
- `discovered` keeps the exact position for the list sent in `CROSS_PAIR_MINE_SYNC` and for `mine_hash()` (an order-independent hash of the snapped squares, used to check two drones agree, chapter 10).
- `routes.add_mine(x, y)` builds the mine's **danger zone**: every 2 ft square within 3 ft of the mine's square (whole squares only), merged with any overlapping zones into one **group** with an id `gid`. It also drops cached shortest paths the zone crosses and bumps `routes.version` (so the cached work lists and shortcut candidates are recomputed):

{{snip: flight/pathfinding/pathnode/routes.py :: def _added => return gid}}

- `mine_rule.apply(self, gid, source)`: the repair.

`source` is `Source.PAIR` when a drone of **this side** found the mine (own photo or the mate's), and `Source.PARTNER` when it was **relayed by the other side** (or the "mine" is a blocked square, chapter 4). The difference matters only for the last rule (jump).

## 5.4 The mine rule

{{snip: flight/pathfinding/pathnode/mine_rule.py :: The mine rule: repairing => only re-bridge.}}

In plain words: *delete what the obstacle sits on, re-plan what it crosses, throw away whatever got cut off, join the two live ends with a shortest path, and, if this side found the mine right at the frontier, move the frontier past it.*

Definitions:
- **Root**: the entry node, or the target. A fragment of the chain is only useful if it connects to a root.
- **Fragment**: a run of consecutive nodes that are still linked to each other after deleting nodes and cutting links.
- **Orphan**: a fragment connected to neither root.
- **Bridge**: a new PENDING link joining the two root fragments.

### Rule 0 and 1: move the entry, delete nodes inside, cut checked links

{{snip: flight/pathfinding/pathnode/mine_rule.py :: def apply(pl, gid, source): => log.count("nodes_in_mine", len(dead))}}

- If the **entry** is inside the new obstacle, `_shift_entry` moves it along the entry edge to the nearest clear 2 ft square (`entry_shifts`). If none is clear, `pl.failure = "entry fully blocked"` and planning stops.
- `dead` = every path node whose position is inside an obstacle now, **except the last** (the target is virtual and is never deleted).
- `broken` = the indices of links that are cut (see rule 2).

### Rule 2: re-plan what the obstacle crosses

{{snip: flight/pathfinding/pathnode/mine_rule.py :: fwd_route = None => log.count("replans")}}

For every link that **crosses** the obstacle and does not touch a dead node (links at a dead node are skipped; the fragment logic below handles them):
- a **CHECKED** link is **cut** (`broken`): the ground was verified before the mine appeared and can no longer be trusted; it is not re-planned in place.
- a **PENDING** link is re-planned as the shortest path between its two nodes (`routes.route`).
- the **FORWARD** link is re-planned by `target.route_from(a.xy)` (shortest path to the target from point A) and the target node is refreshed. This route is remembered in `fwd_route` for the jump rule.

### Rule 3: find the fragments and the orphans

{{snip: flight/pathfinding/pathnode/mine_rule.py :: frags, cur = [], [] => f2 = next(}}

Walk the nodes in order; a dead node, or a broken link, ends the current fragment. `f1` is the fragment that contains the entry (index 0), `f2` the one that contains the target. Every other fragment is an **orphan**: it is dropped, **whether or not it lies inside the obstacle** (it is disconnected from both roots, so no route can use it).

### Rule 4: bridge the two roots (or stay put if they are still joined)

{{snip: flight/pathfinding/pathnode/mine_rule.py :: if f1 is f2: => return}}

If `f1 is f2`, nothing was cut off; the chain is still one piece. The only thing left to decide is the jump (rule 5).

If they differ, the chain was cut in two:

{{snip: flight/pathfinding/pathnode/mine_rule.py :: log.count("orphans", sum(len(f) => chain.links = [chain.links[i] for i in f1[:-1]] + mid_links + tail_links}}

- `u` is the last node of the entry's fragment. If everything on the far side is gone except the target (`frontier`), the new link is **the forward gap** from `u` to the target. Otherwise it is a **PENDING** link from `u` to the first node `v` of the target's fragment (a `bridge`).
- If there is **no route at all** from `u`, the entry's fragment is walled in. `_reroute_from_entry` tries other squares along the entry edge, nearest first, for one that has a route (the whole entry fragment is then discarded as orphaned). If none does, `pl.failure` is set to "no route from ... to ...": planning has failed for this side.
- The chain's `nodes` and `links` are then reassembled from `f1`, the new link(s), and the target's fragment.

### Rule 5: the jump (point A moves past the mine)

{{snip: flight/pathfinding/pathnode/mine_rule.py :: def _frontier_links => return [], [Link(LinkKind.FORWARD, pts)]}}

When a drone of **this side** found a mine that affects the **frontier** (the forward link had to be re-planned, or the bridge reaches the target), point A does not stay before the mine and wait for the detour to be photographed. It **jumps** to the **last corner of that mine's obstacle group** that the new route bends around (`gids[k] == gid`):

- a **JUMP** node is created at that corner;
- everything from the old point A to it becomes a **PENDING** link (the detour, still to be photographed);
- the FORWARD link now starts at the jump node and heads to the target.

Consequence: the drones continue exploring *ahead* of the mine while the detour behind it is checked in parallel. Mines relayed by the partner side (`Source.PARTNER`) and blocked squares take the `return [], [Link(FORWARD, pts)]` path: they only re-bridge, because the partner side is moving *its* point A. If both sides jumped, they would chase each other.

## 5.5 Three cases from a toy run

The script below creates a 20 x 60 ft field with one drone and applies the rule three ways. (These are real outputs of `PathNodePlanner`.)

{{run: demos/toy_mine_rule.py}}

- **A: mine in front of point A, `Source.PAIR`.** Before the mine the chain is entry, checked to y = 5.5, checked to y = 13.5, then the forward gap. The forward link at x = 10 crosses the new zone. Rule 2 re-plans it (`replans: 1`); the frontier was affected and the finder is this side, so rule 5 jumps: a **JUMP** node appears at (5.9, 22.0) (the zone's last corner the route bends around), the detour, a straight pending link (10, 13.5) -> (5.9, 22.0), is the **pending** link, and the forward link now runs from the jump node to the target (which moved to (5.0, 60)).
- **B: mine on ground already checked, `Source.PARTNER`.** The zone covers the nodes at y = 5.5 and 13.5 (`nodes_in_mine: 2`) and cuts the checked links around them. Rule 3 leaves two fragments, the entry and the chain from y = 21.5 on; rule 4 bridges them with a **pending** link around the zone (`bridges: 1`). No jump: the route beyond point A was untouched.
- **C: mine on the entry square.** Rule 0: the entry moves along the bottom edge to x = 5.0 (`entry_shifts: 1`); the forward link is re-drawn from there.

**Goes next to ->** [Chapter 6](06-calcscanpath-after-mine.md): the next `CalcScanPath` pass, with the chain changed.
