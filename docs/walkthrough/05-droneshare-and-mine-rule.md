# 5. DroneShare: finding a mine, and the mine rule

> Part of the [execution walkthrough](README.md). Previous: [4. Scan](04-scan.md). Next: [6. CalcScanPath after a mine](06-calcscanpath-after-mine.md).

**What this chapter covers.** After `Scan` photographs a spot, `DroneShare` does "the actual check": it looks for a mine in that spot, tells the planner, and tells everyone else. When the planner learns of a mine it runs **the mine rule** (`pathnode/mine_rule.py`), which repairs the route around the new obstacle. This is the most intricate logic in the mission.

> **Status.** The per-drone code is in final verification. The obstacle geometry under `routes.py` is a `PathEngine` (`ZoneMap` by default, chapter 2); the mine rule only asks `Routes` the questions listed there (`contains`, `crosses`, `route`, `corners`, `add_mine`, `add_square`).

## 5.1 DroneShare.run

```python
# state_machine/states/impl/drone_share_impl.py:118-133
        waypoint = self.drone.last_reached_waypoint
        if waypoint is not None:
            corners = self.drone.last_photo_corners_latlon
            mine_latlon = await _check_photo_latlon(self.drone) if corners is not None else None
            mines = [mine_latlon] if mine_latlon is not None else []
            if mines:
                flight_log.event("mine_found", waypoint=flight_log.waypoint_brief(waypoint))
            if corners is None and not mines:
                # No footprint at all: a miss for the squares it was aimed at, here and in
                # the side-mate's copy, so an untakeable photo isn't handed out forever.
                await self.drone.side.failed_photo(waypoint.lat, waypoint.long)
            else:
                # Applies the photo (mines first, then path nodes), relays new mines and the
                # footprint to the other side, and sends the photo to the side-mate.
                await self.drone.side.own_photo(corners, mines)
        return CalcScanPath(self.drone, self.flight_settings, self.interdrone)
```

Three outcomes for the photo just taken at `last_reached_waypoint`:

1. **A footprint and maybe a mine.** `_check_photo_latlon` runs mine detection (5.1a) and returns at most **one** mine. `side.own_photo(corners, mines)` applies it (5.2).
2. **No footprint and no mine.** The photo failed (capture error, ray never hit the ground). `side.failed_photo(lat, lon)` records a *miss* for the squares this spot was aimed at (chapter 4), here **and** in the side-mate's copy, so an untakeable photo is not handed out forever.
3. Whatever happens, `DroneShare` always returns `CalcScanPath`. (The `Scan` docstring talks about `Scan -> DroneShare -> Scan`; that describes an older loop. In the current code every photo goes back through `CalcScanPath`.)

### 5.1a Mine detection

```python
# state_machine/states/impl/drone_share_impl.py:54-58, 83-88
    detections = camera.capture_and_detect_mines()
    if not detections:
        return None
    best = max(detections, key=lambda d: d.score)
    px, py, _w, _h = best.box
    ...
    ground = camera.get_pixel_coordinate(px, py, image_width, image_height, drone_pose)
    if ground is None:
        logging.warning("drone %d: mine detection's pixel ray never reached the ground", drone.id)
        return None
    mine_lat, mine_lon = ground
    return mine_lat, mine_lon
```

- A **fresh** frame is captured (the IMX500 camera's on-chip detector is tied to its own capture); the drone has not moved since `Scan`'s photo. Detection is single-shot: the voting machinery in the vision config is bypassed.
- `best = max(detections, key=score)`: only the highest-confidence detection is used. A photo with two mines in view reports **one** of them; the other gets found when a later photo lands on it.
- The pixel `(px, py)` is ray-cast to the ground with the same `DronePose` idea as in chapter 4, giving the mine's **lat/lon**.

**Goes next to ->** `SideAgent.own_photo`.

## 5.2 side.own_photo: apply, relay, share

```python
# flight/pathfinding/side_agent.py:225-240
    async def own_photo(self, corners, mines):
        """This drone's photo is done (its footprint already recorded; corners is the last
        one, or None if it had none) with these mines in it: apply it, relay new mines and
        the footprint to the other side, and send the photo to the mate."""
        for lat, lon, relay_id in self.planner.own_photo(corners, mines):
            if self.partners:
                await self.interdrone.send_cross_pair_mine_relay(lat, lon, relay_id, self.partners)
        if corners is not None and self.partners:
            await self.interdrone.send_cross_pair_coverage(corners, self.partners)
        if self.mate is not None and not self.mate_lost:
            if corners is not None:
                photo = {"cornerCoordinates": corners, "mines": list(mines)}
            else:
                photo = {"cornerCoordinates": [], "mines": []}
            await self.interdrone.share_photos([photo], (self.mate,))
        await self.send_heartbeats()
```

The three jobs, in order:

1. **`planner.own_photo(corners, mines)`** applies the photo to this drone's copy of the planner and returns the *new* mines as `[(lat, lon, relay_id)]`:

```python
# flight/pathfinding/mission_planner.py:19-21, 96-100
def relay_id(lat, lon):
    """A short id for a mine relay, from its position (echoed back in the ack)."""
    return hashlib.sha256(f"{lat:.7f},{lon:.7f}".encode("utf-8")).hexdigest()[:16]
    ...
    def own_photo(self, corners_latlon, mines_latlon):
        """This drone's photo is done, with these mines found in it. Returns the mines to relay
        to the partner, as [(lat, lon, relay_id)] -- only ones new to this side."""
        out = self.session.own_photo(None, self._local(mines_latlon))
        return self._relays(out.mines)
```

   `MissionPlanner.own_photo` -> `SideSession.own_photo`:

```python
# flight/pathfinding/pathnode/session.py:48-49, 55-60
    def _add_pair_mines(self, mines):
        return [m for m in mines if self.planner.add_mine(*m, Source.PAIR)]
    ...
    def own_photo(self, corners, mines):
        """This drone's photo is in (own_footprint already recorded its footprint; corners is
        the last one, or None if there was none) and these mines were found in it."""
        new = self._add_pair_mines(mines)
        self.planner.advance()          # path nodes only after the photo's mines are in
        return self._outbox(corners if self.has_partner else None, new)
```

   - The footprint was **already** recorded by `Scan` (`own_footprint`); this call only adds the mines.
   - `_add_pair_mines` calls `planner.add_mine(x, y, Source.PAIR)` for each and keeps the ones that were **new** (a second mine in the same 2 ft square is the same mine).
   - `planner.advance()` runs **after** the mines are in: "path nodes only after the photo's mines are in", so nothing is laid over ground that has just turned out unsafe.
   - `Outbox` (a small record) carries the new mines, this photo's footprint and this side's point A.
2. **Relay to the other side** (only if `partners` is non-empty): each new mine as `CROSS_PAIR_MINE_RELAY` (with the position-derived `relay_id`, which the receiver echoes in its ack), and this photo's footprint as `CROSS_PAIR_COVERAGE`. Both go to **all** of the other side's drones. `CROSS_PAIR_MINE_RELAY` is one of the two message types resent if the socket fails ("a drone that never hears of a mine plans through it"). The receive side is chapter 7.
3. **Share with the side-mate** as `SHARE_PHOTOS`: `{"cornerCoordinates": corners, "mines": mines}`. The mate's copy applies it with `mate_photo` (chapter 7), so both copies see the same coverage and mines. The mate does **not** relay the mines to the other side again: the finder already did.
4. **`send_heartbeats()`** again: point A may just have moved (see the jump below), and the speaker sends the new point A at once instead of waiting for the next pass.

**Goes next to ->** `PathNodePlanner.add_mine`.

## 5.3 add_mine: a new obstacle

```python
# flight/pathfinding/pathnode/planner.py:210-222
    def add_mine(self, x, y, source):
        """Apply the mine rule (mine_rule.py). source: Source.PAIR (found by this side's
        drones) or Source.PARTNER (relayed). Returns whether the mine was new -- a mine in a
        square that already has one is the same mine."""
        source = Source(source)
        if self.failure:
            return False
        cell = snap_to_cell_center(x, y)
        if any(snap_to_cell_center(*m) == cell for m in self.discovered):
            return False
        self.discovered.append((float(x), float(y)))
        mine_rule.apply(self, self.routes.add_mine(x, y), source)
        return True
```

- **Deduplication.** The mine is snapped to the centre of its 2 ft square; a mine in a square that already has one is ignored (returns `False`, so nothing is relayed).
- `discovered` keeps the exact position for the list sent in `CROSS_PAIR_MINE_SYNC` and for `mine_hash()` (an order-independent hash of the snapped squares, used to check two drones agree, chapter 10).
- `routes.add_mine(x, y)` builds the mine's **danger zone**: every 2 ft square within 3 ft of the mine's square (whole squares only), merged with any overlapping zones into one **group** with an id `gid`. It also drops cached shortest paths the zone crosses and bumps `routes.version` (so the cached work lists and shortcut candidates are recomputed):

```python
# flight/pathfinding/pathnode/routes.py:36-50
    def _added(self, gid):
        self.version += 1
        new = self.engine.last_added
        shapely.prepare(new)
        keys = [k for k, v in self._paths.items() if v is not None]
        if keys:
            lines = np.array([self._paths[k][2] for k in keys], dtype=object)
            for k, hit in zip(keys, shapely.intersects(lines, new)):
                if hit:
                    del self._paths[k]
        x0, y0, x1, y1 = new.bounds
        for (c, r), hit in list(self._inside.items()):
            if not hit and x0 <= (c + 0.5) * CELL <= x1 and y0 <= (r + 0.5) * CELL <= y1:
                del self._inside[(c, r)]
        return gid
```

- `mine_rule.apply(self, gid, source)`: the repair.

`source` is `Source.PAIR` when a drone of **this side** found the mine (own photo or the mate's), and `Source.PARTNER` when it was **relayed by the other side** (or the "mine" is a blocked square, chapter 4). The difference matters only for the last rule (jump).

## 5.4 The mine rule

```python
# flight/pathfinding/pathnode/mine_rule.py:1-21
"""The mine rule: repairing the chain when a new obstacle (a mine's zone, or a blocked square)
appears.

  0. If the entry is now inside it, the entry shifts along its edge to the nearest clear spot.
  1. Path nodes inside it are deleted (never the target); checked links crossing it are cut.
  2. Pending and forward links crossing it are replanned between their endpoints.
  3. Every fragment no longer connected to a root -- the entry or the target -- is an orphan
     and is deleted, whether or not it is inside the obstacle.
  4. The two root fragments are re-bridged by a shortest path: a pending span, or the forward
     gap if the far side is just the target. If no route leaves the entry's fragment at all
     (walled in), the entry moves along its edge to the nearest spot that has one.
  5. Jump: when the frontier was affected and this side's own drones found the mine
     (Source.PAIR), point A jumps to the mine's far corner and the detour behind it becomes a
     pending span. Mines relayed by the partner side, and blocked squares, only re-bridge.

Works on a PathNodePlanner (planner.py): its chain, routes, target and log.

Entry point: apply(pl, gid, source). It runs a MineRule, whose methods are the steps above in
order (_clear_entry, _delete_nodes_inside, _repair_links, _split_into_fragments, then either
_jump_past_mine or _rebridge). The per-link cases are _cut_checked, _replan_forward and
_replan_pending.
```

The entry point is `mine_rule.apply(pl, gid, source)`, which runs one `MineRule` object. Each step below is a method of that class, called in order by `MineRule.run`; the three link cases are one method each.

In plain words: *delete what the obstacle sits on, re-plan what it crosses, throw away whatever got cut off, join the two live ends with a shortest path, and, if this side found the mine right at the frontier, move the frontier past it.*

Definitions:
- **Root**: the entry node, or the target. A fragment of the chain is only useful if it connects to a root.
- **Fragment**: a run of consecutive nodes that are still linked to each other after deleting nodes and cutting links.
- **Orphan**: a fragment connected to neither root.
- **Bridge**: a new PENDING link joining the two root fragments.

### Rule 0 and 1: move the entry, delete nodes inside, cut checked links

```python
# flight/pathfinding/pathnode/mine_rule.py:56-66, 69-84
    def run(self):
        """The steps in order (see the module docstring)."""
        if not self._clear_entry():
            return
        self._delete_nodes_inside()
        self._repair_links()
        f1, f2, frags = self._split_into_fragments()
        if f1 is f2:
            self._jump_past_mine()
        else:
            self._rebridge(f1, f2, frags)
    ...
    def _clear_entry(self):
        """Shift the entry out of the obstacle if it is inside. False when it cannot be moved."""
        if self.routes.contains(*self.chain.nodes[0].xy):
            if not _shift_entry(self.pl):
                return False
            self.broken.add(0)
        return True

    # -- step 1 ----------------------------------------------------------------------------
    def _delete_nodes_inside(self):
        """Mark every path node inside the obstacle dead (never the target)."""
        chain = self.chain
        last = len(chain.nodes) - 1
        self.dead = {id(n) for i, n in enumerate(chain.nodes)
                     if i != last and self.routes.contains(n.x, n.y)}
        self.log.count("nodes_in_mine", len(self.dead))
```

- `_clear_entry`: if the **entry** is inside the new obstacle, `_shift_entry` moves it along the entry edge to the nearest clear 2 ft square (`entry_shifts`). If none is clear, `pl.failure = "entry fully blocked"` and planning stops.
- `_delete_nodes_inside`: `dead` = every path node whose position is inside an obstacle now, **except the last** (the target is virtual and is never deleted).
- `broken` = the indices of links that are cut (see rule 2).

### Rule 2: re-plan what the obstacle crosses

```python
# flight/pathfinding/pathnode/mine_rule.py:87-101, 103-116, 118-125
    def _repair_links(self):
        """Visit every surviving link that crosses the obstacle and repair it by kind."""
        chain = self.chain
        for i, link in enumerate(chain.links):
            a, b = chain.nodes[i], chain.nodes[i + 1]
            if i in self.broken or id(a) in self.dead or id(b) in self.dead:
                continue
            if not self.routes.crosses(link.poly):
                continue
            if link.is_checked:
                self._cut_checked(i)
            elif link.is_forward:
                self._replan_forward(i, link, a)
            else:
                self._replan_pending(i, link, a, b)
    ...
    def _cut_checked(self, i):
        """A photographed link crossing the obstacle is cut."""
        self.broken.add(i)

    def _replan_forward(self, i, link, a):
        """Replan the forward link from its start to the target; the target moves to the route's end."""
        route = self.pl.target.route_from(a.xy)
        if route[0] is None:
            self.broken.add(i)
            return
        link.poly = route[0]
        self.chain.nodes[i + 1] = PathNode(*route[0][-1], Placer.TARGET)
        self.fwd_route = route
        self.log.count("replans")
    ...
    def _replan_pending(self, i, link, a, b):
        """Replan a pending span between its two ends."""
        route = self.routes.route(a.xy, b.xy)
        if route[0] is None:
            self.broken.add(i)
            return
        link.poly = route[0]
        self.log.count("replans")
```

`_repair_links` visits every link that **crosses** the obstacle and does not touch a dead node (links at a dead node are skipped; the fragment logic below handles them):
- `_cut_checked`: a **CHECKED** link is **cut** (`broken`): the ground was verified before the mine appeared and can no longer be trusted; it is not re-planned in place.
- `_replan_pending`: a **PENDING** link is re-planned as the shortest path between its two nodes (`routes.route`).
- `_replan_forward`: the **FORWARD** link is re-planned by `target.route_from(a.xy)` (shortest path to the target from point A) and the target node is refreshed. This route is remembered in `fwd_route` for the jump rule.

### Rule 3: find the fragments and the orphans

```python
# flight/pathfinding/pathnode/mine_rule.py:128-152
    def _split_into_fragments(self):
        """Split the node indices at dead nodes and cut links.

        Returns: (f1, f2, frags): the fragment holding the entry, the one holding the target
        (the same list when the chain is still connected), and all fragments. The rest are
        orphans.
        """
        chain = self.chain
        frags, cur = [], []
        for i, n in enumerate(chain.nodes):
            if id(n) in self.dead:
                if cur:
                    frags.append(cur)
                    cur = []
                continue
            if cur and (i - 1) in self.broken:
                frags.append(cur)
                cur = []
            cur.append(i)
        if cur:
            frags.append(cur)
        last = len(chain.nodes) - 1
        f1 = next((f for f in frags if f[0] == 0), None)
        f2 = next((f for f in frags if f[-1] == last), None)
        return f1, f2, frags
```

`_split_into_fragments` walks the nodes in order; a dead node, or a broken link, ends the current fragment. `f1` is the fragment that contains the entry (index 0), `f2` the one that contains the target. Every other fragment is an **orphan**: it is dropped, **whether or not it lies inside the obstacle** (it is disconnected from both roots, so no route can use it).

### Rule 4: bridge the two roots (or stay put if they are still joined)

`MineRule.run` branches on the fragments:

```python
# flight/pathfinding/pathnode/mine_rule.py:56-66
    def run(self):
        """The steps in order (see the module docstring)."""
        if not self._clear_entry():
            return
        self._delete_nodes_inside()
        self._repair_links()
        f1, f2, frags = self._split_into_fragments()
        if f1 is f2:
            self._jump_past_mine()
        else:
            self._rebridge(f1, f2, frags)
```

If `f1 is f2`, nothing was cut off; the chain is still one piece. The only thing left to decide is the jump (rule 5, `_jump_past_mine`).

If they differ, the chain was cut in two and `_rebridge` joins it:

```python
# flight/pathfinding/pathnode/mine_rule.py:166-201
    def _rebridge(self, f1, f2, frags):
        """Delete orphans and join the entry's fragment to the target's by a shortest path."""
        pl, chain, routes, log = self.pl, self.chain, self.routes, self.log
        last = len(chain.nodes) - 1
        log.count("orphans", sum(len(f) for f in frags if f is not f1 and f is not f2))
        u = chain.nodes[f1[-1]]
        # If nothing but the target survives on the far side, the gap reaches the target: that
        # is the frontier -- the forward link was lost, or (after meeting) the checked join into
        # the target was cut, which re-opens the gap.
        frontier = f2 == [last]
        if frontier:
            route_from = pl.target.route_from
            where = "the target"
        else:
            v = chain.nodes[f2[0]]
            route_from = lambda p: routes.route(p, v.xy)       # noqa: E731
            where = f"({v.x:.1f},{v.y:.1f})"
        route = route_from(u.xy)
        if route[0] is None:
            # Walled in: start again from wherever along the entry edge has a way through.
            route = _reroute_from_entry(pl, route_from)
            if route[0] is None:
                pl.failure = f"no route from ({u.x:.1f},{u.y:.1f}) to {where}"
                return
            log.count("orphans", len(f1) - 1)
            f1 = [0]
        if frontier:
            mid_nodes, mid_links = _frontier_links(pl, route, self.source, self.gid)
            tail_nodes, tail_links = [PathNode(*route[0][-1], Placer.TARGET)], []
        else:
            mid_nodes, mid_links = [], [Link(LinkKind.PENDING, route[0])]
            tail_nodes = [chain.nodes[i] for i in f2]
            tail_links = [chain.links[i] for i in f2[:-1]]
            log.count("bridges")
        chain.nodes = [chain.nodes[i] for i in f1] + mid_nodes + tail_nodes
        chain.links = [chain.links[i] for i in f1[:-1]] + mid_links + tail_links
```

- `u` is the last node of the entry's fragment. If everything on the far side is gone except the target (`frontier`), the new link is **the forward gap** from `u` to the target. Otherwise it is a **PENDING** link from `u` to the first node `v` of the target's fragment (a `bridge`).
- If there is **no route at all** from `u`, the entry's fragment is walled in. `_reroute_from_entry` tries other squares along the entry edge, nearest first, for one that has a route (the whole entry fragment is then discarded as orphaned). If none does, `pl.failure` is set to "no route from ... to ...": planning has failed for this side.
- The chain's `nodes` and `links` are then reassembled from `f1`, the new link(s), and the target's fragment.

### Rule 5: the jump (point A moves past the mine)

```python
# flight/pathfinding/pathnode/mine_rule.py:155-163, 204-216
    def _jump_past_mine(self):
        """Still one piece: only if the forward link was replanned around a mine this side
        found does point A jump past it."""
        chain = self.chain
        if self.fwd_route is not None and self.source == Source.PAIR:
            f = chain.forward_index()
            nodes, links = _frontier_links(self.pl, self.fwd_route, self.source, self.gid)
            chain.splice(f, nodes, links)
            chain.set_target(chain.links[-1].poly[-1])
    ...
def _frontier_links(pl, route, source, gid):
    """The new frontier from route (points, gids). On a mine this side found, jump past it:
    point A moves to the last corner of that mine's group the route bends around."""
    pts, gids = route
    if source == Source.PAIR:
        ks = [k for k in range(1, len(pts) - 1) if gids[k] == gid]
        if ks:
            k = ks[-1]
            pl.log.count("jumps")
            pl.log.event("jump", pts[k])
            return ([PathNode(*pts[k], Placer.JUMP)],
                    [Link(LinkKind.PENDING, pts[:k + 1]), Link(LinkKind.FORWARD, pts[k:])])
    return [], [Link(LinkKind.FORWARD, pts)]
```

When a drone of **this side** found a mine that affects the **frontier** (the forward link had to be re-planned, or the bridge reaches the target), point A does not stay before the mine and wait for the detour to be photographed. It **jumps** to the **last corner of that mine's obstacle group** that the new route bends around (`gids[k] == gid`):

- a **JUMP** node is created at that corner;
- everything from the old point A to it becomes a **PENDING** link (the detour, still to be photographed);
- the FORWARD link now starts at the jump node and heads to the target.

Consequence: the drones continue exploring *ahead* of the mine while the detour behind it is checked in parallel. Mines relayed by the partner side (`Source.PARTNER`) and blocked squares take the `return [], [Link(FORWARD, pts)]` path: they only re-bridge, because the partner side is moving *its* point A. If both sides jumped, they would chase each other.

## 5.5 Three cases from a toy run

The script below creates a 20 x 60 ft field with one drone and applies the rule three ways. (These are real outputs of `PathNodePlanner`.)

```text
=== A. a mine found IN FRONT of point A by this side (Source.PAIR): point A jumps past it
before   : entry(10.0,0.0) -checked- forward(10.0,5.5) -checked- forward(10.0,13.5) -forward- target(10.0,60.0)
      stats: {}  failure=None
      mine at (10.5, 22.0); the forward gap runs straight through it
after    : entry(10.0,0.0) -checked- forward(10.0,5.5) -checked- forward(10.0,13.5) -pending- jump(5.9,22.0) -forward- target(5.0,60.0)
      stats: {'jumps': 1, 'replans': 1}  failure=None
      obstacle corners the routes bend around: [(10.0, 17.9), (5.9, 22.0), (5.9, 24.0), (10.0, 28.1), (12.0, 28.1), (16.1, 24.0)] ...

=== B. a mine relayed by the OTHER side (Source.PARTNER) lands on ground already CHECKED
before   : entry(10.0,0.0) -checked- forward(10.0,5.5) -checked- forward(10.0,13.5) -checked- forward(10.0,21.5) -checked- forward(10.0,25.5) -checked- forward(10.0,33.5) -checked- forward(10.0,41.5) -forward- target(10.0,60.0)
      stats: {}  failure=None
      route is checked up to y = 41.5 ; mine at (10.0, 8.0) on the checked stretch
after    : entry(10.0,0.0) -pending- forward(10.0,21.5) -checked- forward(10.0,25.5) -checked- forward(10.0,33.5) -checked- forward(10.0,41.5) -forward- target(10.0,60.0)
      stats: {'nodes_in_mine': 2, 'bridges': 1}  failure=None

=== C. a mine on the ENTRY square
before   : entry(10.0,0.0) -forward- target(10.0,60.0)
      stats: {}  failure=None
after    : entry(5.0,0.0) -forward- target(5.0,60.0)
      stats: {'entry_shifts': 1}  failure=None
```

- **A: mine in front of point A, `Source.PAIR`.** Before the mine the chain is entry, checked to y = 5.5, checked to y = 13.5, then the forward gap. The forward link at x = 10 crosses the new zone. Rule 2 re-plans it (`replans: 1`); the frontier was affected and the finder is this side, so rule 5 jumps: a **JUMP** node appears at (5.9, 22.0) (the zone's last corner the route bends around), the detour, a straight pending link (10, 13.5) -> (5.9, 22.0), is the **pending** link, and the forward link now runs from the jump node to the target (which moved to (5.0, 60)).
- **B: mine on ground already checked, `Source.PARTNER`.** The zone covers the nodes at y = 5.5 and 13.5 (`nodes_in_mine: 2`) and cuts the checked links around them. Rule 3 leaves two fragments, the entry and the chain from y = 21.5 on; rule 4 bridges them with a **pending** link around the zone (`bridges: 1`). No jump: the route beyond point A was untouched.
- **C: mine on the entry square.** Rule 0: the entry moves along the bottom edge to x = 5.0 (`entry_shifts: 1`); the forward link is re-drawn from there.

**Goes next to ->** [Chapter 6](06-calcscanpath-after-mine.md): the next `CalcScanPath` pass, with the chain changed.
