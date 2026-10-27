# 9. Two sides meeting: point A, the target, and closing the gap

> Part of the [execution walkthrough](README.md). Previous: [8. Shortcuts](08-shortcuts.md). Next: [10. Finishing](10-finishing.md).

**What this chapter covers.** In a 3- or 4-drone mission the two sides start at opposite edges and each aims its **forward gap** at the *other side's* current **point A**. This chapter follows that: what point A is, how it travels between sides, how the target moves, how the gap closes into a single route, and what happens when the two sides meet at slightly different places.

> **Status.** Cross-side traffic and the speaker logic are in final verification; the text describes the code as read.

## 9.1 Recap: the pieces

- **Point A** is this side's frontier: the node before its forward link (`Chain.point_a`). With no forward link it is the target itself.
- **Target** (`target.py`): what the forward gap aims at. With a partner side it is **the partner's latest point A**; at the start (before any relay) it is the partner's *entry*, `(entry_x, far edge)`.
- The **speaker** of each side (lowest id still heard from: drone 1 and drone 3) streams the side's point A to *all* drones of the other side, on change and at least once per second.

## 9.2 The target: `Target.route_from`

`Target` (`target.py`) is abstract (`has_partner`, `route_from`, `may_close`, `retarget`); `make_target` picks `FarEdgeTarget` (no partner side) or `PartnerTarget` (a partner side):

{{snip: flight/pathfinding/pathnode/target.py :: class Target(ABC): => def retarget(self, point): :: class PartnerTarget => return self._far_edge_route(frm)}}

- `route_from(frm)` returns the shortest route (points, obstacle group ids) from a point to the target, or `(None, None)`.
- **With a partner**: first try a route to `partner_a` itself. If `partner_a` is not inside an obstacle this side knows about and a route exists, `may_close()` is `True`: the target is real.
- **Stand-in**: if the partner's point A lies **inside an obstacle this side knows** (for instance a mine this side found and the partner has not heard yet), the route heads for the **nearest obstacle corner** to it instead (up to 8 corners tried), and `may_close()` stays `False`. The partner applies the same rule and re-places *its* point A just outside the obstacle; when the relay arrives, the stand-in is replaced by the real thing.
- **Without a partner** (`FarEdgeTarget`): straight ahead to the far edge if clear, else the far-edge point (one per 2 ft cell) with the shortest route.
- `may_close()` matters in `advance()`:

{{snip: flight/pathfinding/pathnode/planner.py :: def advance(self): => chain.advance_link(chain.links[f], Placer.FORWARD, seen)}}

  "Never close the gap against a stand-in target": a side never declares itself met while the target is a stand-in.

## 9.3 A message's path: partner point A

```mermaid
sequenceDiagram
    participant S1 as Drone 1 (speaker, bottom)
    participant R3 as Drone 3 (receive loop)
    participant C3 as Drone 3 (CalcScanPath)
    S1->>R3: CROSS_PAIR_POINT_A(lat, lon)
    R3->>R3: pending_partner_point_a = (lat, lon)
    Note over C3: next pass
    C3->>C3: apply_inbox: partner_point_a
    C3->>C3: set_partner_point_a: retarget, advance
```

1. Drone 1's `send_heartbeats` (chapter 3, 3.2b) sends `CROSS_PAIR_POINT_A` to drones 3 and 4 when its point A changes (the mine-rule jump changes it immediately) and every second otherwise.
2. The receive loop just stores it, **latest wins** (chapter 7): `pending_partner_point_a`. There is no replan flag: every drone replans after every photo anyway.
3. `apply_inbox` (last step of the inbox) calls `MissionPlanner.partner_point_a` -> `SideSession.partner_point_a` -> `PathNodePlanner.set_partner_point_a`:

{{snip: flight/pathfinding/pathnode/planner.py :: def set_partner_point_a => self.advance()}}

- An **unchanged** point A (the heartbeat) is ignored.
- Otherwise the target moves; the **forward link is re-routed** to it from this side's point A (`route_from`) and the virtual target node replaced. If no route exists the event `stale-target` is logged and the old target is kept.
- `advance()` at the end: the new target may lie on ground that is *already checked*, or exactly on this side's own point A once the partner reached it. Nothing else would advance the chain in that case (no photo is left to take), so the gap would never close.

The **other** stream that lets the gap close is `CROSS_PAIR_COVERAGE`: every footprint the other side photographs is relayed and recorded here (`partner_coverage`), so the ground *between* the two sides counts as photographed no matter who photographed it. Without it each side would have to photograph the other's ground again.

## 9.4 Closing the gap

Each side's forward link runs from its point A to the *other side's* point A. Photos taken anywhere along it (by this side, or relayed from the other) let `advance()` lay path nodes over the checked prefix, so **point A creeps forward**, which the speaker streams to the other side, which re-aims and advances too. The gap is squeezed from both ends:

{{run: demos/toy_meeting.py}}

- **ticks 0 to 6:** both sides' gaps shrink together (they are the same gap, seen from each end). Each jump (ticks 2 to 4: the mine rule moving point A past a mine) moves that side's point A sideways and the other side re-aims.
- **tick 6:** 0.5 ft apart, each side's target is the other's point A.
- **tick 7:** both forward links are gone (`gap closed`): `chain.forward_index()` is `None`, so `PathNodePlanner.met` is true on both sides:

{{snip: flight/pathfinding/pathnode/planner.py :: def met(self): => return self.failure is None :: def point_a(self): => return self.chain.point_a()}}

  Note that at tick 7 each side's point A is the *other's previous* point A: the two sides crossed. This is normal (each closes on a point A that the other has since moved beyond) and is what `join` handles below.

`met` does not mean *done*: rings, pending spans and shortcuts may remain (`done()` requires all of them to be finished; chapter 10). The meeting changes two things immediately: `Shortcuts.anchors` now includes the target, and `done()` can become true.

## 9.5 Joining the two routes

Each side's route runs from its own entry to where it stopped. Because each side stops at the other's point A, which is on the other's route, the two share at least one point. `join.py`:

{{snip: flight/pathfinding/pathnode/join.py :: def join_routes => return None :: def join(own, partner, routes, coverage, log): => return None}}

- `join_routes`: find the first point along **own** that matches (within `ROUTE_MATCH_FT` = 0.01 ft) a vertex of **partner** or lies on a partner segment; the result is `own[:i+1]` followed by the partner's points before that spot, **reversed** (so the joined route reads bottom entry to top entry).
- `join` adds a fallback when relay loss left the two halves overlapping but sharing no point: bridge the closest pair of points **within 30 ft** (`MAX_BRIDGE_FT`), only with a straight line that crosses no obstacle *and* whose squares are all photographed, "so the bridge is as verified as any checked link".
- `MissionPlanner.join_latlon` also rejects a joined route that `engine.crosses`: it must not touch any obstacle *this* copy knows.

The last line of the toy output shows the joined route from bottom entry (12, 0) to top entry (12, 80).

## 9.6 When the halves don't connect: reopening

`PathNodePlanner.check_join` is what drone 1's `EndRun` calls (chapter 10):

{{snip: flight/pathfinding/pathnode/planner.py :: def check_join => self.advance() +1}}

- Under relay loss, both sides can meet **crosswise**: each closes on a point A that the other has since abandoned, and neither route contains the other's end.
- **The bottom side yields**: `_reopen_toward(partner_route[-1])` sets the target to the partner route's end (which is on the partner's route by definition), turns the met target into an ordinary FORWARD-placed node and opens a **new forward gap** to it; `advance()` then continues normally. `met` goes false and `done()` goes false, so `EndRun` returns to `CalcScanPath` to fly the gap.
- **The top side only waits**, so the two never chase each other.
- The `reopens` counter and the events `reopen` / `reopen-failed` are logged.

## 9.7 Mines across sides

- **A mine found by drone 1 or 2**: `own_photo` adds it as `Source.PAIR` (the finder's point A may jump) and sends it to drones 3 and 4 as `CROSS_PAIR_MINE_RELAY`; there it is added as `Source.PARTNER` (only re-bridges; it never jumps *their* point A).
- **`CROSS_PAIR_MINE_SYNC`** (speaker, every ~5 s; every 1 s while waiting in `EndRun`): the full mine list, so a relay lost in transit cannot keep the two mine sets apart. The receiver dedupes by square.
- **Agreement check**: when a side finishes, `_finish_scan` sends `FIELD_CHECKSUM` (`mine_hash`) to the other side; a mismatch is logged (`FIELD_CHECKSUM_ACK` with `matched = False`).

## 9.8 Keeping apart across sides

Drones of different sides share the same middle of the field at the end. `SideAgent.other_side_positions()` gives fresh `POSITION_REPORT`s of the other side's drones, and their announced jobs come from `SIDE_JOB` (`peer_jobs`). `PathNodePlanner.plan(..., others=...)` runs `_keep_apart` with the **same rule as within a side** (lower id keeps its ground; every drone of side 1 is lower than any of side 2). Work is **not** shared across sides (each side's planner is its own): the other side only steers where a photo may be flown (`keep_away_ft` = 10 ft from a lower-id drone's position or announced job, `keep_clear_ft` = 7.5 ft from a higher-id one; `_make_way` also yields to a lower-id drone's announced job within `keep_away_ft`, including the other side's). See chapter 3 (step 4).
