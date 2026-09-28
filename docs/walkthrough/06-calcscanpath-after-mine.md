# 6. Back in CalcScanPath: the same functions, a different chain

> Part of the [execution walkthrough](README.md). Previous: [5. DroneShare and the mine rule](05-droneshare-and-mine-rule.md). Next: [7. Messages from other drones](07-messages-from-other-drones.md).

**What this chapter covers.** `DroneShare` returned `CalcScanPath` (chapter 5). Nothing in the code path has changed, but the *inputs* have: the chain now has a checked stretch, then (after a mine) a jump node and a pending span. This chapter revisits the functions of [chapter 3](03-first-calcscanpath.md) and shows what they return now, with real output from the planner.

> **Status.** The claims and keep-apart behaviour shown in the toy run is the code in final verification (see chapter 3).

The toy run below follows drone 1 on a 20 x 60 ft field (one side of two drones, drone 2 idle at (19.5, 0), announced no job, no partner side). At each "pass" it prints the chain, the photo groups, the claims and each drone's queue, exactly what `CalcScanPath` -> `side.plan()` -> `PathNodePlanner.plan` computes.

```text
--- pass 1: start
chain : entry(10.0,0.0) -forward- target(10.0,60.0)
group 0 path         key=forward                      11 photo(s), first (10.9, 2.7)
drone 1 queue: [(10.9, 2.7, 'path'), (11.0, 8.5, 'path'), (11.0, 14.3, 'path'), (11.0, 20.1, 'path'), (11.0, 25.9, 'path')]
drone 2 queue: []
claims (paths only): {('path', 'forward'): 1}
drone 1 head (announced): ('path', 'forward')
--- pass 2: after a photo at (10.9, 2.7), no mine
chain : entry(10.0,0.0) -checked- forward(10.0,5.5) -forward- target(10.0,60.0)
group 0 path         key=forward                      9 photo(s), first (11.0, 9.9)
group 2 adjacent     key=None                         1 photo(s), first (9.0, 8.3)
group 2 adjacent     key=None                         1 photo(s), first (13.0, 8.3)
group 4 outer        key=None                         1 photo(s), first (7.0, 2.3)
group 4 outer        key=None                         1 photo(s), first (15.0, 2.3)
group 4 outer        key=None                         1 photo(s), first (7.0, 8.3)
group 4 outer        key=None                         1 photo(s), first (15.0, 8.3)
group 4 outer        key=None                         1 photo(s), first (11.0, 10.3)
drone 1 queue: [(11.0, 9.9, 'path'), (11.0, 15.7, 'path'), (11.0, 21.5, 'path'), (11.0, 27.3, 'path'), (11.0, 33.1, 'path')]
drone 2 queue: []
claims (paths only): {('path', 'forward'): 1}
drone 1 head (announced): ('path', 'forward')

*** photo at (11.0, 9.9) contains a mine at (12.0, 10.9); new mines for the other side: 1

--- pass 3: right after the mine
chain : entry(10.0,0.0) -checked- forward(10.0,5.5) -checked- span(8.1,9.5) -pending- jump(7.9,10.0) -forward- target(7.0,60.0)
group 0 path         key=('span', (7.9, 9.95857864376269)) 1 photo(s), first (7.9, 10.0)
group 0 path         key=forward                      10 photo(s), first (7.0, 9.0)
group 2 adjacent     key=None                         1 photo(s), first (7.0, 4.3)
group 2 adjacent     key=None                         1 photo(s), first (7.0, 10.3)
group 4 outer        key=None                         1 photo(s), first (5.0, 2.3)
group 4 outer        key=None                         1 photo(s), first (15.0, 2.3)
group 4 outer        key=None                         1 photo(s), first (5.0, 8.3)
group 4 outer        key=None                         1 photo(s), first (5.0, 14.3)
drone 1 queue: [(7.0, 9.0, 'path'), (7.0, 12.5, 'path'), (7.0, 18.3, 'path'), (7.0, 24.1, 'path'), (7.0, 29.9, 'path')]
drone 2 queue: []
claims (paths only): {('path', 'forward'): 1, ('path', 'span'): 2}
drone 1 head (announced): ('path', 'forward')

*** drone 1 flies to (7.0, 9.0) [path], photo has no mine

--- pass 4: after the span's photo
chain : entry(10.0,0.0) -checked- forward(10.0,5.5) -checked- span(8.1,9.5) -checked- jump(7.9,10.0) -checked- forward(7.9,11.9) -forward- target(7.0,60.0)
group 0 path         key=forward                      8 photo(s), first (7.0, 15.9)
group 2 adjacent     key=None                         1 photo(s), first (7.0, 4.3)
group 2 adjacent     key=None                         1 photo(s), first (5.0, 14.3)
group 4 outer        key=None                         1 photo(s), first (5.0, 2.3)
group 4 outer        key=None                         1 photo(s), first (15.0, 2.3)
group 4 outer        key=None                         1 photo(s), first (3.0, 8.3)
group 4 outer        key=None                         1 photo(s), first (3.0, 14.3)
group 4 outer        key=None                         1 photo(s), first (7.0, 16.3)
drone 1 queue: [(7.0, 15.9, 'path'), (7.0, 21.7, 'path'), (7.0, 27.5, 'path'), (7.0, 33.3, 'path'), (7.0, 39.1, 'path')]
drone 2 queue: [(15.0, 2.3, 'outer')]
claims (paths only): {('path', 'forward'): 1}
drone 1 head (announced): ('path', 'forward')
```

## 6.1 Pass 1 (start): what chapter 3 produced

`entry -forward- target`; one PATH group (`forward`, 11 photos); drone 1 claims all of it. Drone 1's head is `('path', 'forward')`.

## 6.2 Pass 2: after a photo with no mine

`DroneShare` -> `own_photo(corners, [])` -> `planner.advance()` (no mines to add). `advance` walks the forward link and lays a path node where the photographed prefix ends:

```text
before:  entry ---------------forward---------------> target(10, 60)
after:   entry ==checked== (10.0, 5.5) --forward-----> target(10, 60)
```

What each function returns now:

- **`groups()`**: the forward sequence is shorter (9 photos, starting at (11.0, 9.9)), and two new kinds of work appear now that there is a **checked route**:
  - **ADJACENT** (group 2): unphotographed squares within 1 square of the checked route, as free-standing single-photo entries (`key=None`).
  - **OUTER** (group 4): the next layer out, up to 2 squares from the route.
  Both come from `Work.band`: "route cells" (every square the CHECKED links touch) grown by `n` squares (Chebyshev distance), minus what is already seen or inside an obstacle, then tiled into photos by `Coverage.tile`.
- **`ClaimBook.update`**: drone 1 keeps the `forward` job (`_inherit`: its head key still exists). The loose ring photos are then dealt by the same "who can reach it soonest, counting what they already hold" rule. Drone 1 already holds a 50 ft path, so its route-so-far ends at the far end of the field: **every ring photo is dealt to drone 2**, which is idle and near. But keep-apart (`keep_away_ft` = 10 ft) then drops them from drone 2's queue, because each is within 10 ft of drone 1's position or next photo. So drone 2's queue stays **empty** in passes 2 and 3 of the toy run, and the first ring photo reaches it in pass 4, once drone 1 has moved on.
- `claims (paths only)` shows only path claims; the ring photos are claimed too (per photo, by `("loose", point)`).

## 6.3 Pass 3: right after a mine (the interesting one)

The photo at (11.0, 9.9) contained a mine at (12.0, 10.9). `DroneShare` -> `own_photo` -> `add_mine(..., Source.PAIR)` -> mine rule (chapter 5, case A) -> `advance()`:

```text
before:  entry ==ck== (10,5.5) --------------forward-------------> target(10,60)
after:   entry ==ck== (10,5.5) ==ck== span(8.1,9.5) ..pending.. jump(7.9,10.0) --forward--> target(7.0,60)
```

- The forward gap was re-planned around the mine's zone and the frontier **jumped** to (7.9, 10.0). The detour from the old point A became a **pending** link. `advance()` (run after the mine, per chapter 5) had already photographed most of it and laid a `span` path node at (8.1, 9.5); only a last sliver is pending.
- **`groups()` now has two PATH sequences**: `("span", (7.9, 9.96))` with one photo, and `forward` with 10 photos starting at (7.0, 9.0). The span's key is its **far end node**, which does not move as its start is checked, so it identifies the same span on every drone.
- **Claims**: drone 1's job is still `forward` (kept by `_inherit`), so its cost for anything else already includes flying the whole forward path. The **span is dealt to drone 2**. This is the claim rule working as documented: "a redirection behind [point A] tends to go to the other drone".
- **Keep-apart then removes it.** Drone 2 is the higher id; the span photo (7.9, 10.0) is within 10 ft (`keep_away_ft`) of drone 1's position (11.0, 9.9). It is dropped from drone 2's queue for this pass, and drone 2 gets nothing to fly. (The claim stays; only the queue was filtered.)
- **Result**: drone 1's queue starts at (7.0, 9.0), the first forward photo, exactly where it is heading.

The pass also sends messages: `send_heartbeats()` (called at the end of `own_photo` and again in `plan`) publishes the **new point A**, so the other side retargets at once, and `SIDE_JOB` if the job changed. Chapter 9 follows point A to the other side.

## 6.4 Pass 4: after the next photo, no mine

Drone 1 flew to (7.0, 9.0). That photo happened to cover the pending span too, so `advance()` converted it:

```text
entry ==ck== (10,5.5) ==ck== span(8.1,9.5) ==ck== jump(7.9,10.0) ==ck== (7.9,11.9) --forward--> target(7.0,60)
```

- The `span` group is gone: **nobody had to fly to it**. Chain nodes are laid over *any* photographed ground, regardless of which drone photographed it or why.
- The JUMP node stays in the chain (`placed_by = JUMP`) because it is a **shortcut anchor** (chapter 8): the entry and every jump node are anchors, and once the section between two anchors is fully checked, `shortcuts.update` (called from `advance`) asks whether a shorter path exists. Here the section entry -> jump is nearly straight, so there is no candidate.

## 6.5 What is *not* different

- `CalcScanPath.run` itself is identical each pass: clear the flag, `side.plan()`, check `failure`/`done`, `resetWaypoints`, `Scan`.
- The queue is rebuilt from scratch (new `Waypoint` objects, new random ids) every pass.
- A mine found by *this* drone is handled in `DroneShare` (immediately, before the next `CalcScanPath`). A mine found by someone else arrives through `apply_inbox` at the top of `side.plan()`: [chapter 7](07-messages-from-other-drones.md).
