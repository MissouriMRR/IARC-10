# 8. Shortcuts: replacing checked stretches with shorter ones

> Part of the [execution walkthrough](README.md). Previous: [7. Messages from other drones](07-messages-from-other-drones.md). Next: [9. Two sides meeting](09-two-sides-meeting.md).

**What this chapter covers.** A route built by the mine rule is often longer than it needs to be: when a mine is found, the route detours around it, and later discoveries can make an earlier detour unnecessary or leave a zig-zag. `pathnode/shortcuts.py` looks for **checked sections of the chain that a shorter path could replace**, tests them by photographing the shorter path, and splices it in.

> **Status.** The path lengths come from `Routes.shortest`, whose geometry (`ZoneMap`) is due to be replaced (chapter 2); the interface stays the same.

## 8.1 The idea, from the module docstring

```python
# flight/pathfinding/pathnode/shortcuts.py:1-19
"""Shortcuts: replacing a checked section of the chain with a shorter path around the known
obstacles.

- Anchors: the entry, every jump node (earlier point As), every path node on an obstacle
  corner (where every detour bends) and, once met, the target (the final connection node).
- For a pair of anchors -- never the entry and the target together, which would be the whole
  path -- whose chain between them is fully checked: the shortest path between them. It is
  worth trying if it is
    - big: shorter than big_shortcut_ratio x the section and saving >= big_shortcut_saving_ft;
    - ordinary: shorter than shortcut_ratio x the section and saving >= SHORTCUT_MIN_SAVING_FT;
  and the photos it needs (only unphotographed squares count) are within its allowance:
  shortcut_max_waypoints, or one per shortcut_ft_per_photo it saves if that is more.
- One that needs no photos at all is spliced in at once, biggest saving first.
- Otherwise one of each tier is tried at a time (Trial), the biggest saving in its tier. The
  chain is untouched while it is tried; the section is replaced only once all of the
  shortcut is photographed and clear of every known obstacle.
- Greedy: after every change the trial is recomputed. A big one that no longer qualifies as big
  drops back to the ordinary tier; once it no longer qualifies as ordinary, or needs more
  photos than its allowance, that pair is given up for good.
```

Definitions:
- **Anchor**: a chain node a shortcut may start or end at: the **entry**, every **jump** node (an earlier point A), every path node sitting on an **obstacle corner** (where every detour bends), and, once the forward gap is closed (`met`), the **target**.
- **Section**: the chain between two anchors. It is only a candidate if it is fully **checked** (`chain.section(i, j)` is `None` otherwise).
- **Ordinary** vs **big** shortcut (numbers from `PlannerConfig`):

| | ratio | minimum saving | group of its photos |
|---|---|---|---|
| ordinary | new path < **0.9** x section | `SHORTCUT_MIN_SAVING_FT` = 4 ft (2 cells) | SHORTCUT (3) |
| big | new path < **0.8** x section | **20 ft** | BIG_SHORTCUT (1) |

- **Photo allowance**: the shortcut may need at most `max(8, saving / 2 ft)` photos. Only squares not yet photographed count.
- **Trial**: a shortcut being tried. One per tier at a time. The chain is **untouched** while it is tried; the section is replaced only when the whole shortcut is photographed and clear of every known obstacle.

## 8.2 Where it runs

`PathNodePlanner.advance()` (chapter 3) ends with `self.shortcuts.update(self.met)`. `advance()` runs after every photo, every mate photo, every relayed mine and every partner point A, so shortcuts are re-evaluated all the time. `PathNodePlanner.done()` also asks `shortcuts.left_to_try(met)`: the side is not done while a shortcut could still be tried.

## 8.3 Candidates

```python
# flight/pathfinding/pathnode/shortcuts.py:58-65, 67-83, 85-88
    def anchors(self, met):
        """Indices of the anchor nodes (see the module doc)."""
        nodes = self.chain.nodes
        last = len(nodes) - 1
        corners = {(round(x, 6), round(y, 6)) for x, y in self.routes.corners()}
        return [i for i, n in enumerate(nodes)
                if n.placed_by in (Placer.ENTRY, Placer.JUMP) or (i == last and met)
                or (round(n.x, 6), round(n.y, 6)) in corners]
    ...
    def route(self, a, b, section, tier):
        """The shortest path a -> b if it is short enough to be a shortcut of this tier.
        Doesn't check the photo allowance."""
        ratio = self.config.tier_ratio(tier)
        if ratio is None:
            return None
        if math.dist(a.xy, b.xy) <= ROUTE_MATCH_FT:
            poly = [a.xy, b.xy]          # the section is a loop back to where it started
        else:
            found = self.routes.shortest(a.xy, b.xy)
            if found is None:
                return None
            poly = [a.xy] + list(found[0][1:-1]) + [b.xy]
        length = poly_len(poly)
        if length >= ratio * section or section - length < self.config.tier_min_saving(tier):
            return None
        return poly
    ...
    def allowance(self, section, poly):
        """How many photos a shortcut may take (see the module doc)."""
        saving = section - poly_len(poly)
        return max(self.config.shortcut_max_waypoints, saving / self.config.shortcut_ft_per_photo)
```

- `anchors`: indices of nodes that are the entry, a JUMP node, the last node when `met`, or that lie **exactly on an obstacle corner** (`routes.corners()`, rounded to 6 decimals). Path nodes laid by the mine rule's bridge/re-plan sit on corners because shortest paths only bend around corners.
- `route(a, b, section, tier)`: ask `Routes.shortest(a, b)` for the shortest path around all known obstacles. If it is not shorter than `ratio x section`, or the saving is below the tier's minimum, it is **not** a shortcut. (If `a` and `b` are the same point, the section is a loop back to where it started and the shortcut is the trivial `[a, b]`.)

```python
# flight/pathfinding/pathnode/shortcuts.py:90-124
    def candidates(self, tier, met):
        """Every anchor pair short enough to be a shortcut of this tier, biggest saving first,
        as [(saving, a, b, poly, section)]. Cached until the chain or the obstacles change."""
        ratio = self.config.tier_ratio(tier)
        if not self.enabled or not ratio:
            return []
        chain = self.chain
        anchors = self.anchors(met)
        sections = tuple(chain.section(i, j) for i, j in zip(anchors, anchors[1:]))
        # The anchor nodes themselves, not id(node): the key keeps them alive, so a deleted
        # node's id can't be reused by a new one and make a stale entry match.
        key = (self.routes.version, tuple(chain.nodes[i] for i in anchors),
               tuple(None if x is None else round(x, 6) for x in sections), len(self.given_up))
        cached = self._cache.get(tier)
        if cached is not None and cached[0] == key:
            return cached[1]
        found = []
        last = len(chain.nodes) - 1
        for x, i in enumerate(anchors):
            for j in anchors[x + 1:]:
                if i == 0 and j == last:
                    continue            # never the whole path
                a, b = chain.nodes[i], chain.nodes[j]
                section = chain.section(i, j)
                if section is None or (a, b) in self.given_up:
                    continue
                # No path is shorter than the straight line: skip pairs that can't qualify.
                if math.dist(a.xy, b.xy) >= ratio * section:
                    continue
                poly = self.route(a, b, section, tier)
                if poly is not None:
                    found.append((section - poly_len(poly), a, b, poly, section))
        found.sort(key=lambda c: -c[0])
        self._cache[tier] = (key, found)
        return found
```

- Every pair of anchors is considered **except entry + target together** (that would be the whole path).
- A cheap filter first: no path is shorter than the straight line, so `dist(a, b) >= ratio x section` cannot qualify.
- Results are sorted **biggest saving first** and cached on (obstacle version, anchor nodes, section lengths, number of given-up pairs). The cache is dropped as soon as the chain or obstacles change.

`best(tier, met)` picks the first candidate whose `photos_needed(poly)` is within its allowance and which the other tier is not already trying.

## 8.4 Free shortcuts, trials, splicing

```python
# flight/pathfinding/pathnode/shortcuts.py:140-146, 148-165, 167-204, 206-210
    def _splice(self, a, b, poly, section, tier):
        chain = self.chain
        chain.replace(chain.index(a), chain.index(b), poly, Placer.SHORTCUT)
        self.log.count("shortcuts")
        if tier == Tier.BIG:
            self.log.count("big_shortcuts")
        self.log.event("shortcut", tier.value, a.xy, b.xy, section, poly_len(poly))
    ...
    def _apply_free(self, met):
        """Splice in, biggest saving first, every shortcut that needs no photos at all."""
        while True:
            free = next(((a, b, poly, section) for _s, a, b, poly, section
                         in self.candidates(Tier.ORDINARY, met)
                         if self.coverage.all_seen(poly)), None)
            if free is None:
                return
            a, b = free[0], free[1]
            # Finishing the one a tier was trying (its last photo just came in) is that
            # tier's; anything else needed no photos at all.
            tier = next((t for t, tr in self.trials.items()
                         if tr is not None and tr.a is a and tr.b is b), None)
            if tier is not None:
                self.trials[tier] = None
            else:
                self.log.count("instant_shortcuts")
            self._splice(*free, tier or Tier.ORDINARY)
    ...
    def update(self, met):
        """Apply free shortcuts; carry each tier's trial forward -- drop it if its anchors or
        section changed, recompute it around the known obstacles (see the module doc), splice
        it in once fully checked; then start the next trial in each tier."""
        if not self.enabled:
            return
        self._apply_free(met)
        chain = self.chain
        for tier in Tier:
            tr = self.trials[tier]
            if tr is None:
                continue
            i, j = chain.index(tr.a), chain.index(tr.b)
            current = chain.section(i, j) if i is not None and j is not None and j > i else None
            if current is None or abs(current - tr.section) > 1e-6:
                self.trials[tier] = None              # the chain changed under it
                continue
            poly = self.route(tr.a, tr.b, tr.section, tier)
            if poly is not None and (self.coverage.photos_needed(poly)
                                     > self.allowance(tr.section, poly)):
                self._give_up(tier, tr.a, tr.b)
            elif poly is None:
                if tier == Tier.BIG and self.route(tr.a, tr.b, tr.section, Tier.ORDINARY):
                    self.trials[tier] = None
                    self.log.event("shortcut-demoted", tr.a.xy, tr.b.xy)
                else:
                    self._give_up(tier, tr.a, tr.b)
            elif self.coverage.all_seen(poly):
                self._splice(tr.a, tr.b, poly, tr.section, tier)
                self.trials[tier] = None
            else:
                tr.poly = poly
        for tier in Tier:
            if self.trials[tier] is None:
                found = self.best(tier, met)
                if found is not None:
                    a, b, poly, section = found
                    self.trials[tier] = Trial(a, b, poly, section, next(self._uid))
    ...
    def _give_up(self, tier, a, b):
        self.trials[tier] = None
        self.given_up.add((a, b))
        self.log.count("shortcuts_given_up")
        self.log.event("shortcut-given-up", a.xy, b.xy)
```

`update` does the following, in order, after **every** change:

1. **`_apply_free`**: splice, biggest saving first, every candidate that needs **no photos at all** (`coverage.all_seen(poly)`: everything it touches is already photographed). If that candidate is the one a trial was working on, its slot is cleared (its last photo just came in); otherwise it counts as an `instant_shortcut`.
2. **Carry each tier's trial forward.** For each existing trial:
   - if its anchors moved or the section between them changed length (`chain.section`) since the trial began, the trial is **dropped** ("the chain changed under it");
   - otherwise the shortcut is **recomputed** around the currently known obstacles (`route`):
     - it now needs more photos than its allowance: **given up for good** (`given_up` set; `shortcut-given-up`);
     - it no longer qualifies for this tier: a BIG that would still be an ordinary shortcut is **demoted** (its slot is emptied, `shortcut-demoted`), otherwise given up;
     - it is now entirely photographed: **splice it in**;
     - otherwise its polyline is refreshed and the trial continues.
3. **Start the next trial** in every empty tier: `best(tier)` becomes a `Trial`.

The splice itself is `Chain.replace(i, j, poly, Placer.SHORTCUT)` (chain.py): it deletes every node between anchors `i` and `j`, and lays new nodes (`placed_by = SHORTCUT`) with **checked** links along `poly`. This is safe because `all_seen(poly)` guaranteed the new path is photographed and the `route()` guaranteed it avoids the known obstacles.

## 8.5 How a trial gets its photos

`Work.groups()` (chapter 3) adds each tier's trial as one more sequence:

```python
# flight/pathfinding/pathnode/work.py:116-120
        for tier in Tier:
            tr = self.shortcuts.trials[tier]
            if tr is not None:
                out[TIER_GROUP[tier]] = [(("shortcut", tr.a.xy, tr.b.xy),
                                          cov.places_along(tr.poly))]
```

- key `("shortcut", a_xy, b_xy)`: named by its **anchor geometry**, so every drone's copy names the same trial the same way (the ClaimBook uses keys to honour a mate's announced job).
- The photos are `places_along(tr.poly)`, in walking order from anchor `a`.
- **Priority.** The BIG trial is group 1 (just below PATH, **above** the ADJACENT rings, "so squares beside a section about to be replaced aren't photographed for nothing"); the ordinary trial is group 3 (below ADJACENT). A drone flies everything it holds in a higher group first.

## 8.6 Real runs

**Instant shortcuts.** A 14-mine, 2-drone toy mission, printing the shortcut events drone 1's planner logged:

```text
failure: None | stats: {'jumps': 2, 'nodes_in_mine': 8, 'bridges': 3, 'shortcuts': 2, 'instant_shortcuts': 2}
shortcut: tier=ordinary anchors (20.0, 0.0) -> (18.0, 49.9)  section 61.6 ft replaced by 49.9 ft
shortcut: tier=ordinary anchors (18.0, 60.1) -> (24.1, 84.0)  section 36.0 ft replaced by 24.8 ft
```

Both were free: the shorter path was already photographed, so `_apply_free` spliced it as soon as `advance()` ran. The first replaced a 61.6 ft section from the entry with a 49.9 ft path.

**A trial from start to finish.** A hook on `Shortcuts.update` prints drone 1's trials each time they change:

```text
tick  1  trials=[]
tick  9  trials=[]
         event: shortcut ordinary (20.0, 0.0) (14.0, 33.9) 40.5 35.4
tick 12  trials=[('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 6')]
tick 16  trials=[('big', (20.0, 0.0), (20.1, 50.0), 'section 73.6 -> 53.3 ft', 'photos 5'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 6')]
tick 17  trials=[('big', (20.0, 0.0), (20.1, 50.0), 'section 73.6 -> 53.3 ft', 'photos 4'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 6')]
tick 18  trials=[('big', (20.0, 0.0), (20.1, 50.0), 'section 73.6 -> 53.3 ft', 'photos 2'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 6')]
tick 19  trials=[('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 6')]
         event: shortcut-demoted (20.0, 0.0) (20.1, 50.0)
tick 24  trials=[('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 4')]
tick 25  trials=[('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 3')]
tick 26  trials=[('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 1')]
tick 27  trials=[('big', (20.0, 0.0), (22.1, 78.0), 'section 101.9 -> 81.2 ft', 'photos 10'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 1')]
tick 27  trials=[('big', (20.0, 0.0), (22.1, 78.0), 'section 101.9 -> 81.2 ft', 'photos 9'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 1')]
tick 28  trials=[('big', (20.0, 0.0), (22.1, 78.0), 'section 101.9 -> 81.2 ft', 'photos 7'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 1')]
tick 29  trials=[('big', (20.0, 0.0), (22.1, 78.0), 'section 101.9 -> 81.2 ft', 'photos 6'), ('ordinary', (20.0, 0.0), (6.0, 33.9), 'section 43.4 -> 36.9 ft', 'photos 1')]
tick 30  trials=[('ordinary', (20.0, 0.0), (22.1, 78.0), 'section 95.3 -> 81.2 ft', 'photos 6')]
         event: shortcut ordinary (20.0, 0.0) (6.0, 33.9) 43.4 36.9
tick 30  trials=[('ordinary', (20.0, 0.0), (22.1, 78.0), 'section 95.3 -> 81.2 ft', 'photos 4')]
tick 31  trials=[('ordinary', (20.0, 0.0), (22.1, 78.0), 'section 95.3 -> 81.2 ft', 'photos 3')]
tick 32  trials=[('ordinary', (20.0, 0.0), (22.1, 78.0), 'section 95.3 -> 81.2 ft', 'photos 1')]
tick 33  trials=[]
         event: shortcut ordinary (20.0, 0.0) (22.1, 78.0) 95.3 81.2
stats: {'jumps': 2, 'nodes_in_mine': 5, 'bridges': 5, 'replans': 1, 'shortcuts': 3, 'instant_shortcuts': 1}
```

Read it as a story:

- **tick 9**: an instant shortcut is spliced (needs no photos).
- **tick 12**: an **ordinary** trial starts: anchors (20.0, 0.0) (the entry) and (6.0, 33.9); the section is 43.4 ft, the shortcut 36.9 ft (ratio 0.85 < 0.9, saving 6.5 ft >= 4), and it needs 5 photos (allowance `max(8, 3.2)` = 8).
- **tick 15**: a **big** trial starts as well: entry to (20.1, 50.0), 73.6 ft -> 53.3 ft (ratio 0.72 < 0.8, saving 20.3 >= 20), 6 photos (allowance 10.1). Drones fly it first (group 1): its photo count falls 6, 4, 3 over ticks 15 to 17.
- **tick 18**: the big trial disappears with a `shortcut-demoted` event: recomputing it no longer qualifies as big but would still qualify as ordinary, so `update` empties the BIG slot. (The ordinary slot is occupied by the other trial, so it is not restarted.)
- **ticks 23 to 25**: the ordinary trial's photo count falls 4, 3, 1.
- **tick 26**: another big trial starts, entry to (22.1, 78.0), with 10 photos.
- **tick 29**: the ordinary trial's last photo is in: `update` splices it (the `shortcut ordinary ... 43.4 36.9` event). The chain got shorter, so the big trial's section (101.9 ft) no longer matches the chain: it is dropped ("changed under it") and immediately restarted as an *ordinary* trial with the new 95.3 ft section.
- **tick 36**: its last photo arrives; spliced: 95.3 ft -> 81.2 ft.

## 8.7 Related: what the entry anchor means for two sides

Nothing special: shortcuts are per side and per copy. Each drone's copy computes the same candidates from the same chain (given the same events), so trials are named identically and can be handed out through claims. When the two sides have met (`met`), the **target** becomes an anchor too, which allows a shortcut over the meeting point. What is reported at the end is the joined route (chapter 10).
