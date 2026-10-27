# 8. Shortcuts: replacing checked stretches with shorter ones

> Part of the [execution walkthrough](README.md). Previous: [7. Messages from other drones](07-messages-from-other-drones.md). Next: [9. Two sides meeting](09-two-sides-meeting.md).

**What this chapter covers.** A route built by the mine rule is often longer than it needs to be: when a mine is found, the route detours around it, and later discoveries can make an earlier detour unnecessary or leave a zig-zag. `pathnode/shortcuts.py` looks for **checked sections of the chain that a shorter path could replace**, tests them by photographing the shorter path, and splices it in.

> **Status.** The path lengths come from `Routes.shortest`, whose geometry is a `PathEngine` (`ZoneMap` by default, chapter 2).

## 8.1 The idea, from the module docstring

{{snip: flight/pathfinding/pathnode/shortcuts.py :: Shortcuts: replacing a checked => that pair is given up for good.}}

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

{{snip: flight/pathfinding/pathnode/shortcuts.py :: def anchors(self, met): => in corners] :: def route(self, a, b, section, tier): => return poly :: def allowance => shortcut_ft_per_photo)}}

- `anchors`: indices of nodes that are the entry, a JUMP node, the last node when `met`, or that lie **exactly on an obstacle corner** (`routes.corners()`, rounded to 6 decimals). Path nodes laid by the mine rule's bridge/re-plan sit on corners because shortest paths only bend around corners.
- `route(a, b, section, tier)`: ask `Routes.shortest(a, b)` for the shortest path around all known obstacles. If it is not shorter than `ratio x section`, or the saving is below the tier's minimum, it is **not** a shortcut. (If `a` and `b` are the same point, the section is a loop back to where it started and the shortcut is the trivial `[a, b]`.)

{{snip: flight/pathfinding/pathnode/shortcuts.py :: def candidates(self, tier, met): => return found}}

- Every pair of anchors is considered **except entry + target together** (that would be the whole path).
- A cheap filter first: no path is shorter than the straight line, so `dist(a, b) >= ratio x section` cannot qualify.
- Results are sorted **biggest saving first** and cached on (obstacle version, anchor nodes, section lengths, number of given-up pairs). The cache is dropped as soon as the chain or obstacles change.

`best(tier, met)` picks the first candidate whose `photos_needed(poly)` is within its allowance and which the other tier is not already trying.

## 8.4 Free shortcuts, trials, splicing

{{snip: flight/pathfinding/pathnode/shortcuts.py :: def _splice(self, a, b, poly, section, tier): => self.log.event("shortcut" :: def _apply_free => self._splice(*free, tier or Tier.ORDINARY) :: def update(self, met): => self.trials[tier] = Trial(a, b, poly, section, next(self._uid)) :: def _give_up => self.log.event("shortcut-given-up"}}

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

`Work.groups()` (chapter 3) asks one `ShortcutSource` per tier (`work_sources.py`) for its work; each adds that tier's trial as one more sequence:

{{snip: flight/pathfinding/pathnode/work_sources.py :: class ShortcutSource => return set() if tr is None}}

- key `("shortcut", a_xy, b_xy)`: named by its **anchor geometry**, so every drone's copy names the same trial the same way (the ClaimBook uses keys to honour a mate's announced job).
- The photos are `coverage.places_along(tr.poly)`, in walking order from anchor `a`.
- **Priority.** The BIG trial is group 1 (just below PATH, **above** the ADJACENT rings, "so squares beside a section about to be replaced aren't photographed for nothing"); the ordinary trial is group 3 (below ADJACENT). A drone flies everything it holds in a higher group first.

## 8.6 Real runs

**Instant shortcuts.** A 14-mine, 2-drone toy mission, printing the shortcut events drone 1's planner logged:

{{run: demos/toy_shortcut.py}}

Three of the four were free (`instant_shortcuts: 3`): the shorter path was already photographed, so `_apply_free` spliced it as soon as `advance()` ran. The first replaced a 59.9 ft section from the entry with a 49.9 ft path. The fourth went through a trial; one other candidate was given up (`shortcuts_given_up: 1`).

**A trial from start to finish.** A hook on `Shortcuts.update` prints drone 1's trials each time they change:

{{run: demos/toy_shortcut_trial.py}}

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
