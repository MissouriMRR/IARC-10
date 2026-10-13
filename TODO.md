# TODO: open work

Updated 2026-09-28. Only open items; history lives in commit messages.
How it works: [ALGORITHM.md](ALGORITHM.md) (being rewritten).

**Nothing from this work is committed yet.** Large uncommitted change set, including the
legacy-planner archive move. Commit when the user decides.

## In progress

- [ ] Per-drone planner copies (own copy of its side's planner, no roles, `SIDE_JOB`
      announcements, keep-apart within and across sides). Simulator sweep 90/90 ok; flight
      harness `integration_tests/multi_drone_sim_test.py` fails with separation aborts
      (2-drone 0.00 m, 4-drone 0.64 m). Being fixed.
- [ ] `docs/walkthrough/`: state-machine walkthrough with code snippets. Keep in sync with
      every logic change.
- [ ] `docs/testing/` + `tools/testing/`: what tests exist, how to run them; sweep, golden,
      repro, benchmark and snapshot tools.

## Next (in order)

1. [ ] Wire the rebuilt geometry into `pathnode/routes.py` in place of `ZoneMap`:
      `flight/pathfinding/nodeField/` (`Field`, `FieldConnections.nodeGraph` as the stored graph
      A* routes over; `protoMine`).
  - [ ] Expect different routes: 18/200 more reachable, ~0.02 ft longer on average.
  - [ ] Re-run the full verification (sweep, flight harness, pytest).
2. [ ] Expanding mines, per [docs/design/expanding_mines.md](docs/design/expanding_mines.md).
      Blocked on the user answering its 11 open questions (below).
  - [ ] Fix `mine_rule.apply` collapsing middle route fragments when an obstacle cuts the
        route in several places (needed before whole-field expansion).
3. [ ] Rewrite `ALGORITHM.md` from scratch with diagrams; fold in
      [docs/priority_groups.md](docs/priority_groups.md) and the walkthrough.
4. [ ] Cleanups
  - [ ] Stale `pathnode/claims.py` docstrings (may already be fixed; check).
  - [ ] Unused imports (`pyflakes`).
  - [ ] Project `CLAUDE.md` for the team, once things settle.

## Decisions for the user

Recommended default in brackets where one exists.

- [ ] Scoring model: route length vs flight time vs route squares near unknown mines.
      Most other decisions depend on it. [route length first, near-mine squares as tie-break]
- [ ] Corridor + rings vs band coverage. [keep corridor + rings]
- [ ] Is the OUTER ring worth its extra flying? Needs a multi-seed sweep (seed 76 with the
      real footprint: danger squares on route 4 -> 0 for 21% more flying).
      [keep off until the sweep says otherwise]
- [ ] Shortcut parameters (`SHORTCUT_RATIO`, allowance). [keep current]
- [ ] Switch the simulator to the real 8.5 x 6.4 ft footprint (flight code already uses it;
      the 6x4 placeholder distorts photo counts). [yes]
- [ ] The 11 expanding-mines questions in `docs/design/expanding_mines.md`: growth rule,
      level cap, blocked squares grow, IARC `buffer_width` meaning, report latest
      non-compromised snapshot, late-mine fallback, time guard, lost partner side, join hash
      check, fly OUTER while waiting, incremental graph threshold.

## Open technical items

- [ ] Partner stand-in target can stall silently (suspected; `pathnode/target.py`,
      `planner.py`). Reproduce or rule out.
- [ ] Profile on the Pi and set a per-pass time budget (per-drone copies roughly doubled
      planner CPU).
- [ ] Non-rectangular fields are planned as their bounding rectangle.
- [ ] Blocked squares are local to one planner copy; the partner can aim at a blocked square
      (stand-in path above).
- [ ] Misses can't tell position error from a bad footprint: a drone always arriving more than
      half a square off counts no misses; the tile is just re-issued.
- [ ] After an ack the top side leaves. A late mine on the joined route then means a half
      report at the time limit.
- [ ] `tests/test_emergency_land.py`: 3 known failures (harness never sets
      `Interdrone.max_messages_per_tick`).

## Test gaps

- [ ] Big -> ordinary shortcut demotion.
- [ ] Shortcut dropped when its section changes.
- [ ] Join bridge fallback.
- [ ] Flight-side drain order.
- [ ] Transport backoff (no socket-failure harness).
- [ ] Flight harness never rotates the drone or drops messages.

## Related

- `interdrone_communication/TECHNICAL_DEBT.md`: messaging-layer debt.
