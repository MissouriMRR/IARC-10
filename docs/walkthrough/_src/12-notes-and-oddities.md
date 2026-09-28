# 12. Notes and oddities found while walking the code

> Part of the [execution walkthrough](README.md). Previous: [11. Glossary](11-glossary.md).

Things that look wrong, dead, surprising or stale, collected while writing the chapters. Each item says where it is described in context. Nothing here was changed: the code is read-only for this documentation. "As read" means as the files were at the time of the last rebuild; re-check before acting on any of it.

## Likely problems

1. **The checked-in field corners are rejected by the planner.** `mission_config.json` (working tree) and `mission_config_full.json` list `mission_field_corners` in the order 1, 2, 3, 4, which traces a bow-tie. `validate_field_corners` raises "the last two corners look swapped", so the mission branch of `Takeoff` (`configureField` -> `make_planner`) would raise `ValueError` on the ground. Swapping the last two corners passes (about 82 x 300 ft). (Ch. 2)
2. **`start_coord` in `mission_config.json` is nowhere near the field corners** (lat 36.02, lon -95.94 versus lat 34.62, lon -86.81). `MissionPlanner` clamps its x into the field width, so `entry_x` silently becomes a field edge. (Ch. 2)
3. **`Scan` only catches `FormationLost`.** `LegStalled` and `FlightInterrupted` raised by `move_to` are handled by the POIF state but not by `Scan`; they end the state machine task with the drone airborne. (Ch. 4)
4. **Blocking calls inside the event loop.** `try_get_server_message(timeout=0.02)` and `queue_client_message` wait on `concurrent.futures.Future`s from the loop's own thread, freezing every task for up to 20 ms whenever no message is waiting. (Ch. 1)
5. **Climb-retry vs `MISS_BLOCK`.** `Scan` allows up to 8 climb-and-recapture attempts, but each capture calls `planner.own_footprint`, which counts misses for every needed square still uncovered; three misses **block the square as an obstacle**. So a persistent miss is blocked after 3 photos, well before the 8 attempts. Whether that is intended is not stated. (Ch. 4)

## Surprising behaviour (may be intended)

6. **`replan_needed` does nothing useful today.** Reading the code, the state machine never yields between `CalcScanPath` clearing the flag and `Scan` checking it except when the queue is empty (in which case `Scan` bounces anyway), and `CalcScanPath` clears any flag set mid-leg before it is read. The mechanism that matters is that `CalcScanPath` runs after *every* photo and always drains the staging fields. A message never aborts a leg in flight. (Ch. 7)
7. **Idle takeover produces overlapping queues (re-checked in round 1: still true as read).** A drone whose copy gives it nothing takes its mate's queued work *except the single announced next photo*. In the toy run (with the new 10 ft / 7.5 ft keep-apart) both drones still end up flying the same forward path, drone 2 starting further up (from (11.0, 14.3) while drone 1 starts at (10.9, 2.7)). The rule is unchanged: only the single announced next photo is protected. Photos shared over `SHARE_PHOTOS` then remove covered ground from each copy. (Ch. 3)
8. **FIXED (round 1): a mate never heard from is no longer treated as "no job".** `SideAgent.plan` now leaves it out of `held` until it has announced something, so it is unknown rather than idle. (Ch. 3, 7)
9. **`ARM_ACK` means "command received and pings are fine"**, not "motors armed": the receive loop sends it when the `ARM` message arrives, before `Start` calls `Drone.arm()`. (Ch. 1)
10. **In every mode except `"Prompted"`, `Start` waits for an `ARM` message** (nothing in `start_impl.py` sets `cmd_msg` itself), so `"Automatic"` and `"Competition"` need the app (or a sender) to start a flight. (Ch. 1)
11. **The takeoff margin.** `Takeoff` mission branch calls `takeoff(MISSION_ALTITUDE_M)` with the default 1.5 m margin, so the vehicle is commanded to 3.5 m; the `DEMO` branch passes `margin=0.0` for the very reason (its comment says the first leg spends time descending). (Ch. 2)
12. **Photo order is not chronological across drones.** A drone applies its own photo immediately (in `DroneShare`) and its mate's on a later pass (in `apply_inbox`), so copies pass through different intermediate chains. `mine_hash` compares only the mine sets. (Ch. 7)
13. **Only one mine per photo.** `_check_photo_latlon` reports the highest-confidence detection; other mines in the same frame are found later, if at all. (Ch. 5)
14. **`Land` returns to 23 m first.** `Drone.return_to_launch` flies to home at 23 m relative altitude before `RTL`; the code comment notes "Min alt should be in constants file". (Ch. 10)
15. **On a time-out the single-side branch reports the chain's whole route**, including PENDING and FORWARD stretches never photographed; nothing marks them unverified there (the two-sided case sends `complete=False` and may set `reported_half_route`). (Ch. 10)
16. **Mine agreement is only logged.** A `FIELD_CHECKSUM` mismatch is logged on both ends; only the periodic `CROSS_PAIR_MINE_SYNC` (and the reporter's hash check in `EndRun`) reconcile the sets. (Ch. 9, 10)
16b. **Per-type message queues grow without bound.** The receive loop appends *every* message to `interdrone_messages[message.id]` (marked `TODO GET RID OF THIS SOON`); only a few types are ever consumed (`PING_ACK`, `SEND_DRONE_STATUS`, ...), so e.g. every `POSITION_REPORT` (5 Hz from each peer) is kept for the life of the process. (Ch. 7)

## Dead, unused or placeholder

17. **`ExpandNodes` is a no-op.** `MissionPlanner.expand_margin` logs "not applied (next phase)" and returns `None`; `MINE_RADIUS_EXPANSION_FT = 2.0` is a placeholder. (Ch. 10)
18. **`NEW_WAYPOINTS`, `REACHED_WAYPOINT`, `RECONFIRM_WAYPOINTS`, `checkForCollision`** are only exercised by POIF; in the mapping mission no drone sends them, so peers' `list_of_waypoints` stay empty and collision holds rely on `POSITION_REPORT` alone. (Ch. 4)
19. **Unused in the mapping flow:** `Interdrone.read()`, and the `send_survey_*` / `send_mission_end*` senders have no callers in the states. `SEND_APP_COORDS_ACK` is a `pass`. `MissionPlanner.own_photo` ignores its `corners_latlon` argument (the footprint was already recorded by `Scan`).
20. **`Drone.waypoints` removal by identity** (`gotoWaypoint`) guards against a queue replaced mid-leg, but nothing replaces it mid-leg today. (Ch. 4)

## Stale comments and docstrings

21. `pathnode/planner.py` module docstring (and the `_make_way` docstring's "non-lead drone"): "The side's lead (its lower-id drone) owns this planner and plans for every drone of the side". Each drone owns a copy; there are no leads. (`FlightSettings.mission_sides` says so.)
22. `pathnode/claims.py`: "the pair", `("self", "member")`, "the member pioneers". `pathnode/sim.py` module docstring: leads and members, `ASSIGN_WORK`. `pathnode/__init__.py`: "the two leads' point As" and mentions of the old planner.
23. `Land` docstring says it "transitions to the Start state"; it returns `None` and the process ends. `Takeoff` docstring says it transitions to `Land`; the mission branch returns `CalcScanPath`. `Scan` docstring describes `Scan -> DroneShare -> Scan` and says `DroneShare` returns to `CalcScanPath` only when it finds a mine; `DroneShare` always returns `CalcScanPath`. `AppShare` docstring refers to its "own older docstring".
24. `calc_scan_path_impl._camera_footprint_ft` mentions `flight/pathfinding/path_subdivision.py` (deleted). The `along`/`across` assignment (`hFov` -> along) is admitted unconfirmed against the real camera mount.
25. `Drone.replan_needed`'s comment says `Scan`'s goto loop checks it between legs and clears it "once CalcScanPath has actually recomputed"; see item 6.
26. **Keep-apart shrinks queues drastically at the start.** With `keep_away_ft` = 10 ft, a second drone standing near the entry (or near its mate's next photo) sees most of its dealt work filtered out of its queue: in the toy run drone 2's queue is empty for several passes while drone 1 pioneers. It is by design (photos stay work for later), but the side-mate can look idle for a while. (Ch. 3, 6)
