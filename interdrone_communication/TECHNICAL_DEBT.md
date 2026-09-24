# Inter-drone messaging — technical debt register

Recorded 2026-09-21 from a read-only audit of the messaging layer. Nothing in here has been
fixed; this is a backlog for review and action.

**Scope.** The layer spans two places, and most findings involve both:

- `interdrone_communication/` — transport and wire format (`client.py`, `server.py`,
  `networking_interface.py`, `message_types.py`)
- `state_machine/interdrone.py` — the 2219-line `Interdrone` class: send helpers, the receive
  dispatch, and some state-machine control that ended up here

Consumers touched: `state_machine/drone.py` (`pending_*` staging attributes) and
`state_machine/states/impl/*.py`.

**Status markers.** `VERIFIED` = the code path was read end to end. `SUSPECTED` = inference not
fully confirmed; check before acting. Findings 1–5 and 7 were independently re-checked against
the source when this file was written.

Line numbers are from the working tree on branch `poif-test` at the recording date and will
drift; treat them as starting points, not addresses.

---

## P0 — Safety-relevant

### 1. The receive loop has no exception guard, so one bad message silently kills all comms
`VERIFIED` — `state_machine/interdrone.py:1482`, loop tail `:2213`

The entire receive loop is wrapped in `try: while True: ... except KeyboardInterrupt:`. Asyncio
never delivers `KeyboardInterrupt` into a task, so that handler is dead code and there is **no**
`except Exception`. The task is created fire-and-forget (`state_machine/flight_manager.py:79-81`,
`main.py:25`) with no `add_done_callback` — even though this file already defines
`_log_task_failure` (`:118`) for exactly this purpose and applies it to `gather_swarm_status`
(`:2191`).

**Consequence.** Any exception from any single case permanently stops all inter-drone
communication — no `POSITION_REPORT` for collision avoidance, no `EMERGENCY_LAND`, no `LAND` —
while the state machine keeps flying. The only trace is Python's "exception never retrieved"
warning at interpreter exit, which on a drone never happens.

Known live triggers:
- `:1597` `self.drone.vehicle.armed = False` — the `vehicle` property raises `RuntimeError`
  when `_vehicle is None`.
- `:2115-2116` `self.drone._vehicle.parameters[...]` — `TypeError` when `_vehicle is None`.
- ~25 unguarded `message.data["key"]` accesses — see finding 2.

**Direction.** Wrap the per-message dispatch in `try/except Exception: logging.exception(...)` so
a bad message is dropped rather than fatal, and attach `_log_task_failure` to the loop task at
both spawn sites.

### 2. Validation failure hollows out `data` instead of rejecting the message
`VERIFIED` — `interdrone_communication/message_types.py:702-717`

On any schema mismatch, `Message.__post_init__` emits `warnings.warn` and does
`object.__setattr__(self, "data", {})`, then returns. The message still arrives at its handler,
just empty. Nearly every handler then indexes `message.data["key"]` directly, so a malformed or
version-skewed peer message is a `KeyError` — which, per finding 1, takes down the whole layer.

The codebase already knows this is a hazard: `_gather_swarm_status` (`interdrone.py:1213-1219`)
carries a comment explaining why it uses `.get("drone_cmd_msg", ...)`, and `SHARE_PHOTOS`
(`:1871`) uses `.get("photos", [])`. Every other case indexes directly — `POSITION_REPORT`
`:1836-1838`, `NEW_WAYPOINTS` `:1682`/`:1690`, `SEND_SEGMENT_B_WAYPOINTS` `:1776`,
`FIELD_CHECKSUM` `:1940`, `FIELD_CHECKSUM_ACK` `:1949`, all five `CROSS_PAIR_*` cases
`:1963-2014`, `SEND_GROUND_TRUTH_COORDS` `:2115`, `SEND_GPS_OFFSET` `:2149`.

Related, same file: `message_types.py:682` sets `sender_id` to `()` — an empty **tuple** — on the
`UNKNOWN` fallback path, where the field is typed `int`. Any `UNKNOWN` message carries a tuple
sender id into `get_drone_state_from_id`.

**Direction.** Make validation failure reject the message (raise, or return a sentinel the loop
drops with a log) rather than silently emptying it. That removes the need for defensive `.get`
at 25 call sites.

### 3. `send()` can block the flight event loop indefinitely
`VERIFIED` — `interdrone.py:1462-1466` → `interdrone_communication/networking_interface.py:26-28`

`send()` is a plain synchronous method calling `queue_client_message`, which does
`asyncio.run_coroutine_threadsafe(...).result(timeout=None)` — a blocking `concurrent.futures`
wait, issued from the flight event loop's thread, with no timeout. If the networking thread's
loop stalls or dies, every `send()` blocks the entire flight event loop forever, including the
paths used by `start_emergency_land` and every `report_position` during POIF.

Lower severity, same shape: `try_get_server_message(timeout=0.02)` (`:1496`) hard-blocks the
event loop thread for up to 20 ms per tick on a loop that sleeps 0.1 s — roughly a 17% blocking
duty cycle imposed on the state machine even when idle.

**Direction.** Give `queue_client_message` a bounded timeout and have `send` report failure.
Longer term, make the transport awaitable instead of thread-bridged.

### 4. `DISARM` fan-out is addressed to the sender instead of the swarm
`VERIFIED` — `interdrone.py:1586-1597`

```python
case MessageType.DISARM:
    self.cmd_msg = CMD_MSG.DISARM
    if self.flight_settings.current_drone_ID == 1:
        disarm_message = Message.create(
            id=MessageType.DISARM,
            drones_to_send_data=(message.sender_id,),   # <-- sender, not other_drones_in_mission
            ...
```

`EMERGENCY_LAND` (`:2034`) and `LAND` (`:2061`) correctly use `other_drones_in_mission`. As
written, an app-issued DISARM (sender 0) makes drone 1 send DISARM back *to the app* and disarm
only itself; drones 2..N never hear it. Unlike LAND/EMERGENCY_LAND it also has no duplicate
guard.

**Direction.** Use `tuple(other_drones_in_mission)`. The "drone 1 fans this out" step is
hand-rolled five times (`:1590`, `:1645`, `:2032`, `:2059`, `:2072`) — factor it into one helper.

### 5. `SEND_GROUND_TRUTH_COORDS` blocks the loop for 5 s and discards the offset it computes
`VERIFIED` — `interdrone.py:2098-2118`

Three distinct problems in one case body:

- `for i in range(10): ... await asyncio.sleep(0.5)` runs **inline in the receive loop** — five
  seconds during which no message of any type is handled, including `EMERGENCY_LAND`. Other
  potentially-blocking cases (`REQUEST_SWARM_STATUS` `:2186`,
  `COMMAND_OFFSET_DISTRIBUTION` `:2146`) correctly use `ensure_future`; this one does not.
- `flight_settings.gps_lat_offset` / `gps_lon_offset` are set to `0.0` and **never** updated with
  the computed value — the real offset goes only into ardupilot params. A later
  `COMMAND_OFFSET_DISTRIBUTION` → `distribute_gps_offset()` (`:1081-1083`) therefore distributes
  `0.0, 0.0` to the whole swarm, and the app receives an ack reporting success. Two explicit
  `TODO`s in this block confirm the `0.0` is a knowing placeholder, so the misleading ack is the
  actionable part.
- Uses the private `self.drone._vehicle` (`:2115-2116`), bypassing the `vehicle` property's
  connection check.

**Direction.** Move the averaging into a background task; assign the computed offsets to
`flight_settings`; use the public `vehicle` property; don't ack success for a no-op.

### 6. `state.touch()` is called on 6 of ~25 cases, and staleness gates collision avoidance
`VERIFIED` — touch at `interdrone.py:1680, 1788, 1834, 1849, 1869, 1921`; consumers
`state_machine/drone_state.py:139-147`, `states/impl/poif_impl.py:144/153/349`, `drone.py:770`

`DroneState.is_stale()` exists to stop the swarm blocking on a silent peer's frozen belief — its
docstring cites a real deadlock. But `touch()` is missing from `ARM_ACK`, `START_TAKEOFF_ACK`,
`START_DEMO_ACK`, `START_MISSION_ACK`, `NEW_WAYPOINTS_ACK`, `REACHED_WAYPOINT_ACK`, `PING`,
`SEND_SEGMENT_B_WAYPOINTS`, and all five `CROSS_PAIR_*` cases. A peer that is demonstrably alive
— acking commands and relaying mine positions — can still be judged stale and have its route
ignored by collision avoidance.

**Direction.** Call `touch()` once in the loop before dispatch, for every message with a known
`sender_id`, instead of per-case.

---

## P1 — Dispatch and staging

### 7. `interdrone_messages` grows without bound; only 4 of ~42 types are ever drained
`VERIFIED` — `interdrone.py:197-203` (init), `:1500-1502` (enqueue)

```python
self.interdrone_messages.setdefault(message.id, queue.Queue()).put(message)  # TODO GET RID OF THIS SOON
```

`__init__` pre-creates queues for four types, but the `setdefault` in the loop enqueues **every**
message of every type, in addition to dispatching it. Only `PING_ACK`, `PING_NACK`,
`SEND_GPS_OFFSET_ACK` and `SEND_DRONE_STATUS` are ever drained (`ping_drones` `:274-280`,
`distribute_gps_offset` `:1088`, `_gather_swarm_status` `:1193-1197`). `POSITION_REPORT` is a
continuous broadcast and `SHARE_PHOTOS` carries image payloads — both accumulate for the whole
flight. This is a real memory leak on a long mission, not untidiness.

**Direction.** Enqueue only the request/response types that need a mailbox, driven off an
explicit set — or give each of those four a dedicated `asyncio.Queue` attribute and delete the
dict (as the existing TODO intends).

### 8. The 703-line dispatch should be table-driven; roughly a third is duplicated bodies
`VERIFIED` — `interdrone.py:1503-2205`, 42 cases, no `case _:`

Concrete duplication:
- **Four identical "set one `DroneState` flag from sender" bodies** — `ARM_ACK` `:1545`,
  `START_TAKEOFF_ACK` `:1613`, `START_DEMO_ACK` `:1633`, `START_MISSION_ACK` `:1666`. They differ
  only in the field assigned and the exact typo in the `print` ("Something is ary!" appears 4×).
  `ARM_ACK` re-implements `get_drone_state_from_id` inline (`:1547-1550`) rather than calling the
  helper at `:258` that the other three use.
- **Two byte-identical bodies** — `NEW_WAYPOINTS_ACK` `:1749-1752`,
  `REACHED_WAYPOINT_ACK` `:1840-1844`.
- **Four near-identical "promote cmd_msg, drone 1 fans out, else ack" bodies** —
  `START_TAKEOFF` `:1598`, `START_DEMO` `:1621`, `START_MISSION` `:1654`, with `ARM` `:1505` as
  the variant.
- **Two identical "drone 1 relays this ack to the app" bodies** —
  `SEND_GROUND_TRUTH_COORDS_ACK` `:2119`, `COMMAND_OFFSET_DISTRIBUTION_ACK` `:2162`.
- **Two identical "drone 1 relays to the real target" bodies** —
  `SEND_GROUND_TRUTH_COORDS` `:2091`, `COMMAND_OFFSET_DISTRIBUTION` `:2138`.

Error handling is inconsistent in three independent ways: a missing `DroneState` is a bare
`print` in 4 cases and a silent skip in ~10; `message.data` access is defensive in 2 and
unguarded in ~25; `Pathfinder.instance is None` is guarded in `FIELD_CHECKSUM` (`:1932`) and
nowhere else.

There is **no `case _:` default**, so an unhandled or `UNKNOWN` message is swallowed with zero
logging. That is what allowed `MessageType.PING` to have no case at all until the multi-drone
harness happened to surface it.

**Direction.** Replace with `dict[MessageType, Handler]` where each handler is
`async def _on_xxx(self, message) -> None`. The loop then does `touch()`, lookup,
`try/except`, and log-on-miss uniformly. The four ack-flag handlers collapse to one
parameterised entry.

### 9. Verification waypoints can be silently discarded mid-flight after being acked
`VERIFIED` — `interdrone.py:1989` → `states/impl/calc_scan_path_impl.py:254-267`, `:296`/`:348`

`_drain_verification_waypoints` empties `Drone.pending_verification_waypoints` and returns
`Waypoint`s that go into `resetWaypoints(_build_waypoints(...) + verification_waypoints)`.
`resetWaypoints` (`drone.py:628-629`) **replaces** rather than appends. So if a replan fires —
which every `CROSS_PAIR_*`/`SHARE_PHOTOS` arrival triggers — before those waypoints are flown,
the unflown remainder is gone permanently: it is no longer staged, and the sender already
received `CROSS_PAIR_PATCHED_SPAN_ACK` saying verification was queued (`interdrone.py:874-879`).

The equivalent loss for segment B is documented as an accepted gap
(`interdrone.py:1764-1771`); this one is not, and unlike segment B nothing ever re-sends it. Net
effect: a patched span the other pair asked to have re-photographed never is, and nothing
reports the gap.

**Direction.** Keep verification waypoints out of the queue `resetWaypoints` clobbers, or
re-stage the unflown remainder before resetting.

### 10. `last_sent_segment_b` is recorded even when the send silently did nothing
`VERIFIED` — `interdrone.py:562-585`, `calc_scan_path_impl.py:342-346`

`send_segment_b_waypoints` wraps its body in `if state is not None:`, so an unknown target id
makes it a silent no-op with no log. The caller then unconditionally sets
`drone.last_sent_segment_b = b_places_key`, and the dedupe guard suppresses all retries. A
mis-set `paired_drone` therefore yields an ASSISTANT that never receives work, a GAMBLER that
believes it delegated segment B, and no diagnostics. `send_new_waypoints` (`:544-557`) has the
same shape.

**Direction.** Log at warning when `get_drone_state_from_id` returns `None`, and set
`last_sent_segment_b` only on a send that actually happened.

### 11. `CROSS_PAIR_POINT_A_SYNC` forces a replan despite being documented as best-effort
`VERIFIED` — `interdrone.py:2001-2016`

The message is documented three times as an un-acked, un-retried continuous stream where "a
dropped one is superseded by the next" (`message_types.py:98-101`, `interdrone.py:901-904`), yet
its handler sets `drone.replan_needed = "cross_pair_point_a_sync"`, which bounces Scan back to
CalcScanPath (`scan_impl.py:295`). A partner replanning frequently drives this drone into
repeated CalcScanPath↔Scan cycles, each calling `pf.retarget_approach_target`, which mutates the
live graph. Same shape as the starvation bug documented at
`integration_tests/multi_drone_sim_test.py:71-84`. `POSITION_REPORT`, with identical stated
semantics, correctly sets no flag.

**Direction.** Apply point-A sync opportunistically on the next natural CalcScanPath entry
rather than forcing one. Note this finding is largely moot if finding 18's deletion goes ahead.

### 12. `DroneState.waypoint_up_to_date` is written in 6 places and read nowhere
`VERIFIED` — written `interdrone.py:557, 583, 608, 1724, 1752, 1844`; only reader is its own
property (`drone_state.py:248`)

The whole NEW_WAYPOINTS/RECONFIRM_WAYPOINTS resync protocol maintains this flag and nothing
consults it. `send_segment_b_waypoints:583` sets it `False`, and since no
`SEND_SEGMENT_B_WAYPOINTS` ack exists it can never be set back — so even if a reader appeared it
would be permanently wrong for the GAMBLER→ASSISTANT direction.

**Direction.** Delete it, or wire it into the resync decision at `:1707` that currently
recomputes checksums from scratch instead.

---

## P2 — Protocol debt: defined-but-dead, sent-but-unhandled

All `VERIFIED` by grepping both send and receive directions.

### 13. Eight message types have a full send helper but no receive case anywhere

| Type | Send helper | Receive case | Callers of send |
|---|---|---|---|
| `SURVEY_START` 565 | `:640` | none | none |
| `SURVEY_START_ACK` 566 | `:659` | none | none |
| `SURVEY_END` 570 | `:676` | none | none |
| `SURVEY_END_ACK` 571 | `:694` | none | none |
| `MISSION_END` 585 | `:965` | none | none |
| `MISSION_END_ACK` 586 | `:983` | none | none |
| `SEND_SEEN` 590 | `:1000` | none | none |
| `SEND_SEEN_ACK` 591 | `:1021` | none | none |

~120 lines of send code with neither producer nor consumer. `send_survey_start`'s docstring
(`:643`) claims it sets the `CMD_MSG` — it does not. Correspondingly `CMD_MSG.SURVEY_START`
(`:144`) and `CMD_MSG.MISSION_DONE` (`:140`) are never assigned or compared anywhere.

**Direction.** Delete all eight plus the two dead `CMD_MSG` members, or file an issue naming what
would use them.

### 14. Nine schema'd types are neither sent nor handled by the flight software
`APP_TEST` 400, `SET_SCAN_STATUS` 410, `SET_HOVER_STATUS` 412, `REQUEST_DRONE_LOCATIONS` 415,
`SEND_DRONE_LOCATIONS` 425, `SEND_APP_SCANNING_ERROR` 421, `MISC2` 597, `MISC3` 598, `MISC4` 599
— schemas defined in `message_types.py`, zero references anywhere in `state_machine/`.
(`HEARTBEAT`, `SERVER_DEFAULT_RESPONSE` and `SPEED_TEST_*` are legitimately used by the
standalone transport tools and are fine.)

`SEND_DRONE_LOCATIONS`'s schema (`:186-192`) also still hardcodes `drone1_data`/`drone2_data`,
which cannot express the 3- and 4-drone configurations the rest of the layer supports.

### 15. Two types are handled but unreachable from the flight software
- `DEMO_DONE` 537 — case at `:1642` (drone 1 even re-fans it out), but no drone sends it: there
  is no `send_demo_done` helper, and `CMD_MSG.DEMO_DONE` set at `:1643` is read by no state
  (`takeoff_impl.py:70`'s `app_commands` check excludes it). Reachable only from the app.
- `SEND_APP_COORDS_ACK` 436 — case at `:2084` is `pass` with a `TODO`; nothing sends it.

### 16. Reliability semantics differ per message type with no single source of truth
`VERIFIED` — `interdrone_communication/client.py:210-225`

The resend-on-failure set lives in the transport, ack expectations live in the handlers, and the
"deliberately unacked" rationale lives in docstrings. They disagree:

- `SEND_SEGMENT_B_WAYPOINTS` hands an ASSISTANT its entire workload, is **not** in the resend
  set, and has **no ack** — one dropped send leaves the ASSISTANT idle until
  `ASSISTANT_IDLE_TIMEOUT_S` fires.
- `SHARE_PHOTOS`, `FIELD_CHECKSUM` and all five `CROSS_PAIR_*` types define and send acks, but no
  sender tracks or retries on a missing one. The code admits it: `interdrone.py:1974-1976`, "No
  resend-on-timeout mechanism exists yet for `CROSS_PAIR_MINE_RELAY` to cancel." Those ack fields
  exist only to feed a `logging.debug`.
- `ARM_ACK`/`ARM_NACK`/`START_*_ACK` are themselves in the resend set — resending an *ack* is a
  different reliability model from resending a command, adopted without comment.

**Direction.** Put reliability class (`fire_and_forget` / `acked` / `retried`) in one table beside
`EXPECTED_SCHEMA`, and have both transport and handlers read it.

### 17. Drone-1 relay cases may amplify a message addressed to several targets
`SUSPECTED` — the client's send-to-self behaviour was not confirmed end to end.
`interdrone.py:2091-2095`, `:2138-2142`

Both relays test `if message.drones_to_send_data != (self.flight_settings.current_drone_ID,)` —
an exact single-element tuple comparison. If the app addresses `SEND_GROUND_TRUTH_COORDS` to
`(1, 2)`, drone 1 does not recognise itself as a target and re-sends the message unchanged,
including to itself; on re-receipt the condition still holds, so it relays again. The `ARM_NACK`
case (`:1559-1564`) explicitly documents that a send-to-self loopback path exists, which is what
makes this plausible. Would present as an unthrottled relay storm.

**Direction.** Test membership (`current_drone_ID in message.drones_to_send_data`) and strip self
from the relay target list.

---

## P3 — Structure: 2219 lines, four responsibilities

`VERIFIED` by reading the whole file. The seams are already contiguous line ranges — nothing is
interleaved, which makes this a comparatively cheap split.

| Lines | Responsibility | Belongs in |
|---|---|---|
| 39–99 | console stdin pump, `get_input_or` | a `console_input.py`; unrelated to inter-drone comms |
| 109–145 | task-failure log helpers, `CMD_MSG` | shared utils or its own module (`CMD_MSG` is imported by `main.py`, `takeoff_impl`, tests) |
| 162–222 | thread spawn, socket bootstrap, blocking `resources_ready.get()` | transport |
| 261–1263 | ~1000 lines of `send_*` encoders, plus `gather_swarm_status` and `distribute_gps_offset` | protocol/encode — with those two split out again, since they are request/response state machines, not encoders |
| 1265–1454 | `cancel_state`, `restart_state_machine`, `interrupt_into`, `command_vehicle_mode`, `start_emergency_land`, `start_land` | **not messaging** — state-machine control that happens to be reachable from a message |
| 1456–1473 | `read`/`send`/`get_cmd_msg`/`set_cmd_msg` | transport adapter |
| 1480–2219 | the 703-line receive dispatch | per-message handlers (finding 8) |

Boundary violations worth naming:
- **Handlers reach three levels into flight state** — `:1597` `drone.vehicle.armed = False`,
  `:2115` `drone._vehicle.parameters[...]` (private), `:1931` `Pathfinder.instance` read from the
  receive task, `:1778` `drone.resetWaypoints(...)`, `:1736` `drone.checkForCollision(...)`. The
  `pending_*` staging attributes exist to prevent exactly this and are used for 4 of these; the
  other 5 bypass the pattern.
- **Transport leaks into protocol** — `:1495-1497` inlines
  `networking.try_get_server_message(timeout=0.02)` instead of calling its own `read()` at
  `:1456`, which consequently has zero callers and is dead.
- **`Pathfinder.instance` is read from a second concurrent task** — an accepted-but-documented gap
  per `multi_drone_sim_test.py:47-54`, which only exists because handlers touch flight state
  directly.

**Direction.** Extract cheapest-first, each independently safe: console input → `CMD_MSG` +
helpers → state-machine control (1265–1454) → handler table (finding 8) → transport adapter.
That leaves an `interdrone.py` of a few hundred lines.

---

## P4 — Superseded design and stale comments

### 18. The cross-pair point-A sync design is documented as superseded, but production still runs it
`VERIFIED` — `flight/pathfinder.py:447-470`, `:1615`; `calc_scan_path_impl.py:232`

`Pathfinder.set_cross_pair_home_boundary`'s docstring says it *replaces* the old
chase-the-other-pair retargeting, whose halving "had no lower bound" and whose target node "often
became unreachable in dense minefields — seen on 5/5 dense seeds". But
`set_cross_pair_home_boundary` is called only from
`flight/pathfinding/tests/droneWorkflowTest.py`, never from `state_machine/`. Meanwhile
`calc_scan_path_impl.py:232` calls `pf.retarget_approach_target(x, y)` on the real flight path,
fed by the live `CROSS_PAIR_POINT_A_SYNC` pipeline.

So the real two-pair mission runs the design that failed 5/5 dense seeds, and the verified fix
exists only in the simulation driver.

**Direction.** Wire `set_cross_pair_home_boundary` into `configureField` and delete the
point-A-sync pipeline. Deletion set:

- `MessageType.CROSS_PAIR_POINT_A_SYNC` + schema — `message_types.py:101`, `:595`
- `Interdrone.send_cross_pair_point_a_sync` `:894`, and its receive case `:2001`
- `Drone.pending_point_a_sync` `drone.py:346`, `Drone.last_synced_point_a` `:369`
- `_apply_point_a_sync` and `_send_point_a_sync_if_changed` in `calc_scan_path_impl.py`, plus
  their four call sites (`:280`, `:297`, `:317`, `:349`)

### 19. Comments describing the superseded design as live
`VERIFIED`. `message_types.py:89-93` and `:587-594` (the latter points readers at
`droneWorkflowTest.py`'s `_sync_approach_target` as the reference implementation);
`interdrone.py:898-905` and `:2002-2011`; `drone.py:338-346` and `:363-369`;
`calc_scan_path_impl.py:218-232` and `:235-241`.

---

## Smaller verified items

- `interdrone.py:565` — `send_segment_b_waypoints(waypoints: list[tuple[int, int]])` is annotated
  `int` while the schema (`message_types.py:509-513`) and every caller use `float`. This is the
  same annotation bug already fixed in the schema after it caused silent empty-payload sends; the
  helper signature was missed.
- `interdrone.py:1791-1795` — the `REACHED_WAYPOINT` match loop keeps scanning after a match (so
  it binds the *last*), then calls `list.remove()`, which removes the *first* equal element. Two
  equal-comparing waypoints means the wrong object is removed and `last_reached_waypoint` is set
  from the wrong one.
- `interdrone.py:1456` — `read()` is dead; duplicates the inlined call at `:1496`.
- `interdrone.py:2158`, `:2195` — explicit `pass` cases documenting "consumed from the queue
  elsewhere", while `PING_ACK`/`PING_NACK` work the same way but have no case at all, making them
  indistinguishable from the `PING` bug that was already found and fixed.
- `interdrone.py:434-437` — `all_takeoff()` returns `False` for an empty `drone_states`, while
  `all_armed` (`:393`), `all_demo_start` (`:475`) and `all_mission_start` (`:523`) all return
  `True`. `all_mission_start`'s docstring (`:516-522`) records that returning `False` there caused
  a confirmed infinite hang in the one-drone mission. `all_takeoff` still has that shape;
  `takeoff_impl.py:163` guards it with `drone.id == 1`, so it may be unreachable today, but it is
  the same latent hang one config change away.
