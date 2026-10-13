# The simulator as a test tool

File: `flight/pathfinding/pathnode/sim.py`. It runs a whole mission (1-4 drones) through the
real `PathNodePlanner` with no flight code, no cameras and no sockets. It is fast (seconds to
a minute), so it is what the sweeps, golden files and benchmark are built on.

Not simulated: real flight, battery, wind, the state machine. For those use the
[flight harness](flight-harness.md).

## Run one mission

```python
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.sim import Mission, evaluate

m = Mission(generate_true_minefield(20, 3), drones=2).run()
print(m.summary())
print(evaluate(m))
```

Run it from the repo root with `PYTHONPATH=<repo>` (Git Bash: `PYTHONPATH=$PWD python script.py`).
Real output for that snippet:

```
{'makespan': 1022.3, 'photos': 393, 'ticks': 198, 'failure': None, 'done': True,
 'per_drone': {1: (197, 961), 2: (196, 1022)}, 'group_photos': {...}, 'sent': 787,
 'dropped': 0, 'jumps': 1, 'nodes_in_mine': 5, 'orphans': 0, 'bridges': 2, 'replans': 0,
 'entry_shifts': 0, 'shortcuts': 0, ..., 'blocked_squares': 0, 'reopens': 0}
{'complete': True, 'joined': True, 'route_mine_squares': 0, 'spans_field': True,
 'bad_edges_known': 0, 'bad_edges_hidden': 0, 'unseen_cells': 0, 'route_cells_in_danger': 0}
```

## Mission(...) options

| Option | Meaning |
|---|---|
| `true_mines` (first arg) | list of (x, y) in feet. Make one with `generate_true_minefield(n, seed)` (`pathnode/minefield.py`): n mines, uniform in the 80 x 300 ft field, 6 ft margin. Same seed = same field. |
| `drones` | 1-4. 1: one drone; 2: one side of two; 3: side of two + one alone on the far edge; 4: two sides of two. Two-sided missions must meet in the middle and join their routes. |
| `drop` | fraction (0-1) of relayed messages thrown away. |
| `delay` | messages arrive up to this many ticks late. `drop`/`delay` use `seed` for their random numbers. Give both e.g. `drop=0.5, delay=3, seed=100`. |
| `photo_yaw_deg` | rotate every photo footprint by this many degrees, or `"random"` for a random heading per photo (the real footprint turns with the drone). |
| `coverage` | `"corridor"` (default) or `"band"`. |
| `shape_size_ft` | photo footprint (along, across) in ft; default (6, 4) is the planning footprint. |
| `outer_ring`, `adjacent_ring` | how many extra rings of squares get photographed around the route (default 2 and 1). `outer_ring=0` can leave route squares "in danger" (see below). |
| `max_ticks` | safety cap (default 4000); hitting it sets `failure = "hit tick cap"`. Raise it for slow cases, lower it to fail fast. |
| `record_frames` | keep a snapshot per tick so a GIF can be drawn. Costs memory; leave off in sweeps. |

`m.failure` is `None` on success, or `"stalled"`, `"hit tick cap"` or `"drone N: <planner failure>"`.

## evaluate(m): what each metric means

`evaluate` checks drone 1's reported route against the TRUE minefield, independently of what
the planner believes.

| Metric | Good value | Meaning |
|---|---|---|
| `complete` | True | every planner finished and no failure |
| `joined` | True | a route exists (two-sided: both halves joined) |
| `spans_field` | True | route starts on one field edge (y = 0) and ends on the other (y = field height) |
| `unseen_cells` | 0 | route cells that no drone ever photographed |
| `bad_edges_known` | 0 | route segments crossing a mine zone the drones KNOW about |
| `route_mine_squares` | 0 | true mines whose own square the route runs over. This is the safety line. |
| `bad_edges_hidden` | informational | crossings of mines nobody found. Allowed: the route may pass near an unfound mine, never over its square. |
| `route_cells_in_danger` | 0 with default rings | route cells inside any true mine's danger squares. `outer_ring=0` can give 1+. |

A run is "BAD" (in `tools/testing/sweep.py`) if `failure` is set, or `complete`/`spans_field`
is false, or `unseen_cells`, `bad_edges_known` or `route_mine_squares` is non-zero.

## summary(m)

`makespan` (longest single-drone flight, ft), `photos` (total), `ticks`, `per_drone`
({id: (photos, feet flown)}), `sent`/`dropped` (relay messages), and planner counters
(`jumps`, `orphans`, `bridges`, `replans`, `shortcuts`, `blocked_squares`, `reopens`, ...).
Use `makespan` and `photos` to judge efficiency; the counters to see which repair rules fired.

## Make a GIF

```python
from flight.pathfinding.pathnode.render import render_mission_gif
m = Mission(generate_true_minefield(20, 3), drones=2, record_frames=True).run()
render_mission_gif(m, "mission.gif")          # optional: max_frames=300, frame_duration_ms=160
```

Needs `matplotlib` and Pillow. It shows photographed ground, mine zones, each side's chain,
each claimed waypoint by drone, and shortcuts being tried. It refuses to run without
`record_frames=True`. A 2-drone, 20-mine mission renders to a ~340 KB GIF in seconds.
Watching one is the fastest way to understand a failing seed.

## Tools built on it

`tools/testing/`: `sweep.py`, `golden.py`, `repro_cases.py`, `benchmark.py`. See [tools.md](tools.md).
