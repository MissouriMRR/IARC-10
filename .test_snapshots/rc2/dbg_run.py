"""Debug driver: harness + state trace to dbg_trace.txt (drone, t, state, next, wp)."""
import functools, sys, time
import integration_tests.multi_drone_sim_test as h
from state_machine.states.state import State

CNT = {}
T0 = time.monotonic()
OUT = open("dbg_trace.txt", "w")


def subs(c):
    for s in c.__subclasses__():
        yield s
        yield from subs(s)


PL = {}


def dump_stuck(d, wp, pl):
    P = pl.planner
    OUT.write('  LASTPLAN ' + str({k: [(round(a,2), round(b,2), g) for a, b, g in v] for k, v in P.coordinator.last_plan.items()}) + ' heads=' + str(P.claims.heads) + chr(10))
    G = P.work.groups()
    OUT.write('  GROUPS ' + str([[(str(k)[:40], [(round(a, 2), round(b, 2)) for a, b in sq][:4]) for k, sq in g][:4] for g in G]) + chr(10))
    c0, r0 = P.coverage.seen.real_to_cell(x, y)
    OUT.write('  NEAR unseen ' + str([(c, r) for c in range(c0 - 6, c0 + 7) for r in range(r0 - 5, r0 + 6) if not P.coverage.seen.get(c, r)]) + ' cell=' + str((c0, r0)) + chr(10))
    for k, q in sorted(PL.items()):
        sq = [(c, r) for c in (19, 20) for r in range(146, 151)]
        OUT.write('  PL' + str(k) + ' seen=' + str([int(bool(q.planner.coverage.seen.get(c, r))) for c, r in sq]) + ' needed=' + str(sorted(q.planner.work.needed_squares())[:8]) + ' mines=' + str(sorted(q.planner.discovered)) + ' route0=' + str(q.planner.route()[:3]) + ' met=' + str(q.planner.met) + chr(10))
    x, y = pl.latlon_to_local(wp.lat, wp.long)
    cs = d.last_photo_corners_latlon
    OUT.write(f"STUCK wp local=({x:.3f},{y:.3f}) seen={pl.cell_seen_at(wp.lat, wp.long)}\n")
    OUT.write(f"  corners={[tuple(round(v, 3) for v in pl.latlon_to_local(*c)) for c in cs] if cs else None}\n")
    OUT.write(f"  job={pl.current_job(d.id)} misses={dict(P.coverage.misses)} blocked={P.blocked}\n")
    OUT.write(f"  needed={sorted(P.work.needed_squares())[:20]}\n")
    pos = d.side.positions()
    OUT.write(f"  positions={ {k: (tuple(round(v,2) for v in pl.latlon_to_local(*p)) if p else None) for k,p in pos.items()} } peer_jobs={d.peer_jobs} others={ {k: tuple(round(v,2) for v in pl.latlon_to_local(*p)) for k,p in d.side.other_side_positions().items()} } lost={d.side.mate_lost}\n")
    pass
    OUT.write("  queues=" + str(dbg_extra.queue_sizes()) + chr(10))
    OUT.write(f"  waypoints={[round(w.lat, 7) for w in d.waypoints][:5]}\n")
    OUT.flush()


def wrap(cls):
    orig = cls.run_callable if "run_callable" in cls.__dict__ else None
    if orig is None:
        return

    @functools.wraps(orig)
    async def w(self):
        nxt = await orig(self)
        d = self.drone
        wp = getattr(d, "last_reached_waypoint", None)
        try:
            nw = len(d.waypoints)
        except Exception:
            nw = -1
        pl = d.planner
        PL[d.id] = pl
        try:
            info = f"mines={len(pl.planner.discovered)} done={pl.done()} fail={pl.failure}"
        except Exception as e:
            info = repr(e)
        if cls.__name__ == "DroneShare" and wp is not None:
            key = (d.id, wp.lat, wp.long)
            CNT[key] = CNT.get(key, 0) + 1
            if CNT[key] in (150, 151):
                dump_stuck(d, wp, pl)
        OUT.write(f"{time.monotonic()-T0:8.2f} d{d.id} {cls.__name__:12s} -> {type(nxt).__name__:12s} wps={nw} lastwp={(wp.lat, wp.long) if wp else None} {info}\n")
        OUT.flush()
        return nxt
    cls.run_callable = w


import dbg_extra
for c in set(subs(State)):
    wrap(c)
h.main()
