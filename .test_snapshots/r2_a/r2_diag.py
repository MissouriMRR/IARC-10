"""Diagnostics wrapper: python r2_diag.py N [dense] [speedup=X]  (run from the snapshot dir)."""
import asyncio, sys, time, logging
from integration_tests import multi_drone_sim_test as m
from state_machine.drone import Drone

T0 = time.monotonic()
SPEED = [a for a in sys.argv[1:] if a.startswith("speedup=")]
if SPEED:
    m.MOTION_SPEEDUP = float(SPEED[0].split("=")[1])
    sys.argv = [a for a in sys.argv if not a.startswith("speedup=")]

LEGS = []
_goto = Drone.gotoWaypoint
def loc(d, p):
    if d.planner is None: return p
    x, y = d.planner.latlon_to_local(*p)
    return (round(x, 1), round(y, 1))
async def goto(self, *a, **k):
    wp = self.waypoints[0]
    LEGS.append([time.monotonic() - T0, self.id, loc(self, self.currentPosition()), loc(self, (wp.lat, wp.long)), None])
    rec = LEGS[-1]
    try:
        return await _goto(self, *a, **k)
    finally:
        rec[4] = time.monotonic() - T0
Drone.gotoWaypoint = goto
orig = Drone.checkSeparation
def check(self, peer_states):
    r = orig(self, peer_states)
    if self.formation_abort and not getattr(self, "_dbg", False):
        self._dbg = True
        me = self.currentPosition()
        print(f"[{time.monotonic()-T0:7.2f}] BREACH drone {self.id}: {self.formation_abort}", flush=True)
        print(f"   me={me} target={[ (w.lat,w.long) for w in self.waypoints[:1]]}", flush=True)
        print(f"   me local={loc(self, me)}", flush=True)
        for L in [l for l in LEGS if l[1] in (self.id, int(self.formation_abort.split("drone ")[1].split(" ")[0]))][-8:]:
            print(f"   leg t={L[0]:.2f}..{L[4]} drone {L[1]}: {L[2]} -> {L[3]}", flush=True)
        for s in peer_states:
            lp = s.live_position
            age = None if s._live_position_at is None else time.monotonic() - s._live_position_at
            print(f"   peer {s.drone_id}: live={lp} age={age}", flush=True)
    return r
Drone.checkSeparation = check

async def lag():
    while True:
        t = time.monotonic()
        await asyncio.sleep(0.05)
        d = time.monotonic() - t - 0.05
        if d > 0.25:
            print(f"[{t-T0:7.2f}] LOOP LAG {d:.2f}s", flush=True)

orig_run = m.run_simulation
async def run(*a, **k):
    t = asyncio.ensure_future(lag())
    try:
        return await orig_run(*a, **k)
    finally:
        t.cancel()
m.run_simulation = run
m.main()
