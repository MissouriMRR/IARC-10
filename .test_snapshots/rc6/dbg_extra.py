"""Extra tracing: gotoWaypoint start/end/raise and hold events, into dbg_trace.txt."""
import time
from state_machine.drone import Drone
import flight.flight_log as fl

OUT = open("dbg_goto.txt", "w")
T0 = time.monotonic()
_og = Drone.gotoWaypoint


async def _gw(self, *a, **k):
    me = self.currentPosition()
    wp = self.waypoints[0]
    try:
        mate = self.side.peers.side()
        mate = {k: (round(v[0], 7), round(v[1], 7)) if v else None for k, v in mate.items()}
        seen = self.side.peers.mate_unseen()
    except Exception as e:
        mate, seen = repr(e), None
    OUT.write(str(round(time.monotonic()-T0, 2)) + " d" + str(self.id) + " GOTO start from " + str(me) + " to " + str((wp.lat, wp.long)) + " believed=" + str(mate) + " unseen=" + str(seen) + " job=" + str(self.side.planner.current_job(self.id)) + chr(10))
    OUT.flush()
    try:
        r = await _og(self, *a, **k)
    except BaseException as e:
        OUT.write(f"{time.monotonic()-T0:8.2f} d{self.id} GOTO raised {e!r}\n")
        OUT.flush()
        raise
    OUT.write(f"{time.monotonic()-T0:8.2f} d{self.id} GOTO done\n")
    OUT.flush()
    return r


Drone.gotoWaypoint = _gw
_oe = fl.event


def _ev(kind, **f):
    if kind in ("hold_start", "hold_end", "hold_abort", "separation_breach"):
        OUT.write(f"{time.monotonic()-T0:8.2f} EVENT {kind} {str(f)[:400]}\n")
        OUT.flush()
    return _oe(kind, **f)


fl.event = _ev


import asyncio, threading, io
_ar = asyncio.run
LOOP = {}


def _run(coro, *a, **k):
    async def w():
        LOOP["l"] = asyncio.get_running_loop()
        return await coro
    return _ar(w(), *a, **k)


asyncio.run = _run


def _dump():
    time.sleep(20)
    loop = LOOP["l"]
    f = open("dbg_tasks.txt", "w", buffering=1)
    for rep in range(3):
      time.sleep(10)
      f.write('=== t=' + str(time.monotonic()-T0) + chr(10))
      for t in asyncio.all_tasks(loop):
          c = t.get_coro()
          chain = []
          while c is not None:
              fr = getattr(c, "cr_frame", None) or getattr(c, "gi_frame", None)
              if fr is None:
                  break
              sd = fr.f_locals.get("self")
              did = getattr(getattr(sd, "drone", None), "id", "")
              chain.append(fr.f_code.co_filename.split("rc1/")[-1].split("/")[-1] + ":" + str(fr.f_lineno) + ":" + fr.f_code.co_name + str(did))
              c = getattr(c, "cr_await", None) or getattr(c, "gi_yieldfrom", None)
          f.write(t.get_name() + " " + " > ".join(chain) + chr(10))
    f.close()


threading.Thread(target=_dump, daemon=True).start()


import integration_tests.multi_drone_sim_test as _h
ROUTERS = []
_oreg = _h._MessageRouter.register


def _reg(self, did):
    if self not in ROUTERS:
        ROUTERS.append(self)
    return _oreg(self, did)


_h._MessageRouter.register = _reg


def queue_sizes():
    return {k: q.qsize() for r in ROUTERS[-1:] for k, q in r._queues.items()}


from collections import Counter
SENT = Counter()
_orr = _h._MessageRouter.route


def _route(self, message):
    for t in message.drones_to_send_data:
        SENT[(message.sender_id, t, int(message.id) if not hasattr(message.id, "name") else message.id.name)] += 1
        if self._queues.get(t) is None:
            OUT.write("DROPPED to " + str(t) + " " + str(message.id) + chr(10))
    return _orr(self, message)


_h._MessageRouter.route = _route


def _dump2():
    time.sleep(25)
    with open("dbg_sent.txt", "w") as f:
        for k, v in sorted(SENT.items(), key=str):
            f.write(str(k) + " " + str(v) + chr(10))


threading.Thread(target=_dump2, daemon=True).start()

from flight.pathfinding.side_agent import SideAgent
_oai = SideAgent.apply_inbox
APPLIED = Counter()


def _ai(self):
    APPLIED[("photos", self.me)] += len(self.drone.pending_photo_reports)
    APPLIED[("mines", self.me)] += len(self.drone.pending_cross_pair_mines)
    APPLIED[("cov", self.me)] += len(self.drone.pending_partner_coverage)
    return _oai(self)


SideAgent.apply_inbox = _ai


def _dump3():
    time.sleep(25)
    with open("dbg_applied.txt", "w") as f:
        for k, v in sorted(APPLIED.items(), key=str):
            f.write(str(k) + " " + str(v) + chr(10))


threading.Thread(target=_dump3, daemon=True).start()


GOT = Counter()
_otg = _h._FakeNetworking.try_get_server_message


def _tg(self, timeout=0.0):
    m = _otg(self, timeout)
    if m is not None:
        GOT[(id(self) % 1000, m.sender_id, m.id.name)] += 1
    return m


_h._FakeNetworking.try_get_server_message = _tg


def _dump4():
    time.sleep(25)
    with open("dbg_got.txt", "w") as f:
        for k, v in sorted(GOT.items(), key=str):
            f.write(str(k) + " " + str(v) + chr(10))


threading.Thread(target=_dump4, daemon=True).start()


_oin = _h._FakeNetworking.__init__


def _in(self, drone_id, router):
    self.dbg_id = drone_id
    return _oin(self, drone_id, router)


_h._FakeNetworking.__init__ = _in


def _tg2(self, timeout=0.0):
    m = _otg(self, timeout)
    if m is not None:
        GOT[(self.dbg_id, m.sender_id, m.id.name)] += 1
    return m


_h._FakeNetworking.try_get_server_message = _tg2


def _dump5():
    time.sleep(25)
    with open("dbg_pending.txt", "w") as f:
        for r in ROUTERS[-1:]:
            for k, q in r._queues.items():
                c = Counter((m.sender_id, m.id.name) for m in list(q._queue))
                f.write("inbox " + str(k) + " " + str(q.qsize()) + " " + str(sorted(c.items(), key=str)) + chr(10))
                if k == 4:
                    f.write("  first: " + str([(m.sender_id, m.id.name) for m in list(q._queue)[:12]]) + chr(10))


threading.Thread(target=_dump5, daemon=True).start()
