"""Heartbeats: the messages this drone repeats on a timer.

Contract:
- Three streams, each with its own Resend timer (sent again on change, or when its period
  has passed, so a lost message heals):
  - job: everyone sends the job it holds to the mate (unless lost) and the other side,
    every JOB_HEARTBEAT_S. A lost "no job" cannot keep the mate waiting.
  - point A: the speaker only, to the other side, every POINT_A_HEARTBEAT_S or on change.
  - mine sync: the speaker only, the full mine list to the other side, every `mine_sync_s`.
- `send(mine_sync_s)`: checks the mate, announces the job, then (speaker, other side present,
  planner not failed) point A and mine sync. `announce_job()` is also called after a plan.
"""

import time

POINT_A_HEARTBEAT_S = 1.0      # point A is resent at least this often (a lossy stream)
JOB_HEARTBEAT_S = 1.0          # this drone's job is re-announced at least this often
MINE_SYNC_S = 5.0              # the speaker sends its full mine list this often


class Resend:
    """One stream's timer: due() when never sent, the value changed, or `period` has passed."""

    def __init__(self, period):
        self.period = period
        self._sent = None           # (value, time)

    def due(self, value, now, period=None):
        last = self._sent
        return (last is None or last[0] != value
                or now - last[1] >= (self.period if period is None else period))

    def mark(self, value, now):
        self._sent = (value, now)


class Heartbeats:
    def __init__(self, planner, interdrone, watch, me, mate, partners):
        """watch: this side's MateWatch. partners: the other side's drone ids."""
        self.planner = planner
        self.interdrone = interdrone
        self.watch = watch
        self.me = me
        self.mate = mate
        self.partners = partners
        self._job = Resend(JOB_HEARTBEAT_S)
        self._point_a = Resend(POINT_A_HEARTBEAT_S)
        self._mines = Resend(MINE_SYNC_S)

    async def send(self, mine_sync_s=MINE_SYNC_S):
        self.watch.check()
        await self.announce_job()
        if not self.partners or not self.watch.speaker or self.planner.failure:
            return
        now = time.monotonic()
        point_a = self.planner.point_a_latlon()
        if self._point_a.due(point_a, now):
            await self.interdrone.send_cross_pair_point_a(*point_a, self.partners)
            self._point_a.mark(point_a, now)
        if self._mines.due(None, now, mine_sync_s):
            await self.interdrone.send_cross_pair_mine_sync(self.planner.mines_latlon(),
                                                            self.partners)
            self._mines.mark(None, now)

    async def announce_job(self):
        """Tell the mate and the other side which job this drone holds."""
        to = ((() if self.mate is None or self.watch.lost else (self.mate,))
              + tuple(self.partners))
        if not to:
            return
        job = self.planner.current_job(self.me)
        now = time.monotonic()
        if self._job.due(job, now):
            await self.interdrone.send_side_job(job, to)
            self._job.mark(job, now)
