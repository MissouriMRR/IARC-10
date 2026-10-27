"""Harness with drone 1's Takeoff delayed by DELAY_S (startup-race probe)."""
import asyncio, os
import integration_tests.multi_drone_sim_test as h
from state_machine.states.takeoff import Takeoff

_orig = Takeoff.run_callable
DELAY = float(os.environ.get("DELAY_S", "0.5"))


async def slow(self):
    if self.drone.id == 1:
        await asyncio.sleep(DELAY)
    return await _orig(self)


Takeoff.run_callable = slow
h.main()
