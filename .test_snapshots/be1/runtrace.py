import functools, os, runpy, sys
sys.argv = ["multi_drone_sim_test", "2"]
from flight.pathfinding import mission_planner as mp

LOG = open(os.environ["TRACE_OUT"], "w")
SKIP = {"latlon_to_local", "local_to_latlon"}


def wrap(name, fn):
    @functools.wraps(fn)
    def inner(self, *a, **k):
        r = fn(self, *a, **k)
        LOG.write(f"{name} {a!r} {k!r} -> {r!r}\n")
        LOG.flush()
        return r
    return inner


for n, f in list(vars(mp.MissionPlanner).items()):
    if callable(f) and not n.startswith("_") and n not in SKIP:
        setattr(mp.MissionPlanner, n, wrap(n, f))
runpy.run_module("integration_tests.multi_drone_sim_test", run_name="__main__")
