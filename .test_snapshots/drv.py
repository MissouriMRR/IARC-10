import sys
import flight.pathfinding.pathnode.test_path_engine as t
from flight.pathfinding.pathnode.engines import ENGINES
name = sys.argv[1]
only = sys.argv[2:] 
bad = 0
for k, f in list(vars(t).items()):
    if k.startswith("test_") and k != "test_a_planner_built_with_the_field_engine_plans_a_pass" and (not only or any(o in k for o in only)):
        try:
            f(name); print("PASS", name, k, flush=True)
        except Exception as e:
            import traceback; bad += 1; print("FAIL", name, k, repr(e), flush=True); traceback.print_exc()
print("DONE", name, "fails", bad, flush=True)
