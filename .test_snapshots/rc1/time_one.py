import sys, time
from flight.pathfinding.pathnode.minefield import generate_true_minefield
from flight.pathfinding.pathnode.sim import Mission
t = time.time()
m = Mission(generate_true_minefield(90, 3), drones=4).run()
print("4 drones seed 3:", time.time() - t, m.failure)
