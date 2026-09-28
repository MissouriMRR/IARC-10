"""Ground-truth minefields for simulation.

Reproduces flight/pathfinding/tests/droneWorkflowTest.generate_true_minefield exactly (same
RNG, same draws, same 6ft margin) so the two planners can be compared on identical fields,
without this package importing the old test module.
"""
import random

from flight.pathfinding.pathnode.geometry import FIELD_H, FIELD_W


def generate_true_minefield(n=70, seed=909090):
    rng = random.Random(seed)
    return [(rng.uniform(6.0, FIELD_W - 6.0), rng.uniform(6.0, FIELD_H - 6.0)) for _ in range(n)]
