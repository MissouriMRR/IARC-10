"""Ground-truth minefields for simulation.

Deterministic true-minefield generator for the sim.
  - Seeded RNG: the same (n, seed) always gives the same field.
  - 6 ft margin from every field edge.
"""
import random

from flight.pathfinding.pathnode.geometry import FIELD_H, FIELD_W


def generate_true_minefield(n=70, seed=909090):
    rng = random.Random(seed)
    return [(rng.uniform(6.0, FIELD_W - 6.0), rng.uniform(6.0, FIELD_H - 6.0)) for _ in range(n)]
