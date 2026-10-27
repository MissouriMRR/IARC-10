"""Small helpers on polylines -- lists of (x, y) points in local feet."""

import math

from flight.pathfinding.pathnode.config import SAMPLE_FT


def dedupe(poly):
    """poly without consecutive repeated points."""
    out = [poly[0]]
    for p in poly[1:]:
        if math.dist(p, out[-1]) > 1e-9:
            out.append(p)
    return out


def poly_len(poly):
    return sum(math.dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def as_key(poly):
    """A hashable copy of poly (float tuples), for cache keys."""
    return tuple((float(x), float(y)) for x, y in poly)


def arc_position(poly, p):
    """Arc length along poly of the point on it nearest p."""
    best_d, best_t, acc = math.inf, 0.0, 0.0
    for i in range(len(poly) - 1):
        a, b = poly[i], poly[i + 1]
        seg = math.dist(a, b)
        if seg == 0:
            continue
        t = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / seg ** 2
        t = min(max(t, 0.0), 1.0)
        d = math.dist((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), p)
        if d < best_d:
            best_d, best_t = d, acc + t * seg
        acc += seg
    return best_t


def samples(poly):
    """Points every SAMPLE_FT along poly, each tagged with the segment it lies on."""
    out = [(poly[0], 0)]
    for i in range(len(poly) - 1):
        a, b = poly[i], poly[i + 1]
        k = max(1, math.ceil(math.dist(a, b) / SAMPLE_FT))
        for j in range(1, k + 1):
            t = j / k
            out.append(((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), i))
    return out


def on_segment(p, a, b, tol):
    """Whether p lies within tol of segment a-b."""
    seg = math.dist(a, b)
    if seg == 0:
        return False
    t = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / seg ** 2
    if not 0.0 <= t <= 1.0:
        return False
    return math.dist(p, (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))) <= tol
