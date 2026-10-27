"""Joining the two sides' routes into one entry-to-entry route (3-4 drone missions)."""

import math

from flight.pathfinding.pathnode.config import MAX_BRIDGE_FT, ROUTE_MATCH_FT
from flight.pathfinding.pathnode.polyline import on_segment


def join_routes(own, partner):
    """
    Takes: own and partner routes, each from its own entry to where it stopped. Each side
    stops at the other's point A, a point on the other's route, so they share at least one
    point -- on a partner vertex or anywhere on a partner segment.

    Returns: the joined route, spliced at the first shared point along `own`; None if they
    share no point.
    """
    for i, p in enumerate(own):
        for j in range(len(partner)):
            if math.dist(p, partner[j]) <= ROUTE_MATCH_FT:
                return list(own[:i + 1]) + list(partner[:j])[::-1]
            if j + 1 < len(partner) and on_segment(p, partner[j], partner[j + 1], ROUTE_MATCH_FT):
                return list(own[:i + 1]) + list(partner[:j + 1])[::-1]
    return None


def join(own, partner, routes, coverage, log):
    """
    join_routes, with a fallback. Under heavy relay loss both sides can stop against point
    As the other has since abandoned, leaving two checked halves that overlap but share no
    point. Then bridge the closest pair of points -- only with a straight line clear of every
    obstacle AND whose squares are all photographed, so the bridge is as verified as any
    checked link.

    Returns: the joined route, or None.
    """
    joined = join_routes(own, partner)
    if joined is not None:
        return joined
    pairs = sorted((math.dist(p, q), i, j) for i, p in enumerate(own)
                   for j, q in enumerate(partner) if math.dist(p, q) <= MAX_BRIDGE_FT)
    for _d, i, j in pairs:
        p, q = own[i], partner[j]
        if not routes.crosses([p, q]) and coverage.all_seen([p, q]):
            log.event("bridged-join", p, q)
            return list(own[:i + 1]) + list(partner[:j + 1])[::-1]
    return None
