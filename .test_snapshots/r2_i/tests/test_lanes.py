"""Keep-apart looks at the paths lower drones will fly, not just where they stand.

Found with the moving fake vehicle of the flight harness: the planner kept a higher drone
`keep_away_ft` from a lower drone's position and next photo, but its LEG could still cross the
lower drone's route a photo or two further on; with legs that take time both drones were on the
crossing together and aborted at the 1.2 m floor.
"""

from flight.pathfinding.pathnode.coordination import (
    SideCoordinator,
    _lane_gap,
    _polyline_gap,
    _segment_gap,
)

GAP = 10.0
CLEAR = 7.5
fly = SideCoordinator._flyable


def test_segment_gap():
    assert _segment_gap((0, 0), (10, 10), (0, 10), (10, 0)) == 0.0, "they cross"
    assert _segment_gap((0, 0), (10, 0), (0, 3), (10, 3)) == 3.0, "parallel"
    assert _segment_gap((0, 0), (10, 0), (13, 4), (20, 4)) == 5.0, "end to end"
    assert _lane_gap((0, 0), (10, 0), [(5, 2)]) == 2.0, "a lane of one point"
    assert _lane_gap((0, 0), (10, 0), [(5, 6), (5, 9), (20, 9)]) == 6.0
    assert _polyline_gap((0, 0), [(3, 4)]) == 5.0


def test_a_leg_across_a_lower_drones_route_is_not_flown_but_one_clear_of_it_is():
    route = ([(40.0, 300.0), (41.0, 301.0), (41.0, 291.7), (41.0, 280.0)], GAP)   # lower drone's lane
    queue = [(46.0, 285.9, 0)]
    # From the side, ending 5 ft from the lane's last leg, 15 ft from the photo before it: a check
    # against the lower drone's next photo only passes it, a lane check does not.
    assert fly((50.0, 300.0), queue, [], [], CLEAR, [route]) == []
    # Far enough to the east.
    assert fly((70.0, 300.0), [(60.0, 285.9, 0)], [], [], CLEAR, [route]) == [(60.0, 285.9, 0)]
    # A leg that crosses the lane further on, between the photos.
    assert fly((30.0, 285.0), [(52.0, 285.0, 0)], [], [], CLEAR, [route]) == []


def test_a_leg_that_only_leaves_a_lane_it_is_inside_is_fine_one_that_closes_in_is_not():
    lane = ([(40.0, 250.0), (40.0, 270.0)], GAP)
    inside = (46.0, 260.0)                       # 6 ft from the lane, inside GAP
    assert fly(inside, [(70.0, 260.0, 0)], [], [], CLEAR, [lane]), "moving away"
    assert fly(inside, [(42.0, 260.0, 0)], [], [], CLEAR, [lane]) == [], "moving in"
    assert fly(inside, [(46.0, 300.0, 0)], [], [], CLEAR, [lane]), "sliding along, no closer"


def test_no_lanes_changes_nothing():
    queue = [(51.0, 200.0, 0), (51.0, 190.0, 0)]
    assert fly((47.0, 10.0), queue, [], [], CLEAR) == queue
