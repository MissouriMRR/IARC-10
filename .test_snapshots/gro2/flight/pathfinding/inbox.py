"""InboxApplier: feeds what the interdrone receive task staged into the planner.

Contract:
- `apply()` takes everything staged since the last call and empties the staging fields on the
  drone (the receive task only stages; it never touches the planner).
- Fixed order, which matters:
  1. the other side's mines first (no path node may be laid over ground about to turn out
     unsafe),
  2. then the mate's photos (a failed one is a miss; others also tell PeerPositions where the
     mate is),
  3. then the other side's coverage,
  4. then its point A.
- Drone fields read: pending_cross_pair_mines, pending_photo_reports,
  pending_partner_coverage, pending_partner_point_a.
"""


class InboxApplier:
    def __init__(self, planner, drone, peers):
        """peers: the PeerPositions told about the mate's photos."""
        self.planner = planner
        self.drone = drone
        self.peers = peers

    def apply(self):
        planner, drone = self.planner, self.drone
        mines, drone.pending_cross_pair_mines = drone.pending_cross_pair_mines, []
        for report in mines:
            planner.partner_mine(report.lat, report.lon)
        photos, drone.pending_photo_reports = drone.pending_photo_reports, []
        for report in photos:
            if report.failed_at is not None:
                planner.failed_photo(*report.failed_at)
                continue
            planner.mate_photo(report.corner_coordinates, report.mines)
            self.peers.mate_photographed(report.corner_coordinates)
        coverage, drone.pending_partner_coverage = drone.pending_partner_coverage, []
        for corners in coverage:
            planner.partner_coverage(corners)
        point_a, drone.pending_partner_point_a = drone.pending_partner_point_a, None
        if point_a is not None:
            planner.partner_point_a(*point_a)
