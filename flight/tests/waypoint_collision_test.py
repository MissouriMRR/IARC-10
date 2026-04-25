import flight.waypoint

def test_waypoint_collision():
    # Create two waypoints for the same drone
    
    waypoint1 = flight.waypoint.Waypoint(drone_id=1, lat=37.96014600458061, long=-91.77907476733078)
    waypoint2 = flight.waypoint.Waypoint(drone_id=1, lat=37.96047105909782, long=-91.77868610084488)
    waypoint3 = flight.waypoint.Waypoint(drone_id=1, lat=37.96062284617695, long=-91.77875330019057)

    waypoint4 = flight.waypoint.Waypoint(drone_id=1, lat=37.96051974555359, long=-91.77907476733078)
    waypoint5 = flight.waypoint.Waypoint(drone_id=1, lat=37.96030638408169, long=-91.778522642977)

