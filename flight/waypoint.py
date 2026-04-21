from typing import NamedTuple
from math import radians, cos, sin, asin, sqrt

class WaypointGroups(NamedTuple):
    drone1_waypoints: tuple[Waypoint, Waypoint]
    drone2_waypoints: tuple[Waypoint, Waypoint]
    
class Line:
    def __init__(self, start: Waypoint, end: Waypoint):
        self.start = start
        self.end = end
        self.x_length = self._calculate_x_length()
        self.y_length = self._calculate_y_length()
        
    def _calculate_x_length(self) -> float:
        return self.end._get_longitude() - self.start._get_longitude()
    
    def _calculate_y_length(self) -> float:
        return self.end._get_latitude() - self.start._get_latitude()
    
    def get_start(self) -> float:
        return self.start
    
    def get_end(self) -> float:
        return self.end
    
    def get_x_length(self) -> float:
        return self.x_length

    def get_y_length(self) -> float:
        return self.y_length

class Waypoint:
    def __init__(self, drone_id: int, lat: float, long: float):
        self.drone_id = drone_id
        self.lat = lat
        self.long = long

        self.has_visited = False
        self.has_to_wait = False
        self.waypoints_to_reach: list[Waypoint] = []

    def _get_drone_id(self) -> int:
        return self.drone_id

    def _get_latitude(self) -> float:
        return self.lat

    def _get_longitude(self) -> float:
        return self.long

    def _get_has_visited(self) -> bool:
        return self.has_visited

    def _get_has_to_wait(self) -> bool:
        return self.has_to_wait

    def _get_waypoints_to_reach(self) -> list[Waypoint]:
        return self.waypoints_to_reach

    def _set_latitude(self, new_lat: float):
        self.lat = new_lat

    def _set_longitude(self, new_long: float):
        self.long = new_long

    def _set_has_visited(self, new_has_visited: bool):
        self.has_visited = new_has_visited

    def _set_has_to_wait(self, new_has_to_wait: bool):
        self.has_to_wait = new_has_to_wait

    def _add_waypoints_to_reach(self, new_waypoint: Waypoint):
        self.waypoints_to_reach.append(new_waypoint)

    def check_wait(self) -> None:
        needs_to_wait: bool = False
        
        for waypoint in self.waypoints_to_reach:
            if not waypoint._get_has_visited():
                needs_to_wait = True
                break
            
        self._set_has_to_wait(needs_to_wait)

    def check_for_collision(self, drone1_waypoints: list[Waypoint], drone2_waypoints: list[Waypoint]) -> list[WaypointGroups]:
        drone1_lines = [Line(drone1_waypoints[i], drone1_waypoints[i + 1]) for i in range(len(drone1_waypoints) - 1)]
        drone2_lines = [Line(drone2_waypoints[i], drone2_waypoints[i + 1]) for i in range(len(drone2_waypoints) - 1)]
        
        waypoint_groups: list[WaypointGroups] = []
        
        for i in range(len(drone1_waypoints) - 1):
            for j in range(len(drone2_waypoints) - 1):
                s1_x: Waypoint = drone1_lines[i].get_x_length()
                s1_y: Waypoint = drone1_waypoints[i].get_y_length()
                s2_x: Waypoint = drone2_lines[j].get_x_length()
                s2_y: Waypoint = drone2_waypoints[j].get_y_length()
                
                denom: float = (s1_x * s2_y) - (s2_x * s1_y)
                
                if denom == 0:
                    continue # Lines are parallel, no collision
                
                denom_is_positive: bool = denom > 0
                
                s02_x: Waypoint = drone1_waypoints[i].get_x_length() - drone2_waypoints[j].get_x_length()
                s02_y: Waypoint = drone1_waypoints[i].get_y_length() - drone2_waypoints[j].get_y_length()
                
                s_numer: float = (s1_x * s02_y) - (s1_y * s02_x)
                
                if (s_numer < 0) == denom_is_positive:
                    continue # No collision
                
                t_numer: float = (s2_x * s02_y) - (s2_y * s02_x)
                
                if (t_numer < 0) == denom_is_positive:
                    continue # No collision
                
                if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive:
                    continue # No collision
                
                # Collision detected
                waypoint_groups.append(WaypointGroups((drone1_waypoints[i], drone1_waypoints[i + 1]), (drone2_waypoints[j], drone2_waypoints[j + 1])))
        return waypoint_groups
