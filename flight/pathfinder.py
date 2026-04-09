# This class needs to initialize and store the following as member variables:
# Arbitrary coordinate code object
# Field object
# Best path object
# Sight Tracker object

# As a black box the object basically only needs the following functions + whatever private functions are needed to make the following work:
# Accept mission's current corner coordinates during its initialization.
# Accept corner coordinates of any image taken and update "already seen" mat.
# Add discovered mines to field given their lat/long position
# Return lat/long waypoints of waypoints that need to be visited.

# The class needs a static variable used to store an instance of itself for use in the state machine.

# .. and the constants for arbitrary things like end node density etc.
from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer
from flight.pathfinding.path_subdivision import Path
from flight.pathfinding.node_generation import Field
from flight.pathfinding.path_calculation import Graph
import flight.pathfinding.utils.seen_by_drone as seen_by_drone

simWidth = 100

class Pathfinder:
    def __init__(self, field_size: tuple[tuple[float,float]], mine_radius:float, sim_width:float, corner_coords: tuple[tuple[float,float]], overlap: float, altitude: float, fov_deg: float): 
        
        self.field_size = field_size
        
        self.mine_radius = mine_radius

        self.corner_coords = corner_coords
        
        self.sim_width = sim_width 
        
        self.seen_tracker = seen_by_drone.SightTracker(field_size)
        
        self.arb_coord = SimToLatLonTransformer(corner_coords, sim_width)
        
        self.field = Field(0,field_size[0],0,field_size[1])
        
        self.best_node_List = []
        self.best_way_points_latlon = [] #stores best path
        self.best_way_points_local = []
                
        self.best_path = Path()
        self.overlap = overlap
        self.altitude = altitude
        self.fov_deg = fov_deg
        
    
    def add_discovered_mine(self, mine_lat:float, mine_lon: float): 
        x, y = self.arb_coord.latlon_to_local(mine_lat, mine_lon)
        self.field.addMine(x, y, self.mine_radius) 
        
        
    def add_discovered_mines(self,discovered_mines_latlon: list[tuple[float, float]]): 
        for (lat, lon) in discovered_mines_latlon:
            self.add_discovered_mine(lat, lon)
        
        
    def accept_field_corner_coord(self, corner_coords_latlon:tuple[tuple[float,float]]):
        local_corners = []
        for (lat, lon) in corner_coords_latlon:
            x, y = self.arb_coord.latlon_to_local(lat, lon)
            

    def accept_image_corner_coord(self, corner_coords_latlon:tuple[tuple[float,float]]):
        local_corners = []
        for [lat, lon] in corner_coords_latlon:
            x, y = self.arb_coord.latlon_to_local(lat, lon)
            local_corners.append([x, y])
        
    #returns final goto list    
    def get_way_points_latlon(self):
        
        #what coords do I give here
        start = self.field.placeStartNode()
        end = self.field.placeEndNodes()
        
        newGraph=Graph(self.field.nodeGraph)        
        self.best_node_list = newGraph.shortest_path(start,end)
        
        self.best_way_points_local = self.best_path.generate_goto_points(self.best_node_list, self.overlap, self.altitude, self.fovDeg)
        
        for (x, y) in self.best_way_points_local:
            lat, lon = self.arb_coord.local_to_latlon(x, y) 
            self.best_way_points_latlon.append((lat, lon)) 
            
        return self.best_way_points_latlon
        
    
        
        