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
from flight.pathfinding.node_generation.node_generation import Field
from flight.pathfinding.path_calculation import Graph
import flight.pathfinding.utils.seen_by_drone as seen_by_drone

simWidth = 100

class Pathfinder:
    def __init__(self, real_corner_coords: tuple[tuple[float,float]], altitude: float, fov_deg: float): 
        
        self.SIM_WIDTH:float = 300 # Confirm with nat what this is exactly, this should be an internal constant

        self.OVERLAP = 3 # This will be an internal constant

        self.coord_converter = SimToLatLonTransformer(real_corner_coords, self.SIM_WIDTH)

        self.arb_corner_coords = self.coord_converter.get_arb_corners()

        self.arb_field_size = [max([self.arb_corner_coords[0][0], self.arb_corner_coords[1][0], self.arb_corner_coords[2][0], self.arb_corner_coords[3][0]]) - min([self.arb_corner_coords[0][0], self.arb_corner_coords[1][0], self.arb_corner_coords[2][0], self.arb_corner_coords[3][0]]), max([self.arb_corner_coords[0][1], self.arb_corner_coords[1][1], self.arb_corner_coords[2][1], self.arb_corner_coords[3][1]]) - min([self.arb_corner_coords[0][1], self.arb_corner_coords[1][1], self.arb_corner_coords[2][1], self.arb_corner_coords[3][1]])]
        
        self.mine_radius = 3 # Get from the coord converter

        self.real_corner_coords = real_corner_coords 
        
        self.field = Field(0,self.arb_field_size[0],0,self.arb_field_size[1])

        self.seen_tracker = seen_by_drone.SightTracker(self.arb_field_size)

        self.seen_tracker.note_field_borders(self.arb_corner_coords)
        
        self.best_node_List = []
        self.best_way_points_latlon = [] #stores best path
        self.best_way_points_local = []
                
        self.best_path = Path()
        self.altitude = altitude
        self.fov_deg = fov_deg
        
    
    def add_discovered_mine(self, mine_lat:float, mine_lon: float): 
        x, y = self.coord_converter.latlon_to_local(mine_lat, mine_lon)
        self.field.addMine(x, y, self.mine_radius) 
        
        
    def add_discovered_mines(self,discovered_mines_latlon: list[tuple[float, float]]): 
        for (lat, lon) in discovered_mines_latlon:
            self.add_discovered_mine(lat, lon)
            

    def accept_image_corner_coord(self, corner_coords_latlon:tuple[tuple[float,float]]):
        local_corners = []
        for [lat, lon] in corner_coords_latlon:
            x, y = self.coord_converter.latlon_to_local(lat, lon)
            local_corners.append([x, y])
        self.seen_tracker.note_pic(local_corners)


    def increase_radius(self, mine_radius_increment):
        self.mine_radius += mine_radius_increment
        self.field.increaseRadius(mine_radius_increment)

    #returns final goto list    
    def get_way_points_latlon(self):
        
        #what coords do I give here
        start = self.field.placeStartNode()
        end = self.field.placeEndNodes()
        
        newGraph=Graph(self.field.nodeGraph)        
        self.best_node_list = newGraph.shortest_path(start,end)
        
        self.best_way_points_local, best_wp_seg_info = self.best_path.generate_goto_points(self.best_node_list, self.OVERLAP, self.altitude, self.fov_deg)

        self.best_way_points_local = seen_by_drone.remove_extra_coords(self.seen_tracker, self.best_way_points_local, best_wp_seg_info, [self.best_path.ground_covered_image(self.altitude, self.fov_deg), self.best_path.ground_covered_image(self.altitude, self.fov_deg)])
        
        for (x, y) in self.best_way_points_local:
            lat, lon = self.coord_converter.local_to_latlon(x, y) 
            self.best_way_points_latlon.append((lat, lon)) 
            
        return self.best_way_points_latlon
        
    
        
        