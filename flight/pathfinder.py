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
    def __init__(self, field_size: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]], mine_radius:float, sim_width:float, corner_coords: tuple[tuple[float,float]], overlap: float, altitude: float, fovDeg: float): 
        
        self.field_size: tuple[int, int] = field_size
        
        self.mine_radius = mine_radius

        self.corner_coords = corner_coords
        
        self.sim_width = sim_width 
        
        self.seen_tracker = seen_by_drone.SightTracker(field_size)
        
        self.arbCoord = SimToLatLonTransformer(corner_coords, sim_width)
        
        self.field = Field(0,field_size[0],0,field_size[1])
        
        self.bestNodeList = []
        self.bestWayPointslatlon = [] #stores best path
        self.bestWayPointslocal = []
                
        self.bestPath = Path()
        self.overlap = overlap
        self.altitude = altitude
        self.fovDeg = fovDeg
        
    
    def addDiscoveredMine(self, mine_lat:float, mine_lon: float): 
        x, y = self.arbCoord.latlon_to_local(mine_lat, mine_lon)
        self.field.addMine(x, y, self.mine_radius) 
        
        
    def addDiscoveredMines(self,discovered_mines_latlon: list[tuple[float, float]]): 
        for (lat, lon) in discovered_mines_latlon:
            self.addDiscoveredMines(lat, lon)
        
        
    def acceptCornerCoord(self, corner_coords_latlon: tuple[tuple[float,float]]):
        local_corners = []
        for (lat, lon) in corner_coords_latlon:
            x, y = self.arbCoord.latlon_to_local(lat, lon)
            local_corners.append(int(x)) #local coords of image corners
            local_corners.append(int(y))
            
        self.seen_tracker.note_pic(local_corners) 
        
    #returns final goto list    
    def getWayPointslatlon(self):
        
        #what coords do I give here
        start = self.field.placeStartNode()
        end = self.field.placeEndNodes()
        
        newGraph=Graph(self.field.nodeGraph)        
        self.bestNodeList = newGraph.shortest_path(start,end)
        
        self.bestWayPointslocal = self.bestPath.generate_goto_points(self.bestNodeList, self.overlap, self.altitude, self.fovDeg)
        
        for (x, y) in self.bestWayPointslocal:
            lat, lon = self.arbCoord.local_to_latlon(x, y) 
            self.bestWayPointslatlon.append((lat, lon)) 
            
        return self.bestWayPointslatlon
        
    
        
        