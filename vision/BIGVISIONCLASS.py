from vision.common.image import Image
from common.mine_detection import MineDetection
from Cameras.baseCamera import BaseCamera
from math import sin, acos
import dronekit
from common.drone_coordinates import DronePose, GimbalPose, pixel_to_geocoord_gimbal
from vision.common.mine import Mine

# haversine
def hvs(theta):
    return sin(theta / 2) ** 2

# archaversine
def ahvs(theta):
    return acos(1 - 2 * theta)

# arc length between two points on a sphere given longitude and latitude
def great_circle(p_lon, p_lat, q_lon, q_lat):
    delta_lon = abs(p_lon - q_lon)
    delta_lat = abs(p_lat - q_lat)
    delta_sigma = ahvs(hvs(delta_lat) + (1 - hvs(delta_lat) - hvs(p_lat + q_lat)) * hvs(delta_lon))
    return 6378137 * delta_sigma # random number is radius of the earth in meters

"""
Big boy vision class used by the pathfinding algorithm 
"""

class Vision:
    def __init__(self, visionConfig, camera, drone):
        self.visionConfig = visionConfig
        self.mine_list: list[MineDetection] = []
        self.image_list: list[Image] = [] 
        self.camera: BaseCamera = camera #Camera must be initialized before being passed in
    """
    def __init__(self, config: dict):
        self.mine_list: list[Mine] = []
        self.image_list: list[Image] = [] 
        self.camera: Camera = Camera(config)
        self.camera.start_camera()
        self.drone: Drone = dronekit.connect(config["droneAddress"], wait_ready = True, baud = config["baudRate"])

        """

    def _cluster(self, mine_to_check: MineDetection, cluster_threshold: float) -> None:
        """
        goes through the mine list and deletes the most recent entry
        if the a mine is found with world coordinates within some
        threshold euclidean distance
        """
        # this will assume mine coordinates are in lat/lon FOR NOW
        # they are not in this format, they are still just boxes
        for comparison_mine in range(len(self.mine_list) - 2):
            if great_circle(mine_to_check.world_coords, comparison_mine.world_coords) <= cluster_threshold:
                self.mine_list.remove(mine_to_check)
                break


    def get_mine_location(self, dronePose: DronePose, mine: MineDetection) -> tuple[float, float]:
        # convert mine pixel coordinates to world coordinates using drone GPS and gimbal angle
        ground = pixel_to_geocoord_gimbal(
            px=mine.box[0] + (mine.box[2] - mine.box[0]) / 2, # x center of box
            py=mine.box[1] + (mine.box[3] - mine.box[1]) / 2, # y center of box
            image_width=mine.imageSize[0],
            image_height=mine.imageSize[1],
            h_fov=self.visionConfig["h_fov"],
            v_fov=self.visionConfig["v_fov"],
            drone=dronePose,
            gimbal=  GimbalPose(yaw=0, pitch=-90,  roll=0)
        )
        
    def big_picture_function(self) -> Image:
        location = [self.drone.location.global_frame.lat,self.drone.location.global_frame.lon, self.drone.location.global_frame.alt]
        pitch = self.drone.attitude.pitch
        yaw = self.drone.attitude.yaw
        roll = self.drone.attitude.roll
        drone_position = DronePose(location[0], location[1], location[2], yaw, pitch, roll)
        detections : list[MineDetection] = self.camera.capture_and_detect_mines() # take picture and get mine detections

        new_mines: list[Mine] = []

        for mine in detections: # add mine to mine_list if mine(s) found
            location = self.get_mine_location(drone_position, mine) # get world coordinates of mine (for now just do this for the first mine detected)

            new_mines.append(Mine(mine.confidence, location))
            #self._cluster(mine) # cluster mine list

        self.mine_list.extend(new_mines) # add new mines to mine list
        return new_mines
