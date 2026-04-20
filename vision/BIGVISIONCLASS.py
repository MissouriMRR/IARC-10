from camera/camera.py import Camera
from camera/mine_detection.py import MineDetection
from camera/image.py import Image
from math import dist
import dronekit
import common/drone_coordinates.py

class Vision:
    def __init__(self, config: dict):
        self.mine_list: list[Mine] = []
        self.image_list: list[Image] = [] 
        self.camera: Camera = Camera(config)
        self.camera.start_camera()
        self.drone: Drone = dronekit.connect(config["droneAddress"], wait_ready = True, baud = config["baudRate"])
    def _cluster(self, mine_to_check: MineDetection, cluster_threshold: float) -> None:
        """
        goes through the mine list and deletes the most recent entry
        if the a mine is found with world coordinates within some
        threshold euclidean distance
        """
        # this will assume mine coordinates are in lat/lon FOR NOW
        # they are not in this format, they are still just boxes
        for comparison_mine in range(len(self.mine_list) - 2)
            if dist(mine_to_check.world_coords, comparison_mine.world_coords) <= cluster_threshold:
                self.mine_list.remove(mine_to_check)
                break

    def big_ass_picture_function(self) -> Image:
        drone_position = drone_coordinates.DronePose(self.drone.location.lat, 
                                                     self.drone.location.lon,
                                                     self.drone.location.alt,
                                                     self.drone.attitude.yaw,
                                                     self.drone.attitude.pitch,
                                                     self.drone.attitude.roll)
        corner_coords = pixel_to_geocoord_gimbal(0, 0, *config["main_res"], 1.41372, 0.797528202,
                                                drone_position, drone_coordinates.GimbalPose(0, -90, 0))
        detected_mines = camera.capture(False, False, False, drone_position) # take picture
        if detected_mines is None:
            return Image(None, corner_coords)
        for mine in detected_mines: # add mine to mine_list if mine(s) found
            self.mine_list.append(Mine(detected_mines), pixel_to_geocord_gimbal(*detected_mines))
            self._cluster(mine) # cluster mine list
        # create Image with newly found mines and return
        return Image(self.mine_list[-1:len(detected_mines) * -1 - 1], corner_coords) 
