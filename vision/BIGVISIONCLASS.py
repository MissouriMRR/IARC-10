from ast import List
from importlib.resources import path

from vision.common.image import Image
from vision.common.detection import Detection
from Cameras.baseCamera import BaseCamera
from math import sin, acos
import dronekit
from common.drone_coordinates import DronePose, GimbalPose, pixel_to_geocoord_gimbal
from vision.common.mine import Mine
from enum import Enum   
import os
import datetime
from PIL import ImageDraw
import json

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

class DetectionType(Enum):
    MINE = 1
    APRILTAG = 2
class Vision:
    def __init__(self, visionConfig, camera, drone):
        self.visionConfig = visionConfig
        self.mine_list: list[Detection] = []
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
    

       # save image file
    def _save_image(s, image: Image, detections: list[Detection]) -> None:
        print("Saving image...")
        os.makedirs(s.config["pathToPics"], exist_ok = True)
        image_name = f"image_{datetime.now().strftime("%Y%m%d_%H%M%S")}"

        print("Drawing boxes...")
        
        draw = ImageDraw.Draw(image.image)
        for detection in detections:
            box = detection.box
            coords = (box[0], box[1], box[0] + box[2], box[1] + box[3])
            draw.rectangle(coords, fill = None, outline = (0, 255, 0, 255), width = 5)
        image.image.save(f"{path}/{image_name}.png")
        print("--- Image saved ---")

        # save detections in capture
    def _save_detections_to_json(s, detections : List[Detection]) -> None:
        print("Saving detections...")
        os.makedirs(s.config["pathToDetections"], exist_ok = True)
        file_name = f"detections_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
        for detection in detections:
            with open(f"{path}/{file_name}.json", "w") as f:
                data = {
                    # "path": path, # do we need this?
                    "x": detection.box[0],
                    "y": detection.box[1],
                    "width": detection.box[2],
                    "height": detection.box[3],
                    # .item() is needed to convert numpy to python float, np isn't json serializable
                    "category": detection.category.item(), 
                    "confidence": detection.confidence.item()
                }
            json.dump(data, f)
        print("--- Detections saved ---")



    def _cluster(self, mines_to_check: list[Detection], cluster_threshold: float) -> None:
        """
        goes through the mine list and deletes the most recent entry
        if the a mine is found with world coordinates within some
        threshold euclidean distance
        """
        mines_to_remove=[] # list of mines to remove from mine list after checking all mines to check (to avoid deleting mines while iterating through the list)
        # this will assume mine coordinates are in lat/lon FOR NOW
        # they are not in this format, they are still just boxes
        for mine_to_check in mines_to_check:

            for comparison_index in range(len(self.mine_list) - len(mines_to_check)):
                if great_circle(mine_to_check.world_coords, self.mine_list[comparison_index].world_coords) <= cluster_threshold:
                    mines_to_remove.append(mine_to_check)
        for mine in mines_to_remove:
            if mine in self.mine_list:
                self.mine_list.remove(mine)


    def get_mine_location(self, dronePose: DronePose, mine: Detection) -> tuple[float, float]:
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
        return ground
    

    #Whether or not to save images is defined in the config.json
    def scan(self,targetDetectionType: DetectionType) -> Image:
        location = [self.drone.location.global_frame.lat,self.drone.location.global_frame.lon, self.drone.location.global_frame.alt]
        pitch = self.drone.attitude.pitch
        yaw = self.drone.attitude.yaw
        roll = self.drone.attitude.roll
        drone_position = DronePose(location[0], location[1], location[2], yaw, pitch, roll)
        self.camera.capture_image()

        if(targetDetectionType == DetectionType.MINE):
            detections : list[Detection] = self.camera.capture_and_detect_mines() # take picture and get mine detections
        else:
            detections = list[Detection] =self.camera.capture_and_detect_apriltags() # take picture and get apriltag detections (not implemented yet)

        new_mines: list[Mine] = []

        for mine in detections: # add mine to mine_list if mine(s) found
            location = self.get_mine_location(drone_position, mine) # get world coordinates of mine (for now just do this for the first mine detected)

            new_mines.append(Mine(mine.confidence, location))
            #self._cluster(mine) # cluster mine list

        self.mine_list.extend(new_mines) # add new mines to mine list
        return new_mines
