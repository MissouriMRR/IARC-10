from vision.common.image import Image
from vision.common.detection import Detection
from Cameras.baseCamera import BaseCamera
from math import sin, acos
import dronekit
from common.drone_coordinates import DronePose, GimbalPose, pixel_to_geocoord_gimbal
from vision.common.mine import Mine

# The vote lives in vision.common.mine_voting so it stays importable without
# dronekit/picamera2 -- the tests and the offline tooling depend on that.
from vision.common.mine_voting import analyze_frames, vote_on_frames  # noqa: F401
from enum import Enum
import os
from datetime import datetime
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
    return 6378137 * delta_sigma  # random number is radius of the earth in meters


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
        self.camera: BaseCamera = camera  # Camera must be initialized before being passed in
        self.drone = drone

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
        directory = s.visionConfig["pathToPics"]
        os.makedirs(directory, exist_ok=True)
        image_name = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        print("Drawing boxes...")

        draw = ImageDraw.Draw(image.image)
        for detection in detections:
            # box is (cx, cy, w, h); ImageDraw wants opposite corners
            cx, cy, w, h = detection.box
            coords = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
            draw.rectangle(coords, fill=None, outline=(0, 255, 0, 255), width=5)
        image.image.save(f"{directory}/{image_name}.png")
        print("--- Image saved ---")

        # save detections in capture

    def _save_detections_to_json(s, detections: list[Detection]) -> None:
        print("Saving detections...")
        directory = s.visionConfig["pathToDetections"]
        os.makedirs(directory, exist_ok=True)
        file_name = f"detections_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        # One file per capture holding every detection, rather than reopening
        # the same path per detection and keeping only the last.
        data = [
            {
                "x": float(detection.box[0]),
                "y": float(detection.box[1]),
                "width": float(detection.box[2]),
                "height": float(detection.box[3]),
                "confidence": float(detection.score),
            }
            for detection in detections
        ]
        with open(f"{directory}/{file_name}.json", "w") as f:
            json.dump(data, f, indent=2)
        print("--- Detections saved ---")

    def _cluster(self, mines_to_check: list[Detection], cluster_threshold: float) -> None:
        """
        goes through the mine list and deletes the most recent entry
        if the a mine is found with world coordinates within some
        threshold euclidean distance
        """
        mines_to_remove = (
            []
        )  # list of mines to remove from mine list after checking all mines to check (to avoid deleting mines while iterating through the list)
        # this will assume mine coordinates are in lat/lon FOR NOW
        # they are not in this format, they are still just boxes
        for mine_to_check in mines_to_check:

            for comparison_index in range(len(self.mine_list) - len(mines_to_check)):
                if (
                    great_circle(
                        mine_to_check.world_coords, self.mine_list[comparison_index].world_coords
                    )
                    <= cluster_threshold
                ):
                    mines_to_remove.append(mine_to_check)
        for mine in mines_to_remove:
            if mine in self.mine_list:
                self.mine_list.remove(mine)

    def get_mine_location(self, dronePose: DronePose, mine: Detection) -> tuple[float, float]:
        # convert mine pixel coordinates to world coordinates using drone GPS and gimbal angle
        ground = pixel_to_geocoord_gimbal(
            px=mine.box[0],  # box is (cx, cy, w, h) in image pixels
            py=mine.box[1],
            image_width=mine.imageSize[0],
            image_height=mine.imageSize[1],
            h_fov=self.visionConfig["h_fov"],
            v_fov=self.visionConfig["v_fov"],
            drone=dronePose,
            gimbal=GimbalPose(yaw=0, pitch=-90, roll=0),
        )
        return ground

    # Whether or not to save images is defined in the config.json
    def scan(self, targetDetectionType: DetectionType) -> list[Mine]:
        location = [
            self.drone.location.global_frame.lat,
            self.drone.location.global_frame.lon,
            self.drone.location.global_frame.alt,
        ]
        pitch = self.drone.attitude.pitch
        yaw = self.drone.attitude.yaw
        roll = self.drone.attitude.roll
        drone_position = DronePose(location[0], location[1], location[2], yaw, pitch, roll)

        if targetDetectionType == DetectionType.MINE:
            if self.visionConfig.get("useVoting", True):
                # Take a burst of frames and only keep detections that survive
                # the vote. Inference runs on the IMX500 itself, so each extra
                # frame costs one camera frame interval (~33 ms at 30 fps)
                # rather than a full model run on the Pi.
                frames = [
                    self.camera.capture_and_detect_mines()
                    for _ in range(self.visionConfig["scanFrames"])
                ]
                detections = vote_on_frames(
                    frames,
                    iou_threshold=self.visionConfig["voteIoU"],
                    min_hits=self.visionConfig["minFrameHits"],
                    min_average_score=self.visionConfig["minAverageConfidence"],
                )
            else:
                # Single-shot scan: one frame, judged only by its own score.
                # Cameras that gate at the looser voteThreshold can still hand
                # back sub-confThreshold boxes, so re-apply the strict cutoff
                # here -- with no average to defend, it is the only filter left.
                detections = [
                    detection
                    for detection in self.camera.capture_and_detect_mines()
                    if detection.score >= self.visionConfig["confThreshold"]
                ]
        else:
            detections: list[Detection] = (
                self.camera.capture_and_detect_apriltags()
            )  # take picture and get apriltag detections (not implemented yet)

        new_mines: list[Mine] = []

        for mine in detections:  # add mine to mine_list if mine(s) found
            location = self.get_mine_location(
                drone_position, mine
            )  # get world coordinates of mine (for now just do this for the first mine detected)

            # With voting on, mine.score is the burst-averaged confidence rather
            # than a single frame's; with it off, it is that one frame's score.
            new_mines.append(Mine(mine.score, location))
            # self._cluster(mine) # cluster mine list

        self.mine_list.extend(new_mines)  # add new mines to mine list
        return new_mines
