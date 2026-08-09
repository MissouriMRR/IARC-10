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
    return 6378137 * delta_sigma  # random number is radius of the earth in meters


"""
Big boy vision class used by the pathfinding algorithm 
"""


class DetectionType(Enum):
    MINE = 1
    APRILTAG = 2


def _iou(box_a, box_b) -> float:
    """Intersection over union of two (cx, cy, w, h) boxes in image pixels."""
    ax, ay, aw, ah = box_a
    bx, by, bw, bh = box_b

    a_x1, a_y1, a_x2, a_y2 = ax - aw / 2, ay - ah / 2, ax + aw / 2, ay + ah / 2
    b_x1, b_y1, b_x2, b_y2 = bx - bw / 2, by - bh / 2, bx + bw / 2, by + bh / 2

    inter_w = min(a_x2, b_x2) - max(a_x1, b_x1)
    inter_h = min(a_y2, b_y2) - max(a_y1, b_y1)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0

    intersection = inter_w * inter_h
    union = aw * ah + bw * bh - intersection
    if union <= 0:
        return 0.0
    return intersection / union


class _Track:
    """One candidate mine, accumulated across the frames of a single scan."""

    def __init__(self, detection: Detection):
        self.detections: list[Detection] = [detection]

    @property
    def hits(self) -> int:
        return len(self.detections)

    @property
    def average_score(self) -> float:
        return sum(d.score for d in self.detections) / len(self.detections)

    @property
    def best(self) -> Detection:
        """Highest-scoring view of this candidate; its box is the one we report,
        since it is the frame the model was most sure about."""
        return max(self.detections, key=lambda d: d.score)

    @property
    def latest_box(self):
        return self.detections[-1].box


class VoteResult:
    """One candidate's fate in a scan, including why it was rejected.

    vote_on_frames() throws the rejects away; tooling that needs to explain a
    scan (the interactive scan test, saved scan records) wants them, so
    analyze_frames() returns these instead.
    """

    def __init__(self, track: "_Track", min_hits: int, min_average_score: float):
        self.box = track.best.box  # best frame's box, in main-stream pixels
        self.imageSize = track.best.imageSize
        self.hits = track.hits
        self.scores = [d.score for d in track.detections]
        self.average_score = track.average_score
        self.best_score = track.best.score

        self.enough_hits = self.hits >= min_hits
        self.enough_confidence = self.average_score >= min_average_score
        self.confirmed = self.enough_hits and self.enough_confidence

    @property
    def reason(self) -> str:
        """Short human-readable verdict, for overlay labels and saved records."""
        if self.confirmed:
            return "confirmed"
        failures = []
        if not self.enough_hits:
            failures.append("too few frames")
        if not self.enough_confidence:
            failures.append("low avg conf")
        return " + ".join(failures)

    def as_detection(self) -> Detection:
        """The averaged score is the number the vote actually justifies, so that
        is what travels onward rather than any single frame's score."""
        return Detection(self.average_score, self.box, self.imageSize)

    def to_dict(self) -> dict:
        cx, cy, w, h = (float(v) for v in self.box)
        return {
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h,
            "hits": self.hits,
            "scores": [float(s) for s in self.scores],
            "average_score": float(self.average_score),
            "best_score": float(self.best_score),
            "confirmed": self.confirmed,
            "enough_hits": self.enough_hits,
            "enough_confidence": self.enough_confidence,
            "reason": self.reason,
        }


def analyze_frames(
    frames: list[list[Detection]],
    iou_threshold: float,
    min_hits: int,
    min_average_score: float,
) -> list[VoteResult]:
    """Associate detections across frames and judge each candidate.

    Returns every candidate found, confirmed or not, sorted best first. Use
    vote_on_frames() if you only want the ones that passed.
    """
    tracks: list[_Track] = []

    for frame in frames:
        # Strongest detections claim their track first, so a confident mine is
        # not stolen by an overlapping weak box earlier in the list.
        claimed: set[int] = set()
        for detection in sorted(frame, key=lambda d: d.score, reverse=True):
            best_index = -1
            best_iou = iou_threshold
            for index, track in enumerate(tracks):
                if index in claimed:
                    continue
                overlap = _iou(detection.box, track.latest_box)
                if overlap >= best_iou:
                    best_iou = overlap
                    best_index = index

            if best_index == -1:
                tracks.append(_Track(detection))
                claimed.add(len(tracks) - 1)
            else:
                tracks[best_index].detections.append(detection)
                claimed.add(best_index)

    results = [VoteResult(track, min_hits, min_average_score) for track in tracks]
    results.sort(key=lambda r: (r.confirmed, r.average_score), reverse=True)
    return results


def vote_on_frames(
    frames: list[list[Detection]],
    iou_threshold: float,
    min_hits: int,
    min_average_score: float,
) -> list[Detection]:
    """Collapse several frames of detections into the ones worth believing.

    Detections are associated across frames by box IoU, then a candidate is kept
    only if it appears in at least `min_hits` frames AND its mean score across
    those frames is at least `min_average_score`. This is the false-positive
    filter: a one-frame flicker fails the hit count, and a persistent but weak
    detection fails the average.

    Note that the frames of a scan are captured back to back, so a false
    positive that is stable across the burst (a rock, a shadow) will satisfy the
    hit count as readily as a real mine. The averaged score is what has to
    reject those.
    """
    return [
        result.as_detection()
        for result in analyze_frames(
            frames, iou_threshold, min_hits, min_average_score
        )
        if result.confirmed
    ]


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
        os.makedirs(s.config["pathToPics"], exist_ok=True)
        image_name = f"image_{datetime.now().strftime("%Y%m%d_%H%M%S")}"

        print("Drawing boxes...")

        draw = ImageDraw.Draw(image.image)
        for detection in detections:
            box = detection.box
            coords = (box[0], box[1], box[0] + box[2], box[1] + box[3])
            draw.rectangle(coords, fill=None, outline=(0, 255, 0, 255), width=5)
        image.image.save(f"{path}/{image_name}.png")
        print("--- Image saved ---")

        # save detections in capture

    def _save_detections_to_json(s, detections: List[Detection]) -> None:
        print("Saving detections...")
        os.makedirs(s.config["pathToDetections"], exist_ok=True)
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
                    "confidence": detection.confidence.item(),
                }
            json.dump(data, f)
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
            # Take a burst of frames and only keep detections that survive the
            # vote. Inference runs on the IMX500 itself, so each extra frame
            # costs one camera frame interval (~33 ms at 30 fps) rather than a
            # full model run on the Pi.
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
            detections: list[Detection] = (
                self.camera.capture_and_detect_apriltags()
            )  # take picture and get apriltag detections (not implemented yet)

        new_mines: list[Mine] = []

        for mine in detections:  # add mine to mine_list if mine(s) found
            location = self.get_mine_location(
                drone_position, mine
            )  # get world coordinates of mine (for now just do this for the first mine detected)

            # mine.score is the burst-averaged confidence, not a single frame's
            new_mines.append(Mine(mine.score, location))
            # self._cluster(mine) # cluster mine list

        self.mine_list.extend(new_mines)  # add new mines to mine list
        return new_mines
