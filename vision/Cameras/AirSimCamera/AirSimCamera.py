import numpy as np
from PIL import Image as PILImage

try:
    from dt_apriltags import Detector
except ImportError:  # mine detection works without the apriltag library
    Detector = None
from ultralytics import YOLO

from vision.common.detection import Detection
from vision.common.image import Image
from vision.common.drone_coordinates import GimbalPose
from vision.Cameras.baseCamera import BaseCamera


class AirSimCamera(BaseCamera):
    """
    Camera backed by AirSim's simulated camera instead of physical hardware.
    Mine detection runs a torch/ultralytics model on the captured frame, since
    there's no IMX500 to run it on-chip like RPICamera has.

    `airsim` is only ever imported inside the methods that talk to the sim
    (never at module load) so importing this class doesn't require airsim to
    be installed on hardware that isn't running a simulation.
    """

    def __init__(self, visionConfig):
        mount_rotation_deg = visionConfig.get("cameraMountRotationDeg", {})
        super().__init__(
            h_fov_deg=visionConfig.get("hFovDeg", 0.0),
            v_fov_deg=visionConfig.get("vFovDeg", 0.0),
            offset=tuple(visionConfig.get("cameraOffsetM", (0.0, 0.0, 0.0))),
            mount_rotation=GimbalPose(
                yaw=mount_rotation_deg.get("yaw", 0.0),
                pitch=mount_rotation_deg.get("pitch", 0.0),
                roll=mount_rotation_deg.get("roll", 0.0),
            ),
        )
        self.config = visionConfig
        self.client = None
        self.vehicle_name = self.config.get("airsimVehicleName", "")
        self.camera_name = self.config.get("airsimCameraName", "0")
        self.model = None
        self.apriltagDetector = None

        # Same per-frame/voted-average threshold split RPICamera uses -- see
        # RPICamera.__init__ for why voting needs a looser per-frame gate.
        if self.config.get("useVoting", True):
            self.detect_threshold = self.config.get("voteThreshold", self.config["confThreshold"])
        else:
            self.detect_threshold = self.config["confThreshold"]

    def initialize_camera(self) -> None:
        import airsim

        self.client = airsim.MultirotorClient(ip=self.config.get("airsimIp", ""))
        self.client.confirmConnection()

        self.model = YOLO(self.config["modelPath"])

        if Detector is None:
            print("WARNING: dt_apriltags not installed, apriltag detection disabled.")
            self.apriltagDetector = None
        else:
            self.apriltagDetector = Detector(
                families="tag36h11", nthreads=4, quad_decimate=2.0, refine_edges=1, debug=0
            )

    def capture_image(self, only_metadata: bool) -> Image:
        import airsim

        if self.client is None:
            print("Camera not initialized, call initialize_camera()")
            return None

        request = airsim.ImageRequest(self.camera_name, airsim.ImageType.Scene, False, False)
        response = self.client.simGetImages([request], vehicle_name=self.vehicle_name)[0]

        if only_metadata:
            return Image(None, response)

        # AirSim hands back raw BGR bytes (same convention as cv2), so flip to
        # RGB for the PIL image this class stores everywhere else.
        img_bgr = np.frombuffer(response.image_data_uint8, dtype=np.uint8).reshape(
            response.height, response.width, 3
        )
        pil_image = PILImage.fromarray(img_bgr[:, :, ::-1])
        return Image(pil_image, response)

    def capture_and_detect_mines(self) -> list[Detection]:
        if self.model is None:
            print("Camera not initialized, call initialize_camera()")
            return []
        image = self.capture_image(only_metadata=False)
        if image is None or image.image is None:
            return []

        frame_w, frame_h = image.image.size
        results = self.model.predict(source=image.image, verbose=False)[0]

        dets = []
        for box in results.boxes:
            score = float(box.conf[0])
            if score < self.detect_threshold:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets.append(
                Detection(
                    score,
                    ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1),  # (cx, cy, w, h)
                    (frame_w, frame_h),
                )
            )

        dets.sort(key=lambda d: d.score, reverse=True)
        dets = dets[: self.config["maxDetections"]]
        return dets

    def capture_and_detect_apriltags(self) -> list[Detection]:
        if self.client is None:
            print("Camera not initialized, call initialize_camera()")
            return []
        if self.apriltagDetector is None:
            print("dt_apriltags not installed, cannot detect apriltags")
            return []

        image = self.capture_image(only_metadata=False)
        if image is None or image.image is None:
            return []

        frame_size = image.image.size
        gray_array = np.array(image.image.convert("L"), dtype=np.uint8)
        apriltags = self.apriltagDetector.detect(gray_array)

        detections = []
        for tag in apriltags:
            width = tag.corners[2][0] - tag.corners[0][0]
            height = tag.corners[2][1] - tag.corners[0][1]
            detections.append(Detection(1.0, (tag.center[0], tag.center[1], width, height), frame_size))
        return detections
