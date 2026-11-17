import io
import time

import numpy as np
from PIL import Image as PILImage

try:
    from dt_apriltags import Detector
except ImportError:  # mine detection works without the apriltag library
    Detector = None

from vision.common.detection import Detection
from vision.common.image import Image
from vision.common.drone_coordinates import GimbalPose
from vision.Cameras.baseCamera import BaseCamera

# How long capture_image waits for the simulator's next picture before giving up.
CAPTURE_TIMEOUT_S = 5.0


class MrrsimCamera(BaseCamera):
    """
    Camera backed by MRRSimulator's simulated camera instead of physical hardware.
    The simulator streams the camera's pictures to this program over UDP (Vehicles tab:
    "Stream pictures to the script" on the camera) and `mrrsim_camera.CameraReceiver`
    collects them; the launcher sets SIM_CAMERA_PORT to the port to listen on.
    Mine detection runs a torch/ultralytics model on the captured frame, since
    there's no IMX500 to run it on-chip like RPICamera has.

    `mrrsim_camera` and `ultralytics` are only imported in `initialize_camera` (never
    at module load), so importing this class works where they are not installed.
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
        self.receiver = None
        # The camera's name in the Vehicles tab; "" takes the pictures of any camera.
        self.camera_name = self.config.get("mrrsimCameraName", "")
        self.model = None
        self.apriltagDetector = None

        # Same per-frame/voted-average threshold split RPICamera uses -- see
        # RPICamera.__init__ for why voting needs a looser per-frame gate.
        if self.config.get("useVoting", True):
            self.detect_threshold = self.config.get("voteThreshold", self.config["confThreshold"])
        else:
            self.detect_threshold = self.config["confThreshold"]

    def initialize_camera(self) -> None:
        from mrrsim_camera import CameraReceiver
        from ultralytics import YOLO

        self.receiver = CameraReceiver()
        self.model = YOLO(self.config["modelPath"])

        if Detector is None:
            print("WARNING: dt_apriltags not installed, apriltag detection disabled.")
            self.apriltagDetector = None
        else:
            self.apriltagDetector = Detector(
                families="tag36h11", nthreads=4, quad_decimate=2.0, refine_edges=1, debug=0
            )

    def _next_frame(self):
        """The simulator's next picture (of this camera), or None if none comes in time."""
        deadline = time.monotonic() + CAPTURE_TIMEOUT_S
        while (left := deadline - time.monotonic()) > 0:
            frame = self.receiver.get(timeout=left)
            if frame is None:
                return None
            if not self.camera_name or frame.camera == self.camera_name:
                return frame
        return None

    def capture_image(self, only_metadata: bool) -> Image:
        if self.receiver is None:
            print("Camera not initialized, call initialize_camera()")
            return None

        frame = self._next_frame()
        if frame is None:
            print(
                f"No picture from the simulator's camera in {CAPTURE_TIMEOUT_S:g} s: is it set to"
                " stream pictures to the script (Vehicles tab)?"
            )
            return None

        if only_metadata:
            return Image(None, frame)

        # The frame holds the JPEG/PNG exactly as the simulator sent it.
        pil_image = PILImage.open(io.BytesIO(frame.data)).convert("RGB")
        return Image(pil_image, frame)

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
        if self.receiver is None:
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
