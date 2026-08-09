import os
import time
import numpy as np
try:
   from dt_apriltags import Detector
except ImportError:  # mine detection works without the apriltag library
   Detector = None
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics

from vision.common.detection import Detection
from vision.common.image import Image
from Cameras.baseCamera import BaseCamera


class RPICamera(BaseCamera):
   def __init__(self, visionConfig):
      super().__init__()
      self.config = visionConfig
      
      self.input_h=-1
      self.input_w=-1
      self.labels = []
      self.picam2 = None
      self.imx500 = None

      # Order of the four numbers in each raw box the network emits.
      #   "xy" -> [x1, y1, x2, y2]   (Ultralytics YOLO exported with format="imx")
      #   "yx" -> [y1, x1, y2, x2]   (TensorFlow SSD models, picamera2's default)
      # Getting this wrong mirrors every box across the image diagonal, so boxes
      # land in the transposed spot and come out with width/height swapped.
      self.bbox_order = self.config.get("bboxOrder", "xy")



   def initialize_camera(self) -> None:
      self.imx500 = IMX500(self.config["modelPath"])
      self.input_w, self.input_h = self.imx500.get_input_size()

      intrinsics = self.imx500.network_intrinsics or NetworkIntrinsics()
      intrinsics.task = "object detection"
      intrinsics.update_with_defaults()

      self.picam2 = Picamera2(self.imx500.camera_num)



      """
      self.picam2: Picamera2 = Picamera2()
      self.camera_config = self.picam2.create_preview_configuration(
      main = {"size": self.config["mainRes"]},
      lores = {"size": self.config["loresRes"], "format": "YUV420"}
      )
      """
      config = self.picam2.create_preview_configuration(
         controls={"FrameRate": intrinsics.inference_rate},
         buffer_count=12
      )

      self.picam2.configure(config)
      self.picam2.set_controls({"ExposureTime": self.config["shutterSpeed"]}) # in microseconds
      self.picam2.start_preview(False)
      self.imx500.show_network_fw_progress_bar()
      self.picam2.start()

      with open(self.config["labelsPath"]) as f:
         self.labels = [line.strip() for line in f if line.strip()]


      # Detect tags
      if Detector is None:
         print("WARNING: dt_apriltags not installed, apriltag detection disabled.")
         self.apriltagDetector = None
      else:
         self.apriltagDetector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            refine_edges=1,
            debug=0
         )
      

   def capture_and_detect_mines(self) -> list[Detection]:
      if(self.input_w == -1 or self.input_h == -1):
         print("Camera not initialized, call initialize_camera()")
         return []
      metadata = self.capture_image(only_metadata = True).metadata
      outputs = self.imx500.get_outputs(metadata, add_batch=True)
      if outputs is None:
            print("WARNING: model returned no outputs.")
            return []
      if len(outputs) < 3:
            print(f"WARNING: model returned {len(outputs)} output tensors, "
                     f"expected >=3 (boxes, scores, classes). "
                     f"Output format may not match this parser.")
            return []
      


      boxes = np.asarray(outputs[0][0], dtype=float)
      scores = np.asarray(outputs[1][0], dtype=float)
      classes = np.asarray(outputs[2][0])

      if boxes.size == 0:
            return []

      # Normalize the corner order to [y1, x1, y2, x2], which is what
      # convert_inference_coords() takes.
      if self.bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]

      # convert_inference_coords() wants 0..1 coordinates. Some exports emit
      # input-tensor pixels instead, so scale those down first.
      if np.nanmax(np.abs(boxes)) > 1.5:
            boxes = boxes / np.array(
               [self.input_h, self.input_w, self.input_h, self.input_w], dtype=float
            )

      frame_w, frame_h = self.picam2.camera_configuration()["main"]["size"]

      dets = []
      for box, score, cls in zip(boxes, scores, classes):
            if score < self.config["confThreshold"]:
               continue
            cls = int(cls)
            name = self.labels[cls] if 0 <= cls < len(self.labels) else f"id_{cls}"

            # The network sees a square crop of the sensor, not the full preview
            # frame, so normalized inference coords cannot be scaled straight
            # onto the frame. convert_inference_coords() applies the ISP scaler
            # crop and returns (x, y, w, h) in main-stream pixels.
            x, y, w, h = self.imx500.convert_inference_coords(
               tuple(box), metadata, self.picam2
            )
            dets.append(
               Detection(
                  float(score),
                  (x + w / 2, y + h / 2, w, h),  # (cx, cy, w, h), main-frame pixels
                  (frame_w, frame_h),
               )
            )

      dets.sort(key=lambda d: d.score, reverse=True)
      dets = dets[: self.config["maxDetections"]]

      
      return dets

   def capture_and_detect_apriltags(self) -> list[Detection]:
      if(self.input_w == -1 or self.input_h == -1):
         print("Camera not initialized, call initialize_camera(")
         return []
      if self.apriltagDetector is None:
         print("dt_apriltags not installed, cannot detect apriltags")
         return []
      PILImage=self.capture_image(only_metadata = False)
      grayscale_Image = PILImage.image.convert("L") # convert to grayscale for apriltag
      gray_array = np.array(grayscale_Image, dtype=np.uint8)
      apriltags = self.apriltagDetector.detect(gray_array)

      frame_size = self.picam2.camera_configuration()["main"]["size"]
      detections=[]
      for i in apriltags:
         width = i.corners[2][0] - i.corners[0][0]
         height = i.corners[2][1] - i.corners[0][1]
         # corners are already in main-frame pixels, same space as mine boxes
         detections.append(Detection(1.0, (i.center[0], i.center[1], width, height), frame_size))
      return detections
   
   def capture_image(self, only_metadata: bool) -> Image:
      # capture image and return as Cameras.Camera_utils.image.Image object
      # also return metadata for detection parsing
      if self.input_w == -1 or self.input_h == -1 or self.picam2 is None:
         print("Camera not initialized, call initialize_camera()")
         return None
      if only_metadata:
         return Image(None, self.picam2.capture_metadata())

      request = self.picam2.capture_request()
      im = request.make_image("main")
      metadata = request.get_metadata()
      request.release()
      return Image(im, metadata)

      """                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      # performs magic to turn camera tensors into useable data
      def _parse_detections(self, metadata) -> list[Detection]:
         self.network_outputs = self.imx500.get_outputs(metadata, add_batch = True) # output tensors, batch allows multiple detections at once
         if self.network_outputs is None:
            return None
         self.boxes, self.scores, self.classes = self.network_outputs[0][0], self.network_outputs[1][0], self.network_outputs[2][0]
         last_detections = [Detection(box, category, score, metadata, self.picam2, self.imx500) for box, score, category in zip(self.boxes, self.scores, self.classes) if score > self.config["confThreshold"]]
         return last_detections

      # save image file
      def _save_image(s, path, detections) -> None:
         print("Saving image...")
         os.makedirs(s.config["pathToPics"], exist_ok = True)
         image_name = f"image_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
         s.picam2.capture_file(f"{path}/{image_name}.png")
         print("Drawing boxes...")
         im = Image.open(f"{path}/{image_name}.png")
         draw = ImageDraw.Draw(im)
         for detection in detections:
            box = detection.box
            coords = (box[0], box[1], box[0] + box[2], box[1] + box[3])
            draw.rectangle(coords, fill = None, outline = (0, 255, 0, 255), width = 5)
         im.save(f"{path}/{image_name}.png")
         print("--- Image saved ---")

      # save detections in capture
      def _save_detections_to_json(s, path, detections) -> None:
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
      
      # save drone metadata
      def _save_metadata_to_json(s, path, drone_attitude):
         print("Saving metadata...")
         os.makedirs(s.config["pathToMetadata"], exist_ok = True)
         file_name = f"metadata_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
         with open(f"{path}/{file_name}.json", "w") as f:
            data = {
               "pitch": drone.attitude.pitch,
               "yaw": drone.attitude.yaw,
               "roll": drone.attitude.roll,
               "location (lat, lon, alt)": [drone.location.global_frame.lat,drone.location.global_frame.lon, drone.location.global_frame.alt]
            }
            json.dump(data, f)
         print("--- Metadata saved ---")

      # returns all detections in image as list of Detection objects
      # with bonus data saving parameters!
      def capture(s, save_image: bool, save_detections: bool, save_metadata: bool, drone_position,
                  force_capture: bool = False) -> list[MineDetection]:
         # return corner coordinates as well
         # add photo object?
         metadata = s.picam2.capture_metadata()
         detections = s._parse_detections(metadata)
         if detections is None or detections == []:
               print("Nothing detected")
         if not force_capture: return []
         if save_image: s._save_image(s.config["pathToPics"], detections)
         if save_detections: s._save_detections_to_json(s.config["pathToDetections"], detections)
         if save_metadata: s._save_metadata_to_json(s.config["pathToMetadata"], drone_position[3:])
         return [MineDetection(detection.box, drone_position) for detection in detections]


      """

