from picamera2 import Picamera2
from typing import Tuple
from detection import Detection
from datetime import datetime
from PIL import Image, ImageDraw
import time
import json

class Camera():
   def __init__(s, config):
      s.config = config
      s.imx500 = IMX500(config["modelPath"])
      s.LABELS = s.imx500.network_intrinsics.labels

   # this must be run before doing camera things
   # why is it not in __init__?
   # i hate you
   def start_camera(s) -> None:
      s.picam2: Picamera2 = Picamera2()
      s.camera_config = picam2.create_preview_configuration(
        main = {"size": s.config["mainRes"]},
        lores = {"size": s.config["loresRes"], "format": "YUV420"}
      )
      s.picam2.configure(s.camera_config)
      s.picam2.start()
      time.sleep(2) # startup time

   # performs magic to turn camera tensors into useable data
   def _parse_detections(metadata) -> list[Detection]:
      network_outputs = s.imx500.get_outputs(metadata, add_batch = True) # output tensors, batch allows multiple detections at once
      if network_outputs is None:
         return None
      s.boxes, s.scores, s.classes = s.network_outputs[0][0], s.network_outputs[1][0], s.network_outputs[2][0]
      last_detections = [Detection(box, category, score, metadata) for box, score, category in zip(boxes, scores, classes) if score > CONF_THRESHOLD]
      return last_detections

   def _save_image(path, detections) -> None:
      image_name = f"image_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
      s.picam2.capture_file(f"{path}/{image_name}.png")
      with Image.open(f"{path}/{image_name}.png") as im:
         draw = ImageDraw.Draw(im)
         for box in detections:
            draw.rectangle(box, outline = s.config["color"])
      print("Image saved")

   def _save_detections_to_json(path, detections) -> None:
      file_name = f"detections_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
      for detection in detections:
         with open(f"{path}/{file_name}.json", "w") as f:
            data = {
               "path": path,
               "x": detection.box[0],
               "y": detection.box[1],
               "width": detection.box[2],
               "height": detection.box[3],
               "category": detection.category,
               "confidence": detection.confidence
            }
            json.dump(data, f)
      print("Detections saved")
   
   def _save_metadata_to_json(path, drone):
      file_name = f"metadata_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
      with open(f"{path}/{file_name}.json", "w") as f:
         data = {
            "pitch": drone.attitude.pitch,
            "yaw": drone.attitude.yaw,
            "roll": drone.attitude.roll,
            "location (lat, lon, alt)": [drone.location.global_frame.lat,drone.location.global_frame.lon, drone.location.global_frame.alt]
         }
         json.dump(data, f)
      print("Metadata saved")

   # returns all detections in image as list of Detection objects
   # with bonus data saving parameters!
   def capture(s, save_image: bool, save_data: bool, drone) -> list[Detection]:
      # path = ''.join(d for d in str(datetime.now()) if d.isdigit()) if (save_image or save_to_json) else None # EVIL code because i'm EVIL
      metadata = picam2.capture_metadata()
      detections = _parse_detections(metadata)
      if detections is None: return None
      if save_image: _save_image(s.config["pathToPics"], detections)
      if save_data:
         _save_metadata_to_json(s.config["pathToMetadata"], drone)
         _save_detections_to_json(s.config["pathToDetections"], detections)
      return detections