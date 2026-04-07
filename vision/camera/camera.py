from picamera2 import Picamera2, Preview
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics, postprocess_nanodet_detection)
from typing import Tuple
from detection import Detection
from datetime import datetime
from PIL import Image, ImageDraw
import time
import json
import os

class Camera():
   def __init__(s, config):
      s.config = config
      s.imx500 = IMX500(config["modelPath"])
      # this shouldn't be necessary unless we need to detect more than
      #  one type of object.
      # s.LABELS = s.imx500.network_intrinsics.labels

   # this must be run before doing camera things
   # why is it not in __init__?
   # i hate you
   def start_camera(s) -> None:
      s.picam2: Picamera2 = Picamera2()
      s.camera_config = s.picam2.create_preview_configuration(
        main = {"size": s.config["mainRes"]},
        lores = {"size": s.config["loresRes"], "format": "YUV420"}
      )
      s.picam2.configure(s.camera_config)
      s.picam2.set_controls({"ExposureTime": s.config["shutterSpeed"]}) # in microseconds
      s.picam2.start_preview(Preview.QTGL)
      s.picam2.start()
      time.sleep(2) # startup time

   # performs magic to turn camera tensors into useable data
   def _parse_detections(s, metadata) -> list[Detection]:
      s.network_outputs = s.imx500.get_outputs(metadata, add_batch = True) # output tensors, batch allows multiple detections at once
      if s.network_outputs is None:
         return None
      s.boxes, s.scores, s.classes = s.network_outputs[0][0], s.network_outputs[1][0], s.network_outputs[2][0]
      last_detections = [Detection(box, category, score, metadata, s.picam2, s.imx500) for box, score, category in zip(s.boxes, s.scores, s.classes) if score > s.config["confThreshold"]]
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
   def _save_metadata_to_json(s, path, drone):
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
   def capture(s, save_image: bool, save_detections: bool, save_metadata: bool, drone, force_capture: bool = False) -> list[Detection]:
      # path = ''.join(d for d in str(datetime.now()) if d.isdigit()) if (save_image or save_to_json) else None # EVIL code because i'm EVIL
      metadata = s.picam2.capture_metadata()
      detections = s._parse_detections(metadata)
      if detections is None or detections == []:
         print("Nothing detected")
         if not force_capture: return None
      if save_image: s._save_image(s.config["pathToPics"], detections)
      if save_detections: s._save_detections_to_json(s.config["pathToDetections"], detections)
      if save_metadata: s._save_metadata_to_json(s.config["pathToMetadata"], drone)
      return detections
