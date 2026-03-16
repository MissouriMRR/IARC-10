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
      s.imx500 = IMX500(MODEL_PATH)
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
      s.picam2.capture_file(f"{path}.png")
      with Image.open(f"{path}.png") as im:
         draw = ImageDraw.Draw(im)
         for box in detections:
            draw.rectangle(box, outline = s.config["color"])

   def _save_to_json(detections) -> None:
      for detection in detections:
         with open("captures.json", "w") as f:
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

   # returns all detections in image as list of Detection objects
   # with bonus data saving parameters!
   def capture(s, save_image: bool, save_to_json: bool) -> list[Detection]:
      path = ''.join(d for d in str(datetime.now()) if d.isdigit()) if (save_image or save_to_json) else None # EVIL code because i'm EVIL
      metadata = picam2.capture_metadata()
      detections = _parse_detections(metadata)
      if detections is None: return None
      if save_image: _save_image(path, detections)
      if save_to_json: _save_to_json(detections)
      return detections