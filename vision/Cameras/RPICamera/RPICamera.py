import os



import apriltag
import argparse
from random import random
import time
import numpy as np
from picamera2 import Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics
from typing import Tuple
from datetime import datetime

import baseCamera
from vision.common.detection import Detection
from vision.common.image import Image
from vision.common.detection import Detection
from Cameras.baseCamera import BaseCamera

import time
import json


class RPICamera(BaseCamera):
   def __init__(self, visionConfig):
      
      self.config = visionConfig
      
      self.input_h=-1
      self.input_w=-1
      self.labels = []
      self.picam2 = None
      self.imx500 = None
      


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
      config = self.picam2.create_preview_configuration()

      self.picam2.configure(self.camera_config)
      self.picam2.set_controls({"ExposureTime": self.config["shutterSpeed"]}) # in microseconds
      self.picam2.start_preview(config, show_preview=False) # show_preview=False -> no Qt window
      self.picam2.start()
      
      self.input_w, self.input_h = self.imx500.get_input_size()

      with open(self.config["labelsPath"]) as f:
         self.labels = [line.strip() for line in f if line.strip()]


      options = apriltag.DetectorOptions(
         families="tag36h11", # Tag family
         nthreads=4, # Parallel threads
         quad_decimate=2.0, # Downsampling factor
         quad_sigma=0.0, # Gaussian blur sigma
         refine_edges=True, # Edge refinement
         decode_sharpening=0.25, # Sharpening factor
         debug=False # Debug image output
      )

      # Detect tags
      self.apriltagDetector = apriltag.Detector(options)
      

   def capture_and_detect_mines(self) -> list[Detection]:
      if(self.input_w == -1 or self.input_h == -1):
         print("Camera not initialized, call initialize_camera()")
         return []
      metadata = self.capture_image(only_metadata = True).metadata
      outputs = self.imx500.get_outputs(metadata, add_batch=True)
      if outputs is None or len(outputs) < 3:
            print(f"WARNING: model returned {len(outputs)} output tensors, "
                     f"expected >=3 (boxes, scores, classes). "
                     f"Output format may not match this parser.")
            return []
      


      boxes = np.asarray(outputs[0][0], dtype=float)
      scores = np.asarray(outputs[1][0], dtype=float)
      classes = np.asarray(outputs[2][0])


      
      boxes = boxes[:, [1, 0, 3, 2]] #Boxes must be in xy order

      dets = []
      for box, score, cls in zip(boxes, scores, classes):
            if score < self.config["confThreshold"]:
               continue
            cls = int(cls)
            name = self.labels[cls] if 0 <= cls < len(self.labels) else f"id_{cls}"

            dets.append(Detection(float(score), box, (self.input_w, self.input_h)))

      dets.sort(key=lambda d: d.score, reverse=True)
      dets = dets[: self.config["maxDetections"]]

      
      return dets

   def capture_and_detect_apriltags(self) -> list[Detection]:
      if(self.input_w == -1 or self.input_h == -1):
         print("Camera not initialized, call initialize_camera(")
         return []
      PILImage=self.capture_image(only_metadata = False)
      grayscale_Image = PILImage.image.convert("L") # convert to grayscale for apriltag
      gray_array = np.array(grayscale_Image, dtype=np.uint8)
      apriltags = self.apriltagDetector.detect(gray_array)

      detections=[]
      for i in apriltags:
         width = i.bbox[2][0] - i.bbox[0][0]
         height = i.bbox[2][1] - i.bbox[0][1]
         detections.append(Detection(1.0, (i.center[0], i.center[1], width, height), (self.input_w, self.input_h)))
      return detections
   
   def capture_image(self, only_metadata: bool) -> Image:
      # capture image and return as Cameras.Camera_utils.image.Image object
      # also return metadata for detection parsing
      if(self.input_w == -1 or self.input_h == -1):
         print("Camera not initialized, call initialize_camera()")
         return None
      metadata = self.picam2.capture_metadata()
      
      if not(only_metadata):
         im = self.picam2.capture_image()
         
      else: 
         im = None
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

