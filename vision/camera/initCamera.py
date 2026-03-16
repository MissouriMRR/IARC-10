from picamera2 import Picamera2, Preview, MappedArray
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics, postprocess_nanodet_detection)
import cv2
import json

with open("config.json", "r") as f:
    config = json.load(f)

def start_camera(config) -> None:
    picam2: Picamera2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main = {"size": config["mainRes"]},
        lores = {"size": config["loresRes"], "format": "YUV420"}
    )
    picam.configure(camera_config)
    picam2.start()