import camera
import time
import json

if name == "__main__":
   with open("config.json", "r") as f:
      config = json.load(f)
   cam = camera.Camera(config)
   cam.start_camera()
   time.sleep(5)
   cam.capture(save_image = True, save_boxes = True)