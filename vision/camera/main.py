import camera
import time
import json
import dronekit

if name == "__main__":
   # setup
   with open("config.json", "r") as f:
      config = json.load(f)
   cam = camera.Camera(config)
   cam.start_camera()
   drone = dronekit.connect(config["droneAddress"], wait_ready = True, baud = config["baudrate"])
   time.sleep(5)

   # main loop
   while True:
      keyprees = input()
      if keyprees == "r":
         cam.capture(save_image = True, save_data = True, drone = drone)
      elif keyprees == "q":
         break