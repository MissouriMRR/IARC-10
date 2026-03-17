import camera
import time
import json
import dronekit

if __name__ == "__main__":
   # setup
   with open("config.json", "r") as f:
      config = json.load(f)
   cam = camera.Camera(config)
   cam.start_camera()
   #drone = dronekit.connect(config["droneAddress"], wait_ready = True, baud = config["baudrate"])
   print("ready for input")

   # main loop
   while True:
      keypress = input()
      if keypress == "r":
         cam.capture(save_image = True, save_data = True, drone = None)
      elif keypress == "q":
         break
