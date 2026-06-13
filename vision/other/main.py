import camera
import time
import json
import dronekit
import os

if __name__ == "__main__":
   # setup
   with open("config.json", "r") as f:
      config = json.load(f)
   cam = camera.Camera(config)
   cam.start_camera()
   #drone = dronekit.connect(config["droneAddress"], wait_ready = True, baud = config["baudrate"])
   os.system("cls" if os.name == "nt" else "clear") # generic clear screen command for prettiness
   
   # main loop
   while True:
      
      print("Ready for input (r/q): ", end = "")
      keypress = input()
      os.system("cls" if os.name == "nt" else "clear")
      if keypress == "r":
         cam.capture(save_image = True, save_detections = True, save_metadata = False, drone = None, force_capture = False)
      elif keypress == "q":
         print("Quitting...")
         break
