from picamera2 import Picamera2
import os
from datetime import datetime
import time
import dronekit
import json

#-----------------------------------------------------------------------------------------------------------
# VARIABLES TO CHANGE
#-----------------------------------------------------------------------------------------------------------
pathToPics = './captures'
pathToMetadata = './metadata'
droneAddress = "/dev/ttyUSB0"
baudRate = 921600

def take_image(camera: Picamera2, drone: dronekit.Vehicle):
    # create a directory to put the pictures and the metadata in
    os.makedirs("./captures",exist_ok=True)
    os.makedirs("./metadata",exist_ok=True)
    # create the filepath where we'll save the image and metadata
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"image_{timestamp}.jpg"
    picPath = os.path.join(pathToPics, filename)
    filename = f"metadata_{timestamp}.json"
    metadataPath = os.path.join(pathToMetadata, filename)
    # get drone info
    location = [drone.location.global_frame.lat,drone.location.global_frame.lon, drone.location.global_frame.alt]
    pitch = drone.attitude.pitch
    yaw = drone.attitude.yaw
    roll = drone.attitude.roll
    output_dict = {"pitch":pitch, "yaw":yaw, "roll":roll, "location(lat,lon,alt)":location}
    # dump the drone info and save it to a json
    with open(metadataPath, "w") as f:
        json.dump(output_dict, f)
    print("metadata saved")
    # take the image and save it
    camera.capture_file(picPath)
    print("picture taken")
        

def main():
    # init dronekit object
    drone = dronekit.connect(droneAddress,wait_ready=True,baud=baudRate)

    # init camera
    camera = Picamera2()
    # set config
    camera.configure(camera.create_still_configuration())
    # start camera
    camera.start()
    print("camera started")
    # give it a sec to start up
    time.sleep(2)
    #main loop
    while True:
        keypress = input()
        if keypress == 'r':
            take_image(camera,drone)

        elif keypress == "q":
            break
    # turn the camera off
    camera.stop()
    camera.close()
    print("camera stopped")

if __name__ == "__main__":
    main()
