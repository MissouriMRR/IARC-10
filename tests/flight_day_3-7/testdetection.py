from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500
import os
from datetime import datetime
import time
import dronekit
import json
import cv2

#-----------------------------------------------------------------------------------------------------------
# VARIABLES TO CHANGE
#-----------------------------------------------------------------------------------------------------------
pathToPics = './captures'
pathToDetections = './captures/detections'
pathToMetadata = './metadata'
modelPath = '../../vision/models/2-16-2026/v11s_2-16.rpk'
droneAddress = ''
baudRate = 10
confTresh = 0.65

def main():
    # init dronekit object
    drone = dronekit.connect(droneAddress,wait_ready=True,baud=baudRate)

    # init detector
    imx500 = IMX500(modelPath)

    # init camera
    camera = Picamera2(imx500.camera_num)
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
        # take a picture
        if keypress == 'r':
            take_image(camera,drone)
        # exit loop
        elif keypress == "q":
            break
        # run the detection model
        elif keypress == 'd':
            detect(camera,drone,imx500)
    # turn the camera off
    camera.stop()
    camera.close()
    print("camera stopped")

def detect(camera: Picamera2, drone: dronekit.Vehicle,imx500: IMX500):
    # make a directory to throw the detection images in
    os.makedirs("./captures/detections",exist_ok=True)

    # create filepaths
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    picPath = os.path.join(pathToPics,f'image_{timestamp}.jpg')
    metadataPath = os.path.join(pathToMetadata,f'metadata_{timestamp}.json')
    detectionPath = os.path.join(pathToDetections,f'detection_{timestamp}.jpg')

    # get drone info
    location: dronekit.LocationGlobalRelative = drone.location.global_relative_frame
    attitude: dronekit.Attitude = drone.attitude
    output_dict = {"attitude":attitude,"location":location}
    # dump the drone info and save it to a json
    with open(metadataPath, "w") as f:
        json.dump(output_dict, f)
    print("metadata saved")

    # get picture
    request = camera.capture_request()
    # have image for later
    image = request.make_array("main")
    # save picture
    request.save("main",picPath)
    # save detection metadata
    detectionMetadata = request.get_metadata()
    # release the request
    request.release()

    # run detector
    detections = imx500.get_outputs(detectionMetadata,add_batch=True)

    # add all boxes from detections
    boxes = detections[0][0]
    scores = detections[1][0]
    for box,score in zip(boxes,scores):
        if score < confTresh:
            continue
        y1,x1,y2,x2 = box
        cv2.rectangle(image, (x1,y1), (x2,y2), color=(0,255,0), thickness=2)
        
    # save the image with all the detections
    cv2.imwrite(detectionPath, cv2.cvtColor(image,cv2.COLOR_RGB2BGR))


def take_image(camera: Picamera2, drone: dronekit.Vehicle):
    # create the filepath where we'll save the image and metadata
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"image_{timestamp}.jpg"
    picPath = os.path.join(pathToPics, filename)
    filename = f"metadata_{timestamp}.json"
    metadataPath = os.path.join(pathToMetadata, filename)
    # get drone info
    location: dronekit.LocationGlobalRelative = drone.location.global_relative_frame
    attitude: dronekit.Attitude = drone.attitude
    output_dict = {"attitude":attitude,"location":location}
    # dump the drone info and save it to a json
    with open(metadataPath, "w") as f:
        json.dump(output_dict, f)
    print("metadata saved")
    # take the image and save it
    camera.capture_file(picPath)
    print("picture taken")

if __name__ == "__main__":
    main()