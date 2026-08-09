from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500
import os
from datetime import datetime
import time
import dronekit
import json
import cv2
# Drone coordinates imports
import numpy as np
from math import sin, cos, tan, radians, degrees
from dataclasses import dataclass

#-----------------------------------------------------------------------------------------------------------
# VARIABLES TO CHANGE
#-----------------------------------------------------------------------------------------------------------
pathToPics = './captures'
pathToDetections = './captures/detections'
pathToMetadata = './metadata'
modelPath = '../../vision/models/2-16-2026/v11s_2-16.rpk'
droneAddress = '/dev/ttyUSB0'
baudRate = 921600
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
    os.makedirs("./metadata",exist_ok=True)

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
####Drone coordinates code starts here
@dataclass
class DronePose:  #these float values should be given by the flight controller
    lat: float
    lon: float
    altitude: float # meters AGL
    yaw: float # degrees
    pitch: float      # degrees
    roll: float       # degrees


@dataclass
class GimbalPose:       #yaw pitch role values of gimbal----converted to default parameter
    yaw: float = 0        # degrees (relative to drone)
    pitch: float =-90    # degrees  # This default parameter assumes that the camera is pointing straight down
    roll: float =0      # degrees
# ###################### Rotational math

def rotation_matrix(yaw, pitch, roll):  #Rotational matrices for the x,y,and z directions
   #These matrices are basic definitions of rotational motion; I found them on a lesson about rotation on cuemath.com 
    yaw, pitch, roll = map(radians, (yaw, pitch, roll))
#yaw is rotation about the z axis (rotate drone clockwise and counterclockwise) The angle is the equivalent of gamma.
    Rz = np.array([
        [cos(yaw), -sin(yaw), 0],
        [sin(yaw),  cos(yaw), 0],
        [0,         0,        1]
    ])
#pitch is rotation about the y axis (tilt forward/backward)The angle is the equivalent of beta.
    Ry = np.array([
        [ cos(pitch), 0, sin(pitch)],
        [ 0,          1, 0         ],
        [-sin(pitch), 0, cos(pitch)]
    ])
#roll is rotation about the x axis (tilt left/right) The angle is the equivalent of alpha.
    Rx = np.array([
        [1, 0,          0         ],
        [0, cos(roll), -sin(roll)],
        [0, sin(roll),  cos(roll)]
    ])

    return Rz @ Ry @ Rx


# Geometrics

def meters_to_latlon(deltax, deltay, lat):  #coverts meters into latitude and longitude coordinates
    earth_radius = 6378137.0
    dlat = deltay / earth_radius    #dlat and dlong are measured in radians; this is the arc length formula s=rtheta
    dlon = deltax / (earth_radius * cos(radians(lat)))
    return degrees(dlat), degrees(dlon)


def intersect_ground(ray_world, altitude):
    """
    Intersect ray with ground plane Z=0
    """
    if ray_world[2] >= 0:
        return None

    t = altitude / -ray_world[2]
    return ray_world * t

# Main conversion

def pixel_to_geocoord_gimbal(
    px, py,
    image_width, image_height,
    h_fov, v_fov,
    drone: DronePose,
    gimbal: GimbalPose
):
    """
    Converts pixel coordinates to lat/lon
    using drone pose + gimbal-stabilized camera
    """

    # ---- Pixel to camera ray ----
    x = (px - image_width / 2) / (image_width / 2)
    y = -(py - image_height / 2) / (image_height / 2)

    h_fov = radians(h_fov)
    v_fov = radians(v_fov)

    ray_camera = np.array([
        x * tan(h_fov / 2),
        y * tan(v_fov / 2),
        -1
    ])
    ray_camera /= np.linalg.norm(ray_camera)

    # ---- Rotations ----
    R_drone = rotation_matrix(
        drone.yaw,
        drone.pitch,
        drone.roll
    )

    R_gimbal = rotation_matrix(
        gimbal.yaw,
        gimbal.pitch,
        gimbal.roll
    )

    # Camera orientation in world frame
    R_camera_world = R_drone @ R_gimbal

    ray_world = R_camera_world @ ray_camera

    #Ray and ground intersection
    ground_point = intersect_ground(ray_world, drone.altitude)
    if ground_point is None:
        return None

    dx, dy = ground_point[1], ground_point[0]
    dlat, dlon = meters_to_latlon(dx, dy, drone.lat)
    return drone.lat + dlat, drone.lon + dlon  #Returns the latitude and longitude of pixel. Drone position plus the change in lat and lon

# Example usage

if __name__ == "__main__":
    drone_pose = DronePose(
        lat=37.9485877,
        lon=91.7840758,
        altitude=26.882,
        yaw=-4.4222704380350475,      # drone turning about z-axis
        pitch=-99.38076702084126,     # drone pitching forward
        roll=140.02309214544962       # drone banking
    )

    # Gimbal stabilizes roll & pitch, points slightly forward
    gimbal_pose = GimbalPose(
        yaw=0,       # locked forward--- (0,0,0) is camera pointing directly downwards
        pitch=0,   # nadir view
        roll=0
    )

    pixel_to_latlon = pixel_to_geocoord_gimbal(     #px,py,image_width, and image_height may be subject to change once the hardware is studied
        px=900,
        py=30,
        image_width=1280,
        image_height=720,
        h_fov=1.41372,
        v_fov=0.797528202,
        drone=drone_pose,
        gimbal=gimbal_pose
    )

    print("Ground coordinate:", pixel_to_latlon)  #latlon
    
    