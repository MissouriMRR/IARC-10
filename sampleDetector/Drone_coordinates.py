#Pseudocode: import altimeter, flight controller
import math
x = 0
y = 0
focusx = 0
focusy = 0
image_width = 0
image_height = 0
xcamera_coordinate = 0
ycamera_coordinate = 0
camera_swivel_angle = math.radians(0)
camera_depression_angle = math.radians(0)
latitudex = 0
longitudey =0
Altitude = 0
def findGeographicalCoordinates():
    focusx = Altitude*math.cos.radians(camera_swivel_angle)*math.tan.radians(camera_depression_angle)
    focusy = Altitude*math.cos.radians(camera_swivel_angle)*math.tan.radians(camera_depression_angle)
    latitudex = xcamera_coordinate + focusx
    longitudey= ycamera_coordinate + focusy

import numpy as np

def rotation_matrix(yaw, pitch, roll):
    """Build yaw–pitch–roll rotation matrix (Z-Y-X)."""
    yaw   = float(yaw)
    pitch = float(pitch)
    roll  = float(roll)

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [ 0,             1, 0            ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rx = np.array([
        [1, 0,           0          ],
        [0, np.cos(roll),-np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    return Rz @ Ry @ Rx


def pixel_to_ground(u, v, W, H, FOVx_deg, FOVy_deg, yaw, pitch, roll, altitude):
    """
    Convert image pixel (u,v) to ground coordinates (X,Y)
    assuming ground plane z=0 and camera at (0,0,altitude).
    """

    # Convert FOV to radians
    FOVx = np.radians(FOVx_deg)
    FOVy = np.radians(FOVy_deg)

    # 1. Convert pixel to normalized image coordinates in range [-1,1]
    x = (u - W/2) / (W/2)
   # needs work y =
