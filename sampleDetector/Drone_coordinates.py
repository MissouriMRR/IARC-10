import numpy as np
from math import sin, cos, tan, radians, degrees
from dataclasses import dataclass

'''#Pseudocode: import altimeter, flight controller
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
'''   
# ============================================================
# Data classes
# ============================================================

@dataclass
class DronePose:  #these float values should be given by the flight controller
    lat: float
    lat = 37.9486929
    lon: float
    lon =-91.7841478
    altitude: float
    altitude = 30.357 # meters AGL
    yaw: float # degrees
    yaw = -2.106571572697931
    pitch: float      # degrees
    pitch = -79.59291162486421
    roll: float       # degrees
    roll = 12.158130312273618


@dataclass
class GimbalPose:       #yaw pitch role values of gimbal
    yaw: float        # degrees (relative to drone)
    pitch: float      # degrees
    roll: float       # degrees


# ============================================================
# Rotation math
# ============================================================

def rotation_matrix(yaw, pitch, roll):  #Rotational matrices for the x,y,and z directions
    """
    ZYX rotation: yaw (Z), pitch (Y), roll (X)
    """
    yaw, pitch, roll = map(radians, (yaw, pitch, roll))
#yaw z axis (rotate drone left and right)
    Rz = np.array([
        [cos(yaw), -sin(yaw), 0],
        [sin(yaw),  cos(yaw), 0],
        [0,         0,        1]
    ])
#pitch y axis (tilt forward backward)
    Ry = np.array([
        [ cos(pitch), 0, sin(pitch)],
        [ 0,          1, 0         ],
        [-sin(pitch), 0, cos(pitch)]
    ])
#roll x axis (tilt left/right)
    Rx = np.array([
        [1, 0,          0         ],
        [0, cos(roll), -sin(roll)],
        [0, sin(roll),  cos(roll)]
    ])

    return Rz @ Ry @ Rx


# ============================================================
# Geometry helpers
# ============================================================

def meters_to_latlon(dx, dy, lat):  #coverts meters into lat and lon coordinates using derivatives
    earth_radius = 6378137.0
    dlat = dy / earth_radius
    dlon = dx / (earth_radius * cos(radians(lat)))
    return degrees(dlat), degrees(dlon)


def intersect_ground(ray_world, altitude):
    """
    Intersect ray with ground plane Z=0
    """
    if ray_world[2] >= 0:
        return None

    t = altitude / -ray_world[2]
    return ray_world * t


# ============================================================
# Main conversion
# ============================================================

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
    y = (py - image_height / 2) / (image_height / 2)

    h_fov = radians(1.41372)
    v_fov = radians(0.797528202)

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

    # ---- Ray-ground intersection ----
    ground_point = intersect_ground(ray_world, drone.altitude)
    if ground_point is None:
        return None

    dx, dy = ground_point[0], ground_point[1]
    dlat, dlon = meters_to_latlon(dx, dy, drone.lat)

    return drone.lat + dlat, drone.lon + dlon


# ============================================================
# Example usage
# ============================================================

if __name__ == "__main__":
    drone_pose = DronePose(
        lat=37.7749,
        lon=-122.4194,
        altitude=120,
        yaw=15,      # drone turning
        pitch=5,     # drone pitching forward
        roll=8       # drone banking
    )

    # Gimbal stabilizes roll & pitch, points slightly forward
    gimbal_pose = GimbalPose(
        yaw=0,       # locked forward
        pitch=-90,   # nadir view
        roll=0
    )

    latlon = pixel_to_geocoord_gimbal(
        px=640,
        py=360,
        image_width=1280,
        image_height=720,
        h_fov=84,
        v_fov=60,
        drone=drone_pose,
        gimbal=gimbal_pose
    )

    print("Ground coordinate:", latlon)
