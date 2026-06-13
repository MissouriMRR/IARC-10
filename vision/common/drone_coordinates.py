import numpy as np
from math import sin, cos, tan, radians, degrees
from dataclasses import dataclass
# ######Data classes
@dataclass
class DronePose:  #these float values should be given by the flight controller
    lat: float
    lon: float
    altitude: float # meters AGL
    yaw: float # degrees
    pitch: float      # degrees
    roll: float       # degrees

@dataclass
class TargetCoordinates:
    lat: float
    lon: float

@dataclass
class GimbalPose:       #yaw pitch role values of gimbal----converted to default parameter
    yaw: float = 0        # degrees (relative to drone)
    pitch: float =0    # degrees  # This default parameter assumes that the camera is pointing straight down
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
    return Rx @ Ry @ Rz


# Geometrics

def meters_to_latlon(deltax, deltay, lat):  #coverts meters into latitude and longitude coordinates
    earth_radius = 6378137.0
    dlat = deltax / earth_radius    #dlat and dlong are measured in radians; this is the arc length formula s=rtheta
    dlon = deltay / (earth_radius * cos(radians(lat)))
    return degrees(dlat), degrees(dlon)

def compute_offset_from_target(lat, lon, target: TargetCoordinates):
    """
    Compute offset from computed GPS point to target GPS point

    Returns:
        dx: meters east (+) / west (-)
        dy: meters north (+) / south (-)
    """
    dx, dy = latlon_to_meters(lat, lon, target.lat, target.lon)

    return dx, dy
def latlon_to_meters(lat1, lon1, lat2, lon2):
    earth_radius = 6378137.0

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)

    avg_lat = radians((lat1 + lat2) / 2)

    mx = dlat * earth_radius
    my = dlon * earth_radius * cos(avg_lat)

    return mx, my

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

    dx, dy = ground_point[0], ground_point[1]
    dlat, dlon = meters_to_latlon(dx, dy, drone.lat)
    return drone.lat + dlat, drone.lon + dlon, dx,dy #Returns the latitude and longitude of pixel. Drone position plus the change in lat and lon
    # replace with real value
if __name__ == "__main__":
    drone_pose = DronePose(
        lat=37.9485877,
        lon=91.7840758,
        altitude=26.882,
        yaw=-4.4222704380350475,
        pitch=-99.38076702084126,
        roll=140.02309214544962
    )

    actual_target_coordinates = TargetCoordinates(
        lat=37,   # change this to test
        lon=91
    )

    # Gimbal stabilizes roll & pitch, points slightly forward
    gimbal_pose = GimbalPose(
        yaw=0,       # locked forward--- (0,0,0) is camera pointing directly downwards
        pitch=-90,   # nadir view
        roll=0
    )

    # ---- Compute ground point FIRST ----
    ground = pixel_to_geocoord_gimbal(
        px=900,
        py=30,
        image_width=1280,
        image_height=720,
        h_fov=1.41372,
        v_fov=0.797528202,
        drone=drone_pose,
        gimbal=gimbal_pose
    )

    if ground is not None:
        lat, lon, dx, dy = ground

        offset_x, offset_y = compute_offset_from_target(
            lat, lon, actual_target_coordinates
        )

        print("Ground coordinate:", (lat, lon))
        print("Target Coordinates:", (actual_target_coordinates.lat, actual_target_coordinates.lon))
        print("Offset from target (meters):", (offset_x, offset_y))
    else:
        print("Ray does not intersect ground")