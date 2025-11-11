#Pseudocode: import altimeter, flight controller
import math
Altitude = 0
latitudex = 0
longitudey = 0
pi = 3.14
pixel_x = 0
pixel_y =0
cam_angle_camera = math.radians(0)
depression_angle_camera_ = math.radians(0)

def findPixelCoordinates():
    pixel_x = Altitude*math.cos.radians(cam_angle_camera)*math.tan.radians(depression_angle_camera_)
    pixel_y = Altitude*math.sin.radians(cam_angle_camera)*math.tan.radians(depression_angle_camera_)
    pixel_x_coordinate = latitudex +pixel_x
    pixel_y_coordinate = longitudey+pixel_y
    return pixel_x_coordinate, pixel_y_coordinate

findPixelCoordinates()