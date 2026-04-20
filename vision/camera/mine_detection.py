from ../common/drone_coordinates.pyy import pixel_to_geocoord_gimbal

class MineDetection:
    def __init__(self, image_coords: tuple[int, int, int, int], drone_position):
        self.image_coords = image_coords # coordinates of the mine within the image
        # coordinates of the mine within real space
        self.corner_coords = pixel_to_geocoord_gimbal(*image_coords, 1.41372, 0.797528202,
                                                     DronePose(drone_position), GimbalPose(0, 0, 0))
