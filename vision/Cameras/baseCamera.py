from math import radians

import numpy as np

from vision.common.detection import Detection
from vision.common.image import Image
from vision.common.drone_coordinates import (
    DronePose,
    GimbalPose,
    meters_to_latlon,
    pixel_to_geocoord_gimbal,
    rotation_matrix,
)


class BaseCamera:
    def __init__(
        self,
        h_fov_deg: float = 0.0,
        v_fov_deg: float = 0.0,
        offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
        mount_rotation: GimbalPose | None = None,
    ):
        """
        h_fov_deg, v_fov_deg: this camera's horizontal/vertical field of view.
        offset: (right, forward, up) meters from wherever the drone's lat/lon/altitude
            is measured (e.g. its GPS antenna) to this camera, measured with the drone
            level and at zero yaw.
        mount_rotation: this camera's fixed rotation relative to the drone body. The
            camera boresight points along -z in its own frame before rotation, so a
            camera bolted on pointing straight down (nadir) is the default,
            GimbalPose(yaw=0, pitch=0, roll=0).
        """
        self.h_fov = radians(h_fov_deg)
        self.v_fov = radians(v_fov_deg)
        self.offset = offset
        self.mount_rotation = mount_rotation if mount_rotation is not None else GimbalPose()

    def initialize_camera(self) -> None:
        raise NotImplementedError("intialize_camera method must be implemented by subclasses")

    def capture_and_detect_mines(self) -> list[Detection]:
        raise NotImplementedError(
            "capture_and_detect_mines method must be implemented by subclasses"
        )

    def capture_and_detect_apriltags(self) -> list[Detection]:
        raise NotImplementedError(
            "capture_and_detect_apriltags method must be implemented by subclasses"
        )

    def capture_image(self, _only_metadata: bool) -> Image:
        raise NotImplementedError("capture_image method must be implemented by subclasses")

    def _camera_pose(self, drone: DronePose) -> DronePose:
        """`drone`'s pose with this camera's mount offset applied: the offset is
        rotated into world space by the drone's current attitude, since a rigid
        offset moves in 3D as the drone tips, then converted to a lat/lon/altitude
        delta."""
        right, forward, up = self.offset
        world_offset = rotation_matrix(drone.yaw, drone.pitch, drone.roll) @ np.array(
            [right, forward, up]
        )
        east, north, up_offset = world_offset
        dlat, dlon = meters_to_latlon(east, north, drone.lat)
        return DronePose(
            lat=drone.lat + dlat,
            lon=drone.lon + dlon,
            altitude=drone.altitude + up_offset,
            yaw=drone.yaw,
            pitch=drone.pitch,
            roll=drone.roll,
        )

    def get_pixel_coordinate(
        self, px: float, py: float, image_width: int, image_height: int, drone: DronePose
    ) -> tuple[float, float] | None:
        """
        Ground (lat, lon) that pixel (px, py) of an image taken by this camera
        corresponds to, given the drone's pose at the moment of capture. Returns
        None if that pixel's ray never reaches the ground (e.g. above the horizon).
        """
        return pixel_to_geocoord_gimbal(
            px=px,
            py=py,
            image_width=image_width,
            image_height=image_height,
            h_fov=self.h_fov,
            v_fov=self.v_fov,
            drone=self._camera_pose(drone),
            gimbal=self.mount_rotation,
        )

    def get_image_corner_coordinates(
        self, image_width: int, image_height: int, drone: DronePose
    ) -> tuple[tuple[float, float] | None, ...]:
        """
        Ground (lat, lon) of each corner of an image taken by this camera, in
        (top_left, top_right, bottom_left, bottom_right) pixel order.
        """
        corners_px = (
            (0, 0),
            (image_width - 1, 0),
            (0, image_height - 1),
            (image_width - 1, image_height - 1),
        )
        return tuple(
            self.get_pixel_coordinate(px, py, image_width, image_height, drone)
            for px, py in corners_px
        )
