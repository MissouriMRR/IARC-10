"""Implements a CameraInfo class containing camera information."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CameraInfo:
    """
    A class containing information about a camera.

    Attributes
    ----------
    horizontal_fov_deg : float
        The horizontal field of view of the camera in degrees.
    vertical_fov_deg : float
        The vertical field of view of the camera in degrees.
    """
    horizontal_fov_deg: float
    vertical_fov_deg: float

    @property
    def horizontal_fov_rad(self) -> float:
        """Get the horizontal field of view in radians."""
        return math.radians(self.horizontal_fov_deg)

    @property
    def vertical_fov_rad(self) -> float:
        """Get the vertical field of view in radians."""
        return math.radians(self.vertical_fov_deg)
